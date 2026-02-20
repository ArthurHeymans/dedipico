#![no_std]
#![no_main]

mod config;
mod leds;
mod protocol;
mod spi_flash;
mod usb_handler;

use core::cell::RefCell;

use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{SPI0, USB};
use embassy_rp::spi::{self, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel::Channel;
use embassy_usb::driver::{Direction, Endpoint as _, EndpointAddress, EndpointIn as _, EndpointOut as _};
use embassy_usb::Builder;
use panic_probe as _;
use static_cell::StaticCell;

use crate::config::*;
use crate::leds::Leds;
use crate::protocol::BulkOperation;
use crate::spi_flash::SpiFlash;
use crate::usb_handler::DediprogHandler;

// =============================================================================
// Interrupt bindings
// =============================================================================

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// =============================================================================
// Shared state between USB handler (sync) and bulk worker task (async)
// =============================================================================

/// SPI flash peripheral — borrowed by the handler for transceive (blocking)
/// and taken by the worker task for bulk operations (async with DMA).
pub static SPI_FLASH: Mutex<RefCell<Option<SpiFlash<'static>>>> =
    Mutex::new(RefCell::new(None));

/// Pending bulk operation set by the handler, consumed by the worker task.
pub static BULK_OP: Mutex<RefCell<Option<BulkOperation>>> = Mutex::new(RefCell::new(None));

/// Signal to wake the worker task when a bulk operation is ready.
pub static BULK_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// =============================================================================
// USB device type alias
// =============================================================================

type UsbDriver = Driver<'static, USB>;

// =============================================================================
// Entry point
// =============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("DediPico starting up");

    // ---- SPI peripheral ----
    let mut spi_config = spi::Config::default();
    spi_config.frequency = DEFAULT_SPI_FREQ_HZ;
    spi_config.phase = spi::Phase::CaptureOnFirstTransition;
    spi_config.polarity = spi::Polarity::IdleLow;

    let spi: Spi<'static, SPI0, spi::Async> = Spi::new(
        p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, p.DMA_CH0, p.DMA_CH1, spi_config,
    );
    let cs = Output::new(p.PIN_17, Level::High); // CS deasserted (high)

    // Store in shared state
    critical_section::with(|cs_tok| {
        *SPI_FLASH.borrow(cs_tok).borrow_mut() = Some(SpiFlash::new(spi, cs));
    });

    // ---- LEDs ----
    let led_pass = Output::new(p.PIN_25, Level::Low);
    let led_busy = Output::new(p.PIN_14, Level::Low);
    let led_error = Output::new(p.PIN_15, Level::Low);
    let leds = Leds::new(led_pass, led_busy, led_error);

    // ---- USB driver ----
    let driver = Driver::new(p.USB, Irqs);

    let mut usb_config = embassy_usb::Config::new(USB_VID, USB_PID);
    usb_config.manufacturer = Some("DediProg");
    usb_config.product = Some("SF600");
    usb_config.serial_number = Some("S6B000001");
    usb_config.max_power = 200;
    usb_config.max_packet_size_0 = 64;

    // Descriptor buffers (must be 'static)
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 128]),
    );

    // ---- Handler ----
    static HANDLER: StaticCell<DediprogHandler> = StaticCell::new();
    let handler = HANDLER.init(DediprogHandler::new(leds));
    builder.handler(handler);

    // ---- Vendor-class interface with bulk endpoints ----
    //
    // flashprog hard-codes:  EP1 OUT (0x01) for SF600, EP2 IN (0x82) for all.
    // embassy-usb 0.5+ lets us specify exact endpoint addresses.
    let ep1_out = EndpointAddress::from_parts(1, Direction::Out);
    let ep2_in = EndpointAddress::from_parts(2, Direction::In);

    let mut func = builder.function(0xFF, 0x00, 0x00);
    let mut iface = func.interface();
    let mut alt = iface.alt_setting(0xFF, 0x00, 0x00, None);

    let ep_out = alt.endpoint_bulk_out(Some(ep1_out), USB_MAX_PACKET_SIZE);
    let ep_in = alt.endpoint_bulk_in(Some(ep2_in), USB_MAX_PACKET_SIZE);

    drop(func); // release borrow on builder

    // ---- Build and launch ----
    let usb = builder.build();

    spawner.must_spawn(usb_device_task(usb));
    spawner.must_spawn(bulk_worker_task(ep_in, ep_out));

    info!("DediPico ready — VID:PID = {:04x}:{:04x}", USB_VID, USB_PID);

    // Main task has nothing else to do; park forever.
    loop {
        embassy_time::Timer::after_secs(3600).await;
    }
}

// =============================================================================
// USB device task — runs the USB stack, dispatches control transfers
// =============================================================================

#[embassy_executor::task]
async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, UsbDriver>) {
    usb.run().await;
}

// =============================================================================
// Bulk worker task — handles bulk read/write operations using SPI + DMA
// =============================================================================

#[embassy_executor::task]
async fn bulk_worker_task(
    mut ep_in: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    mut ep_out: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
) {
    loop {
        // Sleep until the USB handler signals a new bulk operation.
        BULK_SIGNAL.wait().await;

        let op = critical_section::with(|cs| BULK_OP.borrow(cs).borrow_mut().take());

        let Some(op) = op else {
            continue;
        };

        // Take the SPI peripheral out of shared state for exclusive async use.
        let flash = critical_section::with(|cs| SPI_FLASH.borrow(cs).borrow_mut().take());
        let Some(mut flash) = flash else {
            error!("SPI flash not available for bulk operation");
            continue;
        };

        match op {
            BulkOperation::Read {
                address,
                block_count,
                opcode,
                addr_len,
                dummy_bytes,
            } => {
                info!(
                    "Bulk READ: addr=0x{:08x} blocks={} opcode=0x{:02x}",
                    address, block_count, opcode
                );

                ep_in.wait_enabled().await;
                flash
                    .start_read(opcode, address, addr_len, dummy_bytes)
                    .await;

                // Zero-copy double buffer: 2 slots of 512 bytes.
                // The channel handles synchronization so SPI fills one slot
                // while USB drains the other — no mem::swap needed.
                let mut buf = [[0u8; BULK_BLOCK_SIZE]; 2];
                let mut channel =
                    Channel::<CriticalSectionRawMutex, [u8; BULK_BLOCK_SIZE]>::new(&mut buf);
                let (mut sender, mut receiver) = channel.split();

                let ((), usb_result) = embassy_futures::join::join(
                    // SPI producer: DMA-read blocks directly into channel slots
                    async {
                        for _i in 0..block_count {
                            let slot = sender.send().await;
                            flash.read_block(slot).await;
                            sender.send_done();
                        }
                    },
                    // USB consumer: send filled slots to the host
                    async {
                        let mut result: Result<(), embassy_usb::driver::EndpointError> = Ok(());
                        for i in 0..block_count {
                            {
                                let slot = receiver.receive().await;
                                if result.is_ok() {
                                    if let Err(e) = write_bulk_block(&mut ep_in, slot).await {
                                        error!("Bulk IN write error at block {}", i);
                                        result = Err(e);
                                    }
                                }
                            }
                            receiver.receive_done();
                        }
                        result
                    },
                )
                .await;

                flash.end_transfer();

                if usb_result.is_ok() {
                    info!("Bulk READ complete ({} blocks)", block_count);
                }
            }

            BulkOperation::Write {
                mut address,
                block_count,
                opcode,
                addr_len,
            } => {
                info!(
                    "Bulk WRITE: addr=0x{:08x} blocks={} opcode=0x{:02x}",
                    address, block_count, opcode
                );

                ep_out.wait_enabled().await;

                // Zero-copy double buffer: USB receives into one slot
                // while SPI programs from the other.
                let mut buf = [[0u8; BULK_BLOCK_SIZE]; 2];
                let mut channel =
                    Channel::<CriticalSectionRawMutex, [u8; BULK_BLOCK_SIZE]>::new(&mut buf);
                let (mut sender, mut receiver) = channel.split();

                let (usb_result, ()) = embassy_futures::join::join(
                    // USB producer: receive blocks from host into channel slots
                    async {
                        let mut result: Result<(), embassy_usb::driver::EndpointError> = Ok(());
                        for i in 0..block_count {
                            {
                                let slot = sender.send().await;
                                if result.is_ok() {
                                    if let Err(e) = read_bulk_block(&mut ep_out, slot).await {
                                        error!("Bulk OUT read error at block {}", i);
                                        result = Err(e);
                                    }
                                }
                            }
                            sender.send_done();
                        }
                        result
                    },
                    // SPI consumer: program pages from channel slots
                    async {
                        for _i in 0..block_count {
                            {
                                let slot = receiver.receive().await;
                                // First 256 bytes are real data; rest is padding.
                                let page_data = &slot[..PAGE_SIZE];
                                flash
                                    .write_page(opcode, address, addr_len, page_data)
                                    .await;
                            }
                            receiver.receive_done();
                            address = address.wrapping_add(PAGE_SIZE as u32);
                        }
                    },
                )
                .await;

                if usb_result.is_ok() {
                    info!("Bulk WRITE complete ({} blocks)", block_count);
                }
            }
        }

        // Return the SPI peripheral to shared state so the handler can use it.
        critical_section::with(|cs| {
            *SPI_FLASH.borrow(cs).borrow_mut() = Some(flash);
        });
    }
}

// =============================================================================
// Bulk endpoint helpers — read/write 512-byte blocks in max-packet chunks
// =============================================================================

/// Write a full 512-byte block to the bulk IN endpoint (8 × 64-byte packets).
async fn write_bulk_block(
    ep: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    data: &[u8; BULK_BLOCK_SIZE],
) -> Result<(), embassy_usb::driver::EndpointError> {
    for chunk in data.chunks(USB_MAX_PACKET_SIZE as usize) {
        ep.write(chunk).await?;
    }
    Ok(())
}

/// Read a full 512-byte block from the bulk OUT endpoint.
async fn read_bulk_block(
    ep: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
    buf: &mut [u8; BULK_BLOCK_SIZE],
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut offset = 0;
    while offset < BULK_BLOCK_SIZE {
        let n = ep.read(&mut buf[offset..]).await?;
        offset += n;
    }
    Ok(())
}
