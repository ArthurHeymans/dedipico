#![no_std]
#![no_main]

mod aux;
mod config;
mod leds;
mod protocol;
mod spi_flash;
mod usb_handler;

use core::cell::RefCell;
use core::sync::atomic::{Ordering, compiler_fence};

use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::bind_interrupts;
use embassy_rp::dma::{Channel as DmaChannel, InterruptHandler as DmaInterruptHandler};
use embassy_rp::gpio::{Flex, Input, Level, Output, OutputOpenDrain, Pull};
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_0, PIN_1, PIO0, PIO1, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel::Channel;
use embassy_usb::Builder;
use embassy_usb::driver::{
    Direction, Endpoint as _, EndpointAddress, EndpointIn as _, EndpointOut as _,
};
use panic_probe as _;
use static_cell::StaticCell;

use crate::aux::*;
use crate::config::*;
use crate::leds::Leds;
use crate::protocol::BulkOperation;
use crate::spi_flash::SpiFlash;
use crate::usb_handler::DediprogHandler;

// =============================================================================
// Interrupt bindings
// =============================================================================

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>, DmaInterruptHandler<DMA_CH1>;
});

// =============================================================================
// Shared state between USB handler (sync) and bulk worker task (async)
// =============================================================================

/// SPI flash peripheral — borrowed by the handler for transceive (blocking)
/// and taken by the worker task for bulk operations (async with DMA).
pub static SPI_FLASH: Mutex<RefCell<Option<SpiFlash<'static>>>> = Mutex::new(RefCell::new(None));

/// Pending bulk operation set by the handler, consumed by the worker task.
pub static BULK_OP: Mutex<RefCell<Option<BulkOperation>>> = Mutex::new(RefCell::new(None));

/// Signal to wake the worker task when a bulk operation is ready.
pub static BULK_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

const FLASH_WP_PIN: usize = 5;
const FLASH_HOLD_PIN: usize = 6;
const FLASH_CS_PIN: usize = 7;
const FLASH_IDLE_HIGH_MASK: u32 = (1 << FLASH_WP_PIN) | (1 << FLASH_HOLD_PIN) | (1 << FLASH_CS_PIN);

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

    // ---- Flash bus ----
    // PIO/multi-I/O-friendly pinout:
    //   GP2  = SCK
    //   GP3  = IO0 / MOSI
    //   GP4  = IO1 / MISO
    //   GP5  = IO2 / WP#
    //   GP6  = IO3 / HOLD#
    //   GP7  = CS#
    force_flash_bus_deselected_at_startup();
    let pio = Pio::new(p.PIO0, Irqs);
    let mut cs = Flex::new(p.PIN_7); // released/Hi-Z when idle
    cs.set_high();
    cs.set_as_output();

    // Store in shared state
    critical_section::with(|cs_tok| {
        *SPI_FLASH.borrow(cs_tok).borrow_mut() = Some(SpiFlash::new(
            pio,
            DmaChannel::new(p.DMA_CH0, Irqs),
            DmaChannel::new(p.DMA_CH1, Irqs),
            p.PIN_3,
            p.PIN_4,
            p.PIN_5,
            p.PIN_6,
            p.PIN_2,
            cs,
        ));
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
    static CONFIG_DESC: StaticCell<[u8; 512]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        CONFIG_DESC.init([0; 512]),
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

    // ---- Auxiliary vendor interface ----
    // GP0 = UART TX, GP1 = UART RX. Use PIO1, matching picoprog's PIO UART
    // approach and keeping the flash bus' PIO0 state machines isolated.
    let uart_pio = p.PIO1;
    let uart_tx_pin = p.PIN_0;
    let uart_rx_pin = p.PIN_1;

    // GP8 = RESET# open-drain, GP9 = POWER_SW# open-drain,
    // GP10 = board power-state input, GP11 = auxiliary state input.
    let board_gpio = BoardGpio::new(
        OutputOpenDrain::new(p.PIN_8, Level::High),
        OutputOpenDrain::new(p.PIN_9, Level::High),
        Input::new(p.PIN_10, Pull::None),
        Input::new(p.PIN_11, Pull::None),
    );

    let ep3_out = EndpointAddress::from_parts(3, Direction::Out);
    let ep4_in = EndpointAddress::from_parts(4, Direction::In);
    let mut aux_func = builder.function(0xFF, 0xD1, 0x01);
    let mut aux_iface = aux_func.interface();
    let mut aux_alt = aux_iface.alt_setting(0xFF, 0xD1, 0x01, None);
    let aux_out = aux_alt.endpoint_bulk_out(Some(ep3_out), USB_MAX_PACKET_SIZE);
    let aux_in = aux_alt.endpoint_bulk_in(Some(ep4_in), USB_MAX_PACKET_SIZE);
    drop(aux_func);

    // ---- Build and launch ----
    let usb = builder.build();

    spawner.spawn(usb_device_task(usb).unwrap());
    spawner.spawn(bulk_worker_task(ep_in, ep_out).unwrap());
    spawner.spawn(aux_task(aux_out, aux_in, uart_pio, uart_tx_pin, uart_rx_pin, board_gpio).unwrap());

    info!("DediPico ready — VID:PID = {:04x}:{:04x}", USB_VID, USB_PID);

    // Main task has nothing else to do; park forever.
    loop {
        embassy_time::Timer::after_secs(3600).await;
    }
}

fn force_flash_bus_deselected_at_startup() {
    // Put CS#, WP# and HOLD# at their inactive levels before PIO takes over the
    // bus. Some flash parts latch HOLD# relative to CS#, so the safe state must
    // exist before SpiFlash::new() finishes configuring the PIO pins/programs.
    pac::SIO
        .gpio_out(0)
        .value_set()
        .write_value(FLASH_IDLE_HIGH_MASK);
    pac::SIO
        .gpio_oe(0)
        .value_set()
        .write_value(FLASH_IDLE_HIGH_MASK);

    for pin in [FLASH_WP_PIN, FLASH_HOLD_PIN, FLASH_CS_PIN] {
        pac::PADS_BANK0.gpio(pin).modify(|w| {
            w.set_ie(true);
            w.set_od(false);
            w.set_pue(false);
            w.set_pde(false);
        });
        pac::IO_BANK0.gpio(pin).ctrl().write(|w| {
            w.set_funcsel(pac::io::vals::Gpio0ctrlFuncsel::SIO_0 as _);
            w.set_outover(pac::io::vals::Outover::NORMAL);
            w.set_oeover(pac::io::vals::Oeover::NORMAL);
        });
    }

    compiler_fence(Ordering::SeqCst);
}

// =============================================================================
// USB device task — runs the USB stack, dispatches control transfers
// =============================================================================

#[embassy_executor::task]
async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, UsbDriver>) {
    usb.run().await;
}

// =============================================================================
// Auxiliary vendor USB: GPIO + UART bridge
// =============================================================================

#[embassy_executor::task]
async fn aux_task(
    mut ep_out: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
    mut ep_in: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    uart_pio: Peri<'static, PIO1>,
    uart_tx_pin: Peri<'static, PIN_0>,
    uart_rx_pin: Peri<'static, PIN_1>,
    mut gpio: BoardGpio<'static>,
) {
    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(uart_pio, Irqs);

    let tx_prog = PioUartTxProgram::new(&mut common);
    let mut uart_tx = PioUartTx::new(115_200, &mut common, sm0, uart_tx_pin, &tx_prog);

    let rx_prog = PioUartRxProgram::new(&mut common);
    let mut uart_rx = PioUartRx::new(115_200, &mut common, sm1, uart_rx_pin, &rx_prog);

    let mut usb_buf = [0u8; FRAME_LEN];

    loop {
        ep_out.wait_enabled().await;
        ep_in.wait_enabled().await;

        match select(ep_out.read(&mut usb_buf), uart_rx.read_u8()).await {
            Either::First(Ok(n)) => {
                handle_aux_frame(&mut ep_in, &mut uart_tx, &mut gpio, &usb_buf[..n]).await;
            }
            Either::First(Err(_)) => {}
            Either::Second(byte) => {
                let mut frame = [0u8; FRAME_LEN];
                frame[0] = EVT_UART_DATA;
                frame[1] = 1;
                frame[2] = byte;
                let _ = ep_in.write(&frame).await;
            }
        }
    }
}

async fn handle_aux_frame(
    ep_in: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    uart_tx: &mut PioUartTx<'static, PIO1, 0>,
    gpio: &mut BoardGpio<'static>,
    frame: &[u8],
) {
    match frame.first().copied() {
        Some(CMD_GPIO_GET_STATE) => {
            let _ = ep_in.write(&gpio.state_frame()).await;
        }
        Some(CMD_GPIO_SET_DIRECTION) if frame.len() >= 3 => {
            gpio.set_direction(frame[1], frame[2]);
            let _ = ep_in.write(&gpio.state_frame()).await;
        }
        Some(CMD_GPIO_SET_OUTPUT) if frame.len() >= 3 => {
            gpio.set_output(frame[1], frame[2]);
            let _ = ep_in.write(&gpio.state_frame()).await;
        }
        Some(CMD_GPIO_PULSE_LOW) if frame.len() >= 4 => {
            gpio.pulse_low(frame[1], u16::from_le_bytes([frame[2], frame[3]]))
                .await;
            let _ = ep_in.write(&gpio.state_frame()).await;
        }
        Some(CMD_UART_SET_BAUD) if frame.len() >= 5 => {
            // The PIO UART starts at 115200 baud. Runtime retiming can be added
            // later if the aux protocol needs non-default rates.
        }
        Some(CMD_UART_WRITE) if frame.len() >= 2 => {
            let len = (frame[1] as usize).min(frame.len().saturating_sub(2));
            let _ = embedded_io_async::Write::write_all(uart_tx, &frame[2..2 + len]).await;
        }
        _ => {}
    }
}

// =============================================================================
// Bulk worker task — handles bulk read/write operations using the PIO flash bus
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

        // Take the flash bus out of shared state for exclusive async use.
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
                dummy_cycles,
                io_mode,
                mode_byte,
            } => {
                info!(
                    "Bulk READ: addr=0x{:08x} blocks={} opcode=0x{:02x} io_mode={} dummy_cycles={}",
                    address, block_count, opcode, io_mode, dummy_cycles
                );

                ep_in.wait_enabled().await;
                flash
                    .start_read(opcode, address, addr_len, io_mode, mode_byte, dummy_cycles)
                    .await;

                // Zero-copy double buffer: 2 slots of 512 bytes.
                // The channel handles synchronization so SPI fills one slot
                // while USB drains the other — no mem::swap needed.
                let mut buf = [[0u8; BULK_BLOCK_SIZE]; 2];
                let mut channel =
                    Channel::<CriticalSectionRawMutex, [u8; BULK_BLOCK_SIZE]>::new(&mut buf);
                let (mut sender, mut receiver) = channel.split();
                let mut fast_in = FastBulkIn::new_ep2();

                let ((), usb_result) = embassy_futures::join::join(
                    // SPI producer: DMA-read blocks directly into channel slots
                    async {
                        for _i in 0..block_count {
                            let slot = sender.send().await;
                            flash.read_block(slot, io_mode).await;
                            sender.send_done();
                        }
                    },
                    // USB consumer: send filled slots to the host
                    async {
                        let mut result: Result<(), embassy_usb::driver::EndpointError> = Ok(());
                        for i in 0..block_count {
                            {
                                let slot = receiver.receive().await;
                                if result.is_ok()
                                    && let Err(e) = write_bulk_block(&mut fast_in, slot).await
                                {
                                    error!("Bulk IN write error at block {}", i);
                                    result = Err(e);
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
                                if result.is_ok()
                                    && let Err(e) = read_bulk_block(&mut ep_out, slot).await
                                {
                                    error!("Bulk OUT read error at block {}", i);
                                    result = Err(e);
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
                                flash.write_page(opcode, address, addr_len, page_data).await;
                            }
                            receiver.receive_done();
                            address = address.wrapping_add(PAGE_SIZE as u32);
                        }

                        // EM100 can acknowledge the last page program command before
                        // its emulated read array is visible to the immediately
                        // following flashprog verify pass. Give only the final bulk
                        // write completion a short settle window instead of slowing
                        // every page program.
                        embassy_time::Timer::after_millis(25).await;
                    },
                )
                .await;

                if usb_result.is_ok() {
                    info!("Bulk WRITE complete ({} blocks)", block_count);
                }
            }
        }

        // Return the flash bus to shared state so the handler can use it.
        critical_section::with(|cs| {
            *SPI_FLASH.borrow(cs).borrow_mut() = Some(flash);
        });
    }
}

// =============================================================================
// Bulk endpoint helpers — read/write 512-byte blocks in max-packet chunks
// =============================================================================

/// Fast writer for the fixed DediProg bulk-IN endpoint (EP2 IN / 0x82).
///
/// Embassy's generic RP2040 USB endpoint driver uses one 64-byte DPRAM buffer per
/// endpoint. Full-speed bulk can use more bus slots if the device can pre-arm the
/// next packet while the current one is in flight, so this path enables RP2040's
/// double-buffered endpoint mode for EP2 IN and alternates both 64-byte buffers.
/// The protocol endpoint number is not dynamic: flashprog/dediprog always reads
/// bulk data from EP2 IN, and `main()` allocates EP1 OUT then EP2 IN, giving EP2
/// buffer 0 the same 0x1c0 DPRAM address Embassy assigned originally.
struct FastBulkIn {
    next_buf: usize,
    next_pid: bool,
}

impl FastBulkIn {
    const EP_INDEX: usize = 2;
    const EP_CONTROL_INDEX: usize = Self::EP_INDEX - 1;
    const DPRAM_BUFFER0_ADDR: usize = 0x1c0;
    const BUFFER_STRIDE: usize = USB_MAX_PACKET_SIZE as usize;

    fn new_ep2() -> Self {
        let dpram = pac::USB_DPRAM;
        dpram.ep_in_control(Self::EP_CONTROL_INDEX).modify(|w| {
            w.set_buffer_address(Self::DPRAM_BUFFER0_ADDR as u16);
            w.set_endpoint_type(pac::usb_dpram::vals::EpControlEndpointType::BULK);
            w.set_interrupt_per_buff(true);
            w.set_interrupt_per_double_buff(false);
            w.set_double_buffered(true);
            w.set_enable(true);
        });
        dpram.ep_in_buffer_control(Self::EP_INDEX).write(|w| {
            w.set_reset(true);
            w.set_pid(0, true);
            w.set_pid(1, false);
        });

        Self {
            next_buf: 0,
            // Matches Embassy's first-packet behavior after endpoint enable: it
            // stores `pid = !current_pid`, where current_pid starts true.
            next_pid: false,
        }
    }

    async fn write_packet(&mut self, packet: &[u8]) {
        let dpram = pac::USB_DPRAM;
        let buf_index = self.next_buf;

        while dpram
            .ep_in_buffer_control(Self::EP_INDEX)
            .read()
            .available(buf_index)
        {
            embassy_futures::yield_now().await;
        }

        compiler_fence(Ordering::SeqCst);
        let dst = unsafe {
            core::slice::from_raw_parts_mut(
                (pac::USB_DPRAM.as_ptr() as *mut u8)
                    .add(Self::DPRAM_BUFFER0_ADDR + buf_index * Self::BUFFER_STRIDE),
                packet.len(),
            )
        };
        dst.copy_from_slice(packet);
        compiler_fence(Ordering::SeqCst);

        dpram.ep_in_buffer_control(Self::EP_INDEX).modify(|w| {
            w.set_length(buf_index, packet.len() as u16);
            w.set_pid(buf_index, self.next_pid);
            w.set_full(buf_index, true);
        });
        cortex_m::asm::delay(12);
        dpram.ep_in_buffer_control(Self::EP_INDEX).modify(|w| {
            w.set_length(buf_index, packet.len() as u16);
            w.set_pid(buf_index, self.next_pid);
            w.set_full(buf_index, true);
            w.set_available(buf_index, true);
        });

        self.next_pid = !self.next_pid;
        self.next_buf ^= 1;
    }
}

/// Write a full 512-byte block to the bulk IN endpoint (8 × 64-byte packets).
async fn write_bulk_block(
    fast_in: &mut FastBulkIn,
    data: &[u8; BULK_BLOCK_SIZE],
) -> Result<(), embassy_usb::driver::EndpointError> {
    for chunk in data.chunks(USB_MAX_PACKET_SIZE as usize) {
        fast_in.write_packet(chunk).await;
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
