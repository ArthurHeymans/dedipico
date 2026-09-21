use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;
use defmt::*;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel::Channel;
use embassy_usb::driver::{Endpoint as _, EndpointIn as _, EndpointOut as _};

use crate::UsbDriver;
use crate::config::{BULK_BLOCK_SIZE, PAGE_SIZE, USB_MAX_PACKET_SIZE};
use crate::fast_bulk_in::FastBulkIn;
use crate::protocol::BulkOperation;
use crate::spi_flash::SpiFlash;

struct BulkState {
    pending: Option<BulkOperation>,
    active: bool,
}

static SPI_FLASH: Mutex<RefCell<Option<SpiFlash<'static>>>> = Mutex::new(RefCell::new(None));
static BULK_STATE: Mutex<RefCell<BulkState>> = Mutex::new(RefCell::new(BulkState {
    pending: None,
    active: false,
}));
static BULK_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub fn install_flash(flash: SpiFlash<'static>) {
    put_flash(flash);
}

fn take_flash() -> Option<SpiFlash<'static>> {
    critical_section::with(|cs| SPI_FLASH.borrow(cs).borrow_mut().take())
}

pub fn take_flash_for_control() -> Option<SpiFlash<'static>> {
    critical_section::with(|cs| {
        let state = BULK_STATE.borrow(cs).borrow();
        if state.active || state.pending.is_some() {
            None
        } else {
            SPI_FLASH.borrow(cs).borrow_mut().take()
        }
    })
}

pub fn put_flash(flash: SpiFlash<'static>) {
    critical_section::with(|cs| {
        *SPI_FLASH.borrow(cs).borrow_mut() = Some(flash);
    });
}

pub fn submit(operation: BulkOperation) -> bool {
    let accepted = critical_section::with(|cs| {
        let mut state = BULK_STATE.borrow(cs).borrow_mut();
        if state.active || state.pending.is_some() {
            false
        } else {
            state.pending = Some(operation);
            true
        }
    });
    if accepted {
        BULK_SIGNAL.signal(());
    }
    accepted
}

fn take_operation() -> Option<BulkOperation> {
    critical_section::with(|cs| {
        let mut state = BULK_STATE.borrow(cs).borrow_mut();
        let operation = state.pending.take();
        state.active = operation.is_some();
        operation
    })
}

fn finish_operation() {
    critical_section::with(|cs| BULK_STATE.borrow(cs).borrow_mut().active = false);
}

pub async fn run(
    mut ep_in: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    mut ep_out: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
) {
    loop {
        BULK_SIGNAL.wait().await;

        let Some(operation) = take_operation() else {
            continue;
        };
        let Some(mut flash) = take_flash() else {
            error!("SPI flash not available for bulk operation");
            finish_operation();
            continue;
        };

        match operation {
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

                let mut buf = [[0u8; BULK_BLOCK_SIZE]; 2];
                let mut channel =
                    Channel::<CriticalSectionRawMutex, [u8; BULK_BLOCK_SIZE]>::new(&mut buf);
                let (mut sender, mut receiver) = channel.split();
                let mut fast_in = FastBulkIn::new_ep2();
                let usb_failed = AtomicBool::new(false);

                let ((), usb_result) = embassy_futures::join::join(
                    async {
                        for _ in 0..block_count {
                            let slot = sender.send().await;
                            if !usb_failed.load(Ordering::Relaxed) {
                                flash.read_block(slot, io_mode).await;
                            }
                            sender.send_done();
                        }
                    },
                    async {
                        let mut result = Ok(());
                        for index in 0..block_count {
                            {
                                let slot = receiver.receive().await;
                                if result.is_ok() {
                                    let write_result = match fast_in.as_mut() {
                                        Some(fast_in) => fast_in.write_block(slot).await,
                                        None => write_bulk_block(&mut ep_in, slot).await,
                                    };
                                    if let Err(error) = write_result {
                                        error!("Bulk IN write error at block {}", index);
                                        usb_failed.store(true, Ordering::Relaxed);
                                        result = Err(error);
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
                let mut buf = [[0u8; BULK_BLOCK_SIZE]; 2];
                let mut channel =
                    Channel::<CriticalSectionRawMutex, [u8; BULK_BLOCK_SIZE]>::new(&mut buf);
                let (mut sender, mut receiver) = channel.split();
                let usb_failed = AtomicBool::new(false);

                let (usb_result, flash_result) = embassy_futures::join::join(
                    async {
                        let mut result = Ok(());
                        for index in 0..block_count {
                            {
                                let slot = sender.send().await;
                                if result.is_ok()
                                    && let Err(error) = read_bulk_block(&mut ep_out, slot).await
                                {
                                    error!("Bulk OUT read error at block {}", index);
                                    usb_failed.store(true, Ordering::Relaxed);
                                    result = Err(error);
                                }
                            }
                            sender.send_done();
                        }
                        result
                    },
                    async {
                        let mut result = Ok(());
                        for _ in 0..block_count {
                            {
                                let slot = receiver.receive().await;
                                if result.is_ok() && !usb_failed.load(Ordering::Relaxed) {
                                    result = flash
                                        .write_page(opcode, address, addr_len, &slot[..PAGE_SIZE])
                                        .await;
                                }
                            }
                            receiver.receive_done();
                            address = address.wrapping_add(PAGE_SIZE as u32);
                        }

                        if result.is_ok() {
                            embassy_time::Timer::after_millis(25).await;
                        }
                        result
                    },
                )
                .await;

                match (usb_result, flash_result) {
                    (Ok(()), Ok(())) => info!("Bulk WRITE complete ({} blocks)", block_count),
                    (_, Err(error)) => error!("Bulk WRITE flash error: {}", error),
                    (Err(_), _) => {}
                }
            }
        }

        put_flash(flash);
        finish_operation();
    }
}

async fn write_bulk_block(
    ep: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    data: &[u8; BULK_BLOCK_SIZE],
) -> Result<(), embassy_usb::driver::EndpointError> {
    for chunk in data.chunks(USB_MAX_PACKET_SIZE as usize) {
        ep.write(chunk).await?;
    }
    Ok(())
}

async fn read_bulk_block(
    ep: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
    buf: &mut [u8; BULK_BLOCK_SIZE],
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut offset = 0;
    while offset < BULK_BLOCK_SIZE {
        let count = ep.read(&mut buf[offset..]).await?;
        if count == 0 {
            return Err(embassy_usb::driver::EndpointError::BufferOverflow);
        }
        offset += count;
    }
    Ok(())
}
