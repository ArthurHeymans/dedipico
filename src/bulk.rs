use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel::Channel;
use embassy_time::{Duration, with_timeout};
use embassy_usb::driver::{EndpointIn as _, EndpointOut as _};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use crate::UsbDriver;
use crate::config::{BULK_BLOCK_SIZE, PAGE_SIZE, USB_MAX_PACKET_SIZE};
use crate::fast_bulk_in::FastBulkIn;
use crate::leds::set_error;
use crate::protocol::BulkOperation;
use crate::spi_flash::{SpiError, SpiFlash};

const USB_PROGRESS_TIMEOUT: Duration = Duration::from_millis(3_500);
const BULK_OUT_EP: usize = 1;
const BULK_IN_EP: usize = 2;

struct BulkState {
    pending: Option<QueuedOperation>,
    active: bool,
}

struct QueuedOperation {
    operation: BulkOperation,
    generation: u32,
}

static SPI_FLASH: Mutex<RefCell<Option<SpiFlash<'static>>>> = Mutex::new(RefCell::new(None));
static BULK_STATE: Mutex<RefCell<BulkState>> = Mutex::new(RefCell::new(BulkState {
    pending: None,
    active: false,
}));
static BULK_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static CANCEL_GENERATION: AtomicU32 = AtomicU32::new(0);

pub fn install_flash(flash: SpiFlash<'static>) {
    put_flash(flash);
}

fn take_flash() -> Option<SpiFlash<'static>> {
    critical_section::with(|cs| SPI_FLASH.borrow(cs).borrow_mut().take())
}

pub fn take_flash_for_control() -> Option<SpiFlash<'static>> {
    // embassy-usb invokes control handlers synchronously. There must never be
    // an await between this take and put_flash(): doing so would let another
    // handler observe the flash as unavailable without an active bulk owner.
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
    let is_read = matches!(&operation, BulkOperation::Read { .. });
    let (accepted, recover_endpoint) = critical_section::with(|cs| {
        let mut state = BULK_STATE.borrow(cs).borrow_mut();
        // The host may submit the next setup as soon as its final bulk packet
        // completes, while the worker still has buffered flash work. Preserve
        // the protocol's original capacity-one follow-up queue.
        if state.pending.is_some() {
            (false, false)
        } else {
            let recover_endpoint = !state.active;
            state.pending = Some(QueuedOperation {
                operation,
                generation: CANCEL_GENERATION.load(Ordering::SeqCst),
            });
            (true, recover_endpoint)
        }
    });
    if accepted {
        // When no operation is active, reset any endpoint state left by an
        // earlier timeout before ACKing this setup. A queued follow-up must not
        // touch the endpoint still used by the active operation.
        if recover_endpoint {
            if is_read {
                prepare_bulk_in(false);
            } else {
                prepare_bulk_out(false);
            }
        }
        BULK_SIGNAL.signal(());
    }
    accepted
}

/// Cancel work belonging to the current USB configuration.
pub fn cancel() {
    let cancelled_generation = CANCEL_GENERATION.fetch_add(1, Ordering::SeqCst);
    discard_pending(cancelled_generation, false);
    BULK_SIGNAL.signal(());
}

fn discard_pending(generation: u32, keep_read: bool) {
    critical_section::with(|cs| {
        let mut state = BULK_STATE.borrow(cs).borrow_mut();
        let discard = state.pending.as_ref().is_some_and(|pending| {
            pending.generation == generation
                && !(keep_read && matches!(pending.operation, BulkOperation::Read { .. }))
        });
        if discard {
            state.pending = None;
        }
    });
}

fn is_cancelled(generation: u32) -> bool {
    CANCEL_GENERATION.load(Ordering::SeqCst) != generation
}

fn take_operation() -> Option<QueuedOperation> {
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

#[derive(Clone, Copy, defmt::Format)]
enum BulkUsbError {
    Endpoint,
    Timeout,
}

fn set_endpoint_stalled(index: usize, input: bool, stalled: bool) {
    let dpram = embassy_rp::pac::USB_DPRAM;
    if input {
        dpram
            .ep_in_buffer_control(index)
            .modify(|w| w.set_stall(stalled));
    } else {
        dpram
            .ep_out_buffer_control(index)
            .modify(|w| w.set_stall(stalled));
    }
}

fn prepare_bulk_in(stalled: bool) {
    let dpram = embassy_rp::pac::USB_DPRAM;
    dpram.ep_in_control(BULK_IN_EP - 1).modify(|w| {
        w.set_interrupt_per_buff(true);
        w.set_interrupt_per_double_buff(false);
        w.set_double_buffered(false);
    });
    dpram.ep_in_buffer_control(BULK_IN_EP).write(|w| {
        w.set_reset(true);
        w.set_pid(0, true);
        w.set_pid(1, false);
        w.set_stall(stalled);
    });
}

fn prepare_bulk_out(stalled: bool) {
    let control = embassy_rp::pac::USB_DPRAM.ep_out_buffer_control(BULK_OUT_EP);
    control.write(|w| {
        w.set_reset(true);
        w.set_pid(0, false);
        w.set_length(0, USB_MAX_PACKET_SIZE);
        w.set_stall(stalled);
    });
    if !stalled {
        cortex_m::asm::delay(12);
        control.write(|w| {
            w.set_pid(0, false);
            w.set_length(0, USB_MAX_PACKET_SIZE);
            w.set_available(0, true);
        });
    }
}

fn fail_operation(operation: &BulkOperation) {
    match operation {
        BulkOperation::Read { .. } => prepare_bulk_in(true),
        BulkOperation::Write { .. } => prepare_bulk_out(true),
    }
    set_error(true);
}

async fn wait_enabled(
    endpoint: &mut impl embassy_usb::driver::Endpoint,
) -> Result<(), BulkUsbError> {
    with_timeout(USB_PROGRESS_TIMEOUT, endpoint.wait_enabled())
        .await
        .map_err(|_| BulkUsbError::Timeout)
}

pub async fn run(
    mut ep_in: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    mut ep_out: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
) {
    loop {
        BULK_SIGNAL.wait().await;

        let Some(QueuedOperation {
            operation,
            generation,
        }) = take_operation()
        else {
            continue;
        };

        let Some(mut flash) = take_flash() else {
            error!("SPI flash not available for bulk operation");
            fail_operation(&operation);
            discard_pending(generation, false);
            finish_operation();
            continue;
        };

        let (operation_failed, keep_verify_read) = match &operation {
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
                    *address, *block_count, *opcode, *io_mode, *dummy_cycles
                );

                let enabled = wait_enabled(&mut ep_in).await;
                let start_result = if enabled.is_ok() {
                    flash
                        .start_read(
                            *opcode,
                            *address,
                            *addr_len,
                            *io_mode,
                            *mode_byte,
                            *dummy_cycles,
                            || is_cancelled(generation),
                        )
                        .await
                } else {
                    Err(crate::spi_flash::SpiError::Cancelled)
                };

                let mut buf = [[0u8; BULK_BLOCK_SIZE]; 2];
                let mut channel =
                    Channel::<CriticalSectionRawMutex, [u8; BULK_BLOCK_SIZE]>::new(&mut buf);
                let (mut sender, mut receiver) = channel.split();
                let mut fast_in = FastBulkIn::new_ep2();
                let usb_failed = AtomicBool::new(enabled.is_err());
                let flash_failed = AtomicBool::new(start_result.is_err());

                if let Err(error) = enabled {
                    error!("Bulk READ endpoint unavailable: {}", error);
                }
                if let Err(error) = start_result {
                    error!("Bulk READ setup failed: {}", error);
                    set_endpoint_stalled(BULK_IN_EP, true, true);
                }

                let ((), usb_result) = embassy_futures::join::join(
                    async {
                        for _ in 0..*block_count {
                            let slot = sender.send().await;
                            if !usb_failed.load(Ordering::Relaxed)
                                && !flash_failed.load(Ordering::Relaxed)
                                && let Err(error) = flash.read_block(slot, *io_mode).await
                            {
                                error!("Bulk READ flash error: {}", error);
                                flash_failed.store(true, Ordering::Relaxed);
                            }
                            sender.send_done();
                        }
                    },
                    async {
                        let mut result = Ok(());
                        for index in 0..*block_count {
                            {
                                let slot = receiver.receive().await;
                                if flash_failed.load(Ordering::Relaxed) {
                                    set_endpoint_stalled(BULK_IN_EP, true, true);
                                } else if result.is_ok() {
                                    let write_result = match fast_in.as_mut() {
                                        Some(fast_in) => match with_timeout(
                                            USB_PROGRESS_TIMEOUT,
                                            fast_in.write_block(slot),
                                        )
                                        .await
                                        {
                                            Ok(result) => {
                                                result.map_err(|_| BulkUsbError::Endpoint)
                                            }
                                            Err(_) => Err(BulkUsbError::Timeout),
                                        },
                                        None => write_bulk_block(&mut ep_in, slot).await,
                                    };
                                    if let Err(error) = write_result {
                                        error!("Bulk IN write error at block {}: {}", index, error);
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
                let failed = flash_failed.load(Ordering::Relaxed) || usb_result.is_err();
                if !failed {
                    info!("Bulk READ complete ({} blocks)", *block_count);
                }
                (failed, false)
            }
            BulkOperation::Write {
                address,
                block_count,
                opcode,
                addr_len,
            } => {
                info!(
                    "Bulk WRITE: addr=0x{:08x} blocks={} opcode=0x{:02x}",
                    *address, *block_count, *opcode
                );

                let enabled = wait_enabled(&mut ep_out).await;
                let mut buf = [[0u8; BULK_BLOCK_SIZE]; 2];
                let mut channel =
                    Channel::<CriticalSectionRawMutex, [u8; BULK_BLOCK_SIZE]>::new(&mut buf);
                let (mut sender, mut receiver) = channel.split();
                let usb_failed = AtomicBool::new(enabled.is_err());
                let flash_failed = AtomicBool::new(false);

                if let Err(error) = enabled {
                    error!("Bulk WRITE endpoint unavailable: {}", error);
                }

                let (usb_result, flash_result) = embassy_futures::join::join(
                    async {
                        let mut result = enabled;
                        for index in 0..*block_count {
                            {
                                let slot = sender.send().await;
                                if flash_failed.load(Ordering::Relaxed) {
                                    set_endpoint_stalled(BULK_OUT_EP, false, true);
                                } else if result.is_ok()
                                    && let Err(error) = read_bulk_block(&mut ep_out, slot).await
                                {
                                    error!("Bulk OUT read error at block {}: {}", index, error);
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
                        let mut next_address = *address;
                        for _ in 0..*block_count {
                            {
                                let slot = receiver.receive().await;
                                if result.is_ok() && !usb_failed.load(Ordering::Relaxed) {
                                    result = flash
                                        .write_page(
                                            *opcode,
                                            next_address,
                                            *addr_len,
                                            &slot[..PAGE_SIZE],
                                            || is_cancelled(generation),
                                        )
                                        .await;
                                    if result.is_err() {
                                        flash_failed.store(true, Ordering::Relaxed);
                                    }
                                }
                            }
                            receiver.receive_done();
                            next_address = next_address.wrapping_add(PAGE_SIZE as u32);
                        }

                        if result.is_ok() {
                            embassy_time::Timer::after_millis(25).await;
                        }
                        result
                    },
                )
                .await;

                if let Err(error) = flash_result {
                    error!("Bulk WRITE flash error: {}", error);
                } else if usb_result.is_ok() {
                    info!("Bulk WRITE complete ({} blocks)", *block_count);
                }
                let failed = usb_result.is_err() || flash_result.is_err();
                let keep_verify_read = flash_result.is_err()
                    && !usb_result.is_err()
                    && !matches!(flash_result, Err(SpiError::Cancelled));
                (failed, keep_verify_read)
            }
        };

        put_flash(flash);
        if operation_failed {
            fail_operation(&operation);
            discard_pending(generation, keep_verify_read);
        }
        finish_operation();
    }
}

async fn write_bulk_block(
    ep: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    data: &[u8; BULK_BLOCK_SIZE],
) -> Result<(), BulkUsbError> {
    for chunk in data.chunks(USB_MAX_PACKET_SIZE as usize) {
        match with_timeout(USB_PROGRESS_TIMEOUT, ep.write(chunk)).await {
            Ok(result) => result.map_err(|_| BulkUsbError::Endpoint)?,
            Err(_) => return Err(BulkUsbError::Timeout),
        }
    }
    Ok(())
}

async fn read_bulk_block(
    ep: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
    buf: &mut [u8; BULK_BLOCK_SIZE],
) -> Result<(), BulkUsbError> {
    let mut offset = 0;
    while offset < BULK_BLOCK_SIZE {
        let count = match with_timeout(USB_PROGRESS_TIMEOUT, ep.read(&mut buf[offset..])).await {
            Ok(result) => result.map_err(|_| BulkUsbError::Endpoint)?,
            Err(_) => return Err(BulkUsbError::Timeout),
        };
        if count == 0 {
            return Err(BulkUsbError::Endpoint);
        }
        offset += count;
    }
    Ok(())
}
