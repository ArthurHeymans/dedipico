#![no_std]
#![no_main]

mod aux;
mod aux_usb;
mod bulk;
mod config;
mod fast_bulk_in;
mod leds;
mod protocol;
mod spi_flash;
mod spi_flash_programs;
mod usb_handler;

use core::sync::atomic::{Ordering, compiler_fence};

use dedipico_protocol::identity::{DeviceIdentity, UNIQUE_ID_LEN};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma::{Channel as DmaChannel, InterruptHandler as DmaInterruptHandler};
use embassy_rp::flash::Blocking;
use embassy_rp::gpio::{Flex, Input, Level, Output, OutputOpenDrain, Pull};
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_0, PIN_1, PIO0, PIO1, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::Builder;
use embassy_usb::driver::{Direction, EndpointAddress};
use panic_probe as _;
use static_cell::StaticCell;

use crate::aux::*;
use crate::config::*;
use crate::leds::Leds;
use crate::spi_flash::SpiFlash;
use crate::usb_handler::DediprogHandler;

// =============================================================================
// Interrupt bindings
// =============================================================================

bind_interrupts!(pub(crate) struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>, DmaInterruptHandler<DMA_CH1>;
});

const ONBOARD_FLASH_SIZE: usize = 2 * 1024 * 1024;
const FLASH_WP_PIN: usize = 5;
const FLASH_HOLD_PIN: usize = 6;
const FLASH_CS_PIN: usize = 7;
const FLASH_IDLE_HIGH_MASK: u32 = (1 << FLASH_WP_PIN) | (1 << FLASH_HOLD_PIN) | (1 << FLASH_CS_PIN);

// =============================================================================
// USB device type alias
// =============================================================================

pub(crate) type UsbDriver = Driver<'static, USB>;

// =============================================================================
// Entry point
// =============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("DediPico starting up");

    let mut onboard_flash =
        embassy_rp::flash::Flash::<_, Blocking, ONBOARD_FLASH_SIZE>::new_blocking(p.FLASH);
    let mut unique_id = [0; UNIQUE_ID_LEN];
    if onboard_flash.blocking_unique_id(&mut unique_id).is_err() {
        warn!("Unable to read onboard flash unique ID");
    }
    static IDENTITY: StaticCell<DeviceIdentity> = StaticCell::new();
    let identity = IDENTITY.init(DeviceIdentity::from_unique_id(unique_id));

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
    bulk::install_flash(SpiFlash::new(
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
    usb_config.serial_number = Some(identity.usb_serial());
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
    let handler = HANDLER.init(DediprogHandler::new(leds, identity));
    builder.handler(handler);

    // ---- Vendor-class interface with bulk endpoints ----
    //
    // flashprog hard-codes:  EP1 OUT (0x01) for SF600, EP2 IN (0x82) for all.
    // embassy-usb 0.5+ lets us specify exact endpoint addresses.
    let ep1_out = EndpointAddress::from_parts(1, Direction::Out);
    let ep2_in = EndpointAddress::from_parts(2, Direction::In);
    let ep15_in = EndpointAddress::from_parts(15, Direction::In);

    let mut func = builder.function(0xFF, 0x00, 0x00);
    let mut iface = func.interface();
    let mut alt = iface.alt_setting(0xFF, 0x00, 0x00, None);

    let ep_out = alt.endpoint_bulk_out(Some(ep1_out), USB_MAX_PACKET_SIZE);
    let ep_in = alt.endpoint_bulk_in(Some(ep2_in), USB_MAX_PACKET_SIZE);
    // Embassy does not yet expose RP2040 double-buffer allocation. Reserve the
    // buffer immediately following EP2 so FastBulkIn can safely use it. This is
    // deliberately an interrupt endpoint: flashprog selects the first bulk IN
    // endpoint on interface 0 and must never mistake the reservation for EP2.
    let _ep2_double_buffer_reservation =
        alt.endpoint_interrupt_in(Some(ep15_in), USB_MAX_PACKET_SIZE, 1);

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
    spawner.spawn(
        aux_task(
            spawner,
            aux_out,
            aux_in,
            uart_pio,
            uart_tx_pin,
            uart_rx_pin,
            board_gpio,
        )
        .unwrap(),
    );

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
// Auxiliary vendor USB task
// =============================================================================

#[embassy_executor::task]
async fn aux_task(
    spawner: Spawner,
    ep_out: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
    ep_in: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    uart_pio: Peri<'static, PIO1>,
    uart_tx_pin: Peri<'static, PIN_0>,
    uart_rx_pin: Peri<'static, PIN_1>,
    gpio: BoardGpio<'static>,
) {
    aux_usb::run(
        spawner,
        ep_out,
        ep_in,
        uart_pio,
        uart_tx_pin,
        uart_rx_pin,
        gpio,
    )
    .await;
}

// =============================================================================
// Bulk worker task
// =============================================================================

#[embassy_executor::task]
async fn bulk_worker_task(
    ep_in: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    ep_out: <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut,
) {
    bulk::run(ep_in, ep_out).await;
}
