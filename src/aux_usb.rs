use dedipico_protocol::aux::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either4, select4};
use embassy_rp::Peri;
use embassy_rp::pac;
use embassy_rp::peripherals::{PIN_0, PIN_1, PIO1};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::driver::{Endpoint as _, EndpointIn as _, EndpointOut as _};
use fixed::traits::ToFixed;
use fixed::types::extra::U8;

use crate::aux::BoardGpio;
use crate::{Irqs, UsbDriver};

static UART_RX: Channel<CriticalSectionRawMutex, u8, 256> = Channel::new();
static UART_TX: Channel<CriticalSectionRawMutex, u8, 256> = Channel::new();

#[embassy_executor::task]
async fn uart_rx_task(mut uart_rx: PioUartRx<'static, PIO1, 1>) {
    loop {
        let byte = uart_rx.read_u8().await;
        let _ = UART_RX.try_send(byte);
    }
}

#[embassy_executor::task]
async fn uart_tx_task(mut uart_tx: PioUartTx<'static, PIO1, 0>) {
    loop {
        let byte = UART_TX.receive().await;
        let _ = embedded_io_async::Write::write_all(&mut uart_tx, &[byte]).await;
    }
}

pub async fn run(
    spawner: Spawner,
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

    let tx_program = PioUartTxProgram::new(&mut common);
    let uart_tx = PioUartTx::new(115_200, &mut common, sm0, uart_tx_pin, &tx_program);
    let rx_program = PioUartRxProgram::new(&mut common);
    let uart_rx = PioUartRx::new(115_200, &mut common, sm1, uart_rx_pin, &rx_program);
    spawner.spawn(uart_rx_task(uart_rx).unwrap());
    spawner.spawn(uart_tx_task(uart_tx).unwrap());

    let mut usb_buf = [0u8; PACKET_LEN];
    let mut uart_buf = [0u8; MAX_PAYLOAD_LEN];
    let mut uart_len = 0;
    let mut uart_deadline = None;

    loop {
        ep_out.wait_enabled().await;
        ep_in.wait_enabled().await;

        let flush_at = uart_deadline.unwrap_or(Instant::MAX);
        let pulse_at = gpio.pulse_deadline().unwrap_or(Instant::MAX);

        match select4(
            ep_out.read(&mut usb_buf),
            Timer::at(pulse_at),
            Timer::at(flush_at),
            UART_RX.receive(),
        )
        .await
        {
            Either4::First(Ok(count)) => {
                handle_packet(&mut ep_in, &mut gpio, &usb_buf[..count]).await;
            }
            Either4::First(Err(_)) => {}
            Either4::Second(()) => {
                if gpio.finish_pulse_if_due() {
                    write_packet(&mut ep_in, EVT_GPIO_STATE, 0, &gpio.state()).await;
                }
            }
            Either4::Third(()) => {
                flush_uart(&mut ep_in, &mut uart_buf, &mut uart_len).await;
                uart_deadline = None;
            }
            Either4::Fourth(byte) => {
                if uart_len == 0 {
                    uart_deadline = Some(Instant::now() + Duration::from_millis(1));
                }
                uart_buf[uart_len] = byte;
                uart_len += 1;
                if uart_len == uart_buf.len() {
                    flush_uart(&mut ep_in, &mut uart_buf, &mut uart_len).await;
                    uart_deadline = None;
                }
            }
        }
    }
}

async fn flush_uart(
    ep_in: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    uart_buf: &mut [u8; MAX_PAYLOAD_LEN],
    uart_len: &mut usize,
) {
    if *uart_len != 0 {
        write_packet(ep_in, EVT_UART_DATA, 0, &uart_buf[..*uart_len]).await;
        *uart_len = 0;
    }
}

async fn write_packet(
    ep_in: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    kind: u8,
    request_id: u8,
    payload: &[u8],
) {
    let mut packet = [0u8; PACKET_LEN];
    if let Some(packet) = encode(&mut packet, kind, request_id, payload) {
        let _ = ep_in.write(packet).await;
    }
}

async fn write_response(
    ep_in: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    request_id: u8,
    command: u8,
    status: u8,
    data: &[u8],
) {
    let data = &data[..data.len().min(MAX_PAYLOAD_LEN - 2)];
    let mut response = [0u8; MAX_PAYLOAD_LEN];
    response[0] = command;
    response[1] = status;
    response[2..2 + data.len()].copy_from_slice(data);
    write_packet(ep_in, EVT_RESPONSE, request_id, &response[..2 + data.len()]).await;
}

async fn handle_packet(
    ep_in: &mut <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    gpio: &mut BoardGpio<'static>,
    packet: &[u8],
) {
    let Some(data) = payload(packet) else {
        return;
    };
    let command = packet[0];
    let request_id = packet[1];

    let status = match command {
        CMD_GPIO_GET_STATE if data.is_empty() => STATUS_OK,
        CMD_GPIO_SET_DIRECTION if data.len() == 2 => {
            gpio.set_direction(data[0], data[1]);
            STATUS_OK
        }
        CMD_GPIO_SET_OUTPUT if data.len() == 2 => {
            gpio.set_output(data[0], data[1]);
            STATUS_OK
        }
        CMD_GPIO_PULSE_LOW if data.len() == 3 => {
            gpio.start_pulse_low(data[0], u16::from_le_bytes([data[1], data[2]]));
            STATUS_OK
        }
        CMD_UART_SET_BAUD if data.len() == 4 => {
            set_baudrate(u32::from_le_bytes([data[0], data[1], data[2], data[3]]));
            STATUS_OK
        }
        CMD_UART_WRITE if UART_TX.free_capacity() >= data.len() => {
            for &byte in data {
                let _ = UART_TX.try_send(byte);
            }
            STATUS_OK
        }
        CMD_UART_WRITE => STATUS_BUSY,
        _ => STATUS_INVALID,
    };

    if request_id != 0 {
        let state = match command {
            CMD_GPIO_GET_STATE
            | CMD_GPIO_SET_DIRECTION
            | CMD_GPIO_SET_OUTPUT
            | CMD_GPIO_PULSE_LOW
                if status == STATUS_OK =>
            {
                Some(gpio.state())
            }
            _ => None,
        };
        write_response(
            ep_in,
            request_id,
            command,
            status,
            state.as_ref().map_or(&[], |state| state.as_slice()),
        )
        .await;
    }
}

fn set_baudrate(baudrate: u32) {
    let baudrate = baudrate.clamp(300, 3_000_000);
    let divider =
        (embassy_rp::clocks::clk_sys_freq() / (8 * baudrate)).to_fixed::<fixed::FixedU32<U8>>();

    for state_machine in 0..=1 {
        pac::PIO1
            .sm(state_machine)
            .clkdiv()
            .write(|w| w.0 = divider.to_bits() << 8);
    }
}
