use embassy_rp::gpio::{Input, OutputOpenDrain};
use embassy_time::{Duration, Instant};

pub const PACKET_LEN: usize = 64;
pub const HEADER_LEN: usize = 3;
pub const MAX_PAYLOAD_LEN: usize = PACKET_LEN - HEADER_LEN;

pub const CMD_GPIO_GET_STATE: u8 = 0x01;
pub const CMD_GPIO_SET_DIRECTION: u8 = 0x02;
pub const CMD_GPIO_SET_OUTPUT: u8 = 0x03;
pub const CMD_GPIO_PULSE_LOW: u8 = 0x04;
pub const CMD_UART_SET_BAUD: u8 = 0x10;
pub const CMD_UART_WRITE: u8 = 0x11;

pub const EVT_RESPONSE: u8 = 0x80;
pub const EVT_GPIO_STATE: u8 = 0x81;
pub const EVT_UART_DATA: u8 = 0x90;

pub const STATUS_OK: u8 = 0;
pub const STATUS_INVALID: u8 = 1;
pub const STATUS_BUSY: u8 = 2;

const GPIO_COUNT: u8 = 4;
const GPIO_RESET: u8 = 1 << 0;
const GPIO_POWER: u8 = 1 << 1;
const CAP_OPEN_DRAIN: u8 = 1 << 0;
const CAP_PULSE: u8 = 1 << 1;

#[derive(Clone, Copy)]
struct Pulse {
    pins: u8,
    old_directions: u8,
    old_outputs: u8,
    deadline: Instant,
}

pub struct BoardGpio<'d> {
    reset: OutputOpenDrain<'d>,
    power: OutputOpenDrain<'d>,
    power_state: Input<'d>,
    aux_state: Input<'d>,
    outputs: u8,
    directions: u8,
    pulse: Option<Pulse>,
}

impl<'d> BoardGpio<'d> {
    pub fn new(
        reset: OutputOpenDrain<'d>,
        power: OutputOpenDrain<'d>,
        power_state: Input<'d>,
        aux_state: Input<'d>,
    ) -> Self {
        Self {
            reset,
            power,
            power_state,
            aux_state,
            outputs: GPIO_RESET | GPIO_POWER,
            directions: 0,
            pulse: None,
        }
    }

    pub fn state(&mut self) -> [u8; 4] {
        [
            self.inputs(),
            self.outputs,
            self.directions,
            GPIO_COUNT | CAP_OPEN_DRAIN | CAP_PULSE,
        ]
    }

    pub fn set_direction(&mut self, mask: u8, directions: u8) {
        self.finish_pulse();
        self.directions = (self.directions & !mask) | (directions & mask);
        self.apply_outputs();
    }

    pub fn set_output(&mut self, mask: u8, values: u8) {
        self.finish_pulse();
        self.outputs = (self.outputs & !mask) | (values & mask);
        self.apply_outputs();
    }

    pub fn start_pulse_low(&mut self, mask: u8, duration_ms: u16) {
        self.finish_pulse();

        let pins = mask & (GPIO_RESET | GPIO_POWER);
        let pulse = Pulse {
            pins,
            old_directions: self.directions,
            old_outputs: self.outputs,
            deadline: Instant::now() + Duration::from_millis(duration_ms as u64),
        };

        self.directions |= pins;
        self.outputs &= !pins;
        self.apply_outputs();
        self.pulse = Some(pulse);
    }

    pub fn pulse_deadline(&self) -> Option<Instant> {
        self.pulse.map(|pulse| pulse.deadline)
    }

    pub fn finish_pulse_if_due(&mut self) -> bool {
        if self
            .pulse
            .is_some_and(|pulse| pulse.deadline <= Instant::now())
        {
            self.finish_pulse();
            true
        } else {
            false
        }
    }

    fn finish_pulse(&mut self) {
        let Some(pulse) = self.pulse.take() else {
            return;
        };

        self.directions = (self.directions & !pulse.pins) | (pulse.old_directions & pulse.pins);
        self.outputs = (self.outputs & !pulse.pins) | (pulse.old_outputs & pulse.pins);
        self.apply_outputs();
    }

    fn inputs(&mut self) -> u8 {
        let mut value = 0;
        if self.reset.is_high() {
            value |= GPIO_RESET;
        }
        if self.power.is_high() {
            value |= GPIO_POWER;
        }
        if self.power_state.is_high() {
            value |= 1 << 2;
        }
        if self.aux_state.is_high() {
            value |= 1 << 3;
        }
        value
    }

    fn apply_outputs(&mut self) {
        self.apply_pin(GPIO_RESET);
        self.apply_pin(GPIO_POWER);
    }

    fn apply_pin(&mut self, bit: u8) {
        let drive = self.directions & bit != 0 && self.outputs & bit == 0;
        match bit {
            GPIO_RESET => {
                if drive {
                    self.reset.set_low();
                } else {
                    self.reset.set_high();
                }
            }
            GPIO_POWER => {
                if drive {
                    self.power.set_low();
                } else {
                    self.power.set_high();
                }
            }
            _ => {}
        }
    }
}
