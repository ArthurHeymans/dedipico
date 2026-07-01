use embassy_rp::gpio::{Input, OutputOpenDrain};
use embassy_time::Timer;

pub const FRAME_LEN: usize = 64;

pub const CMD_GPIO_GET_STATE: u8 = 0x01;
pub const CMD_GPIO_SET_DIRECTION: u8 = 0x02;
pub const CMD_GPIO_SET_OUTPUT: u8 = 0x03;
pub const CMD_GPIO_PULSE_LOW: u8 = 0x04;
pub const CMD_UART_SET_BAUD: u8 = 0x10;
pub const CMD_UART_WRITE: u8 = 0x11;

pub const EVT_GPIO_STATE: u8 = 0x81;
pub const EVT_UART_DATA: u8 = 0x90;

const GPIO_COUNT: u8 = 4;
const GPIO_RESET: u8 = 1 << 0;
const GPIO_POWER: u8 = 1 << 1;
const CAP_OPEN_DRAIN: u8 = 1 << 0;
const CAP_PULSE: u8 = 1 << 1;

pub struct BoardGpio<'d> {
    reset: OutputOpenDrain<'d>,
    power: OutputOpenDrain<'d>,
    power_state: Input<'d>,
    aux_state: Input<'d>,
    outputs: u8,
    directions: u8,
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
        }
    }

    pub fn state_frame(&mut self) -> [u8; FRAME_LEN] {
        let mut frame = [0; FRAME_LEN];
        frame[0] = EVT_GPIO_STATE;
        frame[1] = self.inputs();
        frame[2] = self.outputs;
        frame[3] = self.directions;
        frame[4] = GPIO_COUNT | CAP_OPEN_DRAIN | CAP_PULSE;
        frame
    }

    pub fn set_direction(&mut self, mask: u8, directions: u8) {
        self.directions = (self.directions & !mask) | (directions & mask);
        self.apply_outputs();
    }

    pub fn set_output(&mut self, mask: u8, values: u8) {
        self.outputs = (self.outputs & !mask) | (values & mask);
        self.apply_outputs();
    }

    pub async fn pulse_low(&mut self, mask: u8, ms: u16) {
        let pins = mask & (GPIO_RESET | GPIO_POWER);
        let old_directions = self.directions;
        let old_outputs = self.outputs;

        self.directions |= pins;
        self.outputs &= !pins;
        self.apply_outputs();
        Timer::after_millis(ms as u64).await;
        self.directions = old_directions;
        self.outputs = old_outputs;
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
