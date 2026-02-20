/// LED control for Pass / Busy / Error indicators.
use embassy_rp::gpio::Output;

use crate::config;

pub struct Leds<'d> {
    pass: Output<'d>,
    busy: Output<'d>,
    error: Output<'d>,
}

impl<'d> Leds<'d> {
    pub fn new(pass: Output<'d>, busy: Output<'d>, error: Output<'d>) -> Self {
        Self { pass, busy, error }
    }

    /// Set LED state from the raw wValue of CMD_SET_IO_LED (Protocol V2).
    ///
    /// The host sends `wValue = (leds ^ 7) << 8` where bits are:
    ///   bit0 = PASS, bit1 = BUSY, bit2 = ERROR
    /// The XOR with 7 inverts the bits (original hardware: 0 = on).
    /// We invert back to get active-high logic for our GPIOs.
    pub fn set_from_wvalue(&mut self, wvalue: u16) {
        let raw = ((wvalue >> 8) as u8) ^ 0x07;
        self.set(raw);
    }

    /// Set LED state directly (bit0=PASS, bit1=BUSY, bit2=ERROR, 1=on).
    pub fn set(&mut self, state: u8) {
        if state & config::LED_PASS != 0 {
            self.pass.set_high();
        } else {
            self.pass.set_low();
        }
        if state & config::LED_BUSY != 0 {
            self.busy.set_high();
        } else {
            self.busy.set_low();
        }
        if state & config::LED_ERROR != 0 {
            self.error.set_high();
        } else {
            self.error.set_low();
        }
    }

    /// All LEDs off.
    #[allow(dead_code)]
    pub fn all_off(&mut self) {
        self.set(0);
    }

    /// Set the BUSY LED.
    #[allow(dead_code)]
    pub fn set_busy(&mut self, on: bool) {
        if on {
            self.busy.set_high();
        } else {
            self.busy.set_low();
        }
    }
}
