/// SPI flash operations over a lane-aware GPIO bus.
///
/// The RP2040 PL022 SPI block only supports classic 1-1-1 SPI. This GPIO
/// transport keeps the command/write/status paths single-lane, but can switch
/// the bulk-read address/data phases to Dediprog's dual/quad modes:
/// 1-1-2, 1-2-2, 1-1-4 and 1-4-4.
use cortex_m::asm;
use embassy_rp::gpio::{Flex, Level, Output, Pull};

use crate::config;
use crate::protocol::IoMode;

pub struct SpiFlash<'d> {
    io0: Flex<'d>,
    io1: Flex<'d>,
    io2: Flex<'d>,
    io3: Flex<'d>,
    sck: Output<'d>,
    cs: Output<'d>,
    half_period_nops: u16,
}

impl<'d> SpiFlash<'d> {
    pub fn new(
        mut io0: Flex<'d>,
        mut io1: Flex<'d>,
        mut io2: Flex<'d>,
        mut io3: Flex<'d>,
        mut sck: Output<'d>,
        mut cs: Output<'d>,
    ) -> Self {
        io0.set_pull(Pull::None);
        io1.set_pull(Pull::None);
        io2.set_pull(Pull::Up);
        io3.set_pull(Pull::Up);

        sck.set_low();
        cs.set_high();

        let mut this = Self {
            io0,
            io1,
            io2,
            io3,
            sck,
            cs,
            half_period_nops: 0,
        };
        this.idle_io();
        this.set_frequency(config::DEFAULT_SPI_FREQ_HZ);
        this
    }

    // =========================================================================
    // Runtime bus clock approximation
    // =========================================================================

    /// Set an approximate bit-bang half-period. GPIO method-call overhead is a
    /// large part of the real period, so high requested speeds intentionally map
    /// to zero extra NOPs. Slower flashprog `spispeed=` values still get extra
    /// spacing and are useful for marginal wiring.
    pub fn set_frequency(&mut self, freq_hz: u32) {
        let freq_hz = freq_hz.max(1);
        let ideal_half_cycles = 125_000_000u32 / freq_hz / 2;
        self.half_period_nops = ideal_half_cycles.saturating_sub(16).min(u16::MAX as u32) as u16;
    }

    #[inline(always)]
    fn delay_half_period(&self) {
        for _ in 0..self.half_period_nops {
            asm::nop();
        }
    }

    // =========================================================================
    // CS and idle pin control
    // =========================================================================

    #[inline]
    pub fn cs_assert(&mut self) {
        self.sck.set_low();
        self.cs.set_low();
    }

    #[inline]
    pub fn cs_deassert(&mut self) {
        self.sck.set_low();
        self.cs.set_high();
        self.idle_io();
    }

    fn idle_io(&mut self) {
        // IO0 idles low as MOSI. IO1 is MISO. IO2/IO3 are /WP and /HOLD in
        // single-lane mode, so keep them high unless a multi-I/O transaction
        // temporarily takes ownership of the lanes.
        self.io0.set_low();
        self.io0.set_as_output();
        self.io1.set_as_input();
        self.io2.set_high();
        self.io2.set_as_output();
        self.io3.set_high();
        self.io3.set_as_output();
    }

    fn set_lane_dirs(&mut self, width: u8, output: bool) {
        if output {
            self.io0.set_as_output();
            if width >= 2 {
                self.io1.set_as_output();
            } else {
                self.io1.set_as_input();
            }
            if width >= 4 {
                self.io2.set_as_output();
                self.io3.set_as_output();
            } else {
                self.io2.set_high();
                self.io2.set_as_output();
                self.io3.set_high();
                self.io3.set_as_output();
            }
        } else {
            self.io0.set_as_input();
            self.io1.set_as_input();
            if width >= 4 {
                self.io2.set_as_input();
                self.io3.set_as_input();
            } else {
                // Not participating in dual-output reads. Hold legacy /WP and
                // /HOLD high.
                self.io2.set_high();
                self.io2.set_as_output();
                self.io3.set_high();
                self.io3.set_as_output();
            }
        }
    }

    #[inline(always)]
    fn set_lane_values(&mut self, value: u8, width: u8) {
        self.io0.set_level(if value & 0x1 != 0 {
            Level::High
        } else {
            Level::Low
        });
        if width >= 2 {
            self.io1.set_level(if value & 0x2 != 0 {
                Level::High
            } else {
                Level::Low
            });
        }
        if width >= 4 {
            self.io2.set_level(if value & 0x4 != 0 {
                Level::High
            } else {
                Level::Low
            });
            self.io3.set_level(if value & 0x8 != 0 {
                Level::High
            } else {
                Level::Low
            });
        }
    }

    #[inline(always)]
    fn read_lane_values(&self, width: u8) -> u8 {
        if width == 1 {
            return if self.io1.is_high() { 0x1 } else { 0x0 };
        }

        let mut value = 0;
        if self.io0.is_high() {
            value |= 0x1;
        }
        if width >= 2 && self.io1.is_high() {
            value |= 0x2;
        }
        if width >= 4 {
            if self.io2.is_high() {
                value |= 0x4;
            }
            if self.io3.is_high() {
                value |= 0x8;
            }
        }
        value
    }

    #[inline(always)]
    fn clock_high_sample(&mut self, width: u8) -> u8 {
        self.delay_half_period();
        self.sck.set_high();
        self.delay_half_period();
        let value = self.read_lane_values(width);
        self.sck.set_low();
        value
    }

    #[inline(always)]
    fn clock_write(&mut self) {
        self.delay_half_period();
        self.sck.set_high();
        self.delay_half_period();
        self.sck.set_low();
    }

    fn write_byte_width(&mut self, value: u8, width: u8) {
        self.set_lane_dirs(width, true);
        let mut shift = 8u8 - width;
        loop {
            self.set_lane_values((value >> shift) & ((1 << width) - 1), width);
            self.clock_write();
            if shift == 0 {
                break;
            }
            shift -= width;
        }
    }

    fn read_byte_width(&mut self, width: u8) -> u8 {
        self.set_lane_dirs(width, false);
        let mut value = 0u8;
        let mut bits = 0;
        while bits < 8 {
            value <<= width;
            value |= self.clock_high_sample(width) & ((1 << width) - 1);
            bits += width;
        }
        value
    }

    fn dummy_clocks(&mut self, width: u8, cycles: u8) {
        self.set_lane_dirs(width, false);
        for _ in 0..cycles {
            self.clock_high_sample(width);
        }
    }

    fn write_address(&mut self, address: u32, addr_len: u8, width: u8) {
        if addr_len == 4 {
            self.write_byte_width((address >> 24) as u8, width);
        }
        self.write_byte_width((address >> 16) as u8, width);
        self.write_byte_width((address >> 8) as u8, width);
        self.write_byte_width(address as u8, width);
    }

    // =========================================================================
    // Blocking operations (called from USB handler context via critical_section)
    // =========================================================================

    /// Perform a standard 1-1-1 transceive: write command bytes, then read
    /// response bytes while CS remains asserted.
    pub fn transceive_blocking(&mut self, write_data: &[u8], read_buf: &mut [u8]) {
        self.cs_assert();
        for &byte in write_data {
            self.write_byte_width(byte, 1);
        }
        for byte in read_buf.iter_mut() {
            *byte = self.read_byte_width(1);
        }
        self.cs_deassert();
    }

    /// Write-only standard 1-1-1 transaction.
    pub fn write_only_blocking(&mut self, write_data: &[u8]) {
        self.cs_assert();
        for &byte in write_data {
            self.write_byte_width(byte, 1);
        }
        self.cs_deassert();
    }

    // =========================================================================
    // Async operations (called from bulk worker task)
    // =========================================================================

    /// Begin a read sequence. Opcode is always 1-bit unless QPI is requested.
    /// Address/mode/data phases follow the current Dediprog I/O mode.
    pub async fn start_read(
        &mut self,
        opcode: u8,
        address: u32,
        addr_len: u8,
        io_mode: IoMode,
        mode_byte: Option<u8>,
        dummy_cycles: u8,
    ) {
        self.cs_assert();

        let opcode_width = if io_mode == IoMode::Qpi { 4 } else { 1 };
        let address_width = io_mode.address_width();
        let read_width = io_mode.read_width();

        self.write_byte_width(opcode, opcode_width);
        self.write_address(address, addr_len, address_width);

        if let Some(mode) = mode_byte {
            self.write_byte_width(mode, address_width);
        }

        self.dummy_clocks(read_width, dummy_cycles);
    }

    /// Read one block of data. CS must already be asserted via `start_read()`.
    pub async fn read_block(&mut self, buf: &mut [u8], io_mode: IoMode) {
        let width = io_mode.read_width();
        for byte in buf.iter_mut() {
            *byte = self.read_byte_width(width);
        }
    }

    /// Finish a multi-block transfer (deassert CS).
    pub fn end_transfer(&mut self) {
        self.cs_deassert();
    }

    /// Write one page to flash using standard 1-1-1 page program.
    pub async fn write_page(&mut self, opcode: u8, address: u32, addr_len: u8, data: &[u8]) {
        // ---- Write Enable ----
        self.cs_assert();
        self.write_byte_width(config::SPI_CMD_WRITE_ENABLE, 1);
        self.cs_deassert();

        // ---- Page Program ----
        self.cs_assert();
        self.write_byte_width(opcode, 1);
        self.write_address(address, addr_len, 1);
        for &byte in data {
            self.write_byte_width(byte, 1);
        }
        self.cs_deassert();

        // ---- Wait for completion ----
        self.poll_wip().await;
    }

    /// Poll the flash status register until the WIP (Write In Progress) bit clears.
    async fn poll_wip(&mut self) {
        loop {
            self.cs_assert();
            self.write_byte_width(config::SPI_CMD_READ_STATUS, 1);
            let status = self.read_byte_width(1);
            self.cs_deassert();

            if status & config::SPI_STATUS_WIP == 0 {
                break;
            }

            embassy_time::Timer::after_micros(50).await;
        }
    }
}
