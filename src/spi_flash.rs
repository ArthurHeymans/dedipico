/// SPI flash operations over one all-PIO flash bus.
///
/// PL022 is intentionally not used. All phases, including classic 1-1-1 SPI,
/// run through PIO so the same engine can switch between 1-1-1, 1-1-2,
/// 1-2-2, 1-1-4 and 1-4-4. Bulk reads use two DMA channels: one drains the
/// PIO RX FIFO and one feeds one token per byte into the TX FIFO so the state
/// machine clocks exactly the requested number of bytes and then stalls.
use embassy_futures::join::join;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, LoadedProgram, Pin, Pio, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::Peri;
use fixed::types::U24F8;

use crate::config;
use crate::protocol::IoMode;

type Pio0Common<'d> = Common<'d, PIO0>;
type Pio0Sm0<'d> = StateMachine<'d, PIO0, 0>;
type Pio0Pin<'d> = Pin<'d, PIO0>;
type Pio0Program<'d> = LoadedProgram<'d, PIO0>;

struct FlashPioPrograms<'d> {
    tx1: Pio0Program<'d>,
    tx2: Pio0Program<'d>,
    tx4: Pio0Program<'d>,
    rx1: Pio0Program<'d>,
    rx2: Pio0Program<'d>,
    rx4: Pio0Program<'d>,
}

pub struct SpiFlash<'d> {
    _common: Pio0Common<'d>,
    sm: Pio0Sm0<'d>,
    dma_rx: Peri<'d, AnyChannel>,
    dma_tx: Peri<'d, AnyChannel>,
    programs: FlashPioPrograms<'d>,
    io0: Pio0Pin<'d>,
    io1: Pio0Pin<'d>,
    io2: Pio0Pin<'d>,
    io3: Pio0Pin<'d>,
    sck: Pio0Pin<'d>,
    cs: Output<'d>,
    clock_divider: U24F8,
    active_rx_width: u8,
}

impl<'d> SpiFlash<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mut pio: Pio<'d, PIO0>,
        dma_rx: Peri<'d, impl Channel>,
        dma_tx: Peri<'d, impl Channel>,
        io0: Peri<'d, impl PioPin + 'd>,
        io1: Peri<'d, impl PioPin + 'd>,
        io2: Peri<'d, impl PioPin + 'd>,
        io3: Peri<'d, impl PioPin + 'd>,
        sck: Peri<'d, impl PioPin + 'd>,
        cs: Output<'d>,
    ) -> Self {
        let mut io0 = pio.common.make_pio_pin(io0);
        let mut io1 = pio.common.make_pio_pin(io1);
        let mut io2 = pio.common.make_pio_pin(io2);
        let mut io3 = pio.common.make_pio_pin(io3);
        let mut sck = pio.common.make_pio_pin(sck);

        io0.set_pull(Pull::None);
        io1.set_pull(Pull::None);
        io2.set_pull(Pull::Up);
        io3.set_pull(Pull::Up);
        sck.set_pull(Pull::None);

        // Input synchronizers add delay at high clock rates. These flash signals
        // are synchronous to our generated SCK, so bypass them for sampling.
        io0.set_input_sync_bypass(true);
        io1.set_input_sync_bypass(true);
        io2.set_input_sync_bypass(true);
        io3.set_input_sync_bypass(true);

        let programs = FlashPioPrograms {
            tx1: pio.common.load_program(&assemble_tx_program(1)),
            tx2: pio.common.load_program(&assemble_tx_program(2)),
            tx4: pio.common.load_program(&assemble_tx_program(4)),
            rx1: pio.common.load_program(&assemble_rx_program(1)),
            rx2: pio.common.load_program(&assemble_rx_program(2)),
            rx4: pio.common.load_program(&assemble_rx_program(4)),
        };

        let mut this = Self {
            _common: pio.common,
            sm: pio.sm0,
            dma_rx: dma_rx.into(),
            dma_tx: dma_tx.into(),
            programs,
            io0,
            io1,
            io2,
            io3,
            sck,
            cs,
            clock_divider: U24F8::from_num(2),
            active_rx_width: 1,
        };

        this.cs.set_high();
        this.set_frequency(config::DEFAULT_SPI_FREQ_HZ);
        this.idle_io();
        this
    }

    // =========================================================================
    // Runtime PIO clock configuration
    // =========================================================================

    /// Change the generated SCK frequency. PIO programs use two instructions
    /// per SCK cycle: one with SCK low and one with SCK high.
    pub fn set_frequency(&mut self, freq_hz: u32) {
        let target = freq_hz.clamp(1, clk_sys_freq() / 2);
        let clock_freq = U24F8::from_num(clk_sys_freq());
        let pio_bit_freq = U24F8::from_num(target.saturating_mul(2));
        self.clock_divider = clock_freq / pio_bit_freq;
    }

    // =========================================================================
    // CS and idle pin control
    // =========================================================================

    #[inline]
    pub fn cs_assert(&mut self) {
        self.cs.set_low();
    }

    #[inline]
    pub fn cs_deassert(&mut self) {
        self.sm.set_enable(false);
        self.cs.set_high();
        self.idle_io();
    }

    fn idle_io(&mut self) {
        self.sm.set_enable(false);
        self.sm.clear_fifos();
        self.sm.set_pins(Level::Low, &[&self.sck, &self.io0]);
        self.sm.set_pins(Level::High, &[&self.io2, &self.io3]);
        self.sm.set_pin_dirs(
            Direction::Out,
            &[&self.sck, &self.io0, &self.io2, &self.io3],
        );
        self.sm.set_pin_dirs(Direction::In, &[&self.io1]);
    }

    fn configure_tx(&mut self, width: u8) {
        self.sm.set_enable(false);
        self.sm.clear_fifos();

        let mut cfg = Config::default();
        cfg.clock_divider = self.clock_divider;
        cfg.fifo_join = FifoJoin::Duplex;
        cfg.shift_out = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: ShiftDirection::Left,
        };

        let program = match width {
            1 => &self.programs.tx1,
            2 => &self.programs.tx2,
            _ => &self.programs.tx4,
        };
        cfg.use_program(program, &[&self.sck]);
        match width {
            1 => cfg.set_out_pins(&[&self.io0]),
            2 => cfg.set_out_pins(&[&self.io0, &self.io1]),
            _ => cfg.set_out_pins(&[&self.io0, &self.io1, &self.io2, &self.io3]),
        }

        self.sm.set_config(&cfg);
        self.sm.set_pin_dirs(Direction::Out, &[&self.sck]);
        self.sm.set_pins(Level::Low, &[&self.sck]);

        match width {
            1 => {
                self.sm
                    .set_pin_dirs(Direction::Out, &[&self.io0, &self.io2, &self.io3]);
                self.sm.set_pin_dirs(Direction::In, &[&self.io1]);
                self.sm.set_pins(Level::High, &[&self.io2, &self.io3]);
            }
            2 => {
                self.sm.set_pin_dirs(
                    Direction::Out,
                    &[&self.io0, &self.io1, &self.io2, &self.io3],
                );
                self.sm.set_pins(Level::High, &[&self.io2, &self.io3]);
            }
            _ => self.sm.set_pin_dirs(
                Direction::Out,
                &[&self.io0, &self.io1, &self.io2, &self.io3],
            ),
        }

        self.sm.restart();
        self.sm.clkdiv_restart();
        self.clear_tx_stall();
        self.sm.set_enable(true);
        self.wait_tx_stalled();
    }

    fn configure_rx(&mut self, width: u8) {
        self.sm.set_enable(false);
        self.sm.clear_fifos();

        let mut cfg = Config::default();
        cfg.clock_divider = self.clock_divider;
        cfg.fifo_join = FifoJoin::Duplex;
        cfg.shift_in = ShiftConfig {
            auto_fill: false,
            threshold: 8,
            direction: ShiftDirection::Left,
        };

        let program = match width {
            1 => &self.programs.rx1,
            2 => &self.programs.rx2,
            _ => &self.programs.rx4,
        };
        cfg.use_program(program, &[&self.sck]);
        match width {
            1 => cfg.set_in_pins(&[&self.io1]),
            2 => cfg.set_in_pins(&[&self.io0, &self.io1]),
            _ => cfg.set_in_pins(&[&self.io0, &self.io1, &self.io2, &self.io3]),
        }

        self.sm.set_config(&cfg);
        self.sm.set_pin_dirs(Direction::Out, &[&self.sck]);
        self.sm.set_pins(Level::Low, &[&self.sck]);

        match width {
            1 => {
                self.sm.set_pins(Level::Low, &[&self.io0]);
                self.sm.set_pins(Level::High, &[&self.io2, &self.io3]);
                self.sm
                    .set_pin_dirs(Direction::Out, &[&self.io0, &self.io2, &self.io3]);
                self.sm.set_pin_dirs(Direction::In, &[&self.io1]);
            }
            2 => {
                self.sm.set_pins(Level::High, &[&self.io2, &self.io3]);
                self.sm.set_pin_dirs(Direction::In, &[&self.io0, &self.io1]);
                self.sm
                    .set_pin_dirs(Direction::Out, &[&self.io2, &self.io3]);
            }
            _ => self
                .sm
                .set_pin_dirs(Direction::In, &[&self.io0, &self.io1, &self.io2, &self.io3]),
        }

        self.active_rx_width = width;
        self.sm.restart();
        self.sm.clkdiv_restart();
        self.clear_tx_stall();
        self.sm.set_enable(true);
        self.wait_tx_stalled();
    }

    fn clear_tx_stall(&mut self) {
        let _ = self.sm.tx().stalled();
    }

    fn wait_tx_stalled(&mut self) {
        while !self.sm.tx().stalled() {
            cortex_m::asm::nop();
        }
    }

    fn write_byte_configured(&mut self, value: u8) {
        self.clear_tx_stall();
        self.sm.tx().push((value as u32) << 24);
        self.wait_tx_stalled();
    }

    fn write_byte_width(&mut self, value: u8, width: u8) {
        self.configure_tx(width);
        self.write_byte_configured(value);
    }

    fn write_bytes_width(&mut self, data: &[u8], width: u8) {
        self.configure_tx(width);
        for &byte in data {
            self.write_byte_configured(byte);
        }
    }

    fn read_byte_configured(&mut self) -> u8 {
        self.sm.tx().push(0);
        while self.sm.rx().empty() {
            cortex_m::asm::nop();
        }
        self.sm.rx().pull() as u8
    }

    fn read_byte_width(&mut self, width: u8) -> u8 {
        self.configure_rx(width);
        self.read_byte_configured()
    }

    fn dummy_clocks(&mut self, width: u8, cycles: u8) {
        if cycles == 0 {
            return;
        }

        let cycles_per_byte = 8 / width;
        let bytes = cycles.div_ceil(cycles_per_byte);
        self.configure_tx(width);
        for _ in 0..bytes {
            self.write_byte_configured(0);
        }
    }

    fn write_address(&mut self, address: u32, addr_len: u8, width: u8) {
        self.configure_tx(width);
        if addr_len == 4 {
            self.write_byte_configured((address >> 24) as u8);
        }
        self.write_byte_configured((address >> 16) as u8);
        self.write_byte_configured((address >> 8) as u8);
        self.write_byte_configured(address as u8);
    }

    // =========================================================================
    // Blocking operations (called from USB handler context via critical_section)
    // =========================================================================

    /// Perform a standard 1-1-1 transceive: write command bytes, then read
    /// response bytes while CS remains asserted.
    pub fn transceive_blocking(&mut self, write_data: &[u8], read_buf: &mut [u8]) {
        self.cs_assert();
        self.write_bytes_width(write_data, 1);
        if !read_buf.is_empty() {
            self.configure_rx(1);
            for byte in read_buf.iter_mut() {
                *byte = self.read_byte_configured();
            }
        }
        self.cs_deassert();
    }

    /// Write-only standard 1-1-1 transaction.
    pub fn write_only_blocking(&mut self, write_data: &[u8]) {
        self.cs_assert();
        self.write_bytes_width(write_data, 1);
        self.cs_deassert();
    }

    // =========================================================================
    // Async operations (called from bulk worker task)
    // =========================================================================

    /// Begin a read sequence. Opcode is 1-bit unless QPI is requested.
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
        self.configure_rx(read_width);
    }

    /// DMA-read one block of data. CS must already be asserted via `start_read()`.
    pub async fn read_block(&mut self, buf: &mut [u8], io_mode: IoMode) {
        let width = io_mode.read_width();
        if self.active_rx_width != width {
            self.configure_rx(width);
        }

        let len = buf.len();
        let (rx, tx) = self.sm.rx_tx();
        let rx_transfer = rx.dma_pull(self.dma_rx.reborrow(), buf, false);
        let tx_transfer = tx.dma_push_repeated::<_, u32>(self.dma_tx.reborrow(), len);
        join(rx_transfer, tx_transfer).await;
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
        self.write_bytes_width(data, 1);
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

fn assemble_tx_program(width: u8) -> pio::Program<32> {
    let groups_per_byte = 8 / width;
    let side_set = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::<32>::new_with_side_set(side_set);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut bitloop = a.label();

    a.bind(&mut wrap_target);
    a.pull_with_side_set(false, true, 0);
    a.set_with_side_set(pio::SetDestination::X, groups_per_byte - 1, 0);
    a.bind(&mut bitloop);
    a.out_with_side_set(pio::OutDestination::PINS, width, 0);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut bitloop, 1);
    a.bind(&mut wrap_source);

    a.assemble_with_wrap(wrap_source, wrap_target)
}

fn assemble_rx_program(width: u8) -> pio::Program<32> {
    let groups_per_byte = 8 / width;
    let side_set = pio::SideSet::new(false, 1, false);
    let mut a = pio::Assembler::<32>::new_with_side_set(side_set);
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut bitloop = a.label();

    a.bind(&mut wrap_target);
    a.pull_with_side_set(false, true, 0);
    a.mov_with_side_set(
        pio::MovDestination::ISR,
        pio::MovOperation::None,
        pio::MovSource::NULL,
        0,
    );
    a.set_with_side_set(pio::SetDestination::X, groups_per_byte - 1, 0);
    a.bind(&mut bitloop);
    a.r#in_with_side_set(pio::InSource::PINS, width, 1);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut bitloop, 0);
    a.push_with_side_set(false, true, 0);
    a.bind(&mut wrap_source);

    a.assemble_with_wrap(wrap_source, wrap_target)
}
