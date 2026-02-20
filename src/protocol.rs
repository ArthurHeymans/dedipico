// Dediprog protocol command codes and types.

// =============================================================================
// Command codes (bRequest values for vendor control transfers)
// =============================================================================

pub const CMD_TRANSCEIVE: u8 = 0x01;
#[allow(dead_code)]
pub const CMD_POLL_STATUS_REG: u8 = 0x02;
pub const CMD_SET_VPP: u8 = 0x03;
pub const CMD_SET_TARGET: u8 = 0x04;
pub const CMD_READ_EEPROM: u8 = 0x05;
pub const CMD_SET_IO_LED: u8 = 0x07;
pub const CMD_READ_PROG_INFO: u8 = 0x08;
pub const CMD_SET_VCC: u8 = 0x09;
pub const CMD_SET_STANDALONE: u8 = 0x0A;
pub const CMD_SET_VOLTAGE: u8 = 0x0B;
pub const CMD_GET_BUTTON: u8 = 0x11;
pub const CMD_GET_UID: u8 = 0x12;
pub const CMD_SET_CS: u8 = 0x14;
pub const CMD_IO_MODE: u8 = 0x15;
pub const CMD_READ_FPGA_VERSION: u8 = 0x1C;
pub const CMD_SET_HOLD: u8 = 0x1D;
pub const CMD_READ: u8 = 0x20;
pub const CMD_WRITE: u8 = 0x30;
pub const CMD_SET_SPI_CLK: u8 = 0x61;
pub const CMD_CHECK_SOCKET: u8 = 0x62;

// =============================================================================
// SPI speed codes (wValue for CMD_SET_SPI_CLK)
// =============================================================================

#[derive(Clone, Copy, Debug, defmt::Format)]
#[repr(u8)]
pub enum SpiSpeed {
    Mhz24 = 0,
    Mhz8 = 1,
    Mhz12 = 2,
    Mhz3 = 3,
    Mhz2_18 = 4,
    Mhz1_5 = 5,
    Khz750 = 6,
    Khz375 = 7,
}

impl SpiSpeed {
    pub fn from_code(code: u16) -> Self {
        match code {
            0 => SpiSpeed::Mhz24,
            1 => SpiSpeed::Mhz8,
            2 => SpiSpeed::Mhz12,
            3 => SpiSpeed::Mhz3,
            4 => SpiSpeed::Mhz2_18,
            5 => SpiSpeed::Mhz1_5,
            6 => SpiSpeed::Khz750,
            7 => SpiSpeed::Khz375,
            _ => SpiSpeed::Mhz12, // default
        }
    }

    pub fn frequency_hz(self) -> u32 {
        match self {
            SpiSpeed::Mhz24 => 24_000_000,
            SpiSpeed::Mhz8 => 8_000_000,
            SpiSpeed::Mhz12 => 12_000_000,
            SpiSpeed::Mhz3 => 3_000_000,
            SpiSpeed::Mhz2_18 => 2_180_000,
            SpiSpeed::Mhz1_5 => 1_500_000,
            SpiSpeed::Khz750 => 750_000,
            SpiSpeed::Khz375 => 375_000,
        }
    }
}

// =============================================================================
// Read modes (byte 3 of read command packet)
// =============================================================================

#[derive(Clone, Copy, Debug, defmt::Format)]
#[repr(u8)]
pub enum ReadMode {
    Std = 1,
    Fast = 2,
    AtmelFast = 3,
    Addr4bFast = 4,
    Addr4bFast0x0C = 5,
    Configurable = 9,
}

impl ReadMode {
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            1 => Some(ReadMode::Std),
            2 => Some(ReadMode::Fast),
            3 => Some(ReadMode::AtmelFast),
            4 => Some(ReadMode::Addr4bFast),
            5 => Some(ReadMode::Addr4bFast0x0C),
            9 => Some(ReadMode::Configurable),
            _ => None,
        }
    }

    /// Number of dummy bytes needed after the address for this read mode.
    pub fn dummy_bytes(self) -> u8 {
        match self {
            ReadMode::Std => 0,
            ReadMode::Fast
            | ReadMode::AtmelFast
            | ReadMode::Addr4bFast
            | ReadMode::Addr4bFast0x0C => 1,
            ReadMode::Configurable => 0, // caller supplies dummy count
        }
    }

    /// Whether this mode uses 4-byte addressing.
    pub fn uses_4byte_addr(self) -> bool {
        matches!(self, ReadMode::Addr4bFast | ReadMode::Addr4bFast0x0C)
    }
}

// =============================================================================
// Write modes (byte 3 of write command packet)
// =============================================================================

#[derive(Clone, Copy, Debug, defmt::Format)]
#[repr(u8)]
pub enum WriteMode {
    PageProgram = 1,
    Aai2Byte = 4,
    Addr4bPageProgram = 9,
    Addr4bPageProgram0x12 = 11,
}

impl WriteMode {
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            1 => Some(WriteMode::PageProgram),
            4 => Some(WriteMode::Aai2Byte),
            9 => Some(WriteMode::Addr4bPageProgram),
            11 => Some(WriteMode::Addr4bPageProgram0x12),
            _ => None,
        }
    }

    pub fn uses_4byte_addr(self) -> bool {
        matches!(
            self,
            WriteMode::Addr4bPageProgram | WriteMode::Addr4bPageProgram0x12
        )
    }
}

// =============================================================================
// Bulk operation descriptors
// =============================================================================

#[derive(Clone, Debug, defmt::Format)]
pub enum BulkOperation {
    Read {
        address: u32,
        block_count: u16,
        opcode: u8,
        addr_len: u8,
        dummy_bytes: u8,
    },
    Write {
        address: u32,
        block_count: u16,
        opcode: u8,
        addr_len: u8,
    },
}

// =============================================================================
// Command packet parsing (Protocol V2)
// =============================================================================

/// Parse a V2 read/write command packet (10 bytes).
/// Returns (block_count, mode_byte, opcode, start_address).
pub fn parse_rw_cmd_v2(data: &[u8]) -> Option<(u16, u8, u8, u32)> {
    if data.len() < 10 {
        return None;
    }
    let block_count = data[0] as u16 | ((data[1] as u16) << 8);
    let mode = data[3];
    let opcode = data[4];
    let address = data[6] as u32
        | ((data[7] as u32) << 8)
        | ((data[8] as u32) << 16)
        | ((data[9] as u32) << 24);
    Some((block_count, mode, opcode, address))
}
