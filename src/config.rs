// Device identity and hardware constants.

// =============================================================================
// USB descriptors
// =============================================================================

pub const USB_VID: u16 = 0x0483;
pub const USB_PID: u16 = 0xDADA;

// =============================================================================
// SPI flash command opcodes (used internally for page program / status poll)
// =============================================================================

pub const SPI_CMD_WRITE_ENABLE: u8 = 0x06;
pub const SPI_CMD_READ_STATUS: u8 = 0x05;
pub const SPI_STATUS_WIP: u8 = 0x01;

// =============================================================================
// LED bits (active-high on our hardware)
// =============================================================================

pub const LED_PASS: u8 = 0x01;
pub const LED_BUSY: u8 = 0x02;
pub const LED_ERROR: u8 = 0x04;

// =============================================================================
// Bulk transfer parameters
// =============================================================================

/// Each bulk USB transfer is 512 bytes.
pub const BULK_BLOCK_SIZE: usize = 512;

/// Each page program writes 256 bytes of real data.
pub const PAGE_SIZE: usize = 256;

/// USB max packet size for Full Speed bulk endpoints.
pub const USB_MAX_PACKET_SIZE: u16 = 64;

/// Default SPI frequency at power-on (Hz).
pub const DEFAULT_SPI_FREQ_HZ: u32 = 24_000_000;

/// Maximum accepted SPI frequency for the all-PIO flash engine.
pub const MAX_SPI_FREQ_HZ: u32 = 24_000_000;
