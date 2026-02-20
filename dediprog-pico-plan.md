# Dediprog Protocol Reimplementation Plan: Raspberry Pi Pico + Embassy Rust

## 1. Architecture Overview

The Dediprog is a **USB-to-SPI bridge**. The host PC (running flashprog) sends USB control transfers and bulk transfers. The device translates these into SPI bus operations on the target flash chip. Your Pico firmware must emulate this USB protocol so flashprog thinks it's talking to a real Dediprog.

```
[Host PC: flashprog] --USB--> [RP2040 Pico] --SPI--> [SPI Flash Chip]
```

## 2. USB Speed and Bulk Transfer Size

The real Dediprog is a USB 2.0 **High Speed** device (480 Mbit/s) with 512-byte bulk endpoints. The RP2040 only has a USB 1.1 **Full Speed** controller (12 Mbit/s) with a 64-byte max bulk packet size. This is **not a problem** — it works transparently.

The 64-byte limit is the USB **packet** size, not the **transfer** size. These are different layers. When flashprog calls `libusb_bulk_transfer(handle, ep, buf, 512, &transferred, timeout)`, the host controller automatically fragments/reassembles based on the endpoint's `wMaxPacketSize`:

| Device | Speed | Max Packet | 512-byte transfer becomes |
|---|---|---|---|
| Real Dediprog | High Speed (480 Mbit/s) | 512 bytes | 1 USB packet |
| RP2040 Pico | Full Speed (12 Mbit/s) | 64 bytes | 8 USB packets |

Both cases return `transferred = 512` to libusb. The packetization is invisible to flashprog.

**Short packet caveat:** The async read path uses `LIBUSB_TRANSFER_SHORT_NOT_OK` (`dediprog.c:623`), meaning the transfer must deliver exactly 512 bytes. The Pico firmware must not send a short packet (< 64 bytes) in the middle of a 512-byte logical transfer — all 8 full 64-byte packets must be sent. Embassy-usb handles this correctly as long as you write the full 512 bytes to the bulk endpoint before yielding.

**Throughput impact:** Full Speed tops out at ~12 Mbit/s vs 480 Mbit/s for High Speed. For a 16 MiB flash, rough read times:

- Real Dediprog (HS): ~0.3s
- Pico (FS): ~11s

Functionally correct, just slower. No code changes needed on the flashprog side.

## 3. USB Device Descriptor Setup

The Pico must present itself as:

| Field | Value |
|---|---|
| VID | `0x0483` |
| PID | `0xDADA` |
| Configuration | 1 |
| Interface | 0 |
| Bulk IN endpoint | `0x82` (EP2 IN) |
| Bulk OUT endpoint | `0x01` or `0x02` (depends on emulated device, see below) |

Use `embassy-usb` with a custom vendor-class device. You need:
- **Control endpoint (EP0)**: handles all `CMD_*` vendor control transfers
- **Bulk IN endpoint**: streams read data back to host (512-byte chunks)
- **Bulk OUT endpoint**: receives write data from host (512-byte chunks)

### USB Request Types (from `dediprog.c:43-46`)

```
REQTYPE_EP_OUT  = 0x42  (OUT | VENDOR | ENDPOINT)   -- "dediprog_write"
REQTYPE_EP_IN   = 0xC2  (IN  | VENDOR | ENDPOINT)   -- "dediprog_read"
REQTYPE_OTHER_OUT = 0x43  (OUT | VENDOR | OTHER)     -- used in some legacy paths
REQTYPE_OTHER_IN  = 0xC3  (IN  | VENDOR | OTHER)     -- used for legacy set_voltage/read_id
```

All protocol commands use `libusb_control_transfer()` with these request types. The `bRequest` field carries the `enum dediprog_cmds` value.

## 4. Device Identity Emulation

### Which device to emulate

Recommend emulating an **SF600** with firmware version **7.2.21** (or close). This gives you **PROTOCOL_V2** which is well-supported and avoids the complexity of V3's configurable read mode while supporting 4-byte addressing and multi-I/O. Alternatively, emulate **SF600PG2** or **SF700** for PROTOCOL_V3 if you want full configurability.

Decision point based on the protocol selection logic (`dediprog.c:191-214`):

| Emulated Device | Firmware | Protocol | Notes |
|---|---|---|---|
| SF100 | >= 5.5.0 | V2 | Simpler, no standalone mode |
| SF600 | 6.9.0-7.2.21 | V2 | Has standalone mode disable |
| SF600 | > 7.2.21 | V3 | Most flexible read config |
| SF600PG2 | any | V3 | Always V3, dual I/O default |
| SF700 | any | V3 | Always V3 |

### CMD_READ_PROG_INFO (0x08) — Device String

When flashprog sends a control IN transfer with `bRequest = 0x08`, respond with an ASCII string like:

```
SF600 V:7.2.21 S6B000001
```

Format: `SF<model> V:<major>.<minor>.<patch> <serial>` — up to 32 bytes.

The parsing logic is at `dediprog.c:920-949` and `951-979`. The firmware version is extracted by `sscanf(buf, "SF%*s V:%u.%u.%u ", ...)`. The device type is determined by matching the string prefix (`SF100`, `SF200`, `SF600PG2`, `SF600`, `SF700`).

### CMD_READ_EEPROM (0x05) / Legacy ID Read (0x07)

The `dediprog_read_id()` function (`dediprog.c:987-1036`) reads a serial number. For SF600, it sends `CMD_READ_EEPROM` (control IN, `bRequest=0x05`) and expects 16 bytes back. Return a fixed 3-byte ID (e.g., `{0x01, 0x00, 0x00}`) padded to 16 bytes. The ID is decoded as `buf[0]<<16 | buf[1]<<8 | buf[2]`.

## 5. Complete Command Map

Here is every command your firmware must handle, with the USB transfer parameters:

### 5.1 Initialization Commands

#### CMD_SET_TARGET (0x04)
- Direction: OUT (control, `REQTYPE_EP_OUT`)
- `wValue`: target enum (0 = APP_FLASH_1, 2 = APP_FLASH_2)
- `wIndex`: 0
- Data: none
- Action: Store the selected target. For a single-flash setup, just ACK.

#### CMD_SET_VCC (0x09) — Set SPI Voltage
- Direction: OUT
- `wValue`: voltage selector (`0x00`=off, `0x12`=1.8V, `0x11`=2.5V, `0x10`=3.5V)
- `wIndex`: 0
- Data: none
- Action: On the Pico, you can control a voltage regulator via GPIO, or just ACK if using a fixed 3.3V level shifter. **Must respond with success (zero-length ACK).**

#### CMD_SET_SPI_CLK (0x61) — Set SPI Clock Speed
- Direction: OUT
- `wValue`: speed code (`0x0`=24MHz, `0x2`=12MHz, `0x1`=8MHz, `0x3`=3MHz, `0x4`=2.18MHz, `0x5`=1.5MHz, `0x6`=750kHz, `0x7`=375kHz)
- `wIndex`: 0
- Data: none
- Action: Reconfigure the RP2040 SPI peripheral clock divider. Map each code to the closest achievable frequency.

#### CMD_SET_IO_LED (0x07) — Set LEDs / GPIO
- Direction: OUT
- Protocol V2+: `wValue` = `(leds ^ 7) << 8`, `wIndex` = 0
- Protocol V1: `wValue` = 0x9, `wIndex` = `target_leds ^ 7`
- Data: none
- Action: Drive 3 GPIOs for Pass/Busy/Error LEDs. Bits: `bit0=PASS, bit1=BUSY, bit2=ERROR`. The XOR with 7 inverts them (0=on for original hardware).

#### CMD_SET_STANDALONE (0x0A)
- Direction: OUT
- `wValue`: 0 = enter standalone, 1 = leave standalone
- `wIndex`: 0
- Data: none
- Action: ACK and ignore (only relevant for SF600 with SD card).

#### CMD_IO_MODE (0x15) — Set I/O Mode
- Direction: OUT
- `wValue`: I/O mode (0=single, 1=dual-out, 2=dual-io, 3=quad-out, 4=quad-io, 5=QPI)
- `wIndex`: 0
- Data: none
- Action: Store the mode. When using the RP2040's PIO-based QSPI, reconfigure the state machine. With hardware SPI (single only), modes > 0 would need PIO.

### 5.2 SPI Transceive (Generic Command Passthrough)

#### CMD_TRANSCEIVE (0x01) — Send/Receive SPI Command

This is the **most critical** command. It's used for all generic SPI operations (JEDEC ID read, status register read, write enable, erase commands, etc.).

**Phase 1 — Write phase (host sends command bytes):**
- Direction: OUT (control)
- `bRequest`: 0x01
- Protocol V2+: `wValue` = `readcnt ? 0x1 : 0x0`, `wIndex` = 0
- Protocol V1: `wValue` = 0, `wIndex` = `readcnt ? 0x1 : 0x0`
- Data: the SPI command bytes to send (opcode + address + write data), up to 16 bytes

**Phase 2 — Read phase (if readcnt > 0):**
- Direction: IN (control)
- `bRequest`: 0x01
- `wValue`: 0, `wIndex`: 0
- Data: read back `readcnt` bytes (up to 16)

**Firmware action:**
1. Assert CS (drive low)
2. Clock out all write bytes on MOSI
3. While clocking out (or after), clock in read bytes from MISO
4. Deassert CS (drive high)
5. Buffer the read bytes for the subsequent control IN transfer

Important: The write and read happen in a single SPI transaction (CS stays low). The write bytes include the opcode and address, and the read bytes are what the flash returns after that.

### 5.3 Bulk Read

#### CMD_READ (0x20) — Initiate Bulk SPI Read

**Phase 1 — Command setup (control OUT):**
- `bRequest`: 0x20
- `wValue` / `wIndex`: depend on protocol version (see below)
- Data: command packet (5-14 bytes, see packet format below)

**Phase 2 — Data transfer (bulk IN):**
- The host issues N bulk IN transfers of 512 bytes each
- You must send back the flash data in 512-byte chunks

**Command packet format (data payload of the control transfer):**

Bytes 0-4 are common across all protocol versions:
```
[0]: block_count & 0xFF
[1]: (block_count >> 8) & 0xFF
[2]: 0x00 (reserved)
[3]: read_mode (enum dediprog_readmode)
[4]: opcode (SPI read opcode, or 0)
```

**Protocol V1** (`prepare_rw_cmd_v1`, `dediprog.c:438-461`):
- Packet size: 5 bytes
- `wValue` = `start_address & 0xFFFF`
- `wIndex` = `(start_address >> 16) & 0xFF`
- Address is only 3 bytes (EAR used for byte 4)

**Protocol V2** (`prepare_rw_cmd_v2`, `dediprog.c:463-504`):
- Packet size: 10 bytes
- `wValue` = 0, `wIndex` = 0
- Bytes 5: reserved
- Bytes 6-9: start address (little-endian, 32-bit)
- `cmd_buf[3]` may be modified to `READ_MODE_FAST` or `READ_MODE_4B_ADDR_FAST_0x0C`
- `cmd_buf[4]` set to actual SPI opcode

**Protocol V3** (`prepare_rw_cmd_v3`, `dediprog.c:506-555`):
- Packet size: 12 bytes (read) / 14 bytes (write)
- `wValue` = 0, `wIndex` = 0
- Bytes 5: reserved
- Bytes 6-9: start address (little-endian, 32-bit)
- `cmd_buf[3]` = `READ_MODE_CONFIGURABLE` (9)
- `cmd_buf[4]` = actual SPI opcode
- `cmd_buf[10]` = address length (3 or 4)
- `cmd_buf[11]` = dummy half-cycles (`spi_dummy_cycles(op) / 2`)

**Firmware action for bulk read:**
1. Parse the command packet to extract: start address, block count, read mode, opcode, dummy cycles
2. Assert CS
3. Send the SPI read opcode
4. Send address bytes (3 or 4)
5. Send dummy clocks (for fast read modes)
6. Read `block_count * 512` bytes from the flash
7. Deassert CS
8. Feed the data back via bulk IN endpoint in 512-byte chunks

**Important timing note:** The host uses asynchronous bulk transfers (up to 8 in flight, `dediprog.c:602-648`). Your bulk IN endpoint must be able to keep up. Use DMA on the RP2040 SPI peripheral and double-buffer the USB bulk endpoint.

### 5.4 Bulk Write

#### CMD_WRITE (0x30) — Initiate Bulk SPI Write

**Phase 1 — Command setup (control OUT):**
- `bRequest`: 0x30
- `wValue` / `wIndex`: same rules as CMD_READ per protocol version
- Data: command packet (same format as read, but with write mode in byte 3)

**Phase 2 — Data transfer (bulk OUT):**
- Host sends N bulk OUT transfers of 512 bytes each
- Each 512-byte USB packet contains 256 bytes of real data + 256 bytes of 0xFF padding
- You program the flash 256 bytes at a time (one page)

**Write mode values (byte 3 of command packet):**
```
1 = PAGE_PGM (standard 256-byte page program, opcode 0x02)
4 = 2B_AAI (SST-style word auto-address-increment)
9 = 4B_ADDR_256B_PAGE_PGM (4-byte address, opcode configurable)
11 = 4B_ADDR_256B_PAGE_PGM_0x12 (explicit 4BA page program)
```

**Firmware action for each 512-byte bulk OUT packet:**
1. Receive 512 bytes from USB bulk OUT
2. Extract the first 256 bytes (real data)
3. Send SPI Write Enable (0x06)
4. Send Page Program command: `[opcode] [addr2] [addr1] [addr0] [data...]`
5. Poll flash status register (RDSR, 0x05) until WIP bit clears
6. Advance address by 256, repeat for next packet

### 5.5 Other Commands (Optional / Stub)

| Command | Code | Action |
|---|---|---|
| `CMD_SET_VPP` | 0x03 | ACK, ignore (VPP control) |
| `CMD_SET_VOLTAGE` | 0x0B | Legacy, return `0x6F` in 1-byte IN transfer (uses `REQTYPE_OTHER_IN`) |
| `CMD_GET_BUTTON` | 0x11 | Return `0x01` (button not pressed) |
| `CMD_GET_UID` | 0x12 | Return a fixed UID |
| `CMD_SET_CS` | 0x14 | Manual CS control — assert/deassert |
| `CMD_SET_HOLD` | 0x1D | ACK, ignore (HOLD pin control) |
| `CMD_READ_FPGA_VERSION` | 0x1C | Return dummy bytes |
| `CMD_CHECK_SOCKET` | 0x62 | Return success |
| `CMD_POLL_STATUS_REG` | 0x02 | Accelerated status polling, can stub |

## 6. SPI Bus Implementation on RP2040

### Pin Mapping (suggested)

| Function | RP2040 Pin | GPIO |
|---|---|---|
| SPI SCK | GP18 | 18 |
| SPI MOSI (IO0) | GP19 | 19 |
| SPI MISO (IO1) | GP16 | 16 |
| SPI CS | GP17 | 17 (manual GPIO, not hardware CS) |
| LED Pass | GP25 | 25 (onboard LED) |
| LED Busy | GP14 | 14 |
| LED Error | GP15 | 15 |

Use **manual CS** (GPIO output) rather than hardware CS, because:
- SPI transactions can span multiple USB transfers
- CS must stay asserted during the entire bulk read/write sequence
- Some SPI commands need precise CS timing

### SPI Peripheral Configuration

```rust
// Embassy SPI setup
let mut config = spi::Config::default();
config.frequency = 12_000_000; // 12 MHz default, adjust per CMD_SET_SPI_CLK
config.phase = spi::Phase::CaptureOnFirstTransition;
config.polarity = spi::Polarity::IdleLow;
let spi = Spi::new(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, p.DMA_CH0, p.DMA_CH1, config);
```

### Multi-I/O (Dual/Quad) Support

Standard RP2040 SPI only supports single I/O (1-1-1). For dual/quad support, you have two options:

1. **PIO-based SPI**: Use the RP2040's PIO peripheral to implement dual/quad SPI. This is complex but fully flexible.
2. **Single-I/O only**: Report as SF100 with protocol V1 or set features to `SPI_MASTER_NO_4BA_MODES` and no dual/quad flags. flashprog will fall back to single-I/O reads.

**Recommendation**: Start with single I/O only. This is sufficient for all flash operations; dual/quad only affect read speed.

## 7. Embassy Rust Project Structure

### Cargo.toml Dependencies

```toml
[dependencies]
embassy-executor = { version = "0.7", features = ["arch-cortex-m", "executor-thread"] }
embassy-rp = { version = "0.5", features = ["rp2040"] }
embassy-usb = "0.4"
embassy-sync = "0.6"
embassy-time = "0.4"
defmt = "0.3"
defmt-rtt = "0.4"
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-probe = { version = "0.3", features = ["print-defmt"] }
static_cell = "2"
```

### Module Layout

```
src/
├── main.rs              # Entry point, embassy executor, USB + SPI init
├── usb_handler.rs       # USB control request handler (CMD_* dispatch)
├── protocol.rs          # Command parsing, packet builders, enums
├── spi_flash.rs         # SPI flash operations (transceive, bulk read/write)
├── leds.rs              # LED control
└── config.rs            # Device identity, speed tables, constants
```

### Core State Machine

```rust
struct DediprogState {
    // Device identity
    device_type: DeviceType,      // SF600
    firmware_version: (u8,u8,u8), // 7,2,21
    protocol: Protocol,           // V2

    // Current configuration
    spi_speed: SpiSpeed,
    voltage_mv: u16,
    io_mode: IoMode,
    target: FlashTarget,

    // Bulk transfer state
    bulk_op: Option<BulkOperation>,
}

enum BulkOperation {
    Read {
        address: u32,
        remaining_blocks: u16,
        opcode: u8,
        addr_len: u8,
        dummy_cycles: u8,
    },
    Write {
        address: u32,
        remaining_blocks: u16,
        opcode: u8,
        page_size: u16,
    },
}
```

### USB Control Handler Skeleton

```rust
impl Handler for DediprogHandler {
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if req.request_type != RequestType::Vendor { return None; }

        match req.request {
            0x01 => self.cmd_transceive_out(req, data),    // CMD_TRANSCEIVE write phase
            0x04 => self.cmd_set_target(req),               // CMD_SET_TARGET
            0x07 => self.cmd_set_io_led(req),               // CMD_SET_IO_LED
            0x09 => self.cmd_set_vcc(req),                  // CMD_SET_VCC
            0x0A => self.cmd_set_standalone(req),            // CMD_SET_STANDALONE
            0x15 => self.cmd_io_mode(req),                   // CMD_IO_MODE
            0x20 => self.cmd_read_setup(req, data),          // CMD_READ (setup)
            0x30 => self.cmd_write_setup(req, data),         // CMD_WRITE (setup)
            0x61 => self.cmd_set_spi_clk(req),              // CMD_SET_SPI_CLK
            _ => Some(OutResponse::Accepted),                // ACK unknown
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type != RequestType::Vendor { return None; }

        match req.request {
            0x01 => self.cmd_transceive_in(req, buf),       // CMD_TRANSCEIVE read phase
            0x05 => self.cmd_read_eeprom(req, buf),          // CMD_READ_EEPROM
            0x08 => self.cmd_read_prog_info(req, buf),       // CMD_READ_PROG_INFO
            0x0B => self.cmd_set_voltage_legacy(req, buf),   // CMD_SET_VOLTAGE (legacy)
            _ => None,
        }
    }
}
```

## 7. Implementation Order (Step by Step)

### Phase 1: USB Enumeration
1. Set up embassy-rp with USB
2. Configure VID/PID `0x0483:0xDADA`
3. Implement `CMD_READ_PROG_INFO` to return device string
4. Implement `CMD_READ_EEPROM` to return serial ID
5. **Test**: flashprog should detect the device and print its identity

### Phase 2: Basic SPI Passthrough
6. Initialize SPI peripheral
7. Implement `CMD_TRANSCEIVE` (both OUT and IN phases)
8. Implement `CMD_SET_TARGET`, `CMD_SET_VCC`, `CMD_SET_SPI_CLK`, `CMD_SET_IO_LED`
9. **Test**: `flashprog -p dediprog --flash-name` should read the JEDEC ID

### Phase 3: Bulk Read
10. Implement `CMD_READ` command packet parsing
11. Implement bulk IN data streaming (512-byte chunks, DMA)
12. Handle the async transfer pattern (up to 8 in flight)
13. **Test**: `flashprog -p dediprog -r dump.bin` should read the flash

### Phase 4: Bulk Write
14. Implement `CMD_WRITE` command packet parsing
15. Implement bulk OUT data reception
16. Implement page program with write-enable and status polling
17. **Test**: `flashprog -p dediprog -w firmware.bin` should write the flash

### Phase 5: Polish
18. Add proper SPI clock speed switching
19. Add LED indicators
20. Handle edge cases (unaligned reads, partial pages)
21. Handle `CMD_SET_STANDALONE` (ACK)
22. Test with multiple flash chips

## 8. Critical Protocol Details / Gotchas

1. **512-byte chunk alignment**: Bulk reads MUST use exactly 512-byte USB transfers. The host will reject anything else (`dediprog.c:567-568`).

2. **Write padding**: Each 512-byte bulk OUT packet contains only 256 bytes of real data. The rest is 0xFF. See `dediprog.c:758-760`.

3. **Transceive limits**: `max_data_read = 16`, `max_data_write = 11` (16-5 for cmd/addr). The control transfer payload is small.

4. **CMD_SET_VOLTAGE (0x0B) fallback**: flashprog tries reading the device string first. If that fails, it sends `CMD_SET_VOLTAGE` (using `REQTYPE_OTHER_IN`, not `REQTYPE_EP_IN`) and retries (`dediprog.c:1230-1234`). If your device string works on first try, this path is never taken.

5. **Endpoint direction**: The bulk IN endpoint is always `0x82`. The bulk OUT endpoint is `0x02` for SF100/SF200, `0x01` for SF600+ (`dediprog.c:1237-1241`). If you emulate SF600, use EP1 OUT.

6. **Protocol V2 `wValue`/`wIndex` for transceive**: The read-required flag moved from `wIndex` (V1) to `wValue` (V2+). Get this wrong and reads will fail silently.

7. **Standalone mode**: Only checked for SF600. If you emulate SF600, you must ACK the `CMD_SET_STANDALONE` with `wValue=1` (leave standalone). If you emulate SF100, this is never sent.

8. **Firmware version validation** (`dediprog.c:964-969`): The major version is checked against expected ranges. For SF600, major versions 2-7 are accepted. For SF700, only major version 4.

## 9. Testing Strategy

```bash
# 1. Detection
flashprog -p dediprog

# 2. Flash identification
flashprog -p dediprog --flash-name

# 3. Read
flashprog -p dediprog -r /tmp/read.bin

# 4. Erase + Write + Verify
flashprog -p dediprog -w firmware.bin

# 5. With specific parameters
flashprog -p dediprog:spispeed=12M,voltage=3.5V -r /tmp/read.bin
```

Use `flashprog -VVV` for maximum verbosity to see every USB transfer, which you can correlate with your firmware's debug output via `defmt` + RTT.
