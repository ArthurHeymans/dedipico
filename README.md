# DediPico

Dediprog SF600 protocol emulation on a Raspberry Pi Pico (RP2040), written in
Rust with Embassy. Lets you use `flashprog` (or `flashrom`) with a $4 Pico
as a USB-to-SPI flash programmer. The flash bus is implemented entirely with
RP2040 PIO so the same engine can handle single, dual, and quad SPI phases.

```text
[Host PC: flashprog] --USB--> [Raspberry Pi Pico] --SPI--> [SPI Flash Chip]
```

## Pinout

### SPI (directly to flash chip)

| Function        | Pico Pin | GPIO |
|-----------------|----------|------|
| SPI SCK         | 4        | GP2  |
| SPI IO0 / MOSI  | 5        | GP3  |
| SPI IO1 / MISO  | 6        | GP4  |
| SPI IO2 / WP#   | 7        | GP5  |
| SPI IO3 / HOLD# | 9        | GP6  |
| SPI CS#         | 10       | GP7  |

The SPI bus uses PIO0. Keep IO0 through IO3 on consecutive GPIOs (GP3-GP6);
the PIO programs switch those pins between output and input for 1-1-1, 1-1-2,
1-2-2, 1-1-4, and 1-4-4 transactions.

### UART bridge

| Function | Pico Pin | GPIO |
|----------|----------|------|
| UART TX  | 1        | GP0  |
| UART RX  | 2        | GP1  |

### Board control GPIO

| Function          | Pico Pin | GPIO | Direction         |
|-------------------|----------|------|-------------------|
| RESET#            | 11       | GP8  | open-drain output |
| POWER_SW#         | 12       | GP9  | open-drain output |
| Board power state | 14       | GP10 | input             |
| Auxiliary state   | 15       | GP11 | input             |

RESET# and POWER_SW# only pull low or release; they never drive high. Hold
POWER_SW# low for about 5 s to request power-off on ATX-style boards.

### LEDs (active-high)

| Function  | Pico Pin     | GPIO |
|-----------|--------------|------|
| LED Pass  | 25 (onboard) | GP25 |
| LED Busy  | 19           | GP14 |
| LED Error | 20           | GP15 |

### Wiring diagram

```text
            Raspberry Pi Pico
           ┌─────────────────┐
           │               GP2├──── SCK
           │               GP3├──── IO0 / MOSI
           │               GP4├──── IO1 / MISO
           │               GP5├──── IO2 / WP#
           │               GP6├──── IO3 / HOLD#
           │               GP7├──── CS#
           │               GP8├──── RESET# (open-drain)
           │               GP9├──── POWER_SW# (open-drain)
           │              GP10├──── Board power state
           │              GP11├──── Auxiliary state
           │                  │
           │              GP25├──── Pass LED (onboard)
           │              GP14├──── Busy LED
           │              GP15├──── Error LED
           │                  │
           │              3V3 ├──── Flash VCC
           │              GND ├──── Flash GND
           │   USB            │
           └────┘  └──────────┘
                ▲
                │
           to host PC
```

The Pico supplies 3.3 V directly. No level shifter is needed for 3.3 V flash
chips. For 1.8 V parts, add an external level shifter. For quad I/O, make sure
IO2/WP# and IO3/HOLD# are wired. When idle, the firmware releases SCK,
IO0-IO3, and CS# to input/Hi-Z so an in-system mainboard SPI controller can
own the bus.

## Building

```bash
rustup target add thumbv6m-none-eabi
cargo build --release
```

## Flashing

### Install a prebuilt UF2

1. Open the latest release on GitHub.
2. Download `dedipico.uf2` from the release assets.
3. Hold **BOOTSEL** while plugging in the Pico.
4. Copy `dedipico.uf2` to the mounted `RPI-RP2` drive.

### Build and flash locally

With a debug probe (another Pico running debugprobe, a CMSIS-DAP adapter, etc.):

```bash
cargo run --release
```

Or build the UF2 yourself and copy it to the Pico's mass-storage bootloader:

```bash
cargo install elf2uf2-rs --no-default-features
elf2uf2-rs target/thumbv6m-none-eabi/release/dedipico dedipico.uf2
# hold BOOTSEL, plug in Pico, copy dedipico.uf2 to the RPI-RP2 drive
```

## Usage

```bash
# detect the device
flashprog -p dediprog

# identify the flash chip
flashprog -p dediprog --flash-name

# read
flashprog -p dediprog -r dump.bin

# write (erases first)
flashprog -p dediprog -w firmware.bin

# quad/dual read path
flashprog -p dediprog:iomode=quad -r dump.bin
flashprog -p dediprog:iomode=dual -r dump.bin

# with explicit parameters
flashprog -p dediprog:spispeed=12M,voltage=3.5V -r dump.bin

# maximum debug output
flashprog -p dediprog -VVV
```

## What it emulates

The firmware presents itself as a Dediprog **SF600** running firmware
**v7.2.22**, which selects **Protocol V3** in flashprog. It exposes USB VID:PID
`0483:DADA` as a composite device with only vendor-specific interfaces, so Linux
kernel drivers do not claim it and stock `flashprog` can still use it:

- Interface 0: DediProg SF600-compatible vendor interface
  - EP0 control — all `CMD_*` vendor requests
  - EP1 OUT (`0x01`) — bulk write data from host
  - EP2 IN (`0x82`) — bulk read data to host
- Interface 1: DediPico auxiliary vendor interface
  - EP3 OUT (`0x03`) — host to GPIO/UART
  - EP4 IN (`0x84`) — GPIO/UART to host

Supported commands: `TRANSCEIVE`, `READ`, `WRITE`, `SET_VCC`, `SET_SPI_CLK`,
`SET_TARGET`, `SET_IO_LED`, `SET_STANDALONE`, `IO_MODE`, `SET_CS`,
`READ_PROG_INFO`, `READ_EEPROM`, `SET_VOLTAGE` legacy reads, and various stubs
(`SET_VPP`, `SET_HOLD`, `GET_BUTTON`, `GET_UID`, `READ_FPGA_VERSION`,
`CHECK_SOCKET`, etc.).

The auxiliary interface uses variable-length packets of up to 64 bytes. Every
packet starts with `[type, request_id, payload_len]`. A nonzero request ID asks
the device to return a correlated response; request ID zero is reserved for
unsolicited events.

Host-to-device commands and their payloads:

- `0x01` GPIO_GET_STATE: `[]`
- `0x02` GPIO_SET_DIRECTION: `[mask, directions]`
- `0x03` GPIO_SET_OUTPUT: `[mask, values]`
- `0x04` GPIO_PULSE_LOW: `[mask, ms_lo, ms_hi]`
- `0x10` UART_SET_BAUD: `[baud_le32]`
- `0x11` UART_WRITE: UART bytes

Device-to-host packets:

- `0x80` RESPONSE: `[command, status, data...]`
- `0x81` GPIO_STATE event: `[inputs, outputs, directions, caps]`
- `0x90` UART_DATA event: batched UART bytes

GPIO command responses include the four-byte GPIO state as their response data.
GPIO bits are bit0 RESET#, bit1 POWER_SW#, bit2 board power state, and bit3
auxiliary state. For RESET#/POWER_SW#, direction=0 releases the pin; direction=1
with output=0 pulls it low.

UART input is accumulated until the 61-byte payload is full or a 1 ms deadline
from the first buffered byte expires. GPIO responses are serviced before queued
UART data. UART RX and TX use bounded queues so a stalled terminal cannot block
board management. Power/reset pulses are acknowledged immediately and released
by a timer without blocking other auxiliary work.

`dedipicoctl daemon` is the only process that claims the auxiliary USB
interface. It prints a UART pseudo-terminal and listens on a per-user Unix
socket for management commands:

```bash
# terminal 1: keep the daemon running
(cd tools/dedipicoctl && cargo run --release --target x86_64-unknown-linux-gnu -- daemon 115200)
# pty: /dev/pts/N
# socket: /run/user/1000/dedipico.sock

# terminal 2: connect a terminal program to the printed PTY
picocom /dev/pts/N

# management commands go through the daemon while UART remains active
(cd tools/dedipicoctl && cargo run --release --target x86_64-unknown-linux-gnu -- state)
(cd tools/dedipicoctl && cargo run --release --target x86_64-unknown-linux-gnu -- reset)
(cd tools/dedipicoctl && cargo run --release --target x86_64-unknown-linux-gnu -- power)
(cd tools/dedipicoctl && cargo run --release --target x86_64-unknown-linux-gnu -- poweroff)
```

Use `--socket PATH` on both daemon and client commands to override the socket
location. The daemon owns interface 1 while `flashprog` uses interface 0, so
they can remain active at the same time.

## Limitations

- **Full Speed USB only** (12 Mbit/s vs 480 Mbit/s on a real SF600). A 16 MiB
  flash read takes ~11 s instead of ~0.3 s. Functionally identical, just slower.
- **All SPI traffic uses the PIO flash engine.** `iomode=dual` and
  `iomode=quad` exercise Dediprog/flashprog's 1-1-2, 1-2-2, 1-1-4, and 1-4-4
  read paths. Classic 1-1-1 commands and page programs also use PIO rather than
  the RP2040 PL022 SPI block. Bulk reads use DMA for both the RX FIFO and the
  per-byte clock tokens.
- **SPI clock speed switching is supported by the PIO clock divider.** The bus
  defaults to 24 MHz, caps requested speeds at 24 MHz, and is reconfigured at
  runtime when flashprog sends `SET_SPI_CLK` (e.g. `spispeed=12M`).
- **No voltage switching.** The Pico's 3V3 rail is always on. `SET_VCC` is
  acknowledged but does not control power.

## Project structure

```text
src/
├── main.rs           Entry point, USB + flash-bus init, bulk/aux worker tasks
├── usb_handler.rs    embassy_usb::Handler — dispatches CMD_* control transfers
├── protocol.rs       Command codes, enums, V2/V3 packet parsing
├── spi_flash.rs      All-PIO + DMA flash ops (single writes; single/dual/quad reads)
├── aux.rs            Vendor GPIO/UART auxiliary protocol
├── leds.rs           GPIO LED driver
└── config.rs         Device identity, constants
```

## License

MIT
