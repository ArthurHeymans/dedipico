# DediPico

Dediprog SF600 protocol emulation on a Raspberry Pi Pico (RP2040), written in
Rust with Embassy. Lets you use `flashprog` (or `flashrom`) with a $4 Pico
as a USB-to-SPI flash programmer.

```
[Host PC: flashprog] --USB--> [Raspberry Pi Pico] --SPI--> [SPI Flash Chip]
```

## Pinout

### SPI (directly to flash chip)

| Function       | Pico Pin | GPIO |
|----------------|----------|------|
| SPI SCK        | 24       | GP18 |
| SPI MOSI (IO0) | 25       | GP19 |
| SPI MISO (IO1) | 21       | GP16 |
| SPI CS#        | 22       | GP17 |

### LEDs (active-high)

| Function  | Pico Pin | GPIO |
|-----------|----------|------|
| LED Pass  | 25 (onboard) | GP25 |
| LED Busy  | 19       | GP14 |
| LED Error | 20       | GP15 |

### Wiring diagram

```
            Raspberry Pi Pico
           ┌─────────────────┐
           │              GP18├──── SCK
           │              GP19├──── MOSI
           │              GP16├──── MISO
           │              GP17├──── CS#
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
chips. For 1.8 V parts, add an external level shifter.

## Building

```
rustup target add thumbv6m-none-eabi
cargo build --release
```

## Flashing

With a debug probe (another Pico running debugprobe, a CMSIS-DAP adapter, etc.):

```
cargo run --release
```

Or copy the UF2 to the Pico's mass-storage bootloader:

```
cargo install elf2uf2-rs
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

# with explicit parameters
flashprog -p dediprog:spispeed=12M,voltage=3.5V -r dump.bin

# maximum debug output
flashprog -p dediprog -VVV
```

## What it emulates

The firmware presents itself as a Dediprog **SF600** running firmware
**v7.2.21**, which selects **Protocol V2** in flashprog. It exposes
USB VID:PID `0483:DADA` with:

- EP0 control — all `CMD_*` vendor requests
- EP1 OUT (`0x01`) — bulk write data from host
- EP2 IN (`0x82`) — bulk read data to host

Supported commands: `TRANSCEIVE`, `READ`, `WRITE`, `SET_VCC`, `SET_SPI_CLK`,
`SET_TARGET`, `SET_IO_LED`, `SET_STANDALONE`, `IO_MODE`, `READ_PROG_INFO`,
`READ_EEPROM`, and various stubs (`SET_VPP`, `SET_HOLD`, `GET_BUTTON`, etc.).

## Limitations

- **Full Speed USB only** (12 Mbit/s vs 480 Mbit/s on a real SF600). A 16 MiB
  flash read takes ~11 s instead of ~0.3 s. Functionally identical, just slower.
- **Single I/O only.** Dual/Quad SPI would require PIO; the RP2040 hardware SPI
  peripheral only supports standard 1-1-1 mode.
- **SPI clock speed switching** is fully supported. The bus defaults to 30 MHz
  and is reconfigured at runtime when flashprog sends `SET_SPI_CLK` (e.g.
  `spispeed=12M`).
- **No voltage switching.** The Pico's 3V3 rail is always on. `SET_VCC` is
  acknowledged but does not control power.

## Project structure

```
src/
├── main.rs           Entry point, USB + SPI init, bulk worker task
├── usb_handler.rs    embassy_usb::Handler — dispatches CMD_* control transfers
├── protocol.rs       Command codes, enums, V2 packet parsing
├── spi_flash.rs      SPI flash ops (blocking transceive, async bulk R/W)
├── leds.rs           GPIO LED driver
└── config.rs         Device identity, constants
```

## License

MIT
