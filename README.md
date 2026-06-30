# DediPico

Dediprog SF600 protocol emulation on a Raspberry Pi Pico (RP2040), written in
Rust with Embassy. Lets you use `flashprog` (or `flashrom`) with a $4 Pico
as a USB-to-SPI flash programmer. The flash bus is implemented entirely with
RP2040 PIO so the same engine can handle single, dual, and quad SPI phases.

```
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

### LEDs (active-high)

| Function  | Pico Pin     | GPIO |
|-----------|--------------|------|
| LED Pass  | 25 (onboard) | GP25 |
| LED Busy  | 19           | GP14 |
| LED Error | 20           | GP15 |

### Wiring diagram

```
            Raspberry Pi Pico
           ┌─────────────────┐
           │               GP2├──── SCK
           │               GP3├──── IO0 / MOSI
           │               GP4├──── IO1 / MISO
           │               GP5├──── IO2 / WP#
           │               GP6├──── IO3 / HOLD#
           │               GP7├──── CS#
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
IO2/WP# and IO3/HOLD# are wired. External pull-ups on IO2 and IO3 are still a
good idea even though the firmware enables/holds them high when idle.

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
**v7.2.22**, which selects **Protocol V3** in flashprog. It exposes
USB VID:PID `0483:DADA` with:

- EP0 control — all `CMD_*` vendor requests
- EP1 OUT (`0x01`) — bulk write data from host
- EP2 IN (`0x82`) — bulk read data to host

Supported commands: `TRANSCEIVE`, `READ`, `WRITE`, `SET_VCC`, `SET_SPI_CLK`,
`SET_TARGET`, `SET_IO_LED`, `SET_STANDALONE`, `IO_MODE`, `SET_CS`,
`READ_PROG_INFO`, `READ_EEPROM`, `SET_VOLTAGE` legacy reads, and various stubs
(`SET_VPP`, `SET_HOLD`, `GET_BUTTON`, `GET_UID`, `READ_FPGA_VERSION`,
`CHECK_SOCKET`, etc.).

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

```
src/
├── main.rs           Entry point, USB + flash-bus init, bulk worker task
├── usb_handler.rs    embassy_usb::Handler — dispatches CMD_* control transfers
├── protocol.rs       Command codes, enums, V2/V3 packet parsing
├── spi_flash.rs      All-PIO + DMA flash ops (single writes; single/dual/quad reads)
├── leds.rs           GPIO LED driver
└── config.rs         Device identity, constants
```

## License

MIT
