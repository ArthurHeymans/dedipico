/// USB control request handler — dispatches all Dediprog vendor commands.
///
/// This implements `embassy_usb::Handler` and is called synchronously from the
/// USB device task whenever a vendor control transfer arrives on EP0.
use defmt::*;
use embassy_usb::control::{InResponse, OutResponse, Request, RequestType};
use embassy_usb::Handler;

use crate::config;
use crate::leds::Leds;
use crate::protocol::*;
use crate::{BULK_OP, BULK_SIGNAL, SPI_FLASH};

pub struct DediprogHandler {
    /// Buffered read data from the last CMD_TRANSCEIVE SPI transaction.
    transceive_read_buf: [u8; 16],
    transceive_read_len: u8,

    /// LED driver.
    leds: Leds<'static>,

    /// Whether the device has been configured by the host.
    configured: bool,
}

impl DediprogHandler {
    pub fn new(leds: Leds<'static>) -> Self {
        Self {
            transceive_read_buf: [0u8; 16],
            transceive_read_len: 0,
            leds,
            configured: false,
        }
    }

    // =========================================================================
    // CMD_TRANSCEIVE (0x01) — SPI command passthrough
    // =========================================================================

    /// OUT phase: host sends SPI command bytes. We do the full SPI transaction
    /// (write + read) immediately using blocking SPI, then buffer the result
    /// for the subsequent IN phase.
    fn cmd_transceive_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        // Protocol V2: wValue bit 0 = 1 means a read phase will follow
        let needs_read = (req.value & 0x01) != 0;

        if data.is_empty() || data.len() > 16 {
            warn!("TRANSCEIVE OUT: bad length {}", data.len());
            return Some(OutResponse::Rejected);
        }

        if needs_read {
            // Read up to 16 bytes back from the flash.
            // We don't know the exact read count yet (it comes in the IN phase's
            // wLength), so we read the maximum and trim later.
            let mut read_buf = [0u8; 16];
            critical_section::with(|cs| {
                if let Some(flash) = SPI_FLASH.borrow(cs).borrow_mut().as_mut() {
                    flash.transceive_blocking(data, &mut read_buf);
                }
            });
            self.transceive_read_buf = read_buf;
            self.transceive_read_len = 16;
        } else {
            // Write-only: no read phase coming.
            critical_section::with(|cs| {
                if let Some(flash) = SPI_FLASH.borrow(cs).borrow_mut().as_mut() {
                    flash.write_only_blocking(data);
                }
            });
            self.transceive_read_len = 0;
        }

        debug!(
            "TRANSCEIVE OUT: wrote {} bytes, needs_read={}",
            data.len(),
            needs_read
        );
        Some(OutResponse::Accepted)
    }

    /// IN phase: return the buffered SPI read data.
    fn cmd_transceive_in<'a>(&self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        let len = (req.length as usize).min(self.transceive_read_len as usize);
        buf[..len].copy_from_slice(&self.transceive_read_buf[..len]);
        debug!("TRANSCEIVE IN: returning {} bytes", len);
        Some(InResponse::Accepted(&buf[..len]))
    }

    // =========================================================================
    // CMD_READ_PROG_INFO (0x08) — Device identification string
    // =========================================================================

    fn cmd_read_prog_info<'a>(&self, _req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        let s = config::DEVICE_STRING;
        let len = s.len().min(buf.len());
        buf[..len].copy_from_slice(&s[..len]);
        info!("READ_PROG_INFO: returning device string ({} bytes)", len);
        Some(InResponse::Accepted(&buf[..len]))
    }

    // =========================================================================
    // CMD_READ_EEPROM (0x05) — Serial ID
    // =========================================================================

    fn cmd_read_eeprom<'a>(&self, _req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        let id = &config::SERIAL_ID;
        let len = id.len().min(buf.len());
        buf[..len].copy_from_slice(&id[..len]);
        debug!("READ_EEPROM: returning {} bytes", len);
        Some(InResponse::Accepted(&buf[..len]))
    }

    // =========================================================================
    // CMD_SET_VOLTAGE (0x0B) — Legacy voltage read (REQTYPE_OTHER_IN)
    // =========================================================================

    fn cmd_set_voltage_legacy<'a>(
        &self,
        _req: Request,
        buf: &'a mut [u8],
    ) -> Option<InResponse<'a>> {
        // Return 0x6F in a 1-byte IN transfer (legacy path).
        if !buf.is_empty() {
            buf[0] = 0x6F;
            Some(InResponse::Accepted(&buf[..1]))
        } else {
            Some(InResponse::Rejected)
        }
    }

    // =========================================================================
    // CMD_SET_TARGET (0x04)
    // =========================================================================

    fn cmd_set_target(&self, req: Request) -> Option<OutResponse> {
        debug!("SET_TARGET: {}", req.value);
        Some(OutResponse::Accepted)
    }

    // =========================================================================
    // CMD_SET_VCC (0x09)
    // =========================================================================

    fn cmd_set_vcc(&self, req: Request) -> Option<OutResponse> {
        let label = match req.value {
            0x00 => "off",
            0x10 => "3.5V",
            0x11 => "2.5V",
            0x12 => "1.8V",
            _ => "unknown",
        };
        info!("SET_VCC: {} ({})", req.value, label);
        Some(OutResponse::Accepted)
    }

    // =========================================================================
    // CMD_SET_SPI_CLK (0x61)
    // =========================================================================

    fn cmd_set_spi_clk(&self, req: Request) -> Option<OutResponse> {
        let speed = SpiSpeed::from_code(req.value);
        let freq = speed.frequency_hz();
        info!("SET_SPI_CLK: {} Hz", freq);
        critical_section::with(|cs| {
            if let Some(flash) = SPI_FLASH.borrow(cs).borrow_mut().as_mut() {
                flash.set_frequency(freq);
            }
        });
        Some(OutResponse::Accepted)
    }

    // =========================================================================
    // CMD_SET_IO_LED (0x07)
    // =========================================================================

    fn cmd_set_io_led(&mut self, req: Request) -> Option<OutResponse> {
        self.leds.set_from_wvalue(req.value);
        debug!("SET_IO_LED: wValue=0x{:04x}", req.value);
        Some(OutResponse::Accepted)
    }

    // =========================================================================
    // CMD_SET_STANDALONE (0x0A) — ACK and ignore
    // =========================================================================

    fn cmd_set_standalone(&self, req: Request) -> Option<OutResponse> {
        debug!("SET_STANDALONE: wValue={}", req.value);
        Some(OutResponse::Accepted)
    }

    // =========================================================================
    // CMD_IO_MODE (0x15)
    // =========================================================================

    fn cmd_io_mode(&self, req: Request) -> Option<OutResponse> {
        debug!("IO_MODE: {}", req.value);
        // We only support single I/O; dual/quad would need PIO.
        Some(OutResponse::Accepted)
    }

    // =========================================================================
    // CMD_READ (0x20) — Bulk read setup
    // =========================================================================

    fn cmd_read_setup(&self, _req: Request, data: &[u8]) -> Option<OutResponse> {
        let (block_count, mode_byte, opcode, address) = match parse_rw_cmd_v2(data) {
            Some(v) => v,
            None => {
                warn!("READ setup: bad packet (len={})", data.len());
                return Some(OutResponse::Rejected);
            }
        };

        let read_mode = ReadMode::from_byte(mode_byte);
        let (addr_len, dummy_bytes) = match read_mode {
            Some(mode) => {
                let addr_len = if mode.uses_4byte_addr() { 4 } else { 3 };
                (addr_len, mode.dummy_bytes())
            }
            None => {
                // Default: 3-byte address, no dummy (standard read)
                (3u8, 0u8)
            }
        };

        let actual_opcode = if opcode != 0 { opcode } else { 0x03 }; // default: standard read

        info!(
            "READ setup: addr=0x{:08x} blocks={} opcode=0x{:02x} mode={} addr_len={} dummy={}",
            address, block_count, actual_opcode, mode_byte, addr_len, dummy_bytes
        );

        let op = BulkOperation::Read {
            address,
            block_count,
            opcode: actual_opcode,
            addr_len,
            dummy_bytes,
        };
        critical_section::with(|cs| {
            *BULK_OP.borrow(cs).borrow_mut() = Some(op);
        });
        BULK_SIGNAL.signal(());

        Some(OutResponse::Accepted)
    }

    // =========================================================================
    // CMD_WRITE (0x30) — Bulk write setup
    // =========================================================================

    fn cmd_write_setup(&self, _req: Request, data: &[u8]) -> Option<OutResponse> {
        let (block_count, mode_byte, opcode, address) = match parse_rw_cmd_v2(data) {
            Some(v) => v,
            None => {
                warn!("WRITE setup: bad packet (len={})", data.len());
                return Some(OutResponse::Rejected);
            }
        };

        let write_mode = WriteMode::from_byte(mode_byte);
        let addr_len = match write_mode {
            Some(mode) if mode.uses_4byte_addr() => 4u8,
            _ => 3u8,
        };

        let actual_opcode = if opcode != 0 { opcode } else { 0x02 }; // default: page program

        info!(
            "WRITE setup: addr=0x{:08x} blocks={} opcode=0x{:02x} mode={} addr_len={}",
            address, block_count, actual_opcode, mode_byte, addr_len
        );

        let op = BulkOperation::Write {
            address,
            block_count,
            opcode: actual_opcode,
            addr_len,
        };
        critical_section::with(|cs| {
            *BULK_OP.borrow(cs).borrow_mut() = Some(op);
        });
        BULK_SIGNAL.signal(());

        Some(OutResponse::Accepted)
    }

    // =========================================================================
    // Stub commands
    // =========================================================================

    fn cmd_get_button<'a>(&self, _req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if !buf.is_empty() {
            buf[0] = 0x01; // button not pressed
            Some(InResponse::Accepted(&buf[..1]))
        } else {
            Some(InResponse::Rejected)
        }
    }

    fn cmd_check_socket<'a>(&self, _req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        // Return success (0x00).
        if !buf.is_empty() {
            buf[0] = 0x00;
            Some(InResponse::Accepted(&buf[..1]))
        } else {
            Some(InResponse::Accepted(&buf[..0]))
        }
    }
}

// =============================================================================
// embassy_usb::Handler implementation
// =============================================================================

impl Handler for DediprogHandler {
    fn configured(&mut self, configured: bool) {
        self.configured = configured;
        if configured {
            info!("USB configured");
        } else {
            info!("USB deconfigured");
        }
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        // We handle VENDOR requests with any recipient (Endpoint, Other, etc.)
        if req.request_type != RequestType::Vendor {
            return None;
        }

        match req.request {
            CMD_TRANSCEIVE => self.cmd_transceive_out(req, data),
            CMD_SET_VPP => {
                debug!("SET_VPP: ACK");
                Some(OutResponse::Accepted)
            }
            CMD_SET_TARGET => self.cmd_set_target(req),
            CMD_SET_IO_LED => self.cmd_set_io_led(req),
            CMD_SET_VCC => self.cmd_set_vcc(req),
            CMD_SET_STANDALONE => self.cmd_set_standalone(req),
            CMD_IO_MODE => self.cmd_io_mode(req),
            CMD_SET_CS => {
                // Manual CS control — assert (wValue=0) or deassert (wValue=1)
                debug!("SET_CS: wValue={}", req.value);
                critical_section::with(|cs| {
                    if let Some(flash) = SPI_FLASH.borrow(cs).borrow_mut().as_mut() {
                        if req.value == 0 {
                            flash.cs_assert();
                        } else {
                            flash.cs_deassert();
                        }
                    }
                });
                Some(OutResponse::Accepted)
            }
            CMD_SET_HOLD => {
                debug!("SET_HOLD: ACK");
                Some(OutResponse::Accepted)
            }
            CMD_READ => self.cmd_read_setup(req, data),
            CMD_WRITE => self.cmd_write_setup(req, data),
            CMD_SET_SPI_CLK => self.cmd_set_spi_clk(req),
            _ => {
                debug!("Unknown OUT cmd 0x{:02x}, ACK", req.request);
                Some(OutResponse::Accepted)
            }
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type != RequestType::Vendor {
            return None;
        }

        match req.request {
            CMD_TRANSCEIVE => self.cmd_transceive_in(req, buf),
            CMD_READ_EEPROM => self.cmd_read_eeprom(req, buf),
            CMD_READ_PROG_INFO => self.cmd_read_prog_info(req, buf),
            CMD_SET_VOLTAGE => self.cmd_set_voltage_legacy(req, buf),
            CMD_GET_BUTTON => self.cmd_get_button(req, buf),
            CMD_GET_UID => {
                // Return a fixed 8-byte UID.
                let uid: [u8; 8] = [0xDE, 0xD1, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x01];
                let len = uid.len().min(buf.len());
                buf[..len].copy_from_slice(&uid[..len]);
                Some(InResponse::Accepted(&buf[..len]))
            }
            CMD_READ_FPGA_VERSION => {
                // Return dummy FPGA version bytes.
                let ver: [u8; 2] = [0x00, 0x01];
                let len = ver.len().min(buf.len()).min(req.length as usize);
                buf[..len].copy_from_slice(&ver[..len]);
                Some(InResponse::Accepted(&buf[..len]))
            }
            CMD_CHECK_SOCKET => self.cmd_check_socket(req, buf),
            _ => {
                debug!("Unknown IN cmd 0x{:02x}, reject", req.request);
                None
            }
        }
    }
}
