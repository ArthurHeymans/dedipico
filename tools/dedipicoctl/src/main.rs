use std::ffi::CStr;
use std::fs::File;
use std::io::{self, Read, Write};
use std::mem::MaybeUninit;
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::ptr;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand, ValueEnum};

use nusb::MaybeFuture;
use nusb::io::{EndpointRead, EndpointWrite};
use nusb::transfer::{Bulk, In, Out};

const VID: u16 = 0x0483;
const PID: u16 = 0xDADA;
const AUX_IFACE: u8 = 1;
const EP_OUT: u8 = 0x03;
const EP_IN: u8 = 0x84;
const PACKET_LEN: usize = 64;
const HEADER_LEN: usize = 3;
const MAX_PAYLOAD_LEN: usize = PACKET_LEN - HEADER_LEN;
const USB_READ_TIMEOUT: Duration = Duration::from_millis(1);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(1);

const CMD_GPIO_GET_STATE: u8 = 0x01;
const CMD_GPIO_SET_DIRECTION: u8 = 0x02;
const CMD_GPIO_SET_OUTPUT: u8 = 0x03;
const CMD_GPIO_PULSE_LOW: u8 = 0x04;
const CMD_UART_SET_BAUD: u8 = 0x10;
const CMD_UART_WRITE: u8 = 0x11;

const EVT_RESPONSE: u8 = 0x80;
const EVT_GPIO_STATE: u8 = 0x81;
const EVT_UART_DATA: u8 = 0x90;
const STATUS_OK: u8 = 0;
const STATUS_BUSY: u8 = 2;

const RESET: u8 = 1 << 0;
const POWER: u8 = 1 << 1;
const POWER_STATE: u8 = 1 << 2;
const AUX: u8 = 1 << 3;

struct Packet {
    kind: u8,
    request_id: u8,
    payload: Vec<u8>,
}

#[derive(Clone, Copy)]
struct GpioState {
    inputs: u8,
    _outputs: u8,
    directions: u8,
    _capabilities: u8,
}

impl GpioState {
    fn parse(data: &[u8]) -> io::Result<Self> {
        if data.len() < 4 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "short GPIO state response",
            ));
        }
        Ok(Self {
            inputs: data[0],
            _outputs: data[1],
            directions: data[2],
            _capabilities: data[3],
        })
    }

    fn display(self) -> String {
        let mut text = String::new();
        for (name, bit) in [
            ("reset", RESET),
            ("power", POWER),
            ("power-state", POWER_STATE),
            ("aux", AUX),
        ] {
            let direction = if self.directions & bit != 0 {
                "output"
            } else {
                "input"
            };
            let level = if self.inputs & bit != 0 {
                "high"
            } else {
                "low"
            };
            text.push_str(&format!("{name}: {direction} {level}\n"));
        }
        text
    }
}

struct DediPico {
    rx: EndpointRead<Bulk>,
    tx: EndpointWrite<Bulk>,
    next_request_id: u8,
}

impl DediPico {
    fn open() -> Result<Self, Box<dyn std::error::Error>> {
        let info = nusb::list_devices()
            .wait()?
            .find(|dev| dev.vendor_id() == VID && dev.product_id() == PID)
            .ok_or("DediPico not found")?;
        let device = info.open().wait()?;
        let interface = device.claim_interface(AUX_IFACE).wait()?;
        let tx = interface
            .endpoint::<Bulk, Out>(EP_OUT)?
            .writer(PACKET_LEN)
            .with_num_transfers(1)
            .with_write_timeout(REQUEST_TIMEOUT);
        let rx = interface
            .endpoint::<Bulk, In>(EP_IN)?
            .reader(PACKET_LEN)
            .with_num_transfers(2)
            .with_read_timeout(USB_READ_TIMEOUT);
        Ok(Self {
            rx,
            tx,
            next_request_id: 1,
        })
    }

    fn write_packet(&mut self, kind: u8, request_id: u8, payload: &[u8]) -> io::Result<()> {
        if payload.len() > MAX_PAYLOAD_LEN {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "auxiliary packet payload is too large",
            ));
        }

        let mut packet = [0u8; PACKET_LEN];
        packet[0] = kind;
        packet[1] = request_id;
        packet[2] = payload.len() as u8;
        packet[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
        self.tx.write_all(&packet[..HEADER_LEN + payload.len()])?;
        self.tx.flush()
    }

    fn read_packet(&mut self) -> io::Result<Packet> {
        let mut header = [0u8; HEADER_LEN];
        self.rx.read_exact(&mut header)?;
        let payload_len = header[2] as usize;
        if payload_len > MAX_PAYLOAD_LEN {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid auxiliary packet length",
            ));
        }
        let mut payload = vec![0u8; payload_len];
        self.rx.read_exact(&mut payload)?;
        Ok(Packet {
            kind: header[0],
            request_id: header[1],
            payload,
        })
    }

    fn request(&mut self, command: u8, payload: &[u8], pty: &mut File) -> io::Result<Vec<u8>> {
        let request_id = self.next_request_id;
        self.next_request_id = self.next_request_id.wrapping_add(1).max(1);
        self.write_packet(command, request_id, payload)?;

        let deadline = Instant::now() + REQUEST_TIMEOUT;
        loop {
            match self.read_packet() {
                Ok(packet) if packet.kind == EVT_RESPONSE && packet.request_id == request_id => {
                    if packet.payload.len() < 2 || packet.payload[0] != command {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidData,
                            "malformed auxiliary response",
                        ));
                    }
                    if packet.payload[1] == STATUS_BUSY {
                        return Err(io::Error::new(
                            io::ErrorKind::WouldBlock,
                            "device UART transmit queue is full",
                        ));
                    }
                    if packet.payload[1] != STATUS_OK {
                        return Err(io::Error::other(format!(
                            "device rejected command 0x{command:02x} with status {}",
                            packet.payload[1]
                        )));
                    }
                    return Ok(packet.payload[2..].to_vec());
                }
                Ok(packet) => forward_event(packet, pty)?,
                Err(error) if error.kind() == io::ErrorKind::TimedOut => {
                    if Instant::now() >= deadline {
                        return Err(io::Error::new(
                            io::ErrorKind::TimedOut,
                            "timed out waiting for device response",
                        ));
                    }
                }
                Err(error) => return Err(error),
            }
        }
    }

    fn gpio_state(&mut self, pty: &mut File) -> io::Result<GpioState> {
        GpioState::parse(&self.request(CMD_GPIO_GET_STATE, &[], pty)?)
    }

    fn set_direction(&mut self, mask: u8, directions: u8, pty: &mut File) -> io::Result<GpioState> {
        GpioState::parse(&self.request(CMD_GPIO_SET_DIRECTION, &[mask, directions], pty)?)
    }

    fn set_output(&mut self, mask: u8, values: u8, pty: &mut File) -> io::Result<GpioState> {
        GpioState::parse(&self.request(CMD_GPIO_SET_OUTPUT, &[mask, values], pty)?)
    }

    fn pulse_low(&mut self, mask: u8, ms: u16, pty: &mut File) -> io::Result<GpioState> {
        let [lo, hi] = ms.to_le_bytes();
        GpioState::parse(&self.request(CMD_GPIO_PULSE_LOW, &[mask, lo, hi], pty)?)
    }

    fn set_baud(&mut self, baud: u32, pty: &mut File) -> io::Result<()> {
        self.request(CMD_UART_SET_BAUD, &baud.to_le_bytes(), pty)?;
        Ok(())
    }

    fn uart_write(&mut self, mut data: &[u8], pty: &mut File) -> io::Result<()> {
        while !data.is_empty() {
            let len = data.len().min(MAX_PAYLOAD_LEN);
            match self.request(CMD_UART_WRITE, &data[..len], pty) {
                Ok(_) => data = &data[len..],
                // PTY input is best-effort when the target baud rate cannot
                // keep up. Dropping it preserves management responsiveness.
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => return Ok(()),
                Err(error) => return Err(error),
            }
        }
        Ok(())
    }
}

fn forward_event(packet: Packet, pty: &mut File) -> io::Result<()> {
    match packet.kind {
        EVT_UART_DATA => write_nonblocking(pty, &packet.payload),
        EVT_GPIO_STATE => {
            let _ = GpioState::parse(&packet.payload)?;
            Ok(())
        }
        _ => Ok(()),
    }
}

fn write_nonblocking(file: &mut File, mut data: &[u8]) -> io::Result<()> {
    while !data.is_empty() {
        match file.write(data) {
            Ok(0) => break,
            Ok(written) => data = &data[written..],
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => break,
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

fn open_pty() -> io::Result<(File, String)> {
    let mut master = 0;
    let mut slave = 0;
    let rc = unsafe {
        libc::openpty(
            &mut master,
            &mut slave,
            ptr::null_mut(),
            ptr::null(),
            ptr::null(),
        )
    };
    if rc != 0 {
        return Err(io::Error::last_os_error());
    }

    let master_file = unsafe { File::from_raw_fd(master) };
    let slave_file = unsafe { File::from_raw_fd(slave) };
    let name = unsafe {
        let ptr = libc::ttyname(slave_file.as_raw_fd());
        if ptr.is_null() {
            return Err(io::Error::last_os_error());
        }
        CStr::from_ptr(ptr).to_string_lossy().into_owned()
    };

    set_raw(slave_file.as_raw_fd())?;
    set_nonblocking(master_file.as_raw_fd())?;
    drop(slave_file);
    Ok((master_file, name))
}

fn set_raw(fd: i32) -> io::Result<()> {
    let mut term = MaybeUninit::<libc::termios>::uninit();
    if unsafe { libc::tcgetattr(fd, term.as_mut_ptr()) } != 0 {
        return Err(io::Error::last_os_error());
    }
    let mut term = unsafe { term.assume_init() };
    unsafe { libc::cfmakeraw(&mut term) };
    if unsafe { libc::tcsetattr(fd, libc::TCSANOW, &term) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

fn set_nonblocking(fd: i32) -> io::Result<()> {
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    if flags < 0 || unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) } < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

struct SocketGuard(PathBuf);

impl Drop for SocketGuard {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

fn default_socket_path() -> PathBuf {
    std::env::var_os("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(format!("/tmp/dedipico-{}", unsafe { libc::geteuid() })))
        .join("dedipico.sock")
}

fn bind_socket(path: &Path) -> io::Result<(UnixListener, SocketGuard)> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    if path.exists() {
        if UnixStream::connect(path).is_ok() {
            return Err(io::Error::new(
                io::ErrorKind::AddrInUse,
                format!("daemon is already listening on {}", path.display()),
            ));
        }
        std::fs::remove_file(path)?;
    }
    let listener = UnixListener::bind(path)?;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
    listener.set_nonblocking(true)?;
    Ok((listener, SocketGuard(path.to_owned())))
}

fn run_daemon(socket: &Path, baud: u32) -> Result<(), Box<dyn std::error::Error>> {
    let (listener, _socket_guard) = bind_socket(socket)?;
    // Keep only the master open so the PTY has no hidden slave owner. This lets
    // the kernel report detachments when the terminal program closes its fd.
    let (mut master, pty_name) = open_pty()?;
    let mut device = DediPico::open()?;
    device.set_baud(baud, &mut master)?;

    println!("pty: {pty_name}");
    println!("socket: {}", socket.display());

    let mut pty_buf = [0u8; MAX_PAYLOAD_LEN];
    loop {
        let mut pollfds = [
            libc::pollfd {
                fd: listener.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            },
            libc::pollfd {
                fd: master.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            },
        ];
        let result = unsafe { libc::poll(pollfds.as_mut_ptr(), pollfds.len() as _, 0) };
        if result < 0 {
            return Err(io::Error::last_os_error().into());
        }

        if pollfds[0].revents & libc::POLLIN != 0 {
            while let Ok((stream, _)) = listener.accept() {
                handle_client(stream, &mut device, &mut master)?;
            }
        }

        if pollfds[1].revents & libc::POLLIN != 0 {
            match master.read(&mut pty_buf) {
                Ok(0) => {}
                Ok(len) => device.uart_write(&pty_buf[..len], &mut master)?,
                Err(error)
                    if error.kind() == io::ErrorKind::WouldBlock
                        || error.raw_os_error() == Some(libc::EIO) =>
                {
                    // Linux returns EIO when no slave side of the PTY is open.
                }
                Err(error) => return Err(error.into()),
            }
        }

        match device.read_packet() {
            Ok(packet) => forward_event(packet, &mut master)?,
            Err(error) if error.kind() == io::ErrorKind::TimedOut => {}
            Err(error) => return Err(error.into()),
        }
    }
}

fn handle_client(mut stream: UnixStream, device: &mut DediPico, pty: &mut File) -> io::Result<()> {
    stream.set_read_timeout(Some(REQUEST_TIMEOUT))?;
    stream.set_write_timeout(Some(REQUEST_TIMEOUT))?;
    let mut request = String::new();
    stream.read_to_string(&mut request)?;

    let response = match execute_control(request.trim(), device, pty) {
        Ok(response) => response,
        Err(error) => format!("ERR {error}\n"),
    };
    stream.write_all(response.as_bytes())
}

fn execute_control(command: &str, device: &mut DediPico, pty: &mut File) -> io::Result<String> {
    let fields = command.split_ascii_whitespace().collect::<Vec<_>>();
    let parse_u8 = |index: usize| -> io::Result<u8> {
        fields
            .get(index)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "missing argument"))?
            .parse()
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "invalid argument"))
    };
    let parse_u16 = |index: usize| -> io::Result<u16> {
        fields
            .get(index)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "missing argument"))?
            .parse()
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "invalid argument"))
    };

    let state = match fields.first().copied() {
        Some("state") if fields.len() == 1 => device.gpio_state(pty)?,
        Some("dir") if fields.len() == 3 => {
            device.set_direction(parse_u8(1)?, parse_u8(2)?, pty)?
        }
        Some("set") if fields.len() == 3 => {
            let bit = parse_u8(1)?;
            device.set_direction(bit, bit, pty)?;
            device.set_output(bit, parse_u8(2)?, pty)?
        }
        Some("release") if fields.len() == 2 => device.set_direction(parse_u8(1)?, 0, pty)?,
        Some("pulse") if fields.len() == 3 => device.pulse_low(parse_u8(1)?, parse_u16(2)?, pty)?,
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid daemon command",
            ));
        }
    };
    Ok(state.display())
}

fn send_daemon_command(socket: &Path, command: &str) -> io::Result<()> {
    let mut stream = UnixStream::connect(socket).map_err(|error| {
        io::Error::new(
            error.kind(),
            format!(
                "cannot connect to {}: {error}; start `dedipicoctl daemon` first",
                socket.display()
            ),
        )
    })?;
    stream.write_all(command.as_bytes())?;
    stream.shutdown(std::net::Shutdown::Write)?;
    let mut response = String::new();
    stream.read_to_string(&mut response)?;
    if let Some(error) = response.strip_prefix("ERR ") {
        return Err(io::Error::other(error.trim().to_owned()));
    }
    print!("{response}");
    Ok(())
}

#[derive(Parser)]
struct Cli {
    #[arg(long, global = true)]
    socket: Option<PathBuf>,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Own the USB interface and expose both a UART PTY and control socket.
    Daemon {
        #[arg(default_value_t = 115_200)]
        baud: u32,
    },
    State,
    Dir {
        pin: Pin,
        direction: Direction,
    },
    Set {
        pin: Pin,
        value: Level,
    },
    Release {
        pin: Pin,
    },
    Pulse {
        pin: Pin,
        ms: u16,
    },
    Reset {
        #[arg(default_value_t = 100)]
        ms: u16,
    },
    Power {
        #[arg(default_value_t = 500)]
        ms: u16,
    },
    Poweroff {
        #[arg(default_value_t = 5000)]
        ms: u16,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum Pin {
    Reset,
    Power,
    PowerState,
    Aux,
}

impl Pin {
    fn bit(self) -> u8 {
        match self {
            Self::Reset => RESET,
            Self::Power => POWER,
            Self::PowerState => POWER_STATE,
            Self::Aux => AUX,
        }
    }
}

#[derive(Clone, Copy, ValueEnum)]
enum Direction {
    In,
    Out,
}

#[derive(Clone, Copy, ValueEnum)]
enum Level {
    #[value(name = "0")]
    Low,
    #[value(name = "1")]
    High,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    let socket = cli.socket.unwrap_or_else(default_socket_path);

    match cli.command {
        Command::Daemon { baud } => run_daemon(&socket, baud),
        Command::State => {
            send_daemon_command(&socket, "state")?;
            Ok(())
        }
        Command::Dir { pin, direction } => {
            let bit = pin.bit();
            let directions = match direction {
                Direction::In => 0,
                Direction::Out => bit,
            };
            send_daemon_command(&socket, &format!("dir {bit} {directions}"))?;
            Ok(())
        }
        Command::Set { pin, value } => {
            let bit = pin.bit();
            let value = match value {
                Level::Low => 0,
                Level::High => bit,
            };
            send_daemon_command(&socket, &format!("set {bit} {value}"))?;
            Ok(())
        }
        Command::Release { pin } => {
            send_daemon_command(&socket, &format!("release {}", pin.bit()))?;
            Ok(())
        }
        Command::Pulse { pin, ms } => {
            send_daemon_command(&socket, &format!("pulse {} {ms}", pin.bit()))?;
            Ok(())
        }
        Command::Reset { ms } => {
            send_daemon_command(&socket, &format!("pulse {RESET} {ms}"))?;
            Ok(())
        }
        Command::Power { ms } | Command::Poweroff { ms } => {
            send_daemon_command(&socket, &format!("pulse {POWER} {ms}"))?;
            Ok(())
        }
    }
}
