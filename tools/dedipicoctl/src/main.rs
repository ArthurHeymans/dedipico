use std::env;
use std::ffi::CStr;
use std::fs::File;
use std::io::{self, Read, Write};
use std::mem::MaybeUninit;
use std::os::fd::{AsRawFd, FromRawFd};
use std::ptr;
use std::time::Duration;

use nusb::io::{EndpointRead, EndpointWrite};
use nusb::transfer::{Bulk, In, Out};
use nusb::MaybeFuture;

const VID: u16 = 0x0483;
const PID: u16 = 0xDADA;
const AUX_IFACE: u8 = 1;
const EP_OUT: u8 = 0x03;
const EP_IN: u8 = 0x84;
const FRAME_LEN: usize = 64;
const READ_TIMEOUT: Duration = Duration::from_secs(1);
const DRAIN_TIMEOUT: Duration = Duration::from_millis(50);

const CMD_GPIO_GET_STATE: u8 = 0x01;
const CMD_GPIO_SET_DIRECTION: u8 = 0x02;
const CMD_GPIO_SET_OUTPUT: u8 = 0x03;
const CMD_GPIO_PULSE_LOW: u8 = 0x04;
const CMD_UART_SET_BAUD: u8 = 0x10;
const CMD_UART_WRITE: u8 = 0x11;

const EVT_GPIO_STATE: u8 = 0x81;
const EVT_UART_DATA: u8 = 0x90;

const RESET: u8 = 1 << 0;
const POWER: u8 = 1 << 1;
const POWER_STATE: u8 = 1 << 2;
const AUX: u8 = 1 << 3;

struct DediPico {
    rx: EndpointRead<Bulk>,
    tx: EndpointWrite<Bulk>,
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
            .writer(FRAME_LEN)
            .with_num_transfers(1)
            .with_write_timeout(Duration::from_secs(1));
        let rx = interface
            .endpoint::<Bulk, In>(EP_IN)?
            .reader(FRAME_LEN)
            .with_num_transfers(1)
            .with_read_timeout(READ_TIMEOUT);
        let mut dev = Self { rx, tx };
        dev.drain_stale();
        Ok(dev)
    }

    fn drain_stale(&mut self) {
        self.rx.set_read_timeout(DRAIN_TIMEOUT);
        while self.read_frame().is_ok() {}
        self.rx.set_read_timeout(READ_TIMEOUT);
    }

    fn write_frame_once(&mut self, frame: &[u8]) -> io::Result<()> {
        let mut buf = [0u8; FRAME_LEN];
        let len = frame.len().min(FRAME_LEN);
        buf[..len].copy_from_slice(&frame[..len]);
        self.tx.write_all(&buf)?;
        self.tx.flush()
    }

    fn write_frame(&mut self, frame: &[u8]) -> io::Result<()> {
        match self.write_frame_once(frame) {
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                self.drain_stale();
                self.write_frame_once(frame)
            }
            result => result,
        }
    }

    fn read_frame(&mut self) -> io::Result<[u8; FRAME_LEN]> {
        let mut frame = [0u8; FRAME_LEN];
        self.rx.read_exact(&mut frame)?;
        Ok(frame)
    }

    fn read_gpio_state(&mut self) -> io::Result<[u8; FRAME_LEN]> {
        loop {
            let frame = self.read_frame()?;
            if frame[0] == EVT_GPIO_STATE {
                return Ok(frame);
            }
        }
    }

    fn gpio_state(&mut self) -> io::Result<[u8; FRAME_LEN]> {
        self.write_frame(&[CMD_GPIO_GET_STATE])?;
        self.read_gpio_state()
    }

    fn set_direction(&mut self, mask: u8, directions: u8) -> io::Result<[u8; FRAME_LEN]> {
        self.write_frame(&[CMD_GPIO_SET_DIRECTION, mask, directions])?;
        self.read_gpio_state()
    }

    fn set_output(&mut self, mask: u8, values: u8) -> io::Result<[u8; FRAME_LEN]> {
        self.write_frame(&[CMD_GPIO_SET_OUTPUT, mask, values])?;
        self.read_gpio_state()
    }

    fn pulse_low(&mut self, mask: u8, ms: u16) -> io::Result<[u8; FRAME_LEN]> {
        self.write_frame(&[CMD_GPIO_PULSE_LOW, mask, ms as u8, (ms >> 8) as u8])?;
        std::thread::sleep(Duration::from_millis(ms as u64));
        self.read_gpio_state()
    }

    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        let mut frame = [0u8; 5];
        frame[0] = CMD_UART_SET_BAUD;
        frame[1..].copy_from_slice(&baud.to_le_bytes());
        self.write_frame(&frame)
    }

    fn uart_write(&mut self, mut data: &[u8]) -> io::Result<()> {
        while !data.is_empty() {
            let len = data.len().min(FRAME_LEN - 2);
            let mut frame = [0u8; FRAME_LEN];
            frame[0] = CMD_UART_WRITE;
            frame[1] = len as u8;
            frame[2..2 + len].copy_from_slice(&data[..len]);
            self.write_frame(&frame)?;
            data = &data[len..];
        }
        Ok(())
    }
}

fn pin(name: &str) -> Result<u8, String> {
    match name {
        "reset" => Ok(RESET),
        "power" => Ok(POWER),
        "power-state" => Ok(POWER_STATE),
        "aux" => Ok(AUX),
        _ => Err(format!(
            "unknown pin {name}; use reset, power, power-state, aux"
        )),
    }
}

fn print_state(frame: &[u8; FRAME_LEN]) {
    let inputs = frame[1];
    let directions = frame[3];
    for (name, bit) in [
        ("reset", RESET),
        ("power", POWER),
        ("power-state", POWER_STATE),
        ("aux", AUX),
    ] {
        println!(
            "{name}: {} {}",
            if directions & bit != 0 {
                "output"
            } else {
                "input"
            },
            if inputs & bit != 0 { "high" } else { "low" }
        );
    }
}

fn open_pty() -> io::Result<(File, File, String)> {
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

    let name = unsafe {
        let ptr = libc::ttyname(slave);
        if ptr.is_null() {
            return Err(io::Error::last_os_error());
        }
        CStr::from_ptr(ptr).to_string_lossy().into_owned()
    };

    let master_file = unsafe { File::from_raw_fd(master) };
    let slave_file = unsafe { File::from_raw_fd(slave) };
    set_raw(master_file.as_raw_fd())?;
    Ok((master_file, slave_file, name))
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

fn run_tty(mut dev: DediPico, baud: u32) -> io::Result<()> {
    dev.set_baud(baud)?;
    let (mut master, _slave, name) = open_pty()?;
    println!("{name}");
    let mut pty_buf = [0u8; 4096];

    loop {
        let mut pollfd = libc::pollfd {
            fd: master.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let rc = unsafe { libc::poll(&mut pollfd, 1, 10) };
        if rc < 0 {
            return Err(io::Error::last_os_error());
        }
        if rc > 0 && pollfd.revents & libc::POLLIN != 0 {
            let n = master.read(&mut pty_buf)?;
            if n != 0 {
                dev.uart_write(&pty_buf[..n])?;
            }
        }

        match dev.read_frame() {
            Ok(frame) if frame[0] == EVT_UART_DATA => {
                let len = (frame[1] as usize).min(FRAME_LEN - 2);
                master.write_all(&frame[2..2 + len])?;
            }
            Ok(_) => {}
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e),
        }
    }
}

fn usage() -> ! {
    eprintln!("usage:");
    eprintln!("  dedipicoctl state");
    eprintln!("  dedipicoctl dir <reset|power|power-state|aux> <in|out>");
    eprintln!("  dedipicoctl set <reset|power> <0|1>");
    eprintln!("  dedipicoctl release <reset|power>");
    eprintln!("  dedipicoctl pulse <reset|power> <ms>");
    eprintln!("  dedipicoctl reset [ms=100]");
    eprintln!("  dedipicoctl power [ms=500]");
    eprintln!("  dedipicoctl poweroff [ms=5000]");
    eprintln!("  dedipicoctl tty [baud=115200]");
    std::process::exit(2);
}

fn parse_u16(s: Option<&String>, default: u16) -> u16 {
    s.map(|v| v.parse().unwrap_or_else(|_| usage()))
        .unwrap_or(default)
}

fn parse_u32(s: Option<&String>, default: u32) -> u32 {
    s.map(|v| v.parse().unwrap_or_else(|_| usage()))
        .unwrap_or(default)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let Some(cmd) = args.get(1).map(String::as_str) else {
        usage();
    };

    let mut dev = DediPico::open()?;
    match cmd {
        "state" => print_state(&dev.gpio_state()?),
        "dir" if args.len() == 4 => {
            let bit = pin(&args[2])?;
            let dirs = match args[3].as_str() {
                "in" => 0,
                "out" => bit,
                _ => usage(),
            };
            print_state(&dev.set_direction(bit, dirs)?);
        }
        "set" if args.len() == 4 => {
            let bit = pin(&args[2])?;
            let value = match args[3].as_str() {
                "0" => 0,
                "1" => bit,
                _ => usage(),
            };
            dev.set_direction(bit, bit)?;
            print_state(&dev.set_output(bit, value)?);
        }
        "release" if args.len() == 3 => print_state(&dev.set_direction(pin(&args[2])?, 0)?),
        "pulse" if args.len() == 4 => {
            print_state(&dev.pulse_low(pin(&args[2])?, parse_u16(args.get(3), 0))?)
        }
        "reset" => print_state(&dev.pulse_low(RESET, parse_u16(args.get(2), 100))?),
        "power" => print_state(&dev.pulse_low(POWER, parse_u16(args.get(2), 500))?),
        "poweroff" => print_state(&dev.pulse_low(POWER, parse_u16(args.get(2), 5000))?),
        "tty" => run_tty(dev, parse_u32(args.get(2), 115_200))?,
        _ => usage(),
    }
    Ok(())
}
