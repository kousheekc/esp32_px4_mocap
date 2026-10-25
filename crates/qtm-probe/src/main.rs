//! Flight recorder for the QTM stream, using the bridge firmware's own `qtmrt` code.
//!
//! Keeps a QTM RT session open (reconnecting whenever QTM goes away), records every UDP frame
//! that arrives into a rolling in-memory buffer of the last `--window` seconds, and writes that
//! buffer to a CSV file on demand: press Enter, or `pkill -USR1 qtm-probe` from another shell.
//! Leave it running all day; when a failsafe happens, dump the buffer.
//!
//!     cargo run --release -p qtm-probe -- 192.168.3.92
//!     cargo run --release -p qtm-probe -- 192.168.3.92 --bind 192.168.3.50   # force an interface

use std::collections::VecDeque;
use std::fs::File;
use std::io::{self, BufRead, BufWriter, Read, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpStream, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::Parser;
use qtmrt::{FrameTracker, PacketType};
use socket2::{Domain, Protocol, Socket, Type};

/// Same as the firmware: re-request the stream after this long without frames.
const SILENCE: Duration = Duration::from_millis(3000);

#[derive(Parser)]
#[command(about = "Record the QTM stream into a rolling buffer and dump it to CSV on demand")]
struct Args {
    /// QTM PC address
    host: IpAddr,
    /// QTM RT port
    #[arg(long, default_value_t = qtmrt::DEFAULT_PORT)]
    port: u16,
    /// Local UDP port for frames
    #[arg(long, default_value_t = 22230)]
    udp_port: u16,
    /// Requested stream rate in Hz
    #[arg(long, default_value_t = 100)]
    rate: u32,
    /// Rigid body number, 1-based
    #[arg(long, default_value_t = 1)]
    body: usize,
    /// Local IP to use, picks the interface (cable/WiFi)
    #[arg(long)]
    bind: Option<IpAddr>,
    /// UDP receive buffer in bytes; small values show overflow drops a small receiver would see
    #[arg(long, default_value_t = 1 << 20)]
    rcvbuf: usize,
    /// Seconds of history kept in memory and written on a dump
    #[arg(long, default_value_t = 300)]
    window: u64,
    /// Directory dumps are written to
    #[arg(long, default_value = ".")]
    out_dir: PathBuf,
    /// Seconds between status lines on stdout, 0 to disable
    #[arg(long, default_value_t = 60)]
    status_every: u64,
}

/// One line of the recording
#[derive(Clone)]
struct Record {
    /// Wall-clock time, for lining the dump up with other logs
    unix_us: u64,
    /// Monotonic time since the probe started; what gaps and ages are measured on
    mono_us: u64,
    what: What,
}

#[derive(Clone)]
enum What {
    Frame(Frame),
    Event(String),
}

#[derive(Clone)]
struct Frame {
    len: u16,
    frame: u32,
    /// Frame-number step from the previous fresh frame (0 for the first one)
    step: u32,
    /// Frames lost between the previous fresh frame and this one
    missed: u32,
    dup: bool,
    /// Capture time on QTM's clock
    qtm_us: u64,
    /// Arrival minus capture, capture mapped to the local clock (fresh frames only)
    age_us: Option<u32>,
    /// Time since the previous Data packet arrived
    gap_us: Option<u32>,
    pose: Option<Pose>,
}

#[derive(Clone)]
struct Pose {
    pos_mm: [f32; 3],
    quat_xyzw: [f32; 4],
    valid: bool,
}

/// What the side threads tell the main loop
enum Msg {
    Event(String),
    SessionEnded(String),
    Dump,
}

fn unix_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_micros() as u64)
}

/// `2026-10-25T13:02:25.578123Z` from unix microseconds
fn utc(unix_us: u64) -> String {
    let secs = unix_us / 1_000_000;
    let (h, m, s) = (secs / 3600 % 24, secs / 60 % 60, secs % 60);
    // Civil-from-days, Howard Hinnant's algorithm.
    let z = (secs / 86_400) as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{h:02}:{m:02}:{s:02}.{:06}Z",
        unix_us % 1_000_000
    )
}

fn send_command(tcp: &mut TcpStream, cmd: &str) -> io::Result<()> {
    let mut buf = [0u8; 128];
    let n = qtmrt::encode_command(cmd, &mut buf)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "command too long"))?;
    tcp.write_all(&buf[..n])
}

/// Read one whole control packet as (type, payload)
fn read_packet(tcp: &mut TcpStream) -> io::Result<(PacketType, Vec<u8>)> {
    let mut buf = vec![0u8; qtmrt::HEADER_LEN];
    tcp.read_exact(&mut buf)?;
    let len = qtmrt::packet_len(&buf)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "bad packet header"))?;
    buf.resize(len, 0);
    tcp.read_exact(&mut buf[qtmrt::HEADER_LEN..])?;
    let p = qtmrt::parse_packet(&buf).expect("length checked above");
    Ok((p.kind, p.payload.to_vec()))
}

fn text(payload: &[u8]) -> String {
    qtmrt::Packet {
        kind: PacketType::Command,
        payload,
    }
    .text()
    .unwrap_or("?")
    .to_owned()
}

/// Connect and start streaming; QTM streams UDP back to the address the TCP session comes from
fn open_session(args: &Args, log: &mut impl FnMut(String)) -> io::Result<TcpStream> {
    let addr = SocketAddr::new(args.host, args.port);
    let tcp = Socket::new(Domain::for_address(addr), Type::STREAM, Some(Protocol::TCP))?;
    if let Some(ip) = args.bind {
        tcp.bind(&SocketAddr::new(ip, 0).into())
            .map_err(|e| io::Error::new(e.kind(), format!("binding to {ip}: {e}")))?;
    }
    tcp.connect_timeout(&addr.into(), Duration::from_secs(5))
        .map_err(|e| io::Error::new(e.kind(), format!("connecting to {addr}: {e}")))?;
    let mut tcp: TcpStream = tcp.into();
    tcp.set_read_timeout(Some(Duration::from_secs(5)))?;
    log(format!(
        "connected from {} to {addr}",
        tcp.local_addr()?.ip()
    ));

    log(format!("qtm: {}", text(&read_packet(&mut tcp)?.1)));
    send_command(&mut tcp, "Version 1.19")?;
    let (kind, payload) = read_packet(&mut tcp)?;
    if kind == PacketType::Error {
        return Err(io::Error::other(format!(
            "version rejected: {}",
            text(&payload)
        )));
    }
    log(format!("qtm: {}", text(&payload)));
    send_command(&mut tcp, &stream_command(args))?;
    log(format!("requested \"{}\"", stream_command(args)));
    Ok(tcp)
}

fn stream_command(args: &Args) -> String {
    format!(
        "StreamFrames Frequency:{} UDP:{} 6D",
        args.rate, args.udp_port
    )
}

/// Control channel: errors, events, or EOF when QTM goes away
fn spawn_control_reader(mut tcp: TcpStream, tx: Sender<Msg>) -> io::Result<()> {
    tcp.set_read_timeout(None)?;
    thread::spawn(move || {
        loop {
            let msg = match read_packet(&mut tcp) {
                Ok((PacketType::Error, p)) => Msg::Event(format!("qtm: error: {}", text(&p))),
                Ok((PacketType::Event, p)) => Msg::Event(format!("qtm: event {:?}", p.first())),
                Ok((kind, p)) => Msg::Event(format!("qtm: {kind:?} packet ({} B)", p.len())),
                Err(e) => {
                    let _ = tx.send(Msg::SessionEnded(e.to_string()));
                    return;
                }
            };
            if tx.send(msg).is_err() {
                return;
            }
        }
    });
    Ok(())
}

/// Enter on stdin requests a dump
fn spawn_stdin_reader(tx: Sender<Msg>) {
    thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            if line.is_err() || tx.send(Msg::Dump).is_err() {
                return;
            }
        }
    });
}

const CSV_HEADER: &str = "utc,unix_s,mono_s,kind,len,frame,step,missed,qtm_s,age_ms,gap_ms,\
                          valid,x_mm,y_mm,z_mm,qx,qy,qz,qw,note";

fn write_csv(path: &Path, records: &[Record]) -> io::Result<()> {
    let mut w = BufWriter::new(File::create(path)?);
    writeln!(w, "{CSV_HEADER}")?;
    let ms = |us: Option<u32>| us.map_or(String::new(), |v| format!("{:.3}", v as f64 / 1e3));
    for r in records {
        let t = format!(
            "{},{:.6},{:.6}",
            utc(r.unix_us),
            r.unix_us as f64 / 1e6,
            r.mono_us as f64 / 1e6
        );
        match &r.what {
            What::Frame(f) => {
                let pose = f.pose.as_ref().map_or(",,,,,,,".to_owned(), |p| {
                    let [x, y, z] = p.pos_mm;
                    let [qx, qy, qz, qw] = p.quat_xyzw;
                    format!(
                        "{},{x:.2},{y:.2},{z:.2},{qx:.5},{qy:.5},{qz:.5},{qw:.5}",
                        p.valid
                    )
                });
                writeln!(
                    w,
                    "{t},{},{},{},{},{},{:.6},{},{},{pose},",
                    if f.dup { "dup" } else { "frame" },
                    f.len,
                    f.frame,
                    f.step,
                    f.missed,
                    f.qtm_us as f64 / 1e6,
                    ms(f.age_us),
                    ms(f.gap_us),
                )?;
            }
            What::Event(e) => writeln!(w, "{t},event,,,,,,,,,,,,,,,,\"{}\"", e.replace('"', "'"))?,
        }
    }
    w.flush()
}

/// Running counters for the periodic status line and the dump summary
#[derive(Default)]
struct Stats {
    frames: u64,
    missed: u64,
    dup: u64,
    max_gap_us: u32,
    max_age_us: u32,
}

impl Stats {
    fn add(&mut self, f: &Frame) {
        if f.dup {
            self.dup += 1;
        } else {
            self.frames += 1;
        }
        self.missed += u64::from(f.missed);
        self.max_gap_us = self.max_gap_us.max(f.gap_us.unwrap_or(0));
        self.max_age_us = self.max_age_us.max(f.age_us.unwrap_or(0));
    }

    fn line(&self, secs: f64) -> String {
        let expected = self.frames + self.missed;
        format!(
            "{} frames ({:.1} Hz), missed {} ({:.1}%), dup {} | gap max {:.0} ms, age max {:.0} ms",
            self.frames,
            self.frames as f64 / secs.max(1e-9),
            self.missed,
            if expected > 0 {
                100.0 * self.missed as f64 / expected as f64
            } else {
                0.0
            },
            self.dup,
            self.max_gap_us as f64 / 1e3,
            self.max_age_us as f64 / 1e3,
        )
    }
}

struct Recorder {
    start: Instant,
    window_us: u64,
    buf: VecDeque<Record>,
    /// Dumps still being written, joined before exit
    writers: Vec<thread::JoinHandle<()>>,
}

impl Recorder {
    fn now(&self) -> (u64, u64) {
        (unix_us(), self.start.elapsed().as_micros() as u64)
    }

    fn push(&mut self, unix_us: u64, mono_us: u64, what: What) {
        self.buf.push_back(Record {
            unix_us,
            mono_us,
            what,
        });
        while self
            .buf
            .front()
            .is_some_and(|r| mono_us - r.mono_us > self.window_us)
        {
            self.buf.pop_front();
        }
    }

    fn event(&mut self, text: String) {
        let (unix_us, mono_us) = self.now();
        println!("{} {text}", utc(unix_us));
        self.push(unix_us, mono_us, What::Event(text));
    }

    /// Write the buffer out on a thread so recording doesn't pause
    fn dump(&mut self, out_dir: &Path) {
        self.event("dump requested".to_owned());
        let records: Vec<Record> = self.buf.iter().cloned().collect();
        let (Some(first), Some(last)) = (records.first(), records.last()) else {
            return;
        };
        let span = (last.mono_us - first.mono_us) as f64 / 1e6;
        let mut stats = Stats::default();
        for r in &records {
            if let What::Frame(f) = &r.what {
                stats.add(f);
            }
        }
        let stamp = utc(last.unix_us);
        let path = out_dir.join(format!("qtm-{}.csv", stamp[..23].replace(':', "-")));
        println!(
            "dumping {span:.0} s to {}: {}",
            path.display(),
            stats.line(span)
        );
        self.writers.retain(|w| !w.is_finished());
        self.writers
            .push(thread::spawn(move || match write_csv(&path, &records) {
                Ok(()) => println!("wrote {} ({} rows)", path.display(), records.len()),
                Err(e) => eprintln!("writing {}: {e}", path.display()),
            }));
    }
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    let index = args.body.checked_sub(1).expect("--body is 1-based");
    std::fs::create_dir_all(&args.out_dir)?;

    let local_ip = args.bind.unwrap_or(IpAddr::V4(Ipv4Addr::UNSPECIFIED));
    let udp = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    udp.set_recv_buffer_size(args.rcvbuf)?;
    udp.bind(&SocketAddr::new(local_ip, args.udp_port).into())
        .map_err(|e| {
            io::Error::new(e.kind(), format!("binding UDP port {}: {e}", args.udp_port))
        })?;
    let udp: UdpSocket = udp.into();
    udp.set_read_timeout(Some(Duration::from_millis(50)))?;

    let running = Arc::new(AtomicBool::new(true));
    {
        let running = running.clone();
        ctrlc::set_handler(move || running.store(false, Ordering::Relaxed))
            .expect("Ctrl+C handler");
    }
    let dump_signal = Arc::new(AtomicBool::new(false));
    #[cfg(unix)]
    signal_hook::flag::register(signal_hook::consts::SIGUSR1, dump_signal.clone())?;

    let (tx, rx): (Sender<Msg>, Receiver<Msg>) = mpsc::channel();
    spawn_stdin_reader(tx.clone());

    let mut rec = Recorder {
        start: Instant::now(),
        window_us: args.window * 1_000_000,
        buf: VecDeque::new(),
        writers: Vec::new(),
    };
    println!(
        "recording the last {} s; press Enter or `pkill -USR1 qtm-probe` to dump to {}",
        args.window,
        args.out_dir.display()
    );

    let mut session: Option<TcpStream> = None;
    let mut retry_at = Instant::now();
    let mut frames = FrameTracker::new(1_000_000);
    let mut last_fresh: Option<u32> = None;
    let mut last_arrival_us: Option<u64> = None;
    let mut last_data = Instant::now();
    let mut stats = Stats::default();
    let mut stats_start = Instant::now();
    let mut pkt = [0u8; 4096];

    while running.load(Ordering::Relaxed) {
        if session.is_none() && Instant::now() >= retry_at {
            let mut events = Vec::new();
            match open_session(&args, &mut |e| events.push(e)) {
                Ok(tcp) => {
                    spawn_control_reader(tcp.try_clone()?, tx.clone())?;
                    session = Some(tcp);
                    last_data = Instant::now();
                }
                Err(e) => {
                    events.push(format!("session failed ({e}), retrying in 2 s"));
                    retry_at = Instant::now() + Duration::from_secs(2);
                }
            }
            for e in events {
                rec.event(e);
            }
        }

        while let Ok(msg) = rx.try_recv() {
            match msg {
                Msg::Event(e) => rec.event(e),
                Msg::SessionEnded(e) => {
                    rec.event(format!("qtm: session ended ({e}), reconnecting"));
                    session = None;
                    retry_at = Instant::now() + Duration::from_secs(1);
                }
                Msg::Dump => rec.dump(&args.out_dir),
            }
        }
        if dump_signal.swap(false, Ordering::Relaxed) {
            rec.dump(&args.out_dir);
        }

        match udp.recv(&mut pkt) {
            Ok(len) => {
                let (unix_us, mono_us) = rec.now();
                let Some(p) = qtmrt::parse_packet(&pkt[..len]) else {
                    continue;
                };
                if p.kind != PacketType::Data {
                    continue;
                }
                let Some(hdr) = qtmrt::data_header(p.payload) else {
                    continue;
                };
                last_data = Instant::now();
                let gap_us =
                    last_arrival_us.map(|prev| (mono_us - prev).min(u32::MAX as u64) as u32);
                last_arrival_us = Some(mono_us);

                let capture_us = frames.update(hdr, mono_us);
                let counts = frames.take_counts();
                let dup = capture_us.is_none();
                let step = if dup {
                    0
                } else {
                    last_fresh.map_or(0, |prev| hdr.frame.wrapping_sub(prev))
                };
                if !dup {
                    last_fresh = Some(hdr.frame);
                }
                let pose = (!dup)
                    .then(|| qtmrt::parse_6d(p.payload, index))
                    .flatten()
                    .map(|b| Pose {
                        pos_mm: b.pos,
                        quat_xyzw: frames::rotmat_to_quat_xyzw(b.rot),
                        valid: b.valid,
                    });
                let frame = Frame {
                    len: len.min(u16::MAX as usize) as u16,
                    frame: hdr.frame,
                    step,
                    missed: counts.missed,
                    dup,
                    qtm_us: hdr.timestamp_us,
                    age_us: capture_us
                        .map(|c| mono_us.saturating_sub(c).min(u32::MAX as u64) as u32),
                    gap_us,
                    pose,
                };
                stats.add(&frame);
                rec.push(unix_us, mono_us, What::Frame(frame));
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) => {}
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => rec.event(format!("udp recv error: {e}")),
        }

        // Nothing for a while (e.g. QTM not measuring): ask again, cheap and idempotent.
        if let Some(tcp) = session.as_mut()
            && last_data.elapsed() >= SILENCE
        {
            let silent_ms = last_data.elapsed().as_millis();
            rec.event(format!(
                "qtm: no frames for {silent_ms} ms, re-requesting stream"
            ));
            if let Err(e) = send_command(tcp, &stream_command(&args)) {
                rec.event(format!("qtm: re-request failed ({e}), reconnecting"));
                session = None;
            }
            last_data = Instant::now();
        }

        if args.status_every > 0 && stats_start.elapsed() >= Duration::from_secs(args.status_every)
        {
            let line = stats.line(stats_start.elapsed().as_secs_f64());
            println!("{} {line} | buffer {} rows", utc(unix_us()), rec.buf.len());
            stats = Stats::default();
            stats_start = Instant::now();
        }
    }

    if let Some(mut tcp) = session {
        let _ = send_command(&mut tcp, "StreamFrames Stop");
        let _ = tcp.shutdown(std::net::Shutdown::Both);
    }
    for w in rec.writers {
        let _ = w.join();
    }
    Ok(())
}
