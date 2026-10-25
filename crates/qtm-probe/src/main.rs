//! Measure how QTM 6D frames arrive at this machine, using the bridge firmware's own `qtmrt` code.
//!
//! Opens a QTM RT session over TCP, asks QTM to stream 6D frames to a local UDP port and prints a
//! once-per-second stats line in the bridge's format, then a summary on Ctrl+C. Run it once over a
//! cable and once over WiFi to see whether drops and stalls come from QTM or from the WiFi link.
//!
//!     cargo run --release -p qtm-probe -- 192.168.3.92
//!     cargo run --release -p qtm-probe -- 192.168.3.92 --bind 192.168.3.50   # force an interface

use std::io::{self, Read, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpStream, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use qtmrt::{FrameCounts, FrameTracker, PacketType};
use socket2::{Domain, Protocol, Socket, Type};

#[derive(Parser)]
#[command(about = "Measure how QTM 6D frames arrive at this machine")]
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
    qtmrt::Packet { kind: PacketType::Command, payload }.text().unwrap_or("?").to_owned()
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[((p / 100.0 * sorted.len() as f64) as usize).min(sorted.len() - 1)]
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    let index = args.body.checked_sub(1).expect("--body is 1-based");
    let local_ip = args.bind.unwrap_or(IpAddr::V4(Ipv4Addr::UNSPECIFIED));

    let udp = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    udp.set_recv_buffer_size(args.rcvbuf)?;
    udp.bind(&SocketAddr::new(local_ip, args.udp_port).into())?;
    let udp: UdpSocket = udp.into();
    udp.set_read_timeout(Some(Duration::from_millis(200)))?;

    // QTM streams UDP back to the address the TCP session comes from.
    let tcp = Socket::new(Domain::IPV4, Type::STREAM, Some(Protocol::TCP))?;
    if let Some(ip) = args.bind {
        tcp.bind(&SocketAddr::new(ip, 0).into())?;
    }
    tcp.connect_timeout(&SocketAddr::new(args.host, args.port).into(), Duration::from_secs(5))?;
    let mut tcp: TcpStream = tcp.into();
    tcp.set_read_timeout(Some(Duration::from_secs(5)))?;
    println!("connected from {} to {}:{}", tcp.local_addr()?.ip(), args.host, args.port);

    println!("qtm: {}", text(&read_packet(&mut tcp)?.1));
    send_command(&mut tcp, "Version 1.19")?;
    let (kind, payload) = read_packet(&mut tcp)?;
    if kind == PacketType::Error {
        eprintln!("version rejected: {}", text(&payload));
        std::process::exit(1);
    }
    println!("qtm: {}", text(&payload));
    let stream_cmd = format!("StreamFrames Frequency:{} UDP:{} 6D", args.rate, args.udp_port);
    send_command(&mut tcp, &stream_cmd)?;
    println!("requested \"{stream_cmd}\"");

    // Control channel: errors, events, or EOF when QTM goes away.
    let running = Arc::new(AtomicBool::new(true));
    {
        let mut tcp = tcp.try_clone()?;
        tcp.set_read_timeout(None)?;
        let running = running.clone();
        thread::spawn(move || {
            loop {
                match read_packet(&mut tcp) {
                    Ok((PacketType::Error, p)) => println!("qtm: error: {}", text(&p)),
                    Ok((PacketType::Event, p)) => println!("qtm: event {:?}", p.first()),
                    Ok((kind, p)) => println!("qtm: {kind:?} packet ({} B)", p.len()),
                    Err(e) => {
                        if running.swap(false, Ordering::Relaxed) {
                            println!("qtm: session ended ({e})");
                        }
                        return;
                    }
                }
            }
        });
    }
    {
        let running = running.clone();
        ctrlc::set_handler(move || running.store(false, Ordering::Relaxed))
            .expect("Ctrl+C handler");
    }

    let start = Instant::now();
    let mut frames = FrameTracker::new(1_000_000);
    let mut pkt = [0u8; 4096];
    let mut window_start = start;
    let mut pkts: u32 = 0;
    let mut last_arrival: Option<Instant> = None;
    let mut max_gap_ms = 0f64;
    let mut max_age_ms = 0f64;
    let mut last_pose: Option<qtmrt::Body> = None;
    let mut total_pkts: u64 = 0;
    let mut total = FrameCounts::default();
    let mut gaps_ms = Vec::new();
    let mut ages_ms = Vec::new();

    while running.load(Ordering::Relaxed) {
        match udp.recv(&mut pkt) {
            Ok(len) => {
                let arrival = Instant::now();
                let Some(p) = qtmrt::parse_packet(&pkt[..len]) else {
                    continue;
                };
                if p.kind != PacketType::Data {
                    continue;
                }
                let Some(hdr) = qtmrt::data_header(p.payload) else {
                    continue;
                };
                pkts += 1;
                if let Some(prev) = last_arrival {
                    let gap = (arrival - prev).as_secs_f64() * 1e3;
                    gaps_ms.push(gap);
                    max_gap_ms = max_gap_ms.max(gap);
                }
                last_arrival = Some(arrival);

                let arrival_us = (arrival - start).as_micros() as u64;
                if let Some(capture_us) = frames.update(hdr, arrival_us) {
                    let age = arrival_us.saturating_sub(capture_us) as f64 / 1e3;
                    ages_ms.push(age);
                    max_age_ms = max_age_ms.max(age);
                    if let Some(rb) = qtmrt::parse_6d(p.payload, index) {
                        last_pose = Some(rb);
                    }
                }
            }
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {}
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }

        let elapsed = window_start.elapsed();
        if elapsed >= Duration::from_secs(1) {
            let hz = pkts as f64 / elapsed.as_secs_f64();
            let step = frames.frame_step();
            let FrameCounts { missed, dup } = frames.take_counts();
            let pose = match last_pose.take() {
                Some(rb) => format!(
                    "body {}: pos [{:.1} {:.1} {:.1}] mm valid={}",
                    args.body, rb.pos[0], rb.pos[1], rb.pos[2], rb.valid
                ),
                None => format!("body {} not seen", args.body),
            };
            println!(
                "qtm: {pkts} pkts ({hz:.1} Hz) | missed {missed} (frame step {step}), dup {dup} | gap max {max_gap_ms:.0} ms, age max {max_age_ms:.0} ms | {pose}"
            );
            total_pkts += u64::from(pkts);
            total.missed += missed;
            total.dup += dup;
            pkts = 0;
            max_gap_ms = 0.0;
            max_age_ms = 0.0;
            window_start = Instant::now();
        }
    }

    let _ = send_command(&mut tcp, "StreamFrames Stop");
    let _ = tcp.shutdown(std::net::Shutdown::Both);

    let FrameCounts { missed, dup } = frames.take_counts();
    total_pkts += u64::from(pkts);
    total.missed += missed;
    total.dup += dup;
    let elapsed = start.elapsed().as_secs_f64();
    let expected = total_pkts + u64::from(total.missed);
    gaps_ms.sort_by(f64::total_cmp);
    ages_ms.sort_by(f64::total_cmp);
    println!("\n--- {elapsed:.1} s ---");
    println!(
        "received {total_pkts} ({:.1} Hz), missed {} ({:.1}%), dup {}",
        total_pkts as f64 / elapsed,
        total.missed,
        if expected > 0 { 100.0 * total.missed as f64 / expected as f64 } else { 0.0 },
        total.dup
    );
    println!("QTM frame rate seen: {:.1} frames/s (received + missed)", expected as f64 / elapsed);
    for (name, vals) in [("gap", &gaps_ms), ("age", &ages_ms)] {
        println!(
            "{name} ms: p50 {:.1}  p95 {:.1}  p99 {:.1}  max {:.1}",
            percentile(vals, 50.0),
            percentile(vals, 95.0),
            percentile(vals, 99.0),
            vals.last().copied().unwrap_or(0.0)
        );
    }
    Ok(())
}
