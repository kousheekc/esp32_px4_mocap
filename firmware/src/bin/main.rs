#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

// Two boot modes exist:
//
//   BRIDGE: client mode from the stored credentials and bridge NatNet or QTM to MAVLink
//
//   PORTAL: access point mode with a web form at http://192.168.4.1/ for user to set the settings
//
// In BOTH modes, holding BOOT (GPIO9) for 3 s erases settings and reboots back into the portal.

use core::net::Ipv4Addr;
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, select4, Either, Either3, Either4};
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{
    Config as NetConfig, IpAddress, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read as _, Write as _};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, RxConfig, Uart, UartRx, UartTx};
use esp_hal::Async;
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{AuthenticationMethod, Config as WifiConfig, Interface, WifiController};
use firmware::store::FlashStore;
use firmware::{button, config, server};
use frames::{quat_to_euler_frd, rotmat_to_quat_xyzw, transform_pose, Sample};
use log::{info, warn};
use mavbridge::{Decoder, Encoder, TimesyncRequest, MAX_FRAME_LEN};
use portal::{MocapSource, Settings};
use static_cell::StaticCell;

extern crate alloc;

/// Every parsed pose from the RX task to the TX loop, so WiFi bursts are forwarded, not collapsed.
static POSES: Channel<CriticalSectionRawMutex, Sample, 8> = Channel::new();
/// VPE frames written to the UART since the RX task's last stats line.
static VPE_SENT: AtomicU32 = AtomicU32::new(0);
/// Poses dropped because the TX loop fell behind and the queue was full.
static QUEUE_FULL: AtomicU32 = AtomicU32::new(0);
/// TIMESYNC requests from the flight controller, answered by the TX loop.
static TIMESYNC_REQS: Channel<CriticalSectionRawMutex, TimesyncRequest, 4> = Channel::new();
/// TIMESYNC replies sent since the last stats line.
static TIMESYNC_SENT: AtomicU32 = AtomicU32::new(0);
/// Largest capture-to-send age of a VPE since the last stats line (microseconds).
static MAX_AGE_US: AtomicU32 = AtomicU32::new(0);
/// UART RX errors (mostly FIFO overflows) since the last stats line.
static UART_RX_ERRORS: AtomicU32 = AtomicU32::new(0);
/// Latest station RSSI in dBm, sampled by the WiFi task; 0 while disconnected.
static RSSI_DBM: AtomicI32 = AtomicI32::new(0);

fn publish(sample: Sample) {
    if POSES.try_send(sample).is_err() {
        QUEUE_FULL.fetch_add(1, Ordering::Relaxed);
    }
}

/// TX-side counters for the once-per-second stats line; reading resets them.
struct TxStats {
    sent: u32,
    queue_full: u32,
    timesync: u32,
    max_age_ms: u32,
    uart_rx_errors: u32,
}

fn tx_stats() -> TxStats {
    TxStats {
        sent: VPE_SENT.swap(0, Ordering::Relaxed),
        queue_full: QUEUE_FULL.swap(0, Ordering::Relaxed),
        timesync: TIMESYNC_SENT.swap(0, Ordering::Relaxed),
        max_age_ms: MAX_AGE_US.swap(0, Ordering::Relaxed) / 1000,
        uart_rx_errors: UART_RX_ERRORS.swap(0, Ordering::Relaxed),
    }
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32c6 -o unstable-hal -o alloc -o wifi -o embassy -o log -o esp-backtrace -o stable-aarch64-apple-darwin

    esp_println::logger::init_logger_from_env();

    let hal_config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(hal_config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // Settings from flash
    let mut flash_store = FlashStore::new(peripherals.FLASH);
    let loaded = flash_store.load();
    static STORE: StaticCell<Mutex<CriticalSectionRawMutex, FlashStore>> = StaticCell::new();
    let store = STORE.init(Mutex::new(flash_store));

    // Reset settings task
    let boot_button = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );
    spawner.spawn(button::factory_reset_task(boot_button, store).expect("button task"));

    // XIAO ESP32-C6 RF switch: GPIO3 LOW enables it, GPIO14 picks the antenna (HIGH = U.FL).
    // Held for the life of main, which never returns.
    let _rf_switch_en = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    let ant_sel = if config::EXTERNAL_ANTENNA { Level::High } else { Level::Low };
    let _ant_sel = Output::new(peripherals.GPIO14, ant_sel, OutputConfig::default());
    info!(
        "antenna: {}",
        if config::EXTERNAL_ANTENNA { "external (U.FL)" } else { "onboard ceramic" }
    );

    // WiFi + network stack
    let (mut controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default())
        .expect("Failed to initialize Wi-Fi controller");

    let rng = Rng::new();
    let seed = ((rng.random() as u64) << 32) | rng.random() as u64;
    static RESOURCES: StaticCell<StackResources<6>> = StaticCell::new();
    let resources = RESOURCES.init(StackResources::new());

    match loaded {
        // BRIDGE mode
        Some(s) => {
            static SETTINGS: StaticCell<Settings> = StaticCell::new();
            let s: &'static Settings = SETTINGS.init(s);
            info!(
                "bridge mode - {:?} rb{} on \"{}\", VPE @ {} Hz (HB @ {} Hz)",
                s.source, s.rigid_body_id, s.wifi_ssid, s.vpe_rate_hz, s.heartbeat_rate_hz
            );

            let (stack, runner) = embassy_net::new(
                interfaces.station,
                NetConfig::dhcpv4(Default::default()),
                resources,
                seed,
            );
            spawner.spawn(wifi_task(controller, s).expect("wifi task"));
            spawner.spawn(net_task(runner).expect("net task"));
            match s.source {
                MocapSource::NatNet => spawner.spawn(natnet_rx_task(stack, s).expect("natnet task")),
                MocapSource::Qtm => spawner.spawn(qtm_rx_task(stack, s).expect("qtm task")),
            }

            let uart = Uart::new(
                peripherals.UART1,
                // The FC streams telemetry at us; wake the RX task well before the 128 B FIFO fills.
                UartConfig::default()
                    .with_baudrate(s.mavlink_baud)
                    .with_rx(RxConfig::default().with_fifo_full_threshold(32)),
            )
            .expect("UART init failed")
            .with_tx(peripherals.GPIO21)
            .with_rx(peripherals.GPIO2)
            .into_async();
            let (uart_rx, uart_tx) = uart.split();
            spawner.spawn(uart_rx_task(uart_rx).expect("uart rx task"));

            run_bridge(s, uart_tx).await
        }

        // PORTAL mode
        None => {
            info!(
                "portal mode - join WiFi \"{}\" (pass \"{}\"), open http://192.168.4.1/",
                config::AP_SSID,
                config::AP_PASS
            );

            let ap = WifiConfig::AccessPoint(
                AccessPointConfig::default()
                    .with_ssid(config::AP_SSID)
                    .with_password(config::AP_PASS.into())
                    .with_auth_method(AuthenticationMethod::Wpa2Personal)
                    .with_channel(config::AP_CHANNEL),
            );
            controller.set_config(&ap).expect("AP config failed");

            let portal_ip = Ipv4Addr::from(config::PORTAL_IP);
            let net_config = NetConfig::ipv4_static(StaticConfigV4 {
                address: Ipv4Cidr::new(portal_ip, 24),
                gateway: Some(portal_ip),
                dns_servers: heapless::Vec::from_slice(&[portal_ip]).unwrap(),
            });
            let (stack, runner) =
                embassy_net::new(interfaces.access_point, net_config, resources, seed);
            spawner.spawn(net_task(runner).expect("net task"));
            server::spawn(spawner, stack, store);

            loop {
                Timer::after(Duration::from_secs(3600)).await;
            }
        }
    }
}

/// TX loop: each valid pose -> NED -> VPE over UART as soon as it arrives, plus heartbeat and
/// TIMESYNC replies
async fn run_bridge(s: &'static Settings, mut uart: UartTx<'static, Async>) -> ! {
    let mut encoder = Encoder::new(s.mav_sysid, s.mav_compid);
    let mut buf = [0u8; MAX_FRAME_LEN];

    let track_timeout = Duration::from_millis(u64::from(s.track_timeout_ms));
    let mut tracking_active = false;
    let mut ever_tracked = false;
    let mut last_valid = Instant::now();
    let mut reset_counter: u8 = 0;

    let mut heartbeat = Ticker::every(Duration::from_hz(u64::from(s.heartbeat_rate_hz)));

    loop {
        let tracking_deadline = async {
            if tracking_active {
                Timer::at(last_valid + track_timeout).await
            } else {
                core::future::pending::<()>().await
            }
        };

        // TIMESYNC first: PX4 only uses replies that come back within 10 ms.
        match select4(
            TIMESYNC_REQS.receive(),
            POSES.receive(),
            heartbeat.next(),
            tracking_deadline,
        )
        .await
        {
            Either4::First(req) => {
                // Stamp at send time so the delay before sending isn't counted as clock offset.
                let now_ns = Instant::now().as_micros() as i64 * 1000;
                let n = encoder.timesync_reply(&mut buf, now_ns, &req);
                uart.write_all(&buf[..n]).await.expect("UART write failed");
                TIMESYNC_SENT.fetch_add(1, Ordering::Relaxed);
            }
            Either4::Second(rb) => {
                // Invalid poses don't refresh last_valid, so the deadline still fires.
                if !rb.valid {
                    continue;
                }
                // Recovered after a tracking gap -> tell EKF2 to reset vision fusion.
                if !tracking_active {
                    if ever_tracked {
                        reset_counter = reset_counter.wrapping_add(1);
                    }
                    tracking_active = true;
                    info!("tracking acquired (reset_counter={reset_counter})");
                }
                ever_tracked = true;
                let now = Instant::now();
                last_valid = now;

                let ned = transform_pose(rb.pos, rb.quat_xyzw);
                let (roll, pitch, yaw) = quat_to_euler_frd(ned.q);
                // Capture time, so PX4 (once timesynced) isn't fooled by WiFi delivering late.
                let age_us = now.as_micros().saturating_sub(rb.capture_us);
                MAX_AGE_US.fetch_max(age_us.min(u64::from(u32::MAX)) as u32, Ordering::Relaxed);
                let n = encoder.vision_position_estimate(
                    &mut buf,
                    rb.capture_us,
                    ned.pos[0],
                    ned.pos[1],
                    ned.pos[2],
                    roll,
                    pitch,
                    yaw,
                    reset_counter,
                );
                uart.write_all(&buf[..n]).await.expect("UART write failed");
                VPE_SENT.fetch_add(1, Ordering::Relaxed);
            }
            Either4::Third(()) => {
                let n = encoder.heartbeat(&mut buf);
                uart.write_all(&buf[..n]).await.expect("UART write failed");
            }
            Either4::Fourth(()) => {
                // No valid pose for a while: stop sending, let EKF2 coast.
                tracking_active = false;
                info!("tracking lost (no valid pose for {} ms)", s.track_timeout_ms);
            }
        }
    }
}

/// UART RX: pick TIMESYNC requests out of the flight controller's MAVLink stream so PX4 can
/// learn our clock and use the capture times we put in VISION_POSITION_ESTIMATE.
#[embassy_executor::task]
async fn uart_rx_task(mut uart: UartRx<'static, Async>) -> ! {
    let mut decoder = Decoder::new();
    let mut buf = [0u8; 128];
    loop {
        match uart.read_async(&mut buf).await {
            Ok(n) => {
                for &b in &buf[..n] {
                    if let Some(req) = decoder.push(b) {
                        // Full queue means the TX loop is stalled; a missed request is harmless.
                        let _ = TIMESYNC_REQS.try_send(req);
                    }
                }
            }
            // Counted, not logged: printing each one stalls the executor and causes more.
            Err(_) => {
                UART_RX_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>, s: &'static Settings) -> ! {
    let mut sta = StationConfig::default()
        .with_ssid(s.wifi_ssid.as_str())
        .with_password(s.wifi_pass.as_str().into());
    if s.wifi_pass.is_empty() {
        sta = sta.with_auth_method(AuthenticationMethod::None);
    }
    controller
        .set_config(&WifiConfig::Station(sta))
        .expect("wifi set_config failed");

    loop {
        info!("wifi: connecting to {:?}...", s.wifi_ssid.as_str());
        match controller.connect_async().await {
            Ok(_) => {
                info!("wifi: connected");
                // Sample RSSI for the stats lines until the link drops.
                let sample_rssi = async {
                    let mut ticker = Ticker::every(Duration::from_secs(1));
                    loop {
                        if let Ok(rssi) = controller.rssi() {
                            RSSI_DBM.store(rssi, Ordering::Relaxed);
                        }
                        ticker.next().await;
                    }
                };
                let info = match select(controller.wait_for_disconnect_async(), sample_rssi).await {
                    Either::First(info) => info,
                    Either::Second(never) => never,
                };
                RSSI_DBM.store(0, Ordering::Relaxed);
                warn!("wifi: disconnected ({info:?}), reconnecting");
            }
            Err(e) => {
                warn!(
                    "wifi: connect failed ({e:?}), retrying in 1 s — hold BOOT 3 s to re-enter setup"
                );
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) -> ! {
    runner.run().await
}

/// RX loop: join multicast group and publish every parsed target pose into the POSE watch
#[embassy_executor::task]
async fn natnet_rx_task(stack: Stack<'static>, s: &'static Settings) -> ! {
    let version = natnet::Version::new(s.natnet_major, s.natnet_minor);

    stack.wait_config_up().await;
    info!("net: up, config {:?}", stack.config_v4());

    let group = Ipv4Addr::from(s.multicast_addr);
    stack
        .join_multicast_group(group)
        .expect("multicast join failed");
    info!("net: joined multicast group {group}:{}", s.data_port);

    let mut rx_meta = [PacketMetadata::EMPTY; 8];
    let mut rx_buf = [0u8; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buf = [0u8; 32];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(s.data_port).expect("UDP bind failed");

    let mut pkt = [0u8; 2048];
    let mut window_start = Instant::now();
    let mut pkts: u32 = 0;
    let mut last_len = 0usize;
    let mut last_pose: Option<natnet::RigidBody> = None;

    loop {
        match socket.recv_from(&mut pkt).await {
            Ok((len, _meta)) => {
                pkts += 1;
                last_len = len;
                if let Some(rb) = natnet::parse_frame(&pkt[..len], version, s.rigid_body_id) {
                    publish(Sample {
                        pos: rb.pos,
                        quat_xyzw: rb.quat,
                        valid: rb.valid,
                        // NatNet capture times aren't mapped yet: stamp on arrival.
                        capture_us: Instant::now().as_micros(),
                    });
                    last_pose = Some(rb);
                }
            }
            Err(e) => warn!("natnet: recv error {e:?}"),
        }

        let elapsed = window_start.elapsed();
        if elapsed >= Duration::from_secs(1) {
            let hz = pkts as f32 / (elapsed.as_millis() as f32 / 1000.0);
            let TxStats { sent, queue_full, timesync, uart_rx_errors, .. } = tx_stats();
            let rssi = RSSI_DBM.load(Ordering::Relaxed);
            match last_pose.take() {
                Some(rb) => info!(
                    "natnet: {pkts} pkts ({hz:.1} Hz, last {last_len} B, rssi {rssi} dBm) | sent {sent}, queue full {queue_full}, timesync {timesync}, uart rx err {uart_rx_errors} | rb{}: pos [{:.3} {:.3} {:.3}] quat [{:.3} {:.3} {:.3} {:.3}] valid={}",
                    rb.id, rb.pos[0], rb.pos[1], rb.pos[2],
                    rb.quat[0], rb.quat[1], rb.quat[2], rb.quat[3], rb.valid
                ),
                None => info!(
                    "natnet: {pkts} pkts ({hz:.1} Hz, last {last_len} B, rssi {rssi} dBm) | sent {sent}, queue full {queue_full}, timesync {timesync}, uart rx err {uart_rx_errors} | rb{} not seen",
                    s.rigid_body_id
                ),
            }
            pkts = 0;
            window_start = Instant::now();
        }
    }
}

/// RX loop: open a QTM RT session over TCP, have QTM stream 6D frames to our UDP port, and
/// publish the target body into the POSE watch. Reconnects whenever the TCP session drops.
#[embassy_executor::task]
async fn qtm_rx_task(stack: Stack<'static>, s: &'static Settings) -> ! {
    let host = (IpAddress::Ipv4(Ipv4Addr::from(s.qtm_host)), s.qtm_port);
    let index = (s.rigid_body_id - 1) as usize;

    stack.wait_config_up().await;
    info!("net: up, config {:?}", stack.config_v4());

    let mut rx_meta = [PacketMetadata::EMPTY; 8];
    let mut rx_buf = [0u8; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buf = [0u8; 32];
    let mut udp = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    udp.bind(config::QTM_UDP_PORT).expect("UDP bind failed");

    let mut tcp_rx = [0u8; 1024];
    let mut tcp_tx = [0u8; 256];

    loop {
        let mut tcp = TcpSocket::new(stack, &mut tcp_rx, &mut tcp_tx);
        // TCP is idle while frames flow over UDP, so keep-alives stop the timeout firing.
        tcp.set_keep_alive(Some(Duration::from_secs(2)));
        tcp.set_timeout(Some(Duration::from_secs(6)));

        info!("qtm: connecting to {}:{}...", Ipv4Addr::from(s.qtm_host), s.qtm_port);
        match tcp.connect(host).await {
            Ok(()) => {
                let e = qtm_session(&mut tcp, &udp, s, index).await;
                warn!("qtm: session ended ({e}), reconnecting");
            }
            Err(e) => warn!("qtm: connect failed ({e:?}), retrying"),
        }
        tcp.abort();
        let _ = tcp.flush().await;
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Runs one QTM session until the TCP connection fails; returns why
async fn qtm_session(
    tcp: &mut TcpSocket<'_>,
    udp: &UdpSocket<'_>,
    s: &Settings,
    index: usize,
) -> &'static str {
    let mut ctl = [0u8; 512];

    // Greeting, then pin the protocol version so the packet layout is known.
    match qtm_read(tcp, &mut ctl).await {
        Ok(p) => info!("qtm: {}", p.text().unwrap_or("?")),
        Err(e) => return e,
    }
    if let Err(e) = qtm_command(tcp, "Version 1.19").await {
        return e;
    }
    match qtm_read(tcp, &mut ctl).await {
        Ok(p) if p.kind == qtmrt::PacketType::Error => {
            warn!("qtm: version rejected: {}", p.text().unwrap_or("?"));
            return "version rejected";
        }
        Ok(p) => info!("qtm: {}", p.text().unwrap_or("?")),
        Err(e) => return e,
    }

    let mut stream_cmd: heapless::String<64> = heapless::String::new();
    let _ = core::fmt::Write::write_fmt(
        &mut stream_cmd,
        format_args!(
            "StreamFrames Frequency:{} UDP:{} 6D",
            s.vpe_rate_hz,
            config::QTM_UDP_PORT
        ),
    );
    if let Err(e) = qtm_command(tcp, &stream_cmd).await {
        return e;
    }
    info!("qtm: requested \"{stream_cmd}\"");

    let silence = Duration::from_millis(config::QTM_SILENCE_MS);
    let mut last_data = Instant::now();
    let mut pkt = [0u8; 2048];
    let mut window_start = Instant::now();
    let mut pkts: u32 = 0;
    let mut last_len = 0usize;
    let mut last_pose: Option<qtmrt::Body> = None;
    let mut frames = qtmrt::FrameTracker::new(1_000_000);

    loop {
        match select3(
            tcp.wait_read_ready(),
            udp.recv_from(&mut pkt),
            Timer::at(last_data + silence),
        )
        .await
        {
            // Control channel: errors, events, or EOF when QTM goes away.
            Either3::First(()) => match qtm_read(tcp, &mut ctl).await {
                Ok(p) => match p.kind {
                    qtmrt::PacketType::Error => warn!("qtm: error: {}", p.text().unwrap_or("?")),
                    qtmrt::PacketType::Event => info!("qtm: event {:?}", p.payload.first()),
                    kind => info!("qtm: {kind:?} packet ({} B)", p.payload.len()),
                },
                Err(e) => return e,
            },
            Either3::Second(Ok((len, _meta))) => {
                let arrival = Instant::now();
                last_data = arrival;
                pkts += 1;
                last_len = len;
                let Some(p) = qtmrt::parse_packet(&pkt[..len]) else {
                    continue;
                };
                if p.kind != qtmrt::PacketType::Data {
                    continue;
                }
                let Some(hdr) = qtmrt::data_header(p.payload) else {
                    continue;
                };
                // Duplicates fall through so the stats line still prints.
                if let Some(capture_us) = frames.update(hdr, arrival.as_micros())
                    && let Some(rb) = qtmrt::parse_6d(p.payload, index)
                {
                    publish(Sample {
                        pos: rb.pos.map(|v| v * 1e-3),
                        quat_xyzw: rotmat_to_quat_xyzw(rb.rot),
                        valid: rb.valid,
                        capture_us,
                    });
                    last_pose = Some(rb);
                }
            }
            Either3::Second(Err(e)) => warn!("qtm: recv error {e:?}"),
            // Nothing for a while (e.g. QTM not measuring): ask again, cheap and idempotent.
            Either3::Third(()) => {
                info!("qtm: no frames for {} ms, re-requesting stream", config::QTM_SILENCE_MS);
                if let Err(e) = qtm_command(tcp, &stream_cmd).await {
                    return e;
                }
                last_data = Instant::now();
            }
        }

        let elapsed = window_start.elapsed();
        if elapsed >= Duration::from_secs(1) {
            let hz = pkts as f32 / (elapsed.as_millis() as f32 / 1000.0);
            let TxStats { sent, queue_full, timesync, max_age_ms, uart_rx_errors } = tx_stats();
            let rssi = RSSI_DBM.load(Ordering::Relaxed);
            let step = frames.frame_step();
            let qtmrt::FrameCounts { missed, dup } = frames.take_counts();
            match last_pose.take() {
                Some(rb) => info!(
                    "qtm: {pkts} pkts ({hz:.1} Hz, last {last_len} B, rssi {rssi} dBm) | missed {missed} (frame step {step}), dup {dup} | sent {sent}, queue full {queue_full}, age max {max_age_ms} ms | timesync {timesync}, uart rx err {uart_rx_errors} | body {}: pos [{:.1} {:.1} {:.1}] mm valid={}",
                    s.rigid_body_id, rb.pos[0], rb.pos[1], rb.pos[2], rb.valid
                ),
                None => info!(
                    "qtm: {pkts} pkts ({hz:.1} Hz, last {last_len} B, rssi {rssi} dBm) | missed {missed} (frame step {step}), dup {dup} | sent {sent}, queue full {queue_full}, age max {max_age_ms} ms | timesync {timesync}, uart rx err {uart_rx_errors} | body {} not seen",
                    s.rigid_body_id
                ),
            }
            pkts = 0;
            window_start = Instant::now();
        }
    }
}

async fn qtm_command(tcp: &mut TcpSocket<'_>, cmd: &str) -> Result<(), &'static str> {
    let mut buf = [0u8; 80];
    let n = qtmrt::encode_command(cmd, &mut buf).ok_or("command too long")?;
    tcp.write_all(&buf[..n]).await.map_err(|_| "TCP write failed")
}

/// Read one whole control packet; packets larger than `buf` end the session
async fn qtm_read<'b>(
    tcp: &mut TcpSocket<'_>,
    buf: &'b mut [u8],
) -> Result<qtmrt::Packet<'b>, &'static str> {
    tcp.read_exact(&mut buf[..qtmrt::HEADER_LEN])
        .await
        .map_err(|_| "TCP closed")?;
    let len = qtmrt::packet_len(buf).ok_or("bad packet header")?;
    let body = buf.get_mut(qtmrt::HEADER_LEN..len).ok_or("control packet too large")?;
    tcp.read_exact(body).await.map_err(|_| "TCP closed")?;
    qtmrt::parse_packet(buf).ok_or("bad packet")
}
