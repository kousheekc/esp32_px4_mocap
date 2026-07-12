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
//   BRIDGE: client mode from the stored credentials and bridge Natnet to MAVLink
//
//   PORTAL: access point mode with a web form at http://192.168.4.1/ for user to set the settings
//
// In BOTH modes, holding BOOT (GPIO9) for 3 s erases settings and reboots back into the portal.

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{
    Config as NetConfig, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::Write as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::Async;
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{AuthenticationMethod, Config as WifiConfig, Interface, WifiController};
use firmware::store::FlashStore;
use firmware::{button, config, server};
use frames::{quat_to_euler_frd, transform_pose};
use log::{info, warn};
use mavbridge::{Encoder, MAX_FRAME_LEN};
use portal::Settings;
use static_cell::StaticCell;

extern crate alloc;

/// Latest pose from the RX task to the TX loop.
static POSE: Watch<CriticalSectionRawMutex, natnet::RigidBody, 1> = Watch::new();

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
                "bridge mode - rb{} on \"{}\", VPE @ {} Hz (HB @ {} Hz)",
                s.rigid_body_id, s.wifi_ssid, s.vpe_rate_hz, s.heartbeat_rate_hz
            );

            let (stack, runner) = embassy_net::new(
                interfaces.station,
                NetConfig::dhcpv4(Default::default()),
                resources,
                seed,
            );
            spawner.spawn(wifi_task(controller, s).expect("wifi task"));
            spawner.spawn(net_task(runner).expect("net task"));
            spawner.spawn(natnet_rx_task(stack, s).expect("natnet task"));

            let uart = Uart::new(
                peripherals.UART1,
                UartConfig::default().with_baudrate(s.mavlink_baud),
            )
            .expect("UART init failed")
            .with_tx(peripherals.GPIO21)
            .with_rx(peripherals.GPIO2)
            .into_async();

            run_bridge(s, uart).await
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

/// TX loop: fresh valid pose -> NED -> VPE over UART
async fn run_bridge(s: &'static Settings, mut uart: Uart<'static, Async>) -> ! {
    let mut encoder = Encoder::new(s.mav_sysid, s.mav_compid);
    let mut buf = [0u8; MAX_FRAME_LEN];
    let mut pose_rx = POSE.receiver().expect("pose receiver");

    let track_timeout = Duration::from_millis(u64::from(s.track_timeout_ms));
    let mut tracking_active = false;
    let mut ever_tracked = false;
    let mut last_valid = Instant::now();
    let mut reset_counter: u8 = 0;

    let ticks_per_heartbeat = u64::from(s.vpe_rate_hz / s.heartbeat_rate_hz);
    let mut ticker = Ticker::every(Duration::from_hz(u64::from(s.vpe_rate_hz)));
    let mut tick: u64 = 0;

    loop {
        ticker.next().await;

        if tick % ticks_per_heartbeat == 0 {
            let n = encoder.heartbeat(&mut buf);
            uart.write_all(&buf[..n]).await.expect("UART write failed");
        }
        tick += 1;

        let now = Instant::now();

        if let Some(rb) = pose_rx.try_changed().filter(|rb| rb.valid) {
            // Recovered after a tracking gap -> tell EKF2 to reset vision fusion.
            if !tracking_active {
                if ever_tracked {
                    reset_counter = reset_counter.wrapping_add(1);
                }
                tracking_active = true;
                info!("tracking acquired (reset_counter={reset_counter})");
            }
            ever_tracked = true;
            last_valid = now;

            let ned = transform_pose(rb.pos, rb.quat);
            let (roll, pitch, yaw) = quat_to_euler_frd(ned.q);
            let n = encoder.vision_position_estimate(
                &mut buf,
                now.as_micros(),
                ned.pos[0],
                ned.pos[1],
                ned.pos[2],
                roll,
                pitch,
                yaw,
                reset_counter,
            );
            uart.write_all(&buf[..n]).await.expect("UART write failed");
        } else if tracking_active && now.duration_since(last_valid) > track_timeout {
            // No fresh/valid pose for a while: stop sending, let EKF2 coast.
            tracking_active = false;
            info!("tracking lost (no valid pose for {} ms)", s.track_timeout_ms);
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
                let info = controller.wait_for_disconnect_async().await;
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
                    POSE.sender().send(rb);
                    last_pose = Some(rb);
                }
            }
            Err(e) => warn!("natnet: recv error {e:?}"),
        }

        let elapsed = window_start.elapsed();
        if elapsed >= Duration::from_secs(1) {
            let hz = pkts as f32 / (elapsed.as_millis() as f32 / 1000.0);
            match last_pose.take() {
                Some(rb) => info!(
                    "natnet: {pkts} pkts ({hz:.1} Hz, last {last_len} B) | rb{}: pos [{:.3} {:.3} {:.3}] quat [{:.3} {:.3} {:.3} {:.3}] valid={}",
                    rb.id, rb.pos[0], rb.pos[1], rb.pos[2],
                    rb.quat[0], rb.quat[1], rb.quat[2], rb.quat[3], rb.valid
                ),
                None => info!(
                    "natnet: {pkts} pkts ({hz:.1} Hz, last {last_len} B) | rb{} not seen",
                    s.rigid_body_id
                ),
            }
            pkts = 0;
            window_start = Instant::now();
        }
    }
}
