//! Settings portal (thanks Claude)

use core::fmt::Write as _;
use core::net::{Ipv4Addr, SocketAddr, SocketAddrV4};

use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_io_async::Write as _;
use log::{info, warn};
use static_cell::StaticCell;

use crate::config;
use crate::store::FlashStore;


#[embassy_executor::task]
pub async fn dhcp_server_task(stack: Stack<'static>) -> ! {
    static BUFFERS: StaticCell<edge_nal_embassy::UdpBuffers<1, 1600, 1600, 2>> = StaticCell::new();
    let udp = edge_nal_embassy::Udp::new(stack, BUFFERS.init(edge_nal_embassy::UdpBuffers::new()));

    let ip = Ipv4Addr::from(config::PORTAL_IP);
    let mut pkt_buf = [0u8; 1600];

    loop {
        let mut socket = match edge_nal::UdpBind::bind(
            &udp,
            SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 67)),
        )
        .await
        {
            Ok(s) => s,
            Err(e) => {
                warn!("dhcp: bind failed ({e:?}), retrying");
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };

        let mut gw_buf = [Ipv4Addr::UNSPECIFIED; 1];
        let mut options = edge_dhcp::server::ServerOptions::new(ip, Some(&mut gw_buf));
        let dns = [ip];
        options.dns = &dns;

        let mut server = edge_dhcp::server::Server::<_, 8>::new(
            || embassy_time::Instant::now().as_secs(),
            ip,
        );

        if let Err(e) =
            edge_dhcp::io::server::run(&mut server, &options, &mut socket, &mut pkt_buf).await
        {
            warn!("dhcp: server error ({e:?}), restarting");
        }
    }
}

#[embassy_executor::task]
pub async fn captive_dns_task(stack: Stack<'static>) -> ! {
    static BUFFERS: StaticCell<edge_nal_embassy::UdpBuffers<1, 1024, 1024, 2>> = StaticCell::new();
    let udp = edge_nal_embassy::Udp::new(stack, BUFFERS.init(edge_nal_embassy::UdpBuffers::new()));

    let ip = Ipv4Addr::from(config::PORTAL_IP);
    let mut tx_buf = [0u8; 1024];
    let mut rx_buf = [0u8; 1024];

    loop {
        if let Err(e) = edge_captive::io::run(
            &udp,
            SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 53)),
            &mut tx_buf,
            &mut rx_buf,
            ip,
            core::time::Duration::from_secs(60),
        )
        .await
        {
            warn!("dns: server error ({e:?}), restarting");
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}

const HTTP_HANDLERS: usize = 2;

struct HttpBufs {
    rx: [u8; 1024],
    tx: [u8; 1024],
    req: [u8; 2048],
    page: heapless::String<4096>,
}

impl HttpBufs {
    const fn new() -> Self {
        Self {
            rx: [0; 1024],
            tx: [0; 1024],
            req: [0; 2048],
            page: heapless::String::new(),
        }
    }
}

pub fn spawn(
    spawner: embassy_executor::Spawner,
    stack: Stack<'static>,
    store: &'static Mutex<CriticalSectionRawMutex, FlashStore>,
) {
    static HTTP_BUFS: static_cell::ConstStaticCell<[HttpBufs; HTTP_HANDLERS]> =
        static_cell::ConstStaticCell::new([HttpBufs::new(), HttpBufs::new()]);

    spawner.spawn(dhcp_server_task(stack).expect("dhcp task"));
    spawner.spawn(captive_dns_task(stack).expect("dns task"));
    for bufs in HTTP_BUFS.take() {
        spawner.spawn(http_task(stack, store, bufs).expect("http task"));
    }

    info!(
        "portal: http://{}.{}.{}.{}/ (join WiFi \"{}\" first)",
        config::PORTAL_IP[0],
        config::PORTAL_IP[1],
        config::PORTAL_IP[2],
        config::PORTAL_IP[3],
        config::AP_SSID
    );
}

#[embassy_executor::task(pool_size = HTTP_HANDLERS)]
async fn http_task(
    stack: Stack<'static>,
    store: &'static Mutex<CriticalSectionRawMutex, FlashStore>,
    bufs: &'static mut HttpBufs,
) -> ! {
    loop {
        let mut socket = TcpSocket::new(stack, &mut bufs.rx[..], &mut bufs.tx[..]);

        socket.set_timeout(Some(Duration::from_secs(5)));

        if let Err(e) = socket.accept(config::HTTP_PORT).await {
            warn!("http: accept failed ({e:?})");
            continue;
        }

        let reboot =
            match handle_connection(&mut socket, &mut bufs.req, &mut bufs.page, store).await {
                Ok(reboot) => reboot,
                Err(e) => {
                    info!("http: connection dropped ({e})");
                    false
                }
            };

        socket.close();
        let _ = socket.flush().await;

        if reboot {
            info!("portal: settings saved — rebooting into bridge mode");
            Timer::after(Duration::from_secs(1)).await;
            esp_hal::system::software_reset();
        }
    }
}

async fn handle_connection(
    socket: &mut TcpSocket<'_>,
    req_buf: &mut [u8],
    page: &mut heapless::String<4096>,
    store: &'static Mutex<CriticalSectionRawMutex, FlashStore>,
) -> Result<bool, &'static str> {
    let (method, path, body) = read_request(socket, req_buf).await?;

    match (method, path) {
        ("GET", "/") => {
            let current = store.lock().await.load().unwrap_or_default();
            page.clear();
            portal::html::render_page(&current, None, page).map_err(|_| "page too large")?;
            write_response(socket, "200 OK", page.as_str()).await?;
            Ok(false)
        }
        ("POST", "/save") => {
            let mut new = store.lock().await.load().unwrap_or_default();
            let result = match portal::form::apply_form(&mut new, body) {
                Ok(()) => store.lock().await.save(&new),
                Err(e) => Err(e),
            };
            match result {
                Ok(()) => {
                    page.clear();
                    portal::html::render_reboot_page(page).map_err(|_| "page too large")?;
                    write_response(socket, "200 OK", page.as_str()).await?;
                    Ok(true)
                }
                Err(e) => {
                    warn!("portal: rejected settings ({e})");
                    page.clear();
                    portal::html::render_page(&new, Some(e), page)
                        .map_err(|_| "page too large")?;
                    write_response(socket, "200 OK", page.as_str()).await?;
                    Ok(false)
                }
            }
        }
        ("GET", _) => {
            write_redirect(socket).await?;
            Ok(false)
        }
        _ => {
            write_response(socket, "405 Method Not Allowed", "").await?;
            Ok(false)
        }
    }
}

async fn read_request<'b>(
    socket: &mut TcpSocket<'_>,
    buf: &'b mut [u8],
) -> Result<(&'b str, &'b str, &'b [u8]), &'static str> {
    let mut filled = 0;

    let header_end = loop {
        if filled == buf.len() {
            return Err("request too large");
        }
        let n = socket
            .read(&mut buf[filled..])
            .await
            .map_err(|_| "socket read failed")?;
        if n == 0 {
            return Err("connection closed mid-request");
        }
        filled += n;
        if let Some(pos) = find(&buf[..filled], b"\r\n\r\n") {
            break pos + 4;
        }
    };

    let headers = core::str::from_utf8(&buf[..header_end]).map_err(|_| "headers not UTF-8")?;
    let mut first_line = headers.lines().next().ok_or("empty request")?.split(' ');
    let method_len = first_line.next().ok_or("no method")?.len();
    let path_len = first_line.next().ok_or("no path")?.len();

    let content_length = headers
        .lines()
        .find_map(|l| {
            let (name, value) = l.split_once(':')?;
            name.eq_ignore_ascii_case("content-length")
                .then(|| value.trim().parse::<usize>().ok())?
        })
        .unwrap_or(0);
    if header_end + content_length > buf.len() {
        return Err("request body too large");
    }
    while filled < header_end + content_length {
        let n = socket
            .read(&mut buf[filled..])
            .await
            .map_err(|_| "socket read failed")?;
        if n == 0 {
            return Err("connection closed mid-body");
        }
        filled += n;
    }

    let method = core::str::from_utf8(&buf[..method_len]).unwrap();
    let path =
        core::str::from_utf8(&buf[method_len + 1..method_len + 1 + path_len]).unwrap();
    let body = &buf[header_end..header_end + content_length];
    Ok((method, path, body))
}

async fn write_response(
    socket: &mut TcpSocket<'_>,
    status: &str,
    body: &str,
) -> Result<(), &'static str> {
    let mut header: heapless::String<128> = heapless::String::new();
    write!(
        header,
        "HTTP/1.1 {status}\r\nContent-Type: text/html\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    )
    .map_err(|_| "header too large")?;
    socket
        .write_all(header.as_bytes())
        .await
        .map_err(|_| "socket write failed")?;
    socket
        .write_all(body.as_bytes())
        .await
        .map_err(|_| "socket write failed")
}

async fn write_redirect(socket: &mut TcpSocket<'_>) -> Result<(), &'static str> {
    let mut response: heapless::String<128> = heapless::String::new();
    write!(
        response,
        "HTTP/1.1 302 Found\r\nLocation: http://{}.{}.{}.{}/\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
        config::PORTAL_IP[0], config::PORTAL_IP[1], config::PORTAL_IP[2], config::PORTAL_IP[3]
    )
    .map_err(|_| "header too large")?;
    socket
        .write_all(response.as_bytes())
        .await
        .map_err(|_| "socket write failed")
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}
