//! urlencoded form parsing for the portal form: bytes in, `Settings` out

use crate::Settings;
use heapless::{String, Vec};

pub const FIELD_KEYS: &[&str] = &[
    "ssid", "pass", "mcast", "port", "nn_major", "nn_minor", "rbid", "sysid", "compid", "baud",
    "vpe_hz", "hb_hz", "timeout_ms",
];

pub fn apply_form(s: &mut Settings, body: &[u8]) -> Result<(), &'static str> {
    for pair in body.split(|&b| b == b'&') {
        if pair.is_empty() {
            continue;
        }
        let eq = pair
            .iter()
            .position(|&b| b == b'=')
            .ok_or("malformed form field (no '=')")?;
        let key: Vec<u8, 16> = percent_decode(&pair[..eq])?;
        let value: Vec<u8, 64> = percent_decode(&pair[eq + 1..])?;
        let key = core::str::from_utf8(&key).map_err(|_| "form key is not UTF-8")?;
        let value = core::str::from_utf8(&value).map_err(|_| "form value is not UTF-8")?;

        match key {
            "ssid" => s.wifi_ssid = string_field(value, "SSID too long (max 32)")?,
            "pass" => s.wifi_pass = string_field(value, "password too long (max 64)")?,
            "mcast" => s.multicast_addr = parse_dotted_quad(value)?,
            "port" => s.data_port = int_field(value, "data port")?,
            "nn_major" => s.natnet_major = int_field(value, "NatNet major")?,
            "nn_minor" => s.natnet_minor = int_field(value, "NatNet minor")?,
            "rbid" => s.rigid_body_id = int_field(value, "rigid body id")?,
            "sysid" => s.mav_sysid = int_field(value, "sysid")?,
            "compid" => s.mav_compid = int_field(value, "compid")?,
            "baud" => s.mavlink_baud = int_field(value, "baud")?,
            "vpe_hz" => s.vpe_rate_hz = int_field(value, "VPE rate")?,
            "hb_hz" => s.heartbeat_rate_hz = int_field(value, "heartbeat rate")?,
            "timeout_ms" => s.track_timeout_ms = int_field(value, "tracking timeout")?,
            _ => {} // unknown key: ignore
        }
    }
    s.validate()
}

fn percent_decode<const N: usize>(raw: &[u8]) -> Result<Vec<u8, N>, &'static str> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < raw.len() {
        let b = match raw[i] {
            b'+' => b' ',
            b'%' => {
                let hi = *raw.get(i + 1).ok_or("truncated percent-escape")?;
                let lo = *raw.get(i + 2).ok_or("truncated percent-escape")?;
                i += 2;
                (hex(hi)? << 4) | hex(lo)?
            }
            other => other,
        };
        out.push(b).map_err(|_| "form value too long")?;
        i += 1;
    }
    Ok(out)
}

fn hex(b: u8) -> Result<u8, &'static str> {
    match b {
        b'0'..=b'9' => Ok(b - b'0'),
        b'a'..=b'f' => Ok(b - b'a' + 10),
        b'A'..=b'F' => Ok(b - b'A' + 10),
        _ => Err("bad percent-escape hex"),
    }
}

fn string_field<const N: usize>(value: &str, err: &'static str) -> Result<String<N>, &'static str> {
    String::try_from(value).map_err(|_| err)
}

fn int_field<T: core::str::FromStr>(value: &str, what: &'static str) -> Result<T, &'static str> {
    let _ = what;
    value.trim().parse().map_err(|_| "field is not a valid number")
}

fn parse_dotted_quad(value: &str) -> Result<[u8; 4], &'static str> {
    let mut out = [0u8; 4];
    let mut parts = value.trim().split('.');
    for slot in &mut out {
        *slot = parts
            .next()
            .ok_or("multicast address needs 4 octets")?
            .parse()
            .map_err(|_| "multicast octet is not 0-255")?;
    }
    if parts.next().is_some() {
        return Err("multicast address needs exactly 4 octets");
    }
    Ok(out)
}
