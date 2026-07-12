//! User-configurable settings
//!
//! Format:
//!
//! | offset | size | field                       |
//! |--------|------|-----------------------------|
//! | 0      | 4    | magic `b"MCFG"`             |
//! | 4      | 1    | format version (= 1)        |
//! | 5      | 1    | ssid length (<= 32)         |
//! | 6      | 1    | password length (<= 64)     |
//! | 7      | 1    | reserved (0)                |
//! | 8      | 32   | ssid bytes, zero-padded     |
//! | 40     | 64   | password bytes, zero-padded |
//! | 104    | 4    | multicast address           |
//! | 108    | 2    | data port                   |
//! | 110    | 1+1  | natnet major, minor         |
//! | 112    | 4    | rigid body id (i32)         |
//! | 116    | 1+1  | mav sysid, compid           |
//! | 118    | 4    | mavlink baud                |
//! | 122    | 2    | vpe rate (Hz)               |
//! | 124    | 2    | heartbeat rate (Hz)         |
//! | 126    | 2    | reserved (0)                |
//! | 128    | 4    | tracking timeout (ms)       |
//! | 132    | 4    | CRC32 (IEEE) of bytes 0..132|

#![cfg_attr(not(test), no_std)]

pub mod form;
pub mod html;

use heapless::String;

const MAGIC: &[u8; 4] = b"MCFG";
const FORMAT_VERSION: u8 = 1;

pub const WIRE_LEN: usize = 136;

/// User-configurable settings
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Settings {
    /// WiFi SSID (<= 32 bytes)
    pub wifi_ssid: String<32>,
    /// WPA2 passphrase (<= 64 bytes)
    pub wifi_pass: String<64>,
    /// NatNet multicast group (Motive default 239.255.42.99)
    pub multicast_addr: [u8; 4],
    /// NatNet data port (Motive default 1511)
    pub data_port: u16,
    /// NatNet protocol version
    pub natnet_major: u8,
    pub natnet_minor: u8,
    /// Rigid body id
    pub rigid_body_id: i32,
    /// MAVLink identity (compid 197 = vision/odometry source)
    pub mav_sysid: u8,
    pub mav_compid: u8,
    /// UART1 baud to the flight controller
    pub mavlink_baud: u32,
    /// VISION_POSITION_ESTIMATE rate
    pub vpe_rate_hz: u16,
    /// HEARTBEAT rate
    pub heartbeat_rate_hz: u16,
    /// Stop TX after no fresh pose for this long, so EKF2 coasts
    pub track_timeout_ms: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            wifi_ssid: String::try_from("mocap-wifi").unwrap(),
            wifi_pass: String::try_from("mocap-password").unwrap(),
            multicast_addr: [239, 255, 42, 99],
            data_port: 1511,
            natnet_major: 3,
            natnet_minor: 1,
            rigid_body_id: 32,
            mav_sysid: 1,
            mav_compid: 197,
            mavlink_baud: 921_600,
            vpe_rate_hz: 100,
            heartbeat_rate_hz: 1,
            track_timeout_ms: 200,
        }
    }
}

impl Settings {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.wifi_ssid.is_empty() {
            return Err("WiFi SSID must not be empty");
        }
        if !(224..=239).contains(&self.multicast_addr[0]) {
            return Err("multicast address must be in 224.0.0.0/4");
        }
        if self.data_port == 0 {
            return Err("data port must not be 0");
        }
        if !(2..=4).contains(&self.natnet_major) {
            return Err("NatNet major version must be 2..=4");
        }
        if !(9_600..=3_000_000).contains(&self.mavlink_baud) {
            return Err("baud must be within 9600..=3000000");
        }
        if self.heartbeat_rate_hz == 0 || self.vpe_rate_hz == 0 {
            return Err("rates must be at least 1 Hz");
        }
        if self.vpe_rate_hz < self.heartbeat_rate_hz
            || self.vpe_rate_hz % self.heartbeat_rate_hz != 0
        {
            return Err("VPE rate must be a multiple of the heartbeat rate");
        }
        if self.track_timeout_ms < 10 {
            return Err("tracking timeout must be at least 10 ms");
        }
        Ok(())
    }

    pub fn to_bytes(&self, out: &mut [u8; WIRE_LEN]) {
        out.fill(0);
        out[0..4].copy_from_slice(MAGIC);
        out[4] = FORMAT_VERSION;
        out[5] = self.wifi_ssid.len() as u8;
        out[6] = self.wifi_pass.len() as u8;
        out[8..8 + self.wifi_ssid.len()].copy_from_slice(self.wifi_ssid.as_bytes());
        out[40..40 + self.wifi_pass.len()].copy_from_slice(self.wifi_pass.as_bytes());
        out[104..108].copy_from_slice(&self.multicast_addr);
        out[108..110].copy_from_slice(&self.data_port.to_le_bytes());
        out[110] = self.natnet_major;
        out[111] = self.natnet_minor;
        out[112..116].copy_from_slice(&self.rigid_body_id.to_le_bytes());
        out[116] = self.mav_sysid;
        out[117] = self.mav_compid;
        out[118..122].copy_from_slice(&self.mavlink_baud.to_le_bytes());
        out[122..124].copy_from_slice(&self.vpe_rate_hz.to_le_bytes());
        out[124..126].copy_from_slice(&self.heartbeat_rate_hz.to_le_bytes());
        out[128..132].copy_from_slice(&self.track_timeout_ms.to_le_bytes());
        let crc = crc32(&out[..WIRE_LEN - 4]);
        out[132..136].copy_from_slice(&crc.to_le_bytes());
    }

    pub fn from_bytes(data: &[u8]) -> Option<Settings> {
        if data.len() < WIRE_LEN {
            return None;
        }
        let data = &data[..WIRE_LEN];
        if &data[0..4] != MAGIC || data[4] != FORMAT_VERSION {
            return None;
        }
        let crc_stored = u32::from_le_bytes(data[132..136].try_into().unwrap());
        if crc32(&data[..WIRE_LEN - 4]) != crc_stored {
            return None;
        }
        let ssid_len = data[5] as usize;
        let pass_len = data[6] as usize;
        if ssid_len > 32 || pass_len > 64 {
            return None;
        }
        let s = Settings {
            wifi_ssid: str_from(&data[8..8 + ssid_len])?,
            wifi_pass: str_from(&data[40..40 + pass_len])?,
            multicast_addr: data[104..108].try_into().unwrap(),
            data_port: u16::from_le_bytes(data[108..110].try_into().unwrap()),
            natnet_major: data[110],
            natnet_minor: data[111],
            rigid_body_id: i32::from_le_bytes(data[112..116].try_into().unwrap()),
            mav_sysid: data[116],
            mav_compid: data[117],
            mavlink_baud: u32::from_le_bytes(data[118..122].try_into().unwrap()),
            vpe_rate_hz: u16::from_le_bytes(data[122..124].try_into().unwrap()),
            heartbeat_rate_hz: u16::from_le_bytes(data[124..126].try_into().unwrap()),
            track_timeout_ms: u32::from_le_bytes(data[128..132].try_into().unwrap()),
        };
        s.validate().ok()?;
        Some(s)
    }
}

fn str_from<const N: usize>(bytes: &[u8]) -> Option<String<N>> {
    let s = core::str::from_utf8(bytes).ok()?;
    String::try_from(s).ok()
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}
