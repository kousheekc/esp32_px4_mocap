//! Compile-time configuration

pub const AP_SSID: &str = "esp";
pub const AP_PASS: &str = "12345678";
pub const AP_CHANNEL: u8 = 6;

pub const PORTAL_IP: [u8; 4] = [192, 168, 4, 1];
pub const HTTP_PORT: u16 = 80;

pub const RESET_HOLD_MS: u64 = 3000;

/// Local UDP port QTM streams frames to
pub const QTM_UDP_PORT: u16 = 22230;
/// Re-request streaming after this long without a QTM frame
pub const QTM_SILENCE_MS: u64 = 3000;
