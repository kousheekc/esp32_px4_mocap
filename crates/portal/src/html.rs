//! Portal HTML

use crate::Settings;
use core::fmt::{self, Display, Write};

struct Esc<'a>(&'a str);

impl Display for Esc<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for c in self.0.chars() {
            match c {
                '&' => f.write_str("&amp;")?,
                '<' => f.write_str("&lt;")?,
                '>' => f.write_str("&gt;")?,
                '"' => f.write_str("&quot;")?,
                _ => f.write_char(c)?,
            }
        }
        Ok(())
    }
}

/// Render the settings form prefilled from `s`, with an optional error banner
pub fn render_page(s: &Settings, msg: Option<&str>, out: &mut impl Write) -> fmt::Result {
    out.write_str(
        "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
         <title>Mocap Bridge Setup</title><style>\
         body{font-family:sans-serif;max-width:30em;margin:2em auto;padding:0 1em}\
         label{display:block;margin:.8em 0 .2em}input{width:100%;box-sizing:border-box;padding:.4em}\
         fieldset{margin:1em 0;border:1px solid #aaa;border-radius:4px}\
         button{margin-top:1.2em;padding:.6em 1.2em;font-size:1em}\
         .msg{padding:.6em;border:1px solid #c00;border-radius:4px;color:#c00}\
         .note{color:#666;font-size:.85em}</style></head><body>\
         <h1>Mocap Bridge Setup</h1>",
    )?;
    if let Some(m) = msg {
        write!(out, "<p class=\"msg\">{}</p>", Esc(m))?;
    }
    write!(
        out,
        "<form method=\"POST\" action=\"/save\">\
         <fieldset><legend>WiFi (Motive network)</legend>\
         <label>SSID</label><input name=\"ssid\" value=\"{ssid}\" maxlength=\"32\" required>\
         <label>Password</label><input name=\"pass\" value=\"{pass}\" maxlength=\"64\">\
         </fieldset>\
         <fieldset><legend>NatNet (Motive streaming)</legend>\
         <label>Multicast address</label><input name=\"mcast\" value=\"{m0}.{m1}.{m2}.{m3}\">\
         <label>Data port</label><input name=\"port\" type=\"number\" min=\"1\" max=\"65535\" value=\"{port}\">\
         <label>NatNet version major</label><input name=\"nn_major\" type=\"number\" min=\"2\" max=\"4\" value=\"{nnmaj}\">\
         <label>NatNet version minor</label><input name=\"nn_minor\" type=\"number\" min=\"0\" max=\"255\" value=\"{nnmin}\">\
         <label>Rigid body id</label><input name=\"rbid\" type=\"number\" value=\"{rbid}\">\
         </fieldset>\
         <fieldset><legend>MAVLink (flight controller)</legend>\
         <label>System id</label><input name=\"sysid\" type=\"number\" min=\"1\" max=\"255\" value=\"{sysid}\">\
         <label>Component id</label><input name=\"compid\" type=\"number\" min=\"1\" max=\"255\" value=\"{compid}\">\
         <label>UART baud</label><input name=\"baud\" type=\"number\" value=\"{baud}\">\
         <label>VPE rate (Hz)</label><input name=\"vpe_hz\" type=\"number\" min=\"1\" max=\"1000\" value=\"{vpe}\">\
         <label>Heartbeat rate (Hz)</label><input name=\"hb_hz\" type=\"number\" min=\"1\" max=\"100\" value=\"{hb}\">\
         <label>Tracking timeout (ms)</label><input name=\"timeout_ms\" type=\"number\" min=\"10\" value=\"{timeout}\">\
         </fieldset>\
         <button type=\"submit\">Save &amp; restart with new settings</button></form>\
         <p class=\"note\">If the device can't join your network after saving, hold the BOOT \
         button for 3 seconds to erase the settings and return to this setup page.</p>\
         </body></html>",
        ssid = Esc(s.wifi_ssid.as_str()),
        pass = Esc(s.wifi_pass.as_str()),
        m0 = s.multicast_addr[0],
        m1 = s.multicast_addr[1],
        m2 = s.multicast_addr[2],
        m3 = s.multicast_addr[3],
        port = s.data_port,
        nnmaj = s.natnet_major,
        nnmin = s.natnet_minor,
        rbid = s.rigid_body_id,
        sysid = s.mav_sysid,
        compid = s.mav_compid,
        baud = s.mavlink_baud,
        vpe = s.vpe_rate_hz,
        hb = s.heartbeat_rate_hz,
        timeout = s.track_timeout_ms,
    )
}

/// Response to a successful save, shown just before reboot
pub fn render_reboot_page(out: &mut impl Write) -> fmt::Result {
    out.write_str(
        "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
         <title>Rebooting</title></head><body style=\"font-family:sans-serif;max-width:30em;margin:2em auto\">\
         <h1>Settings saved</h1>\
         <p>The bridge is restarting and will now connect to your network. \
         This access point will disappear.</p>\
         <p>To change settings later, hold the BOOT button for 3 seconds — the \
         device returns to setup mode with default settings.</p></body></html>",
    )
}
