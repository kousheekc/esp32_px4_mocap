//! Portal HTML

use crate::{MocapSource, Settings};
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
         label{display:block;margin:.8em 0 .2em}input,select{width:100%;box-sizing:border-box;padding:.4em}\
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
         <fieldset><legend>WiFi (mocap network)</legend>\
         <label>SSID</label><input name=\"ssid\" value=\"{ssid}\" maxlength=\"32\" required>\
         <label>Password</label><input name=\"pass\" value=\"{pass}\" maxlength=\"64\">\
         </fieldset>\
         <fieldset><legend>Mocap system</legend>\
         <label>Source</label><select name=\"source\">\
         <option value=\"natnet\"{sel_nn}>OptiTrack (NatNet)</option>\
         <option value=\"qtm\"{sel_qtm}>Qualisys (QTM)</option></select>\
         <label>Rigid body</label><input name=\"rbid\" type=\"number\" value=\"{rbid}\">\
         <p class=\"note\">OptiTrack: the Streaming ID. Qualisys: position in QTM's 6DOF body list (1 = first).</p>\
         </fieldset>\
         <fieldset><legend>NatNet (OptiTrack only)</legend>\
         <label>Multicast address</label><input name=\"mcast\" value=\"{m0}.{m1}.{m2}.{m3}\">\
         <label>Data port</label><input name=\"port\" type=\"number\" min=\"1\" max=\"65535\" value=\"{port}\">\
         <label>NatNet version major</label><input name=\"nn_major\" type=\"number\" min=\"2\" max=\"4\" value=\"{nnmaj}\">\
         <label>NatNet version minor</label><input name=\"nn_minor\" type=\"number\" min=\"0\" max=\"255\" value=\"{nnmin}\">\
         </fieldset>\
         <fieldset><legend>QTM (Qualisys only)</legend>\
         <label>QTM PC address</label><input name=\"qtm_host\" value=\"{q0}.{q1}.{q2}.{q3}\">\
         <label>RT port</label><input name=\"qtm_port\" type=\"number\" min=\"1\" max=\"65535\" value=\"{qport}\">\
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
        sel_nn = if s.source == MocapSource::NatNet { " selected" } else { "" },
        sel_qtm = if s.source == MocapSource::Qtm { " selected" } else { "" },
        m0 = s.multicast_addr[0],
        m1 = s.multicast_addr[1],
        m2 = s.multicast_addr[2],
        m3 = s.multicast_addr[3],
        port = s.data_port,
        nnmaj = s.natnet_major,
        nnmin = s.natnet_minor,
        q0 = s.qtm_host[0],
        q1 = s.qtm_host[1],
        q2 = s.qtm_host[2],
        q3 = s.qtm_host[3],
        qport = s.qtm_port,
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
         <p>To change settings later, hold the BOOT button for 3 seconds then the \
         device returns to setup mode with default settings.</p></body></html>",
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn page_fits_portal_buffer() {
        // firmware/src/server.rs renders into a heapless::String<4096>
        let mut page: heapless::String<4096> = heapless::String::new();
        let msg = "a fairly long validation error message for the banner";
        render_page(&Settings::default(), Some(msg), &mut page).unwrap();
    }
}
