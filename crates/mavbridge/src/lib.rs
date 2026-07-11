//! MAVLink v2 encoding: HEARTBEAT (#0) and VISION_POSITION_ESTIMATE (#102)

#![cfg_attr(not(test), no_std)]

use mavlink::dialects::common::{
    MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, HEARTBEAT_DATA,
    VISION_POSITION_ESTIMATE_DATA,
};
use mavlink::{MavHeader, MavlinkVersion};

/// Worst-case v2 frame: 10 B header + 255 B payload + 2 B checksum
pub const MAX_FRAME_LEN: usize = 267;

/// `mavlink::embedded::Write` over a fixed buffer
struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl mavlink::embedded::Write for BufWriter<'_> {
    fn write_all(&mut self, data: &[u8]) -> Result<(), mavlink::error::MessageWriteError> {
        self.buf[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();
        Ok(())
    }
}

/// MAVLink v2 encoder owning the per-link sequence counter
pub struct Encoder {
    header: MavHeader,
}

impl Encoder {
    pub fn new(sysid: u8, compid: u8) -> Self {
        Self {
            header: MavHeader {
                system_id: sysid,
                component_id: compid,
                sequence: 0,
            },
        }
    }

    /// Serialize `msg` as v2 into `buf`, returning the frame length
    fn encode(&mut self, buf: &mut [u8], msg: &MavMessage) -> usize {
        let mut writer = BufWriter { buf, pos: 0 };
        mavlink::write_versioned_msg(&mut writer, MavlinkVersion::V2, self.header, msg)
            .expect("mavlink encode");
        self.header.sequence = self.header.sequence.wrapping_add(1);
        writer.pos
    }

    /// HEARTBEAT: onboard vision source
    pub fn heartbeat(&mut self, buf: &mut [u8]) -> usize {
        let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });
        self.encode(buf, &msg)
    }

    /// VISION_POSITION_ESTIMATE: NED position (m) + FRD euler (rad).
    #[allow(clippy::too_many_arguments)]
    pub fn vision_position_estimate(
        &mut self,
        buf: &mut [u8],
        usec: u64,
        x: f32,
        y: f32,
        z: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
        reset_counter: u8,
    ) -> usize {
        let mut covariance = [0.0f32; 21];
        covariance[0] = f32::NAN;
        let msg = MavMessage::VISION_POSITION_ESTIMATE(VISION_POSITION_ESTIMATE_DATA {
            usec,
            x,
            y,
            z,
            roll,
            pitch,
            yaw,
            covariance,
            reset_counter,
        });
        self.encode(buf, &msg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEADER_LEN: usize = 10;

    #[test]
    fn heartbeat_frame_shape() {
        let mut enc = Encoder::new(1, 197);
        let mut buf = [0u8; MAX_FRAME_LEN];
        let n = enc.heartbeat(&mut buf);

        assert_eq!(buf[0], 0xFD, "v2 STX");
        assert_eq!(buf[4], 0, "first frame seq");
        assert_eq!(buf[5], 1, "sysid");
        assert_eq!(buf[6], 197, "compid");
        assert_eq!(&buf[7..10], &[0, 0, 0], "msgid 0");
        assert_eq!(n, HEADER_LEN + 9 + 2);
    }

    #[test]
    fn vpe_frame_shape() {
        let mut enc = Encoder::new(1, 197);
        let mut buf = [0u8; MAX_FRAME_LEN];
        let n = enc.vision_position_estimate(&mut buf, 1, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0);

        assert_eq!(buf[0], 0xFD, "v2 STX");
        assert_eq!(buf[4], 0, "first frame seq");
        assert_eq!(buf[5], 1, "sysid");
        assert_eq!(buf[6], 197, "compid");
        assert_eq!(&buf[7..10], &[102, 0, 0], "msgid 102");
        // zero reset_counter: zero cov tail truncated -> 36 B payload
        assert_eq!(n, HEADER_LEN + 36 + 2);

        // non-zero reset_counter is the last byte: full 117 B payload
        let n = enc.vision_position_estimate(&mut buf, 1, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 5);
        assert_eq!(n, HEADER_LEN + 117 + 2);
    }

    #[test]
    fn sequence_increments() {
        let mut enc = Encoder::new(1, 197);
        let mut buf = [0u8; MAX_FRAME_LEN];
        enc.heartbeat(&mut buf);
        enc.vision_position_estimate(&mut buf, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
        assert_eq!(buf[4], 1, "shared per-link counter");
        enc.heartbeat(&mut buf);
        assert_eq!(buf[4], 2);
    }
}
