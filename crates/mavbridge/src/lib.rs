//! MAVLink v2 encoding: HEARTBEAT (#0), VISION_POSITION_ESTIMATE (#102) and TIMESYNC (#111)
//! replies, plus a decoder that picks the flight controller's TIMESYNC requests out of the
//! UART byte stream.

#![cfg_attr(not(test), no_std)]

use mavlink::dialects::common::{
    MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, HEARTBEAT_DATA, TIMESYNC_DATA,
    VISION_POSITION_ESTIMATE_DATA,
};
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, MavlinkVersion};

/// Worst-case v2 frame: 10 B header + 255 B payload + 2 B checksum
pub const MAX_FRAME_LEN: usize = 267;

const V1_STX: u8 = 0xFE;
const V2_STX: u8 = 0xFD;
const MSG_ID_TIMESYNC: u32 = 111;
/// Incoming frames may also carry a 13 B v2 signature
const MAX_RX_FRAME_LEN: usize = MAX_FRAME_LEN + 13;

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

    /// TIMESYNC reply: our clock in `tc1` (ns), the requester's `ts1` mirrored back
    pub fn timesync_reply(&mut self, buf: &mut [u8], now_ns: i64, req: &TimesyncRequest) -> usize {
        let msg = MavMessage::TIMESYNC(TIMESYNC_DATA {
            tc1: now_ns,
            ts1: req.ts1,
            target_system: req.sysid,
            target_component: req.compid,
        });
        self.encode(buf, &msg)
    }
}

/// A TIMESYNC request (`tc1 == 0`) the flight controller sent to estimate our clock offset
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimesyncRequest {
    /// Requester's timestamp (ns), to be mirrored back
    pub ts1: i64,
    pub sysid: u8,
    pub compid: u8,
}

/// Byte-at-a-time MAVLink v1/v2 frame splitter that yields only TIMESYNC requests
pub struct Decoder {
    buf: [u8; MAX_RX_FRAME_LEN],
    len: usize,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_RX_FRAME_LEN],
            len: 0,
        }
    }

    /// Feed one received byte; returns a request once a valid TIMESYNC request frame completes.
    /// A frame that fails its checksum is dropped and scanning resumes at the next start byte.
    pub fn push(&mut self, byte: u8) -> Option<TimesyncRequest> {
        if self.len == 0 && byte != V1_STX && byte != V2_STX {
            return None;
        }
        self.buf[self.len] = byte;
        self.len += 1;

        let total = frame_len(&self.buf[..self.len])?;
        if self.len < total {
            return None;
        }
        let req = parse_timesync_request(&self.buf[..total]);
        self.len = 0;
        req
    }
}

/// Full frame length once enough of the header is in, else `None`
fn frame_len(b: &[u8]) -> Option<usize> {
    match b[0] {
        V2_STX if b.len() >= 3 => {
            let signed = b[2] & 0x01 != 0;
            Some(10 + b[1] as usize + 2 + if signed { 13 } else { 0 })
        }
        V1_STX if b.len() >= 2 => Some(6 + b[1] as usize + 2),
        _ => None,
    }
}

fn parse_timesync_request(frame: &[u8]) -> Option<TimesyncRequest> {
    // Check the id from the header first so the other traffic on the link isn't fully decoded.
    let id = match frame[0] {
        V2_STX => u32::from_le_bytes([frame[7], frame[8], frame[9], 0]),
        _ => u32::from(frame[5]),
    };
    if id != MSG_ID_TIMESYNC {
        return None;
    }
    let mut reader = PeekReader::<_, MAX_RX_FRAME_LEN>::new(frame);
    match mavlink::read_any_msg::<MavMessage, _>(&mut reader) {
        Ok((hdr, MavMessage::TIMESYNC(t))) if t.tc1 == 0 => Some(TimesyncRequest {
            ts1: t.ts1,
            sysid: hdr.system_id,
            compid: hdr.component_id,
        }),
        _ => None,
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

    /// A TIMESYNC frame as the flight controller (sysid 1, compid 1) would send it
    fn timesync_frame(tc1: i64, ts1: i64) -> Vec<u8> {
        let mut enc = Encoder::new(1, 1);
        let mut buf = [0u8; MAX_FRAME_LEN];
        let msg = MavMessage::TIMESYNC(TIMESYNC_DATA {
            tc1,
            ts1,
            target_system: 0,
            target_component: 0,
        });
        let n = enc.encode(&mut buf, &msg);
        buf[..n].to_vec()
    }

    fn decode_all(dec: &mut Decoder, bytes: &[u8]) -> Vec<TimesyncRequest> {
        bytes.iter().filter_map(|&b| dec.push(b)).collect()
    }

    #[test]
    fn decodes_request_after_garbage_and_other_traffic() {
        let mut stream = vec![0x00, 0x42, 0xFF];
        let mut buf = [0u8; MAX_FRAME_LEN];
        let n = Encoder::new(1, 1).heartbeat(&mut buf);
        stream.extend_from_slice(&buf[..n]);
        stream.extend(timesync_frame(0, 123_456_789));

        let reqs = decode_all(&mut Decoder::new(), &stream);
        assert_eq!(
            reqs,
            vec![TimesyncRequest { ts1: 123_456_789, sysid: 1, compid: 1 }]
        );
    }

    #[test]
    fn ignores_replies() {
        assert!(decode_all(&mut Decoder::new(), &timesync_frame(5, 6)).is_empty());
    }

    #[test]
    fn bad_checksum_dropped_then_recovers() {
        let mut bad = timesync_frame(0, 1);
        let last = bad.len() - 1;
        bad[last] ^= 0xFF;
        let mut stream = bad;
        stream.extend(timesync_frame(0, 2));
        let reqs = decode_all(&mut Decoder::new(), &stream);
        assert_eq!(reqs.len(), 1);
        assert_eq!(reqs[0].ts1, 2);
    }

    #[test]
    fn reply_round_trips_through_decoder_fields() {
        let req = TimesyncRequest { ts1: 42, sysid: 1, compid: 1 };
        let mut buf = [0u8; MAX_FRAME_LEN];
        let n = Encoder::new(1, 197).timesync_reply(&mut buf, 1_000, &req);
        assert_eq!(&buf[7..10], &[111, 0, 0], "msgid 111");
        let mut reader = PeekReader::<_, MAX_RX_FRAME_LEN>::new(&buf[..n]);
        let (_, msg) = mavlink::read_any_msg::<MavMessage, _>(&mut reader).unwrap();
        match msg {
            MavMessage::TIMESYNC(t) => {
                assert_eq!((t.tc1, t.ts1, t.target_system, t.target_component), (1_000, 42, 1, 1));
            }
            other => panic!("unexpected {other:?}"),
        }
    }
}
