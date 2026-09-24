//! Qualisys QTM real-time protocol (little-endian, port 22223) packet encoder/parser

#![cfg_attr(not(test), no_std)]

/// QTM little-endian RT port
pub const DEFAULT_PORT: u16 = 22223;

/// Packet header: size (incl. header) + type
pub const HEADER_LEN: usize = 8;

/// 6D component id (position + rotation matrix per body)
const COMPONENT_6D: u32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketType {
    Error,
    Command,
    Xml,
    Data,
    NoMoreData,
    Event,
    Other(u32),
}

impl From<u32> for PacketType {
    fn from(v: u32) -> Self {
        match v {
            0 => Self::Error,
            1 => Self::Command,
            2 => Self::Xml,
            3 => Self::Data,
            4 => Self::NoMoreData,
            6 => Self::Event,
            other => Self::Other(other),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Packet<'a> {
    pub kind: PacketType,
    pub payload: &'a [u8],
}

impl Packet<'_> {
    /// Payload of an Error/Command packet as text, without the trailing NUL
    pub fn text(&self) -> Option<&str> {
        let end = self.payload.iter().position(|&b| b == 0).unwrap_or(self.payload.len());
        core::str::from_utf8(&self.payload[..end]).ok()
    }
}

/// 6D body in QTM's global frame
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    /// Position (millimetres)
    pub pos: [f32; 3],
    /// Rotation matrix, column-major
    pub rot: [f32; 9],
    /// QTM sends NaN for bodies it can't currently track
    pub valid: bool,
}

struct ByteReader<'a> {
    data: &'a [u8],
    off: usize,
}

impl<'a> ByteReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, off: 0 }
    }

    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let end = self.off.checked_add(n)?;
        let s = self.data.get(self.off..end)?;
        self.off = end;
        Some(s)
    }

    fn skip(&mut self, n: usize) -> Option<()> {
        self.take(n).map(|_| ())
    }

    fn u16(&mut self) -> Option<u16> {
        self.take(2).map(|b| u16::from_le_bytes([b[0], b[1]]))
    }

    fn u32(&mut self) -> Option<u32> {
        self.take(4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f32(&mut self) -> Option<f32> {
        self.take(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
}

/// Encode `cmd` as a NUL-terminated Command packet; `None` if `out` is too small
pub fn encode_command(cmd: &str, out: &mut [u8]) -> Option<usize> {
    let len = HEADER_LEN + cmd.len() + 1;
    let out = out.get_mut(..len)?;
    out[0..4].copy_from_slice(&(len as u32).to_le_bytes());
    out[4..8].copy_from_slice(&1u32.to_le_bytes());
    out[8..len - 1].copy_from_slice(cmd.as_bytes());
    out[len - 1] = 0;
    Some(len)
}

/// Total size of the packet starting at `data`, once its header is available
pub fn packet_len(data: &[u8]) -> Option<usize> {
    let b = data.get(..4)?;
    let len = u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize;
    (len >= HEADER_LEN).then_some(len)
}

/// Split one complete packet off the front of `data`
pub fn parse_packet(data: &[u8]) -> Option<Packet<'_>> {
    let len = packet_len(data)?;
    let data = data.get(..len)?;
    let kind = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
    Some(Packet {
        kind: kind.into(),
        payload: &data[HEADER_LEN..],
    })
}

/// Parse a Data packet payload, returning the 6D body at `index` (0-based), or `None` if it's
/// malformed, has no 6D component, or has fewer bodies
pub fn parse_6d(payload: &[u8], index: usize) -> Option<Body> {
    let mut r = ByteReader::new(payload);
    r.skip(8)?; // timestamp
    r.u32()?; // frame number
    let n_components = r.u32()?;

    for _ in 0..n_components {
        let start = r.off;
        let size = r.u32()? as usize;
        let kind = r.u32()?;
        if size < HEADER_LEN {
            return None;
        }
        if kind != COMPONENT_6D {
            r.skip(size - HEADER_LEN)?;
            continue;
        }

        let n_bodies = r.u32()? as usize;
        r.u16()?; // 2D drop rate
        r.u16()?; // 2D out-of-sync rate
        if index >= n_bodies {
            return None;
        }
        r.skip(index.checked_mul(12 * 4)?)?;
        let pos = [r.f32()?, r.f32()?, r.f32()?];
        let mut rot = [0f32; 9];
        for v in &mut rot {
            *v = r.f32()?;
        }
        if r.off > start + size {
            return None;
        }
        let valid = pos.iter().chain(rot.iter()).all(|v| v.is_finite());
        return Some(Body { pos, rot, valid });
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    const IDENTITY: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

    fn data_payload(components: &[Vec<u8>]) -> Vec<u8> {
        let mut p = Vec::new();
        p.extend_from_slice(&123_456u64.to_le_bytes());
        p.extend_from_slice(&42u32.to_le_bytes());
        p.extend_from_slice(&(components.len() as u32).to_le_bytes());
        for c in components {
            p.extend_from_slice(c);
        }
        p
    }

    fn component(kind: u32, body: &[u8]) -> Vec<u8> {
        let mut c = Vec::new();
        c.extend_from_slice(&((HEADER_LEN + body.len()) as u32).to_le_bytes());
        c.extend_from_slice(&kind.to_le_bytes());
        c.extend_from_slice(body);
        c
    }

    fn six_d(bodies: &[([f32; 3], [f32; 9])]) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&(bodies.len() as u32).to_le_bytes());
        b.extend_from_slice(&0u16.to_le_bytes());
        b.extend_from_slice(&0u16.to_le_bytes());
        for (pos, rot) in bodies {
            for v in pos.iter().chain(rot.iter()) {
                b.extend_from_slice(&v.to_le_bytes());
            }
        }
        component(COMPONENT_6D, &b)
    }

    #[test]
    fn command_roundtrip() {
        let mut buf = [0u8; 64];
        let n = encode_command("Version 1.19", &mut buf).unwrap();
        assert_eq!(n, 8 + 12 + 1);
        assert_eq!(packet_len(&buf), Some(n));
        let p = parse_packet(&buf[..n]).unwrap();
        assert_eq!(p.kind, PacketType::Command);
        assert_eq!(p.text(), Some("Version 1.19"));
    }

    #[test]
    fn command_too_long() {
        let mut buf = [0u8; 10];
        assert_eq!(encode_command("Version 1.19", &mut buf), None);
    }

    #[test]
    fn picks_body_by_index() {
        let p = data_payload(&[six_d(&[
            ([1.0, 2.0, 3.0], IDENTITY),
            ([4.0, 5.0, 6.0], IDENTITY),
        ])]);
        let b = parse_6d(&p, 1).unwrap();
        assert_eq!(b.pos, [4.0, 5.0, 6.0]);
        assert_eq!(b.rot, IDENTITY);
        assert!(b.valid);
        assert_eq!(parse_6d(&p, 2), None);
    }

    #[test]
    fn skips_other_components() {
        let other = component(1, &[0xAA; 20]);
        let p = data_payload(&[other, six_d(&[([7.0, 8.0, 9.0], IDENTITY)])]);
        assert_eq!(parse_6d(&p, 0).unwrap().pos, [7.0, 8.0, 9.0]);
    }

    #[test]
    fn nan_is_invalid() {
        let p = data_payload(&[six_d(&[([f32::NAN; 3], [f32::NAN; 9])])]);
        assert!(!parse_6d(&p, 0).unwrap().valid);
    }

    #[test]
    fn truncated_is_none() {
        let p = data_payload(&[six_d(&[([1.0, 2.0, 3.0], IDENTITY)])]);
        assert_eq!(parse_6d(&p[..p.len() - 1], 0), None);
        assert_eq!(parse_6d(&data_payload(&[]), 0), None);
    }
}
