//! NatNet frame-of-data parser

#![cfg_attr(not(test), no_std)]

/// NatNet message id for a frame of mocap data
pub const NAT_FRAMEOFDATA: u16 = 7;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version {
    pub major: u8,
    pub minor: u8,
}

impl Version {
    pub const fn new(major: u8, minor: u8) -> Self {
        Self { major, minor }
    }

    /// Pre-3.0 embeds each rigid body's marker data inline
    fn has_embedded_rb_markers(self) -> bool {
        self.major < 3
    }

    /// 2.0+ appends the mean marker error
    fn has_mean_error(self) -> bool {
        self.major >= 2
    }

    /// 2.6+ appends the params bitfield (bit 0 = tracking valid)
    fn has_params(self) -> bool {
        self.major > 2 || (self.major == 2 && self.minor >= 6)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBody {
    pub id: i32,
    pub pos: [f32; 3],
    pub quat: [f32; 4],
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

    fn i16(&mut self) -> Option<i16> {
        self.take(2).map(|b| i16::from_le_bytes([b[0], b[1]]))
    }

    fn i32(&mut self) -> Option<i32> {
        self.take(4).map(|b| i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f32(&mut self) -> Option<f32> {
        self.take(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn count(&mut self) -> Option<usize> {
        let v = self.i32()?;
        usize::try_from(v).ok()
    }

    fn skip_string(&mut self) -> Option<()> {
        let rest = self.data.get(self.off..)?;
        let nul = rest.iter().position(|&b| b == 0)?;
        self.off += nul + 1;
        Some(())
    }
}

/// Parse a NatNet datagram, returning the rigid body with id `target_id`, or `None` if it's not a well-formed frame-of-data containing it
pub fn parse_frame(data: &[u8], version: Version, target_id: i32) -> Option<RigidBody> {
    let mut r = ByteReader::new(data);

    // header: messageID + byte count
    if r.u16()? != NAT_FRAMEOFDATA {
        return None;
    }
    r.u16()?; // nDataBytes
    r.i32()?; // frame number

    // marker sets: name + count + xyz floats
    let n_marker_sets = r.count()?;
    for _ in 0..n_marker_sets {
        r.skip_string()?;
        let n_markers = r.count()?;
        r.skip(n_markers.checked_mul(3 * 4)?)?;
    }

    // legacy "other" markers
    let n_other = r.count()?;
    r.skip(n_other.checked_mul(3 * 4)?)?;

    // rigid bodies
    let n_rigid = r.count()?;
    let mut found: Option<RigidBody> = None;

    for _ in 0..n_rigid {
        let id = r.i32()?;
        let pos = [r.f32()?, r.f32()?, r.f32()?];
        let quat = [r.f32()?, r.f32()?, r.f32()?, r.f32()?];

        if version.has_embedded_rb_markers() {
            // pre-3.0: associated markers embedded here
            let n_rb_markers = r.count()?;
            r.skip(n_rb_markers.checked_mul(3 * 4)?)?; // positions
            if version.major >= 2 {
                r.skip(n_rb_markers.checked_mul(4)?)?; // ids
                r.skip(n_rb_markers.checked_mul(4)?)?; // sizes
            }
        }

        if version.has_mean_error() {
            r.f32()?; // mean marker error
        }

        let valid = if version.has_params() {
            (r.i16()? & 0x01) != 0
        } else {
            true
        };

        if id == target_id {
            found = Some(RigidBody { id, pos, quat, valid });
        }
    }

    found
}