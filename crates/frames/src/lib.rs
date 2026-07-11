//! Motive -> PX4 frame conversion

#![cfg_attr(not(test), no_std)]

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// Pose in PX4 NED. `pos` is [x, y, z] (metres); `q` is `[w, x, y, z]`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub pos: [f32; 3],
    pub q: [f32; 4],
}

/// 180 deg about X
fn flip_about_x() -> UnitQuaternion<f32> {
    UnitQuaternion::new_unchecked(Quaternion::new(0.0, 1.0, 0.0, 0.0))
}

/// Motive pose -> PX4 NED pose
/// Position: rotate by the change of basis
/// Orientation: change of basis by conjugation, q_ned = r q r^-1
pub fn transform_pose(pos: [f32; 3], quat_xyzw: [f32; 4]) -> Pose {
    let r = flip_about_x();
    let p = r * Vector3::from(pos);
    let q = Quaternion::new(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
    let q_ned = r.into_inner() * q * r.conjugate().into_inner();
    Pose {
        pos: [p.x, p.y, p.z],
        q: [q_ned.w, q_ned.i, q_ned.j, q_ned.k],
    }
}

/// Quaternion `[w, x, y, z]` -> (roll, pitch, yaw) (radians)
pub fn quat_to_euler_frd(q: [f32; 4]) -> (f32, f32, f32) {
    UnitQuaternion::from_quaternion(Quaternion::new(q[0], q[1], q[2], q[3])).euler_angles()
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4};

    const EPS: f32 = 1e-5;

    fn assert_near(a: f32, b: f32, msg: &str) {
        assert!((a - b).abs() < EPS, "{msg}: {a} vs {b}");
    }

    #[test]
    fn position_mapping() {
        let ned = transform_pose([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(ned.pos, [1.0, -2.0, -3.0]);
    }

    #[test]
    fn quaternion_mapping() {
        let ned = transform_pose([0.0; 3], [0.1, 0.2, 0.3, 0.4]);
        assert_eq!(ned.q, [0.4, 0.1, -0.2, -0.3]);
    }

    #[test]
    fn z_rotation_negative_yaw() {
        // 90 deg about Motive Z -> yaw = -90 deg.
        let s = FRAC_PI_4.sin();
        let c = FRAC_PI_4.cos();
        let ned = transform_pose([0.0; 3], [0.0, 0.0, s, c]);
        let (r, p, y) = quat_to_euler_frd(ned.q);
        assert_near(r, 0.0, "roll");
        assert_near(p, 0.0, "pitch");
        assert_near(y, -FRAC_PI_2, "yaw");
    }

    #[test]
    fn x_rotation_keeps_roll() {
        // 90 deg about X -> roll = 90 deg.
        let s = FRAC_PI_4.sin();
        let c = FRAC_PI_4.cos();
        let ned = transform_pose([0.0; 3], [s, 0.0, 0.0, c]);
        let (r, p, y) = quat_to_euler_frd(ned.q);
        assert_near(r, FRAC_PI_2, "roll");
        assert_near(p, 0.0, "pitch");
        assert_near(y, 0.0, "yaw");
    }

    #[test]
    fn gimbal_lock_singularities() {
        // At the singularity the result must be ~±pi/2 and never NaN.
        let (_, p, _) = quat_to_euler_frd([0.71, 0.0, 0.71, 0.0]);
        assert!((p - FRAC_PI_2).abs() < 1e-3, "pitch clamped +: {p}");
        let (_, p, _) = quat_to_euler_frd([0.71, 0.0, -0.71, 0.0]);
        assert!((p + FRAC_PI_2).abs() < 1e-3, "pitch clamped -: {p}");

        // At pitch = +90 deg exactly -> pitch = pi/2.
        let s = FRAC_PI_4.sin();
        let c = FRAC_PI_4.cos();
        let (_, p, _) = quat_to_euler_frd([c, 0.0, s, 0.0]);
        assert!((p - FRAC_PI_2).abs() < 1e-3, "near-lock pitch: {p}");
    }

}
