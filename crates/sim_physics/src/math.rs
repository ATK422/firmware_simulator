//! Vectors and rotations, as plain arrays: positions and velocities are `[x, y, z]` in metres and m/s.
//!
//! The world frame is north-east-down (`z` is depth, positive down, the same as the acoustic environments) and the
//! body frame is forward-starboard-down, both right-handed. An attitude is the unit quaternion that turns body
//! vectors into world vectors.

pub type Vec3 = [f64; 3];

pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale(a: Vec3, k: f64) -> Vec3 {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// Element-wise product (a diagonal matrix times a vector).
pub fn mul(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

/// `a` scaled to unit length (zero stays zero).
pub fn unit(a: Vec3) -> Vec3 {
    let n = norm(a);
    if n == 0.0 { a } else { scale(a, 1.0 / n) }
}

/// A rotation as a unit quaternion `w + xi + yj + zk`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    /// From roll, pitch and yaw (radians; the aerospace z-y-x sequence: yaw about down, then pitch about the new
    /// starboard axis, then roll about the new forward axis).
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Quat {
        let (sr, cr) = (roll / 2.0).sin_cos();
        let (sp, cp) = (pitch / 2.0).sin_cos();
        let (sy, cy) = (yaw / 2.0).sin_cos();
        Quat {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// Roll, pitch, yaw in radians (yaw in -pi..pi; pitch saturates at +-pi/2).
    pub fn euler(&self) -> Vec3 {
        let Quat { w, x, y, z } = *self;
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        [roll, pitch, yaw]
    }

    pub fn conj(&self) -> Quat {
        Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }

    /// The Hamilton product `self * o` (rotate by `o`, then by `self`).
    pub fn mul(&self, o: &Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }

    /// `v` rotated by this quaternion (body to world for an attitude).
    pub fn rotate(&self, v: Vec3) -> Vec3 {
        // v + 2 q_v x (q_v x v + w v)
        let q = [self.x, self.y, self.z];
        let t = add(cross(q, v), scale(v, self.w));
        add(v, scale(cross(q, t), 2.0))
    }

    /// `v` rotated back (world to body for an attitude).
    pub fn rotate_inv(&self, v: Vec3) -> Vec3 {
        self.conj().rotate(v)
    }

    pub fn normalized(&self) -> Quat {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        Quat { w: self.w / n, x: self.x / n, y: self.y / n, z: self.z / n }
    }

    /// The rate of change of an attitude turning at body rate `omega` (rad/s): `q' = q (0, omega) / 2`.
    pub fn derivative(&self, omega: Vec3) -> Quat {
        let d = self.mul(&Quat { w: 0.0, x: omega[0], y: omega[1], z: omega[2] });
        Quat { w: d.w / 2.0, x: d.x / 2.0, y: d.y / 2.0, z: d.z / 2.0 }
    }

    /// `self + d * k` (an integration step; not normalized).
    pub fn add_scaled(&self, d: &Quat, k: f64) -> Quat {
        Quat { w: self.w + d.w * k, x: self.x + d.x * k, y: self.y + d.y * k, z: self.z + d.z * k }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) -> bool {
        norm(sub(a, b)) < 1e-12
    }

    #[test]
    fn vector_algebra() {
        assert_eq!(cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]);
        assert_eq!(dot([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]), 32.0);
        assert_eq!(mul([1.0, 2.0, 3.0], [2.0, 2.0, 2.0]), [2.0, 4.0, 6.0]);
        assert_eq!(norm(unit([3.0, 0.0, 4.0])), 1.0);
        assert_eq!(unit([0.0; 3]), [0.0; 3]);
    }

    #[test]
    fn euler_angles_rotate_body_axes() {
        // Yawed 90 degrees: forward points east, starboard points south
        let q = Quat::from_euler(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        assert!(close(q.rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]));
        assert!(close(q.rotate([0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0]));
        // Pitched nose up 90 degrees: forward points up (negative z)
        let q = Quat::from_euler(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        assert!(close(q.rotate([1.0, 0.0, 0.0]), [0.0, 0.0, -1.0]));
        // Rolled to starboard 90 degrees: starboard points down
        let q = Quat::from_euler(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
        assert!(close(q.rotate([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]));
        // Round trips
        let q = Quat::from_euler(0.3, -0.2, 2.5);
        let e = q.euler();
        assert!(close(e, [0.3, -0.2, 2.5]), "{e:?}");
        assert!(close(q.rotate_inv(q.rotate([0.1, 0.2, 0.3])), [0.1, 0.2, 0.3]));
        assert!(close(q.mul(&q.conj()).rotate([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]));
    }

    #[test]
    fn integrating_a_body_rate_turns_the_attitude() {
        // 1 rad/s about the down axis for 1 s: yaw 1 rad
        let mut q = Quat::IDENTITY;
        for _ in 0..1000 {
            q = q.add_scaled(&q.derivative([0.0, 0.0, 1.0]), 1e-3).normalized();
        }
        assert!((q.euler()[2] - 1.0).abs() < 1e-3, "{:?}", q.euler());
    }
}
