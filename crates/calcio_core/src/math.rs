//! Small, dependency-free 3D math: vectors, quaternions, scalar and geometry helpers.

use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// 3D vector (f32). Axes: x = pitch length, y = up, z = pitch width.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Shorthand constructor.
#[inline]
pub const fn v3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

impl Vec3 {
    pub const ZERO: Vec3 = v3(0.0, 0.0, 0.0);
    pub const ONE: Vec3 = v3(1.0, 1.0, 1.0);
    pub const X: Vec3 = v3(1.0, 0.0, 0.0);
    pub const Y: Vec3 = v3(0.0, 1.0, 0.0);
    pub const Z: Vec3 = v3(0.0, 0.0, 1.0);
    pub const UP: Vec3 = Vec3::Y;

    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        v3(x, y, z)
    }
    #[inline]
    pub const fn splat(s: f32) -> Self {
        v3(s, s, s)
    }
    #[inline]
    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    #[inline]
    pub fn cross(self, o: Vec3) -> Vec3 {
        v3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    #[inline]
    pub fn length_sq(self) -> f32 {
        self.dot(self)
    }
    #[inline]
    pub fn length(self) -> f32 {
        self.length_sq().sqrt()
    }
    #[inline]
    pub fn distance(self, o: Vec3) -> f32 {
        (self - o).length()
    }
    #[inline]
    pub fn distance_sq(self, o: Vec3) -> f32 {
        (self - o).length_sq()
    }
    /// Unit vector, or zero if the length is (almost) zero.
    #[inline]
    pub fn normalize_or_zero(self) -> Vec3 {
        let l = self.length();
        if l > 1e-9 {
            self / l
        } else {
            Vec3::ZERO
        }
    }
    /// Unit vector, or `fallback` if the length is (almost) zero.
    #[inline]
    pub fn normalize_or(self, fallback: Vec3) -> Vec3 {
        let l = self.length();
        if l > 1e-9 {
            self / l
        } else {
            fallback
        }
    }
    #[inline]
    pub fn lerp(self, o: Vec3, t: f32) -> Vec3 {
        self + (o - self) * t
    }
    #[inline]
    pub fn mul_elem(self, o: Vec3) -> Vec3 {
        v3(self.x * o.x, self.y * o.y, self.z * o.z)
    }
    #[inline]
    pub fn min(self, o: Vec3) -> Vec3 {
        v3(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }
    #[inline]
    pub fn max(self, o: Vec3) -> Vec3 {
        v3(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }
    #[inline]
    pub fn abs(self) -> Vec3 {
        v3(self.x.abs(), self.y.abs(), self.z.abs())
    }
    /// Vector with the same length but no more than `max`.
    #[inline]
    pub fn clamp_length(self, max: f32) -> Vec3 {
        let l2 = self.length_sq();
        if l2 > max * max {
            self * (max / l2.sqrt())
        } else {
            self
        }
    }
    /// Ground-plane projection (y = 0).
    #[inline]
    pub fn flat(self) -> Vec3 {
        v3(self.x, 0.0, self.z)
    }
    /// Length of the ground-plane projection.
    #[inline]
    pub fn flat_length(self) -> f32 {
        (self.x * self.x + self.z * self.z).sqrt()
    }
    /// Ground-plane distance.
    #[inline]
    pub fn flat_distance(self, o: Vec3) -> f32 {
        (self - o).flat_length()
    }
    /// Unit ground-plane direction or zero.
    #[inline]
    pub fn flat_dir(self) -> Vec3 {
        self.flat().normalize_or_zero()
    }
    /// Heading angle on the ground plane: 0 = +x, π/2 = +z.
    #[inline]
    pub fn heading(self) -> f32 {
        self.z.atan2(self.x)
    }
    /// Unit ground vector from a heading angle (inverse of [`Vec3::heading`]).
    #[inline]
    pub fn from_heading(angle: f32) -> Vec3 {
        v3(angle.cos(), 0.0, angle.sin())
    }
    /// Rotate around the vertical axis by `angle` (same convention as heading).
    #[inline]
    pub fn rotate_y(self, angle: f32) -> Vec3 {
        let (s, c) = angle.sin_cos();
        v3(self.x * c - self.z * s, self.y, self.x * s + self.z * c)
    }
    /// Component of `self` perpendicular to unit vector `n`.
    #[inline]
    pub fn reject(self, n: Vec3) -> Vec3 {
        self - n * self.dot(n)
    }
    /// Any unit vector perpendicular to `self` (which must be non-zero).
    pub fn any_perpendicular(self) -> Vec3 {
        let a = if self.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        self.cross(a).normalize_or(Vec3::Z)
    }
    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    #[inline]
    fn add(self, o: Vec3) -> Vec3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    #[inline]
    fn sub(self, o: Vec3) -> Vec3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Mul<f32> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, s: f32) -> Vec3 {
        v3(self.x * s, self.y * s, self.z * s)
    }
}
impl Mul<Vec3> for f32 {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}
impl Div<f32> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn div(self, s: f32) -> Vec3 {
        v3(self.x / s, self.y / s, self.z / s)
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    #[inline]
    fn neg(self) -> Vec3 {
        v3(-self.x, -self.y, -self.z)
    }
}
impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}
impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, o: Vec3) {
        *self = *self - o;
    }
}
impl MulAssign<f32> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, s: f32) {
        *self = *self * s;
    }
}
impl DivAssign<f32> for Vec3 {
    #[inline]
    fn div_assign(&mut self, s: f32) {
        *self = *self / s;
    }
}

/// Unit quaternion for orientations (w + xi + yj + zk).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Rotation of `angle` radians around unit `axis`.
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Quat {
        let a = axis.normalize_or(Vec3::Y);
        let (s, c) = (angle * 0.5).sin_cos();
        Quat {
            w: c,
            x: a.x * s,
            y: a.y * s,
            z: a.z * s,
        }
    }

    /// Rotation around the vertical axis using the heading convention of [`Vec3::heading`].
    pub fn from_heading(angle: f32) -> Quat {
        // Heading rotates +x towards +z, which is a negative rotation about +y.
        Quat::from_axis_angle(Vec3::Y, -angle)
    }

    pub fn length(self) -> f32 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(self) -> Quat {
        let l = self.length();
        if l < 1e-12 {
            return Quat::IDENTITY;
        }
        Quat {
            w: self.w / l,
            x: self.x / l,
            y: self.y / l,
            z: self.z / l,
        }
    }

    pub fn conjugate(self) -> Quat {
        Quat {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Rotates vector `v` by this (unit) quaternion.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let u = v3(self.x, self.y, self.z);
        let t = u.cross(v) * 2.0;
        v + t * self.w + u.cross(t)
    }

    /// Integrates angular velocity `omega` (rad/s, world frame) over `dt`.
    pub fn integrate(self, omega: Vec3, dt: f32) -> Quat {
        let speed = omega.length();
        if speed * dt < 1e-9 {
            return self;
        }
        (Quat::from_axis_angle(omega / speed, speed * dt) * self).normalize()
    }
}

impl Mul for Quat {
    type Output = Quat;
    fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }
}

// ---------------------------------------------------------------------------
// Scalar helpers
// ---------------------------------------------------------------------------

#[inline]
pub fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    x.max(lo).min(hi)
}

#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Hermite smoothstep of `x` between `e0` and `e1`.
#[inline]
pub fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Moves `current` towards `target` by at most `max_delta`.
#[inline]
pub fn move_towards(current: f32, target: f32, max_delta: f32) -> f32 {
    let d = target - current;
    if d.abs() <= max_delta {
        target
    } else {
        current + d.signum() * max_delta
    }
}

/// Wraps an angle into (-π, π].
#[inline]
pub fn wrap_angle(a: f32) -> f32 {
    let two_pi = std::f32::consts::TAU;
    let mut r = (a + std::f32::consts::PI) % two_pi;
    if r <= 0.0 {
        r += two_pi;
    }
    r - std::f32::consts::PI
}

/// Rotates angle `current` towards `target` by at most `max_delta` (shortest way).
#[inline]
pub fn rotate_towards(current: f32, target: f32, max_delta: f32) -> f32 {
    let d = wrap_angle(target - current);
    if d.abs() <= max_delta {
        wrap_angle(target)
    } else {
        wrap_angle(current + d.signum() * max_delta)
    }
}

/// Frame-rate independent exponential approach factor for smoothing.
#[inline]
pub fn approach_factor(rate: f32, dt: f32) -> f32 {
    1.0 - (-rate * dt).exp()
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/// Closest point on segment `a..b` to `p`, and its parameter t in [0, 1].
pub fn closest_point_on_segment(p: Vec3, a: Vec3, b: Vec3) -> (Vec3, f32) {
    let ab = b - a;
    let l2 = ab.length_sq();
    if l2 < 1e-12 {
        return (a, 0.0);
    }
    let t = clamp((p - a).dot(ab) / l2, 0.0, 1.0);
    (a + ab * t, t)
}

/// Closest points between segments `p1..q1` and `p2..q2`.
/// Returns (point on first, point on second, s, t).
pub fn closest_points_segments(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (Vec3, Vec3, f32, f32) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.length_sq();
    let e = d2.length_sq();
    let f = d2.dot(r);
    let eps = 1e-12;
    let (s, t);
    if a <= eps && e <= eps {
        return (p1, p2, 0.0, 0.0);
    }
    if a <= eps {
        s = 0.0;
        t = clamp(f / e, 0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= eps {
            t = 0.0;
            s = clamp(-c / a, 0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s0 = if denom > eps {
                clamp((b * f - c * e) / denom, 0.0, 1.0)
            } else {
                0.0
            };
            let mut t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t0 = 0.0;
                s0 = clamp(-c / a, 0.0, 1.0);
            } else if t0 > 1.0 {
                t0 = 1.0;
                s0 = clamp((b - c) / a, 0.0, 1.0);
            }
            s = s0;
            t = t0;
        }
    }
    (p1 + d1 * s, p2 + d2 * t, s, t)
}

/// Closest point on triangle `abc` to `p` and its barycentric weights (wa, wb, wc).
pub fn closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> (Vec3, [f32; 3]) {
    // Ericson, Real-Time Collision Detection, 5.1.5
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, [1.0, 0.0, 0.0]);
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, [0.0, 1.0, 0.0]);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v, [1.0 - v, v, 0.0]);
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, [0.0, 0.0, 1.0]);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w, [1.0 - w, 0.0, w]);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, [0.0, 1.0 - w, w]);
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (a + ab * v + ac * w, [1.0 - v - w, v, w])
}

/// Distance from point `p` to the infinite line through `a` with unit direction `dir`,
/// together with the signed position along the line.
pub fn point_line_distance(p: Vec3, a: Vec3, dir: Vec3) -> (f32, f32) {
    let ap = p - a;
    let along = ap.dot(dir);
    ((ap - dir * along).length(), along)
}

// ---------------------------------------------------------------------------
// Deterministic RNG
// ---------------------------------------------------------------------------

/// Small, fast, seedable PRNG (xorshift64*). Deterministic across platforms.
#[derive(Clone, Debug)]
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        // Avoid the all-zero state and decorrelate small seeds.
        let mut s = seed ^ 0x9E37_79B9_7F4A_7C15;
        if s == 0 {
            s = 0x1234_5678_9ABC_DEF1;
        }
        let mut r = Rng { state: s };
        for _ in 0..4 {
            r.next_u64();
        }
        r
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in [0, 1).
    pub fn f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    /// Uniform in [lo, hi).
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.f32()
    }

    /// Uniform integer in [0, n).
    pub fn index(&mut self, n: usize) -> usize {
        if n == 0 {
            0
        } else {
            (self.next_u64() % n as u64) as usize
        }
    }

    /// Standard normal sample (Box–Muller).
    pub fn gaussian(&mut self) -> f32 {
        let u1 = self.f32().max(1e-7);
        let u2 = self.f32();
        (-2.0 * u1.ln()).sqrt() * (std::f32::consts::TAU * u2).cos()
    }

    /// True with probability `p`.
    pub fn chance(&mut self, p: f32) -> bool {
        self.f32() < p
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::{FRAC_PI_2, PI};

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }
    fn vclose(a: Vec3, b: Vec3) -> bool {
        a.distance(b) < 1e-4
    }

    #[test]
    fn vec_basics() {
        let a = v3(1.0, 2.0, 3.0);
        let b = v3(-2.0, 0.5, 4.0);
        assert!(vclose(a + b, v3(-1.0, 2.5, 7.0)));
        assert!(vclose(a - b, v3(3.0, 1.5, -1.0)));
        assert!(close(a.dot(b), -2.0 + 1.0 + 12.0));
        assert!(vclose(Vec3::X.cross(Vec3::Y), Vec3::Z));
        assert!(close(v3(3.0, 4.0, 0.0).length(), 5.0));
        assert!(close(v3(0.0, 0.0, 9.0).normalize_or_zero().length(), 1.0));
        assert_eq!(Vec3::ZERO.normalize_or_zero(), Vec3::ZERO);
        assert!(vclose(a.lerp(b, 0.5), (a + b) * 0.5));
        assert!(close(v3(3.0, 7.0, 4.0).flat_length(), 5.0));
        assert!(close(v3(30.0, 0.0, 40.0).clamp_length(5.0).length(), 5.0));
    }

    #[test]
    fn heading_roundtrip() {
        for i in 0..16 {
            let a = wrap_angle(i as f32 * 0.4 - 3.0);
            let v = Vec3::from_heading(a);
            assert!(close(wrap_angle(v.heading() - a), 0.0));
            assert!(vclose(Vec3::X.rotate_y(a), v));
        }
        assert!(vclose(Vec3::from_heading(FRAC_PI_2), Vec3::Z));
    }

    #[test]
    fn quat_rotation() {
        let q = Quat::from_axis_angle(Vec3::Z, FRAC_PI_2);
        assert!(vclose(q.rotate(Vec3::X), Vec3::Y));
        let q2 = Quat::from_axis_angle(Vec3::Y, PI);
        assert!(vclose(q2.rotate(Vec3::X), -Vec3::X));
        // composition: rotate by q then q2
        let c = q2 * q;
        assert!(vclose(c.rotate(Vec3::X), q2.rotate(q.rotate(Vec3::X))));
        // heading quaternion matches Vec3::rotate_y
        let h = Quat::from_heading(0.7);
        assert!(vclose(h.rotate(Vec3::X), Vec3::X.rotate_y(0.7)));
    }

    #[test]
    fn quat_integration_matches_axis_angle() {
        let omega = v3(0.0, 0.0, 10.0);
        let mut q = Quat::IDENTITY;
        let dt = 1.0 / 240.0;
        for _ in 0..24 {
            q = q.integrate(omega, dt);
        }
        // 10 rad/s for 0.1 s = 1 rad around z
        let expected = Quat::from_axis_angle(Vec3::Z, 1.0).rotate(Vec3::X);
        assert!(vclose(q.rotate(Vec3::X), expected));
        assert!(close(q.length(), 1.0));
    }

    #[test]
    fn scalar_helpers() {
        assert!(close(clamp(5.0, 0.0, 1.0), 1.0));
        assert!(close(lerp(2.0, 4.0, 0.25), 2.5));
        assert!(close(smoothstep(0.0, 1.0, 0.5), 0.5));
        assert!(close(move_towards(0.0, 10.0, 3.0), 3.0));
        assert!(close(move_towards(9.0, 10.0, 3.0), 10.0));
        assert!(close(wrap_angle(3.0 * PI), PI));
        assert!(close(wrap_angle(-3.5 * PI), 0.5 * PI));
        // shortest path across the ±π seam
        let r = rotate_towards(3.0, -3.0, 0.1);
        assert!(r > 3.0 || r < -3.0);
        assert!(close(rotate_towards(0.0, 1.0, 2.0), 1.0));
        assert!(approach_factor(10.0, 0.016) > 0.0 && approach_factor(10.0, 0.016) < 1.0);
    }

    #[test]
    fn segment_queries() {
        let (p, t) = closest_point_on_segment(v3(0.5, 1.0, 0.0), Vec3::ZERO, Vec3::X);
        assert!(vclose(p, v3(0.5, 0.0, 0.0)) && close(t, 0.5));
        let (p, t) = closest_point_on_segment(v3(5.0, 1.0, 0.0), Vec3::ZERO, Vec3::X);
        assert!(vclose(p, Vec3::X) && close(t, 1.0));

        // crossing segments, 1 apart in z
        let (a, b, s, t) = closest_points_segments(
            v3(-1.0, 0.0, 0.0),
            v3(1.0, 0.0, 0.0),
            v3(0.0, -1.0, 1.0),
            v3(0.0, 1.0, 1.0),
        );
        assert!(vclose(a, Vec3::ZERO) && vclose(b, v3(0.0, 0.0, 1.0)));
        assert!(close(s, 0.5) && close(t, 0.5));
        // parallel segments
        let (a, b, _, _) =
            closest_points_segments(Vec3::ZERO, Vec3::X, v3(0.0, 1.0, 0.0), v3(1.0, 1.0, 0.0));
        assert!(close(a.distance(b), 1.0));

        let (q, w) = closest_point_on_triangle(v3(0.2, 5.0, 0.2), Vec3::ZERO, Vec3::X, Vec3::Z);
        assert!(vclose(q, v3(0.2, 0.0, 0.2)) && close(w[0] + w[1] + w[2], 1.0));
        let (q, w) = closest_point_on_triangle(v3(-1.0, 0.0, -1.0), Vec3::ZERO, Vec3::X, Vec3::Z);
        assert!(vclose(q, Vec3::ZERO) && close(w[0], 1.0));
        let (q, _) = closest_point_on_triangle(v3(1.0, 1.0, 1.0), Vec3::ZERO, Vec3::X, Vec3::Z);
        assert!(vclose(q, v3(0.5, 0.0, 0.5)));

        let (d, along) = point_line_distance(v3(2.0, 3.0, 0.0), Vec3::ZERO, Vec3::X);
        assert!(close(d, 3.0) && close(along, 2.0));
    }

    #[test]
    fn rng_is_deterministic_and_in_range() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
        let mut r = Rng::new(7);
        let mut sum = 0.0;
        let mut sq = 0.0;
        let n = 20000;
        for _ in 0..n {
            let x = r.f32();
            assert!((0.0..1.0).contains(&x));
            let y = r.range(-2.0, 3.0);
            assert!((-2.0..3.0).contains(&y));
            let g = r.gaussian();
            sum += g;
            sq += g * g;
        }
        let mean = sum / n as f32;
        let var = sq / n as f32 - mean * mean;
        assert!(mean.abs() < 0.05, "gaussian mean {mean}");
        assert!((var - 1.0).abs() < 0.08, "gaussian var {var}");
        assert!(r.index(5) < 5);
    }
}
