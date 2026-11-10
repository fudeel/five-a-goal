//! The football: state and aerodynamics.
//!
//! Forces in flight:
//! * gravity
//! * quadratic drag `F = ½ ρ C_d A v²` with a smooth **drag crisis** (C_d drops when
//!   the boundary layer turns turbulent, so hard shots travel further than slow lobs)
//! * **Magnus** lift `F = ½ ρ C_L A v²` along `ω × v` (curl, dip, float)
//! * spin decay from air friction
//!
//! Contacts (ground, boards, frame, net, bodies) live in their own modules.

use crate::math::{lerp, smoothstep, v3, Quat, Vec3};

pub const GRAVITY: f32 = 9.81;
pub const AIR_DENSITY: f32 = 1.225;

/// Physical properties of the ball.
#[derive(Clone, Debug)]
pub struct BallSpec {
    pub radius: f32,
    pub mass: f32,
    /// Moment of inertia factor: I = k·m·r² (thin hollow sphere ≈ 2/3).
    pub inertia_k: f32,
    /// Drag coefficient below the drag crisis.
    pub cd_slow: f32,
    /// Drag coefficient above the drag crisis.
    pub cd_fast: f32,
    /// Speed range (m/s) over which the drag crisis happens.
    pub crisis_from: f32,
    pub crisis_to: f32,
    /// Lift coefficient model: C_L = lift_gain·S / (1 + lift_sat·S), S = rω/v.
    pub lift_gain: f32,
    pub lift_sat: f32,
    /// Spin decay rate in the air: base + per (m/s) of speed (1/s).
    pub spin_decay_base: f32,
    pub spin_decay_speed: f32,
}

impl Default for BallSpec {
    fn default() -> Self {
        Self {
            radius: 0.11,
            mass: 0.43,
            inertia_k: 2.0 / 3.0,
            cd_slow: 0.45,
            cd_fast: 0.24,
            crisis_from: 9.0,
            crisis_to: 15.0,
            lift_gain: 1.0,
            lift_sat: 1.2,
            spin_decay_base: 0.08,
            spin_decay_speed: 0.006,
        }
    }
}

impl BallSpec {
    #[inline]
    pub fn area(&self) -> f32 {
        std::f32::consts::PI * self.radius * self.radius
    }
    #[inline]
    pub fn inertia(&self) -> f32 {
        self.inertia_k * self.mass * self.radius * self.radius
    }
    /// Drag coefficient at `speed` (m/s).
    pub fn drag_coefficient(&self, speed: f32) -> f32 {
        lerp(
            self.cd_slow,
            self.cd_fast,
            smoothstep(self.crisis_from, self.crisis_to, speed),
        )
    }
    /// Lift coefficient for spin parameter `s = rω/v`.
    pub fn lift_coefficient(&self, s: f32) -> f32 {
        let s = s.max(0.0);
        self.lift_gain * s / (1.0 + self.lift_sat * s)
    }
    /// Aerodynamic + gravity acceleration for velocity `v` and spin `w`.
    pub fn acceleration(&self, v: Vec3, w: Vec3) -> Vec3 {
        let mut a = v3(0.0, -GRAVITY, 0.0);
        let speed = v.length();
        if speed < 1e-4 {
            return a;
        }
        let q = 0.5 * AIR_DENSITY * self.area() * speed * speed / self.mass;
        a -= v * (q * self.drag_coefficient(speed) / speed);
        let wxv = w.cross(v);
        let wxv_len = wxv.length();
        if wxv_len > 1e-6 {
            let s = self.radius * w.length() / speed;
            a += wxv * (q * self.lift_coefficient(s) / wxv_len);
        }
        a
    }
}

/// Dynamic state of the ball.
#[derive(Clone, Debug)]
pub struct Ball {
    pub pos: Vec3,
    pub vel: Vec3,
    /// Angular velocity (rad/s, world frame).
    pub spin: Vec3,
    /// Orientation (for rendering the panel pattern rotating).
    pub orient: Quat,
    /// Position at the start of the last step (for crossing tests).
    pub prev_pos: Vec3,
    /// True while resting/rolling on the ground (set by the ground contact).
    pub grounded: bool,
}

impl Ball {
    pub fn new(pos: Vec3) -> Self {
        Self {
            pos,
            vel: Vec3::ZERO,
            spin: Vec3::ZERO,
            orient: Quat::IDENTITY,
            prev_pos: pos,
            grounded: false,
        }
    }

    /// Places the ball at rest on the ground at `(x, z)`.
    pub fn place(&mut self, spec: &BallSpec, x: f32, z: f32) {
        self.pos = v3(x, spec.radius, z);
        self.prev_pos = self.pos;
        self.vel = Vec3::ZERO;
        self.spin = Vec3::ZERO;
        self.grounded = true;
    }

    #[inline]
    pub fn speed(&self) -> f32 {
        self.vel.length()
    }

    /// Height of the bottom of the ball above the ground.
    #[inline]
    pub fn height(&self, spec: &BallSpec) -> f32 {
        self.pos.y - spec.radius
    }

    /// Integrates gravity and aerodynamics over `dt` (midpoint method), decays spin
    /// and advances the orientation. Contacts are resolved separately.
    pub fn integrate_flight(&mut self, spec: &BallSpec, dt: f32) {
        self.prev_pos = self.pos;
        let v0 = self.vel;
        let a0 = spec.acceleration(v0, self.spin);
        let v_mid = v0 + a0 * (0.5 * dt);
        let a1 = spec.acceleration(v_mid, self.spin);
        let v1 = v0 + a1 * dt;
        self.pos += (v0 + v1) * (0.5 * dt);
        self.vel = v1;
        if !self.grounded {
            let decay = spec.spin_decay_base + spec.spin_decay_speed * v1.length();
            self.spin *= (-decay * dt).exp();
        }
        self.orient = self.orient.integrate(self.spin, dt);
    }

    /// Integrates rotation only (used while the ground contact controls motion).
    pub fn integrate_orientation(&mut self, dt: f32) {
        self.orient = self.orient.integrate(self.spin, dt);
    }

    pub fn is_finite(&self) -> bool {
        self.pos.is_finite() && self.vel.is_finite() && self.spin.is_finite()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PHYSICS_DT;

    /// Flies a ball (no contacts) until it comes back down to y = r. Returns
    /// (landing point, max height, flight time).
    fn fly(spec: &BallSpec, vel: Vec3, spin: Vec3) -> (Vec3, f32, f32) {
        let mut b = Ball::new(v3(0.0, spec.radius, 0.0));
        b.vel = vel;
        b.spin = spin;
        let mut t = 0.0;
        let mut apex: f32 = 0.0;
        loop {
            b.integrate_flight(spec, PHYSICS_DT);
            t += PHYSICS_DT;
            apex = apex.max(b.pos.y);
            if (b.pos.y <= spec.radius && b.vel.y < 0.0) || t > 20.0 {
                return (b.pos, apex, t);
            }
        }
    }

    #[test]
    fn drag_crisis() {
        let s = BallSpec::default();
        assert!(s.drag_coefficient(5.0) > 0.4);
        assert!(s.drag_coefficient(25.0) < 0.26);
        assert!(s.drag_coefficient(12.0) < s.drag_coefficient(8.0));
    }

    #[test]
    fn no_spin_no_lateral_drift() {
        let s = BallSpec::default();
        let (land, _, _) = fly(&s, v3(20.0, 6.0, 0.0), Vec3::ZERO);
        assert!(land.z.abs() < 1e-4);
        assert!(land.x > 5.0);
    }

    #[test]
    fn drag_shortens_range() {
        let s = BallSpec::default();
        let (v, vy) = (22.0f32, 10.0f32);
        let (land, _, _) = fly(&s, v3(v, vy, 0.0), Vec3::ZERO);
        let vacuum = v * 2.0 * vy / GRAVITY;
        assert!(
            land.x < vacuum * 0.9,
            "range {} vs vacuum {}",
            land.x,
            vacuum
        );
        assert!(land.x > vacuum * 0.5);
    }

    #[test]
    fn topspin_dips_backspin_floats() {
        let s = BallSpec::default();
        let vel = v3(20.0, 7.0, 0.0);
        // For travel along +x: ω_z < 0 is topspin, ω_z > 0 is backspin.
        let (_, apex_none, t_none) = fly(&s, vel, Vec3::ZERO);
        let (_, apex_top, t_top) = fly(&s, vel, v3(0.0, 0.0, -50.0));
        let (_, apex_back, t_back) = fly(&s, vel, v3(0.0, 0.0, 50.0));
        assert!(apex_top < apex_none && t_top < t_none);
        assert!(apex_back > apex_none && t_back > t_none);
    }

    #[test]
    fn sidespin_curves_the_right_way() {
        let s = BallSpec::default();
        // ω_y > 0 (counter-clockwise seen from above) with travel along +x curls towards -z.
        let (land, _, _) = fly(&s, v3(24.0, 5.0, 0.0), v3(0.0, 60.0, 0.0));
        assert!(land.z < -0.5, "curl {}", land.z);
        let (land, _, _) = fly(&s, v3(24.0, 5.0, 0.0), v3(0.0, -60.0, 0.0));
        assert!(land.z > 0.5);
    }

    #[test]
    fn realistic_curl_magnitude() {
        // A 25 m/s curler with ~8 rev/s should bend a few metres over ~20 m.
        let s = BallSpec::default();
        let mut b = Ball::new(v3(0.0, 0.5, 0.0));
        b.vel = v3(25.0, 3.0, 0.0);
        b.spin = v3(0.0, 50.0, 0.0);
        while b.pos.x < 20.0 {
            b.integrate_flight(&s, PHYSICS_DT);
        }
        assert!(b.pos.z < -1.0 && b.pos.z > -6.0, "bend {}", b.pos.z);
    }

    #[test]
    fn spin_decays_and_orientation_turns() {
        let s = BallSpec::default();
        let mut b = Ball::new(v3(0.0, 5.0, 0.0));
        b.vel = v3(15.0, 0.0, 0.0);
        b.spin = v3(0.0, 0.0, 30.0);
        for _ in 0..240 {
            b.integrate_flight(&s, PHYSICS_DT);
        }
        assert!(b.spin.z < 30.0 && b.spin.z > 20.0);
        assert!(b.orient.rotate(Vec3::X).distance(Vec3::X) > 0.1);
        assert!(b.is_finite());
    }
}
