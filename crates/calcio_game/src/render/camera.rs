//! Broadcast-style camera on the side of the pitch, plus projection helpers.

use calcio_core::math::{approach_factor, clamp, v3, Vec3};
use calcio_core::pitch::Side;
use macroquad::prelude::{vec3, Camera3D};

#[derive(Clone, Debug)]
pub struct CameraRig {
    pub position: Vec3,
    pub target: Vec3,
    /// Vertical field of view (radians).
    pub fovy: f32,
    /// Height above the pitch.
    pub height: f32,
    /// Distance from the pitch centre line (z) to the camera.
    pub distance: f32,
    /// Smoothed focus point and its velocity (critically damped spring).
    focus: Vec3,
    focus_vel: Vec3,
    zoom: f32,
    shake: f32,
    shake_t: f32,
}

/// What the camera should look at this frame.
#[derive(Clone, Copy, Debug)]
pub struct CamTarget {
    pub ball: Vec3,
    pub ball_vel: Vec3,
    /// The human player's position, kept in frame.
    pub human: Option<Vec3>,
    /// Set while celebrating a goal scored into this side.
    pub goal: Option<Side>,
}

impl CameraRig {
    /// Elevated sideline view looking across the pitch towards -z.
    pub fn broadcast() -> Self {
        let mut c = Self {
            position: Vec3::ZERO,
            target: Vec3::ZERO,
            fovy: 34.0f32.to_radians(),
            height: 14.0,
            distance: 27.0,
            focus: Vec3::ZERO,
            focus_vel: Vec3::ZERO,
            zoom: 1.0,
            shake: 0.0,
            shake_t: 0.0,
        };
        c.snap(Vec3::ZERO);
        c
    }

    fn desired(&self, focus: Vec3) -> (Vec3, Vec3) {
        let tx = clamp(focus.x, -14.5, 14.5);
        let tz = clamp(focus.z * 0.35, -3.0, 3.0);
        let target = v3(tx, 0.4 + clamp(focus.y * 0.25, 0.0, 1.5), tz - 1.0);
        let position = v3(
            tx * 0.88,
            self.height * self.zoom,
            self.distance * self.zoom,
        );
        (position, target)
    }

    /// Jumps straight to the framing for `focus`.
    pub fn snap(&mut self, focus: Vec3) {
        self.focus = focus;
        self.focus_vel = Vec3::ZERO;
        self.zoom = 1.0;
        let (p, t) = self.desired(focus);
        self.position = p;
        self.target = t;
    }

    /// Adds a short shake (e.g. ball hitting the post).
    pub fn kick_shake(&mut self, amount: f32) {
        self.shake = self.shake.max(amount.min(0.25));
    }

    /// Broadcast framing: look ahead of the ball, keep the human in shot, zoom in
    /// near the goals and on goals, all with spring smoothing.
    pub fn update(&mut self, t: &CamTarget, dt: f32) {
        let ahead = (t.ball_vel.flat() * 0.35).clamp_length(4.5);
        let mut want = t.ball + ahead;
        if let Some(h) = t.human {
            // Pull towards the human so he doesn't fall off the screen.
            let d = (h - want).flat();
            want += d.clamp_length(9.0) * 0.22;
        }
        let mut zoom = 1.0 - 0.14 * clamp((t.ball.x.abs() - 10.0) / 8.0, 0.0, 1.0);
        if let Some(side) = t.goal {
            want = v3(side.sign() * 18.5, 0.6, 0.0);
            zoom = 0.68;
        }
        // Critically damped spring towards `want`.
        let omega = 3.2;
        let x = self.focus - want;
        let k = (-omega * dt).exp();
        let new_x = (x + (self.focus_vel + x * omega) * dt) * k;
        self.focus_vel = (self.focus_vel - (self.focus_vel + x * omega) * (omega * dt)) * k;
        self.focus = want + new_x;
        self.zoom += (zoom - self.zoom) * approach_factor(1.5, dt);
        let (p, tg) = self.desired(self.focus);
        let k = approach_factor(6.0, dt);
        self.position = self.position.lerp(p, k);
        self.target = self.target.lerp(tg, k);
        if self.shake > 0.001 {
            self.shake_t += dt;
            let s = self.shake;
            self.position += v3(
                (self.shake_t * 53.0).sin() * s,
                (self.shake_t * 71.0).cos() * s * 0.6,
                0.0,
            );
            self.shake *= (-6.0 * dt).exp();
        }
    }

    /// Smoothly follows `focus` (usually the ball).
    pub fn follow(&mut self, focus: Vec3, dt: f32) {
        let (p, t) = self.desired(focus);
        let k = approach_factor(3.0, dt);
        self.position = self.position.lerp(p, k);
        self.target = self.target.lerp(t, k);
    }

    pub fn to_mq(&self) -> Camera3D {
        Camera3D {
            position: vec3(self.position.x, self.position.y, self.position.z),
            target: vec3(self.target.x, self.target.y, self.target.z),
            up: vec3(0.0, 1.0, 0.0),
            fovy: self.fovy,
            ..Default::default()
        }
    }

    /// Screen-right and screen-up directions projected on the ground (unit vectors).
    /// Used to map keyboard input to world directions.
    pub fn ground_axes(&self) -> (Vec3, Vec3) {
        let fwd = (self.target - self.position).flat().normalize_or(-Vec3::Z);
        let right = v3(-fwd.z, 0.0, fwd.x);
        (right, fwd)
    }

    /// Projects a world point to screen pixels. `None` if behind the camera.
    pub fn project(&self, p: Vec3, screen_w: f32, screen_h: f32) -> Option<(f32, f32)> {
        let f = (self.target - self.position).normalize_or(-Vec3::Z);
        let r = f.cross(Vec3::Y).normalize_or(Vec3::X);
        let u = r.cross(f);
        let d = p - self.position;
        let z = d.dot(f);
        if z <= 0.05 {
            return None;
        }
        let aspect = screen_w / screen_h.max(1.0);
        let th = (self.fovy * 0.5).tan();
        let nx = d.dot(r) / (z * th * aspect);
        let ny = d.dot(u) / (z * th);
        Some(((nx * 0.5 + 0.5) * screen_w, (0.5 - ny * 0.5) * screen_h))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn target_projects_to_centre() {
        let c = CameraRig::broadcast();
        let (x, y) = c.project(c.target, 1600.0, 900.0).unwrap();
        assert!((x - 800.0).abs() < 0.5 && (y - 450.0).abs() < 0.5);
    }

    #[test]
    fn spring_follow_converges_and_is_stable() {
        let mut c = CameraRig::broadcast();
        let t = CamTarget {
            ball: v3(10.0, 0.11, 4.0),
            ball_vel: Vec3::ZERO,
            human: None,
            goal: None,
        };
        for dt in [0.004f32, 0.016, 0.05, 0.1] {
            for _ in 0..400 {
                c.update(&t, dt);
                assert!(c.position.is_finite() && c.target.is_finite());
            }
        }
        assert!((c.target.x - 10.0).abs() < 0.2, "target {:?}", c.target);
        let (x, _) = c.project(t.ball, 1600.0, 900.0).unwrap();
        assert!((x - 800.0).abs() < 200.0);
    }

    #[test]
    fn ground_axes_match_screen() {
        let c = CameraRig::broadcast();
        let (right, up) = c.ground_axes();
        // camera sits at +z looking towards -z: screen right is +x, screen up is -z
        assert!(right.x > 0.99 && up.z < -0.99);
        let (x0, _) = c.project(c.target, 1600.0, 900.0).unwrap();
        let (x1, _) = c.project(c.target + right, 1600.0, 900.0).unwrap();
        assert!(x1 > x0);
    }
}
