//! Locomotion: gait parameters from running biomechanics and the foot
//! trajectories that drive the legs, in any direction relative to the body:
//! forward walk / jog / run / sprint, backpedalling and side shuffles, and every
//! blend in between.
//!
//! The stance foot moves against the direction of travel at exactly the ground
//! speed, so feet don't slide; the swing foot follows a walk, run, backpedal or
//! shuffle curve. The legs can be turned relative to the upper body
//! (`hip_turn`): a player running sideways at speed opens the hips towards the
//! run while the chest and head stay on the ball.

use super::body::Body;
use super::pose::{ArmPose, Pose};
use calcio_core::math::{clamp, lerp, smoothstep, v3};
use std::f32::consts::TAU;

/// Piecewise-linear lookup in a table of (x, y) points.
fn table(pts: &[(f32, f32)], x: f32) -> f32 {
    if x <= pts[0].0 {
        return pts[0].1;
    }
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        if x <= b.0 {
            return lerp(a.1, b.1, (x - a.0) / (b.0 - a.0));
        }
    }
    pts[pts.len() - 1].1
}

/// Forward gait cycles per second (one cycle = two steps) at ground speed `v`
/// (m/s). Walk ≈ 0.9 Hz (1.8 steps/s), jog ≈ 1.4 Hz (2.8 steps/s), sprint ≈ 2 Hz.
pub fn cadence(v: f32) -> f32 {
    table(&[(0.0, 0.72), (1.0, 0.9), (1.8, 1.08), (2.6, 1.3), (3.5, 1.36), (5.0, 1.42), (6.0, 1.52), (7.0, 1.72), (8.0, 1.95)], v)
}

/// Fraction of the forward gait cycle a foot is on the ground.
pub fn duty(v: f32) -> f32 {
    table(&[(0.0, 0.64), (1.8, 0.6), (2.6, 0.42), (4.0, 0.34), (5.0, 0.29), (6.0, 0.26), (8.0, 0.22)], v)
}

/// Backpedalling: short, quick steps on the balls of the feet.
fn cadence_back(v: f32) -> f32 {
    table(&[(0.0, 0.72), (1.0, 1.15), (2.0, 1.65), (3.0, 2.05), (4.0, 2.35), (5.0, 2.6)], v)
}

fn duty_back(v: f32) -> f32 {
    table(&[(0.0, 0.64), (1.0, 0.6), (2.0, 0.5), (3.0, 0.44), (4.0, 0.4)], v)
}

/// Side shuffle: the feet alternate wide and narrow without crossing.
fn cadence_side(v: f32) -> f32 {
    table(&[(0.0, 0.72), (0.5, 1.25), (1.0, 1.55), (2.0, 2.0), (3.0, 2.4), (4.0, 2.7)], v)
}

fn duty_side(v: f32) -> f32 {
    table(&[(0.0, 0.62), (0.8, 0.58), (1.6, 0.46), (3.0, 0.4)], v)
}

/// 0 = walking gait, 1 = running gait (forward).
pub fn run_weight(v: f32) -> f32 {
    smoothstep(1.9, 2.7, v)
}

/// Direction style weights (forward, backward, sideways) for travel angle `dir`
/// relative to the legs; they sum to 1.
pub fn style(dir: f32) -> [f32; 3] {
    let (s, c) = dir.sin_cos();
    [c.max(0.0).powi(2), (-c).max(0.0).powi(2), s * s]
}

/// Cadence, duty factor and foot spacing of a gait.
#[derive(Clone, Copy, Debug)]
pub struct GaitParams {
    pub cadence: f32,
    pub duty: f32,
    /// Half the distance between the feet across the legs.
    pub half_width: f32,
}

pub fn params(b: &Body, v: f32, dir: f32) -> GaitParams {
    let w = style(dir);
    let rw = run_weight(v);
    let fwd_w = lerp(b.hip_half_width + 0.015, b.hip_half_width * 0.55, rw);
    let back_w = b.hip_half_width + 0.035 * b.scale;
    let side_w = lerp(0.17, 0.26, smoothstep(0.3, 1.6, v)) * b.scale;
    let half_width = w[0] * fwd_w + w[1] * back_w + w[2] * side_w;
    let duty = w[0] * duty(v) + w[1] * duty_back(v) + w[2] * duty_side(v);
    let mut cad = w[0] * cadence(v) + w[1] * cadence_back(v) + w[2] * cadence_side(v);
    // Shuffling feet may not cross: quicker, shorter steps when moving sideways.
    let gap = (half_width - 0.065 * b.scale).max(0.03);
    if w[2] > 0.02 {
        cad = cad.max(v * duty * w[2] / (2.0 * gap));
    }
    GaitParams { cadence: cad.max(0.1), duty, half_width }
}

/// Lowest ankle height that keeps the sole on (and not in) the ground for a
/// foot pitched by `pitch` (positive = toes up): the foot rolls heel → toe.
pub fn ground_ankle(b: &Body, pitch: f32) -> f32 {
    let (s, c) = pitch.sin_cos();
    let heel = -b.ankle_height * c - b.heel * s;
    let toe = -b.ankle_height * 0.8 * c + b.toe * s;
    -heel.min(toe)
}

/// Horizontal shift of the ankle along the foot (positive = towards the toes)
/// when the foot rolls about its heel (toes up) or the ball of the foot (toes
/// down) instead of lying flat: keeps the contact point where it was.
pub fn roll_shift(b: &Body, pitch: f32) -> f32 {
    let (s, c) = pitch.sin_cos();
    if pitch < 0.0 {
        b.toe * (1.0 - c) - 0.8 * b.ankle_height * s
    } else {
        -b.heel * (1.0 - c) - b.ankle_height * s
    }
}

/// Unit vector along a foot turned by `angle` (character space, + = right).
pub fn foot_axis(angle: f32) -> (f32, f32) {
    angle.sin_cos()
}

/// Catmull-Rom through a list of points sampled at equal steps of `s` ∈ [0, 1].
fn spline(pts: &[(f32, f32)], s: f32) -> (f32, f32) {
    let n = pts.len() - 1;
    let f = clamp(s, 0.0, 1.0) * n as f32;
    let i = (f.floor() as usize).min(n - 1);
    let t = f - i as f32;
    let p = |k: isize| pts[(k.clamp(0, n as isize)) as usize];
    let (p0, p1, p2, p3) = (p(i as isize - 1), p(i as isize), p(i as isize + 1), p(i as isize + 2));
    let cr = |a: f32, b: f32, c: f32, d: f32| {
        0.5 * (2.0 * b + (c - a) * t + (2.0 * a - 5.0 * b + 4.0 * c - d) * t * t + (3.0 * b - a - 3.0 * c + d) * t * t * t)
    };
    (cr(p0.0, p1.0, p2.0, p3.0), cr(p0.1, p1.1, p2.1, p3.1))
}

/// Swing curves: (normalised travel −1 → 1 along the direction of travel,
/// normalised height).
const WALK_SWING: [(f32, f32); 5] = [(-1.0, 0.0), (-0.75, 0.55), (-0.1, 0.8), (0.6, 0.45), (1.0, 0.0)];
const RUN_SWING: [(f32, f32); 6] = [(-1.0, 0.0), (-1.25, 0.55), (-0.85, 0.95), (0.05, 0.75), (0.85, 0.3), (1.0, 0.0)];
const BACK_SWING: [(f32, f32); 5] = [(-1.0, 0.0), (-0.65, 0.65), (0.1, 0.85), (0.75, 0.4), (1.0, 0.0)];
const SIDE_SWING: [(f32, f32); 5] = [(-1.0, 0.0), (-0.6, 0.7), (0.1, 0.8), (0.7, 0.35), (1.0, 0.0)];

/// Gait state for one frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct Gait {
    /// Ground speed (m/s).
    pub speed: f32,
    /// Cycle phase (0..1).
    pub phase: f32,
    /// Direction of travel relative to the legs: 0 = forward, +π/2 = right,
    /// ±π = backwards.
    pub dir: f32,
    /// Legs turned relative to the upper body (positive = right).
    pub hip_turn: f32,
}

/// A locomotion pose plus which feet are on the ground.
#[derive(Clone, Copy, Debug)]
pub struct Loco {
    pub pose: Pose,
    pub stance: [bool; 2],
    /// Duration of a swing (s).
    pub swing_time: f32,
}

/// Leg "forwardness" along the travel (+1 leg ahead, −1 behind) of foot `i`.
pub fn leg_forward(phase: f32, i: usize) -> f32 {
    let p = (phase + i as f32 * 0.5).fract();
    (TAU * (p + 0.07)).cos()
}

/// Locomotion pose (see [`locomote`]).
pub fn pose(b: &Body, g: &Gait) -> Pose {
    locomote(b, g).pose
}

/// Locomotion pose for moving at `g.speed` in direction `g.dir`.
pub fn locomote(b: &Body, g: &Gait) -> Loco {
    let v = g.speed;
    let w = style(g.dir);
    let gp = params(b, v, g.dir);
    let f = gp.cadence;
    let du = gp.duty;
    let stance_len = v * du / f;
    let rw = run_weight(v);
    let rwb = smoothstep(1.5, 2.6, v);
    let quick = smoothstep(0.5, 2.0, v);
    let sprint = smoothstep(5.5, 8.0, v);
    let (dx, dz) = g.dir.sin_cos();
    let (sh, ch) = g.hip_turn.sin_cos();
    // Leg frame → character frame (yaw by hip_turn, positive = right).
    let to_char = |x: f32, z: f32| (x * ch + z * sh, -x * sh + z * ch);
    let mut p = Pose::stand(b);

    // Feet.
    let lift = (w[0] * lerp(0.075, lerp(0.24, 0.42, sprint), rw) + w[1] * lerp(0.06, 0.15, rwb) + w[2] * lerp(0.045, 0.09, quick)) * b.scale;
    let toe_out = w[0] * lerp(0.12, 0.04, rw) + w[1] * 0.1 + w[2] * 0.2;
    // Forward: heel strike (toes up) when landing, push off the toes.
    let land_f = lerp(0.28, 0.08, rw);
    let toe_off_f = -lerp(0.55, 0.8, rw);
    let mut stance = [false; 2];
    let mut contact = [0.0f32; 2];
    for i in 0..2 {
        let side = if i == 0 { -1.0 } else { 1.0 };
        let ph = (g.phase + i as f32 * 0.5).fract();
        let swing_s = if ph < du { 0.0 } else { (ph - du) / (1.0 - du) };
        let (n, ny, pitch) = if ph < du {
            stance[i] = true;
            // Stance: the foot moves back along the travel at ground speed and
            // rolls heel → toe (forward) or stays on the ball of the foot.
            let s = ph / du;
            let heel_strike = land_f * (1.0 - smoothstep(0.0, 0.18, s));
            let push = toe_off_f * smoothstep(0.55, 1.0, s);
            let pf = heel_strike + push;
            let pb = -0.28 * (1.0 - smoothstep(0.0, 0.35, s)) - 0.1 - 0.15 * smoothstep(0.75, 1.0, s);
            let ps = -0.06 - 0.12 * smoothstep(0.7, 1.0, s);
            (1.0 - 2.0 * s, 0.0, w[0] * pf + w[1] * pb + w[2] * ps)
        } else {
            let s = (ph - du) / (1.0 - du);
            let (wz, wy) = spline(&WALK_SWING, s);
            let (rz, ry) = spline(&RUN_SWING, s);
            let (bz, by) = spline(&BACK_SWING, s);
            let (sz, sy) = spline(&SIDE_SWING, s);
            let nz = w[0] * lerp(wz, rz, rw) + w[1] * bz + w[2] * sz;
            let ny = (w[0] * lerp(wy, ry, rw) + w[1] * by + w[2] * sy).max(0.0);
            // Toes point down after push-off, come up before a heel strike;
            // backpedal and shuffle land on the ball of the foot.
            let pf = lerp(toe_off_f, land_f, smoothstep(0.3, 0.9, s));
            let pb = lerp(-0.25, -0.38, smoothstep(0.4, 1.0, s));
            let ps = lerp(-0.18, -0.06, smoothstep(0.0, 1.0, s));
            (nz, ny, w[0] * pf + w[1] * pb + w[2] * ps)
        };
        // The foot lands not far in front of the hip and pushes off well
        // behind it (running even more so).
        let along = (n * 0.5 - w[0] * lerp(0.1, 0.15, rw)) * stance_len;
        let (cx, cz) = to_char(side * gp.half_width + dx * along, dz * along);
        let (ax, az) = foot_axis(g.hip_turn + side * toe_out);
        let roll = roll_shift(b, pitch);
        p.feet[i] = v3(cx + ax * roll, ground_ankle(b, pitch) + ny * lift, cz + az * roll);
        // The pelvis must let the stance foot (and the swing foot about to
        // land) reach the ground; a foot just pushed off may leave it.
        contact[i] = if stance[i] { 1.0 } else { smoothstep(0.75, 1.0, swing_s) };
        p.foot_pitch[i] = pitch;
        p.foot_yaw[i] = toe_out;
        p.foot_turn[i] = g.hip_turn;
        let (kx, kz) = to_char(side * (0.08 + 0.2 * w[2]), 1.0);
        p.knee_dir[i] = v3(kx, 0.0, kz);
    }

    // Pelvis: lower when running or in an athletic stance, bobbing twice per
    // cycle, swaying over the stance foot.
    let mid_l = g.phase - du * 0.5;
    let bob2 = (2.0 * TAU * mid_l).cos();
    let bob_f = lerp(0.008, -lerp(0.035, 0.022, sprint), rw) * bob2;
    let bob_b = lerp(0.01, -0.02, rwb) * bob2;
    let bob_s = 0.01 * bob2;
    let comp_f = lerp(0.012, lerp(0.06, 0.045, sprint), rw);
    let comp_b = lerp(0.045, 0.075, rwb);
    let comp_s = lerp(0.09, 0.13, quick);
    p.pelvis.y += (w[0] * (bob_f - comp_f) + w[1] * (bob_b - comp_b) + w[2] * (bob_s - comp_s)) * b.scale;
    let sway = -lerp(0.022, 0.008, rw) * (TAU * mid_l).cos() * b.scale * (1.0 - w[2]);
    let (px, pz) = to_char(sway, 0.0);
    p.pelvis.x += px;
    p.pelvis.z += pz;

    // Pelvis yaw and roll with the stride (along the body's own forward axis);
    // chest counter-rotates; the upper body undoes the hip turn.
    let sag = dz;
    let fl = leg_forward(g.phase, 0) * sag;
    let yaw_amp = w[0] * lerp(0.09, 0.13, rw) + w[1] * 0.06;
    let tilt = w[0] * lerp(0.03, 0.1, rw) + w[1] * 0.1 + w[2] * 0.12;
    let roll = lerp(0.045, 0.03, rw) * (TAU * mid_l).sin() * (1.0 - w[2]);
    p.pelvis_rot = v3(tilt, g.hip_turn + yaw_amp * fl, roll);
    // Forward lean grows with speed (sprinters ~12–15°); athletic stances
    // keep the chest over the balls of the feet.
    let lean = w[0] * lerp(0.02, lerp(0.09, 0.2, sprint), rw) + w[1] * (0.1 + 0.04 * rwb) + w[2] * 0.12;
    p.spine = v3(lean * 0.6, -g.hip_turn * 0.35, 0.0);
    p.chest = v3(lean * 0.4, -g.hip_turn * 0.45 - yaw_amp * fl * 1.7, -roll * 0.6);
    // Keep the head level and looking ahead of the upper body.
    p.head = v3(-(tilt + lean) * 0.8 + 0.05, -g.hip_turn * 0.2 + yaw_amp * fl * 0.6, 0.0);

    // Arms swing opposite to the legs, elbows bend more with speed; out to the
    // sides for balance when shuffling.
    let amp_f = lerp(0.28, lerp(0.6, 1.0, sprint), rw);
    let elbow_f = lerp(0.35, lerp(1.35, 1.6, sprint), rw);
    for i in 0..2 {
        let fwd = leg_forward(g.phase, i) * sag;
        let flex = w[0] * (lerp(0.04, 0.18, rw) - amp_f * fwd) + w[1] * (0.15 - (0.3 + 0.15 * rwb) * fwd) + w[2] * 0.25;
        p.arms[i] = ArmPose {
            flex,
            abduct: w[0] * lerp(0.12, 0.2, rw) + w[1] * 0.28 + w[2] * 0.45,
            twist: w[0] * lerp(0.0, 0.25, rw) + w[1] * 0.1 + w[2] * 0.2,
            // The elbow closes as the arm comes forward and opens behind.
            elbow: w[0] * (elbow_f - lerp(0.1, 0.25, rw) * fwd) + w[1] * (0.95 + 0.25 * rwb - 0.15 * fwd) + w[2] * 0.9,
            ..ArmPose::RELAXED
        };
    }
    p.fit_legs(b, contact);
    Loco { pose: p, stance, swing_time: (1.0 - du) / f }
}

/// Idle stance with breathing and a slow weight shift. `ready` (0..1) blends
/// into an athletic ready stance: feet wider, knees bent, on the balls of the
/// feet, arms out; keepers crouch lower with the hands in front.
pub fn idle(b: &Body, time: f32, seed: f32, ready: f32, keeper: bool) -> Pose {
    let mut p = Pose::stand(b);
    let t = time + seed * 7.0;
    let breathe = (TAU * t / 3.8).sin();
    let shift = (TAU * t / 6.5).sin() * (1.0 - 0.7 * ready);
    let r = ready;
    let kw = if keeper { 1.0 } else { 0.0 };
    let pitch = -0.12 * r;
    for i in 0..2 {
        let s = if i == 0 { -1.0 } else { 1.0 };
        p.feet[i].x = s * (b.hip_half_width + (0.03 + lerp(0.07, 0.13, kw) * r) * b.scale);
        p.feet[i].z += 0.02 * s * r;
        p.feet[i].y = ground_ankle(b, pitch);
        p.foot_pitch[i] = pitch;
        p.foot_yaw[i] = 0.12 + 0.1 * r;
        // Knees over the feet in a wide stance.
        p.knee_dir[i] = v3(s * (0.1 + 0.7 * r), 0.0, 1.0);
    }
    let bounce = (TAU * (t * 1.9)).sin() * 0.008 * r * (1.0 - 0.5 * kw);
    p.pelvis.x = 0.018 * shift * b.scale;
    p.pelvis.y -= (0.01 + lerp(0.08, 0.15, kw) * r) * b.scale - bounce;
    p.pelvis.z -= 0.02 * r * b.scale;
    p.pelvis_rot.x = 0.18 * r;
    p.pelvis_rot.z = -0.025 * shift;
    p.spine.x += 0.01 * breathe + 0.12 * r;
    p.chest.x -= 0.015 * breathe - 0.04 * r;
    p.head.x -= 0.3 * r;
    p.head.z = 0.02 * shift;
    for i in 0..2 {
        let a = &mut p.arms[i];
        a.flex += 0.02 * breathe + lerp(0.3, 0.55, kw) * r;
        a.abduct += lerp(0.18, 0.4, kw) * r;
        a.elbow += lerp(0.8, 0.65, kw) * r;
        a.twist += 0.15 * r;
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::render::anim::rig::{fk, Frame};
    use calcio_core::math::Vec3;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn cadence_and_stride_are_realistic() {
        // jog 3.5 m/s: ~2.7 steps/s; sprint 7.5 m/s: ~3.8–4.2 steps/s
        assert!((2.0 * cadence(3.5) - 2.72).abs() < 0.15);
        let sprint_steps = 2.0 * cadence(7.5);
        assert!(sprint_steps > 3.6 && sprint_steps < 4.3, "{sprint_steps}");
        // stride (one step) at sprint ≈ 1.8–2.1 m
        let step = 7.5 / sprint_steps;
        assert!(step > 1.7 && step < 2.2, "step {step}");
        // backpedal at 3 m/s: short quick steps (~0.6–0.85 m)
        let b = Body::standard();
        let back = params(&b, 3.0, PI);
        let step = 3.0 / (2.0 * back.cadence);
        assert!(step > 0.6 && step < 0.85, "backpedal step {step}");
    }

    #[test]
    fn stance_foot_moves_against_travel_at_ground_speed() {
        let b = Body::standard();
        for dir in [0.0f32, PI, FRAC_PI_2, -FRAC_PI_2, 0.7, 2.4] {
            for v in [1.0f32, 2.5, 4.0] {
                let gp = params(&b, v, dir);
                let dt = 0.002;
                // mid-stance, where the foot is flat or on the ball
                let ph0 = 0.45 * gp.duty;
                let g0 = Gait { speed: v, phase: ph0, dir, hip_turn: 0.0 };
                let g1 = Gait { phase: ph0 + gp.cadence * dt, ..g0 };
                let (p0, p1) = (pose(&b, &g0), pose(&b, &g1));
                let vel = (p1.feet[0] - p0.feet[0]) / dt;
                let expected = v3(dir.sin(), 0.0, dir.cos()) * -v;
                assert!((vel - expected).flat_length() < 0.05 * v, "dir {dir} v {v}: stance foot {vel:?} vs {expected:?}");
            }
        }
    }

    #[test]
    fn shuffling_feet_never_cross() {
        let b = Body::standard();
        for v in [0.5f32, 1.5, 3.0] {
            for k in 0..60 {
                let p = pose(&b, &Gait { speed: v, phase: k as f32 / 60.0, dir: FRAC_PI_2, hip_turn: 0.0 });
                let gap = p.feet[1].x - p.feet[0].x;
                assert!(gap > 0.12, "v {v} k {k}: gap {gap}");
            }
        }
    }

    #[test]
    fn soles_touch_but_do_not_enter_the_ground_in_stance() {
        let b = Body::standard();
        let base = Frame::character(Vec3::ZERO, 0.0);
        for dir in [0.0f32, PI, FRAC_PI_2] {
            for v in [1.0f32, 3.5, 7.0] {
                for k in 0..40 {
                    let l = locomote(&b, &Gait { speed: v, phase: k as f32 / 40.0, dir, hip_turn: 0.0 });
                    let s = fk(&l.pose, &b, &base);
                    for i in 0..2 {
                        let low = s.heel[i].y.min(s.toe[i].y);
                        assert!(low > -0.005, "dir {dir} v {v} k {k}: sole at {low}");
                        if l.stance[i] {
                            assert!(low < 0.01, "dir {dir} v {v} k {k}: stance foot floating at {low}");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn gait_poses_are_valid_through_the_cycle() {
        let b = Body::standard();
        let base = Frame::character(Vec3::ZERO, 0.0);
        for (dir, turn) in [(0.0f32, 0.0f32), (PI, 0.0), (FRAC_PI_2, 0.0), (0.4, 1.1), (-2.5, -0.6)] {
            for v in [0.0f32, 1.0, 2.2, 3.5, 5.5, 7.5] {
                for k in 0..40 {
                    let p = pose(&b, &Gait { speed: v, phase: k as f32 / 40.0, dir, hip_turn: turn });
                    assert!(p.is_finite());
                    let s = fk(&p, &b, &base);
                    assert!(s.is_finite());
                    for i in 0..2 {
                        assert!(s.ankle[i].y > b.ankle_height * 0.9, "v {v} k {k}");
                    }
                }
            }
        }
        for r in [0.0f32, 0.5, 1.0] {
            for keeper in [false, true] {
                let p = idle(&b, 1.3, 0.2, r, keeper);
                let s = fk(&p, &b, &base);
                assert!(s.is_finite());
                for i in 0..2 {
                    let low = s.heel[i].y.min(s.toe[i].y);
                    assert!(low.abs() < 0.01, "idle ready {r}: sole at {low}");
                }
            }
        }
    }
}
