//! Action animations driven by the simulation's action states: kicks (pass,
//! shot, cross, chip, volley) and the keeper's rolls and throws.
//!
//! A kick follows the core's [`KickState`]: the plant foot steps beside the
//! ball, the kicking leg draws back, meets the ball exactly at the contact
//! time (foot IK to where the ball is) and follows through along the kick, as
//! high as the power asks; arms and torso counter-balance.

use super::body::Body;
use super::pose::Pose;
use super::rig::Frame;
use calcio_core::kick::{KickKind, KickState};
use calcio_core::math::{clamp, lerp, smoothstep, v3, wrap_angle, Vec3};
use calcio_core::player::Player;
use calcio_core::World;
use std::f32::consts::PI;

/// Remembered per kick: where the plant foot started and where it goes, and
/// where the ball is struck.
#[derive(Clone, Copy, Debug)]
pub struct KickAnim {
    /// Wind-up time this memory belongs to (to notice a new kick).
    windup: f32,
    plant_from: Vec3,
    plant_from_heading: f32,
    kick_from: Vec3,
    kick_from_heading: f32,
    /// Ball at contact (world): predicted during the wind-up, frozen after.
    ball: Vec3,
    plant_to: Vec3,
}

/// How a ball is struck.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Technique {
    /// Side-foot: foot turned out, the inside of the foot meets the ball.
    Inside,
    /// Instep drive: toes pointed, laces through the ball.
    Laces,
    /// Wrapped instep for a curling cross.
    Wrap,
    /// Short stab under the ball.
    Chip,
}

impl Technique {
    fn of(kind: KickKind) -> Technique {
        match kind {
            KickKind::Pass | KickKind::Roll => Technique::Inside,
            KickKind::Shot | KickKind::Throw => Technique::Laces,
            KickKind::Cross => Technique::Wrap,
            KickKind::Lob => Technique::Chip,
        }
    }
}

/// Weight of the kick over the locomotion pose at kick progress `u` (0..2).
pub fn kick_weight(u: f32) -> f32 {
    if u < 1.0 {
        smoothstep(0.0, 0.22, u)
    } else {
        1.0 - smoothstep(1.55, 2.0, u)
    }
}

/// Is foot `i` driven by the kick at progress `u`? (kicking foot always,
/// plant foot until it can walk again)
pub fn kick_drives_foot(k: &KickState, i: usize, u: f32) -> bool {
    let kicking = if k.foot.side() < 0.0 { 0 } else { 1 };
    if k.req.kind.from_hands() {
        // The step towards the target is taken with the foot opposite the throwing hand.
        return i != kicking && u < 1.6;
    }
    i == kicking || u < 1.6
}

/// Catmull-Rom through keys (time, value) — the times must increase.
fn keyed<T: Copy + std::ops::Add<Output = T> + std::ops::Sub<Output = T> + std::ops::Mul<f32, Output = T>>(keys: &[(f32, T)], u: f32) -> T {
    let n = keys.len();
    if u <= keys[0].0 {
        return keys[0].1;
    }
    if u >= keys[n - 1].0 {
        return keys[n - 1].1;
    }
    let mut i = 0;
    while i + 2 < n && u > keys[i + 1].0 {
        i += 1;
    }
    let (t1, t2) = (keys[i].0, keys[i + 1].0);
    let t = (u - t1) / (t2 - t1);
    let p1 = keys[i].1;
    let p2 = keys[i + 1].1;
    let p0 = if i > 0 { keys[i - 1].1 } else { p1 };
    let p3 = if i + 2 < n { keys[i + 2].1 } else { p2 };
    let t2_ = t * t;
    let t3 = t2_ * t;
    // 0.5 * (2p1 + (p2 - p0) t + (2p0 - 5p1 + 4p2 - p3) t² + (3p1 - p0 - 3p2 + p3) t³)
    (p1 * 2.0 + (p2 - p0) * t + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2_ + (p1 * 3.0 - p0 - p2 * 3.0 + p3) * t3) * 0.5
}

fn rot_y(v: Vec3, a: f32) -> Vec3 {
    let (s, c) = a.sin_cos();
    v3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c)
}

/// Applies the kick (or throw) of `p` to `pose`. `shown` are the world
/// ground positions and headings of the feet as last displayed.
#[allow(clippy::too_many_arguments)]
pub fn kick(
    pose: &mut Pose,
    mem: &mut Option<KickAnim>,
    b: &Body,
    p: &Player,
    world: &World,
    frame: &Frame,
    shown: [(Vec3, f32); 2],
) {
    let Some(k) = p.kick else {
        *mem = None;
        return;
    };
    let s = k.foot.side();
    let (ki, pi) = if s < 0.0 { (0usize, 1usize) } else { (1, 0) };
    let kd_world = k.req.dir.flat().normalize_or(p.forward());
    let right_k = kd_world.cross(Vec3::Y);
    // Ball at contact.
    let ball_now = world.ball.pos;
    let predicted = if k.done_contact { None } else { Some(ball_now + world.ball.vel.flat() * (k.windup - k.t).max(0.0)) };
    let fresh = mem.map(|m| (m.windup - k.windup).abs() > 1e-4 || k.t < 0.02 && m.ball.distance(ball_now) > 2.0).unwrap_or(true);
    if fresh {
        let ball = predicted.unwrap_or(ball_now);
        *mem = Some(KickAnim {
            windup: k.windup,
            plant_from: shown[pi].0,
            plant_from_heading: shown[pi].1,
            kick_from: shown[ki].0,
            kick_from_heading: shown[ki].1,
            ball,
            plant_to: ball,
        });
    }
    let m = mem.as_mut().unwrap();
    if let Some(pred) = predicted {
        m.ball = pred;
    }
    let tech = Technique::of(k.req.kind);
    let power = clamp(k.req.power, 0.0, 1.0);
    let hands = k.req.kind.from_hands();
    // Plant foot beside the ball (for throws: a step towards the target).
    m.plant_to = if hands {
        let reach = if k.req.kind == KickKind::Roll { 0.7 } else { 0.45 };
        p.pos + kd_world * reach - right_k * (s * 0.14)
    } else {
        let side = if tech == Technique::Inside { 0.26 } else { 0.22 };
        v3(m.ball.x, 0.0, m.ball.z) - right_k * (s * side) - kd_world * 0.06
    };
    let u = k.progress();
    let w = kick_weight(u);
    if w <= 0.0 {
        return;
    }

    // Character-space quantities.
    let bc = frame.to_local(m.ball);
    let kd = frame.dir_to_local(kd_world);
    let ak = kd.x.atan2(kd.z);
    let k_side = rot_y(v3(1.0, 0.0, 0.0), ak); // right of the kick, local

    // --- Plant foot: a step (lift arc) to its spot, then planted.
    {
        let from = frame.to_local(m.plant_from);
        let to = frame.to_local(m.plant_to);
        let step_end = if hands { 0.75 } else { 0.55 };
        let e = smoothstep(0.0, step_end, u);
        let lift = 0.09 * b.scale * (PI * clamp(u / step_end, 0.0, 1.0)).sin();
        let pos = v3(lerp(from.x, to.x, e), b.ankle_height + lift, lerp(from.z, to.z, e));
        let from_turn = wrap_angle(m.plant_from_heading - p.facing);
        let turn = from_turn + wrap_angle(ak - from_turn) * e;
        let pitch = if e < 1.0 { 0.25 * (PI * e).sin() } else { 0.0 };
        blend_foot(pose, pi, pos, pitch, turn, rot_y(v3(-s * 0.1, 0.0, 1.0), turn), w);
    }

    if hands {
        throw_upper_body(pose, b, &k, frame, world, s, ak, u, w);
        return;
    }

    // --- Kicking foot: start → back-swing → contact → follow-through → down.
    let start = frame.to_local(m.kick_from);
    let start = v3(start.x, b.ankle_height, start.z);
    let start_turn = wrap_angle(m.kick_from_heading - p.facing);
    let volley = clamp((bc.y - 0.11) / 0.5, 0.0, 1.0);
    let hw = b.hip_half_width;
    let kick_hip = v3(s * hw, 0.0, 0.0);
    let (back_d, back_h) = match tech {
        Technique::Inside => (0.18 + 0.18 * power, 0.12 + 0.05 * power),
        Technique::Chip => (0.3, 0.3),
        _ => (0.3 + 0.25 * power, 0.28 + 0.22 * power),
    };
    let back = v3(kick_hip.x + s * 0.03, b.ankle_height + back_h + 0.1 * volley, bc.z.min(0.4) - back_d - 0.1);
    // Ankle at contact relative to the ball centre (local, kick-aligned).
    let contact_off = match tech {
        Technique::Inside => k_side * (s * 0.075) + v3(0.0, -0.03, 0.0) - kd * 0.05,
        Technique::Laces => v3(0.0, 0.08, 0.0) - kd * 0.13,
        Technique::Wrap => k_side * (s * 0.05) + v3(0.0, 0.06, 0.0) - kd * 0.12,
        Technique::Chip => v3(0.0, -0.02, 0.0) - kd * 0.16,
    };
    let mut contact = bc + contact_off;
    contact.y = contact.y.max(b.ankle_height + 0.01);
    let (f_len, f_h) = match tech {
        Technique::Inside => (0.25 + 0.15 * power, 0.06 + 0.12 * power),
        Technique::Laces => (0.35 + 0.35 * power, 0.2 + 0.55 * power),
        Technique::Wrap => (0.35, 0.35 + 0.15 * power),
        Technique::Chip => (0.12, 0.3),
    };
    let across = if tech == Technique::Wrap { -k_side * (s * 0.25) } else { Vec3::ZERO };
    let follow = contact + kd * f_len + across + v3(0.0, f_h + 0.2 * volley, 0.0);
    let land = v3(s * (hw + 0.04), b.ankle_height, bc.z.max(0.0) + 0.15);
    let foot = keyed(&[(0.0, start), (0.6, back), (1.0, contact), (1.4, follow), (1.85, land)], u);
    let (p_back, p_contact, p_follow) = match tech {
        Technique::Inside => (0.05, 0.12, 0.1),
        Technique::Laces => (-0.9, -1.05, -0.8),
        Technique::Wrap => (-0.6, -0.7, -0.6),
        Technique::Chip => (-0.3, -0.45, -0.2),
    };
    let start_pitch = pose.foot_pitch[ki];
    let pitch = keyed(&[(0.0, start_pitch), (0.6, p_back), (1.0, p_contact), (1.4, p_follow), (1.85, 0.0)], u);
    let turn_out = match tech {
        Technique::Inside => 1.15,
        Technique::Wrap => 0.5,
        Technique::Laces => 0.12,
        Technique::Chip => 0.2,
    };
    let kick_turn = ak + s * turn_out;
    let turn = keyed(&[(0.0, start_turn), (0.5, kick_turn), (1.4, kick_turn), (1.85, ak * 0.5)], u);
    // The knee turns out with the foot for a side-foot pass.
    let knee = rot_y(v3(s * 0.1, -0.1, 1.0), ak + s * turn_out * 0.75);
    blend_foot(pose, ki, foot, pitch, turn, knee, w);
    pose.foot_yaw[ki] *= 1.0 - w;

    // --- Body: hips cock back with the back-swing and rotate through.
    let swing = keyed(&[(0.0, 0.0), (0.6, 1.0), (1.0, -0.2), (1.4, -1.0), (2.0, 0.0)], u);
    let force = match tech {
        Technique::Inside => 0.45,
        Technique::Chip => 0.6,
        _ => 0.7 + 0.3 * power,
    };
    let hip_yaw = s * 0.32 * force * swing + ak * 0.5;
    pose.pelvis_rot.y = lerp(pose.pelvis_rot.y, hip_yaw, w);
    pose.spine.y = lerp(pose.spine.y, -hip_yaw * 0.35, w);
    pose.chest.y = lerp(pose.chest.y, -hip_yaw * 0.3, w);
    // Lean over the plant foot (away from the kicking leg), more on volleys.
    let lean_side = -s * (0.1 + 0.35 * volley) * smoothstep(0.2, 0.9, u.min(2.0 - u).max(0.0) * 1.6);
    pose.pelvis_rot.z += lean_side * w;
    pose.spine.z += lean_side * 0.5 * w;
    // Over the ball for a low drive, back for a chip; arched at the back-swing.
    let over = match tech {
        Technique::Laces => 0.22 * (1.0 - volley),
        Technique::Chip => -0.18,
        Technique::Wrap => 0.05,
        Technique::Inside => 0.12,
    };
    let lean = keyed(&[(0.0, 0.0), (0.6, -0.08), (1.0, over), (1.4, over * 0.6), (2.0, 0.0)], u);
    pose.spine.x += lean * w;
    // The standing knee bends at contact.
    pose.pelvis.y -= 0.05 * b.scale * smoothstep(0.4, 1.0, u) * (1.0 - smoothstep(1.4, 2.0, u)) * w;

    // Arms: the plant-side arm flies out for balance, the kicking-side arm swings opposite the leg.
    let arm_out = smoothstep(0.1, 0.7, u) * (1.0 - smoothstep(1.4, 2.0, u));
    {
        let a = &mut pose.arms[pi];
        a.abduct = lerp(a.abduct, 0.6 + 0.6 * force, arm_out * w);
        a.flex = lerp(a.flex, 0.35 - 0.3 * swing, arm_out * w);
        a.elbow = lerp(a.elbow, 0.5, arm_out * w);
    }
    {
        let a = &mut pose.arms[ki];
        a.flex = lerp(a.flex, -0.35 * swing * force + 0.1, arm_out * w);
        a.abduct = lerp(a.abduct, 0.35 + 0.2 * force, arm_out * w);
        a.elbow = lerp(a.elbow, 0.7, arm_out * w);
    }
    let contact_plant = if u < 1.6 { 1.0 } else { 0.0 };
    let mut c = [0.0; 2];
    c[pi] = contact_plant;
    pose.fit_legs(b, c);
}

/// Keeper roll (underarm) and throw (overarm): the throwing hand stays on the
/// ball, which the core moves along the throw.
#[allow(clippy::too_many_arguments)]
fn throw_upper_body(pose: &mut Pose, b: &Body, k: &KickState, frame: &Frame, world: &World, s: f32, ak: f32, u: f32, w: f32) {
    let hi = if s < 0.0 { 0 } else { 1 };
    let oi = 1 - hi;
    let roll = k.req.kind == KickKind::Roll;
    let ball = frame.to_local(world.ball.pos);
    // Wind-up: turn the throwing shoulder back, then through.
    let swing = keyed(&[(0.0, 0.0), (0.7, 1.0), (1.0, -0.4), (1.5, -0.8), (2.0, 0.0)], u);
    let yaw = s * 0.45 * swing + ak * 0.6;
    pose.pelvis_rot.y = lerp(pose.pelvis_rot.y, yaw * 0.6, w);
    pose.spine.y = lerp(pose.spine.y, yaw * 0.25, w);
    pose.chest.y = lerp(pose.chest.y, yaw * 0.25, w);
    if roll {
        // Bowl it: crouch low, chest forward.
        let crouch = smoothstep(0.2, 0.9, u) * (1.0 - smoothstep(1.3, 2.0, u));
        pose.pelvis.y -= 0.3 * b.scale * crouch * w;
        pose.spine.x += 0.55 * crouch * w;
        pose.chest.x += 0.2 * crouch * w;
    } else {
        let arch = keyed(&[(0.0, 0.0), (0.7, -0.15), (1.0, 0.15), (1.5, 0.3), (2.0, 0.0)], u);
        pose.spine.x += arch * w;
    }
    // Throwing hand on the ball until it is released, then the arm follows through.
    let a = &mut pose.arms[hi];
    if !k.done_contact {
        a.target = ball;
        a.target_w = w;
    } else {
        let f = smoothstep(1.0, 1.4, u) * (1.0 - smoothstep(1.6, 2.0, u));
        if roll {
            a.flex = lerp(a.flex, 1.1, f * w);
            a.elbow = lerp(a.elbow, 0.2, f * w);
        } else {
            a.flex = lerp(a.flex, 1.3, f * w);
            a.elbow = lerp(a.elbow, 0.4, f * w);
            a.abduct = lerp(a.abduct, 0.2, f * w);
        }
        a.target_w = 0.0;
    }
    // The other hand helps hold the ball, then points the way for balance.
    let o = &mut pose.arms[oi];
    let both = 1.0 - smoothstep(0.15, 0.45, u);
    if both > 0.0 && !k.done_contact {
        o.target = ball;
        o.target_w = both * w;
    } else {
        o.target_w = 0.0;
    }
    let point = smoothstep(0.3, 0.7, u) * (1.0 - smoothstep(1.4, 2.0, u));
    o.flex = lerp(o.flex, if roll { 0.5 } else { 1.2 }, point * w);
    o.abduct = lerp(o.abduct, 0.35, point * w);
    o.elbow = lerp(o.elbow, 0.3, point * w);
    let mut c = [0.0; 2];
    c[oi] = if u < 1.6 { 1.0 } else { 0.0 };
    c[hi] = 1.0;
    pose.fit_legs(b, c);
}

/// Moves foot `i` towards the given target by weight `w`.
fn blend_foot(pose: &mut Pose, i: usize, pos: Vec3, pitch: f32, turn: f32, knee: Vec3, w: f32) {
    pose.feet[i] = pose.feet[i].lerp(pos, w);
    pose.foot_pitch[i] = lerp(pose.foot_pitch[i], pitch, w);
    pose.foot_turn[i] = pose.foot_turn[i] + wrap_angle(turn - pose.foot_turn[i]) * w;
    pose.knee_dir[i] = pose.knee_dir[i].lerp(knee, w);
}
