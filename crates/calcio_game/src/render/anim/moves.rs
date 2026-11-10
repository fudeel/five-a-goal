//! Action animations other than kicks: slide tackles, standing tackles,
//! falls and getting up, first touches (foot, thigh, chest), headers, the
//! keeper's hands, dives and goal celebrations.
//!
//! Every move is an overlay on the locomotion pose, timed by the simulation's
//! own state (slide / fall / jump timers, seconds since a tackle, touch or
//! header), so what is drawn is what the physics did.

use super::body::Body;
use super::pose::{ArmPose, Pose};
use super::rig::Frame;
use calcio_core::math::{clamp, lerp, smoothstep, v3, wrap_angle, Vec3};
use calcio_core::player::{Player, PlayerSpec};
use calcio_core::World;
use std::f32::consts::PI;

fn side_index(s: f32) -> usize {
    if s < 0.0 {
        0
    } else {
        1
    }
}

/// Crouched half-way pose used when getting up from the ground.
fn crouch(base: &Pose, b: &Body) -> Pose {
    let mut c = *base;
    c.pelvis.y = 0.52 * b.scale;
    c.pelvis.z = -0.08;
    c.pelvis_rot = v3(0.45, 0.0, 0.0);
    c.spine = v3(0.45, 0.0, 0.0);
    c.chest = v3(0.2, 0.0, 0.0);
    c.head = v3(-0.5, 0.0, 0.0);
    for i in 0..2 {
        let s = if i == 0 { -1.0 } else { 1.0 };
        c.feet[i] = v3(s * (b.hip_half_width + 0.06), b.ankle_height, if i == 0 { 0.12 } else { -0.1 });
        c.foot_pitch[i] = 0.0;
        c.knee_dir[i] = v3(s * 0.3, 0.0, 1.0);
        c.arms[i] = ArmPose { flex: 0.6, abduct: 0.3, elbow: 0.4, ..ArmPose::RELAXED };
    }
    c
}

/// Slide tackle: reclined on the trailing hip, leading leg extended along
/// the slide, trailing leg folded under, a hand down behind for support;
/// then sitting up and getting back on the feet.
pub fn slide(pose: &mut Pose, p: &Player, b: &Body, spec: &PlayerSpec) {
    let Some(sl) = p.slide else {
        return;
    };
    let s = sl.leg;
    let (li, ti) = (side_index(s), side_index(-s));
    let mut sp = *pose;
    sp.root_rot = Vec3::ZERO;
    sp.pelvis = v3(0.0, 0.2 * b.scale, -0.05);
    sp.pelvis_rot = v3(-1.1, 0.0, -s * 0.35);
    sp.spine = v3(0.25, 0.0, s * 0.1);
    sp.chest = v3(0.15, 0.0, 0.0);
    sp.head = v3(0.35, 0.0, 0.0);
    sp.feet[li] = v3(s * 0.1, 0.12, 0.82 * b.scale);
    sp.foot_pitch[li] = 0.45;
    sp.foot_turn[li] = 0.0;
    sp.foot_yaw[li] = 0.1;
    sp.knee_dir[li] = v3(s * 0.2, 1.0, 0.1);
    sp.feet[ti] = v3(s * 0.02, 0.09, 0.32 * b.scale);
    sp.foot_pitch[ti] = -0.2;
    sp.foot_turn[ti] = -s * 0.6;
    sp.foot_yaw[ti] = 0.1;
    sp.knee_dir[ti] = v3(-s * 0.9, 0.5, 0.2);
    sp.arms[ti] = ArmPose { target: v3(-s * 0.34, 0.04, -0.3), target_w: 1.0, ..ArmPose::RELAXED };
    sp.arms[li] = ArmPose { flex: 0.9, abduct: 0.9, elbow: 0.5, ..ArmPose::RELAXED };

    let drop = smoothstep(0.0, 0.16, sl.t);
    let w = match sl.stopped {
        None => drop,
        Some(st) if st < spec.slide_lie => 1.0,
        Some(st) => {
            // Getting up: sit up into a crouch, then stand.
            let g = clamp((st - spec.slide_lie) / spec.slide_getup, 0.0, 1.0);
            let c = crouch(pose, b);
            let mid = sp.lerp(&c, smoothstep(0.0, 0.5, g));
            *pose = mid.lerp(pose, smoothstep(0.5, 1.0, g));
            return;
        }
    };
    *pose = pose.lerp(&sp, w);
}

/// Tripped: pitch forward over the feet with the arms out to break the fall,
/// lie face down, push up, kneel and stand.
pub fn fall(pose: &mut Pose, p: &Player, b: &Body) {
    let Some(f) = p.fall else {
        return;
    };
    let fall_in = 0.38;
    let getup = 0.65f32.min(f.duration * 0.45);
    let lie_end = f.duration - getup;
    let heading = wrap_angle(f.dir.heading() - p.facing);
    let (pitch, arms_k, crouch_k) = if f.t < fall_in {
        let k = f.t / fall_in;
        (1.42 * k * k, smoothstep(0.0, 0.7, k), 0.0)
    } else if f.t < lie_end {
        (1.42, 1.0, 0.0)
    } else {
        let g = clamp((f.t - lie_end) / getup, 0.0, 1.0);
        (1.42 * (1.0 - smoothstep(0.15, 0.85, g)), 1.0 - smoothstep(0.0, 0.5, g), (PI * g).sin())
    };
    pose.root_rot = v3(pitch, heading, 0.0);
    pose.root_offset.y = 0.12 * smoothstep(0.9, 1.42, pitch);
    let c = crouch(pose, b);
    *pose = pose.lerp(&c, crouch_k);
    for (i, a) in pose.arms.iter_mut().enumerate() {
        let s = if i == 0 { -1.0 } else { 1.0 };
        a.flex = lerp(a.flex, 2.4 + 0.1 * s, arms_k);
        a.abduct = lerp(a.abduct, 0.45, arms_k);
        a.elbow = lerp(a.elbow, 0.9, arms_k);
        a.target_w = 0.0;
    }
    for i in 0..2 {
        pose.foot_pitch[i] = lerp(pose.foot_pitch[i], -0.6, smoothstep(0.5, 1.4, pitch));
    }
}

/// Standing tackle: lunge with the preferred leg at the ball (stretching the
/// foot to where it was), standing knee bent, arms out. A failed tackle
/// leaves the player off balance.
pub fn tackle(pose: &mut Pose, p: &Player, b: &Body, ball_local: Option<Vec3>) {
    let s = p.preferred_foot().side();
    let i = side_index(s);
    if p.since_tackle < 0.5 {
        let u = p.since_tackle / 0.5;
        let w = smoothstep(0.0, 0.12, u) * (1.0 - smoothstep(0.55, 1.0, u));
        let reach = (b.thigh + b.shin) * 0.95;
        let mut target = ball_local.unwrap_or(v3(s * 0.15, b.ankle_height, 0.75 * b.scale));
        target.y = target.y.clamp(b.ankle_height, 0.35);
        let hip = v3(s * b.hip_half_width, pose.pelvis.y - 0.18 * b.scale, 0.0);
        let to = target - hip;
        if to.length() > reach {
            target = hip + to * (reach / to.length());
        }
        pose.feet[i] = pose.feet[i].lerp(target, w);
        pose.foot_pitch[i] = lerp(pose.foot_pitch[i], 0.2, w);
        pose.foot_turn[i] = lerp(pose.foot_turn[i], s * 0.9, w); // side of the foot to the ball
        pose.knee_dir[i] = pose.knee_dir[i].lerp(v3(s * 0.7, 0.0, 1.0), w);
        pose.pelvis.y -= 0.17 * b.scale * w;
        pose.spine.x += 0.25 * w;
        pose.pelvis_rot.y += s * 0.25 * w;
        let o = 1 - i;
        pose.arms[o].abduct = lerp(pose.arms[o].abduct, 1.0, w);
        pose.arms[o].flex = lerp(pose.arms[o].flex, 0.4, w);
        pose.arms[i].flex = lerp(pose.arms[i].flex, -0.3, w);
        pose.arms[i].abduct = lerp(pose.arms[i].abduct, 0.5, w);
    }
    if p.stumble > 0.0 && p.dive_timer <= 0.0 && p.down <= 0.0 {
        // Off balance: torso pitches, arms flail.
        let k = clamp(p.stumble / 0.6, 0.0, 1.0);
        pose.spine.x += 0.3 * k;
        pose.pelvis.y -= 0.06 * k * b.scale;
        for (j, a) in pose.arms.iter_mut().enumerate() {
            a.abduct = lerp(a.abduct, 0.9 + 0.2 * j as f32, k);
            a.flex = lerp(a.flex, 0.5, k);
        }
    }
}

/// The foot a first touch is made with (None for the chest, or no touch).
pub fn touch_foot(p: &Player, frame: &Frame) -> Option<usize> {
    if !p.touch_trap || p.since_touch > 0.35 || p.touch_height > 0.75 {
        return None;
    }
    let at = frame.to_local(p.touch_at);
    let s = if at.x.abs() > 0.05 { at.x.signum() } else { p.preferred_foot().side() };
    Some(side_index(s))
}

/// First touch: cushion the ball with the foot, the thigh or the chest
/// depending on its height.
pub fn first_touch(pose: &mut Pose, p: &Player, b: &Body, frame: &Frame) -> Option<usize> {
    if !p.touch_trap || p.since_touch > 0.35 {
        return None;
    }
    let u = p.since_touch / 0.35;
    let w = (1.0 - smoothstep(0.4, 1.0, u)) * smoothstep(0.0, 0.05, u).max(0.6);
    let at = frame.to_local(p.touch_at);
    let h = p.touch_height;
    if h > 0.75 {
        // Chest: lean back, chest out, arms wide.
        pose.spine.x -= 0.32 * w;
        pose.chest.x -= 0.15 * w;
        pose.head.x += 0.35 * w;
        for a in pose.arms.iter_mut() {
            a.abduct = lerp(a.abduct, 0.9, w);
            a.flex = lerp(a.flex, 0.3, w);
        }
        return None;
    }
    let s = if at.x.abs() > 0.05 { at.x.signum() } else { p.preferred_foot().side() };
    let i = side_index(s);
    if h > 0.3 {
        // Thigh: knee up under the ball.
        let knee_up = v3(s * b.hip_half_width, b.ankle_height + 0.32 * b.scale, 0.22 * b.scale);
        pose.feet[i] = pose.feet[i].lerp(knee_up, w);
        pose.foot_pitch[i] = lerp(pose.foot_pitch[i], -0.5, w);
        pose.knee_dir[i] = pose.knee_dir[i].lerp(v3(s * 0.1, 0.6, 1.0), w);
        pose.spine.x -= 0.1 * w;
        pose.arms[1 - i].abduct = lerp(pose.arms[1 - i].abduct, 0.7, w);
    } else {
        // Foot: meet the ball with the inside of the foot and give with it.
        let give = v3(at.x, b.ankle_height + 0.04 + 0.6 * h, at.z - 0.12 - 0.15 * u);
        pose.feet[i] = pose.feet[i].lerp(give, w);
        pose.foot_turn[i] = lerp(pose.foot_turn[i], s * 0.9, w);
        pose.foot_pitch[i] = lerp(pose.foot_pitch[i], 0.15, w);
        pose.knee_dir[i] = pose.knee_dir[i].lerp(v3(s * 0.6, 0.0, 1.0), w);
        pose.arms[1 - i].abduct = lerp(pose.arms[1 - i].abduct, 0.55, w);
    }
    Some(i)
}

/// Header: arch back as the ball comes, then snap the forehead through it.
pub fn header(pose: &mut Pose, p: &Player, world: &World) {
    let spec = &world.player_spec;
    let mut w_cock = 0.0;
    if p.header.is_some() {
        let head = p.head(spec);
        let to = world.ball.pos - head;
        let closing = -to.dot(world.ball.vel - p.vel) / to.length().max(0.05);
        if closing > 0.0 {
            let t = to.length() / closing.max(1.0);
            w_cock = 1.0 - smoothstep(0.1, 0.45, t);
        }
    }
    let w_hit = if p.since_header < 0.35 { 1.0 - smoothstep(0.05, 0.35, p.since_header) } else { 0.0 };
    let snap = if p.since_header < 0.35 { smoothstep(0.0, 0.08, p.since_header) } else { 0.0 };
    let back = w_cock * (1.0 - w_hit);
    pose.spine.x += -0.28 * back + 0.25 * snap * w_hit;
    pose.chest.x += -0.12 * back + 0.15 * snap * w_hit;
    pose.head.x += -0.15 * back + 0.25 * snap * w_hit;
    let arms = back.max(w_hit);
    for a in pose.arms.iter_mut() {
        a.flex = lerp(a.flex, 0.7, arms);
        a.abduct = lerp(a.abduct, 0.7, arms);
        a.elbow = lerp(a.elbow, 1.1, arms);
    }
}

/// A keeper (not diving, not holding) reaches for a ball coming at him.
pub fn keeper_hands(pose: &mut Pose, p: &Player, b: &Body, frame: &Frame, world: &World) {
    if p.holding_ball || p.dive_timer > 0.0 || p.down > 0.0 || p.kick.is_some() || p.on_ground() {
        return;
    }
    let ball = world.ball.pos;
    let to = ball - p.pos;
    let d = to.flat_length();
    let approaching = world.ball.vel.flat().dot(-to.flat().normalize_or_zero()) > 3.0;
    if !approaching || d > 3.0 || ball.y > 2.6 || ball.y < 0.25 {
        return;
    }
    let w = 1.0 - smoothstep(0.8, 3.0, d);
    let local = frame.to_local(ball);
    for (i, a) in pose.arms.iter_mut().enumerate() {
        let s = if i == 0 { -1.0 } else { 1.0 };
        a.target = local + v3(s * 0.1, 0.0, -0.08);
        a.target_w = w;
    }
    // Get the body behind it: bend down for low balls.
    if ball.y < 0.9 {
        pose.pelvis.y -= 0.2 * b.scale * w * (1.0 - ball.y / 0.9);
        pose.spine.x += 0.45 * w * (1.0 - ball.y / 0.9);
    }
}

/// Goal: the scorers celebrate (arms up, pumping, little jumps); the other
/// team drops its heads, hands on the hips or on the head.
pub fn celebrate(pose: &mut Pose, b: &Body, joy: bool, time: f32, seed: f32, w: f32) {
    if w <= 0.0 {
        return;
    }
    let t = time + seed * 3.0;
    if joy {
        let pump = (t * 7.0).sin();
        for (i, a) in pose.arms.iter_mut().enumerate() {
            // Arms up in a V, pumping.
            let ph = if i == 0 { 0.0 } else { 0.6 };
            let up = 2.45 + 0.2 * (t * 7.0 + ph).sin();
            a.flex = lerp(a.flex, 0.35, w);
            a.abduct = lerp(a.abduct, up, w);
            a.elbow = lerp(a.elbow, 0.3 + 0.5 * pump.max(0.0), w);
            a.twist = lerp(a.twist, 0.4, w);
            a.target_w = 0.0;
        }
        pose.root_offset.y += 0.07 * b.scale * (t * 6.0).sin().max(0.0) * w;
        pose.spine.x -= 0.12 * w;
        pose.head.x -= 0.25 * w;
    } else {
        pose.spine.x += 0.15 * w;
        pose.head.x += 0.45 * w;
        let on_head = seed.fract() > 0.5;
        for (i, a) in pose.arms.iter_mut().enumerate() {
            let s = if i == 0 { -1.0 } else { 1.0 };
            a.target = if on_head {
                v3(s * 0.09, b.height() - 0.06, 0.02)
            } else {
                v3(s * (b.hip_half_width + 0.1), b.hip_height + 0.08 * b.scale, 0.02)
            };
            a.target_w = w;
        }
    }
}

/// Keeper dive: body stretched along the dive, the lower leg long, the upper
/// leg bent, both hands reaching (on top of the whole-body roll).
pub fn dive_limbs(pose: &mut Pose, b: &Body, side: f32, k: f32) {
    let lower = side_index(side);
    let upper = 1 - lower;
    pose.feet[upper].y += 0.18 * k * b.scale;
    pose.feet[upper].z += 0.12 * k;
    pose.knee_dir[upper] = pose.knee_dir[upper].lerp(v3(0.0, 0.3, 1.0), k);
    pose.feet[lower].x += side * 0.08 * k;
    pose.spine.z += side * 0.15 * k;
    pose.head.z -= side * 0.2 * k;
}
