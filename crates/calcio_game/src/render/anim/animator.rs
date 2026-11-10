//! Per-player animation state: advances gait phases, builds poses from the
//! simulation state, keeps planted feet locked to the ground and solves the
//! skeleton every frame.
//!
//! Layers, in order:
//! 1. idle / ready stance blended with the directional gait
//!    (forward, backpedal, side shuffle, open hips at speed);
//! 2. lean into acceleration and bank into turns (whole body about the feet);
//! 3. foot locking: a planted foot stays put in the world; when the body turns
//!    or drifts while standing, the feet re-plant with small shuffle steps;
//! 4. head (and chest) look at the ball;
//! 5. action overlays (kick, tackle, keeper).

use super::actions::{self, KickAnim};
use super::body::Body;
use super::locomotion::{self, Gait, Loco};
use super::moves;
use super::pose::Pose;
use super::rig::{fk, Frame, Skeleton};
use calcio_core::math::{approach_factor, clamp, lerp, smoothstep, v3, wrap_angle, Vec3};
use calcio_core::player::{Player, Role};
use calcio_core::World;
use std::f32::consts::PI;

/// A repositioning step taken while standing.
#[derive(Clone, Copy, Debug)]
struct Step {
    /// Start position (world, ground level) and heading.
    from: Vec3,
    from_heading: f32,
    t: f32,
    dur: f32,
}

/// A foot's contact with the ground.
#[derive(Clone, Copy, Debug, Default)]
struct Foot {
    /// Planted at `pos` (world, ground level) with world heading `heading`.
    planted: bool,
    pos: Vec3,
    heading: f32,
    /// Offset from the gait target at lift-off (character space), faded out
    /// during the swing, and its heading part.
    carry: Vec3,
    carry_turn: f32,
    swing_t: f32,
    step: Option<Step>,
    /// Where the foot was last shown (world, ground level) and its heading.
    shown: Vec3,
    shown_heading: f32,
    /// Driven by an action (kick, tackle) last frame.
    driven: bool,
}

/// Animation memory of one player.
#[derive(Clone, Debug)]
pub struct PlayerAnim {
    pub body: Body,
    /// Gait cycle phase (0..1).
    pub phase: f32,
    /// Smoothed ground speed.
    pub speed: f32,
    /// Direction of travel relative to the facing (0 = forward, + = right).
    pub travel: f32,
    /// Legs turned relative to the upper body.
    pub hip_turn: f32,
    pub pose: Pose,
    pub skel: Skeleton,
    vel: Vec3,
    acc: Vec3,
    backpedal: bool,
    gait_mode: bool,
    ready: f32,
    look: (f32, f32),
    feet: [Foot; 2],
    kick: Option<KickAnim>,
    /// Ball (world) when the current standing tackle started.
    tackle_ball: Option<Vec3>,
    last_since_tackle: f32,
    /// Goal celebration / dejection weight.
    cheer: f32,
    step_cool: f32,
    last_pos: Vec3,
    fresh: bool,
    seed: f32,
}

impl PlayerAnim {
    pub fn new(p: &Player) -> Self {
        let body = Body::for_player(p.id);
        let pose = Pose::stand(&body);
        let skel = fk(&pose, &body, &Frame::character(p.pos, p.facing));
        Self {
            body,
            phase: (p.id as f32 * 0.37).fract(),
            speed: 0.0,
            travel: 0.0,
            hip_turn: 0.0,
            pose,
            skel,
            vel: Vec3::ZERO,
            acc: Vec3::ZERO,
            backpedal: false,
            gait_mode: false,
            ready: 0.0,
            look: (0.0, 0.2),
            feet: [Foot::default(); 2],
            kick: None,
            tackle_ball: None,
            last_since_tackle: 10.0,
            cheer: 0.0,
            step_cool: 0.0,
            last_pos: p.pos,
            fresh: true,
            seed: p.id as f32 * 0.618,
        }
    }
}

/// Animates every player of a world.
#[derive(Clone, Debug, Default)]
pub struct Animator {
    pub players: Vec<PlayerAnim>,
    pub time: f32,
    /// Team that just scored (celebrations), set by the game each frame.
    pub celebrate: Option<usize>,
}

impl Animator {
    pub fn new() -> Self {
        Self::default()
    }

    /// Advances all animations by `dt` and solves the skeletons.
    pub fn update(&mut self, world: &World, dt: f32) {
        self.time += dt;
        if self.players.len() != world.players.len() {
            self.players = world.players.iter().map(PlayerAnim::new).collect();
        }
        for (st, p) in self.players.iter_mut().zip(world.players.iter()) {
            animate(st, p, world, self.time, dt, self.celebrate);
        }
    }

    pub fn skeleton(&self, id: usize) -> Option<&Skeleton> {
        self.players.get(id).map(|a| &a.skel)
    }
}

fn animate(st: &mut PlayerAnim, p: &Player, world: &World, time: f32, dt: f32, celebrate: Option<usize>) {
    let b = st.body;
    let frame = Frame::character(p.pos, p.facing);
    if (p.pos - st.last_pos).flat_length() > 1.0 {
        st.fresh = true; // repositioned (kick-off, restart)
    }
    st.last_pos = p.pos;

    // Smoothed ground velocity and acceleration.
    let vel = p.vel.flat();
    if st.fresh {
        st.vel = vel;
        st.acc = Vec3::ZERO;
    }
    let prev = st.vel;
    st.vel += (vel - st.vel) * approach_factor(12.0, dt);
    if dt > 1e-5 {
        st.acc += ((st.vel - prev) / dt - st.acc) * approach_factor(6.0, dt);
    }
    let local = frame.dir_to_local(st.vel);
    let speed = local.flat_length();
    st.speed = speed;
    if speed > 0.05 {
        st.travel = local.x.atan2(local.z);
    }
    let theta = st.travel;

    // Legs: backpedal (with hysteresis) or turn the hips into the run when the
    // sideways speed is high; the upper body keeps facing.
    if st.backpedal {
        if theta.abs() < 1.35 {
            st.backpedal = false;
        }
    } else if theta.abs() > 1.85 {
        st.backpedal = true;
    }
    let rel = if st.backpedal { wrap_angle(theta - PI) } else { theta };
    let lateral = speed * theta.sin().abs();
    let turn_target = clamp(rel * 0.9 * smoothstep(0.8, 2.5, lateral), -1.25, 1.25);
    if st.fresh {
        st.hip_turn = turn_target;
    }
    st.hip_turn += (turn_target - st.hip_turn) * approach_factor(6.0, dt);
    let dir = wrap_angle(theta - st.hip_turn);
    let gp = locomotion::params(&b, speed, dir);
    st.phase = (st.phase + gp.cadence * dt).fract();
    if st.gait_mode {
        if speed < 0.28 {
            st.gait_mode = false;
        }
    } else if speed > 0.42 {
        st.gait_mode = true;
    }

    // Ready stance near the ball (keepers whenever the ball is in their half).
    let keeper = p.role == Role::Goalkeeper;
    let d_ball = world.ball.pos.flat_distance(p.pos);
    let ready_t = if p.holding_ball {
        0.0
    } else if keeper {
        1.0 - smoothstep(14.0, 22.0, d_ball)
    } else {
        1.0 - smoothstep(3.0, 7.0, d_ball)
    };
    st.ready += (ready_t - st.ready) * approach_factor(3.0, dt);

    // 1. Base pose.
    let idle = locomotion::idle(&b, time, st.seed, st.ready, keeper);
    let loco = locomotion::locomote(&b, &Gait { speed, phase: st.phase, dir, hip_turn: st.hip_turn });
    let w = smoothstep(0.12, 0.7, speed);
    let mut pose = idle.lerp(&loco.pose, w);

    let body_down = p.down > 0.0 || p.dive_timer > 0.0 || p.on_ground();
    // 2. Lean into acceleration, bank into turns.
    if !body_down {
        let a = frame.dir_to_local(st.acc);
        // Actions set their own balance.
        let k = 0.55 * (1.0 - 0.75 * p.kick.map(|k| actions::kick_weight(k.progress())).unwrap_or(0.0));
        let pitch = clamp((a.z / 9.81).atan() * k, -0.28, 0.32);
        let roll = clamp((a.x / 9.81).atan() * k, -0.32, 0.32);
        lean(&mut pose, pitch, roll, st.hip_turn);
    }

    // 3. Feet on the ground.
    let mut locked = [!body_down; 2];
    if let Some(k) = p.kick {
        for (i, l) in locked.iter_mut().enumerate() {
            if actions::kick_drives_foot(&k, i, k.progress()) {
                *l = false;
            }
        }
    }
    let tackle_foot = if p.preferred_foot().side() < 0.0 { 0 } else { 1 };
    if p.since_tackle < 0.5 {
        locked[tackle_foot] = false;
    }
    if p.since_tackle < st.last_since_tackle {
        // A new tackle: remember where the ball was.
        st.tackle_ball = (world.ball.pos.flat_distance(p.pos) < 1.6).then_some(world.ball.pos);
    }
    st.last_since_tackle = p.since_tackle;
    if let Some(i) = moves::touch_foot(p, &frame) {
        locked[i] = false;
    }
    if p.jump.is_some() || p.air > 0.0 {
        locked = [false; 2];
    }
    lock_feet(st, &mut pose, &loco, &idle, &frame, p.facing, locked, dt);
    if !body_down {
        let contact = [0, 1].map(|i| {
            let f = &st.feet[i];
            if !locked[i] {
                0.0
            } else if f.planted {
                1.0
            } else if let Some(step) = f.step {
                smoothstep(0.7, 1.0, step.t / step.dur)
            } else {
                smoothstep(0.75, 1.0, f.swing_t / loco.swing_time.max(0.05))
            }
        });
        pose.fit_legs(&b, contact);
    }

    // 4. Look at the ball.
    let eye = frame.point(v3(0.0, b.height() * 0.93, 0.08));
    let to = frame.dir_to_local(world.ball.pos - eye);
    let horiz = to.flat_length().max(0.05);
    let limit = lerp(1.5, 1.0, smoothstep(3.0, 7.0, speed));
    let mut target = (to.x.atan2(to.z), (-to.y).atan2(horiz));
    if p.holding_ball || target.0.abs() > 2.4 {
        target = (0.0, 0.12);
    }
    target.0 = clamp(target.0, -limit, limit);
    target.1 = clamp(target.1, -0.5, 1.0);
    if st.fresh {
        st.look = target;
    }
    let k = approach_factor(7.0, dt);
    st.look.0 += (target.0 - st.look.0) * k;
    st.look.1 += (target.1 - st.look.1) * k;
    if !body_down {
        look(&mut pose, st.look.0, st.look.1);
    }

    // 5. Actions.
    let shown = [0, 1].map(|i| (st.feet[i].shown, st.feet[i].shown_heading));
    actions::kick(&mut pose, &mut st.kick, &b, p, world, &frame, shown);
    let tackle_ball = st.tackle_ball.map(|w| frame.to_local(w));
    moves::tackle(&mut pose, p, &b, tackle_ball);
    moves::first_touch(&mut pose, p, &b, &frame);
    moves::header(&mut pose, p, world);
    overlay_jump(&mut pose, p, &b, world);
    if keeper {
        moves::keeper_hands(&mut pose, p, &b, &frame, world);
        overlay_keeper(&mut pose, p, &b, world);
    }
    moves::slide(&mut pose, p, &b, &world.player_spec);
    moves::fall(&mut pose, p, &b);
    let cheer_t = if celebrate.is_some() && !p.on_ground() && p.dive_timer <= 0.0 { 1.0 } else { 0.0 };
    st.cheer += (cheer_t - st.cheer) * approach_factor(3.0, dt);
    moves::celebrate(&mut pose, &b, celebrate == Some(p.team), time, st.seed, st.cheer);
    // Feet driven by actions: remember where they are for the hand-back.
    for i in 0..2 {
        if !locked[i] {
            let f = &mut st.feet[i];
            f.shown = frame.point(v3(pose.feet[i].x, 0.0, pose.feet[i].z));
            f.shown_heading = p.facing + pose.foot_turn[i];
        }
    }
    st.pose = pose;
    st.skel = fk(&pose, &b, &frame);
    st.fresh = false;
}

/// Leans the whole body about the feet: forward/back (`pitch`) and sideways
/// (`roll`, positive = right), in character space.
fn lean(pose: &mut Pose, pitch: f32, roll: f32, hip_turn: f32) {
    let h = pose.pelvis.y;
    pose.pelvis.z += h * pitch.sin();
    pose.pelvis.x += h * roll.sin();
    pose.pelvis.y -= h * (1.0 - pitch.cos() * roll.cos());
    // The pelvis rotation is applied after its yaw: express the lean in the
    // turned hips' axes.
    let (sh, ch) = hip_turn.sin_cos();
    pose.pelvis_rot.x += 0.8 * (pitch * ch + roll * sh);
    pose.pelvis_rot.z += 0.8 * (roll * ch - pitch * sh);
    pose.head.x -= 0.6 * pitch;
    pose.head.z -= 0.6 * roll;
}

/// Turns the head (and some chest) to gaze at yaw / pitch in character space.
fn look(pose: &mut Pose, yaw: f32, pitch: f32) {
    let current = pose.pelvis_rot.y + pose.spine.y + pose.chest.y + pose.head.y;
    let extra = wrap_angle(yaw - current);
    pose.chest.y += 0.3 * extra;
    pose.head.y = clamp(pose.head.y + 0.7 * extra, -1.15, 1.15);
    // Looking down at a close ball also bends the upper back a little.
    pose.head.x = clamp(pose.head.x + 0.75 * pitch, -0.6, 0.85);
    pose.chest.x += 0.2 * pitch.max(0.0);
}

/// World-space foot locking and repositioning steps.
///
/// A planted foot remembers its flat-foot ankle position on the ground; while
/// it rolls onto the heel or the ball of the foot the ankle is shifted so the
/// contact point stays put.
#[allow(clippy::too_many_arguments)]
fn lock_feet(st: &mut PlayerAnim, pose: &mut Pose, loco: &Loco, idle: &Pose, frame: &Frame, facing: f32, locked: [bool; 2], dt: f32) {
    let b = st.body;
    let ground = |l: Vec3| frame.point(v3(l.x, 0.0, l.z));
    let side = |i: usize| if i == 0 { -1.0 } else { 1.0 };
    // Flat-foot reference (world, ground) of a local ankle target.
    let flat_ref = |l: Vec3, pitch: f32, angle: f32| {
        let (ax, az) = locomotion::foot_axis(angle);
        let r = locomotion::roll_shift(&b, pitch);
        ground(v3(l.x - ax * r, 0.0, l.z - az * r))
    };
    let gait = st.gait_mode;
    st.step_cool -= dt;
    if st.fresh {
        for i in 0..2 {
            let at = flat_ref(pose.feet[i], pose.foot_pitch[i], pose.foot_turn[i] + side(i) * pose.foot_yaw[i]);
            st.feet[i] = Foot {
                planted: true,
                pos: at,
                heading: facing + pose.foot_turn[i],
                shown: ground(pose.feet[i]),
                shown_heading: facing + pose.foot_turn[i],
                ..Foot::default()
            };
        }
    }

    // Standing: re-plant the foot that is furthest from where it should be.
    if !gait && st.step_cool <= 0.0 && (0..2).all(|i| locked[i] && st.feet[i].planted) {
        let mut best = (0usize, 0.0f32);
        for i in 0..2 {
            let f = &st.feet[i];
            let l = frame.to_local(f.pos);
            let home = idle.feet[i];
            let err = v3(l.x - home.x, 0.0, l.z - home.z).length() + 0.12 * wrap_angle(f.heading - facing - idle.foot_turn[i]).abs();
            if err > best.1 {
                best = (i, err);
            }
        }
        if best.1 > 0.07 * b.scale {
            let f = &mut st.feet[best.0];
            f.planted = false;
            f.step = Some(Step { from: f.shown, from_heading: f.heading, t: 0.0, dur: lerp(0.26, 0.17, smoothstep(0.1, 0.4, best.1)) });
        }
    }

    for i in 0..2 {
        let proc = pose.feet[i];
        let proc_turn = pose.foot_turn[i];
        let toe_out = side(i) * pose.foot_yaw[i];
        let f = &mut st.feet[i];
        if !locked[i] {
            // Driven by an action: hand over to it, re-plant afterwards.
            f.planted = false;
            f.step = None;
            f.driven = true;
            continue;
        }
        if f.driven {
            // Handed back by an action: blend from where it was left.
            f.driven = false;
            if gait {
                let l = frame.to_local(f.shown);
                f.carry = v3(l.x - proc.x, 0.0, l.z - proc.z);
                f.carry_turn = wrap_angle(f.shown_heading - facing - proc_turn);
                f.swing_t = 0.0;
            } else {
                f.step = Some(Step { from: f.shown, from_heading: f.shown_heading, t: 0.0, dur: 0.22 });
            }
        }

        // Contact transitions.
        if gait {
            if loco.stance[i] {
                if !f.planted {
                    let k = 1.0 - smoothstep(0.0, 0.7 * loco.swing_time, f.swing_t);
                    let (at, turn) = if f.step.is_some() {
                        (frame.to_local(f.shown), wrap_angle(f.shown_heading - facing))
                    } else {
                        (proc + f.carry * k, proc_turn + f.carry_turn * k)
                    };
                    f.planted = true;
                    f.step = None;
                    f.pos = flat_ref(at, pose.foot_pitch[i], turn + toe_out);
                    f.heading = facing + turn;
                }
            } else if f.planted || f.step.is_some() {
                let (from, heading) = (f.shown, f.shown_heading);
                let l = frame.to_local(from);
                f.planted = false;
                f.step = None;
                f.carry = v3(l.x - proc.x, 0.0, l.z - proc.z);
                f.carry_turn = wrap_angle(heading - facing - proc_turn);
                f.swing_t = 0.0;
            }
        } else if !f.planted && f.step.is_none() {
            // Coming out of a run or an action: finish with a step.
            f.step = Some(Step { from: f.shown, from_heading: f.shown_heading, t: 0.0, dur: 0.2 });
        }

        // Targets.
        if f.planted {
            if !gait {
                pose.feet[i].y = idle.feet[i].y;
                pose.foot_pitch[i] = idle.foot_pitch[i];
            }
            let pitch = pose.foot_pitch[i];
            let mut l = frame.to_local(f.pos);
            if v3(l.x - proc.x, 0.0, l.z - proc.z).length() > 0.9 {
                f.pos = flat_ref(proc, pitch, proc_turn + toe_out);
                f.heading = facing + proc_turn;
                l = frame.to_local(f.pos);
            }
            // Pivot on the ball of the foot rather than twisting the knee.
            let twist = wrap_angle(f.heading - facing - proc_turn);
            if twist.abs() > 0.6 {
                f.heading = facing + proc_turn + twist.signum() * 0.6;
            }
            let turn = wrap_angle(f.heading - facing);
            let (ax, az) = locomotion::foot_axis(turn + toe_out);
            let r = locomotion::roll_shift(&b, pitch);
            pose.feet[i].x = l.x + ax * r;
            pose.feet[i].z = l.z + az * r;
            pose.knee_dir[i] = rotate_y(pose.knee_dir[i], 0.7 * wrap_angle(turn - proc_turn));
            pose.foot_turn[i] = turn;
        } else if let Some(step) = f.step.as_mut() {
            step.t += dt;
            let u = clamp(step.t / step.dur, 0.0, 1.0);
            let e = smoothstep(0.0, 1.0, u);
            let from = frame.to_local(step.from);
            let home = idle.feet[i];
            let lift = 0.055 * b.scale * (PI * u).sin();
            pose.feet[i] = v3(lerp(from.x, home.x, e), home.y + lift, lerp(from.z, home.z, e));
            pose.foot_pitch[i] = idle.foot_pitch[i] + 0.2 * (PI * u).sin();
            let from_turn = wrap_angle(step.from_heading - facing);
            let turn = from_turn + wrap_angle(idle.foot_turn[i] - from_turn) * e;
            pose.knee_dir[i] = rotate_y(idle.knee_dir[i], 0.7 * wrap_angle(turn - idle.foot_turn[i]));
            pose.foot_turn[i] = turn;
            if u >= 1.0 {
                f.planted = true;
                f.step = None;
                f.pos = flat_ref(home, idle.foot_pitch[i], idle.foot_turn[i] + side(i) * idle.foot_yaw[i]);
                f.heading = facing + idle.foot_turn[i];
                st.step_cool = 0.03;
            }
        } else {
            f.swing_t += dt;
            let k = 1.0 - smoothstep(0.0, 0.7 * loco.swing_time, f.swing_t);
            pose.feet[i].x += f.carry.x * k;
            pose.feet[i].z += f.carry.z * k;
            pose.foot_turn[i] += f.carry_turn * k;
        }
        f.shown = ground(pose.feet[i]);
        f.shown_heading = facing + pose.foot_turn[i];
    }
}

/// Rotates a character-space direction about the vertical (positive = right).
fn rotate_y(v: Vec3, a: f32) -> Vec3 {
    let (s, c) = a.sin_cos();
    v3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c)
}

/// Jump: dip and push at take-off, legs tucked in the air (the body follows
/// the core's flight), knees absorbing the landing. Arms swing up for balance.
fn overlay_jump(pose: &mut Pose, p: &Player, b: &Body, world: &World) {
    let Some(j) = p.jump else {
        return;
    };
    let spec = &world.player_spec;
    if let Some(l) = j.landed {
        let k = 1.0 - clamp(l / spec.landing, 0.0, 1.0);
        pose.pelvis.y -= 0.12 * b.scale * (std::f32::consts::PI * (1.0 - k) * 0.5 + 0.5).sin() * k;
        pose.spine.x += 0.15 * k;
        return;
    }
    if !j.airborne {
        // Dip, then extend through the push.
        let k = clamp(j.t / j.takeoff, 0.0, 1.0);
        let dip = (k * std::f32::consts::PI).sin();
        pose.pelvis.y -= 0.14 * b.scale * dip;
        pose.spine.x += 0.2 * dip;
        for a in pose.arms.iter_mut() {
            a.flex = lerp(a.flex, -0.4, dip);
            a.elbow = lerp(a.elbow, 0.5, dip);
        }
        for i in 0..2 {
            pose.foot_pitch[i] = lerp(pose.foot_pitch[i], -0.5, smoothstep(0.6, 1.0, k));
        }
        return;
    }
    // In the air: the whole body rides on the jump.
    pose.root_offset.y += p.air;
    let up = clamp(p.air_vel / 3.0, -1.0, 1.0);
    for i in 0..2 {
        pose.feet[i].y += 0.08 * b.scale + 0.06 * (1.0 - up.abs());
        pose.feet[i].z -= 0.06;
        pose.foot_pitch[i] = -0.5;
        pose.knee_dir[i] = v3(if i == 0 { -0.15 } else { 0.15 }, 0.0, 1.0);
    }
    let header = p.header.is_some();
    for a in pose.arms.iter_mut() {
        a.flex = lerp(a.flex, if header { 0.9 } else { 0.6 }, 0.9);
        a.abduct = lerp(a.abduct, 0.55, 0.9);
        a.elbow = lerp(a.elbow, 1.0, 0.9);
    }
    pose.spine.x -= 0.12 * up.max(0.0);
}

/// Keeper holding the ball, diving and lying down.
fn overlay_keeper(pose: &mut Pose, p: &Player, b: &Body, world: &World) {
    if p.holding_ball && p.kick.is_none() {
        for i in 0..2 {
            let s = if i == 0 { -1.0 } else { 1.0 };
            pose.arms[i].target = v3(s * 0.11, 1.05 * b.scale, 0.3);
            pose.arms[i].target_w = 1.0;
        }
        return;
    }
    if p.down <= 0.0 {
        return;
    }
    let fwd = p.forward();
    let right = fwd.cross(Vec3::Y);
    let side = if p.dive_dir.dot(right) >= 0.0 { 1.0 } else { -1.0 };
    let k = clamp(p.down, 0.0, 1.0);
    pose.root_rot.z = side * 1.32 * k;
    if p.dive_timer > 0.0 {
        let t = 1.0 - p.dive_timer / world.keeper_spec.dive_time;
        pose.root_offset.y = (p.dive_height * 0.45).min(0.9) * (t * std::f32::consts::PI).sin();
    }
    for i in 0..2 {
        pose.arms[i].flex = 2.9 * k + pose.arms[i].flex * (1.0 - k);
        pose.arms[i].elbow *= 1.0 - k;
        pose.arms[i].abduct = 0.15;
        pose.arms[i].target_w = 0.0;
    }
    moves::dive_limbs(pose, b, side, k);
}

#[cfg(test)]
mod tests {
    use super::*;
    use calcio_core::math::v3;

    fn world_with_player() -> (World, usize) {
        let mut w = World::new();
        let id = w.spawn_player(0, Role::Pivot, 9, v3(0.0, 0.0, 0.0));
        let spec = w.ball_spec.clone();
        w.ball.place(&spec, 5.0, 5.0);
        (w, id)
    }

    /// Drives a player with a velocity / facing function and returns the
    /// worst world-space slide of a foot while it is planted.
    fn max_planted_slide(f: impl Fn(f32) -> (Vec3, f32), secs: f32) -> f32 {
        let (mut world, id) = world_with_player();
        let mut anim = Animator::new();
        let dt = 1.0 / 120.0;
        let mut t = 0.0;
        let mut worst = 0.0f32;
        let mut prev: [Option<(u8, Vec3)>; 2] = [None; 2];
        while t < secs {
            let (vel, facing) = f(t);
            {
                let p = &mut world.players[id];
                p.vel = vel;
                p.facing = facing;
                p.pos += vel * dt;
            }
            anim.update(&world, dt);
            let st = &anim.players[id];
            assert!(st.skel.is_finite(), "t {t}");
            for i in 0..2 {
                // contact point: heel, toe or (flat) ankle
                let pitch = st.pose.foot_pitch[i];
                let (kind, c) = if pitch > 0.03 {
                    (1, st.skel.heel[i])
                } else if pitch < -0.03 {
                    (2, st.skel.toe[i])
                } else {
                    (0, st.skel.ankle[i])
                };
                if st.feet[i].planted && t > 0.5 {
                    if let Some((k, q)) = prev[i] {
                        if k == kind {
                            let sp = (c - q).flat_length() / dt;
                            if sp > 0.25 && std::env::var("ANIM_DIAG").is_ok() {
                                println!("t {t:.3} foot {i} kind {kind} speed {sp:.2} phase {:.3} pitch {pitch:.2} gait {} c {:?}", st.phase, st.gait_mode, c);
                            }
                            worst = worst.max(sp);
                        }
                    }
                    prev[i] = Some((kind, c));
                } else {
                    prev[i] = None;
                }
            }
            t += dt;
        }
        worst
    }

    #[test]
    fn planted_feet_do_not_slide() {
        // Straight run, backpedal, side shuffle, curve and a turn on the spot.
        let cases: [(&str, Box<dyn Fn(f32) -> (Vec3, f32)>); 5] = [
            ("run", Box::new(|_| (v3(4.0, 0.0, 0.0), 0.0))),
            ("back", Box::new(|_| (v3(-2.5, 0.0, 0.0), 0.0))),
            ("side", Box::new(|_| (v3(0.0, 0.0, 1.2), 0.0))),
            ("curve", Box::new(|t: f32| (Vec3::from_heading(0.8 * t) * 5.0, 0.8 * t))),
            ("turn", Box::new(|t: f32| (Vec3::ZERO, 2.5 * t))),
        ];
        for (name, f) in cases.iter() {
            let slide = max_planted_slide(f, 4.0);
            assert!(slide < 0.25, "{name}: planted foot slides at {slide} m/s");
        }
    }

    #[test]
    fn turning_on_the_spot_takes_steps() {
        let (mut world, id) = world_with_player();
        let mut anim = Animator::new();
        let dt = 1.0 / 120.0;
        let mut steps = 0;
        let mut was = [true; 2];
        for k in 0..240 {
            world.players[id].facing = (k as f32 * dt * 3.0).min(PI);
            anim.update(&world, dt);
            for i in 0..2 {
                let planted = anim.players[id].feet[i].planted;
                if was[i] && !planted {
                    steps += 1;
                }
                was[i] = planted;
            }
        }
        assert!((2..=8).contains(&steps), "{steps} steps for a half turn");
        // and ends standing square again
        let st = &anim.players[id];
        for i in 0..2 {
            assert!(st.feet[i].planted);
            assert!(wrap_angle(st.feet[i].heading - PI).abs() < 0.4);
        }
    }

    /// The kicking foot is at the ball when the core strikes it.
    #[test]
    fn kicking_foot_meets_the_ball_at_contact() {
        use calcio_core::events::GameEvent;
        use calcio_core::kick::{KickKind, KickRequest};
        for (kind, approach, lat) in [(KickKind::Shot, 0.0, 0.1), (KickKind::Pass, 2.0, 0.1), (KickKind::Lob, 1.0, -0.15), (KickKind::Cross, 2.5, 0.2), (KickKind::Shot, 3.0, -0.1)] {
            let mut world = World::new();
            let id = world.spawn_player(0, Role::Pivot, 9, v3(-approach * 0.5, 0.0, 0.0));
            world.players[id].vel = v3(approach, 0.0, 0.0);
            let spec = world.ball_spec.clone();
            world.ball.place(&spec, 0.5, lat);
            let mut anim = Animator::new();
            for _ in 0..60 {
                anim.update(&world, 1.0 / 240.0);
            }
            let target = v3(16.0, 0.5, 1.0);
            world.players[id].pending_kick = Some(KickRequest::new(kind, (target - world.ball.pos).flat(), 0.7).with_target(target));
            let mut met = None;
            for _ in 0..480 {
                world.players[id].desired = v3(approach, 0.0, 0.0);
                let ball_before = world.ball.pos;
                world.step(calcio_core::PHYSICS_DT);
                anim.update(&world, calcio_core::PHYSICS_DT);
                for e in world.drain_events() {
                    if let GameEvent::Kick { .. } = e {
                        let k = world.players[id].kick.expect("kick state");
                        let i = if k.foot.side() < 0.0 { 0 } else { 1 };
                        let s = &anim.players[id].skel;
                        // closest point of the foot (ankle..toe) to the ball centre
                        let mut d = f32::MAX;
                        for j in 0..=8 {
                            let q = s.ankle[i].lerp(s.toe[i], j as f32 / 8.0);
                            d = d.min(q.distance(ball_before));
                        }
                        met = Some(d);
                    }
                }
                assert!(anim.players[id].skel.is_finite());
            }
            let d = met.unwrap_or_else(|| panic!("{kind:?}: no kick"));
            assert!(d < 0.2, "{kind:?} approach {approach}: foot {d} m from the ball at contact");
        }
    }
}
