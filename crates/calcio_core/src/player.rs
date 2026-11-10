//! Footballers: movement physics, collisions with each other and the arena, and
//! the ball bouncing off their bodies.

use crate::ball::{Ball, BallSpec, GRAVITY};
use crate::contact::{self, Surface};
use crate::math::{clamp, closest_point_on_segment, lerp, rotate_towards, v3, Rng, Vec3};
use crate::pitch::{Capsule, FramePart, Pitch, Side};

/// Tactical role (used by formations and AI).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Goalkeeper,
    /// Last defender (futsal "fixo").
    Defender,
    /// Wide player (futsal "ala").
    Winger,
    /// Most advanced player (futsal "pivô").
    Pivot,
}

/// Athletic and body parameters shared by all players.
#[derive(Clone, Debug)]
pub struct PlayerSpec {
    /// Personal-space radius for player–player and arena collisions.
    pub radius: f32,
    /// Radius of the body capsule for ball collisions.
    pub body_radius: f32,
    pub height: f32,
    pub mass: f32,
    /// Acceleration from standing / at full speed (m/s²).
    pub accel_slow: f32,
    pub accel_fast: f32,
    /// Braking deceleration (m/s²).
    pub decel: f32,
    pub walk_speed: f32,
    pub jog_speed: f32,
    pub sprint_speed: f32,
    /// Top speeds while facing elsewhere (jockeying): backwards and sideways.
    pub back_speed: f32,
    pub side_speed: f32,
    /// Speed factor while in possession (dribbling).
    pub with_ball_factor: f32,
    /// Turn rate (rad/s) when standing / at sprint speed.
    pub turn_slow: f32,
    pub turn_fast: f32,
    /// Ball bounce off the body.
    pub body_surface: Surface,
    /// Jump height (m, feet off the ground) standing and with a run-up.
    pub jump_standing: f32,
    pub jump_running: f32,
    /// Take-off (dip and push) time standing and running (s).
    pub takeoff_standing: f32,
    pub takeoff_running: f32,
    /// Knees absorbing the landing (s): no new jump, slower acceleration.
    pub landing: f32,
    /// Centre of the head above the feet when standing.
    pub head_height: f32,
    /// Slide tackle: speed added to the run (m/s), launch speed limits,
    /// sliding friction (m/s²), time lying after stopping and getting up (s).
    pub slide_boost: f32,
    pub slide_min_speed: f32,
    pub slide_max_speed: f32,
    pub slide_decel: f32,
    pub slide_lie: f32,
    pub slide_getup: f32,
    /// A tripped player is on the ground this long (s).
    pub fall_time: f32,
}

impl Default for PlayerSpec {
    fn default() -> Self {
        Self {
            radius: 0.32,
            body_radius: 0.2,
            height: 1.8,
            mass: 75.0,
            accel_slow: 7.0,
            accel_fast: 3.2,
            decel: 10.0,
            walk_speed: 1.8,
            jog_speed: 5.4,
            sprint_speed: 7.5,
            back_speed: 4.0,
            side_speed: 5.0,
            with_ball_factor: 0.92,
            turn_slow: 9.0,
            turn_fast: 4.0,
            body_surface: Surface {
                restitution: 0.35,
                restitution_drop: 0.005,
                restitution_min: 0.2,
                friction: 0.35,
            },
            jump_standing: 0.42,
            jump_running: 0.6,
            takeoff_standing: 0.2,
            takeoff_running: 0.11,
            landing: 0.18,
            head_height: 1.68,
            slide_boost: 1.5,
            slide_min_speed: 4.0,
            slide_max_speed: 8.0,
            slide_decel: 6.5,
            slide_lie: 0.3,
            slide_getup: 0.55,
            fall_time: 1.5,
        }
    }
}

/// A slide tackle: feet first along `dir`, leading leg extended.
#[derive(Clone, Copy, Debug)]
pub struct Slide {
    pub t: f32,
    pub dir: Vec3,
    /// Leading (extended) leg: −1 left, +1 right.
    pub leg: f32,
    /// The leg met the ball.
    pub touched: bool,
    /// Seconds since the slide came to a stop (lying, then getting up).
    pub stopped: Option<f32>,
}

/// On the ground after being tripped.
#[derive(Clone, Copy, Debug)]
pub struct Fall {
    pub t: f32,
    /// Direction the body falls (forwards).
    pub dir: Vec3,
    pub duration: f32,
}

/// A jump: take-off, flight, landing.
#[derive(Clone, Copy, Debug)]
pub struct Jump {
    /// Seconds since the jump started.
    pub t: f32,
    /// Take-off duration (dip and push).
    pub takeoff: f32,
    /// Launch speed upwards (m/s).
    pub launch: f32,
    pub airborne: bool,
    /// Seconds since landing (once landed).
    pub landed: Option<f32>,
}

/// Where a player wants to head the ball.
#[derive(Clone, Copy, Debug)]
pub struct HeaderAim {
    /// Aim direction on the ground (unit).
    pub dir: Vec3,
    /// Exact target, if any (point in the goal, teammate).
    pub target: Option<Vec3>,
    /// 0..1: how hard (glancing flick .. powerful header).
    pub power: f32,
    /// Seconds left for the ball to arrive.
    pub ttl: f32,
}

/// One footballer.
#[derive(Clone, Debug)]
pub struct Player {
    pub id: usize,
    /// 0 = home, 1 = away.
    pub team: usize,
    pub role: Role,
    pub number: u8,
    /// Feet position on the ground (y = 0).
    pub pos: Vec3,
    pub vel: Vec3,
    /// Body heading (radians, see `Vec3::heading`).
    pub facing: f32,
    /// Desired ground velocity requested by the controller (human or AI).
    pub desired: Vec3,
    /// Optional heading to face when standing still.
    pub look_at: Option<Vec3>,
    pub sprint: bool,
    /// Walk instead of jogging.
    pub walk: bool,
    /// Keep facing `look_at` while moving (jockeying): allows backward and side running.
    pub strafe: bool,
    pub human: bool,
    /// While > 0 the player is off balance (failed tackle) and slow.
    pub stumble: f32,
    /// While > 0 the ball passes through this player's body (just kicked it).
    pub ball_ghost: f32,
    /// Distance run, drives the run-cycle animation.
    pub stride: f32,
    /// Seconds since the last kick (animation).
    pub since_kick: f32,
    /// Seconds since the last tackle attempt (animation).
    pub since_tackle: f32,
    /// Wants to control the ball when it comes into reach.
    pub dribble: bool,
    /// While > 0 the player cannot touch the ball again.
    pub touch_cooldown: f32,
    /// Goalkeeper holding the ball in his hands.
    pub holding_ball: bool,
    /// Technique 0..1 (touch and kick accuracy).
    pub skill: f32,
    /// Goalkeeper dive: lateral direction, remaining time, hand height.
    pub dive_dir: Vec3,
    pub dive_timer: f32,
    pub dive_height: f32,
    /// 0 = upright, 1 = lying on the ground (after a dive).
    pub down: f32,
    /// Seconds the keeper has been holding the ball.
    pub hold_timer: f32,
    /// Seconds until the keeper can attempt another save.
    pub save_cooldown: f32,
    /// Wants to make a standing tackle this step.
    pub tackle_request: bool,
    /// Seconds until another tackle is allowed.
    pub tackle_cooldown: f32,
    /// Kick waiting to be executed when the ball is in reach.
    pub pending_kick: Option<crate::kick::KickRequest>,
    /// Kick in progress (wind-up, contact, follow-through).
    pub kick: Option<crate::kick::KickState>,
    /// Prefers the left foot (and throws with the left hand).
    pub left_footed: bool,
    /// Disables the automatic run onto the ball (e.g. the human chose to leave it).
    pub human_free_run: bool,
    /// Wants to jump now: how high (m).
    pub jump_request: Option<f32>,
    /// Jump in progress.
    pub jump: Option<Jump>,
    /// Feet above the ground (m) and vertical speed.
    pub air: f32,
    pub air_vel: f32,
    /// Ready to head the ball (aimed) when it meets the head.
    pub header: Option<HeaderAim>,
    /// Wants to slide tackle along this direction.
    pub slide_request: Option<Vec3>,
    pub slide: Option<Slide>,
    /// Tripped and on the ground.
    pub fall: Option<Fall>,
    /// Last touch of the ball with the foot / body (animation): seconds since,
    /// height of the ball's bottom, trap or dribble touch, where the ball was.
    pub since_touch: f32,
    pub touch_height: f32,
    pub touch_trap: bool,
    pub touch_at: Vec3,
    /// Seconds since the last aimed header (animation).
    pub since_header: f32,
}

impl Player {
    pub fn new(id: usize, team: usize, role: Role, number: u8, pos: Vec3) -> Self {
        Self {
            id,
            team,
            role,
            number,
            pos: v3(pos.x, 0.0, pos.z),
            vel: Vec3::ZERO,
            facing: 0.0,
            desired: Vec3::ZERO,
            look_at: None,
            sprint: false,
            walk: false,
            strafe: false,
            human: false,
            stumble: 0.0,
            ball_ghost: 0.0,
            stride: 0.0,
            since_kick: 10.0,
            since_tackle: 10.0,
            dribble: true,
            touch_cooldown: 0.0,
            holding_ball: false,
            skill: 0.75,
            human_free_run: false,
            pending_kick: None,
            kick: None,
            left_footed: id % 4 == 2,
            jump_request: None,
            jump: None,
            air: 0.0,
            air_vel: 0.0,
            header: None,
            slide_request: None,
            slide: None,
            fall: None,
            since_touch: 10.0,
            touch_height: 0.0,
            touch_trap: false,
            touch_at: Vec3::ZERO,
            since_header: 10.0,
            tackle_request: false,
            tackle_cooldown: 0.0,
            dive_dir: Vec3::ZERO,
            dive_timer: 0.0,
            dive_height: 0.0,
            down: 0.0,
            hold_timer: 0.0,
            save_cooldown: 0.0,
        }
    }

    #[inline]
    pub fn speed(&self) -> f32 {
        self.vel.flat_length()
    }

    /// Unit facing vector on the ground.
    #[inline]
    pub fn forward(&self) -> Vec3 {
        Vec3::from_heading(self.facing)
    }

    /// Top speed right now.
    pub fn max_speed(&self, spec: &PlayerSpec, with_ball: bool) -> f32 {
        let mut s = if self.walk {
            spec.walk_speed
        } else if self.sprint {
            spec.sprint_speed
        } else {
            spec.jog_speed
        };
        if with_ball {
            s *= spec.with_ball_factor;
        }
        if self.stumble > 0.0 {
            s *= 0.35;
        }
        s
    }

    /// Body capsule for ball collisions (follows a jump; lies down in a slide
    /// or a fall).
    pub fn body(&self, spec: &PlayerSpec) -> Capsule {
        if let Some(s) = self.slide {
            // Reclined behind the hips, feet first.
            return Capsule {
                a: self.pos + v3(0.0, 0.2, 0.0),
                b: self.pos - s.dir * 0.55 + v3(0.0, 0.5, 0.0),
                radius: spec.body_radius,
                part: FramePart::Support,
            };
        }
        if let Some(f) = self.fall {
            return Capsule {
                a: self.pos + v3(0.0, 0.15, 0.0),
                b: self.pos + f.dir * 0.95 + v3(0.0, 0.15, 0.0),
                radius: 0.17,
                part: FramePart::Support,
            };
        }
        Capsule {
            a: self.pos + v3(0.0, 0.25 + self.air, 0.0),
            b: self.pos + v3(0.0, spec.height - 0.15 + self.air, 0.0),
            radius: spec.body_radius,
            part: FramePart::Support,
        }
    }

    /// Top speed in world direction `dir` given the current facing: running
    /// backwards or sideways is slower than running forwards.
    pub fn max_speed_towards(&self, spec: &PlayerSpec, with_ball: bool, dir: Vec3) -> f32 {
        let fwd_max = self.max_speed(spec, with_ball);
        if !self.strafe {
            return fwd_max;
        }
        let c = dir.flat().normalize_or(self.forward()).dot(self.forward());
        let side = spec.side_speed.min(fwd_max);
        let back = spec.back_speed.min(fwd_max);
        if c >= 0.0 {
            side + (fwd_max - side) * c
        } else {
            side + (back - side) * (-c)
        }
    }

    /// Integrates movement towards `desired` with realistic acceleration limits.
    pub fn integrate(&mut self, spec: &PlayerSpec, with_ball: bool, dt: f32) {
        if self.step_ground(spec, dt) {
            self.pos += self.vel * dt;
            self.pos.y = 0.0;
            self.integrate_timers(dt);
            return;
        }
        if self.step_jump(spec, dt) {
            // In the air: momentum only.
            self.pos += self.vel * dt;
            self.pos.y = 0.0;
            self.stride += self.speed() * dt;
            self.integrate_timers(dt);
            return;
        }
        let want = self.desired.flat();
        let vmax = self.max_speed_towards(spec, with_ball, want);
        let target = want.clamp_length(vmax);
        let speed = self.speed();
        let dv = target - self.vel;
        // Braking is stronger than accelerating; acceleration fades near top speed.
        let braking = target.length() < speed - 0.05 || target.dot(self.vel) < 0.0;
        let frac = clamp(speed / spec.sprint_speed, 0.0, 1.0);
        let mut accel = if braking {
            spec.decel
        } else {
            lerp(spec.accel_slow, spec.accel_fast, frac)
        };
        if self.strafe && !braking && target.dot(self.forward()) < 0.0 {
            accel *= 0.75; // pushing off backwards is weaker
        }
        if self.jump.is_some() {
            accel *= 0.35; // planted for take-off / absorbing the landing
        }
        self.vel += dv.clamp_length(accel * dt);
        self.vel.y = 0.0;
        self.pos += self.vel * dt;
        self.pos.y = 0.0;

        // Facing: follow the direction of travel (or the look-at point when
        // strafing), turning slower at speed.
        let turn = lerp(spec.turn_slow, spec.turn_fast, frac);
        let strafe_look = if self.strafe {
            self.look_at.map(|t| t - self.pos).filter(|d| d.flat_length() > 0.05).map(|d| d.heading())
        } else {
            None
        };
        let kick_dir = self.kick.map(|k| k.req.dir.heading());
        let want = if let Some(h) = kick_dir {
            Some(h)
        } else if let Some(h) = strafe_look {
            Some(h)
        } else if speed > 0.6 {
            Some(self.vel.heading())
        } else if let Some(t) = self.look_at {
            let d = t - self.pos;
            if d.flat_length() > 0.05 {
                Some(d.heading())
            } else {
                None
            }
        } else if target.length() > 0.05 {
            Some(target.heading())
        } else {
            None
        };
        if let Some(h) = want {
            let rate = if kick_dir.is_some() { crate::kick::KICK_TURN_RATE } else { turn };
            self.facing = rotate_towards(self.facing, h, rate * dt);
        }

        self.stride += speed * dt;
        self.integrate_timers(dt);
    }

    /// Starts a requested slide and advances a slide or a fall. Returns true
    /// while the player is on the ground (no control: friction only).
    fn step_ground(&mut self, spec: &PlayerSpec, dt: f32) -> bool {
        if let Some(dir) = self.slide_request.take() {
            if self.can_act() && self.jump.is_none() && self.role != Role::Goalkeeper {
                let dir = dir.flat().normalize_or(self.forward());
                let along = self.vel.flat().dot(dir).max(0.0);
                let v = clamp(along + spec.slide_boost, spec.slide_min_speed, spec.slide_max_speed);
                self.vel = dir * v;
                self.facing = dir.heading();
                self.slide = Some(Slide { t: 0.0, dir, leg: self.preferred_foot().side(), touched: false, stopped: None });
                self.pending_kick = None;
                self.header = None;
                self.tackle_cooldown = 1.0;
            }
        }
        let friction = |v: &mut Vec3, decel: f32| {
            let s = v.flat_length();
            let ns = (s - decel * dt).max(0.0);
            *v = if s > 1e-6 { v.flat() * (ns / s) } else { Vec3::ZERO };
        };
        if let Some(mut s) = self.slide {
            s.t += dt;
            friction(&mut self.vel, spec.slide_decel);
            if self.vel.flat_length() < 0.3 {
                self.vel = Vec3::ZERO;
                let st = s.stopped.get_or_insert(0.0);
                *st += dt;
                if *st >= spec.slide_lie + spec.slide_getup {
                    self.slide = None;
                    return true;
                }
            }
            self.slide = Some(s);
            return true;
        }
        if let Some(mut f) = self.fall {
            f.t += dt;
            friction(&mut self.vel, 9.0);
            self.fall = if f.t >= f.duration { None } else { Some(f) };
            return true;
        }
        false
    }

    /// Sliding or lying after a trip.
    pub fn on_ground(&self) -> bool {
        self.slide.is_some() || self.fall.is_some()
    }

    /// Free to start an action (kick, tackle, slide, jump).
    pub fn can_act(&self) -> bool {
        !self.on_ground() && self.stumble <= 0.0 && self.dive_timer <= 0.0 && !self.holding_ball && self.kick.is_none() && !self.airborne()
    }

    /// The extended leg of a slide while it is still moving (world capsule).
    pub fn slide_leg(&self) -> Option<Capsule> {
        let s = self.slide?;
        if s.stopped.is_some() || s.t < 0.06 {
            return None;
        }
        let side = s.dir.cross(Vec3::Y) * (s.leg * 0.1);
        Some(Capsule {
            a: self.pos + side + s.dir * 0.15 + v3(0.0, 0.14, 0.0),
            b: self.pos + side + s.dir * 0.95 + v3(0.0, 0.1, 0.0),
            radius: 0.09,
            part: FramePart::Support,
        })
    }

    /// Starts a requested jump and advances the current one. Returns true while
    /// airborne.
    fn step_jump(&mut self, spec: &PlayerSpec, dt: f32) -> bool {
        if let Some(h) = self.jump_request.take() {
            let can = self.jump.is_none() && self.air <= 0.0 && self.can_act();
            if can {
                let run = clamp(self.speed() / 4.0, 0.0, 1.0);
                let max_h = lerp(spec.jump_standing, spec.jump_running, run);
                let h = clamp(h, 0.1, max_h);
                self.jump = Some(Jump {
                    t: 0.0,
                    takeoff: lerp(spec.takeoff_standing, spec.takeoff_running, run),
                    launch: (2.0 * GRAVITY * h).sqrt(),
                    airborne: false,
                    landed: None,
                });
            }
        }
        let Some(mut j) = self.jump else {
            return false;
        };
        j.t += dt;
        if let Some(l) = j.landed.as_mut() {
            *l += dt;
            self.jump = if *l >= spec.landing { None } else { Some(j) };
            return false;
        }
        if !j.airborne {
            if j.t >= j.takeoff {
                j.airborne = true;
                self.air_vel = j.launch;
            } else {
                self.jump = Some(j);
                return false;
            }
        }
        self.air_vel -= GRAVITY * dt;
        self.air += self.air_vel * dt;
        if self.air <= 0.0 {
            self.air = 0.0;
            self.air_vel = 0.0;
            j.landed = Some(0.0);
            self.jump = Some(j);
            return false;
        }
        self.jump = Some(j);
        true
    }

    /// Centre of the head (world).
    pub fn head(&self, spec: &PlayerSpec) -> Vec3 {
        self.pos + v3(0.0, self.air + spec.head_height, 0.0) + self.forward() * 0.04
    }

    /// Off the ground (or about to be): can't play the ball with the feet.
    pub fn airborne(&self) -> bool {
        self.air > 0.08
    }

    pub fn preferred_foot(&self) -> crate::kick::Foot {
        if self.left_footed {
            crate::kick::Foot::Left
        } else {
            crate::kick::Foot::Right
        }
    }

    /// Waiting to kick or kicking.
    pub fn busy_kicking(&self) -> bool {
        self.pending_kick.is_some() || self.kick.is_some()
    }

    /// Counts down all per-player timers.
    pub fn integrate_timers(&mut self, dt: f32) {
        self.stumble = (self.stumble - dt).max(0.0);
        self.ball_ghost = (self.ball_ghost - dt).max(0.0);
        self.touch_cooldown = (self.touch_cooldown - dt).max(0.0);
        self.tackle_cooldown = (self.tackle_cooldown - dt).max(0.0);
        self.save_cooldown = (self.save_cooldown - dt).max(0.0);
        if self.holding_ball {
            self.hold_timer += dt;
        }
        // Get up after a dive.
        if self.stumble <= 0.0 && self.down > 0.0 {
            self.down = (self.down - dt * 3.0).max(0.0);
        }
        self.since_kick += dt;
        self.since_tackle += dt;
        self.since_touch += dt;
        self.since_header += dt;
        if let Some(h) = self.header.as_mut() {
            h.ttl -= dt;
            if h.ttl <= 0.0 {
                self.header = None;
            }
        }
    }
}

/// Pushes overlapping players apart (equal masses) and removes closing speed.
pub fn resolve_player_collisions(players: &mut [Player], spec: &PlayerSpec) {
    let min_d = 2.0 * spec.radius;
    for i in 0..players.len() {
        for j in (i + 1)..players.len() {
            let d = (players[j].pos - players[i].pos).flat();
            let dist = d.length();
            if dist >= min_d {
                continue;
            }
            let n = if dist > 1e-5 {
                d / dist
            } else {
                Vec3::from_heading(i as f32 * 1.7)
            };
            let push = (min_d - dist) * 0.5;
            players[i].pos -= n * push;
            players[j].pos += n * push;
            let rel = (players[j].vel - players[i].vel).dot(n);
            if rel < 0.0 {
                let imp = n * (rel * 0.5);
                players[i].vel += imp;
                players[j].vel -= imp;
            }
        }
    }
}

/// Keeps a player inside the boards and out of the goal frames.
pub fn resolve_arena(p: &mut Player, spec: &PlayerSpec, pitch: &Pitch) {
    let r = spec.radius;
    let (bx, bz) = (pitch.board_x() - r, pitch.board_z() - r);
    if p.pos.x.abs() > bx {
        p.pos.x = p.pos.x.signum() * bx;
        p.vel.x = 0.0;
    }
    if p.pos.z.abs() > bz {
        p.pos.z = p.pos.z.signum() * bz;
        p.vel.z = 0.0;
    }
    // Goals are solid boxes for players (mouth included, so nobody walks into the net).
    for side in Side::BOTH {
        let f = pitch.frame(side);
        let s = side.sign();
        let x_in = s * p.pos.x - pitch.half_length(); // depth behind the goal line
        let x_min = -pitch.goal.post_radius - r;
        let x_max = f.depth_bottom + r;
        let z_max = f.post_z + pitch.goal.post_radius + r;
        if x_in > x_min && x_in < x_max && p.pos.z.abs() < z_max {
            let pen_front = x_in - x_min;
            let pen_back = x_max - x_in;
            let pen_side = z_max - p.pos.z.abs();
            if pen_front <= pen_back && pen_front <= pen_side {
                p.pos.x -= s * pen_front;
                if s * p.vel.x > 0.0 {
                    p.vel.x = 0.0;
                }
            } else if pen_back <= pen_side {
                p.pos.x += s * pen_back;
                if s * p.vel.x < 0.0 {
                    p.vel.x = 0.0;
                }
            } else {
                p.pos.z += p.pos.z.signum() * pen_side;
                p.vel.z = 0.0;
            }
        }
    }
}

/// Ball against all bodies. Returns the id of the player whose body was hit.
pub fn ball_body_collisions(
    ball: &mut Ball,
    bs: &BallSpec,
    players: &mut [Player],
    spec: &PlayerSpec,
) -> Option<(usize, f32)> {
    let mut best: Option<(usize, f32)> = None;
    for p in players.iter_mut() {
        if p.ball_ghost > 0.0 {
            continue;
        }
        let cap = p.body(spec);
        let speed = contact::capsule_moving(ball, bs, &cap, &spec.body_surface, p.vel);
        if speed > 0.0 && best.map(|b| speed > b.1).unwrap_or(true) {
            best = Some((p.id, speed));
        }
    }
    best
}

/// A player ready to head the ball (see [`Player::header`]) meets it with the
/// head: the ball leaves towards the aim with a header's pace. Unaimed balls
/// are left to the body collision (they just bounce off).
pub fn ball_header(ball: &mut Ball, bs: &BallSpec, players: &mut [Player], spec: &PlayerSpec, rng: &mut Rng) -> Option<usize> {
    for p in players.iter_mut() {
        let Some(aim) = p.header else {
            continue;
        };
        if p.ball_ghost > 0.0 || p.stumble > 0.0 || p.on_ground() {
            continue;
        }
        let head = p.head(spec);
        // Head plus a stretch of the neck and upper body towards the ball.
        let reach = bs.radius + 0.2;
        let (q, _) = closest_point_on_segment(head, ball.prev_pos, ball.pos);
        if q.distance(head) > reach {
            continue;
        }
        let incoming = ball.vel.length();
        let speed = clamp(5.0 + 10.0 * aim.power + 0.3 * incoming, 5.0, 19.0);
        let target = aim.target.unwrap_or(head + aim.dir * (6.0 + 14.0 * aim.power));
        let to = target - q;
        let t_f = to.flat_length().max(1.0) / speed;
        // Rise to make up for the drop over the flight (drag ignored).
        let vy = clamp((to.y + 0.5 * GRAVITY * t_f * t_f) / t_f, -0.5 * speed, 0.8 * speed);
        let dir = to.flat().normalize_or(aim.dir);
        // Error: powerful headers and fast balls are harder to place.
        let sigma = (0.05 + 0.04 * aim.power + 0.004 * incoming) * (1.45 - p.skill).max(0.2);
        let flat = dir.rotate_y(rng.gaussian() * sigma) * (speed * speed - vy * vy).max(1.0).sqrt();
        let vel = v3(flat.x, vy + rng.gaussian() * sigma * speed * 0.6, flat.z);
        ball.pos = head + (q - head).normalize_or(dir) * reach;
        ball.vel = vel;
        ball.spin = dir.cross(Vec3::Y) * 6.0; // a little backspin off the forehead
        ball.grounded = false;
        p.header = None;
        p.ball_ghost = 0.18;
        p.since_header = 0.0;
        return Some(p.id);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PHYSICS_DT;

    #[test]
    fn acceleration_is_realistic() {
        let spec = PlayerSpec::default();
        let mut p = Player::new(0, 0, Role::Pivot, 9, Vec3::ZERO);
        p.desired = v3(10.0, 0.0, 0.0);
        let mut t = 0.0;
        while p.speed() < 0.9 * spec.jog_speed {
            p.integrate(&spec, false, PHYSICS_DT);
            t += PHYSICS_DT;
        }
        assert!(t > 0.7 && t < 1.8, "reached 90% jog in {t}");
        p.sprint = true;
        for _ in 0..(240 * 4) {
            p.integrate(&spec, false, PHYSICS_DT);
        }
        assert!((p.speed() - spec.sprint_speed).abs() < 0.05);
        // braking from sprint stops within ~1 s
        p.desired = Vec3::ZERO;
        let mut t = 0.0;
        while p.speed() > 0.01 {
            p.integrate(&spec, false, PHYSICS_DT);
            t += PHYSICS_DT;
        }
        assert!(t < 1.0, "braking took {t}");
    }

    #[test]
    fn jumps_reach_realistic_heights_and_land() {
        let spec = PlayerSpec::default();
        for (speed, want) in [(0.0f32, spec.jump_standing), (5.0, spec.jump_running)] {
            let mut p = Player::new(0, 0, Role::Pivot, 9, Vec3::ZERO);
            p.vel = v3(speed, 0.0, 0.0);
            p.desired = p.vel;
            p.jump_request = Some(2.0);
            let (mut peak, mut t, mut left_ground, mut landed) = (0.0f32, 0.0f32, None, None);
            while t < 2.0 {
                p.integrate(&spec, false, PHYSICS_DT);
                t += PHYSICS_DT;
                peak = peak.max(p.air);
                if p.air > 0.0 && left_ground.is_none() {
                    left_ground = Some(t);
                }
                if left_ground.is_some() && landed.is_none() && p.air == 0.0 {
                    landed = Some(t);
                }
            }
            assert!((peak - want).abs() < 0.03, "speed {speed}: peak {peak} vs {want}");
            let takeoff = left_ground.unwrap();
            let flight = landed.unwrap() - takeoff;
            let expected = 2.0 * (2.0 * want / GRAVITY).sqrt();
            assert!((flight - expected).abs() < 0.03, "flight {flight} vs {expected}");
            assert!(takeoff > 0.08 && takeoff < 0.25, "take-off {takeoff}");
            assert!(p.jump.is_none(), "landing should finish");
            // momentum carried through the air
            if speed > 0.0 {
                assert!(p.pos.x > speed * (takeoff + flight) * 0.8);
            }
        }
    }

    #[test]
    fn body_capsule_rises_with_the_jump() {
        let spec = PlayerSpec::default();
        let mut p = Player::new(0, 0, Role::Pivot, 9, Vec3::ZERO);
        let top0 = p.body(&spec).b.y;
        p.air = 0.5;
        assert!((p.body(&spec).b.y - top0 - 0.5).abs() < 1e-5);
        assert!((p.head(&spec).y - spec.head_height - 0.5).abs() < 1e-5);
        assert!(p.airborne());
    }

    #[test]
    fn facing_follows_movement() {
        let spec = PlayerSpec::default();
        let mut p = Player::new(0, 0, Role::Pivot, 9, Vec3::ZERO);
        p.desired = v3(0.0, 0.0, 5.0);
        for _ in 0..240 {
            p.integrate(&spec, false, PHYSICS_DT);
        }
        assert!((p.forward().z - 1.0).abs() < 0.01);
    }

    #[test]
    fn strafing_backwards_keeps_facing_and_is_slower() {
        let spec = PlayerSpec::default();
        let mut p = Player::new(0, 0, Role::Defender, 4, Vec3::ZERO);
        p.facing = 0.0; // facing +x
        p.strafe = true;
        p.look_at = Some(v3(10.0, 0.0, 0.0));
        p.desired = v3(-8.0, 0.0, 0.0); // run backwards
        p.sprint = true;
        for _ in 0..(240 * 3) {
            p.integrate(&spec, false, PHYSICS_DT);
            p.look_at = Some(p.pos + v3(10.0, 0.0, 0.0));
        }
        assert!(p.forward().x > 0.99, "still facing the ball");
        assert!(p.vel.x < -3.5 && p.vel.x > -spec.back_speed - 0.01, "backwards speed {}", p.vel.x);
        // sideways shuffle
        p.desired = v3(0.0, 0.0, 8.0);
        for _ in 0..(240 * 3) {
            p.integrate(&spec, false, PHYSICS_DT);
            p.look_at = Some(p.pos + v3(10.0, 0.0, 0.0));
        }
        assert!((p.vel.z - spec.side_speed).abs() < 0.05, "side speed {}", p.vel.z);
    }

    #[test]
    fn walking_is_slow_and_turning_takes_time() {
        let spec = PlayerSpec::default();
        let mut p = Player::new(0, 0, Role::Pivot, 9, Vec3::ZERO);
        p.walk = true;
        p.desired = v3(5.0, 0.0, 0.0);
        for _ in 0..(240 * 2) {
            p.integrate(&spec, false, PHYSICS_DT);
        }
        assert!((p.speed() - spec.walk_speed).abs() < 0.02, "walk speed {}", p.speed());
        // turn on the spot by 180°: takes a few tenths of a second
        let mut q = Player::new(1, 0, Role::Pivot, 9, Vec3::ZERO);
        q.look_at = Some(v3(-5.0, 0.0, 0.0));
        let mut t = 0.0;
        while q.forward().x > -0.99 {
            q.integrate(&spec, false, PHYSICS_DT);
            t += PHYSICS_DT;
        }
        assert!(t > 0.25 && t < 0.6, "180° turn took {t}");
    }

    #[test]
    fn players_never_overlap() {
        let spec = PlayerSpec::default();
        let mut ps = vec![
            Player::new(0, 0, Role::Pivot, 9, v3(-3.0, 0.0, 0.0)),
            Player::new(1, 1, Role::Defender, 4, v3(3.0, 0.0, 0.05)),
            Player::new(2, 1, Role::Winger, 7, v3(0.0, 0.0, 3.0)),
        ];
        ps[0].desired = v3(6.0, 0.0, 0.0);
        ps[1].desired = v3(-6.0, 0.0, 0.0);
        ps[2].desired = v3(0.0, 0.0, -6.0);
        for _ in 0..(240 * 2) {
            for p in ps.iter_mut() {
                p.integrate(&spec, false, PHYSICS_DT);
            }
            resolve_player_collisions(&mut ps, &spec);
            for i in 0..3 {
                for j in (i + 1)..3 {
                    let d = ps[i].pos.flat_distance(ps[j].pos);
                    assert!(d > 2.0 * spec.radius - 0.02, "overlap {d}");
                }
            }
        }
    }

    #[test]
    fn arena_and_goals_block_players() {
        let spec = PlayerSpec::default();
        let pitch = Pitch::new();
        let mut p = Player::new(0, 0, Role::Pivot, 9, v3(17.0, 0.0, 0.0));
        p.desired = v3(8.0, 0.0, 0.0);
        p.sprint = true;
        for _ in 0..(240 * 3) {
            p.integrate(&spec, false, PHYSICS_DT);
            resolve_arena(&mut p, &spec, &pitch);
        }
        assert!(
            p.pos.x < 20.0 - spec.radius + 0.01,
            "walked into the goal: {:?}",
            p.pos
        );
        let mut q = Player::new(1, 0, Role::Pivot, 9, v3(0.0, 0.0, 8.0));
        q.desired = v3(0.0, 0.0, 8.0);
        for _ in 0..(240 * 3) {
            q.integrate(&spec, false, PHYSICS_DT);
            resolve_arena(&mut q, &spec, &pitch);
        }
        assert!(q.pos.z <= pitch.board_z() - spec.radius + 1e-4);
    }

    #[test]
    fn ball_rebounds_off_a_body() {
        let spec = PlayerSpec::default();
        let bs = BallSpec::default();
        let mut ps = vec![Player::new(0, 0, Role::Defender, 4, v3(5.0, 0.0, 0.0))];
        let mut b = Ball::new(v3(0.0, 1.0, 0.0));
        b.vel = v3(20.0, 0.0, 0.0);
        let mut hit = None;
        for _ in 0..120 {
            b.integrate_flight(&bs, PHYSICS_DT);
            if let Some(h) = ball_body_collisions(&mut b, &bs, &mut ps, &spec) {
                hit = Some(h);
                break;
            }
        }
        let (id, speed) = hit.expect("ball should hit the body");
        assert_eq!(id, 0);
        assert!(speed > 15.0);
        assert!(b.vel.x < 0.0 && b.vel.x > -10.0, "rebound {:?}", b.vel);
    }
}
