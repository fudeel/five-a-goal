//! Footballers: movement physics, collisions with each other and the arena, and
//! the ball bouncing off their bodies.

use crate::ball::{Ball, BallSpec};
use crate::contact::{self, Surface};
use crate::math::{clamp, lerp, rotate_towards, v3, Vec3};
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
    pub jog_speed: f32,
    pub sprint_speed: f32,
    /// Speed factor while in possession (dribbling).
    pub with_ball_factor: f32,
    /// Turn rate (rad/s) when standing / at sprint speed.
    pub turn_slow: f32,
    pub turn_fast: f32,
    /// Ball bounce off the body.
    pub body_surface: Surface,
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
            jog_speed: 5.4,
            sprint_speed: 7.5,
            with_ball_factor: 0.92,
            turn_slow: 14.0,
            turn_fast: 4.0,
            body_surface: Surface {
                restitution: 0.35,
                restitution_drop: 0.005,
                restitution_min: 0.2,
                friction: 0.35,
            },
        }
    }
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
    /// Disables the automatic run onto the ball (e.g. the human chose to leave it).
    pub human_free_run: bool,
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
        let mut s = if self.sprint {
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

    /// Body capsule for ball collisions.
    pub fn body(&self, spec: &PlayerSpec) -> Capsule {
        Capsule {
            a: self.pos + v3(0.0, 0.25, 0.0),
            b: self.pos + v3(0.0, spec.height - 0.15, 0.0),
            radius: spec.body_radius,
            part: FramePart::Support,
        }
    }

    /// Integrates movement towards `desired` with realistic acceleration limits.
    pub fn integrate(&mut self, spec: &PlayerSpec, with_ball: bool, dt: f32) {
        let vmax = self.max_speed(spec, with_ball);
        let target = self.desired.flat().clamp_length(vmax);
        let speed = self.speed();
        let dv = target - self.vel;
        // Braking is stronger than accelerating; acceleration fades near top speed.
        let braking = target.length() < speed - 0.05 || target.dot(self.vel) < 0.0;
        let frac = clamp(speed / spec.sprint_speed, 0.0, 1.0);
        let accel = if braking {
            spec.decel
        } else {
            lerp(spec.accel_slow, spec.accel_fast, frac)
        };
        self.vel += dv.clamp_length(accel * dt);
        self.vel.y = 0.0;
        self.pos += self.vel * dt;
        self.pos.y = 0.0;

        // Facing: follow the direction of travel, turning slower at speed.
        let turn = lerp(spec.turn_slow, spec.turn_fast, frac);
        let want = if speed > 0.6 {
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
            self.facing = rotate_towards(self.facing, h, turn * dt);
        }

        self.stride += speed * dt;
        self.integrate_timers(dt);
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
