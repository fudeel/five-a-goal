//! Ball control: dribbling as real touches, first touch / receiving, and
//! possession tracking. Nothing glues the ball to a foot — a touch is a small
//! kick, after which the ball rolls freely under the normal physics.

use crate::ball::{Ball, BallSpec};
use crate::math::{clamp, v3, Rng, Vec3};
use crate::player::{Player, PlayerSpec};

/// Tunables for ball control.
#[derive(Clone, Debug)]
pub struct ControlSpec {
    /// Max ground distance from the feet at which the ball can be played.
    pub reach: f32,
    /// Max height of the bottom of the ball for a foot touch.
    pub foot_height: f32,
    /// Max height of the bottom of the ball for a thigh/chest trap.
    pub trap_height: f32,
    /// Seconds between dribble touches at jog / sprint speed.
    pub touch_interval_jog: f32,
    pub touch_interval_sprint: f32,
    /// How much faster than the player the ball is pushed (m/s), jog / sprint.
    pub lead_jog: f32,
    pub lead_sprint: f32,
    /// Distance in front of the feet where the ball is kept.
    pub carry_distance: f32,
    /// Above this relative speed a ball cannot be controlled at all (m/s).
    pub max_control_speed: f32,
    /// If the carrier is this close to the ball, others must tackle to win it.
    pub shield_distance: f32,
}

impl Default for ControlSpec {
    fn default() -> Self {
        Self {
            reach: 0.72,
            foot_height: 0.4,
            trap_height: 1.05,
            touch_interval_jog: 0.3,
            touch_interval_sprint: 0.22,
            lead_jog: 1.1,
            lead_sprint: 1.7,
            carry_distance: 0.42,
            max_control_speed: 24.0,
            shield_distance: 0.85,
        }
    }
}

/// Who has the ball.
#[derive(Clone, Debug, Default)]
pub struct Possession {
    /// Player currently controlling the ball (close to it, last to touch it).
    pub owner: Option<usize>,
    /// Last player to touch the ball (kick, touch or body).
    pub last_touch: Option<usize>,
    /// Simulation time of the last touch.
    pub last_touch_time: f64,
}

impl Possession {
    pub fn touched(&mut self, id: usize, time: f64) {
        self.last_touch = Some(id);
        self.last_touch_time = time;
    }

    /// Updates `owner` from the last toucher's distance to the ball.
    pub fn update_owner(&mut self, players: &[Player], ball: &Ball, bs: &BallSpec) {
        if let Some(h) = players.iter().find(|p| p.holding_ball) {
            self.owner = Some(h.id);
            self.last_touch = Some(h.id);
            return;
        }
        self.owner = self.last_touch.filter(|&id| {
            let p = &players[id];
            let d = p.pos.flat_distance(ball.pos);
            let rel = (ball.vel - p.vel).flat_length();
            d < 1.6 && rel < 5.0 && ball.height(bs) < 0.8
        });
    }

    /// Team of the owner, if any.
    pub fn owner_team(&self, players: &[Player]) -> Option<usize> {
        self.owner.map(|id| players[id].team)
    }
}

/// What a touch did (for events).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TouchKind {
    Dribble,
    Trap,
}

/// Can `p` play the ball right now (distance, height, angle)?
pub fn in_reach(
    p: &Player,
    ball: &Ball,
    bs: &BallSpec,
    cs: &ControlSpec,
    height_limit: f32,
) -> bool {
    let to_ball = (ball.pos - p.pos).flat();
    let d = to_ball.length();
    if d > cs.reach || ball.height(bs) > height_limit {
        return false;
    }
    // Behind the player only if very close.
    d < 0.4 || to_ball.normalize_or_zero().dot(p.forward()) > -0.3
}

/// Velocity a dribbling player wants the ball to have.
fn dribble_velocity(p: &Player, spec: &PlayerSpec, cs: &ControlSpec, ball: &Ball) -> Vec3 {
    let want = p.desired.flat();
    let move_speed = want.length().min(p.max_speed(spec, true));
    if move_speed < 0.4 {
        // Standing: settle the ball just in front of the feet.
        let spot = p.pos + p.forward() * cs.carry_distance;
        return (spot - ball.pos).flat() * 2.5;
    }
    let dir = want / want.length();
    // Push the ball a little faster than the player is running *now*, so he
    // catches up with it for the next touch.
    let along = p.vel.dot(dir).max(0.0).min(move_speed);
    let k = clamp(
        (along - spec.jog_speed * 0.6) / (spec.sprint_speed - spec.jog_speed * 0.6),
        0.0,
        1.0,
    );
    let lead = cs.lead_jog + (cs.lead_sprint - cs.lead_jog) * k;
    // Steer the ball towards the running line in front of where the player
    // will be a moment from now (keeps momentum through turns).
    let future = p.pos + p.vel.flat() * 0.25;
    let spot = future + dir * cs.carry_distance;
    let lateral = ((spot - ball.pos).flat().reject(dir) * (2.0 / 3.0)).clamp_length(2.2);
    // Never push it faster than the player can follow.
    let ball_speed = (along + lead).min(p.max_speed(spec, true) + 0.5);
    dir * ball_speed + lateral * 3.0
}

/// Movement for a ball carrier who is not yet in reach of his ball: run onto
/// it, arriving from behind relative to where he wants to go next.
pub fn carrier_movement(p: &Player, ball: &Ball, bs: &BallSpec, cs: &ControlSpec) -> Option<Vec3> {
    let want = p.desired.flat();
    if want.length() < 0.3 || in_reach(p, ball, bs, cs, cs.foot_height) {
        return None;
    }
    let dir = want.normalize_or_zero();
    let lead_time = 0.15;
    let ball_next = ball.pos + ball.vel.flat() * lead_time;
    let spot = ball_next - dir * 0.3;
    let to_spot = (spot - p.pos).flat();
    if to_spot.length() > 3.0 {
        return None;
    }
    let speed = want.length().max(ball.vel.flat_length() + 1.0);
    Some(to_spot.normalize_or(dir) * speed)
}

/// Does the player need to touch the ball now to keep it?
fn needs_touch(p: &Player, ball: &Ball, target: Vec3) -> bool {
    let rel = ball.vel.flat() - target;
    let ahead = (ball.pos - p.pos)
        .flat()
        .dot(p.desired.flat().normalize_or(p.forward()));
    rel.length() > 0.9 || ahead < 0.38
}

/// Plays a touch: sets the ball's velocity (with an error that grows with speed,
/// incoming pace and height). Returns the touch kind.
#[allow(clippy::too_many_arguments)]
pub fn play_touch(
    p: &mut Player,
    spec: &PlayerSpec,
    cs: &ControlSpec,
    ball: &mut Ball,
    bs: &BallSpec,
    rng: &mut Rng,
    skill: f32,
) -> TouchKind {
    let incoming = (ball.vel - p.vel).length();
    let height = ball.height(bs);
    let kind = if incoming > 4.5 || height > cs.foot_height {
        TouchKind::Trap
    } else {
        TouchKind::Dribble
    };
    let mut target = dribble_velocity(p, spec, cs, ball);
    // Error: heavier for fast incoming balls, high balls and sprinting.
    let difficulty = 0.03 * incoming + 0.6 * height + 0.04 * p.speed();
    let err = difficulty * (1.4 - skill).max(0.2);
    let dir_err = rng.gaussian() * err * 0.35;
    let speed_err = 1.0 + rng.gaussian() * err * 0.25;
    target = target.rotate_y(dir_err) * speed_err.max(0.3);
    ball.vel = v3(
        target.x,
        if height > 0.05 {
            ball.vel.y.min(0.8) * 0.25
        } else {
            0.0
        },
        target.z,
    );
    // Rolling spin consistent with the new velocity, a touch of side spin.
    ball.spin = v3(
        target.z / bs.radius,
        rng.gaussian() * 2.0,
        -target.x / bs.radius,
    );
    let speed = p.speed();
    let interval = if p.sprint && speed > spec.jog_speed {
        cs.touch_interval_sprint
    } else {
        cs.touch_interval_jog
    };
    p.touch_cooldown = interval;
    p.ball_ghost = 0.12;
    kind
}

/// Chooses which player (if any) touches the ball this step and plays it.
/// Returns (player id, kind).
#[allow(clippy::too_many_arguments)]
pub fn step_control(
    players: &mut [Player],
    spec: &PlayerSpec,
    cs: &ControlSpec,
    ball: &mut Ball,
    bs: &BallSpec,
    poss: &Possession,
    rng: &mut Rng,
) -> Option<(usize, TouchKind)> {
    if players.iter().any(|p| p.holding_ball) {
        return None; // in the keeper's hands
    }
    let rel_speed_ok = |p: &Player, b: &Ball| (b.vel - p.vel).length() < cs.max_control_speed;
    // The carrier keeps priority while close to the ball.
    let shielded_by = poss
        .owner
        .filter(|&o| players[o].pos.flat_distance(ball.pos) < cs.shield_distance);
    let mut best: Option<(usize, f32)> = None;
    for p in players.iter() {
        if !p.dribble || p.touch_cooldown > 0.0 || p.stumble > 0.0 || p.holding_ball {
            continue;
        }
        if let Some(o) = shielded_by {
            if o != p.id {
                continue; // carrier is protecting the ball (opponents must tackle)
            }
        }
        if !in_reach(p, ball, bs, cs, cs.trap_height) || !rel_speed_ok(p, ball) {
            continue;
        }
        let d = p.pos.flat_distance(ball.pos);
        if best.map(|b| d < b.1).unwrap_or(true) {
            best = Some((p.id, d));
        }
    }
    let (id, _) = best?;
    let p = &mut players[id];
    let target = dribble_velocity(p, spec, cs, ball);
    let fresh = poss.last_touch != Some(id) || poss.owner != Some(id);
    if !fresh && !needs_touch(p, ball, target) {
        return None;
    }
    let skill = p.skill;
    let kind = play_touch(p, spec, cs, ball, bs, rng, skill);
    Some((id, kind))
}

/// Tuning for standing tackles.
#[derive(Clone, Debug)]
pub struct TackleSpec {
    /// Max distance from the tackler's feet to the ball.
    pub reach: f32,
    /// Base success chance.
    pub base_chance: f32,
    /// Seconds before the same player can tackle again.
    pub cooldown: f32,
    /// Off-balance time after a failed tackle / a whiff.
    pub fail_stumble: f32,
    pub whiff_stumble: f32,
}

impl Default for TackleSpec {
    fn default() -> Self {
        Self {
            reach: 1.15,
            base_chance: 0.5,
            cooldown: 0.8,
            fail_stumble: 0.6,
            whiff_stumble: 0.25,
        }
    }
}

/// Outcome of a tackle attempt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TackleResult {
    Won,
    Failed,
    Whiff,
}

/// Resolves a standing tackle by `tackler` on whoever has the ball.
pub fn resolve_tackle(
    tackler: usize,
    players: &mut [Player],
    ball: &mut Ball,
    bs: &BallSpec,
    poss: &Possession,
    ts: &TackleSpec,
    rng: &mut Rng,
) -> TackleResult {
    let t = &players[tackler];
    let to_ball = (ball.pos - t.pos).flat();
    let d = to_ball.length();
    let in_front = d < 0.45 || to_ball.normalize_or_zero().dot(t.forward()) > 0.2;
    if d > ts.reach || !in_front || ball.height(bs) > 0.6 {
        players[tackler].stumble = ts.whiff_stumble;
        return TackleResult::Whiff;
    }
    let mut chance = ts.base_chance + 0.3 * (t.skill - 0.5);
    let carrier = poss
        .owner
        .filter(|&o| players[o].team != players[tackler].team);
    if let Some(c) = carrier {
        let cp = &players[c];
        // An exposed ball (pushed away from the carrier) is easy to win.
        let exposure = cp.pos.flat_distance(ball.pos);
        chance += clamp((exposure - 0.35) * 0.8, -0.1, 0.4);
        // Tackling from behind the carrier rarely works.
        let from_behind = (t.pos - cp.pos)
            .flat()
            .normalize_or_zero()
            .dot(cp.forward())
            < -0.3;
        if from_behind {
            chance -= 0.25;
        }
        // Fast dribbles are harder to stop cleanly.
        chance -= 0.03 * cp.speed();
    } else {
        chance = 0.95; // loose ball
    }
    if rng.chance(clamp(chance, 0.05, 0.95)) {
        // Poke the ball away in the tackler's facing direction.
        let dir = players[tackler].forward().rotate_y(rng.gaussian() * 0.4);
        let speed = rng.range(3.0, 6.0);
        ball.vel = dir * speed + v3(0.0, rng.range(0.0, 0.8), 0.0);
        ball.spin = Vec3::Y.cross(dir) * (speed / bs.radius);
        ball.grounded = false;
        if let Some(c) = carrier {
            players[c].touch_cooldown = 0.45;
            players[c].stumble = 0.2;
        }
        players[tackler].ball_ghost = 0.12;
        players[tackler].touch_cooldown = 0.15;
        TackleResult::Won
    } else {
        players[tackler].stumble = ts.fail_stumble;
        TackleResult::Failed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::player::Role;
    use crate::world::World;
    use crate::PHYSICS_DT;

    fn dribbler() -> World {
        let mut w = World::new();
        let id = w.spawn_player(0, Role::Pivot, 9, v3(-15.0, 0.0, 0.0));
        w.players[id].dribble = true;
        w.ball.place(&w.ball_spec.clone(), -14.6, 0.0);
        w
    }

    #[test]
    fn dribbles_in_a_straight_line_and_through_a_turn() {
        let mut w = dribbler();
        let mut max_gap = 0.0f32;
        for i in 0..(240 * 6) {
            let t = i as f32 * PHYSICS_DT;
            w.players[0].desired = if t < 2.5 {
                v3(6.0, 0.0, 0.0)
            } else {
                v3(0.0, 0.0, 6.0)
            };
            w.players[0].sprint = t > 1.0 && t < 2.0;
            w.step(PHYSICS_DT);
            let gap = w.players[0].pos.flat_distance(w.ball.pos);
            max_gap = max_gap.max(gap);
        }
        assert!(max_gap < 2.2, "lost the ball: gap {max_gap}");
        assert!(
            w.players[0].pos.x > -3.0,
            "should have advanced: {:?}",
            w.players[0].pos
        );
        assert!(
            w.players[0].pos.z > 3.0,
            "should have turned: {:?}",
            w.players[0].pos
        );
        assert_eq!(w.possession.owner, Some(0));
    }

    #[test]
    fn standing_player_traps_an_incoming_pass() {
        let mut w = World::new();
        let id = w.spawn_player(0, Role::Pivot, 9, v3(0.0, 0.0, 0.0));
        w.players[id].dribble = true;
        w.players[id].facing = std::f32::consts::PI; // facing -x, towards the ball
        w.ball.place(&w.ball_spec.clone(), -10.0, 0.0);
        w.ball.vel = v3(11.0, 0.0, 0.0);
        for _ in 0..(240 * 2) {
            w.step(PHYSICS_DT);
        }
        assert!(
            w.ball.vel.length() < 1.0,
            "ball not controlled: {:?}",
            w.ball.vel
        );
        assert!(w.players[0].pos.flat_distance(w.ball.pos) < 1.0);
        assert_eq!(w.possession.owner, Some(0));
    }

    #[test]
    fn tackles_win_exposed_balls_more_often() {
        use crate::control::TackleSpec;
        let bs = BallSpec::default();
        let ts = TackleSpec::default();
        let rate = |exposure: f32| {
            let mut rng = Rng::new(11);
            let mut won = 0;
            for _ in 0..400 {
                let mut ps = vec![
                    Player::new(0, 0, Role::Pivot, 9, v3(0.0, 0.0, 0.0)),
                    Player::new(1, 1, Role::Defender, 4, v3(exposure + 0.9, 0.0, 0.0)),
                ];
                ps[0].facing = 0.0;
                ps[1].facing = std::f32::consts::PI;
                let mut ball = Ball::new(v3(exposure, bs.radius, 0.0));
                let poss = Possession {
                    owner: Some(0),
                    last_touch: Some(0),
                    last_touch_time: 0.0,
                };
                if resolve_tackle(1, &mut ps, &mut ball, &bs, &poss, &ts, &mut rng)
                    == TackleResult::Won
                {
                    won += 1;
                }
            }
            won as f32 / 400.0
        };
        let tight = rate(0.35);
        let loose = rate(0.8);
        assert!(loose > tight + 0.15, "tight {tight} loose {loose}");
        assert!(tight > 0.2 && tight < 0.7);
    }

    #[test]
    fn opponent_cannot_take_a_shielded_ball_without_tackling() {
        let mut w = dribbler();
        let def = w.spawn_player(1, Role::Defender, 4, v3(-14.0, 0.0, 0.6));
        w.players[def].dribble = true;
        w.possession.touched(0, 0.0);
        for _ in 0..240 {
            w.players[0].desired = Vec3::ZERO;
            w.step(PHYSICS_DT);
            assert_ne!(w.possession.last_touch, Some(def));
        }
    }
}
