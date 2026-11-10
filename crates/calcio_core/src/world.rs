//! The simulation world. Grows step by step (ball, nets, players, match, AI).

use crate::ball::{Ball, BallSpec};
use crate::contact::{self, ArenaSurfaces, Turf};
use crate::control::{self, ControlSpec, Possession, TackleResult, TackleSpec, TouchKind};
use crate::events::GameEvent;
use crate::keeper::{self, KeeperSpec, SaveKind};
use crate::kick::{self, KickSpec};
use crate::math::{v3, Rng, Vec3};
use crate::net::{GoalNet, NetSpec};
use crate::pitch::{Capsule, Pitch, Side};
use crate::player::{self, Player, PlayerSpec, Role};

/// Owns the whole simulation state and advances it with fixed timesteps.
#[derive(Debug, Clone)]
pub struct World {
    pub pitch: Pitch,
    pub ball_spec: BallSpec,
    pub turf: Turf,
    pub surfaces: ArenaSurfaces,
    /// Goal frame colliders of both goals.
    pub frame_colliders: Vec<(Side, Capsule)>,
    pub ball: Ball,
    pub net_spec: NetSpec,
    /// Nets of the left and right goal (index = `Side::index()`).
    pub nets: [GoalNet; 2],
    pub player_spec: PlayerSpec,
    pub players: Vec<Player>,
    pub control_spec: ControlSpec,
    pub kick_spec: KickSpec,
    pub tackle_spec: TackleSpec,
    pub keeper_spec: KeeperSpec,
    /// Goal defended by each team (set by the match every step).
    pub defend: [Side; 2],
    pub possession: Possession,
    /// Events produced since the last [`World::drain_events`].
    pub events: Vec<GameEvent>,
    pub rng: Rng,
    /// Simulated time since the world was created (s).
    pub time: f64,
}

/// Was the net on `side` already reported as hit in the pending events?
fn net_touching(side: Side, events: &[GameEvent]) -> bool {
    events
        .iter()
        .rev()
        .take(8)
        .any(|e| matches!(e, GameEvent::NetHit { side: s, .. } if *s == side))
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

impl World {
    pub fn new() -> Self {
        let pitch = Pitch::new();
        let ball_spec = BallSpec::default();
        let mut ball = Ball::new(Vec3::ZERO);
        ball.place(&ball_spec, 0.0, 0.0);
        let frame_colliders = Side::BOTH
            .iter()
            .flat_map(|s| pitch.goal_colliders(*s).into_iter().map(move |c| (*s, c)))
            .collect();
        let net_spec = NetSpec::default();
        let mut nets = [
            GoalNet::new(&pitch, Side::Left, &net_spec),
            GoalNet::new(&pitch, Side::Right, &net_spec),
        ];
        for n in nets.iter_mut() {
            n.settle(&net_spec, crate::PHYSICS_DT, 8.0);
        }
        Self {
            pitch,
            ball_spec,
            turf: Turf::default(),
            surfaces: ArenaSurfaces::default(),
            frame_colliders,
            ball,
            net_spec,
            nets,
            player_spec: PlayerSpec::default(),
            players: Vec::new(),
            control_spec: ControlSpec::default(),
            kick_spec: KickSpec::default(),
            tackle_spec: TackleSpec::default(),
            keeper_spec: KeeperSpec::default(),
            defend: [Side::Left, Side::Right],
            possession: Possession::default(),
            events: Vec::new(),
            rng: Rng::new(0xC0FFEE),
            time: 0.0,
        }
    }

    /// Advances the simulation by one fixed timestep `dt` (s).
    pub fn step(&mut self, dt: f32) {
        self.step_tackles();
        self.step_kicks(dt);
        self.step_control();
        self.step_players(dt);
        self.step_ball(dt);
        self.possession
            .update_owner(&self.players, &self.ball, &self.ball_spec);
        self.time += dt as f64;
    }

    /// Adds a player and returns its id.
    pub fn spawn_player(&mut self, team: usize, role: Role, number: u8, pos: Vec3) -> usize {
        let id = self.players.len();
        self.players.push(Player::new(id, team, role, number, pos));
        id
    }

    /// The human-controlled player, if any.
    pub fn human_player(&self) -> Option<usize> {
        self.players.iter().position(|p| p.human)
    }

    fn step_tackles(&mut self) {
        for i in 0..self.players.len() {
            if !self.players[i].tackle_request {
                continue;
            }
            self.players[i].tackle_request = false;
            let p = &self.players[i];
            if p.tackle_cooldown > 0.0 || p.stumble > 0.0 || self.possession.owner == Some(i) {
                continue;
            }
            let res = control::resolve_tackle(
                i,
                &mut self.players,
                &mut self.ball,
                &self.ball_spec,
                &self.possession,
                &self.tackle_spec,
                &mut self.rng,
            );
            let p = &mut self.players[i];
            p.tackle_cooldown = self.tackle_spec.cooldown;
            p.since_tackle = 0.0;
            if res != TackleResult::Whiff {
                self.events.push(GameEvent::Tackle {
                    player: i,
                    won: res == TackleResult::Won,
                });
            }
            if res == TackleResult::Won {
                self.possession.touched(i, self.time);
                self.possession.owner = None;
            }
        }
    }

    /// Executes pending kicks whose ball is in reach.
    fn step_kicks(&mut self, dt: f32) {
        for i in 0..self.players.len() {
            let Some(mut req) = self.players[i].pending_kick else {
                continue;
            };
            req.age += dt;
            let p = &self.players[i];
            if p.since_kick < 0.35 {
                // Just kicked: the follow-through can't strike the ball again.
                self.players[i].pending_kick = None;
                continue;
            }
            if !kick::can_kick(p, &self.ball, &self.ball_spec, &self.kick_spec) || p.holding_ball {
                self.players[i].pending_kick = if req.age > self.kick_spec.patience {
                    None
                } else {
                    Some(req)
                };
                continue;
            }
            let team = p.team;
            let kicker = p.clone();
            let mates: Vec<Player> = self
                .players
                .iter()
                .filter(|m| m.team == team)
                .cloned()
                .collect();
            let mate_refs: Vec<&Player> = mates.iter().collect();
            let plan = kick::plan_kick(
                &req,
                &kicker,
                &mate_refs,
                &self.ball,
                &self.ball_spec,
                &self.turf,
                &self.kick_spec,
                &mut self.rng,
            );
            self.ball.vel = plan.vel;
            self.ball.spin = plan.spin;
            self.ball.grounded =
                plan.vel.y.abs() < 0.05 && self.ball.height(&self.ball_spec) < 0.01;
            let p = &mut self.players[i];
            p.pending_kick = None;
            p.since_kick = 0.0;
            p.ball_ghost = 0.22;
            p.touch_cooldown = 0.4;
            p.facing = plan.vel.flat().heading();
            self.possession.touched(i, self.time);
            self.possession.owner = None;
            self.events.push(GameEvent::Kick {
                player: i,
                kind: req.kind,
                speed: plan.vel.length(),
            });
            // Only one player can strike the ball in a step.
            for (j, q) in self.players.iter_mut().enumerate() {
                if j != i && q.pending_kick.is_some() {
                    q.pending_kick = None;
                }
            }
            break;
        }
    }

    fn step_control(&mut self) {
        if let Some((id, kind)) = control::step_control(
            &mut self.players,
            &self.player_spec,
            &self.control_spec,
            &mut self.ball,
            &self.ball_spec,
            &self.possession,
            &mut self.rng,
        ) {
            self.possession.touched(id, self.time);
            self.possession.owner = Some(id);
            self.events.push(GameEvent::Touch {
                player: id,
                trap: kind == TouchKind::Trap,
            });
        }
    }

    fn step_players(&mut self, dt: f32) {
        let owner = self.possession.owner;
        let last = self.possession.last_touch;
        for p in self.players.iter_mut() {
            let with_ball = owner == Some(p.id);
            if p.dive_timer > 0.0 {
                keeper::step_dive(p, &self.keeper_spec, dt);
                p.integrate_timers(dt);
                continue;
            }
            let wanted = p.desired;
            if let Some(req) = p.pending_kick {
                // Step to the ball to strike it, approaching from behind the kick line.
                let spot = self.ball.pos + self.ball.vel.flat() * 0.1 - req.dir * 0.45;
                let to = (spot - p.pos).flat();
                if to.length() < 3.0 && to.length() > 0.15 {
                    p.desired = to.normalize_or(req.dir) * (wanted.length().max(3.5));
                }
            } else if (with_ball || last == Some(p.id)) && p.dribble && !p.human_free_run {
                if let Some(m) =
                    control::carrier_movement(p, &self.ball, &self.ball_spec, &self.control_spec)
                {
                    p.desired = m;
                }
            }
            p.integrate(&self.player_spec, with_ball, dt);
            p.desired = wanted;
        }
        player::resolve_player_collisions(&mut self.players, &self.player_spec);
        for p in self.players.iter_mut() {
            player::resolve_arena(p, &self.player_spec, &self.pitch);
        }
    }

    /// Keeper `id` puts the ball down at his feet (to kick or roll it).
    pub fn release_ball(&mut self, id: usize) {
        let r = self.ball_spec.radius;
        let p = &mut self.players[id];
        if !p.holding_ball {
            return;
        }
        p.holding_ball = false;
        p.save_cooldown = 1.0;
        let front = p.pos + p.forward() * 0.45;
        self.ball.pos = v3(front.x, r, front.z);
        self.ball.prev_pos = self.ball.pos;
        self.ball.vel = p.vel.flat();
        self.ball.spin = Vec3::ZERO;
        self.ball.grounded = true;
    }

    fn step_ball(&mut self, dt: f32) {
        // In the keeper's hands: the ball moves with him.
        if let Some(h) = self.players.iter().find(|p| p.holding_ball) {
            self.ball.prev_pos = self.ball.pos;
            self.ball.pos = keeper::hold_point(h);
            self.ball.vel = h.vel;
            self.ball.spin = Vec3::ZERO;
            for net in self.nets.iter_mut() {
                net.step(&self.net_spec, dt);
            }
            return;
        }
        let spec = &self.ball_spec;
        self.ball.integrate_flight(spec, dt);
        for i in 0..self.players.len() {
            if self.players[i].role != Role::Goalkeeper {
                continue;
            }
            let defend = self.defend[self.players[i].team.min(1)];
            if let Some(kind) = keeper::try_save(
                &mut self.players[i],
                &mut self.ball,
                spec,
                &self.keeper_spec,
                &self.pitch,
                defend,
                &mut self.rng,
            ) {
                self.events.push(GameEvent::Save {
                    player: i,
                    caught: kind == SaveKind::Catch,
                });
                self.possession.touched(i, self.time);
                if kind == SaveKind::Catch {
                    self.possession.owner = Some(i);
                    for net in self.nets.iter_mut() {
                        net.step(&self.net_spec, dt);
                    }
                    return;
                }
                break;
            }
        }
        if let Some((side, part, speed)) =
            contact::goal_frames(&mut self.ball, spec, &self.frame_colliders, &self.surfaces)
        {
            if speed > 1.0 {
                self.events.push(GameEvent::FrameHit { side, part, speed });
            }
        }
        if let Some((id, speed)) =
            player::ball_body_collisions(&mut self.ball, spec, &mut self.players, &self.player_spec)
        {
            if speed > 0.5 {
                self.events.push(GameEvent::BodyHit { player: id, speed });
                self.possession.touched(id, self.time);
            }
        }
        if let Some(h) = contact::boards(&mut self.ball, spec, &self.pitch, &self.surfaces) {
            if h.speed > 1.0 {
                self.events.push(GameEvent::BoardHit {
                    speed: h.speed,
                    cage: h.cage,
                });
            }
        }
        let start = self.ball.prev_pos;
        for net in self.nets.iter_mut() {
            if net.ball_is_near(&self.ball, spec, dt) {
                let was_touching = net_touching(net.side, &self.events);
                let hit = net.step_with_ball(&self.net_spec, &mut self.ball, spec, dt);
                if hit.contacts > 0 && !was_touching && hit.impulse_speed > 0.3 {
                    self.events.push(GameEvent::NetHit {
                        side: net.side,
                        speed: self.ball.vel.length() + hit.impulse_speed,
                    });
                }
            } else {
                net.step(&self.net_spec, dt);
            }
        }
        let g = contact::ground(&mut self.ball, spec, &self.turf, dt);
        if let Some(speed) = g.bounce {
            if speed > 1.5 {
                self.events.push(GameEvent::GroundBounce { speed });
            }
        }
        for side in Side::BOTH {
            if self
                .pitch
                .goal_crossing(side, start, self.ball.pos, spec.radius)
            {
                self.events.push(GameEvent::GoalScored { side });
            }
        }
        // Keep the crossing test consistent for the next step.
        self.ball.prev_pos = start;
    }

    /// Fires the ball from `from` towards `target` at `speed` (debug / tests).
    pub fn debug_fire(&mut self, from: Vec3, target: Vec3, speed: f32, spin: Vec3) {
        let dir = (target - from).normalize_or(Vec3::X);
        self.ball.pos = from;
        self.ball.prev_pos = from;
        self.ball.vel = dir * speed;
        self.ball.spin = spin;
        self.ball.grounded = false;
    }

    /// Takes all events produced so far.
    pub fn drain_events(&mut self) -> Vec<GameEvent> {
        std::mem::take(&mut self.events)
    }

    /// Debug helper: fires a random shot at a random goal (posts and bar included).
    pub fn debug_test_shot(&mut self) {
        let side = if self.rng.chance(0.5) {
            Side::Left
        } else {
            Side::Right
        };
        let s = side.sign();
        let hl = self.pitch.half_length();
        let start = v3(
            s * (hl - self.rng.range(7.0, 14.0)),
            self.ball_spec.radius,
            self.rng.range(-7.0, 7.0),
        );
        let gw = self.pitch.goal.width * 0.5 + 0.3;
        let target = v3(
            s * (hl + 0.2),
            self.rng.range(0.2, self.pitch.goal.height + 0.3),
            self.rng.range(-gw, gw),
        );
        let dir = (target - start).normalize_or(Vec3::X);
        let speed = self.rng.range(18.0, 30.0);
        self.ball.pos = start;
        self.ball.prev_pos = start;
        // Aim a little high to compensate for gravity over the distance.
        let dist = start.flat_distance(target);
        let lift = 0.5 * 9.81 * (dist / speed) * 1.1;
        self.ball.vel = dir * speed + v3(0.0, lift, 0.0);
        self.ball.spin = v3(
            self.rng.range(-20.0, 20.0),
            self.rng.range(-50.0, 50.0),
            self.rng.range(-20.0, 20.0),
        );
        self.ball.grounded = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PHYSICS_DT;

    #[test]
    fn shot_into_goal_scores_once_and_bulges_net() {
        let mut w = World::new();
        w.debug_fire(v3(14.0, 0.5, 0.0), v3(20.5, 1.0, 0.3), 26.0, Vec3::ZERO);
        let mut goals = Vec::new();
        let mut net_hits = 0;
        let rest_max_x = w.nets[1].pos.iter().fold(f32::MIN, |m, p| m.max(p.x));
        let mut max_bulge = 0.0f32;
        for _ in 0..(240 * 3) {
            w.step(PHYSICS_DT);
            for e in w.drain_events() {
                match e {
                    GameEvent::GoalScored { side } => goals.push(side),
                    GameEvent::NetHit { .. } => net_hits += 1,
                    _ => {}
                }
            }
            let mx = w.nets[1].pos.iter().fold(f32::MIN, |m, p| m.max(p.x));
            max_bulge = max_bulge.max(mx - rest_max_x);
        }
        assert_eq!(goals, vec![Side::Right]);
        assert!(net_hits >= 1);
        assert!(max_bulge > 0.15, "bulge {max_bulge}");
    }

    #[test]
    fn pass_and_receive_between_teammates() {
        use crate::kick::{KickKind, KickRequest};
        let mut w = World::new();
        let a = w.spawn_player(0, Role::Pivot, 9, v3(-5.0, 0.0, 0.0));
        let b = w.spawn_player(0, Role::Winger, 7, v3(7.0, 0.0, 4.0));
        w.players[b].facing = std::f32::consts::PI;
        w.ball.place(&w.ball_spec.clone(), -4.6, 0.0);
        w.players[a].pending_kick = Some(KickRequest::new(KickKind::Pass, v3(1.0, 0.0, 0.3), 0.4));
        let mut kicked = false;
        for _ in 0..(240 * 4) {
            w.step(PHYSICS_DT);
            for e in w.drain_events() {
                if let GameEvent::Kick { player, .. } = e {
                    assert_eq!(player, a);
                    kicked = true;
                }
            }
        }
        assert!(kicked);
        assert_eq!(
            w.possession.last_touch,
            Some(b),
            "receiver should control the pass"
        );
        assert!(w.players[b].pos.flat_distance(w.ball.pos) < 1.2);
    }

    #[test]
    fn wide_shot_does_not_score() {
        let mut w = World::new();
        w.debug_fire(v3(14.0, 0.5, 0.0), v3(20.5, 1.0, 2.6), 26.0, Vec3::ZERO);
        for _ in 0..(240 * 3) {
            w.step(PHYSICS_DT);
            assert!(!w
                .drain_events()
                .iter()
                .any(|e| matches!(e, GameEvent::GoalScored { .. })));
        }
    }

    #[test]
    fn random_test_shots_are_stable() {
        let mut w = World::new();
        for _ in 0..12 {
            w.debug_test_shot();
            for _ in 0..(240 * 3) {
                w.step(PHYSICS_DT);
                assert!(w.ball.is_finite());
                assert!(w.pitch.inside_arena(w.ball.pos, 0.01));
            }
            assert!(w.nets.iter().all(|n| n.is_finite()));
            w.drain_events();
        }
    }
}
