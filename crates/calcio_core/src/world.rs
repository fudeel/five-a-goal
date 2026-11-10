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
            if p.tackle_cooldown > 0.0 || !p.can_act() || self.possession.owner == Some(i) {
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

    /// Kicks: requests start a wind-up when the ball will be at the foot;
    /// at the contact time the ball is struck if it is still there (otherwise
    /// the foot meets nothing), then the follow-through plays out.
    fn step_kicks(&mut self, dt: f32) {
        let mut struck = false;
        for i in 0..self.players.len() {
            let Some(mut k) = self.players[i].kick else {
                continue;
            };
            k.t += dt;
            if !k.done_contact && k.t >= k.windup {
                k.done_contact = true;
                let p = &self.players[i];
                let ok = if k.req.kind.from_hands() {
                    p.holding_ball
                } else {
                    !p.holding_ball
                        && p.stumble <= 0.0
                        && !self.players.iter().any(|q| q.holding_ball)
                        && kick::ball_at_foot(p, k.foot, &self.ball, &self.ball_spec, &self.kick_spec)
                };
                if ok && !struck {
                    if k.req.kind.from_hands() {
                        self.release_from_hands(i, k.req.kind);
                    }
                    self.strike(i, &k.req, k.foot);
                    struck = true;
                } else {
                    k.missed = true;
                    self.players[i].since_kick = 0.0;
                    self.events.push(GameEvent::Whiff { player: i });
                }
            }
            self.players[i].kick = if k.finished() { None } else { Some(k) };
        }
        for i in 0..self.players.len() {
            let Some(mut req) = self.players[i].pending_kick else {
                continue;
            };
            req.age += dt;
            let p = &self.players[i];
            if p.kick.is_some() || p.since_kick < 0.35 {
                // Already kicking, or just kicked: the follow-through can't strike again.
                self.players[i].pending_kick = None;
                continue;
            }
            let start = if req.kind.from_hands() {
                p.holding_ball.then(|| p.preferred_foot())
            } else if p.holding_ball {
                None
            } else {
                kick::ready_to_strike(p, &req, &self.ball, &self.ball_spec, &self.kick_spec)
            };
            let p = &mut self.players[i];
            match start {
                Some(foot) => {
                    let windup = kick::effective_windup(p, &req);
                    p.kick = Some(kick::KickState::new(req, foot, windup));
                    p.pending_kick = None;
                }
                None => {
                    let keep = req.age <= self.kick_spec.patience && (!req.kind.from_hands() || p.holding_ball);
                    p.pending_kick = keep.then_some(req);
                }
            }
        }
    }

    /// The keeper lets go of the ball at the release point of a throw / roll.
    fn release_from_hands(&mut self, id: usize, kind: kick::KickKind) {
        let p = &mut self.players[id];
        let at = kick::hand_point(p, kind, 1.0);
        p.holding_ball = false;
        p.save_cooldown = 1.0;
        self.ball.pos = v3(at.x, at.y.max(self.ball_spec.radius), at.z);
        self.ball.prev_pos = self.ball.pos;
        self.ball.vel = p.vel.flat();
        self.ball.spin = Vec3::ZERO;
        self.ball.grounded = kind == kick::KickKind::Roll;
    }

    /// Launches the ball for `req`, struck by player `i` with `foot`.
    fn strike(&mut self, i: usize, req: &kick::KickRequest, foot: kick::Foot) {
        let p = &self.players[i];
        let team = p.team;
        let mut kicker = p.clone();
        let mut req = *req;
        if req.kind == kick::KickKind::Shot && req.pick_corner {
            if let Some(t) = req.target {
                let side = if t.x > 0.0 { Side::Right } else { Side::Left };
                let goal = self.pitch.goal_centre(side);
                let keeper = self
                    .players
                    .iter()
                    .find(|q| q.team != team && q.role == Role::Goalkeeper && q.pos.flat_distance(goal) < 6.0)
                    .map(|q| q.pos);
                kick::aim_away_from_keeper(&mut req, &self.pitch, side, self.ball.pos, keeper);
            }
        }
        if foot != p.preferred_foot() && !req.kind.from_hands() {
            // Weaker foot: less accurate and a little less power.
            kicker.skill *= 0.72;
            req.power *= 0.9;
        }
        let mates: Vec<Player> = self.players.iter().filter(|m| m.team == team).cloned().collect();
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
        self.ball.grounded = plan.vel.y.abs() < 0.05 && self.ball.height(&self.ball_spec) < 0.01;
        let p = &mut self.players[i];
        p.since_kick = 0.0;
        p.ball_ghost = 0.22;
        p.touch_cooldown = 0.4;
        self.possession.touched(i, self.time);
        self.possession.owner = None;
        self.events.push(GameEvent::Kick {
            player: i,
            kind: req.kind,
            speed: plan.vel.length(),
        });
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
            if let Some(k) = p.kick {
                if !k.done_contact && !k.req.kind.from_hands() {
                    // Arrive beside the ball so it meets the kicking foot at contact.
                    let rem = (k.windup - k.t).max(0.03);
                    let f = k.req.dir;
                    let r = f.cross(Vec3::Y);
                    let ball_then = self.ball.pos + self.ball.vel.flat() * rem;
                    let spot = ball_then - f * 0.36 - r * (k.foot.side() * 0.13);
                    let to = (spot - p.pos).flat();
                    p.desired = (to / rem).clamp_length(self.player_spec.sprint_speed);
                } else if k.req.kind.from_hands() {
                    // Step into the throw.
                    p.desired = k.req.dir * 1.2;
                } else {
                    // Follow-through: carry on, without accelerating.
                    p.desired = wanted.clamp_length(p.vel.flat_length().max(2.5));
                }
            } else if let Some(req) = p.pending_kick {
                // Step to the ball to strike it, approaching from behind the kick
                // line with the ball beside the preferred foot.
                let side = req.dir.cross(Vec3::Y) * p.preferred_foot().side();
                let spot = self.ball.pos + self.ball.vel.flat() * 0.1 - req.dir * 0.42 - side * 0.13;
                let to = (spot - p.pos).flat();
                if to.length() < 3.0 && to.length() > 0.15 {
                    // Catch up with a rolling ball, then match its pace.
                    let dir = to.normalize_or(req.dir);
                    let ball_along = self.ball.vel.flat().dot(dir).max(0.0);
                    let closing = (to.length() * 4.0).clamp(1.0, 3.0);
                    let speed = wanted.length().max(3.5).max(ball_along + closing).min(self.player_spec.sprint_speed);
                    p.desired = dir * speed;
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
        self.step_trips();
        player::resolve_player_collisions(&mut self.players, &self.player_spec);
        for p in self.players.iter_mut() {
            player::resolve_arena(p, &self.player_spec, &self.pitch);
        }
    }

    /// Slides that start this step, and players whose feet a sliding leg takes away.
    fn step_trips(&mut self) {
        for i in 0..self.players.len() {
            let p = &self.players[i];
            if let Some(s) = p.slide {
                if s.t <= crate::PHYSICS_DT * 1.01 {
                    self.events.push(GameEvent::Slide { player: i });
                }
            }
            let Some(leg) = p.slide_leg() else {
                continue;
            };
            if p.vel.flat_length() < 1.5 {
                continue;
            }
            for j in 0..self.players.len() {
                let q = &self.players[j];
                if j == i || q.on_ground() || q.airborne() || q.dive_timer > 0.0 || q.holding_ball {
                    continue;
                }
                // The legs of a standing player: a short segment above his feet.
                let (pa, pb) = (q.pos + v3(0.0, 0.1, 0.0), q.pos + v3(0.0, 0.5, 0.0));
                let (x, y, _, _) = crate::math::closest_points_segments(leg.a, leg.b, pa, pb);
                if x.distance(y) < leg.radius + 0.12 {
                    let dir = q.vel.flat().normalize_or(self.players[i].vel.flat().normalize_or(q.forward()));
                    let duration = self.player_spec.fall_time;
                    let q = &mut self.players[j];
                    q.fall = Some(player::Fall { t: 0.0, dir, duration });
                    q.vel = q.vel.flat() * 0.6;
                    q.pending_kick = None;
                    q.kick = None;
                    q.header = None;
                    self.events.push(GameEvent::Trip { player: j, by: i });
                    if self.possession.owner == Some(j) {
                        self.possession.owner = None;
                    }
                }
            }
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
            self.ball.pos = match h.kick {
                Some(k) if k.req.kind.from_hands() => kick::hand_point(h, k.req.kind, k.t / k.windup),
                _ => keeper::hold_point(h),
            };
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
        // Sliding legs knock the ball on.
        for i in 0..self.players.len() {
            let Some(leg) = self.players[i].slide_leg() else {
                continue;
            };
            let surf = crate::contact::Surface { restitution: 0.45, restitution_drop: 0.0, restitution_min: 0.3, friction: 0.4 };
            let v = self.players[i].vel;
            if contact::capsule_moving(&mut self.ball, spec, &leg, &surf, v) > 0.0 {
                if let Some(s) = self.players[i].slide.as_mut() {
                    if !s.touched {
                        s.touched = true;
                        self.events.push(GameEvent::SlideTouch { player: i });
                    }
                }
                self.possession.touched(i, self.time);
                self.possession.owner = None;
            }
        }
        if let Some(id) = player::ball_header(&mut self.ball, spec, &mut self.players, &self.player_spec, &mut self.rng) {
            self.events.push(GameEvent::Header { player: id });
            self.possession.touched(id, self.time);
            self.possession.owner = None;
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

    /// A player at x = 0 facing +x with the ball just in front of the right foot.
    fn kicker_world(left_footed: bool, ball_lat: f32) -> (World, usize) {
        let mut w = World::new();
        let a = w.spawn_player(0, Role::Pivot, 9, v3(0.0, 0.0, 0.0));
        w.players[a].left_footed = left_footed;
        w.players[a].facing = 0.0;
        // character right = +z when facing +x
        w.ball.place(&w.ball_spec.clone(), 0.4, ball_lat);
        (w, a)
    }

    #[test]
    fn kick_winds_up_strikes_at_contact_and_follows_through() {
        use crate::kick::{KickKind, KickRequest};
        let (mut w, a) = kicker_world(false, 0.12);
        let req = KickRequest::new(KickKind::Shot, Vec3::X, 0.8).with_target(v3(20.5, 1.0, 0.0));
        let windup = KickKind::Shot.windup(0.8);
        w.players[a].pending_kick = Some(req);
        let mut t = 0.0;
        let mut kicked_at = None;
        while t < 1.5 {
            w.step(PHYSICS_DT);
            t += PHYSICS_DT;
            for e in w.drain_events() {
                if let GameEvent::Kick { player, .. } = e {
                    assert_eq!(player, a);
                    kicked_at = Some(t);
                }
            }
            if kicked_at.is_none() {
                assert!(w.ball.speed() < 1.0, "ball struck before contact (t {t})");
            }
        }
        let k = kicked_at.expect("no kick");
        assert!((k - windup).abs() < 0.03, "contact at {k}, wind-up {windup}");
        assert!(w.players[a].kick.is_none(), "follow-through should be over");
        assert!(w.ball.pos.x > 10.0);
    }

    #[test]
    fn ball_gone_at_contact_is_an_air_kick() {
        use crate::kick::{KickKind, KickRequest};
        let (mut w, a) = kicker_world(false, 0.12);
        w.players[a].pending_kick = Some(KickRequest::new(KickKind::Pass, Vec3::X, 0.6));
        w.step(PHYSICS_DT);
        assert!(w.players[a].kick.is_some(), "wind-up should start");
        // Someone takes the ball away during the wind-up.
        w.ball.place(&w.ball_spec.clone(), 3.0, 3.0);
        let mut whiff = false;
        for _ in 0..240 {
            w.step(PHYSICS_DT);
            for e in w.drain_events() {
                assert!(!matches!(e, GameEvent::Kick { .. }), "kicked a ball that wasn't there");
                whiff |= matches!(e, GameEvent::Whiff { player } if player == a);
            }
        }
        assert!(whiff);
        assert!(w.ball.speed() < 0.5);
    }

    #[test]
    fn players_use_their_preferred_foot_and_the_other_when_they_must() {
        use crate::kick::{Foot, KickKind, KickRequest};
        for (left_footed, lat, want) in [(false, 0.0, Foot::Right), (true, 0.0, Foot::Left), (false, -0.3, Foot::Left), (true, 0.3, Foot::Right)] {
            let (mut w, a) = kicker_world(left_footed, lat);
            w.players[a].pending_kick = Some(KickRequest::new(KickKind::Pass, Vec3::X, 0.5));
            w.step(PHYSICS_DT);
            let k = w.players[a].kick.expect("wind-up");
            assert_eq!(k.foot, want, "left-footed {left_footed}, ball at {lat}");
        }
    }

    #[test]
    fn keeper_rolls_and_throws_from_the_hands() {
        use crate::kick::{KickKind, KickRequest};
        for kind in [KickKind::Roll, KickKind::Throw] {
            let mut w = World::new();
            let gk = w.spawn_player(0, Role::Goalkeeper, 1, v3(-17.0, 0.0, 0.0));
            let mate = w.spawn_player(0, Role::Winger, 7, v3(if kind == KickKind::Roll { -6.0 } else { 2.0 }, 0.0, 4.0));
            w.players[gk].holding_ball = true;
            w.players[gk].facing = 0.3;
            let target = w.players[mate].pos;
            let dir = (target - w.players[gk].pos).flat().normalize_or(Vec3::X);
            w.players[gk].pending_kick = Some(KickRequest::new(kind, dir, 0.5).with_target(target));
            let mut released = None;
            let mut max_h: f32 = 0.0;
            for _ in 0..(240 * 3) {
                w.step(PHYSICS_DT);
                if released.is_none() && !w.players[gk].holding_ball {
                    released = Some(w.ball.pos);
                }
                if released.is_some() {
                    max_h = max_h.max(w.ball.pos.y);
                }
            }
            let at = released.expect("never released");
            match kind {
                KickKind::Roll => {
                    assert!(at.y < 0.2, "roll released at {at:?}");
                    assert!(max_h < 0.3, "roll bounced to {max_h}");
                }
                _ => assert!(at.y > 1.7, "throw released at {at:?}"),
            }
            assert!(w.ball.pos.flat_distance(target) < 3.0, "{kind:?} ended {:?} vs mate {target:?}", w.ball.pos);
        }
    }

    #[test]
    fn aimed_header_goes_where_aimed_and_unaimed_bounces() {
        use crate::player::HeaderAim;
        for aimed in [true, false] {
            let mut w = World::new();
            let a = w.spawn_player(0, Role::Pivot, 9, v3(10.0, 0.0, 0.0));
            w.players[a].facing = std::f32::consts::PI; // facing -x, towards the ball
            let target = v3(10.0, 0.5, 8.0); // to his side
            if aimed {
                w.players[a].header = Some(HeaderAim { dir: Vec3::Z, target: Some(target), power: 0.6, ttl: 2.0 });
            }
            // A ball dropping onto his head from the -x side.
            let head = w.players[a].head(&w.player_spec);
            w.ball.pos = head + v3(-3.0, 0.6, 0.0);
            w.ball.prev_pos = w.ball.pos;
            w.ball.vel = v3(10.0, -1.0, 0.0);
            w.ball.grounded = false;
            let mut header = false;
            for _ in 0..120 {
                w.step(PHYSICS_DT);
                for e in w.drain_events() {
                    header |= matches!(e, GameEvent::Header { player } if player == a);
                }
                if header || w.ball.pos.x > 12.0 || w.ball.vel.x < 0.0 {
                    break;
                }
            }
            if aimed {
                assert!(header, "no header");
                let dir = w.ball.vel.flat().normalize_or_zero();
                let want = (target - head).flat().normalize_or_zero();
                assert!(dir.dot(want) > 0.95, "header went {dir:?}, aimed {want:?}");
            } else {
                assert!(!header);
                // bounced back off the head/body
                assert!(w.ball.vel.x < 3.0, "ball went through ({:?})", w.ball.vel);
            }
        }
    }

    #[test]
    fn slide_covers_realistic_distance_then_gets_up() {
        let mut w = World::new();
        let a = w.spawn_player(0, Role::Defender, 4, v3(0.0, 0.0, 0.0));
        w.players[a].vel = v3(6.0, 0.0, 0.0);
        w.players[a].desired = v3(6.0, 0.0, 0.0);
        w.players[a].slide_request = Some(Vec3::X);
        w.ball.place(&w.ball_spec.clone(), -10.0, 5.0);
        let mut t = 0.0;
        let mut stopped_at = None;
        let mut up_at = None;
        while t < 4.0 {
            // he keeps "wanting" to run: a slide gives no control
            w.players[a].desired = v3(0.0, 0.0, 6.0);
            w.step(PHYSICS_DT);
            t += PHYSICS_DT;
            let p = &w.players[a];
            if stopped_at.is_none() && p.vel.flat_length() < 0.01 {
                stopped_at = Some((t, p.pos.x));
                assert!(p.pos.z.abs() < 0.01, "steered during the slide");
            }
            if up_at.is_none() && p.slide.is_none() {
                up_at = Some(t);
            }
        }
        let (ts, x) = stopped_at.expect("never stopped");
        assert!(x > 2.0 && x < 4.5, "slid {x} m");
        assert!(ts > 0.6 && ts < 1.4, "slide lasted {ts} s");
        let up = up_at.expect("never got up");
        assert!(up > ts + 0.5 && up < ts + 1.2, "got up at {up}");
    }

    #[test]
    fn sliding_leg_knocks_the_ball_and_trips_players() {
        let mut w = World::new();
        let a = w.spawn_player(0, Role::Defender, 4, v3(0.0, 0.0, 0.0));
        let b = w.spawn_player(1, Role::Pivot, 9, v3(2.6, 0.0, 0.15));
        w.players[a].vel = v3(5.0, 0.0, 0.0);
        w.players[a].slide_request = Some(Vec3::X);
        w.ball.place(&w.ball_spec.clone(), 1.6, 0.1);
        let (mut touched, mut tripped) = (false, false);
        w.players[b].dribble = false; // just standing there
        for _ in 0..240 {
            w.players[b].desired = Vec3::ZERO;
            w.step(PHYSICS_DT);
            for e in w.drain_events() {
                touched |= matches!(e, GameEvent::SlideTouch { player } if player == a);
                tripped |= matches!(e, GameEvent::Trip { player, by } if player == b && by == a);
            }
        }
        assert!(touched, "the leg should meet the ball");
        assert!(w.ball.pos.flat_distance(v3(1.6, 0.0, 0.1)) > 0.4, "ball knocked on to {:?}", w.ball.pos);
        assert!(tripped, "the player in the way should go down");
        assert!(w.players[b].fall.is_some());
        for _ in 0..(240 * 2) {
            w.step(PHYSICS_DT);
        }
        assert!(w.players[b].fall.is_none(), "should get up");
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
