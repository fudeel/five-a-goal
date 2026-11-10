//! Match rules and flow: kick-offs, goals, halves, clock, full time.
//!
//! [`Match`] owns the [`World`] and drives it; the AI plugs in here too.

use crate::ai::{Ai, AiContext, AiTuning};
use crate::events::GameEvent;
use crate::formation::{self, Slot};
use crate::kick::{KickKind, KickRequest};
use crate::math::{v3, Rng, Vec3};
use crate::pitch::Side;
use crate::player::Role;
use crate::world::World;

/// AI strength.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Difficulty {
    Easy,
    Normal,
    Hard,
}

/// How the match is set up.
#[derive(Clone, Debug)]
pub struct MatchConfig {
    /// Players per team (1..=5).
    pub team_size: usize,
    /// Length of one half (s).
    pub half_length: f32,
    /// Team of the human (0 = home). `None` = AI vs AI.
    pub human_team: Option<usize>,
    /// Index into the line-up of the slot the human controls.
    pub human_slot: usize,
    pub difficulty: Difficulty,
    pub seed: u64,
}

impl Default for MatchConfig {
    fn default() -> Self {
        Self {
            team_size: 5,
            half_length: 180.0,
            human_team: Some(0),
            human_slot: 4,
            difficulty: Difficulty::Normal,
            seed: 7,
        }
    }
}

impl MatchConfig {
    /// The slot the human controls, clamped to the line-up (and never the keeper
    /// when there is an outfield alternative).
    pub fn human_slot_index(&self) -> usize {
        let l = formation::lineup(self.team_size);
        let mut i = self.human_slot.min(l.len() - 1);
        if l[i].role == Role::Goalkeeper && l.len() > 1 {
            i = l.len() - 1;
        }
        i
    }
}

/// Where the match is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Phase {
    /// Players in their halves, waiting for `team` to kick off.
    Kickoff {
        team: usize,
    },
    Playing,
    /// A goal was scored by `team`; restart after the timer.
    Goal {
        team: usize,
        timer: f32,
    },
    HalfTime {
        timer: f32,
    },
    FullTime,
}

#[derive(Clone, Debug)]
pub struct MatchState {
    pub score: [u32; 2],
    /// 1 or 2.
    pub half: u8,
    /// Seconds played in the current half.
    pub clock: f32,
    pub phase: Phase,
    /// Seconds spent in the current phase.
    pub phase_time: f32,
    /// Team that kicked off the first half.
    pub first_kickoff: usize,
    /// Player id of the kick-off taker (during `Kickoff`).
    pub kickoff_taker: Option<usize>,
    pub last_scorer: Option<usize>,
    /// Seconds the ball has been dead and out of everybody's reach.
    pub stuck_time: f32,
}

/// Events for the presentation layer (in addition to the world's physics events).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MatchEvent {
    KickoffReady {
        team: usize,
    },
    /// The referee restarted play with a dropped ball at this spot.
    DroppedBall {
        at: Vec3,
    },
    Goal {
        team: usize,
        scorer: Option<usize>,
    },
    HalfTime,
    SecondHalf,
    FullTime,
}

pub struct Match {
    pub config: MatchConfig,
    pub world: World,
    pub state: MatchState,
    /// Team index of each line-up slot's player: `slots[team][i]` = player id.
    pub slots: [Vec<usize>; 2],
    /// Physics events of the last steps (drained by the game).
    pub events: Vec<GameEvent>,
    pub match_events: Vec<MatchEvent>,
    pub rng: Rng,
    pub ai: Ai,
}

const GOAL_PAUSE: f32 = 3.2;
const HALFTIME_PAUSE: f32 = 3.0;
const AI_KICKOFF_DELAY: f32 = 1.2;

impl Match {
    pub fn new(config: MatchConfig) -> Self {
        let mut world = World::new();
        world.rng = Rng::new(config.seed);
        let mut rng = Rng::new(config.seed ^ 0xA5A5);
        let lineup = formation::lineup(config.team_size);
        let mut slots = [Vec::new(), Vec::new()];
        let tuning = [0, 1].map(|t| {
            if config.human_team == Some(t) {
                AiTuning::for_difficulty(Difficulty::Normal)
            } else {
                AiTuning::for_difficulty(config.difficulty)
            }
        });
        for team in 0..2 {
            for (i, s) in lineup.iter().enumerate() {
                let id = world.spawn_player(team, s.role, s.number, Vec3::ZERO);
                world.players[id].skill =
                    (tuning[team].skill + rng.range(-0.06, 0.06)).clamp(0.3, 0.98);
                world.players[id].human =
                    config.human_team == Some(team) && i == config.human_slot_index();
                slots[team].push(id);
            }
        }
        let first = 0;
        let state = MatchState {
            score: [0, 0],
            half: 1,
            clock: 0.0,
            phase: Phase::Kickoff { team: first },
            phase_time: 0.0,
            first_kickoff: first,
            kickoff_taker: None,
            last_scorer: None,
            stuck_time: 0.0,
        };
        let ai = Ai::new(world.players.len(), tuning);
        let mut m = Self {
            config,
            world,
            state,
            slots,
            events: Vec::new(),
            match_events: Vec::new(),
            rng,
            ai,
        };
        m.setup_kickoff(first);
        m
    }

    /// The goal `team` attacks in the current half.
    pub fn attack_side(&self, team: usize) -> Side {
        let first_half_right = team == 0;
        if (self.state.half == 1) == first_half_right {
            Side::Right
        } else {
            Side::Left
        }
    }

    pub fn defend_side(&self, team: usize) -> Side {
        self.attack_side(team).opposite()
    }

    /// Line-up slot of a player.
    pub fn slot_of(&self, id: usize) -> Slot {
        let p = &self.world.players[id];
        let i = self.slots[p.team]
            .iter()
            .position(|&x| x == id)
            .unwrap_or(0);
        formation::lineup(self.config.team_size)[i]
    }

    /// Places everybody for a kick-off by `team`.
    pub fn setup_kickoff(&mut self, team: usize) {
        let lineup = formation::lineup(self.config.team_size);
        let bs = self.world.ball_spec.clone();
        self.world.ball.place(&bs, 0.0, 0.0);
        for t in 0..2 {
            let attack = self.attack_side(t);
            for (i, &id) in self.slots[t].iter().enumerate() {
                let mut pos = formation::to_world(&self.world.pitch, attack, lineup[i].base);
                if t != team {
                    // Opponents stay outside the centre circle.
                    let r = self.world.pitch.spec.centre_circle_radius + 0.6;
                    if pos.flat_length() < r {
                        pos = pos.flat().normalize_or(v3(-attack.sign(), 0.0, 0.0)) * r;
                    }
                }
                let p = &mut self.world.players[id];
                p.pos = pos;
                p.vel = Vec3::ZERO;
                p.desired = Vec3::ZERO;
                p.pending_kick = None;
                p.kick = None;
                p.jump = None;
                p.jump_request = None;
                p.air = 0.0;
                p.air_vel = 0.0;
                p.header = None;
                p.slide = None;
                p.slide_request = None;
                p.fall = None;
                p.holding_ball = false;
                p.stumble = 0.0;
                p.dive_timer = 0.0;
                p.down = 0.0;
                p.facing = if attack == Side::Right {
                    0.0
                } else {
                    std::f32::consts::PI
                };
            }
        }
        // Taker: the human if his team kicks off, else the most advanced outfield player.
        let taker = self.slots[team]
            .iter()
            .copied()
            .find(|&id| self.world.players[id].human)
            .or_else(|| self.slots[team].last().copied());
        if let Some(id) = taker {
            let attack = self.attack_side(team);
            let p = &mut self.world.players[id];
            p.pos = v3(-attack.sign() * 0.45, 0.0, 0.05);
            p.facing = if attack == Side::Right {
                0.0
            } else {
                std::f32::consts::PI
            };
        }
        self.state.kickoff_taker = taker;
        self.world.possession = Default::default();
        self.state.phase = Phase::Kickoff { team };
        self.state.phase_time = 0.0;
        self.match_events.push(MatchEvent::KickoffReady { team });
    }

    pub fn is_running(&self) -> bool {
        !matches!(self.state.phase, Phase::FullTime)
    }

    /// One fixed step of the whole match. Controllers (human input, AI) must have
    /// written player intents before calling this.
    pub fn step(&mut self, dt: f32) {
        self.state.phase_time += dt;
        let ctx = AiContext {
            attack: [self.attack_side(0), self.attack_side(1)],
            team_size: self.config.team_size,
            phase: self.state.phase,
            slots: &self.slots,
        };
        self.world.defend = [self.defend_side(0), self.defend_side(1)];
        self.ai.think(&mut self.world, &ctx, dt);
        match self.state.phase {
            Phase::Kickoff { team } => self.step_kickoff(team, dt),
            Phase::Playing => {
                self.world.step(dt);
                self.state.clock += dt;
                self.process_events();
                self.check_stuck_ball(dt);
                if matches!(self.state.phase, Phase::Playing)
                    && self.state.clock >= self.config.half_length
                {
                    self.end_half();
                }
            }
            Phase::Goal { team, timer } => {
                self.world.step(dt);
                self.absorb_events();
                let timer = timer - dt;
                if timer <= 0.0 {
                    self.setup_kickoff(1 - team);
                } else {
                    self.state.phase = Phase::Goal { team, timer };
                }
            }
            Phase::HalfTime { timer } => {
                let timer = timer - dt;
                if timer <= 0.0 {
                    self.state.half = 2;
                    self.state.clock = 0.0;
                    self.match_events.push(MatchEvent::SecondHalf);
                    self.setup_kickoff(1 - self.state.first_kickoff);
                } else {
                    self.state.phase = Phase::HalfTime { timer };
                }
            }
            Phase::FullTime => {
                self.world.step(dt);
                self.absorb_events();
            }
        }
    }

    fn step_kickoff(&mut self, team: usize, dt: f32) {
        let taker = self.state.kickoff_taker;
        // Only the taker may move until the ball is played.
        for p in self.world.players.iter_mut() {
            if Some(p.id) != taker && !p.human {
                p.desired = Vec3::ZERO;
            }
        }
        if let Some(t) = taker {
            if !self.world.players[t].human
                && self.state.phase_time > AI_KICKOFF_DELAY
                && !self.world.players[t].busy_kicking()
            {
                // Simple kick-off: play it to the nearest teammate (or just forward).
                let attack = self.attack_side(team);
                let tp = self.world.players[t].pos;
                let mate = self.slots[team]
                    .iter()
                    .copied()
                    .filter(|&id| id != t && self.world.players[id].role != Role::Goalkeeper)
                    .min_by(|&a, &b| {
                        let da = self.world.players[a].pos.distance(tp);
                        let db = self.world.players[b].pos.distance(tp);
                        da.partial_cmp(&db).unwrap()
                    });
                match mate {
                    Some(m) => {
                        let mp = self.world.players[m].pos;
                        let req = KickRequest::new(KickKind::Pass, (mp - tp).flat(), 0.25)
                            .with_target(mp);
                        self.world.players[t].pending_kick = Some(req);
                    }
                    None => {
                        // Nobody to pass to: start by dribbling forward.
                        let p = &mut self.world.players[t];
                        p.desired = v3(attack.sign() * 3.0, 0.0, 0.0);
                        p.dribble = true;
                    }
                }
            }
        }
        self.world.step(dt);
        self.absorb_events();
        // Keep everybody in their own half (and opponents outside the circle).
        let r = self.world.pitch.spec.centre_circle_radius + 0.3;
        for tm in 0..2 {
            let own = -self.attack_side(tm).sign();
            for &id in &self.slots[tm] {
                if Some(id) == taker {
                    continue;
                }
                let p = &mut self.world.players[id];
                if p.pos.x * own < 0.3 {
                    p.pos.x = own * 0.3;
                }
                if tm != team && p.pos.flat_length() < r {
                    p.pos = p.pos.flat().normalize_or(v3(own, 0.0, 0.0)) * r;
                }
            }
        }
        if self.world.ball.pos.flat_length() > 0.3
            || self.world.possession.last_touch.is_some() && self.world.ball.speed() > 0.5
        {
            self.state.phase = Phase::Playing;
            self.state.phase_time = 0.0;
            self.state.kickoff_taker = None;
        }
    }

    /// Moves world events into `events` without applying rules.
    fn absorb_events(&mut self) {
        let ev = self.world.drain_events();
        self.events.extend(ev);
    }

    fn process_events(&mut self) {
        let ev = self.world.drain_events();
        for e in &ev {
            if let GameEvent::GoalScored { side } = e {
                if matches!(self.state.phase, Phase::Playing) {
                    let team = if self.attack_side(0) == *side { 0 } else { 1 };
                    self.state.score[team] += 1;
                    let scorer = self.world.possession.last_touch;
                    self.state.last_scorer = scorer;
                    self.state.phase = Phase::Goal {
                        team,
                        timer: GOAL_PAUSE,
                    };
                    self.state.phase_time = 0.0;
                    self.match_events.push(MatchEvent::Goal { team, scorer });
                }
            }
        }
        self.events.extend(ev);
    }

    /// Dropped ball when the ball comes to rest where nobody can play it
    /// (behind a goal, on top of the net) or nobody plays it for a long time.
    fn check_stuck_ball(&mut self, dt: f32) {
        let w = &self.world;
        let b = &w.ball;
        let nearest = w
            .players
            .iter()
            .map(|p| p.pos.flat_distance(b.pos))
            .fold(f32::MAX, f32::min);
        let slow = b.speed() < 0.4 && !w.players.iter().any(|p| p.holding_ball);
        let unreachable =
            b.pos.x.abs() > w.pitch.half_length() + 0.1 || b.height(&w.ball_spec) > 1.2;
        if slow && (unreachable || nearest > 1.2) && w.possession.owner.is_none() {
            self.state.stuck_time += dt;
        } else {
            self.state.stuck_time = 0.0;
        }
        let limit = if unreachable { 2.5 } else { 12.0 };
        if self.state.stuck_time > limit {
            self.state.stuck_time = 0.0;
            let hl = w.pitch.half_length() - 2.5;
            let hw = w.pitch.half_width() - 1.0;
            let at = v3(b.pos.x.clamp(-hl, hl), 0.0, b.pos.z.clamp(-hw, hw));
            let bs = self.world.ball_spec.clone();
            self.world.ball.place(&bs, at.x, at.z);
            self.world.ball.vel = v3(0.0, 2.5, 0.0); // dropped from the referee's hand
            self.world.ball.grounded = false;
            self.world.possession = Default::default();
            self.match_events.push(MatchEvent::DroppedBall { at });
        }
    }

    fn end_half(&mut self) {
        if self.state.half == 1 {
            self.state.phase = Phase::HalfTime {
                timer: HALFTIME_PAUSE,
            };
            self.match_events.push(MatchEvent::HalfTime);
        } else {
            self.state.phase = Phase::FullTime;
            self.match_events.push(MatchEvent::FullTime);
        }
        self.state.phase_time = 0.0;
    }

    pub fn drain_events(&mut self) -> (Vec<GameEvent>, Vec<MatchEvent>) {
        (
            std::mem::take(&mut self.events),
            std::mem::take(&mut self.match_events),
        )
    }

    /// "mm:ss" of the current half clock.
    pub fn clock_text(&self) -> String {
        let t = self.state.clock.min(self.config.half_length).max(0.0) as u32;
        format!("{:02}:{:02}", t / 60, t % 60)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PHYSICS_DT;

    fn ai_match(size: usize, half: f32) -> Match {
        Match::new(MatchConfig {
            team_size: size,
            half_length: half,
            human_team: None,
            ..MatchConfig::default()
        })
    }

    #[test]
    fn kickoff_places_teams_in_their_halves() {
        for size in 1..=5 {
            let m = ai_match(size, 60.0);
            assert_eq!(m.world.players.len(), size * 2);
            for p in &m.world.players {
                let own = -m.attack_side(p.team).sign();
                if Some(p.id) != m.state.kickoff_taker {
                    assert!(
                        p.pos.x * own > 0.0,
                        "size {size}: {:?} not in own half",
                        p.pos
                    );
                }
                if p.team == 1 {
                    assert!(p.pos.flat_length() > m.world.pitch.spec.centre_circle_radius);
                }
            }
        }
    }

    #[test]
    fn ai_kickoff_starts_play_and_clock_runs() {
        let mut m = ai_match(5, 60.0);
        for _ in 0..(240 * 4) {
            m.step(PHYSICS_DT);
        }
        assert_eq!(m.state.phase, Phase::Playing);
        assert!(m.state.clock > 1.0);
    }

    #[test]
    fn goal_updates_score_and_restarts_with_the_other_team() {
        let mut m = ai_match(5, 120.0);
        // get into play
        while m.state.phase != Phase::Playing {
            m.step(PHYSICS_DT);
        }
        // team 0 attacks the right goal in the first half
        assert_eq!(m.attack_side(0), Side::Right);
        let gk = m.slots[1][0];
        m.world.players[gk].pos = v3(15.0, 0.0, 8.0);
        m.world
            .debug_fire(v3(14.0, 0.6, 0.0), v3(20.5, 1.0, -0.4), 25.0, Vec3::ZERO);
        let mut saw_goal = false;
        for _ in 0..(240 * 6) {
            m.step(PHYSICS_DT);
            let (_, me) = m.drain_events();
            if me
                .iter()
                .any(|e| matches!(e, MatchEvent::Goal { team: 0, .. }))
            {
                saw_goal = true;
            }
            if saw_goal && matches!(m.state.phase, Phase::Kickoff { .. }) {
                break;
            }
        }
        assert!(saw_goal);
        assert_eq!(m.state.score, [1, 0]);
        assert_eq!(m.state.phase, Phase::Kickoff { team: 1 });
    }

    #[test]
    fn halves_swap_sides_and_full_time_ends_the_match() {
        let mut m = ai_match(3, 4.0);
        let mut events = Vec::new();
        for _ in 0..(240 * 20) {
            m.step(PHYSICS_DT);
            events.extend(m.drain_events().1);
            if !m.is_running() {
                break;
            }
        }
        assert!(events.contains(&MatchEvent::HalfTime));
        assert!(events.contains(&MatchEvent::SecondHalf));
        assert_eq!(m.state.phase, Phase::FullTime);
        assert_eq!(m.state.half, 2);
        assert_eq!(m.attack_side(0), Side::Left);
    }

    #[test]
    fn every_team_size_plays_to_full_time_with_a_human() {
        for size in 1..=5 {
            for slot in 0..size {
                let mut m = Match::new(MatchConfig {
                    team_size: size,
                    human_slot: slot,
                    half_length: 8.0,
                    ..MatchConfig::default()
                });
                let mut steps = 0;
                while m.is_running() && steps < 240 * 60 {
                    // A mostly idle human who takes his kick-offs.
                    if let (Phase::Kickoff { .. }, Some(t)) = (m.state.phase, m.state.kickoff_taker)
                    {
                        if m.world.players[t].human && !m.world.players[t].busy_kicking() {
                            let dir = v3(m.attack_side(0).sign(), 0.0, 0.0);
                            m.world.players[t].pending_kick =
                                Some(KickRequest::new(KickKind::Pass, dir, 0.2));
                        }
                    }
                    m.step(PHYSICS_DT);
                    steps += 1;
                    assert!(m.world.ball.is_finite());
                }
                assert_eq!(
                    m.state.phase,
                    Phase::FullTime,
                    "size {size} slot {slot} did not finish"
                );
            }
        }
    }

    #[test]
    fn human_controls_exactly_one_player() {
        for size in 1..=5 {
            let m = Match::new(MatchConfig {
                team_size: size,
                ..MatchConfig::default()
            });
            let humans: Vec<_> = m.world.players.iter().filter(|p| p.human).collect();
            assert_eq!(humans.len(), 1);
            assert_eq!(humans[0].team, 0);
            if size > 1 {
                assert_ne!(humans[0].role, Role::Goalkeeper);
            }
        }
    }
}
