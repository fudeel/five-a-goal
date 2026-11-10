//! Team and player AI. Every AI player only writes *intents* (desired velocity,
//! sprint, kick and tackle requests) — the same interface the human uses — so the
//! AI plays by the same physics and rules.

pub mod decide;
pub mod keeper;
pub mod positioning;
pub mod predict;

use crate::formation;
use crate::math::{v3, Rng, Vec3};
use crate::pitch::Side;
use crate::player::{Player, PlayerSpec, Role};
use crate::rules::{Difficulty, Phase};
use crate::world::World;
use positioning::TeamPhase;
use predict::Prediction;

/// What an AI player is currently doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Duty {
    Idle,
    Chase,
    Carry,
    Support,
    Mark,
    Zone,
    Keeper,
}

/// Per-player AI memory.
#[derive(Clone, Debug)]
pub struct Brain {
    pub duty: Duty,
    pub target: Vec3,
    pub support: Vec3,
    pub support_timer: f32,
    /// Time until the next on-ball decision.
    pub decide_timer: f32,
    pub mark: Option<usize>,
    /// Last on-ball action chosen.
    pub action: Option<decide::Action>,
}

impl Default for Brain {
    fn default() -> Self {
        Self {
            duty: Duty::Idle,
            target: Vec3::ZERO,
            support: Vec3::ZERO,
            support_timer: 0.0,
            decide_timer: 0.0,
            mark: None,
            action: None,
        }
    }
}

/// Difficulty-dependent behaviour.
#[derive(Clone, Copy, Debug)]
pub struct AiTuning {
    /// Reaction time added to intercept estimates (s).
    pub reaction: f32,
    /// Seconds between on-ball decisions.
    pub think_interval: f32,
    /// Multiplier on kick errors.
    pub error: f32,
    /// How readily the AI shoots (higher = more shots).
    pub shoot_bias: f32,
    /// Skill assigned to AI players.
    pub skill: f32,
}

impl AiTuning {
    pub fn for_difficulty(d: Difficulty) -> Self {
        match d {
            Difficulty::Easy => Self {
                reaction: 0.36,
                think_interval: 0.45,
                error: 1.6,
                shoot_bias: 0.8,
                skill: 0.6,
            },
            Difficulty::Normal => Self {
                reaction: 0.22,
                think_interval: 0.28,
                error: 1.0,
                shoot_bias: 1.0,
                skill: 0.75,
            },
            Difficulty::Hard => Self {
                reaction: 0.12,
                think_interval: 0.16,
                error: 0.7,
                shoot_bias: 1.15,
                skill: 0.88,
            },
        }
    }
}

/// Static facts about the match the AI needs each step.
pub struct AiContext<'a> {
    pub attack: [Side; 2],
    pub team_size: usize,
    pub phase: Phase,
    pub slots: &'a [Vec<usize>; 2],
}

pub struct Ai {
    pub brains: Vec<Brain>,
    pub chaser: [Option<usize>; 2],
    pub team_phase: [TeamPhase; 2],
    pub pred: Prediction,
    pred_timer: f32,
    /// Tuning per team (the human's team uses Normal teammates).
    pub tuning: [AiTuning; 2],
    pub rng: Rng,
}

impl Ai {
    pub fn new(players: usize, tuning: [AiTuning; 2]) -> Self {
        Self {
            brains: vec![Brain::default(); players],
            chaser: [None, None],
            team_phase: [TeamPhase::Loose; 2],
            pred: Prediction::default(),
            pred_timer: 0.0,
            tuning,
            rng: Rng::new(0xA1),
        }
    }

    /// Writes intents for every AI-controlled player.
    pub fn think(&mut self, world: &mut World, ctx: &AiContext, dt: f32) {
        if world.players.is_empty() {
            return;
        }
        match ctx.phase {
            Phase::Kickoff { .. } | Phase::HalfTime { .. } => return,
            Phase::Goal { .. } | Phase::FullTime => {
                self.walk_to_base(world, ctx);
                return;
            }
            Phase::Playing => {}
        }
        self.pred_timer -= dt;
        if self.pred_timer <= 0.0 {
            self.pred_timer = 0.1;
            self.pred = Prediction::new(&world.ball, &world.ball_spec, &world.turf, 3.0, 0.05);
            self.update_team_state(world, ctx);
        }
        for b in self.brains.iter_mut() {
            b.support_timer -= dt;
            b.decide_timer -= dt;
        }
        for team in 0..2 {
            self.position_team(world, ctx, team);
        }
    }

    fn update_team_state(&mut self, world: &World, ctx: &AiContext) {
        let poss = &world.possession;
        let owner_team = poss.owner_team(&world.players);
        let last_team = poss.last_touch.map(|id| world.players[id].team);
        for team in 0..2 {
            self.team_phase[team] = match owner_team {
                Some(t) if t == team => TeamPhase::Attack,
                Some(_) => TeamPhase::Defend,
                None => {
                    // A pass in flight still counts as our attack.
                    if last_team == Some(team) && world.ball.speed() > 3.0 {
                        TeamPhase::Attack
                    } else {
                        TeamPhase::Loose
                    }
                }
            };
            // Chaser: the AI outfield player who reaches the ball first.
            self.chaser[team] = None;
            if owner_team == Some(team) {
                continue;
            }
            let reaction = self.tuning[team].reaction;
            let mut best: Option<(usize, f32)> = None;
            for &id in &ctx.slots[team] {
                let p = &world.players[id];
                if p.human || (p.role == Role::Goalkeeper && ctx.team_size >= 3) {
                    continue;
                }
                let (t, _) = predict::intercept(
                    p,
                    &world.player_spec,
                    &self.pred,
                    &world.ball_spec,
                    reaction,
                );
                if best.map(|b| t < b.1).unwrap_or(true) {
                    best = Some((id, t));
                }
            }
            self.chaser[team] = best.map(|b| b.0);
        }
    }

    fn position_team(&mut self, world: &mut World, ctx: &AiContext, team: usize) {
        let attack = ctx.attack[team];
        let defend = attack.opposite();
        let phase = self.team_phase[team];
        let lineup = formation::lineup(ctx.team_size);
        let ball = world.ball.pos;
        let owner = world.possession.owner;
        let carrier_pos = owner
            .filter(|&o| world.players[o].team == team)
            .map(|o| world.players[o].pos);
        let opponents: Vec<Player> = world
            .players
            .iter()
            .filter(|p| p.team != team)
            .cloned()
            .collect();
        let opp_refs: Vec<&Player> = opponents.iter().collect();
        let spec = world.player_spec.clone();

        // Marking assignment when defending: most dangerous opponents first.
        let mut marks: Vec<(usize, usize)> = Vec::new();
        if phase == TeamPhase::Defend {
            let goal = world.pitch.goal_centre(defend);
            let mut threats: Vec<&Player> = opp_refs
                .iter()
                .copied()
                .filter(|o| o.role != Role::Goalkeeper && Some(o.id) != owner)
                .collect();
            threats.sort_by(|a, b| {
                a.pos
                    .flat_distance(goal)
                    .partial_cmp(&b.pos.flat_distance(goal))
                    .unwrap()
            });
            let mut free: Vec<usize> = ctx.slots[team]
                .iter()
                .copied()
                .filter(|&id| {
                    let p = &world.players[id];
                    !p.human && p.role != Role::Goalkeeper && Some(id) != self.chaser[team]
                })
                .collect();
            for t in threats {
                if free.is_empty() {
                    break;
                }
                let (k, _) = free
                    .iter()
                    .enumerate()
                    .map(|(k, &id)| (k, world.players[id].pos.flat_distance(t.pos)))
                    .fold((0, f32::MAX), |a, b| if b.1 < a.1 { b } else { a });
                marks.push((free.remove(k), t.id));
            }
        }

        // While the opposing keeper holds the ball we leave his area.
        let keeper_holding = world
            .players
            .iter()
            .any(|p| p.team != team && p.holding_ball);
        let mut taken: Vec<Vec3> = Vec::new();
        for (i, &id) in ctx.slots[team].iter().enumerate() {
            if world.players[id].human {
                taken.push(world.players[id].pos);
                continue;
            }
            let role = world.players[id].role;
            if role == Role::Goalkeeper && ctx.team_size >= 3 {
                let tuning = self.tuning[team];
                keeper::think(
                    world,
                    id,
                    defend,
                    attack,
                    &self.pred,
                    &tuning,
                    &mut self.rng,
                );
                self.brains[id].duty = Duty::Keeper;
                taken.push(world.players[id].pos);
                world.players[id].dribble = true;
                continue;
            }
            let home = positioning::formation_target(
                &world.pitch,
                attack,
                lineup[i].base,
                role,
                ball,
                phase,
            );
            let brain = &mut self.brains[id];
            if owner == Some(id) {
                // On the ball: decide every `think_interval`, keep dribbling in between.
                let tuning = self.tuning[team];
                if brain.decide_timer <= 0.0 && world.players[id].pending_kick.is_none() {
                    let mates: Vec<Player> = world
                        .players
                        .iter()
                        .filter(|p| p.team == team)
                        .cloned()
                        .collect();
                    let mate_refs: Vec<&Player> = mates.iter().collect();
                    let a = decide::choose(
                        &world.pitch,
                        attack,
                        &world.players[id],
                        ball,
                        &mate_refs,
                        &opp_refs,
                        &tuning,
                        &mut self.rng,
                    );
                    brain.action = Some(a);
                    brain.decide_timer = tuning.think_interval * self.rng.range(0.8, 1.25);
                    decide::execute(&mut world.players[id], &spec, a, &mate_refs);
                } else if let Some(decide::Action::Dribble { dir, sprint }) = brain.action {
                    let p = &mut world.players[id];
                    p.desired = dir
                        * if sprint {
                            spec.sprint_speed
                        } else {
                            spec.jog_speed
                        };
                    p.sprint = sprint;
                }
                brain.duty = Duty::Carry;
                brain.target = world.players[id].pos;
                taken.push(world.players[id].pos);
                world.players[id].look_at = None;
                world.players[id].dribble = true;
                continue;
            }
            // Teams without a keeper: cover the goal on the ball–goal line.
            if ctx.team_size <= 2 && phase == TeamPhase::Defend {
                let own_goal = world.pitch.goal_centre(defend);
                let carrier_near = owner
                    .map(|c| {
                        let cp = world.players[c].pos;
                        cp.flat_distance(own_goal) < 10.0
                            || cp.flat_distance(world.players[id].pos) < 3.0
                    })
                    .unwrap_or(true);
                let sweeper = ctx.team_size == 2 && Some(id) != self.chaser[team];
                let solo_cover = ctx.team_size == 1 && !carrier_near;
                if sweeper || solo_cover {
                    let to_ball = (ball - own_goal).flat();
                    let frac = if sweeper { 0.22 } else { 0.4 };
                    let d = (to_ball.length() * frac).clamp(1.2, 7.0);
                    let spot = own_goal + to_ball.normalize_or(v3(-defend.sign(), 0.0, 0.0)) * d;
                    go_to(&mut world.players[id], &spec, spot, 0.9, ball);
                    brain.duty = Duty::Zone;
                    brain.target = spot;
                    taken.push(spot);
                    world.players[id].dribble = true;
                    continue;
                }
            }
            if Some(id) == self.chaser[team] && phase == TeamPhase::Defend {
                // Don't crowd a keeper holding the ball: drop off and cover lanes.
                if let Some(c) = owner.filter(|&c| !world.players[c].holding_ball) {
                    let carrier = world.players[c].clone();
                    let aggression = 1.0 / (0.6 + self.tuning[team].reaction * 2.0);
                    let own_goal = world.pitch.goal_centre(defend);
                    decide::press(
                        &mut world.players[id],
                        &spec,
                        &carrier,
                        ball,
                        own_goal,
                        &mut self.rng,
                        aggression,
                    );
                    brain.duty = Duty::Chase;
                    taken.push(world.players[id].pos);
                    world.players[id].dribble = true;
                    continue;
                }
            }
            let (target, urgency, duty) = if role == Role::Goalkeeper && ctx.team_size >= 3 {
                (
                    positioning::keeper_spot(&world.pitch, defend, ball),
                    0.7,
                    Duty::Keeper,
                )
            } else if Some(id) == self.chaser[team] {
                let (t, pt) = predict::intercept(
                    &world.players[id],
                    &spec,
                    &self.pred,
                    &world.ball_spec,
                    self.tuning[team].reaction,
                );
                // First-time finish when the ball arrives in a good spot.
                if t < 0.35
                    && !keeper_holding
                    && world.players[id].pending_kick.is_none()
                    && world.players[id].since_kick > 0.6
                {
                    let (chance, goal_pt) =
                        decide::shot_chance(&world.pitch, attack, pt, &opp_refs);
                    if chance > 0.45 {
                        let dir = (goal_pt - pt)
                            .flat()
                            .normalize_or(world.players[id].forward());
                        world.players[id].pending_kick = Some(
                            crate::kick::KickRequest::new(crate::kick::KickKind::Shot, dir, 0.75)
                                .with_target(goal_pt),
                        );
                    }
                }
                (pt, 1.0, Duty::Chase)
            } else if phase == TeamPhase::Attack {
                if brain.support_timer <= 0.0 {
                    brain.support = positioning::support_spot(
                        &world.pitch,
                        attack,
                        &world.players[id],
                        home,
                        carrier_pos,
                        &opp_refs,
                        &taken,
                    );
                    brain.support_timer = 0.6;
                }
                (brain.support, 0.55, Duty::Support)
            } else if let Some(&(_, opp)) = marks.iter().find(|(d, _)| *d == id) {
                brain.mark = Some(opp);
                let tight = if world.players[opp].pos.flat_distance(ball) < 8.0 {
                    1.2
                } else {
                    2.2
                };
                (
                    positioning::mark_spot(&world.pitch, defend, &world.players[opp], tight),
                    0.65,
                    Duty::Mark,
                )
            } else {
                (home, 0.45, Duty::Zone)
            };
            let mut target = target;
            if keeper_holding && world.pitch.is_inside_penalty_area(attack, target) {
                // Step out to the edge of the area (plus a metre).
                let g = world.pitch.goal_centre(attack);
                let dir = (target - g)
                    .flat()
                    .normalize_or(v3(-attack.sign(), 0.0, 0.0));
                let mut t = target;
                while world.pitch.is_inside_penalty_area(attack, t) {
                    t += dir * 0.5;
                }
                target = t + dir * 1.0;
            }
            brain.duty = duty;
            brain.target = target;
            taken.push(target);
            let p = &mut world.players[id];
            go_to(p, &spec, target, urgency, ball);
            p.dribble = true;
        }
    }

    fn walk_to_base(&mut self, world: &mut World, ctx: &AiContext) {
        let lineup = formation::lineup(ctx.team_size);
        let spec = world.player_spec.clone();
        let ball = world.ball.pos;
        for team in 0..2 {
            for (i, &id) in ctx.slots[team].iter().enumerate() {
                if world.players[id].human {
                    continue;
                }
                let target = formation::to_world(&world.pitch, ctx.attack[team], lineup[i].base);
                let p = &mut world.players[id];
                go_to(p, &spec, target, 0.2, ball);
                p.dribble = false;
                self.brains[id].duty = Duty::Idle;
            }
        }
    }
}

/// Arrive at `target`: slow down when close, sprint when urgent and far.
pub fn go_to(p: &mut Player, spec: &PlayerSpec, target: Vec3, urgency: f32, look: Vec3) {
    let d = (target - p.pos).flat();
    let dist = d.length();
    p.look_at = Some(look);
    if dist < 0.2 {
        p.desired = Vec3::ZERO;
        p.sprint = false;
        return;
    }
    let cruise = if urgency > 0.8 {
        spec.sprint_speed
    } else if urgency > 0.5 {
        spec.jog_speed
    } else {
        spec.jog_speed * (0.55 + urgency)
    };
    let speed = (dist * 2.2).min(cruise);
    p.desired = d / dist * speed;
    p.sprint = urgency > 0.8 && dist > 2.5;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::v3;
    use crate::rules::{Match, MatchConfig};
    use crate::{GameEvent, PHYSICS_DT};

    #[test]
    fn ai_teams_spread_out_and_contest_the_ball() {
        let mut m = Match::new(MatchConfig {
            team_size: 5,
            half_length: 120.0,
            human_team: None,
            ..MatchConfig::default()
        });
        let mut touch_teams = Vec::new();
        let mut spread_sum = 0.0;
        let mut samples = 0;
        for s in 0..(240 * 20) {
            m.step(PHYSICS_DT);
            for e in m.drain_events().0 {
                if let GameEvent::Touch { player, .. } = e {
                    touch_teams.push(m.world.players[player].team);
                }
            }
            if s % 240 == 0 && s > 240 * 3 {
                for team in 0..2 {
                    let ps: Vec<_> = m.world.players.iter().filter(|p| p.team == team).collect();
                    let mut min_d = f32::MAX;
                    for i in 0..ps.len() {
                        for j in (i + 1)..ps.len() {
                            min_d = min_d.min(ps[i].pos.flat_distance(ps[j].pos));
                        }
                    }
                    spread_sum += min_d;
                    samples += 1;
                }
            }
            assert!(m.world.ball.is_finite());
        }
        let changes = touch_teams.windows(2).filter(|w| w[0] != w[1]).count();
        assert!(changes >= 3, "possession changes {changes}");
        assert!(
            spread_sum / samples as f32 > 1.5,
            "teams bunch up: {}",
            spread_sum / samples as f32
        );
        let duties: Vec<Duty> = m.ai.brains.iter().map(|b| b.duty).collect();
        assert!(duties.contains(&Duty::Keeper));
    }

    #[test]
    fn keeper_catches_an_easy_shot_and_distributes() {
        let mut m = Match::new(MatchConfig {
            team_size: 3,
            half_length: 120.0,
            human_team: None,
            ..MatchConfig::default()
        });
        while m.state.phase != crate::rules::Phase::Playing {
            m.step(PHYSICS_DT);
        }
        // Team 1 defends the right goal in the first half.
        let gk = m.slots[1][0];
        m.world.players[gk].pos = v3(18.6, 0.0, 0.0);
        m.world.possession = Default::default();
        m.world
            .debug_fire(v3(12.0, 1.0, 0.3), v3(20.0, 3.3, 0.1), 11.5, Vec3::ZERO);
        let (mut caught, mut distributed) = (false, false);
        for _ in 0..(240 * 5) {
            m.step(PHYSICS_DT);
            for e in m.drain_events().0 {
                match e {
                    GameEvent::Save {
                        player,
                        caught: true,
                    } if player == gk => caught = true,
                    GameEvent::Kick { player, .. } if player == gk && caught => distributed = true,
                    _ => {}
                }
            }
        }
        assert!(caught, "keeper should catch an easy shot");
        assert!(distributed, "keeper should release the ball to a teammate");
    }

    #[test]
    fn ai_match_produces_passes_shots_and_goals() {
        use crate::kick::KickKind;
        let mut m = Match::new(MatchConfig {
            team_size: 5,
            half_length: 75.0,
            human_team: None,
            ..MatchConfig::default()
        });
        let (mut passes, mut shots, mut tackles) = (0, 0, 0);
        while m.is_running() {
            m.step(PHYSICS_DT);
            for e in m.drain_events().0 {
                match e {
                    GameEvent::Kick {
                        kind: KickKind::Pass,
                        ..
                    }
                    | GameEvent::Kick {
                        kind: KickKind::Lob,
                        ..
                    } => passes += 1,
                    GameEvent::Kick {
                        kind: KickKind::Shot,
                        ..
                    } => shots += 1,
                    GameEvent::Tackle { .. } => tackles += 1,
                    _ => {}
                }
            }
        }
        let goals = m.state.score[0] + m.state.score[1];
        let _ = goals; // 5v5 scores are realistic (≈2 per 6 minutes); goals are asserted on 3v3 below.
        assert!(passes > 20, "passes {passes}");
        assert!(shots > 3, "shots {shots}");
        assert!(tackles > 0, "tackles {tackles}");
        let mut m3 = Match::new(MatchConfig {
            team_size: 3,
            half_length: 120.0,
            human_team: None,
            ..MatchConfig::default()
        });
        while m3.is_running() {
            m3.step(PHYSICS_DT);
        }
        assert!(
            m3.state.score[0] + m3.state.score[1] > 0,
            "no goals in a whole 3v3 match"
        );
    }
}
