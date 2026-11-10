//! Goalkeeper AI: reads shots from the predicted trajectory, steps across or
//! dives, claims loose balls in the area and distributes after a catch.

use super::positioning::{keeper_spot, lane_clearance, openness};
use super::predict::{intercept, Prediction};
use super::{go_to, AiTuning};
use crate::keeper;
use crate::kick::{KickKind, KickRequest};
use crate::math::{v3, Rng, Vec3};
use crate::pitch::Side;
use crate::player::{Player, Role};
use crate::world::World;

/// Where and when a predicted ball passes the keeper's depth (if it is a shot).
pub fn incoming_shot(
    world: &World,
    gk: &Player,
    defend: Side,
    pred: &Prediction,
) -> Option<(f32, Vec3)> {
    let into_field = -defend.sign();
    // Moving towards our goal fast enough to be a shot.
    if world.ball.vel.x * defend.sign() < 4.0 {
        return None;
    }
    let depth = |p: Vec3| (p.x - gk.pos.x) * into_field;
    for i in 1..pred.points.len() {
        let (a, b) = (pred.points[i - 1], pred.points[i]);
        if depth(a) > 0.0 && depth(b) <= 0.0 {
            let t = (i - 1) as f32 * pred.step + pred.step * depth(a) / (depth(a) - depth(b));
            let p = a.lerp(b, depth(a) / (depth(a) - depth(b)));
            let half = world.pitch.goal.width * 0.5 + 0.7;
            if p.z.abs() < half && p.y < 2.7 {
                return Some((t, p));
            }
            return None;
        }
    }
    None
}

/// Writes the keeper's intents.
#[allow(clippy::too_many_arguments)]
pub fn think(
    world: &mut World,
    id: usize,
    defend: Side,
    attack: Side,
    pred: &Prediction,
    tuning: &AiTuning,
    rng: &mut Rng,
) {
    let ks = world.keeper_spec.clone();
    let spec = world.player_spec.clone();
    let ball = world.ball.pos;
    let gk = world.players[id].clone();

    if gk.holding_ball {
        let p = &mut world.players[id];
        p.desired = Vec3::ZERO;
        p.look_at = Some(world.pitch.goal_centre(attack));
        if gk.hold_timer > ks.hold_min + rng.range(0.0, 0.6) {
            distribute(world, id, attack);
        }
        return;
    }
    if gk.dive_timer > 0.0 || gk.stumble > 0.0 {
        return;
    }
    // Shot coming?
    if let Some((t, p)) = incoming_shot(world, &gk, defend, pred) {
        let dz = p.z - gk.pos.z;
        let can_step = dz.abs() <= 0.35 + 4.5 * (t - tuning.reaction).max(0.0);
        let me = &mut world.players[id];
        if dz.abs() < ks.reach_stand * 0.7 || can_step {
            go_to(me, &spec, v3(gk.pos.x, 0.0, p.z), 1.0, ball);
        } else if t < ks.dive_time * 0.95 + 0.12 - tuning.reaction * 0.35 && dz.abs() < 2.8 {
            keeper::start_dive(me, &ks, v3(0.0, 0.0, dz.signum()), p.y);
        } else {
            go_to(me, &spec, v3(gk.pos.x, 0.0, p.z), 1.0, ball);
        }
        return;
    }
    // Loose ball in the area: come and claim it if we get there first.
    let loose = world.possession.owner.is_none()
        || world.possession.owner_team(&world.players) == Some(gk.team);
    if loose && world.pitch.is_inside_penalty_area(defend, ball) && world.ball.speed() < 9.0 {
        let (t_me, pt) = intercept(&gk, &spec, pred, &world.ball_spec, tuning.reaction);
        let t_opp = world
            .players
            .iter()
            .filter(|o| o.team != gk.team)
            .map(|o| intercept(o, &spec, pred, &world.ball_spec, 0.15).0)
            .fold(f32::MAX, f32::min);
        if t_me < t_opp - 0.1 && world.pitch.is_inside_penalty_area(defend, pt) {
            go_to(&mut world.players[id], &spec, pt, 1.0, ball);
            return;
        }
    }
    let spot = keeper_spot(&world.pitch, defend, ball);
    go_to(&mut world.players[id], &spec, spot, 0.75, ball);
}

/// Throws / rolls / kicks the ball to the most open teammate.
fn distribute(world: &mut World, id: usize, attack: Side) {
    let gk = world.players[id].clone();
    let opponents: Vec<&Player> = world.players.iter().filter(|o| o.team != gk.team).collect();
    let mut best: Option<(f32, Vec3)> = None;
    for m in world
        .players
        .iter()
        .filter(|m| m.team == gk.team && m.id != id && m.role != Role::Goalkeeper)
    {
        let d = m.pos.flat_distance(gk.pos);
        let clear = lane_clearance(gk.pos, m.pos, &opponents);
        let open = openness(m.pos, &opponents);
        let progress = (m.pos.x - gk.pos.x) * attack.sign();
        if open < 3.5 || clear < 1.5 {
            continue; // never give it to a pressed teammate near our goal
        }
        let score =
            clear.min(3.0) + open.min(6.0) * 0.8 + progress * 0.06 - (d - 14.0).max(0.0) * 0.1;
        if best.map(|b| score > b.0).unwrap_or(true) {
            best = Some((score, m.pos + m.vel.flat() * (d / 10.0)));
        }
    }
    if gk.busy_kicking() {
        return;
    }
    let p = &world.players[id];
    let (req, face) = match best {
        Some((_, target)) => {
            let dir = (target - p.pos)
                .flat()
                .normalize_or(v3(attack.sign(), 0.0, 0.0));
            let d = target.flat_distance(p.pos);
            // Roll it short, throw it to midfield, kick it long.
            let kind = if d < 13.0 {
                KickKind::Roll
            } else if d < 26.0 {
                KickKind::Throw
            } else {
                KickKind::Lob
            };
            (KickRequest::new(kind, dir, 0.45).with_target(target), dir)
        }
        None => {
            // Nobody safe: launch it long towards a wing.
            let wing = if gk.pos.z >= 0.0 { 0.5 } else { -0.5 };
            let dir = v3(attack.sign(), 0.0, wing).normalize_or(v3(attack.sign(), 0.0, 0.0));
            (KickRequest::new(KickKind::Lob, dir, 0.85), dir)
        }
    };
    if !req.kind.from_hands() {
        world.release_ball(id);
    }
    let p = &mut world.players[id];
    p.facing = face.heading();
    p.pending_kick = Some(req);
}
