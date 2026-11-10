//! On-ball decisions (shoot / pass / dribble / clear) and pressing with tackles.
//! Decisions are made every `think_interval` seconds (reaction time) and are
//! executed through the same kick / tackle requests the human uses.

use super::positioning::{lane_clearance, openness};
use super::AiTuning;
use crate::formation;
use crate::kick::{self, KickKind, KickRequest};
use crate::math::{clamp, v3, Rng, Vec3};
use crate::pitch::{Pitch, Side};
use crate::player::{Player, PlayerSpec, Role};

/// The chosen action for a ball carrier.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Action {
    Shoot {
        target: Vec3,
        power: f32,
        curl: f32,
    },
    Pass {
        to: usize,
        target: Vec3,
        lob: bool,
        power: f32,
    },
    Dribble {
        dir: Vec3,
        sprint: bool,
    },
    Clear {
        dir: Vec3,
    },
}

/// Angle (radians) the goal mouth subtends from `p`.
pub fn mouth_angle(pitch: &Pitch, attack: Side, p: Vec3) -> f32 {
    let gx = pitch.goal_line_x(attack);
    let hw = pitch.goal.width * 0.5;
    let a = (v3(gx, 0.0, -hw) - p).flat().normalize_or_zero();
    let b = (v3(gx, 0.0, hw) - p).flat().normalize_or_zero();
    a.dot(b).clamp(-1.0, 1.0).acos()
}

/// Rough probability that a shot from `p` scores, plus the best target.
pub fn shot_chance(pitch: &Pitch, attack: Side, p: Vec3, opponents: &[&Player]) -> (f32, Vec3) {
    let goal = pitch.goal_centre(attack);
    let d = p.flat_distance(goal);
    if d > 18.0 || attack.sign() * (goal.x - p.x) < 0.3 {
        return (0.0, goal);
    }
    let keeper = opponents
        .iter()
        .find(|o| o.role == Role::Goalkeeper && o.pos.flat_distance(goal) < 5.0);
    // Aim for the side away from the keeper (or the far post).
    let lateral = match keeper {
        Some(k) => {
            if k.pos.z > p.z * 0.15 {
                -0.82
            } else {
                0.82
            }
        }
        None => {
            if p.z > 0.0 {
                -0.6
            } else {
                0.6
            }
        }
    };
    let target = kick::goal_target(pitch, attack, lateral, 0.6);
    let dist_f = clamp(1.25 - d / 13.0, 0.0, 1.0);
    let angle_f = clamp(mouth_angle(pitch, attack, p) / 0.45, 0.0, 1.0);
    let lane = lane_clearance(
        p,
        target,
        &opponents
            .iter()
            .copied()
            .filter(|o| o.role != Role::Goalkeeper)
            .collect::<Vec<_>>(),
    );
    let lane_f = if lane < 0.5 {
        0.2
    } else if lane < 1.0 {
        0.6
    } else {
        1.0
    };
    let keeper_f = match keeper {
        Some(k) => {
            clamp((k.pos.z - target.z).abs() / 1.6, 0.35, 1.0)
                * if k.stumble > 0.0 { 1.5 } else { 1.0 }
        }
        None => 1.4,
    };
    (
        clamp(dist_f * angle_f * lane_f * keeper_f, 0.0, 1.0),
        target,
    )
}

/// Evaluates the carrier's options and picks the best one.
#[allow(clippy::too_many_arguments)]
pub fn choose(
    pitch: &Pitch,
    attack: Side,
    me: &Player,
    ball: Vec3,
    mates: &[&Player],
    opponents: &[&Player],
    tuning: &AiTuning,
    rng: &mut Rng,
) -> Action {
    let goal = pitch.goal_centre(attack);
    let my_norm = formation::to_norm(pitch, attack, ball);
    let pressure = openness(me.pos, opponents);

    // --- Shoot
    let (p_goal, target) = shot_chance(pitch, attack, ball, opponents);
    let dist = ball.flat_distance(goal);
    // Without a keeper (1v1, 2v2) every shot looks good: only take the clear ones.
    let has_keeper = opponents.iter().any(|o| o.role == Role::Goalkeeper);
    let (min_p, weight) = if has_keeper { (0.1, 2.4) } else { (0.3, 1.3) };
    let shoot_score = if p_goal < min_p {
        0.0
    } else {
        p_goal * weight * tuning.shoot_bias
    };
    let mut best = (
        shoot_score,
        Action::Shoot {
            target,
            power: clamp(0.5 + dist / 26.0, 0.5, 0.95),
            curl: if rng.chance(0.3) {
                -target.z.signum() * attack.sign() * 0.6
            } else {
                0.0
            },
        },
    );

    // --- Pass
    for m in mates {
        if m.id == me.id {
            continue;
        }
        let d = m.pos.flat_distance(ball);
        if !(2.5..24.0).contains(&d) {
            continue;
        }
        let lead = m.pos + m.vel.flat() * (d / 11.0);
        let clear = lane_clearance(ball, lead, opponents);
        let safe = clamp((clear - 0.6) / 2.0, 0.0, 1.0);
        let open = openness(lead, opponents).min(5.0) / 5.0;
        let progress = clamp(
            formation::to_norm(pitch, attack, lead).x - my_norm.x,
            -0.5,
            0.6,
        );
        let (their_shot, _) = shot_chance(pitch, attack, lead, opponents);
        let back_to_keeper = m.role == Role::Goalkeeper;
        let mut score = safe * (0.3 + 0.45 * open + 0.7 * progress + 0.8 * their_shot)
            - if d > 16.0 { 0.15 } else { 0.0 };
        if back_to_keeper {
            if open < 0.9 {
                continue;
            }
            score -= 0.4;
        }
        // Encourage passing when pressed.
        score += clamp((2.0 - pressure) * 0.12, 0.0, 0.2);
        let mut lob = false;
        if safe < 0.2 && open > 0.6 && d > 8.0 && !back_to_keeper {
            // Lane blocked but receiver free: lofted pass.
            score = 0.25 + 0.3 * open + 0.4 * progress;
            lob = true;
        }
        if score > best.0 {
            let power = clamp(0.25 + d / 40.0, 0.2, 0.7);
            best = (
                score,
                Action::Pass {
                    to: m.id,
                    target: lead,
                    lob,
                    power,
                },
            );
        }
    }

    // --- Dribble: towards goal, bending away from the nearest opponent.
    let to_goal = (goal - ball)
        .flat()
        .normalize_or(v3(attack.sign(), 0.0, 0.0));
    let nearest = opponents.iter().min_by(|a, b| {
        a.pos
            .flat_distance(ball)
            .partial_cmp(&b.pos.flat_distance(ball))
            .unwrap()
    });
    let mut dir = to_goal;
    if let Some(n) = nearest {
        let away = (ball - n.pos).flat();
        let ad = away.length();
        if ad < 4.0 {
            dir = (to_goal + away.normalize_or_zero() * ((4.0 - ad) / 4.0) * 1.2)
                .normalize_or(to_goal);
        }
    }
    // Stay inside the pitch.
    let ahead = ball + dir * 3.0;
    if ahead.z.abs() > pitch.half_width() - 1.0 {
        dir = (dir + v3(0.0, 0.0, -ahead.z.signum() * 0.8)).normalize_or(dir);
    }
    let space = openness(ball + dir * 3.0, opponents);
    let dribble_score = 0.18 + 0.06 * space.min(6.0) - clamp((1.6 - pressure) * 0.25, 0.0, 0.4);
    if dribble_score > best.0 {
        best = (
            dribble_score,
            Action::Dribble {
                dir,
                sprint: space > 5.0,
            },
        );
    }

    // --- Clear: under pressure deep in our own half, away from the presser
    // and towards the wing.
    if my_norm.x < -0.55 && pressure < 1.4 && best.0 < 0.35 {
        let mut dir =
            v3(attack.sign(), 0.0, 0.0) + v3(0.0, 0.0, if ball.z >= 0.0 { 0.7 } else { -0.7 });
        if let Some(n) = nearest {
            dir += (ball - n.pos).flat().normalize_or_zero() * 1.2;
        }
        let mut dir = dir.normalize_or(v3(attack.sign(), 0.0, 0.0));
        if dir.x * attack.sign() < 0.2 {
            dir = (dir + v3(attack.sign() * 0.6, 0.0, 0.0)).normalize_or(dir);
        }
        best = (0.35, Action::Clear { dir });
    }
    best.1
}

/// Turns an action into intents on the player.
pub fn execute(p: &mut Player, spec: &PlayerSpec, action: Action, mates: &[&Player]) {
    let _ = mates;
    match action {
        Action::Shoot {
            target,
            power,
            curl,
        } => {
            let dir = (target - p.pos).flat().normalize_or(p.forward());
            p.pending_kick = Some(
                KickRequest::new(KickKind::Shot, dir, power)
                    .with_target(target)
                    .with_curl(curl),
            );
            p.desired = dir * spec.jog_speed * 0.6;
            p.sprint = false;
        }
        Action::Pass {
            target, lob, power, ..
        } => {
            let dir = (target - p.pos).flat().normalize_or(p.forward());
            let kind = if lob { KickKind::Lob } else { KickKind::Pass };
            p.pending_kick = Some(KickRequest::new(kind, dir, power).with_target(target));
            p.desired = dir * spec.jog_speed * 0.5;
            p.sprint = false;
        }
        Action::Dribble { dir, sprint } => {
            p.desired = dir
                * if sprint {
                    spec.sprint_speed
                } else {
                    spec.jog_speed
                };
            p.sprint = sprint;
        }
        Action::Clear { dir } => {
            p.pending_kick = Some(KickRequest::new(KickKind::Lob, dir, 0.9));
            p.desired = dir * spec.jog_speed * 0.5;
        }
    }
}

/// Pressing the ball carrier: get goal-side, close down, tackle when the ball is
/// exposed. Returns true if a tackle was requested.
pub fn press(
    p: &mut Player,
    spec: &PlayerSpec,
    carrier: &Player,
    ball: Vec3,
    own_goal: Vec3,
    rng: &mut Rng,
    aggression: f32,
) -> bool {
    let to_goal = (own_goal - carrier.pos).flat().normalize_or_zero();
    let d = p.pos.flat_distance(carrier.pos);
    let goal_side = carrier.pos + to_goal * 0.85;
    let target = if d > 2.2 {
        goal_side
    } else {
        ball.lerp(goal_side, 0.35)
    };
    super::go_to(p, spec, target, if d > 3.0 { 1.0 } else { 0.7 }, ball);
    let ball_d = p.pos.flat_distance(ball);
    let exposed = carrier.pos.flat_distance(ball) > 0.5;
    let in_front = (ball - p.pos).flat().normalize_or_zero().dot(p.forward()) > 0.3;
    if ball_d < 1.05 && in_front && p.tackle_cooldown <= 0.0 && p.stumble <= 0.0 {
        let chance = if exposed { 0.55 } else { 0.12 } * aggression;
        if rng.chance(chance) {
            p.tackle_request = true;
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shot_chance_falls_with_distance_and_angle() {
        let pitch = Pitch::new();
        let none: Vec<&Player> = Vec::new();
        let (near, _) = shot_chance(&pitch, Side::Right, v3(15.0, 0.0, 0.0), &none);
        let (far, _) = shot_chance(&pitch, Side::Right, v3(5.0, 0.0, 0.0), &none);
        let (wide, _) = shot_chance(&pitch, Side::Right, v3(18.5, 0.0, 8.0), &none);
        assert!(
            near > far && near > wide,
            "near {near} far {far} wide {wide}"
        );
        let (behind, _) = shot_chance(&pitch, Side::Right, v3(21.0, 0.0, 3.0), &none);
        assert_eq!(behind, 0.0);
    }

    #[test]
    fn carrier_shoots_when_clear_in_front_of_goal_and_passes_when_blocked() {
        let pitch = Pitch::new();
        let tuning = AiTuning::for_difficulty(crate::rules::Difficulty::Normal);
        let mut rng = Rng::new(1);
        let me = Player::new(0, 0, Role::Pivot, 9, v3(14.0, 0.0, 0.5));
        let mate = Player::new(1, 0, Role::Winger, 7, v3(8.0, 0.0, 6.0));
        let gk = Player::new(2, 1, Role::Goalkeeper, 1, v3(19.0, 0.0, 1.2));
        let a = choose(
            &pitch,
            Side::Right,
            &me,
            v3(14.4, 0.11, 0.5),
            &[&me, &mate],
            &[&gk],
            &tuning,
            &mut rng,
        );
        assert!(matches!(a, Action::Shoot { .. }), "{a:?}");
        if let Action::Shoot { target, .. } = a {
            assert!(target.z < 0.0, "should aim away from the keeper");
        }
        // Far from goal, surrounded, with an open teammate ahead: pass.
        let me2 = Player::new(0, 0, Role::Defender, 4, v3(-8.0, 0.0, 0.0));
        let mate2 = Player::new(1, 0, Role::Pivot, 9, v3(2.0, 0.0, -5.0));
        let o1 = Player::new(3, 1, Role::Pivot, 9, v3(-7.0, 0.0, 0.6));
        let o2 = Player::new(4, 1, Role::Winger, 7, v3(-6.8, 0.0, -0.4));
        let a2 = choose(
            &pitch,
            Side::Right,
            &me2,
            v3(-7.6, 0.11, 0.0),
            &[&me2, &mate2],
            &[&o1, &o2, &gk],
            &tuning,
            &mut rng,
        );
        assert!(matches!(a2, Action::Pass { to: 1, .. }), "{a2:?}");
    }
}
