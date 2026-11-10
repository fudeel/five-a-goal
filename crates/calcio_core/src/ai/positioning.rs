//! Off-the-ball positioning: formation shape, support spots, marking, keeper line.

use crate::formation;
use crate::math::{clamp, closest_point_on_segment, v3, Vec3};
use crate::pitch::{Pitch, Side};
use crate::player::{Player, Role};

/// What the team is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TeamPhase {
    Attack,
    Defend,
    Loose,
}

/// Formation position (world) of a slot for the current ball position and phase.
pub fn formation_target(
    pitch: &Pitch,
    attack: Side,
    base: Vec3,
    role: Role,
    ball: Vec3,
    phase: TeamPhase,
) -> Vec3 {
    let b = formation::to_norm(pitch, attack, ball);
    let (bx, bz) = (clamp(b.x, -1.0, 1.0), clamp(b.z, -1.1, 1.1));
    let (mut x, mut z) = match phase {
        TeamPhase::Attack => (base.x * 0.55 + 0.45 + bx * 0.35, base.z + bz * 0.25),
        TeamPhase::Defend => (base.x * 0.8 - 0.05 + bx * 0.45, base.z * 0.7 + bz * 0.45),
        TeamPhase::Loose => (base.x * 0.7 + 0.15 + bx * 0.4, base.z * 0.85 + bz * 0.35),
    };
    if role == Role::Defender {
        // The last man stays behind the ball and never too high.
        x = x.min(bx - 0.12).min(0.35);
    }
    x = clamp(x, -0.88, 0.85);
    z = clamp(z, -1.05, 1.05);
    formation::to_world(pitch, attack, v3(x, 0.0, z))
}

/// Minimum distance from any opponent to point `p`.
pub fn openness(p: Vec3, opponents: &[&Player]) -> f32 {
    opponents
        .iter()
        .map(|o| o.pos.flat_distance(p))
        .fold(f32::MAX, f32::min)
}

/// Minimum distance from any opponent to the segment a→b (a pass lane).
pub fn lane_clearance(a: Vec3, b: Vec3, opponents: &[&Player]) -> f32 {
    let (a, b) = (a.flat(), b.flat());
    opponents
        .iter()
        .map(|o| {
            let (q, t) = closest_point_on_segment(o.pos.flat(), a, b);
            // Opponents right next to the passer don't block as much as the lane centre.
            let d = q.distance(o.pos.flat());
            if t < 0.05 {
                d + 0.5
            } else {
                d
            }
        })
        .fold(f32::MAX, f32::min)
}

/// Picks a support spot near the formation target that is open and passable.
#[allow(clippy::too_many_arguments)]
pub fn support_spot(
    pitch: &Pitch,
    attack: Side,
    me: &Player,
    home: Vec3,
    carrier: Option<Vec3>,
    opponents: &[&Player],
    taken: &[Vec3],
) -> Vec3 {
    let mut best = (f32::MIN, home);
    let lim_x = pitch.half_length() - 1.0;
    let lim_z = pitch.half_width() - 0.8;
    let mut candidates = vec![home, me.pos];
    for k in 0..10 {
        let a = k as f32 / 10.0 * std::f32::consts::TAU;
        candidates.push(home + Vec3::from_heading(a) * 3.5);
    }
    for c in candidates {
        let c = v3(clamp(c.x, -lim_x, lim_x), 0.0, clamp(c.z, -lim_z, lim_z));
        let open = openness(c, opponents).min(6.0) / 6.0;
        let (lane, dist_term) = match carrier {
            Some(cp) => {
                let d = cp.flat_distance(c);
                let pen = if d < 5.0 {
                    (5.0 - d) * 0.25
                } else if d > 14.0 {
                    (d - 14.0) * 0.15
                } else {
                    0.0
                };
                (lane_clearance(cp, c, opponents).min(3.0) / 3.0, -pen)
            }
            None => (1.0, 0.0),
        };
        let progress = formation::to_norm(pitch, attack, c).x * 0.5;
        let zone = -0.08 * c.flat_distance(home);
        let crowd: f32 = taken
            .iter()
            .map(|t| (3.0 - t.flat_distance(c)).max(0.0) * 0.3)
            .sum();
        let score = 1.2 * open + 1.5 * lane + progress + zone + dist_term - crowd;
        if score > best.0 {
            best = (score, c);
        }
    }
    best.1
}

/// Goal-side marking position against `opp`.
pub fn mark_spot(pitch: &Pitch, defend: Side, opp: &Player, tight: f32) -> Vec3 {
    let goal = pitch.goal_centre(defend);
    let to_goal = (goal - opp.pos).flat();
    let d = to_goal.length();
    opp.pos + to_goal.normalize_or_zero() * tight.min(d * 0.5)
}

/// Keeper position: on the line from goal centre to the ball, coming out as the
/// ball gets closer to narrow the angle.
pub fn keeper_spot(pitch: &Pitch, defend: Side, ball: Vec3) -> Vec3 {
    let g = pitch.goal_centre(defend);
    let to_ball = (ball - g).flat();
    let dist = to_ball.length();
    let mut dir = to_ball.normalize_or(v3(-defend.sign(), 0.0, 0.0));
    // Never face backwards into the goal.
    if dir.x * -defend.sign() < 0.25 {
        dir = v3(
            -defend.sign() * 0.25,
            0.0,
            dir.z.signum() * (1.0f32 - 0.0625).sqrt(),
        );
    }
    let out = clamp(0.7 + 0.1 * dist, 0.7, 3.0);
    let mut p = g + dir * out;
    p.z = clamp(p.z, -1.8, 1.8);
    // Stay in front of the goal frame.
    let min_in = 0.42;
    if (pitch.half_length() - defend.sign() * p.x) < min_in {
        p.x = defend.sign() * (pitch.half_length() - min_in);
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formation_shifts_with_ball_and_phase() {
        let p = Pitch::new();
        let base = v3(-0.15, 0.0, 0.0);
        let atk = formation_target(
            &p,
            Side::Right,
            base,
            Role::Pivot,
            v3(5.0, 0.0, 0.0),
            TeamPhase::Attack,
        );
        let def = formation_target(
            &p,
            Side::Right,
            base,
            Role::Pivot,
            v3(5.0, 0.0, 0.0),
            TeamPhase::Defend,
        );
        assert!(atk.x > def.x + 3.0);
        let left = formation_target(
            &p,
            Side::Right,
            base,
            Role::Pivot,
            v3(0.0, 0.0, -8.0),
            TeamPhase::Defend,
        );
        let right = formation_target(
            &p,
            Side::Right,
            base,
            Role::Pivot,
            v3(0.0, 0.0, 8.0),
            TeamPhase::Defend,
        );
        assert!(right.z > left.z + 3.0);
        // mirrored for the other side
        let m = formation_target(
            &p,
            Side::Left,
            base,
            Role::Pivot,
            v3(-5.0, 0.0, 0.0),
            TeamPhase::Attack,
        );
        assert!((m.x + atk.x).abs() < 1e-4);
    }

    #[test]
    fn keeper_stays_in_front_of_goal_and_narrows_angle() {
        let p = Pitch::new();
        let near = keeper_spot(&p, Side::Left, v3(-16.0, 0.0, 3.0));
        let far = keeper_spot(&p, Side::Left, v3(5.0, 0.0, 0.0));
        assert!(near.x > -20.0 && far.x > -20.0);
        assert!(near.z > 0.0);
        let gd = |q: Vec3| q.flat_distance(p.goal_centre(Side::Left));
        assert!(gd(far) > gd(near) || gd(far) > 2.0);
        // ball behind the goal line → keeper still in front
        let behind = keeper_spot(&p, Side::Left, v3(-21.0, 0.0, 4.0));
        assert!(behind.x > -20.0 + 0.4);
    }

    #[test]
    fn lane_clearance_detects_blockers() {
        let blocker = Player::new(0, 1, Role::Defender, 4, v3(5.0, 0.0, 0.2));
        let opp = [&blocker];
        assert!(lane_clearance(Vec3::ZERO, v3(10.0, 0.0, 0.0), &opp) < 0.3);
        assert!(lane_clearance(Vec3::ZERO, v3(0.0, 0.0, 10.0), &opp) > 4.0);
    }
}
