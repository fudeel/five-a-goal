//! Goalkeeper mechanics: hand reach, dives, catches, parries and holding the ball.
//! The AI (ai::keeper) decides *when* to dive; this module decides what happens
//! physically.

use crate::ball::{Ball, BallSpec};
use crate::math::{clamp, closest_points_segments, v3, Rng, Vec3};
use crate::pitch::{Pitch, Side};
use crate::player::{Player, Role};

#[derive(Clone, Debug)]
pub struct KeeperSpec {
    /// Radius of the hands zone around the body when standing (m).
    pub reach_stand: f32,
    /// Radius of the hands zone along the body when diving (m).
    pub reach_dive: f32,
    /// Highest point the hands reach standing / jumping (m).
    pub max_hand_height: f32,
    /// Lateral speed of a full dive (m/s) and its duration (s).
    pub dive_speed: f32,
    pub dive_time: f32,
    /// Time on the ground after a dive (s).
    pub recover_time: f32,
    /// Relative ball speed below which a central ball is caught (m/s).
    pub catch_speed: f32,
    /// Holding time before distributing (s).
    pub hold_min: f32,
}

impl Default for KeeperSpec {
    fn default() -> Self {
        Self {
            reach_stand: 0.56,
            reach_dive: 0.38,
            max_hand_height: 2.35,
            dive_speed: 4.0,
            dive_time: 0.55,
            recover_time: 0.5,
            catch_speed: 15.0,
            hold_min: 1.1,
        }
    }
}

/// What the keeper did with the ball.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaveKind {
    Catch,
    Parry,
}

/// Starts a dive along `dir` (lateral, unit) reaching up to `height`.
pub fn start_dive(p: &mut Player, ks: &KeeperSpec, dir: Vec3, height: f32) {
    if p.dive_timer > 0.0 || p.stumble > 0.0 || p.holding_ball {
        return;
    }
    p.dive_dir = dir.flat().normalize_or(Vec3::Z);
    p.dive_height = clamp(height, 0.15, ks.max_hand_height);
    p.dive_timer = ks.dive_time;
}

/// Advances a dive (moves the body sideways) and handles recovery.
pub fn step_dive(p: &mut Player, ks: &KeeperSpec, dt: f32) {
    if p.dive_timer <= 0.0 {
        return;
    }
    let k = p.dive_timer / ks.dive_time; // 1 → 0
    let v = p.dive_dir * (ks.dive_speed * (0.4 + 0.9 * k));
    p.down = (1.0 - k).min(1.0);
    p.pos += v * dt;
    p.vel = v;
    p.desired = Vec3::ZERO;
    p.dive_timer -= dt;
    if p.dive_timer <= 0.0 {
        p.dive_timer = 0.0;
        p.stumble = ks.recover_time;
        p.vel = Vec3::ZERO;
        p.down = 1.0;
    }
}

/// The hands zone as a capsule (a, b, radius).
pub fn hands_zone(p: &Player, ks: &KeeperSpec) -> (Vec3, Vec3, f32) {
    if p.dive_timer > 0.0 {
        let a = p.pos + v3(0.0, 0.25, 0.0) - p.dive_dir * 0.3;
        let b = p.pos + p.dive_dir * 1.05 + v3(0.0, p.dive_height, 0.0);
        (a, b, ks.reach_dive)
    } else {
        let a = p.pos + v3(0.0, 0.2, 0.0) + p.forward() * 0.15;
        let b = p.pos + v3(0.0, ks.max_hand_height - ks.reach_stand, 0.0) + p.forward() * 0.15;
        (a, b, ks.reach_stand)
    }
}

/// Where the ball sits while held.
pub fn hold_point(p: &Player) -> Vec3 {
    p.pos + p.forward() * 0.32 + v3(0.0, 1.05, 0.0)
}

/// Tries a save on the ball's motion this step (`prev_pos → pos`). Returns the
/// save if the ball entered the hands zone.
#[allow(clippy::too_many_arguments)]
pub fn try_save(
    gk: &mut Player,
    ball: &mut Ball,
    bs: &BallSpec,
    ks: &KeeperSpec,
    pitch: &Pitch,
    defend: Side,
    rng: &mut Rng,
) -> Option<SaveKind> {
    if gk.role != Role::Goalkeeper
        || gk.holding_ball
        || gk.ball_ghost > 0.0
        || gk.save_cooldown > 0.0
    {
        return None;
    }
    if gk.stumble > 0.0 && gk.dive_timer <= 0.0 {
        return None; // on the ground after a dive
    }
    if !pitch.is_inside_penalty_area(defend, ball.pos)
        || !pitch.is_inside_penalty_area(defend, gk.pos)
    {
        return None;
    }
    let (a, b, r) = hands_zone(gk, ks);
    let (pb, pc, s, _) = closest_points_segments(ball.prev_pos, ball.pos, a, b);
    let reach = r + bs.radius;
    if pb.distance(pc) > reach {
        return None;
    }
    let contact = ball.prev_pos.lerp(ball.pos, s);
    let rel = ball.vel - gk.vel;
    let speed = rel.length();
    let diving = gk.dive_timer > 0.0;
    // Central balls (whose line passes close to the body) are much easier to hold.
    let dir = ball.vel.normalize_or(Vec3::X);
    let (q1, q2, _, _) = closest_points_segments(contact - dir * 0.5, contact + dir * 1.5, a, b);
    let off_axis = clamp(q1.distance(q2) / reach, 0.0, 1.0);
    let mut catch_limit = ks.catch_speed * (0.75 + 0.5 * gk.skill) * (1.0 - 0.45 * off_axis);
    if diving {
        catch_limit *= 0.6;
    }
    gk.save_cooldown = 0.35;
    if speed < catch_limit && rng.chance(0.92) {
        gk.holding_ball = true;
        gk.hold_timer = 0.0;
        ball.pos = hold_point(gk);
        ball.prev_pos = ball.pos;
        ball.vel = Vec3::ZERO;
        ball.spin = Vec3::ZERO;
        return Some(SaveKind::Catch);
    }
    // Parry: push the ball wide of the posts (towards the corner) or over the bar,
    // not back into the middle where the shooter follows up.
    let goal = pitch.goal_centre(defend);
    let away = (contact - goal)
        .flat()
        .normalize_or(v3(-defend.sign(), 0.0, 0.0));
    let side = if (contact.z - gk.pos.z).abs() > 0.05 {
        (contact.z - gk.pos.z).signum()
    } else if rng.chance(0.5) {
        1.0
    } else {
        -1.0
    };
    let wide = v3(-defend.sign() * 0.45, 0.0, side).normalize_or(away);
    let mut out = (wide + away * 0.25).normalize_or(wide) * (speed * rng.range(0.3, 0.5));
    if contact.y > 1.7 {
        out.y = out.y.abs() + 3.0; // tipped over
    }
    // Never parry it back into the own goal.
    if (out.x * -defend.sign()) < 1.0 {
        out.x = -defend.sign() * (out.x.abs().max(1.0));
    }
    ball.pos = pc + (contact - pc).normalize_or(away) * (reach + 0.01);
    ball.vel = out;
    ball.spin *= 0.3;
    ball.grounded = false;
    Some(SaveKind::Parry)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PHYSICS_DT;

    fn keeper() -> Player {
        let mut gk = Player::new(0, 1, Role::Goalkeeper, 1, v3(19.2, 0.0, 0.0));
        gk.facing = std::f32::consts::PI;
        gk
    }

    fn fly_at(
        gk: &mut Player,
        from: Vec3,
        target: Vec3,
        speed: f32,
        dive: Option<(Vec3, f32, f32)>,
    ) -> Option<SaveKind> {
        let pitch = Pitch::new();
        let bs = BallSpec::default();
        let ks = KeeperSpec::default();
        let mut rng = Rng::new(3);
        let mut ball = Ball::new(from);
        ball.vel = (target - from).normalize_or_zero() * speed;
        for i in 0..240 {
            if let Some((d, h, at)) = dive {
                if (i as f32 * PHYSICS_DT - at).abs() < PHYSICS_DT * 0.5 {
                    start_dive(gk, &ks, d, h);
                }
            }
            ball.integrate_flight(&bs, PHYSICS_DT);
            step_dive(gk, &ks, PHYSICS_DT);
            gk.save_cooldown = (gk.save_cooldown - PHYSICS_DT).max(0.0);
            if let Some(s) = try_save(gk, &mut ball, &bs, &ks, &pitch, Side::Right, &mut rng) {
                return Some(s);
            }
        }
        None
    }

    #[test]
    fn catches_an_easy_central_ball() {
        let mut gk = keeper();
        let r = fly_at(&mut gk, v3(12.0, 1.0, 0.0), v3(20.0, 3.4, 0.0), 11.0, None);
        assert_eq!(r, Some(SaveKind::Catch));
        assert!(gk.holding_ball);
    }

    #[test]
    fn diving_save_on_a_shot_to_the_side() {
        let mut gk = keeper();
        // Shot 1.3 m to the keeper's side; dive started immediately.
        let r = fly_at(
            &mut gk,
            v3(10.0, 0.6, 0.0),
            v3(20.0, 0.8, 1.3),
            22.0,
            Some((Vec3::Z, 0.8, 0.1)),
        );
        assert!(r.is_some(), "should reach it with a dive");
    }

    #[test]
    fn corner_shot_without_dive_beats_him() {
        let mut gk = keeper();
        let r = fly_at(&mut gk, v3(12.0, 0.5, 0.0), v3(20.0, 0.4, 1.35), 28.0, None);
        assert_eq!(r, None);
    }

    #[test]
    fn no_hands_outside_the_area() {
        let mut gk = keeper();
        gk.pos = v3(12.0, 0.0, 0.0);
        let r = fly_at(&mut gk, v3(6.0, 1.0, 0.0), v3(14.0, 3.4, 0.0), 10.0, None);
        assert_eq!(r, None);
    }

    #[test]
    fn hard_shot_at_the_keeper_is_parried_away_from_goal() {
        let pitch = Pitch::new();
        let bs = BallSpec::default();
        let ks = KeeperSpec::default();
        let mut rng = Rng::new(9);
        let mut gk = keeper();
        gk.skill = 0.3;
        let mut ball = Ball::new(v3(14.0, 0.9, 0.45));
        ball.vel = v3(30.0, 0.0, 0.0);
        let mut res = None;
        for _ in 0..120 {
            ball.integrate_flight(&bs, PHYSICS_DT);
            if let Some(s) = try_save(&mut gk, &mut ball, &bs, &ks, &pitch, Side::Right, &mut rng) {
                res = Some(s);
                break;
            }
        }
        assert_eq!(res, Some(SaveKind::Parry));
        assert!(ball.vel.x < 0.0, "parried back into play: {:?}", ball.vel);
    }
}
