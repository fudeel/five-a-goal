//! Kicks: ground passes, shots, lobs/chips, with curl. Used identically by the
//! human player and the AI.
//!
//! Launch parameters are found by running the *real* ball simulation forward
//! (drag, Magnus, bounces, rolling) and searching for the speed / elevation /
//! yaw that hits the target — so passes arrive at a controllable pace and curled
//! shots are aimed outside the post to bend back in.

use crate::ball::{Ball, BallSpec};
use crate::contact::{self, Turf};
use crate::math::{clamp, v3, Rng, Vec3};
use crate::player::Player;
use crate::PHYSICS_DT;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KickKind {
    /// Along the ground to a teammate or in a direction.
    Pass,
    /// Driven shot at a point in the goal.
    Shot,
    /// High ball with backspin (lofted pass, chip).
    Lob,
}

/// A request to kick, produced by the human input or the AI.
#[derive(Clone, Copy, Debug)]
pub struct KickRequest {
    pub kind: KickKind,
    /// Aim direction on the ground (unit).
    pub dir: Vec3,
    /// Exact target point (receiver, landing spot, point in the goal), if known.
    pub target: Option<Vec3>,
    /// 0..1 (how hard / how far).
    pub power: f32,
    /// -1..1: positive curls to the left of the travel direction.
    pub curl: f32,
    /// Seconds this request has been waiting for the ball to come into reach.
    pub age: f32,
}

impl KickRequest {
    pub fn new(kind: KickKind, dir: Vec3, power: f32) -> Self {
        Self {
            kind,
            dir: dir.flat().normalize_or(Vec3::X),
            target: None,
            power: clamp(power, 0.0, 1.0),
            curl: 0.0,
            age: 0.0,
        }
    }
    pub fn with_target(mut self, t: Vec3) -> Self {
        self.target = Some(t);
        self
    }
    pub fn with_curl(mut self, c: f32) -> Self {
        self.curl = clamp(c, -1.0, 1.0);
        self
    }
}

#[derive(Clone, Debug)]
pub struct KickSpec {
    /// Max distance from the feet to kick the ball.
    pub reach: f32,
    /// Max height of the bottom of the ball for a kick (volleys included).
    pub max_height: f32,
    /// How long a request waits for the ball before being dropped (s).
    pub patience: f32,
    /// Pass arrival speed range (soft..firm) at the receiver (m/s).
    pub pass_arrive_soft: f32,
    pub pass_arrive_firm: f32,
    /// Shot speed range (m/s).
    pub shot_min: f32,
    pub shot_max: f32,
    /// Lob launch elevation (degrees).
    pub lob_elevation: f32,
    /// Side spin at full curl (rad/s).
    pub curl_spin: f32,
    /// Global accuracy error multiplier (0 = perfect).
    pub error_scale: f32,
    /// Half-angle of the auto-target cone for passes (radians).
    pub assist_cone: f32,
}

impl Default for KickSpec {
    fn default() -> Self {
        Self {
            reach: 0.9,
            max_height: 0.75,
            patience: 0.45,
            pass_arrive_soft: 2.5,
            pass_arrive_firm: 7.5,
            shot_min: 14.0,
            shot_max: 31.0,
            lob_elevation: 40.0,
            curl_spin: 55.0,
            error_scale: 1.0,
            assist_cone: 0.55,
        }
    }
}

/// Is the ball where `p` can kick it?
pub fn can_kick(p: &Player, ball: &Ball, bs: &BallSpec, ks: &KickSpec) -> bool {
    let to = (ball.pos - p.pos).flat();
    let d = to.length();
    d < ks.reach
        && ball.height(bs) < ks.max_height
        && (d < 0.5 || to.normalize_or_zero().dot(p.forward()) > -0.2)
}

/// Backspin axis for travel direction `dir` (topspin is the opposite).
#[inline]
fn backspin_axis(dir: Vec3) -> Vec3 {
    dir.flat().normalize_or(Vec3::X).cross(Vec3::Y)
}

/// Runs the ball physics (no players/nets) for up to `max_t` seconds, calling
/// `f(ball, t)` every step; stops when `f` returns true.
pub fn simulate(
    ball: &Ball,
    bs: &BallSpec,
    turf: &Turf,
    max_t: f32,
    mut f: impl FnMut(&Ball, f32) -> bool,
) -> Ball {
    let mut b = ball.clone();
    let mut t = 0.0;
    while t < max_t {
        b.integrate_flight(bs, PHYSICS_DT);
        contact::ground(&mut b, bs, turf, PHYSICS_DT);
        t += PHYSICS_DT;
        if f(&b, t) {
            break;
        }
    }
    b
}

/// Ball state launched from `from` with velocity and spin.
fn launched(from: Vec3, vel: Vec3, spin: Vec3) -> Ball {
    let mut b = Ball::new(from);
    b.vel = vel;
    b.spin = spin;
    b.grounded = vel.y.abs() < 0.05;
    b
}

/// Speed of a ground ball when it has travelled `dist` (0 if it stops first).
fn roll_speed_at(
    from: Vec3,
    dir: Vec3,
    v0: f32,
    spin_y: f32,
    dist: f32,
    bs: &BallSpec,
    turf: &Turf,
) -> f32 {
    let spin = Vec3::Y.cross(dir) * (v0 / bs.radius) + v3(0.0, spin_y, 0.0);
    let b0 = launched(from, dir * v0, spin);
    let mut out = 0.0;
    simulate(&b0, bs, turf, 6.0, |b, _| {
        if b.pos.flat_distance(from) >= dist {
            out = b.vel.flat_length();
            return true;
        }
        b.vel.flat_length() < 0.05
    });
    out
}

/// Initial speed so a ground pass arrives at `dist` with `arrive` m/s.
pub fn solve_pass_speed(
    from: Vec3,
    dir: Vec3,
    dist: f32,
    arrive: f32,
    bs: &BallSpec,
    turf: &Turf,
) -> f32 {
    let (mut lo, mut hi) = (0.5f32, 32.0f32);
    for _ in 0..16 {
        let mid = 0.5 * (lo + hi);
        if roll_speed_at(from, dir, mid, 0.0, dist, bs, turf) < arrive {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

/// Height and lateral offset (positive = to the right of `dir`) of a launched ball when it
/// has travelled `dist` along `dir`. `None` if it never gets that far.
fn flight_at(
    from: Vec3,
    vel: Vec3,
    spin: Vec3,
    dir: Vec3,
    dist: f32,
    bs: &BallSpec,
    turf: &Turf,
) -> Option<(f32, f32)> {
    let b0 = launched(from, vel, spin);
    let right = dir.cross(Vec3::Y).normalize_or(Vec3::Z);
    let mut res = None;
    let mut prev = from;
    simulate(&b0, bs, turf, 4.0, |b, _| {
        let along = (b.pos - from).dot(dir);
        if along >= dist {
            let pa = (prev - from).dot(dir);
            let t = if along > pa {
                (dist - pa) / (along - pa)
            } else {
                1.0
            };
            let p = prev.lerp(b.pos, clamp(t, 0.0, 1.0));
            res = Some((p.y, (p - from).dot(right)));
            return true;
        }
        prev = b.pos;
        b.vel.flat_length() < 0.3
    });
    res
}

/// Solves elevation and yaw so a ball at `speed` with the given spin passes
/// through `target`. Returns the launch velocity.
fn solve_shot(
    from: Vec3,
    target: Vec3,
    speed: f32,
    topspin: f32,
    side_spin: f32,
    bs: &BallSpec,
    turf: &Turf,
) -> (Vec3, Vec3) {
    let base_dir = (target - from).flat().normalize_or(Vec3::X);
    let dist = (target - from).flat_length();
    let right0 = base_dir.cross(Vec3::Y).normalize_or(Vec3::Z);
    let mut yaw = 0.0f32;
    let mut vel = base_dir * speed;
    let mut spin = Vec3::ZERO;
    for _ in 0..4 {
        let dir = base_dir.rotate_y(-yaw);
        spin = Vec3::Y.cross(dir) * topspin + v3(0.0, side_spin, 0.0);
        // Elevation search.
        let (mut lo, mut hi) = (-0.08f32, 0.7f32);
        for _ in 0..14 {
            let el = 0.5 * (lo + hi);
            let v = dir * (speed * el.cos()) + v3(0.0, speed * el.sin(), 0.0);
            let h = flight_at(from, v, spin, base_dir, dist, bs, turf)
                .map(|r| r.0)
                .unwrap_or(-1.0);
            if h < target.y {
                lo = el;
            } else {
                hi = el;
            }
        }
        let el = 0.5 * (lo + hi);
        vel = dir * (speed * el.cos()) + v3(0.0, speed * el.sin(), 0.0);
        // Lateral miss correction (curl compensation).
        if let Some((_, lat)) = flight_at(from, vel, spin, base_dir, dist, bs, turf) {
            let want = (target - from).dot(right0);
            let miss = lat - want;
            if miss.abs() < 0.02 {
                break;
            }
            yaw += miss / dist.max(1.0);
        } else {
            break;
        }
    }
    (vel, spin)
}

/// Speed for a lob at `elevation` landing at `dist` (first bounce).
fn solve_lob_speed(
    from: Vec3,
    dir: Vec3,
    dist: f32,
    elevation: f32,
    backspin: f32,
    bs: &BallSpec,
    turf: &Turf,
) -> f32 {
    let spin = backspin_axis(dir) * backspin;
    let landing = |s: f32| {
        let v = dir * (s * elevation.cos()) + v3(0.0, s * elevation.sin(), 0.0);
        let b0 = launched(from, v, spin);
        let mut up = false;
        let mut land = 0.0;
        simulate(&b0, bs, turf, 6.0, |b, _| {
            if b.pos.y > bs.radius + 0.05 {
                up = true;
            }
            if up && b.pos.y <= bs.radius + 1e-3 {
                land = b.pos.flat_distance(from);
                return true;
            }
            false
        });
        land
    };
    let (mut lo, mut hi) = (2.0f32, 32.0f32);
    for _ in 0..16 {
        let mid = 0.5 * (lo + hi);
        if landing(mid) < dist {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

/// A point inside the goal mouth of `side`: `lateral` in -1..1 across the mouth
/// (−1 = negative z post), `height` above the ground.
pub fn goal_target(
    pitch: &crate::pitch::Pitch,
    side: crate::pitch::Side,
    lateral: f32,
    height: f32,
) -> Vec3 {
    let half = pitch.goal.width * 0.5 - 0.22;
    v3(
        pitch.goal_line_x(side) + side.sign() * 0.3,
        clamp(height, 0.15, pitch.goal.height - 0.15),
        clamp(lateral, -1.0, 1.0) * half,
    )
}

/// Picks the teammate best aligned with `dir` and returns where to pass to
/// (leading a moving receiver).
pub fn auto_target(kicker: &Player, mates: &[&Player], dir: Vec3, cone: f32) -> Option<Vec3> {
    let mut best: Option<(f32, Vec3)> = None;
    for m in mates {
        if m.id == kicker.id {
            continue;
        }
        let to = (m.pos - kicker.pos).flat();
        let d = to.length();
        if !(2.0..30.0).contains(&d) {
            continue;
        }
        let ang = to.normalize_or_zero().dot(dir).clamp(-1.0, 1.0).acos();
        if ang > cone {
            continue;
        }
        let score = ang * 2.0 + d * 0.03;
        if best.map(|b| score < b.0).unwrap_or(true) {
            // Lead the receiver by his velocity over the (rough) travel time.
            let t = d / 11.0;
            let lead = m.vel.flat() * t;
            best = Some((score, m.pos + lead));
        }
    }
    best.map(|b| b.1)
}

/// Result of planning a kick.
#[derive(Clone, Copy, Debug)]
pub struct KickPlan {
    pub vel: Vec3,
    pub spin: Vec3,
    /// Where the kick was aimed.
    pub aim: Vec3,
}

/// Computes the ball's launch for `req` kicked by `kicker` (with accuracy error).
#[allow(clippy::too_many_arguments)]
pub fn plan_kick(
    req: &KickRequest,
    kicker: &Player,
    mates: &[&Player],
    ball: &Ball,
    bs: &BallSpec,
    turf: &Turf,
    ks: &KickSpec,
    rng: &mut Rng,
) -> KickPlan {
    let from = v3(ball.pos.x, bs.radius, ball.pos.z);
    let dir = req.dir.flat().normalize_or(kicker.forward());
    let power = clamp(req.power, 0.0, 1.0);
    let curl_w = req.curl * ks.curl_spin;
    // Accuracy: worse with power, running speed, turning against the body and volleys.
    let facing = kicker.forward().dot(dir).clamp(-1.0, 1.0);
    let volley = (ball.height(bs) - 0.15).max(0.0);
    let base_err = ks.error_scale * (1.45 - kicker.skill).max(0.15);
    let speed_term = 0.004 * kicker.speed();
    let turn_term = 0.035 * (1.0 - facing);
    let (vel, spin, aim, yaw_sigma, pitch_sigma) = match req.kind {
        KickKind::Pass => {
            let target = req
                .target
                .or_else(|| auto_target(kicker, mates, dir, ks.assist_cone));
            let (dir, dist) = match target {
                Some(t) => (
                    (t - from).flat().normalize_or(dir),
                    (t - from).flat_length(),
                ),
                None => (dir, 6.0 + 22.0 * power),
            };
            let arrive = ks.pass_arrive_soft + (ks.pass_arrive_firm - ks.pass_arrive_soft) * power;
            let speed = solve_pass_speed(from, dir, dist, arrive, bs, turf);
            let spin = Vec3::Y.cross(dir) * (speed / bs.radius) + v3(0.0, curl_w * 0.4, 0.0);
            let sigma = 0.012 + 0.015 * power + speed_term + turn_term + 0.05 * volley;
            (dir * speed, spin, from + dir * dist, sigma, 0.0)
        }
        KickKind::Shot => {
            let target = req.target.unwrap_or(from + dir * 20.0 + v3(0.0, 0.8, 0.0));
            let speed = (ks.shot_min + (ks.shot_max - ks.shot_min) * power + 0.25 * kicker.speed())
                .min(ks.shot_max + 2.0);
            let topspin = 8.0 + 6.0 * power;
            let (vel, spin) = solve_shot(from, target, speed, topspin, curl_w, bs, turf);
            let sigma = 0.012 + 0.06 * power * power + speed_term + turn_term + 0.06 * volley;
            (vel, spin, target, sigma, sigma * 0.8)
        }
        KickKind::Lob => {
            let target = req.target.unwrap_or(from + dir * (8.0 + 24.0 * power));
            let (dir, dist) = (
                (target - from).flat().normalize_or(dir),
                (target - from).flat_length().max(2.0),
            );
            let el = ks.lob_elevation.to_radians();
            let backspin = 18.0;
            let speed = solve_lob_speed(from, dir, dist, el, backspin, bs, turf);
            let vel = dir * (speed * el.cos()) + v3(0.0, speed * el.sin(), 0.0);
            let spin = backspin_axis(dir) * backspin + v3(0.0, curl_w * 0.5, 0.0);
            let sigma = 0.02 + speed_term + turn_term + 0.05 * volley;
            (vel, spin, target, sigma, 0.02)
        }
    };
    // Apply the error: yaw, elevation and a little speed.
    let yaw = rng.gaussian() * yaw_sigma * base_err;
    let pitch = rng.gaussian() * pitch_sigma * base_err;
    let speed_k = 1.0 + rng.gaussian() * 0.03 * base_err;
    let flat = vel.flat().rotate_y(yaw);
    let h = flat.length();
    let el = vel.y.atan2(h) + pitch;
    let s = vel.length() * speed_k;
    let vel = flat.normalize_or(dir) * (s * el.cos()) + v3(0.0, s * el.sin(), 0.0);
    KickPlan { vel, spin, aim }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::player::Role;

    fn kicker_at(p: Vec3, facing: f32) -> Player {
        let mut k = Player::new(0, 0, Role::Pivot, 9, p);
        k.facing = facing;
        k
    }

    fn perfect() -> KickSpec {
        KickSpec {
            error_scale: 0.0,
            ..KickSpec::default()
        }
    }

    #[test]
    fn pass_arrives_at_receiver_with_controlled_pace() {
        let bs = BallSpec::default();
        let turf = Turf::default();
        let ks = perfect();
        let k = kicker_at(Vec3::ZERO, 0.0);
        let mate = Player::new(1, 0, Role::Winger, 7, v3(12.0, 0.0, 3.0));
        let mut ball = Ball::new(v3(0.4, bs.radius, 0.0));
        ball.grounded = true;
        let mut rng = Rng::new(1);
        let req = KickRequest::new(KickKind::Pass, v3(1.0, 0.0, 0.2), 0.3);
        let plan = plan_kick(&req, &k, &[&k, &mate], &ball, &bs, &turf, &ks, &mut rng);
        let start = launched(ball.pos, plan.vel, plan.spin);
        let mut closest = f32::MAX;
        let mut speed_there = 0.0;
        simulate(&start, &bs, &turf, 5.0, |b, _| {
            let d = b.pos.flat_distance(mate.pos);
            if d < closest {
                closest = d;
                speed_there = b.vel.flat_length();
            }
            false
        });
        assert!(closest < 0.6, "pass missed by {closest}");
        assert!(
            speed_there > 2.5 && speed_there < 6.5,
            "arrival speed {speed_there}"
        );
    }

    #[test]
    fn shot_hits_the_aimed_point() {
        let bs = BallSpec::default();
        let turf = Turf::default();
        let ks = perfect();
        let k = kicker_at(v3(8.0, 0.0, -3.0), 0.2);
        let mut ball = Ball::new(v3(8.4, bs.radius, -3.0));
        ball.grounded = true;
        let target = v3(20.0, 1.5, 1.0);
        let mut rng = Rng::new(2);
        for (power, curl) in [(0.3, 0.0), (0.9, 0.0), (0.6, 1.0), (0.6, -1.0)] {
            let req = KickRequest::new(KickKind::Shot, v3(1.0, 0.0, 0.3), power)
                .with_target(target)
                .with_curl(curl);
            let plan = plan_kick(&req, &k, &[&k], &ball, &bs, &turf, &ks, &mut rng);
            let start = launched(ball.pos, plan.vel, plan.spin);
            let mut at_line = None;
            let mut mid_z = 0.0;
            simulate(&start, &bs, &turf, 3.0, |b, _| {
                if (b.pos.x - 14.2).abs() < 0.1 {
                    mid_z = b.pos.z;
                }
                if b.pos.x >= 20.0 {
                    at_line = Some(b.pos);
                    return true;
                }
                false
            });
            let p = at_line.expect("shot should reach the goal line");
            assert!(
                p.distance(target) < 0.35,
                "power {power} curl {curl}: arrived at {p:?}"
            );
            if curl != 0.0 {
                // The straight line from ball to target passes z ≈ -1.0 at x = 14.2;
                // a left-curler (+) starts to the right (+z side of the line) and bends back.
                let line_z = -3.0 + (14.2 - 8.4) / (20.0 - 8.4) * 4.0;
                assert!(
                    (mid_z - line_z) * curl > 0.2,
                    "curl {curl}: mid z {mid_z} line {line_z}"
                );
            }
        }
    }

    #[test]
    fn chip_clears_an_obstacle() {
        let bs = BallSpec::default();
        let turf = Turf::default();
        let ks = perfect();
        let k = kicker_at(Vec3::ZERO, 0.0);
        let mut ball = Ball::new(v3(0.4, bs.radius, 0.0));
        ball.grounded = true;
        let mut rng = Rng::new(3);
        let req = KickRequest::new(KickKind::Lob, Vec3::X, 0.5).with_target(v3(15.4, 0.0, 0.0));
        let plan = plan_kick(&req, &k, &[&k], &ball, &bs, &turf, &ks, &mut rng);
        let start = launched(ball.pos, plan.vel, plan.spin);
        let mut h5 = 0.0;
        let mut landed = None;
        let mut up = false;
        simulate(&start, &bs, &turf, 4.0, |b, _| {
            if (b.pos.x - 5.4).abs() < 0.1 {
                h5 = b.pos.y;
            }
            up |= b.pos.y > 0.5;
            if up && b.pos.y <= bs.radius + 1e-3 {
                landed = Some(b.pos.x);
                return true;
            }
            false
        });
        assert!(h5 > 2.0, "height at 5 m: {h5}");
        let lx = landed.unwrap();
        assert!((lx - 15.4).abs() < 0.6, "landed at {lx}");
    }

    #[test]
    fn errors_grow_with_power() {
        let bs = BallSpec::default();
        let turf = Turf::default();
        let ks = KickSpec::default();
        let k = kicker_at(Vec3::ZERO, 0.0);
        let mut ball = Ball::new(v3(0.4, bs.radius, 0.0));
        ball.grounded = true;
        let spread = |power: f32| {
            let mut rng = Rng::new(5);
            let mut zs = Vec::new();
            for _ in 0..30 {
                let req = KickRequest::new(KickKind::Shot, Vec3::X, power)
                    .with_target(v3(15.0, 1.0, 0.0));
                let plan = plan_kick(&req, &k, &[&k], &ball, &bs, &turf, &ks, &mut rng);
                zs.push(plan.vel.z / plan.vel.x);
            }
            let m = zs.iter().sum::<f32>() / zs.len() as f32;
            (zs.iter().map(|z| (z - m) * (z - m)).sum::<f32>() / zs.len() as f32).sqrt()
        };
        assert!(spread(1.0) > spread(0.2) * 1.5);
    }
}
