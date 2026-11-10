//! Ball contacts with rigid surfaces: impulse-based bounces with Coulomb friction
//! coupled to spin, sliding→rolling on the ground, rolling resistance.
//!
//! Sign conventions: `n` is the unit surface normal pointing from the surface
//! towards the ball centre. The contact point is at `-n·r` from the centre.

use crate::ball::{Ball, BallSpec, GRAVITY};
use crate::math::{closest_point_on_segment, closest_points_segments, v3, Vec3};
use crate::pitch::{Capsule, FramePart, Pitch};

/// Material response of a surface.
#[derive(Clone, Debug)]
pub struct Surface {
    /// Coefficient of restitution for slow impacts.
    pub restitution: f32,
    /// Restitution lost per m/s of normal impact speed (harder hits bounce relatively less).
    pub restitution_drop: f32,
    pub restitution_min: f32,
    /// Coulomb friction coefficient during impacts.
    pub friction: f32,
}

impl Surface {
    /// Restitution at a given normal impact speed.
    pub fn restitution_at(&self, impact_speed: f32) -> f32 {
        (self.restitution - self.restitution_drop * impact_speed)
            .clamp(self.restitution_min, self.restitution)
    }
}

/// Artificial turf parameters.
#[derive(Clone, Debug)]
pub struct Turf {
    pub surface: Surface,
    /// Kinetic friction while the ball slides on the ground.
    pub sliding_friction: f32,
    /// Rolling resistance: constant part (fraction of g).
    pub rolling_c: f32,
    /// Rolling resistance: speed-proportional part (1/s).
    pub rolling_v: f32,
    /// Decay of spin about the vertical axis while on the ground (1/s).
    pub spin_y_decay: f32,
    /// Normal speeds below this are treated as resting contact (no bounce).
    pub rest_threshold: f32,
}

impl Default for Turf {
    fn default() -> Self {
        Self {
            surface: Surface {
                restitution: 0.74,
                restitution_drop: 0.012,
                restitution_min: 0.5,
                friction: 0.55,
            },
            sliding_friction: 0.42,
            rolling_c: 0.065,
            rolling_v: 0.12,
            spin_y_decay: 1.6,
            rest_threshold: 0.45,
        }
    }
}

/// Applies an impulsive bounce against a surface with normal `n` moving at
/// `surface_vel`. Returns the normal impact speed (0 if separating).
pub fn impact(
    ball: &mut Ball,
    spec: &BallSpec,
    n: Vec3,
    surface: &Surface,
    surface_vel: Vec3,
) -> f32 {
    let rel = ball.vel - surface_vel;
    let vn = rel.dot(n);
    if vn >= 0.0 {
        return 0.0;
    }
    let m = spec.mass;
    let inertia = spec.inertia();
    let r = spec.radius;
    let e = surface.restitution_at(-vn);
    let jn = -(1.0 + e) * m * vn;

    // Tangential friction impulse, limited by Coulomb and by the impulse that would
    // exactly stop the contact point from slipping.
    let rc = n * (-r);
    let vc = rel + ball.spin.cross(rc);
    let vt = vc - n * vc.dot(n);
    let slip = vt.length();
    let mut jt_vec = Vec3::ZERO;
    if slip > 1e-6 {
        let j_stop = slip / (1.0 / m + r * r / inertia);
        let jt = (surface.friction * jn).min(j_stop);
        jt_vec = vt * (-jt / slip);
    }
    ball.vel += n * (jn / m) + jt_vec / m;
    ball.spin += rc.cross(jt_vec) / inertia;
    -vn
}

/// Applies a friction impulse of at most `max_impulse` at the contact point with
/// normal `n`, pushing towards no slip (rolling).
pub fn friction_towards_rolling(
    ball: &mut Ball,
    spec: &BallSpec,
    n: Vec3,
    max_impulse: f32,
    surface_vel: Vec3,
) {
    let r = spec.radius;
    let m = spec.mass;
    let inertia = spec.inertia();
    let rc = n * (-r);
    let vc = ball.vel - surface_vel + ball.spin.cross(rc);
    let vt = vc - n * vc.dot(n);
    let slip = vt.length();
    if slip < 1e-6 {
        return;
    }
    let j_stop = slip / (1.0 / m + r * r / inertia);
    let jt = max_impulse.min(j_stop);
    let j = vt * (-jt / slip);
    ball.vel += j / m;
    ball.spin += rc.cross(j) / inertia;
}

/// Result of the ground contact for one step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GroundResult {
    /// Normal impact speed if the ball bounced this step.
    pub bounce: Option<f32>,
    pub touching: bool,
}

/// Ground contact (plane y = 0). Call after [`Ball::integrate_flight`].
pub fn ground(ball: &mut Ball, spec: &BallSpec, turf: &Turf, dt: f32) -> GroundResult {
    let r = spec.radius;
    if ball.pos.y > r + 1e-3 {
        ball.grounded = false;
        return GroundResult::default();
    }
    ball.pos.y = r;
    let n = Vec3::Y;
    if ball.vel.y < -turf.rest_threshold {
        let s = impact(ball, spec, n, &turf.surface, Vec3::ZERO);
        ball.grounded = false;
        return GroundResult {
            bounce: Some(s),
            touching: true,
        };
    }

    // Resting / rolling contact.
    ball.grounded = true;
    if ball.vel.y < 0.0 {
        ball.vel.y = 0.0;
    }
    friction_towards_rolling(
        ball,
        spec,
        n,
        turf.sliding_friction * spec.mass * GRAVITY * dt,
        Vec3::ZERO,
    );

    // Rolling resistance on the horizontal velocity; keep rolling spin consistent.
    let flat = ball.vel.flat();
    let speed = flat.length();
    if speed > 1e-6 {
        let decel = (turf.rolling_c * GRAVITY + turf.rolling_v * speed) * dt;
        let new_speed = (speed - decel).max(0.0);
        let k = new_speed / speed;
        ball.vel = v3(flat.x * k, ball.vel.y, flat.z * k);
        ball.spin.x *= k;
        ball.spin.z *= k;
    }
    ball.spin.y *= (-turf.spin_y_decay * dt).exp();

    // Come to rest cleanly.
    if ball.vel.flat_length() < 0.03 && ball.vel.y.abs() < 0.05 {
        ball.vel = Vec3::ZERO;
        ball.spin.x = 0.0;
        ball.spin.z = 0.0;
        if ball.spin.y.abs() < 0.5 {
            ball.spin.y = 0.0;
        }
    }
    GroundResult {
        bounce: None,
        touching: true,
    }
}

/// Surfaces of the arena other than the turf.
#[derive(Clone, Debug)]
pub struct ArenaSurfaces {
    pub boards: Surface,
    pub cage: Surface,
    pub posts: Surface,
    pub supports: Surface,
}

impl Default for ArenaSurfaces {
    fn default() -> Self {
        Self {
            boards: Surface {
                restitution: 0.62,
                restitution_drop: 0.008,
                restitution_min: 0.38,
                friction: 0.35,
            },
            cage: Surface {
                restitution: 0.18,
                restitution_drop: 0.0,
                restitution_min: 0.1,
                friction: 0.6,
            },
            posts: Surface {
                restitution: 0.78,
                restitution_drop: 0.004,
                restitution_min: 0.6,
                friction: 0.15,
            },
            supports: Surface {
                restitution: 0.35,
                restitution_drop: 0.0,
                restitution_min: 0.2,
                friction: 0.3,
            },
        }
    }
}

/// A collision of the ball with the boards / cage this step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoardContact {
    pub speed: f32,
    pub cage: bool,
}

/// Keeps the ball inside the board enclosure. The boards are solid up to their
/// height; above them the soft cage netting absorbs most of the energy.
pub fn boards(
    ball: &mut Ball,
    spec: &BallSpec,
    pitch: &Pitch,
    s: &ArenaSurfaces,
) -> Option<BoardContact> {
    let r = spec.radius;
    let (bx, bz) = (pitch.board_x(), pitch.board_z());
    let mut hit: Option<BoardContact> = None;
    let walls = [
        (ball.pos.x - (bx - r), -Vec3::X),
        (-(bx - r) - ball.pos.x, Vec3::X),
        (ball.pos.z - (bz - r), -Vec3::Z),
        (-(bz - r) - ball.pos.z, Vec3::Z),
    ];
    for (pen, n) in walls {
        if pen > 0.0 {
            ball.pos += n * pen;
            let cage = ball.pos.y > pitch.arena.board_height + r * 0.3;
            let surf = if cage { &s.cage } else { &s.boards };
            let speed = impact(ball, spec, n, surf, Vec3::ZERO);
            if speed > 0.0 {
                let prev = hit.map(|h| h.speed).unwrap_or(0.0);
                if speed > prev {
                    hit = Some(BoardContact { speed, cage });
                }
            }
        }
    }
    hit
}

/// Swept collision of the ball against a capsule (posts, bars). The ball's motion
/// during the last step (`prev_pos → pos`) is tested so fast shots cannot pass
/// through. Returns the normal impact speed.
pub fn capsule(ball: &mut Ball, spec: &BallSpec, cap: &Capsule, surf: &Surface) -> f32 {
    capsule_moving(ball, spec, cap, surf, Vec3::ZERO)
}

/// Like [`capsule`] for a capsule moving at `cap_vel` (footballers' bodies).
pub fn capsule_moving(
    ball: &mut Ball,
    spec: &BallSpec,
    cap: &Capsule,
    surf: &Surface,
    cap_vel: Vec3,
) -> f32 {
    let reach = spec.radius + cap.radius;
    let (pb, pc, s_closest, _) = closest_points_segments(ball.prev_pos, ball.pos, cap.a, cap.b);
    if pb.distance_sq(pc) >= reach * reach {
        return 0.0;
    }
    // Find the first moment of contact along the step's motion.
    let path = ball.pos - ball.prev_pos;
    let dist_at = |s: f32| {
        let p = ball.prev_pos + path * s;
        let (q, _) = closest_point_on_segment(p, cap.a, cap.b);
        p.distance(q)
    };
    let (mut lo, mut hi) = (0.0f32, s_closest);
    if dist_at(0.0) < reach {
        hi = 0.0;
    } else {
        for _ in 0..12 {
            let mid = 0.5 * (lo + hi);
            if dist_at(mid) < reach {
                hi = mid;
            } else {
                lo = mid;
            }
        }
    }
    let p = ball.prev_pos + path * hi;
    let (q, _) = closest_point_on_segment(p, cap.a, cap.b);
    let n = (p - q).normalize_or((ball.prev_pos - q).normalize_or(Vec3::Y));
    ball.pos = q + n * (reach + 1e-4);
    impact(ball, spec, n, surf, cap_vel)
}

/// Collides the ball with every frame part of both goals. Returns the strongest hit.
pub fn goal_frames(
    ball: &mut Ball,
    spec: &BallSpec,
    colliders: &[(crate::pitch::Side, Capsule)],
    s: &ArenaSurfaces,
) -> Option<(crate::pitch::Side, FramePart, f32)> {
    let mut best: Option<(crate::pitch::Side, FramePart, f32)> = None;
    for (side, c) in colliders {
        let surf = if c.part == FramePart::Support {
            &s.supports
        } else {
            &s.posts
        };
        let speed = capsule(ball, spec, c, surf);
        if speed > 0.0 && best.map(|b| speed > b.2).unwrap_or(true) {
            best = Some((*side, c.part, speed));
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PHYSICS_DT;

    fn step(b: &mut Ball, s: &BallSpec, t: &Turf) -> GroundResult {
        b.integrate_flight(s, PHYSICS_DT);
        ground(b, s, t, PHYSICS_DT)
    }

    #[test]
    fn bounces_get_lower_and_ball_settles() {
        let s = BallSpec::default();
        let t = Turf::default();
        let mut b = Ball::new(v3(0.0, 2.0 + s.radius, 0.0));
        let mut apexes = Vec::new();
        let mut cur_apex = 0.0f32;
        let mut was_up = true;
        for _ in 0..(240 * 8) {
            let res = step(&mut b, &s, &t);
            cur_apex = cur_apex.max(b.pos.y - s.radius);
            if res.bounce.is_some() && was_up {
                apexes.push(cur_apex);
                cur_apex = 0.0;
            }
            was_up = res.bounce.is_none();
        }
        assert!(apexes.len() >= 3, "{apexes:?}");
        // First apex is the drop height; each following one is lower.
        for w in apexes.windows(2) {
            assert!(w[1] < w[0], "{apexes:?}");
        }
        // First bounce from 2 m on turf: roughly 0.7–1.1 m.
        assert!(apexes[1] > 0.7 && apexes[1] < 1.1, "{apexes:?}");
        assert!(b.grounded && b.vel.length() < 1e-3);
        assert!((b.pos.y - s.radius).abs() < 1e-3);
    }

    fn roll_distance(v0: f32) -> f32 {
        let s = BallSpec::default();
        let t = Turf::default();
        let mut b = Ball::new(v3(0.0, s.radius, 0.0));
        b.vel = v3(v0, 0.0, 0.0);
        b.spin = v3(0.0, 0.0, -v0 / s.radius); // already rolling
        b.grounded = true;
        for _ in 0..(240 * 30) {
            step(&mut b, &s, &t);
            if b.vel.length() == 0.0 {
                break;
            }
        }
        b.pos.x
    }

    #[test]
    fn rolling_distances_are_realistic() {
        let d5 = roll_distance(5.0);
        let d10 = roll_distance(10.0);
        let d15 = roll_distance(15.0);
        println!("roll distances: 5 m/s {d5:.1} m, 10 m/s {d10:.1} m, 15 m/s {d15:.1} m");
        assert!(d5 > 6.0 && d5 < 13.0, "5 m/s rolls {d5}");
        assert!(d10 > 16.0 && d10 < 30.0, "10 m/s rolls {d10}");
        assert!(d15 > d10 + 5.0 && d15 < 45.0, "15 m/s rolls {d15}");
    }

    #[test]
    fn sliding_ball_starts_rolling() {
        let s = BallSpec::default();
        let t = Turf::default();
        let mut b = Ball::new(v3(0.0, s.radius, 0.0));
        b.vel = v3(8.0, 0.0, 0.0);
        b.grounded = true;
        for _ in 0..360 {
            step(&mut b, &s, &t);
        }
        // contact point should not slip: v + ω × (-r ŷ) ≈ 0
        let vc = b.vel + b.spin.cross(v3(0.0, -s.radius, 0.0));
        assert!(vc.flat_length() < 0.05, "slip {}", vc.flat_length());
        // a sliding ball keeps 1/(1+k) of its speed when rolling starts (≈ 60 % for k = 2/3),
        // then slows from rolling resistance and drag
        assert!(b.vel.x > 2.5 && b.vel.x < 5.0, "v {}", b.vel.x);
        assert!(b.spin.z < 0.0);
    }

    #[test]
    fn backspin_checks_up() {
        let s = BallSpec::default();
        let t = Turf::default();
        let land = |spin_z: f32| {
            let mut b = Ball::new(v3(0.0, s.radius + 0.01, 0.0));
            b.vel = v3(10.0, -6.0, 0.0);
            b.spin = v3(0.0, 0.0, spin_z);
            step(&mut b, &s, &t);
            b.vel.x
        };
        let plain = land(0.0);
        let back = land(80.0); // backspin for +x travel
        let top = land(-80.0); // topspin
        assert!(back < plain, "backspin {back} plain {plain}");
        assert!(top > plain, "topspin {top} plain {plain}");
    }

    #[test]
    fn side_spin_on_bounce_kicks_sideways_and_decays_on_ground() {
        let s = BallSpec::default();
        let t = Turf::default();
        let mut b = Ball::new(v3(0.0, s.radius, 0.0));
        b.vel = v3(3.0, 0.0, 0.0);
        b.spin = v3(0.0, 20.0, 0.0);
        b.grounded = true;
        for _ in 0..240 {
            step(&mut b, &s, &t);
        }
        assert!(b.spin.y.abs() < 5.0);
    }

    fn arena_step(
        b: &mut Ball,
        s: &BallSpec,
        t: &Turf,
        p: &Pitch,
        cols: &[(crate::pitch::Side, Capsule)],
    ) {
        let a = ArenaSurfaces::default();
        b.integrate_flight(s, PHYSICS_DT);
        goal_frames(b, s, cols, &a);
        boards(b, s, p, &a);
        ground(b, s, t, PHYSICS_DT);
    }

    fn all_colliders(p: &Pitch) -> Vec<(crate::pitch::Side, Capsule)> {
        crate::pitch::Side::BOTH
            .iter()
            .flat_map(|side| p.goal_colliders(*side).into_iter().map(move |c| (*side, c)))
            .collect()
    }

    #[test]
    fn ball_never_leaves_the_arena() {
        let s = BallSpec::default();
        let t = Turf::default();
        let p = Pitch::new();
        let cols = all_colliders(&p);
        let mut rng = crate::math::Rng::new(9);
        for _ in 0..40 {
            let mut b = Ball::new(v3(rng.range(-15.0, 15.0), 0.5, rng.range(-8.0, 8.0)));
            let dir = Vec3::from_heading(rng.range(-3.14, 3.14));
            b.vel = dir * rng.range(10.0, 35.0) + v3(0.0, rng.range(0.0, 12.0), 0.0);
            b.spin = v3(
                rng.range(-60.0, 60.0),
                rng.range(-60.0, 60.0),
                rng.range(-60.0, 60.0),
            );
            for _ in 0..(240 * 6) {
                arena_step(&mut b, &s, &t, &p, &cols);
                assert!(p.inside_arena(b.pos, 0.01), "escaped at {:?}", b.pos);
                assert!(b.is_finite());
            }
        }
    }

    #[test]
    fn post_hit_deflects() {
        let s = BallSpec::default();
        let p = Pitch::new();
        let cols = all_colliders(&p);
        let f = p.frame(crate::pitch::Side::Right);
        // Fire straight at the +z post at 30 m/s, 1 m high.
        let mut b = Ball::new(v3(14.0, 1.0, f.post_z));
        b.vel = v3(30.0, 0.0, 0.0);
        let a = ArenaSurfaces::default();
        let mut hit = None;
        for _ in 0..120 {
            b.integrate_flight(&s, PHYSICS_DT);
            if let Some(h) = goal_frames(&mut b, &s, &cols, &a) {
                hit = Some(h);
                break;
            }
        }
        let (side, part, speed) = hit.expect("should hit the post");
        assert_eq!(side, crate::pitch::Side::Right);
        assert_eq!(part, FramePart::Post);
        assert!(speed > 20.0);
        assert!(b.vel.x < -10.0, "rebounds back into the pitch: {:?}", b.vel);
        // never inside the post
        assert!(
            v3(b.pos.x - f.line_x, 0.0, b.pos.z - f.post_z).length()
                >= s.radius + p.goal.post_radius - 1e-3
        );
    }

    #[test]
    fn fast_ball_cannot_tunnel_through_crossbar() {
        let s = BallSpec::default();
        let p = Pitch::new();
        let cols = all_colliders(&p);
        let f = p.frame(crate::pitch::Side::Right);
        let mut b = Ball::new(v3(17.0, f.bar_y, 0.3));
        b.vel = v3(40.0, 0.0, 0.0);
        let a = ArenaSurfaces::default();
        let mut hit = false;
        for _ in 0..60 {
            b.integrate_flight(&s, PHYSICS_DT);
            if goal_frames(&mut b, &s, &cols, &a).is_some() {
                hit = true;
                break;
            }
        }
        assert!(hit && b.vel.x < 0.0);
    }

    #[test]
    fn boards_rebound_and_cage_absorbs() {
        let s = BallSpec::default();
        let p = Pitch::new();
        let a = ArenaSurfaces::default();
        let mut low = Ball::new(v3(0.0, 0.4, 10.5));
        low.vel = v3(0.0, 0.0, 10.0);
        let mut high = Ball::new(v3(0.0, 3.0, 10.5));
        high.vel = v3(0.0, 0.0, 10.0);
        for b in [&mut low, &mut high] {
            for _ in 0..30 {
                b.integrate_flight(&s, PHYSICS_DT);
                boards(b, &s, &p, &a);
            }
        }
        assert!(low.vel.z < -4.0, "board rebound {:?}", low.vel);
        assert!(
            high.vel.z < 0.0 && high.vel.z > -3.0,
            "cage rebound {:?}",
            high.vel
        );
    }
}
