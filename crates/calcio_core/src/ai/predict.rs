//! Ball trajectory prediction (real physics, no players) and intercept times.

use crate::ball::{Ball, BallSpec};
use crate::contact::Turf;
use crate::kick;
use crate::math::Vec3;
use crate::player::{Player, PlayerSpec};

/// Predicted ball positions every `step` seconds.
#[derive(Clone, Debug, Default)]
pub struct Prediction {
    pub step: f32,
    pub points: Vec<Vec3>,
}

impl Prediction {
    pub fn new(ball: &Ball, bs: &BallSpec, turf: &Turf, horizon: f32, step: f32) -> Self {
        let mut points = vec![ball.pos];
        let mut next = step;
        kick::simulate(ball, bs, turf, horizon, |b, t| {
            if t >= next {
                points.push(b.pos);
                next += step;
            }
            false
        });
        Self { step, points }
    }

    /// Ball position at time `t` (clamped to the horizon).
    pub fn at(&self, t: f32) -> Vec3 {
        if self.points.is_empty() {
            return Vec3::ZERO;
        }
        let f = (t / self.step).max(0.0);
        let i = f.floor() as usize;
        if i + 1 >= self.points.len() {
            return *self.points.last().unwrap();
        }
        self.points[i].lerp(self.points[i + 1], f - i as f32)
    }

    pub fn horizon(&self) -> f32 {
        self.step * (self.points.len().saturating_sub(1)) as f32
    }
}

/// Rough time for `p` to run to `target` (reaction + acceleration + top speed).
pub fn time_to_reach(p: &Player, spec: &PlayerSpec, target: Vec3, reaction: f32) -> f32 {
    let to = (target - p.pos).flat();
    let d = (to.length() - 0.5).max(0.0);
    let vmax = spec.sprint_speed * 0.95;
    // Current velocity towards the target helps, velocity away hurts.
    let along = p.vel.flat().dot(to.normalize_or_zero());
    let accel = 5.0;
    let t_acc = ((vmax - along) / accel).max(0.0);
    let d_acc = along * t_acc + 0.5 * accel * t_acc * t_acc;
    let t = if d <= d_acc {
        // Still accelerating when arriving: solve d = along t + a t²/2.
        let a = 0.5 * accel;
        let disc = along * along + 4.0 * a * d;
        (-along + disc.sqrt()) / (2.0 * a)
    } else {
        t_acc + (d - d_acc) / vmax
    };
    reaction + t + if p.stumble > 0.0 { p.stumble } else { 0.0 }
}

/// Earliest moment the predicted ball passes within heading reach of `p`
/// (standing or jumping) at a spot he can run to: (time, ball centre).
pub fn header_intercept(p: &Player, spec: &PlayerSpec, pred: &Prediction, reaction: f32) -> Option<(f32, Vec3)> {
    let lo = spec.head_height - 0.22;
    let hi = spec.head_height + spec.jump_running + 0.08;
    for (i, pt) in pred.points.iter().enumerate() {
        let t = i as f32 * pred.step;
        if t < 0.12 || pt.y < lo || pt.y > hi {
            continue;
        }
        if time_to_reach(p, spec, *pt, reaction) + 0.08 <= t {
            return Some((t, *pt));
        }
    }
    None
}

/// Earliest time and point at which `p` can meet the predicted ball (ball low enough).
pub fn intercept(
    p: &Player,
    spec: &PlayerSpec,
    pred: &Prediction,
    bs: &BallSpec,
    reaction: f32,
) -> (f32, Vec3) {
    for (i, pt) in pred.points.iter().enumerate() {
        let t = i as f32 * pred.step;
        if pt.y - bs.radius > 1.0 {
            continue;
        }
        if time_to_reach(p, spec, *pt, reaction) <= t {
            return (t, *pt);
        }
    }
    let last = *pred.points.last().unwrap_or(&p.pos);
    (
        pred.horizon() + time_to_reach(p, spec, last, reaction),
        last,
    )
}
