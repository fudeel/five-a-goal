//! Goal nets as cloth: knots are Verlet particles joined by strings.
//!
//! * Panels (roof, back, two sides) are grids that share their seam knots.
//! * Every panel edge sits on a frame bar, so edge knots are pinned.
//! * Strings only resist **stretching** (they go slack under compression), with
//!   a little slack so the net sags naturally under gravity.
//! * Position-based constraint solving with a few iterations per step.
//! * A net that is at rest and far from the ball goes to sleep.

use crate::ball::{Ball, BallSpec};
use crate::math::{closest_point_on_triangle, v3, Vec3};
use crate::pitch::{Pitch, Side};
use std::collections::HashMap;

/// Tunable net parameters.
#[derive(Clone, Debug)]
pub struct NetSpec {
    /// Target mesh size (m).
    pub spacing: f32,
    /// Rest length = initial length × slack.
    pub slack: f32,
    /// Mass of one knot (kg). A futsal net weighs a few kilograms.
    pub knot_mass: f32,
    /// Linear velocity damping (1/s).
    pub damping: f32,
    /// Constraint solver iterations per step.
    pub iterations: usize,
    /// Strings are hard-limited to this stretch ratio.
    pub max_stretch: f32,
    /// Knot speed below which the net may sleep (m/s).
    pub sleep_speed: f32,
    /// Seconds of stillness before sleeping.
    pub sleep_delay: f32,
    /// Collision thickness of a string (m).
    pub strand_radius: f32,
    /// Friction between ball and strings (0..1).
    pub ball_friction: f32,
    /// Max ball travel per coupled substep (anti-tunnelling).
    pub max_substep_travel: f32,
    /// Fraction of the net's spring-back that is returned to the ball (0..1).
    pub rebound: f32,
}

/// Summary of a coupled ball–net step.
#[derive(Clone, Copy, Debug, Default)]
pub struct NetHit {
    /// Number of string contacts during the step.
    pub contacts: u32,
    /// Ball speed change caused by the net (m/s).
    pub impulse_speed: f32,
}

impl Default for NetSpec {
    fn default() -> Self {
        Self {
            spacing: 0.15,
            slack: 1.03,
            knot_mass: 0.005,
            damping: 2.0,
            iterations: 8,
            max_stretch: 1.35,
            sleep_speed: 0.01,
            sleep_delay: 1.0,
            strand_radius: 0.006,
            ball_friction: 0.45,
            max_substep_travel: 0.035,
            rebound: 0.12,
        }
    }
}

/// A string between two knots.
#[derive(Clone, Copy, Debug)]
pub struct Link {
    pub a: u32,
    pub b: u32,
    pub rest: f32,
}

/// One goal net.
#[derive(Clone, Debug)]
pub struct GoalNet {
    pub side: Side,
    pub pos: Vec<Vec3>,
    pub prev: Vec<Vec3>,
    /// 0 for pinned knots, 1/mass for free ones.
    pub inv_mass: Vec<f32>,
    /// Outward normal of the panel each knot belongs to (away from the goal mouth).
    pub normal: Vec<Vec3>,
    pub links: Vec<Link>,
    /// Mesh cells (4 knots each, in order) used for ball collision.
    pub cells: Vec<[u32; 4]>,
    /// Axis-aligned bounds of the rest shape (for proximity tests).
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
    pub sleeping: bool,
    still_time: f32,
}

struct Builder {
    pos: Vec<Vec3>,
    normal: Vec<Vec3>,
    pinned: Vec<bool>,
    links: Vec<(u32, u32)>,
    cells: Vec<[u32; 4]>,
    index: HashMap<(i32, i32, i32), u32>,
}

impl Builder {
    fn node(&mut self, p: Vec3, n: Vec3, pinned: bool) -> u32 {
        let key = (
            (p.x * 1000.0).round() as i32,
            (p.y * 1000.0).round() as i32,
            (p.z * 1000.0).round() as i32,
        );
        if let Some(&i) = self.index.get(&key) {
            if pinned {
                self.pinned[i as usize] = true;
            }
            // Seam knot: average the panel normals.
            let i = i as usize;
            self.normal[i] = (self.normal[i] + n).normalize_or(n);
            return i as u32;
        }
        let i = self.pos.len() as u32;
        self.pos.push(p);
        self.normal.push(n);
        self.pinned.push(pinned);
        self.index.insert(key, i);
        i
    }

    /// Bilinear panel with corners c00 → c10 (u) and c00 → c01 (v), nu × nv cells.
    /// `out` is the outward normal. All edge knots are pinned to the frame.
    fn panel(&mut self, c: [Vec3; 4], nu: usize, nv: usize, out: Vec3) {
        let [c00, c10, c11, c01] = c;
        let mut ids = vec![0u32; (nu + 1) * (nv + 1)];
        for j in 0..=nv {
            let v = j as f32 / nv as f32;
            for i in 0..=nu {
                let u = i as f32 / nu as f32;
                let p = c00.lerp(c10, u).lerp(c01.lerp(c11, u), v);
                let edge = i == 0 || j == 0 || i == nu || j == nv;
                ids[j * (nu + 1) + i] = self.node(p, out, edge);
            }
        }
        for j in 0..=nv {
            for i in 0..=nu {
                let id = ids[j * (nu + 1) + i];
                if i < nu {
                    self.links.push((id, ids[j * (nu + 1) + i + 1]));
                }
                if j < nv {
                    self.links.push((id, ids[(j + 1) * (nu + 1) + i]));
                }
                if i < nu && j < nv {
                    self.cells.push([
                        id,
                        ids[j * (nu + 1) + i + 1],
                        ids[(j + 1) * (nu + 1) + i + 1],
                        ids[(j + 1) * (nu + 1) + i],
                    ]);
                }
            }
        }
    }
}

impl GoalNet {
    /// Builds the net for the goal on `side`.
    pub fn new(pitch: &Pitch, side: Side, spec: &NetSpec) -> Self {
        let f = pitch.frame(side);
        let o = f.out;
        let pr = pitch.goal.post_radius;
        // Attach to the back surface of the posts and crossbar.
        let front = |p: Vec3| p + v3(o * pr, 0.0, 0.0);
        let (pt_l, pt_r) = (front(f.post_top(-1.0)), front(f.post_top(1.0)));
        let (pb_l, pb_r) = (front(f.post_base(-1.0)), front(f.post_base(1.0)));
        let (bt_l, bt_r) = (f.back_top(-1.0), f.back_top(1.0));
        let (bb_l, bb_r) = (f.back_bottom(-1.0), f.back_bottom(1.0));

        let n_w = ((2.0 * f.post_z) / spec.spacing).round().max(2.0) as usize;
        let n_d = (f.depth_top / spec.spacing).round().max(2.0) as usize;
        let n_h = (f.bar_y / spec.spacing).round().max(2.0) as usize;

        let mut b = Builder {
            pos: Vec::new(),
            normal: Vec::new(),
            pinned: Vec::new(),
            links: Vec::new(),
            cells: Vec::new(),
            index: HashMap::new(),
        };
        // Roof: u across the goal, v from crossbar to back bar.
        b.panel([pt_l, pt_r, bt_r, bt_l], n_w, n_d, Vec3::Y);
        // Back: u across, v from top back bar down to the ground bar.
        let back_n =
            v3(o * f.bar_y, f.depth_bottom - f.depth_top, 0.0).normalize_or(v3(o, 0.0, 0.0));
        b.panel([bt_l, bt_r, bb_r, bb_l], n_w, n_h, back_n);
        // Sides: u from post to back, v from ground to top.
        for (s, pb, bb, bt, pt) in [
            (-1.0, pb_l, bb_l, bt_l, pt_l),
            (1.0, pb_r, bb_r, bt_r, pt_r),
        ] {
            b.panel([pb, bb, bt, pt], n_d, n_h, v3(0.0, 0.0, s));
        }

        let knot_inv = 1.0 / spec.knot_mass;
        let inv_mass = b
            .pinned
            .iter()
            .map(|&p| if p { 0.0 } else { knot_inv })
            .collect();
        let links = b
            .links
            .iter()
            .filter(|(a, c)| a != c)
            .map(|&(a, c)| Link {
                a,
                b: c,
                rest: b.pos[a as usize].distance(b.pos[c as usize]) * spec.slack,
            })
            .collect();
        let mut mn = Vec3::splat(f32::MAX);
        let mut mx = Vec3::splat(f32::MIN);
        for p in &b.pos {
            mn = mn.min(*p);
            mx = mx.max(*p);
        }
        Self {
            side,
            prev: b.pos.clone(),
            pos: b.pos,
            inv_mass,
            normal: b.normal,
            links,
            cells: b.cells,
            bounds_min: mn,
            bounds_max: mx,
            sleeping: false,
            still_time: 0.0,
        }
    }

    /// Distance from `p` to the net's rest bounds (0 if inside).
    pub fn distance_to_bounds(&self, p: Vec3) -> f32 {
        let d = (self.bounds_min - p)
            .max(p - self.bounds_max)
            .max(Vec3::ZERO);
        d.length()
    }

    pub fn wake(&mut self) {
        self.sleeping = false;
        self.still_time = 0.0;
    }

    /// Verlet integration of free knots (gravity + damping).
    pub fn integrate(&mut self, spec: &NetSpec, dt: f32) {
        let keep = (1.0 - spec.damping * dt).max(0.0);
        let g = v3(0.0, -crate::ball::GRAVITY * dt * dt, 0.0);
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                continue;
            }
            let p = self.pos[i];
            let vel = (p - self.prev[i]) * keep;
            self.prev[i] = p;
            self.pos[i] = p + vel + g;
        }
    }

    /// One pass over all strings (stretch only).
    pub fn solve_links(&mut self, spec: &NetSpec) {
        for l in &self.links {
            let (a, b) = (l.a as usize, l.b as usize);
            let (wa, wb) = (self.inv_mass[a], self.inv_mass[b]);
            let w = wa + wb;
            if w == 0.0 {
                continue;
            }
            let d = self.pos[b] - self.pos[a];
            let len = d.length();
            if len <= l.rest || len < 1e-9 {
                continue;
            }
            let max_len = l.rest * spec.max_stretch;
            let target = if len > max_len { max_len } else { l.rest };
            let corr = d * ((len - target) / (len * w));
            self.pos[a] += corr * wa;
            self.pos[b] -= corr * wb;
        }
    }

    /// Knots cannot sink into the turf.
    pub fn solve_ground(&mut self) {
        for i in 0..self.pos.len() {
            if self.inv_mass[i] > 0.0 && self.pos[i].y < 0.01 {
                self.pos[i].y = 0.01;
                // ground friction: kill horizontal motion
                self.prev[i].x = self.pos[i].x * 0.5 + self.prev[i].x * 0.5;
                self.prev[i].z = self.pos[i].z * 0.5 + self.prev[i].z * 0.5;
            }
        }
    }

    /// Largest knot speed in the last step (m/s).
    pub fn max_speed(&self, dt: f32) -> f32 {
        let mut m = 0.0f32;
        for i in 0..self.pos.len() {
            m = m.max(self.pos[i].distance_sq(self.prev[i]));
        }
        m.sqrt() / dt
    }

    /// Full step without a ball nearby. Handles sleeping.
    pub fn step(&mut self, spec: &NetSpec, dt: f32) {
        if self.sleeping {
            return;
        }
        self.integrate(spec, dt);
        for _ in 0..spec.iterations {
            self.solve_links(spec);
        }
        self.solve_ground();
        self.update_sleep(spec, dt);
    }

    pub fn update_sleep(&mut self, spec: &NetSpec, dt: f32) {
        if self.max_speed(dt) < spec.sleep_speed {
            self.still_time += dt;
            if self.still_time > spec.sleep_delay {
                self.sleeping = true;
                self.prev.clone_from(&self.pos);
            }
        } else {
            self.still_time = 0.0;
        }
    }

    /// Lets the net hang under gravity until it is at rest (used at start-up so
    /// matches begin with a naturally sagging net).
    pub fn settle(&mut self, spec: &NetSpec, dt: f32, max_seconds: f32) {
        // Heavy damping converges quickly to the same rest shape.
        let heavy = NetSpec {
            damping: 12.0,
            ..spec.clone()
        };
        let steps = (max_seconds / dt) as usize;
        for _ in 0..steps {
            self.step(&heavy, dt);
            if self.sleeping {
                break;
            }
        }
        self.wake();
        self.prev.clone_from(&self.pos);
        self.sleeping = true;
    }

    pub fn is_finite(&self) -> bool {
        self.pos.iter().all(|p| p.is_finite())
    }
}

impl GoalNet {
    /// True if the ball is close enough to interact during the next step.
    pub fn ball_is_near(&self, ball: &Ball, spec: &BallSpec, dt: f32) -> bool {
        self.distance_to_bounds(ball.pos) < spec.radius + 0.5 + ball.vel.length() * dt
    }

    /// Steps the net together with the ball. Call after the ball has been
    /// integrated for this step (`ball.prev_pos → ball.pos`). The ball is treated
    /// as a heavy particle inside the constraint solver, so momentum flows both
    /// ways: the ball stretches the net, the net slows the ball down.
    ///
    /// For the ball the net is a continuous surface made of the mesh cells (a 22 cm
    /// ball cannot pass through a 15 cm mesh), which keeps it from slipping through
    /// when the mesh is stretched.
    pub fn step_with_ball(
        &mut self,
        spec: &NetSpec,
        ball: &mut Ball,
        bs: &BallSpec,
        dt: f32,
    ) -> NetHit {
        self.wake();
        let from = ball.prev_pos;
        let to = ball.pos;
        let v0 = ball.vel;
        let travel = from.distance(to);
        let k = ((travel / spec.max_substep_travel).ceil() as usize).clamp(1, 12);
        let h = dt / k as f32;
        let reach = bs.radius + spec.strand_radius;
        let w_ball = 1.0 / bs.mass;
        let mut corr = Vec3::ZERO;
        let mut hit = NetHit::default();
        let mut push_dir = Vec3::ZERO;
        let mut ball_sub_start = from;
        // Candidate triangles near the ball's path.
        let mid = from.lerp(to, 0.5);
        let cand_r = reach + 0.5 * travel + 0.6;
        let mut tris: Vec<[usize; 3]> = Vec::new();
        for c in &self.cells {
            let ids = [c[0] as usize, c[1] as usize, c[2] as usize, c[3] as usize];
            let centre = (self.pos[ids[0]] + self.pos[ids[2]]) * 0.5;
            if centre.distance(mid) < cand_r {
                tris.push([ids[0], ids[1], ids[2]]);
                tris.push([ids[0], ids[2], ids[3]]);
            }
        }
        for sub in 1..=k {
            let path_pt = from.lerp(to, sub as f32 / k as f32);
            let mut bp = path_pt + corr;
            self.integrate(spec, h);
            for _ in 0..spec.iterations {
                self.solve_links(spec);
                for t in &tris {
                    let (pa, pb, pc) = (self.pos[t[0]], self.pos[t[1]], self.pos[t[2]]);
                    // Outward triangle normal (oriented with the panel normals).
                    let mut n = (pb - pa).cross(pc - pa).normalize_or_zero();
                    if n == Vec3::ZERO {
                        continue;
                    }
                    let outward = self.normal[t[0]] + self.normal[t[1]] + self.normal[t[2]];
                    if n.dot(outward) < 0.0 {
                        n = -n;
                    }
                    let (q, bw) = closest_point_on_triangle(bp, pa, pb, pc);
                    let d = bp - q;
                    let dist = d.length();
                    if dist > reach * 1.6 {
                        continue;
                    }
                    // Which side was the ball on at the start of this substep?
                    let (qa, qb, qc) = (self.prev[t[0]], self.prev[t[1]], self.prev[t[2]]);
                    let q0 = qa * bw[0] + qb * bw[1] + qc * bw[2];
                    let was = (ball_sub_start - q0).dot(n);
                    let now = d.dot(n);
                    let crossed = was * now < 0.0 && was.abs() > 1e-6;
                    if dist >= reach && !crossed {
                        continue;
                    }
                    let side = if was.abs() > 1e-6 {
                        was.signum()
                    } else {
                        now.signum()
                    };
                    let (nrm, pen) = if crossed {
                        (n * side, reach + now.abs())
                    } else {
                        let nrm = if dist > 1e-6 { d / dist } else { n * side };
                        (nrm, reach - dist)
                    };
                    let w = [
                        self.inv_mass[t[0]] * bw[0],
                        self.inv_mass[t[1]] * bw[1],
                        self.inv_mass[t[2]] * bw[2],
                    ];
                    let w_tri = w[0] * bw[0] + w[1] * bw[1] + w[2] * bw[2];
                    let lambda = pen / (w_ball + w_tri);
                    bp += nrm * (lambda * w_ball);
                    for j in 0..3 {
                        self.pos[t[j]] -= nrm * (lambda * w[j]);
                    }
                    push_dir += nrm * lambda;
                    // Friction against sliding along the net.
                    let q1 =
                        self.pos[t[0]] * bw[0] + self.pos[t[1]] * bw[1] + self.pos[t[2]] * bw[2];
                    let rel = (bp - ball_sub_start) - (q1 - q0);
                    let tang = rel - nrm * rel.dot(nrm);
                    let tl = tang.length();
                    if tl > 1e-7 {
                        let f = (spec.ball_friction * lambda * w_ball).min(tl);
                        bp -= tang * (f / tl);
                    }
                    hit.contacts += 1;
                }
            }
            self.solve_ground();
            corr = bp - path_pt;
            ball_sub_start = bp;
        }
        if hit.contacts > 0 {
            let dv = (corr / dt).clamp_length(v0.length() + 5.0);
            let mut v1 = v0 + dv;
            // The net gives back only part of its spring-back energy.
            let n = push_dir.normalize_or_zero();
            if n != Vec3::ZERO {
                let vn0 = v0.dot(n);
                let vn1 = v1.dot(n);
                if vn1 > 0.0 {
                    let allowed = if vn0 < 0.0 {
                        vn1 * spec.rebound
                    } else {
                        vn0 + (vn1 - vn0) * spec.rebound
                    };
                    v1 += n * (allowed - vn1);
                }
            }
            hit.impulse_speed = (v1 - v0).length();
            ball.vel = v1;
            ball.pos = to + corr;
            ball.spin *= 0.985;
        }
        self.update_sleep(spec, dt);
        hit
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PHYSICS_DT;

    #[test]
    fn net_structure() {
        let p = Pitch::new();
        let spec = NetSpec::default();
        for side in Side::BOTH {
            let n = GoalNet::new(&p, side, &spec);
            assert!(
                n.pos.len() > 400 && n.pos.len() < 900,
                "knots {}",
                n.pos.len()
            );
            assert!(n.links.len() > n.pos.len());
            let free = n.inv_mass.iter().filter(|&&w| w > 0.0).count();
            assert!(free > 200, "free knots {free}");
            // every knot sits behind the goal line
            for k in &n.pos {
                assert!(side.sign() * k.x >= p.half_length() - 1e-4);
            }
        }
    }

    #[test]
    fn net_settles_without_exploding_and_pins_hold() {
        let p = Pitch::new();
        let spec = NetSpec::default();
        let mut n = GoalNet::new(&p, Side::Right, &spec);
        let pinned: Vec<(usize, Vec3)> = (0..n.pos.len())
            .filter(|&i| n.inv_mass[i] == 0.0)
            .map(|i| (i, n.pos[i]))
            .collect();
        n.settle(&spec, PHYSICS_DT, 10.0);
        assert!(n.is_finite());
        // The settled shape is a true rest state: woken up, it barely moves and sleeps again.
        let rest = n.pos.clone();
        n.wake();
        for _ in 0..(240 * 3) {
            n.step(&spec, PHYSICS_DT);
        }
        assert!(n.sleeping, "net should come to rest");
        let drift = n
            .pos
            .iter()
            .zip(rest.iter())
            .map(|(a, b)| a.distance(*b))
            .fold(0.0, f32::max);
        assert!(drift < 0.02, "drift {drift}");
        for (i, p0) in pinned {
            assert_eq!(n.pos[i], p0);
        }
        // sagging: the roof centre hangs below the crossbar
        let f = p.frame(Side::Right);
        let roof_mid = n
            .pos
            .iter()
            .filter(|k| (k.x - (f.line_x + 0.4)).abs() < 0.1 && k.z.abs() < 0.1 && k.y > 1.5)
            .fold(f32::MAX, |m, k| m.min(k.y));
        assert!(
            roof_mid < f.bar_y - 0.02 && roof_mid > f.bar_y - 0.4,
            "roof sag {roof_mid}"
        );
        // strings stay within their stretch limit
        for l in &n.links {
            let len = n.pos[l.a as usize].distance(n.pos[l.b as usize]);
            assert!(len <= l.rest * spec.max_stretch + 1e-3);
        }
    }

    fn shoot(start: Vec3, vel: Vec3, seconds: f32) -> (Ball, GoalNet, bool) {
        let (b, n, e, _) = shoot_track(start, vel, seconds);
        (b, n, e)
    }

    /// Like `shoot`, also returns (max x reached, max speed back towards the pitch after contact).
    fn shoot_track(start: Vec3, vel: Vec3, seconds: f32) -> (Ball, GoalNet, bool, (f32, f32)) {
        let p = Pitch::new();
        let spec = NetSpec::default();
        let bs = BallSpec::default();
        let turf = crate::contact::Turf::default();
        let mut net = GoalNet::new(&p, Side::Right, &spec);
        net.settle(&spec, PHYSICS_DT, 8.0);
        let mut b = Ball::new(start);
        b.vel = vel;
        let mut escaped = false;
        let mut max_x = f32::MIN;
        let mut back_speed = 0.0f32;
        let mut touched = false;
        let f = p.frame(Side::Right);
        let back_limit = f.line_x + f.depth_bottom + 0.6;
        for _ in 0..((seconds / PHYSICS_DT) as usize) {
            b.integrate_flight(&bs, PHYSICS_DT);
            if net.ball_is_near(&b, &bs, PHYSICS_DT) {
                touched |= net.step_with_ball(&spec, &mut b, &bs, PHYSICS_DT).contacts > 0;
            } else {
                net.step(&spec, PHYSICS_DT);
            }
            crate::contact::ground(&mut b, &bs, &turf, PHYSICS_DT);
            if b.pos.x > back_limit || !b.is_finite() || !net.is_finite() {
                escaped = true;
                break;
            }
            max_x = max_x.max(b.pos.x);
            if touched {
                back_speed = back_speed.max(-b.vel.x);
            }
        }
        (b, net, escaped, (max_x, back_speed))
    }

    #[test]
    fn shots_never_pass_through_the_net() {
        let mut rng = crate::math::Rng::new(77);
        for i in 0..24 {
            let speed = 10.0 + (i as f32 / 23.0) * 25.0; // 10 .. 35 m/s
            let start = v3(
                rng.range(12.0, 18.0),
                rng.range(0.2, 1.0),
                rng.range(-4.0, 4.0),
            );
            let target = v3(20.5, rng.range(0.2, 1.8), rng.range(-1.3, 1.3));
            let dir = (target - start).normalize_or(Vec3::X);
            let (b, _, escaped, (max_x, back)) =
                shoot_track(start, dir * speed + v3(0.0, 1.0, 0.0), 3.0);
            assert!(!escaped, "shot {i} at {speed} m/s escaped: {:?}", b.pos);
            assert!(max_x > 20.3, "shot {i} never went in: {max_x}");
            // the net kills the ball: little spring-back towards the pitch
            assert!(
                back < 0.25 * speed + 0.5,
                "shot {i}: rebound {back} from {speed} m/s"
            );
        }
    }

    #[test]
    fn net_slows_the_ball_and_bulges() {
        let (b, net, escaped) = shoot(v3(17.0, 1.0, 0.0), v3(30.0, 0.5, 0.0), 0.25);
        assert!(!escaped);
        assert!(b.vel.length() < 15.0, "ball still at {:?}", b.vel);
        let max_x = net.pos.iter().fold(f32::MIN, |m, p| m.max(p.x));
        assert!(max_x > 21.0 + 0.1, "net should bulge, max x {max_x}");
    }

    #[test]
    fn side_netting_from_outside_keeps_ball_outside() {
        // Ball behind the goal line coming at the side netting from outside.
        let (b, _, escaped) = shoot(v3(20.5, 0.6, 4.0), v3(0.0, 1.5, -14.0), 1.5);
        assert!(!escaped);
        assert!(
            b.pos.z > 1.54,
            "ball got inside the goal through the side netting: {:?}",
            b.pos
        );
    }

    #[test]
    fn net_returns_to_rest_after_a_goal() {
        let (_, mut net, _) = shoot(v3(16.0, 0.5, 0.5), v3(25.0, 1.5, 0.0), 1.0);
        let spec = NetSpec::default();
        for _ in 0..(240 * 6) {
            net.step(&spec, PHYSICS_DT);
        }
        assert!(net.sleeping || net.max_speed(PHYSICS_DT) < 0.05);
    }
}
