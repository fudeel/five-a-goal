//! Pitch, arena and goal geometry (futsal-size cage pitch).
//!
//! Goal lines are at x = ±L/2, touchlines at z = ±W/2. Lines are centred on these
//! coordinates. The goal at negative x belongs to [`Side::Left`], the one at
//! positive x to [`Side::Right`].

use crate::math::{v3, Vec3};

/// One end of the pitch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Side {
    /// Goal at negative x.
    Left,
    /// Goal at positive x.
    Right,
}

impl Side {
    /// -1 for Left, +1 for Right.
    #[inline]
    pub fn sign(self) -> f32 {
        match self {
            Side::Left => -1.0,
            Side::Right => 1.0,
        }
    }
    #[inline]
    pub fn opposite(self) -> Side {
        match self {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
        }
    }
    #[inline]
    pub fn index(self) -> usize {
        match self {
            Side::Left => 0,
            Side::Right => 1,
        }
    }
    pub const BOTH: [Side; 2] = [Side::Left, Side::Right];
}

/// Playing-area markings (metres).
#[derive(Clone, Debug)]
pub struct PitchSpec {
    pub length: f32,
    pub width: f32,
    pub line_width: f32,
    pub centre_circle_radius: f32,
    /// Radius of the penalty-area quarter circles, centred on the outside of each post.
    pub penalty_radius: f32,
    /// Distance of the penalty mark from the goal line.
    pub penalty_spot: f32,
    /// Distance of the second penalty mark from the goal line.
    pub second_penalty_spot: f32,
}

impl Default for PitchSpec {
    fn default() -> Self {
        Self {
            length: 40.0,
            width: 20.0,
            line_width: 0.08,
            centre_circle_radius: 3.0,
            penalty_radius: 6.0,
            penalty_spot: 6.0,
            second_penalty_spot: 10.0,
        }
    }
}

/// The enclosure around the pitch.
#[derive(Clone, Debug)]
pub struct ArenaSpec {
    /// Height of the solid rebound boards.
    pub board_height: f32,
    /// Distance from touchline to side boards.
    pub side_runoff: f32,
    /// Distance from goal line to end boards.
    pub end_runoff: f32,
    /// Visible height of the cage netting above the boards (collision is unbounded).
    pub cage_height: f32,
}

impl Default for ArenaSpec {
    fn default() -> Self {
        Self {
            board_height: 1.0,
            side_runoff: 1.0,
            end_runoff: 1.5,
            cage_height: 5.0,
        }
    }
}

/// Goal frame (metres). `width`/`height` are the inner mouth dimensions.
#[derive(Clone, Debug)]
pub struct GoalSpec {
    pub width: f32,
    pub height: f32,
    pub post_radius: f32,
    /// Net depth behind the goal line at crossbar height.
    pub depth_top: f32,
    /// Net depth behind the goal line at ground level.
    pub depth_bottom: f32,
    /// Radius of the thin support bars holding the net.
    pub support_radius: f32,
}

impl Default for GoalSpec {
    fn default() -> Self {
        Self {
            width: 3.0,
            height: 2.0,
            post_radius: 0.04,
            depth_top: 0.8,
            depth_bottom: 1.0,
            support_radius: 0.018,
        }
    }
}

/// What a frame collider is, for restitution and events.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FramePart {
    Post,
    Crossbar,
    Support,
}

/// A capsule collider (segment + radius).
#[derive(Clone, Copy, Debug)]
pub struct Capsule {
    pub a: Vec3,
    pub b: Vec3,
    pub radius: f32,
    pub part: FramePart,
}

/// Key points of one goal frame (centres of the bars).
#[derive(Clone, Copy, Debug)]
pub struct GoalFrame {
    pub side: Side,
    /// Goal line x coordinate.
    pub line_x: f32,
    /// Outward direction (+1 / -1 along x): from the pitch into the goal.
    pub out: f32,
    /// Post centre z (positive side); posts are at ±post_z.
    pub post_z: f32,
    /// Crossbar centre height.
    pub bar_y: f32,
    pub depth_top: f32,
    pub depth_bottom: f32,
}

impl GoalFrame {
    /// Bottom of a post (`s` = -1 or +1 for the z side).
    pub fn post_base(&self, s: f32) -> Vec3 {
        v3(self.line_x, 0.0, s * self.post_z)
    }
    /// Top of a post where it meets the crossbar.
    pub fn post_top(&self, s: f32) -> Vec3 {
        v3(self.line_x, self.bar_y, s * self.post_z)
    }
    /// Top back corner of the net support frame.
    pub fn back_top(&self, s: f32) -> Vec3 {
        v3(
            self.line_x + self.out * self.depth_top,
            self.bar_y,
            s * self.post_z,
        )
    }
    /// Bottom back corner of the net support frame.
    pub fn back_bottom(&self, s: f32) -> Vec3 {
        v3(
            self.line_x + self.out * self.depth_bottom,
            0.0,
            s * self.post_z,
        )
    }
}

/// Complete static geometry of the arena.
#[derive(Clone, Debug, Default)]
pub struct Pitch {
    pub spec: PitchSpec,
    pub arena: ArenaSpec,
    pub goal: GoalSpec,
}

impl Pitch {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn half_length(&self) -> f32 {
        self.spec.length * 0.5
    }
    #[inline]
    pub fn half_width(&self) -> f32 {
        self.spec.width * 0.5
    }
    /// x of the end boards (positive side).
    #[inline]
    pub fn board_x(&self) -> f32 {
        self.half_length() + self.arena.end_runoff
    }
    /// z of the side boards (positive side).
    #[inline]
    pub fn board_z(&self) -> f32 {
        self.half_width() + self.arena.side_runoff
    }
    /// x of the goal line on `side`.
    #[inline]
    pub fn goal_line_x(&self, side: Side) -> f32 {
        side.sign() * self.half_length()
    }
    /// Centre of the goal mouth on the ground.
    pub fn goal_centre(&self, side: Side) -> Vec3 {
        v3(self.goal_line_x(side), 0.0, 0.0)
    }

    pub fn frame(&self, side: Side) -> GoalFrame {
        let g = &self.goal;
        GoalFrame {
            side,
            line_x: self.goal_line_x(side),
            out: side.sign(),
            post_z: g.width * 0.5 + g.post_radius,
            bar_y: g.height + g.post_radius,
            depth_top: g.depth_top,
            depth_bottom: g.depth_bottom,
        }
    }

    /// Capsule colliders of one goal: 2 posts, crossbar, and the net support frame.
    pub fn goal_colliders(&self, side: Side) -> Vec<Capsule> {
        let f = self.frame(side);
        let pr = self.goal.post_radius;
        let sr = self.goal.support_radius;
        let mut v = Vec::with_capacity(10);
        for s in [-1.0, 1.0] {
            v.push(Capsule {
                a: f.post_base(s),
                b: f.post_top(s),
                radius: pr,
                part: FramePart::Post,
            });
        }
        v.push(Capsule {
            a: f.post_top(-1.0),
            b: f.post_top(1.0),
            radius: pr,
            part: FramePart::Crossbar,
        });
        for s in [-1.0, 1.0] {
            // top side bar, inclined back stanchion, ground side bar
            v.push(Capsule {
                a: f.post_top(s),
                b: f.back_top(s),
                radius: sr,
                part: FramePart::Support,
            });
            v.push(Capsule {
                a: f.back_top(s),
                b: f.back_bottom(s),
                radius: sr,
                part: FramePart::Support,
            });
            v.push(Capsule {
                a: f.post_base(s),
                b: f.back_bottom(s),
                radius: sr,
                part: FramePart::Support,
            });
        }
        v.push(Capsule {
            a: f.back_top(-1.0),
            b: f.back_top(1.0),
            radius: sr,
            part: FramePart::Support,
        });
        v.push(Capsule {
            a: f.back_bottom(-1.0),
            b: f.back_bottom(1.0),
            radius: sr,
            part: FramePart::Support,
        });
        v
    }

    /// Distance from the goal line of `side` into the pitch (negative = behind the line).
    #[inline]
    pub fn depth_from_goal_line(&self, side: Side, p: Vec3) -> f32 {
        self.half_length() - side.sign() * p.x
    }

    /// Futsal penalty area: quarter circles of radius 6 m centred on the outside of
    /// each post, joined by a straight line. Points behind the goal line count as inside.
    pub fn is_inside_penalty_area(&self, side: Side, p: Vec3) -> bool {
        let dx = self.depth_from_goal_line(side, p).max(0.0);
        let half_line = self.goal.width * 0.5 + 2.0 * self.goal.post_radius;
        let dz = (p.z.abs() - half_line).max(0.0);
        (dx * dx + dz * dz).sqrt() <= self.spec.penalty_radius
    }

    /// True if a ball of `radius` moving from `prev` to `curr` has just crossed
    /// wholly over the goal line of `side` between the posts and under the bar.
    pub fn goal_crossing(&self, side: Side, prev: Vec3, curr: Vec3, radius: f32) -> bool {
        let plane = self.half_length() + self.spec.line_width * 0.5 + radius;
        let a = side.sign() * prev.x;
        let b = side.sign() * curr.x;
        if !(a < plane && b >= plane) {
            return false;
        }
        let t = (plane - a) / (b - a);
        let hit = prev.lerp(curr, t);
        hit.z.abs() < self.goal.width * 0.5 && hit.y < self.goal.height && hit.y >= -0.01
    }

    /// Is the point inside the board enclosure (with a margin)?
    pub fn inside_arena(&self, p: Vec3, margin: f32) -> bool {
        p.x.abs() <= self.board_x() + margin
            && p.z.abs() <= self.board_z() + margin
            && p.y >= -margin
    }

    /// Which half a point is in.
    pub fn half_of(&self, p: Vec3) -> Side {
        if p.x < 0.0 {
            Side::Left
        } else {
            Side::Right
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dimensions() {
        let p = Pitch::new();
        assert_eq!(p.half_length(), 20.0);
        assert_eq!(p.half_width(), 10.0);
        assert_eq!(p.board_x(), 21.5);
        assert_eq!(p.board_z(), 11.0);
        assert_eq!(p.goal_line_x(Side::Left), -20.0);
        assert_eq!(Side::Left.opposite(), Side::Right);
    }

    #[test]
    fn goal_colliders_shape() {
        let p = Pitch::new();
        let c = p.goal_colliders(Side::Right);
        assert_eq!(c.iter().filter(|c| c.part == FramePart::Post).count(), 2);
        assert_eq!(
            c.iter().filter(|c| c.part == FramePart::Crossbar).count(),
            1
        );
        // all supports lie behind the goal line
        for k in c.iter().filter(|c| c.part == FramePart::Support) {
            assert!(k.a.x >= 20.0 - 1e-5 && k.b.x >= 20.0 - 1e-5);
            assert!(k.a.x <= 21.0 + 1e-5 && k.b.x <= 21.0 + 1e-5);
        }
        // inner mouth width is 3 m
        let posts: Vec<_> = c.iter().filter(|c| c.part == FramePart::Post).collect();
        let inner = (posts[0].a.z - posts[1].a.z).abs() - 2.0 * p.goal.post_radius;
        assert!((inner - 3.0).abs() < 1e-5);
    }

    #[test]
    fn penalty_area() {
        let p = Pitch::new();
        assert!(p.is_inside_penalty_area(Side::Left, v3(-17.0, 0.0, 0.0)));
        assert!(p.is_inside_penalty_area(Side::Left, v3(-14.1, 0.0, 1.0)));
        assert!(!p.is_inside_penalty_area(Side::Left, v3(-13.5, 0.0, 0.0)));
        // corner of the arc region: 6 m from the post outside at z = 1.58
        assert!(p.is_inside_penalty_area(Side::Right, v3(20.0, 0.0, 7.5)));
        assert!(!p.is_inside_penalty_area(Side::Right, v3(20.0, 0.0, 7.7)));
        assert!(!p.is_inside_penalty_area(Side::Right, v3(-17.0, 0.0, 0.0)));
        // behind the goal line counts as inside
        assert!(p.is_inside_penalty_area(Side::Right, v3(20.6, 0.0, 0.0)));
    }

    #[test]
    fn goal_crossing_detection() {
        let p = Pitch::new();
        let r = 0.11;
        // straight into the right goal
        assert!(p.goal_crossing(Side::Right, v3(20.1, 0.5, 0.0), v3(20.3, 0.5, 0.0), r));
        // not yet wholly over the line
        assert!(!p.goal_crossing(Side::Right, v3(19.9, 0.5, 0.0), v3(20.1, 0.5, 0.0), r));
        // wide of the post
        assert!(!p.goal_crossing(Side::Right, v3(20.1, 0.5, 1.8), v3(20.3, 0.5, 1.8), r));
        // over the bar
        assert!(!p.goal_crossing(Side::Right, v3(20.1, 2.3, 0.0), v3(20.3, 2.3, 0.0), r));
        // wrong goal / wrong direction
        assert!(!p.goal_crossing(Side::Left, v3(20.1, 0.5, 0.0), v3(20.3, 0.5, 0.0), r));
        assert!(!p.goal_crossing(Side::Right, v3(20.3, 0.5, 0.0), v3(20.1, 0.5, 0.0), r));
        // left goal
        assert!(p.goal_crossing(Side::Left, v3(-20.1, 0.2, -1.0), v3(-20.4, 0.2, -1.0), r));
    }

    #[test]
    fn arena_bounds() {
        let p = Pitch::new();
        assert!(p.inside_arena(v3(21.0, 0.5, 10.5), 0.0));
        assert!(!p.inside_arena(v3(22.0, 0.5, 0.0), 0.0));
        assert_eq!(p.half_of(v3(-1.0, 0.0, 0.0)), Side::Left);
    }
}
