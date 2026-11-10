//! Things that happened during simulation steps, for the camera, HUD and rules.

use crate::kick::KickKind;
use crate::pitch::{FramePart, Side};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GameEvent {
    /// Ball struck a post, the crossbar or a support bar.
    FrameHit {
        side: Side,
        part: FramePart,
        speed: f32,
    },
    /// Ball rebounded off the boards (or the cage above them).
    BoardHit { speed: f32, cage: bool },
    /// Ball bounced on the turf.
    GroundBounce { speed: f32 },
    /// Ball hit the net of the goal on `side` (from inside or outside).
    NetHit { side: Side, speed: f32 },
    /// A player kicked the ball.
    Kick {
        player: usize,
        kind: KickKind,
        speed: f32,
    },
    /// A player went into a slide tackle.
    Slide { player: usize },
    /// A sliding leg knocked the ball.
    SlideTouch { player: usize },
    /// `player` was tripped (by a slide) and went down.
    Trip { player: usize, by: usize },
    /// A player headed the ball where he aimed.
    Header { player: usize },
    /// A kick met nothing: the ball was no longer at the foot.
    Whiff { player: usize },
    /// The goalkeeper caught or parried the ball.
    Save { player: usize, caught: bool },
    /// A standing tackle was attempted.
    Tackle { player: usize, won: bool },
    /// A player touched the ball with the foot (dribble or first touch).
    Touch { player: usize, trap: bool },
    /// Ball bounced off a player's body.
    BodyHit { player: usize, speed: f32 },
    /// Ball wholly crossed the goal line into the goal on `side`.
    GoalScored { side: Side },
}
