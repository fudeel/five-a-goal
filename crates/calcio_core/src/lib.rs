//! # calcio_core
//!
//! Everything that makes CalcioFive play: math, ball physics, goal-net cloth,
//! footballers, match rules and AI. No rendering, no dependencies, fully
//! deterministic and unit-tested headless.
//!
//! World axes: **x** = pitch length (goals at x = ±20 m), **y** = up,
//! **z** = pitch width (touchlines at z = ±10 m). SI units throughout.

pub mod ai;
pub mod ball;
pub mod contact;
pub mod control;
pub mod events;
pub mod formation;
pub mod keeper;
pub mod kick;
pub mod math;
pub mod net;
pub mod pitch;
pub mod player;
pub mod rules;
pub mod time;
pub mod world;

pub use ball::{Ball, BallSpec};
pub use events::GameEvent;
pub use math::{v3, Quat, Rng, Vec3};
pub use pitch::{Pitch, Side};
pub use player::{Player, PlayerSpec, Role};
pub use rules::{Difficulty, Match, MatchConfig, MatchEvent, Phase};
pub use time::FixedClock;
pub use world::World;

/// Physics update rate (Hz). Ball, net and players are all stepped at this rate.
pub const PHYSICS_HZ: u32 = 240;
/// Physics timestep (s).
pub const PHYSICS_DT: f32 = 1.0 / PHYSICS_HZ as f32;
