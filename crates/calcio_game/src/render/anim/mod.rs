//! Skeletal animation: body proportions, poses, rig (FK/IK), gaits and the
//! per-player animator.

pub mod actions;
pub mod animator;
pub mod body;
pub mod locomotion;
pub mod moves;
pub mod pose;
pub mod rig;

pub use animator::Animator;
pub use rig::Skeleton;
