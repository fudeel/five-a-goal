//! A blendable pose in *character space* (x = right, y = up, z = forward, origin
//! on the ground under the pelvis). Angles are radians.
//!
//! Legs are described by ankle targets + knee pole directions (solved with IK),
//! arms by anatomical channels or, optionally, hand targets (IK).

use super::body::Body;
use calcio_core::math::{v3, Vec3};

/// Rotation triple applied in the parent's frame: `x` = pitch (positive leans
/// forward), `y` = yaw (positive turns right), `z` = roll (positive tilts right).
pub type Euler = Vec3;

/// Arm channels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmPose {
    /// Forward raise of the upper arm.
    pub flex: f32,
    /// Sideways raise of the upper arm (away from the body).
    pub abduct: f32,
    /// Rotation of the upper arm about its own axis (turns the elbow plane).
    pub twist: f32,
    /// Elbow bend.
    pub elbow: f32,
    /// Hand target (character space) and how much it overrides the channels.
    pub target: Vec3,
    pub target_w: f32,
}

impl ArmPose {
    pub const RELAXED: ArmPose =
        ArmPose { flex: 0.08, abduct: 0.12, twist: 0.0, elbow: 0.28, target: Vec3::ZERO, target_w: 0.0 };

    pub fn lerp(&self, o: &ArmPose, t: f32) -> ArmPose {
        let l = |a: f32, b: f32| a + (b - a) * t;
        ArmPose {
            flex: l(self.flex, o.flex),
            abduct: l(self.abduct, o.abduct),
            twist: l(self.twist, o.twist),
            elbow: l(self.elbow, o.elbow),
            target: self.target.lerp(o.target, t),
            target_w: l(self.target_w, o.target_w),
        }
    }
}

/// Full-body pose. Index 0 = left side, 1 = right side.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Whole-body rotation about the root (dives, slides, lying on the ground).
    pub root_rot: Euler,
    /// Extra root translation (character space), e.g. lifting the body in a jump.
    pub root_offset: Vec3,
    /// Pelvis centre (character space).
    pub pelvis: Vec3,
    pub pelvis_rot: Euler,
    /// Lower spine relative to the pelvis, upper spine relative to the lower.
    pub spine: Euler,
    pub chest: Euler,
    /// Head relative to the chest.
    pub head: Euler,
    /// Ankle targets (character space).
    pub feet: [Vec3; 2],
    /// Foot pitch (positive = toes up) and yaw (positive = toes turned outward).
    pub foot_pitch: [f32; 2],
    pub foot_yaw: [f32; 2],
    /// Extra absolute foot heading (positive = turned right), e.g. a planted foot
    /// that hasn't followed the body's rotation yet.
    pub foot_turn: [f32; 2],
    /// Where the knees point (character space direction).
    pub knee_dir: [Vec3; 2],
    pub arms: [ArmPose; 2],
}

impl Pose {
    /// Relaxed standing pose (knees softly bent).
    pub fn stand(b: &Body) -> Pose {
        let hw = b.hip_half_width;
        let ankle = b.ankle_height;
        Pose {
            root_rot: Vec3::ZERO,
            root_offset: Vec3::ZERO,
            pelvis: v3(0.0, b.hip_height + b.pelvis_above_hips() - 0.015 * b.scale, 0.0),
            pelvis_rot: Vec3::ZERO,
            spine: v3(0.02, 0.0, 0.0),
            chest: v3(0.0, 0.0, 0.0),
            head: v3(0.03, 0.0, 0.0),
            feet: [v3(-hw - 0.03, ankle, 0.0), v3(hw + 0.03, ankle, 0.0)],
            foot_pitch: [0.0; 2],
            foot_yaw: [0.12, 0.12],
            foot_turn: [0.0; 2],
            knee_dir: [v3(-0.1, 0.0, 1.0), v3(0.1, 0.0, 1.0)],
            arms: [ArmPose::RELAXED; 2],
        }
    }

    /// Linear blend towards `o` by `t` (0 = self, 1 = o).
    pub fn lerp(&self, o: &Pose, t: f32) -> Pose {
        let l = |a: f32, b: f32| a + (b - a) * t;
        Pose {
            root_rot: self.root_rot.lerp(o.root_rot, t),
            root_offset: self.root_offset.lerp(o.root_offset, t),
            pelvis: self.pelvis.lerp(o.pelvis, t),
            pelvis_rot: self.pelvis_rot.lerp(o.pelvis_rot, t),
            spine: self.spine.lerp(o.spine, t),
            chest: self.chest.lerp(o.chest, t),
            head: self.head.lerp(o.head, t),
            feet: [self.feet[0].lerp(o.feet[0], t), self.feet[1].lerp(o.feet[1], t)],
            foot_pitch: [l(self.foot_pitch[0], o.foot_pitch[0]), l(self.foot_pitch[1], o.foot_pitch[1])],
            foot_yaw: [l(self.foot_yaw[0], o.foot_yaw[0]), l(self.foot_yaw[1], o.foot_yaw[1])],
            foot_turn: [l(self.foot_turn[0], o.foot_turn[0]), l(self.foot_turn[1], o.foot_turn[1])],
            knee_dir: [
                self.knee_dir[0].lerp(o.knee_dir[0], t).normalize_or(o.knee_dir[0]),
                self.knee_dir[1].lerp(o.knee_dir[1], t).normalize_or(o.knee_dir[1]),
            ],
            arms: [self.arms[0].lerp(&o.arms[0], t), self.arms[1].lerp(&o.arms[1], t)],
        }
    }

    /// Lowers the pelvis just enough for the legs to reach feet that are on
    /// (or about to touch) the ground; `contact[i]` weights foot `i` (0..1).
    /// Feet in the air are left to the knees.
    pub fn fit_legs(&mut self, b: &Body, contact: [f32; 2]) {
        let q = super::rig::euler(self.pelvis_rot);
        let reach = (b.thigh + b.shin) * 0.995;
        let mut drop = 0.0f32;
        for i in 0..2 {
            if contact[i] <= 0.0 {
                continue;
            }
            let s = if i == 0 { -1.0 } else { 1.0 };
            let hip = self.pelvis + q.rotate(v3(s * b.hip_half_width, -b.pelvis_above_hips(), 0.0));
            let foot = self.feet[i];
            let hd = (v3(hip.x - foot.x, 0.0, hip.z - foot.z)).length();
            let max_y = foot.y + (reach * reach - hd * hd).max(0.0).sqrt();
            drop = drop.max((hip.y - max_y).max(0.0) * contact[i]);
        }
        self.pelvis.y -= drop.min(0.22 * b.scale);
    }

    pub fn is_finite(&self) -> bool {
        self.pelvis.is_finite()
            && self.feet.iter().all(|f| f.is_finite())
            && self.root_rot.is_finite()
            && self.spine.is_finite()
            && self.chest.is_finite()
    }
}
