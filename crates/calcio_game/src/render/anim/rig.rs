//! Skeleton rig: forward kinematics from a [`Pose`] to world-space joints and
//! frames, and two-bone IK for legs and arms.

use super::body::Body;
use super::pose::{Euler, Pose};
use calcio_core::math::{clamp, v3, Quat, Vec3};

/// An orthonormal frame: origin + axes (x = right, y = up, z = forward).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub o: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

impl Frame {
    /// Character frame of a player standing at `pos` facing `heading`.
    pub fn character(pos: Vec3, heading: f32) -> Frame {
        let z = Vec3::from_heading(heading);
        let x = z.cross(Vec3::Y).normalize_or(Vec3::Z);
        Frame { o: pos, x, y: Vec3::Y, z }
    }

    #[inline]
    pub fn point(&self, p: Vec3) -> Vec3 {
        self.o + self.x * p.x + self.y * p.y + self.z * p.z
    }

    #[inline]
    pub fn dir(&self, v: Vec3) -> Vec3 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    /// Coordinates of world point `p` in this frame.
    #[inline]
    pub fn to_local(&self, p: Vec3) -> Vec3 {
        let d = p - self.o;
        v3(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    /// Coordinates of world direction `v` in this frame.
    #[inline]
    pub fn dir_to_local(&self, v: Vec3) -> Vec3 {
        v3(v.dot(self.x), v.dot(self.y), v.dot(self.z))
    }

    /// Same frame rotated by `q` expressed in its own coordinates.
    pub fn rotated(&self, q: Quat) -> Frame {
        Frame { o: self.o, x: self.dir(q.rotate(Vec3::X)), y: self.dir(q.rotate(Vec3::Y)), z: self.dir(q.rotate(Vec3::Z)) }
    }

    pub fn moved(&self, o: Vec3) -> Frame {
        Frame { o, ..*self }
    }
}

/// Quaternion for a pitch / yaw / roll triple (see [`Euler`]).
pub fn euler(e: Euler) -> Quat {
    let yaw = Quat::from_axis_angle(Vec3::Y, e.y);
    let pitch = Quat::from_axis_angle(Vec3::X, e.x);
    let roll = Quat::from_axis_angle(Vec3::Z, -e.z);
    yaw * pitch * roll
}

/// Two-bone IK: from `root` towards `target` with bone lengths `l1`, `l2`; the
/// middle joint bends towards `pole`. Returns (middle joint, end joint).
pub fn two_bone_ik(root: Vec3, target: Vec3, l1: f32, l2: f32, pole: Vec3) -> (Vec3, Vec3) {
    let to = target - root;
    let dir = to.normalize_or(v3(0.0, -1.0, 0.0));
    let d = clamp(to.length(), (l1 - l2).abs() + 1e-3, (l1 + l2) * 0.9995);
    let end = root + dir * d;
    let cos_a = clamp((l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d), -1.0, 1.0);
    let sin_a = (1.0 - cos_a * cos_a).max(0.0).sqrt();
    let perp = (pole - dir * pole.dot(dir)).normalize_or(dir.any_perpendicular());
    let mid = root + dir * (cos_a * l1) + perp * (sin_a * l1);
    (mid, end)
}

/// World-space skeleton (index 0 = left, 1 = right).
#[derive(Clone, Copy, Debug)]
pub struct Skeleton {
    pub body: Body,
    /// Character frame after the whole-body rotation.
    pub root: Frame,
    /// Pelvis frame (origin at the hip-joint level, centre).
    pub pelvis: Frame,
    /// Lower spine (origin = pelvis centre) and chest (origin = base of the rib cage).
    pub spine: Frame,
    pub chest: Frame,
    /// Head frame, origin at the centre of the head.
    pub head: Frame,
    pub neck: Vec3,
    pub hip: [Vec3; 2],
    pub knee: [Vec3; 2],
    pub ankle: [Vec3; 2],
    pub heel: [Vec3; 2],
    pub toe: [Vec3; 2],
    /// Foot frames (origin at the ankle, z towards the toes, y up from the sole).
    pub foot: [Frame; 2],
    pub knee_pole: [Vec3; 2],
    pub shoulder: [Vec3; 2],
    pub elbow: [Vec3; 2],
    pub wrist: [Vec3; 2],
    /// Direction the elbows point (for orienting arm meshes).
    pub elbow_pole: [Vec3; 2],
}

impl Skeleton {
    /// Lowest point of the body above the ground (soles and everything else).
    pub fn lowest_point(&self) -> f32 {
        let mut m = f32::MAX;
        for i in 0..2 {
            m = m.min(self.heel[i].y).min(self.toe[i].y).min(self.knee[i].y).min(self.wrist[i].y);
        }
        m.min(self.pelvis.o.y).min(self.head.o.y)
    }

    pub fn is_finite(&self) -> bool {
        self.pelvis.o.is_finite()
            && self.head.o.is_finite()
            && (0..2).all(|i| {
                self.knee[i].is_finite() && self.ankle[i].is_finite() && self.elbow[i].is_finite() && self.wrist[i].is_finite()
            })
    }
}

/// Forward kinematics. `base` is the character frame on the ground.
pub fn fk(pose: &Pose, b: &Body, base: &Frame) -> Skeleton {
    let root = base.rotated(euler(pose.root_rot)).moved(base.o + base.dir(pose.root_offset));
    // Pelvis and spine chain.
    let pelvis = root.rotated(euler(pose.pelvis_rot)).moved(root.point(pose.pelvis));
    let spine = pelvis.rotated(euler(pose.spine));
    let chest = spine.rotated(euler(pose.chest)).moved(spine.point(v3(0.0, b.spine, -0.01 * b.scale)));
    let neck = chest.point(v3(0.0, b.chest, -0.025 * b.scale));
    let head_f = chest.rotated(euler(pose.head));
    let head = head_f.moved(neck + head_f.dir(v3(0.0, b.neck + b.head_up, 0.02 * b.scale)));

    // Legs (IK to ankle targets).
    let mut hip = [Vec3::ZERO; 2];
    let mut knee = [Vec3::ZERO; 2];
    let mut ankle = [Vec3::ZERO; 2];
    let mut heel = [Vec3::ZERO; 2];
    let mut toe = [Vec3::ZERO; 2];
    let mut foot = [root; 2];
    let mut knee_pole = [Vec3::Z; 2];
    for i in 0..2 {
        let s = if i == 0 { -1.0 } else { 1.0 };
        hip[i] = pelvis.point(v3(s * b.hip_half_width, -b.pelvis_above_hips(), 0.0));
        let target = root.point(pose.feet[i]);
        let pole = root.dir(pose.knee_dir[i]).normalize_or(root.z);
        let (k, a) = two_bone_ik(hip[i], target, b.thigh, b.shin, pole);
        knee[i] = k;
        ankle[i] = a;
        knee_pole[i] = pole;
        // Foot: yaw outwards, then pitch (toes up positive), in the root frame.
        let yaw = Quat::from_axis_angle(Vec3::Y, s * pose.foot_yaw[i] + pose.foot_turn[i]);
        let pitch = Quat::from_axis_angle(Vec3::X, -pose.foot_pitch[i]);
        let f = root.rotated(yaw * pitch).moved(a);
        heel[i] = f.point(v3(0.0, -b.ankle_height, -b.heel));
        toe[i] = f.point(v3(0.0, -b.ankle_height * 0.8, b.toe));
        foot[i] = f;
    }

    // Arms: channels, optionally pulled towards hand targets with IK.
    let mut shoulder = [Vec3::ZERO; 2];
    let mut elbow = [Vec3::ZERO; 2];
    let mut wrist = [Vec3::ZERO; 2];
    let mut elbow_pole = [Vec3::ZERO; 2];
    let down = v3(0.0, -1.0, 0.0);
    for i in 0..2 {
        let s = if i == 0 { -1.0 } else { 1.0 };
        let arm = &pose.arms[i];
        shoulder[i] = chest.point(v3(s * b.shoulder_half_width, b.chest - b.shoulder_drop, -0.03 * b.scale));
        let q_upper = Quat::from_axis_angle(Vec3::Z, s * arm.abduct)
            * Quat::from_axis_angle(Vec3::X, -arm.flex)
            * Quat::from_axis_angle(Vec3::Y, s * arm.twist);
        let q_fore = q_upper * Quat::from_axis_angle(Vec3::X, -arm.elbow);
        let mut e = shoulder[i] + chest.dir(q_upper.rotate(down)) * b.upper_arm;
        let mut w = e + chest.dir(q_fore.rotate(down)) * b.forearm;
        let pole = chest.dir(v3(s * 0.35, -0.25, -1.0)).normalize_or(-chest.z);
        if arm.target_w > 0.0 {
            let t = root.point(arm.target);
            let (ie, iw) = two_bone_ik(shoulder[i], t, b.upper_arm, b.forearm, pole);
            let k = clamp(arm.target_w, 0.0, 1.0);
            e = e.lerp(ie, k);
            w = w.lerp(iw, k);
            e = shoulder[i] + (e - shoulder[i]).normalize_or(down) * b.upper_arm;
            w = e + (w - e).normalize_or(down) * b.forearm;
        }
        elbow[i] = e;
        wrist[i] = w;
        elbow_pole[i] = pole;
    }

    Skeleton {
        body: *b,
        root,
        pelvis,
        spine,
        chest,
        head,
        neck,
        hip,
        knee,
        ankle,
        heel,
        toe,
        foot,
        knee_pole,
        shoulder,
        elbow,
        wrist,
        elbow_pole,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn ik_reaches_reachable_targets_and_bends_towards_pole() {
        let root = v3(0.0, 1.0, 0.0);
        let target = v3(0.0, 0.3, 0.2);
        let (mid, end) = two_bone_ik(root, target, 0.45, 0.43, Vec3::Z);
        assert!(end.distance(target) < 1e-4);
        assert!(close(mid.distance(root), 0.45, 1e-4));
        assert!(close(end.distance(mid), 0.43, 1e-4));
        assert!(mid.z > 0.1, "knee should point forward: {mid:?}");
        // Unreachable target: straight leg towards it.
        let (mid, end) = two_bone_ik(root, v3(0.0, -2.0, 0.0), 0.45, 0.43, Vec3::Z);
        assert!(close(end.distance(root), 0.88, 1e-3));
        assert!(mid.is_finite());
        // Degenerate pole along the bone.
        let (mid, _) = two_bone_ik(root, v3(0.0, 0.4, 0.0), 0.45, 0.43, v3(0.0, -1.0, 0.0));
        assert!(mid.is_finite());
    }

    #[test]
    fn standing_fk_has_sensible_anatomy() {
        let b = Body::standard();
        let pose = Pose::stand(&b);
        let base = Frame::character(v3(3.0, 0.0, -2.0), 0.0);
        let s = fk(&pose, &b, &base);
        assert!(s.is_finite());
        for i in 0..2 {
            assert!(close(s.hip[i].distance(s.knee[i]), b.thigh, 1e-3));
            assert!(close(s.knee[i].distance(s.ankle[i]), b.shin, 1e-3));
            assert!(close(s.shoulder[i].distance(s.elbow[i]), b.upper_arm, 1e-3));
            assert!(close(s.elbow[i].distance(s.wrist[i]), b.forearm, 1e-3));
            // soles on the ground
            assert!(s.heel[i].y.abs() < 0.02 && s.toe[i].y.abs() < 0.03, "{:?} {:?}", s.heel[i], s.toe[i]);
            // toes point forward (+x for heading 0)
            assert!(s.toe[i].x > s.heel[i].x);
        }
        // left side is on the left (−z for a player facing +x)
        assert!(s.hip[0].z < s.hip[1].z && s.shoulder[0].z < s.shoulder[1].z);
        // head on top, about 1.8 m tall
        let top = s.head.o.y + 0.115;
        assert!(top > 1.76 && top < 1.84, "height {top}");
        assert!((b.height() - 1.8).abs() < 0.03, "body height {}", b.height());
        // shoulders ≈ 1.45–1.5 m
        assert!(s.shoulder[0].y > 1.42 && s.shoulder[0].y < 1.52, "shoulder {:?}", s.shoulder[0]);
        // hands hang near the thighs
        assert!(s.wrist[0].y > 0.72 && s.wrist[0].y < 0.92, "wrist {:?}", s.wrist[0]);
    }

    #[test]
    fn pitch_yaw_roll_signs() {
        let base = Frame::character(Vec3::ZERO, 0.0); // forward = +x, right = +z
        let lean = base.rotated(euler(v3(0.3, 0.0, 0.0)));
        assert!(lean.y.x > 0.2, "pitch leans forward");
        let turn = base.rotated(euler(v3(0.0, 0.5, 0.0)));
        assert!(turn.z.z > 0.3, "yaw turns right");
        let tilt = base.rotated(euler(v3(0.0, 0.0, 0.3)));
        assert!(tilt.y.z > 0.2, "roll tilts right");
    }

    #[test]
    fn arm_channels_move_the_hand_the_right_way() {
        let b = Body::standard();
        let base = Frame::character(Vec3::ZERO, 0.0);
        let mut pose = Pose::stand(&b);
        pose.arms[1].flex = 1.4;
        pose.arms[1].elbow = 0.0;
        pose.arms[0].abduct = 1.4;
        pose.arms[0].flex = 0.0;
        let s = fk(&pose, &b, &base);
        assert!(s.wrist[1].x > s.shoulder[1].x + 0.4, "right arm raised forward");
        assert!(s.wrist[0].z < s.shoulder[0].z - 0.35, "left arm raised to the left side");
        // Hand target IK
        pose.arms[1].target = v3(0.1, 1.2, 0.45);
        pose.arms[1].target_w = 1.0;
        let s = fk(&pose, &b, &base);
        assert!(s.wrist[1].distance(base.point(v3(0.1, 1.2, 0.45))) < 0.01);
    }
}
