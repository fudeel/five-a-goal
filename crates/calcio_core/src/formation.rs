//! Team line-ups for every team size (1–5). Positions are in *normalised attack
//! coordinates*: `x` from −1 (own goal line) to +1 (opponent goal line), `z` from
//! −1 to +1 across the pitch. [`to_world`] maps them onto the pitch for a team.

use crate::math::{v3, Vec3};
use crate::pitch::{Pitch, Side};
use crate::player::Role;

/// One position in a line-up.
#[derive(Clone, Copy, Debug)]
pub struct Slot {
    pub role: Role,
    pub number: u8,
    /// Kick-off / base position (normalised).
    pub base: Vec3,
}

const fn slot(role: Role, number: u8, x: f32, z: f32) -> Slot {
    Slot {
        role,
        number,
        base: Vec3 { x, y: 0.0, z },
    }
}

/// Line-up for `size` players (1..=5). Index 0 is the goalkeeper when there is one.
pub fn lineup(size: usize) -> Vec<Slot> {
    match size.clamp(1, 5) {
        1 => vec![slot(Role::Pivot, 10, -0.25, 0.0)],
        2 => vec![
            slot(Role::Defender, 4, -0.55, -0.15),
            slot(Role::Pivot, 10, -0.2, 0.2),
        ],
        3 => vec![
            slot(Role::Goalkeeper, 1, -0.92, 0.0),
            slot(Role::Defender, 4, -0.5, -0.2),
            slot(Role::Pivot, 10, -0.18, 0.25),
        ],
        4 => vec![
            slot(Role::Goalkeeper, 1, -0.92, 0.0),
            slot(Role::Defender, 4, -0.55, 0.0),
            slot(Role::Winger, 7, -0.32, 0.55),
            slot(Role::Pivot, 10, -0.15, -0.3),
        ],
        _ => vec![
            slot(Role::Goalkeeper, 1, -0.92, 0.0),
            slot(Role::Defender, 4, -0.58, 0.0),
            slot(Role::Winger, 7, -0.35, 0.6),
            slot(Role::Winger, 11, -0.35, -0.6),
            slot(Role::Pivot, 9, -0.15, 0.0),
        ],
    }
}

/// Normalised attack coordinates → world position for a team attacking `attack`.
pub fn to_world(pitch: &Pitch, attack: Side, n: Vec3) -> Vec3 {
    let s = attack.sign();
    v3(
        s * n.x * pitch.half_length(),
        0.0,
        n.z * pitch.half_width() * 0.85,
    )
}

/// World position → normalised attack coordinates.
pub fn to_norm(pitch: &Pitch, attack: Side, p: Vec3) -> Vec3 {
    let s = attack.sign();
    v3(
        s * p.x / pitch.half_length(),
        0.0,
        p.z / (pitch.half_width() * 0.85),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lineups_have_the_right_size_and_keepers() {
        for n in 1..=5 {
            let l = lineup(n);
            assert_eq!(l.len(), n);
            let gks = l.iter().filter(|s| s.role == Role::Goalkeeper).count();
            assert_eq!(gks, if n >= 3 { 1 } else { 0 });
            // everybody starts in their own half
            assert!(l.iter().all(|s| s.base.x < 0.0));
        }
    }

    #[test]
    fn mapping_roundtrip() {
        let p = Pitch::new();
        for side in Side::BOTH {
            let n = v3(-0.5, 0.0, 0.4);
            let w = to_world(&p, side, n);
            assert!(to_norm(&p, side, w).distance(n) < 1e-5);
            // own half is opposite to the attacked goal
            assert!(w.x * side.sign() < 0.0);
        }
    }
}
