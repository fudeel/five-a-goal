//! Body measurements of a footballer (metres). Proportions follow adult male
//! anthropometry for a 1.80 m athlete, scaled per player for variety.

/// Segment lengths and widths of one player.
#[derive(Clone, Copy, Debug)]
pub struct Body {
    /// Overall height scale (1.0 = 1.80 m).
    pub scale: f32,
    /// Width / muscle scale (1.0 = average athletic build).
    pub build: f32,
    /// Height of the hip joints with straight legs (ankle + thigh + shin).
    pub hip_height: f32,
    pub hip_half_width: f32,
    pub thigh: f32,
    pub shin: f32,
    /// Ankle joint height above the sole.
    pub ankle_height: f32,
    /// Heel behind the ankle / toe in front of the ankle.
    pub heel: f32,
    pub toe: f32,
    /// Pelvis centre → base of the rib cage.
    pub spine: f32,
    /// Base of the rib cage → base of the neck.
    pub chest: f32,
    pub neck: f32,
    /// Neck base → centre of the head.
    pub head_up: f32,
    pub shoulder_half_width: f32,
    /// Shoulder joints below the neck base.
    pub shoulder_drop: f32,
    pub upper_arm: f32,
    pub forearm: f32,
    pub hand: f32,
}

impl Body {
    /// Average 1.80 m footballer.
    pub fn standard() -> Self {
        let (thigh, shin, ankle_height) = (0.445, 0.43, 0.072);
        Self {
            scale: 1.0,
            build: 1.0,
            hip_height: ankle_height + thigh + shin,
            hip_half_width: 0.09,
            thigh,
            shin,
            ankle_height,
            heel: 0.065,
            toe: 0.195,
            spine: 0.30,
            chest: 0.255,
            neck: 0.062,
            head_up: 0.105,
            shoulder_half_width: 0.185,
            shoulder_drop: 0.05,
            upper_arm: 0.315,
            forearm: 0.265,
            hand: 0.19,
        }
    }

    /// Scaled body: `scale` multiplies lengths, `build` multiplies widths.
    pub fn scaled(scale: f32, build: f32) -> Self {
        let s = Self::standard();
        Self {
            scale,
            build,
            hip_height: s.hip_height * scale,
            hip_half_width: s.hip_half_width * scale * build,
            thigh: s.thigh * scale,
            shin: s.shin * scale,
            ankle_height: s.ankle_height * scale,
            heel: s.heel * scale,
            toe: s.toe * scale,
            spine: s.spine * scale,
            chest: s.chest * scale,
            neck: s.neck * scale,
            head_up: s.head_up * scale,
            shoulder_half_width: s.shoulder_half_width * scale * build,
            shoulder_drop: s.shoulder_drop * scale,
            upper_arm: s.upper_arm * scale,
            forearm: s.forearm * scale,
            hand: s.hand * scale,
        }
    }

    /// Deterministic variety from a player id: heights 1.70–1.92 m, slim to stocky.
    pub fn for_player(id: usize) -> Self {
        let mut h = (id as u32).wrapping_mul(0x9E37_79B9) ^ 0x5bd1_e995;
        h ^= h >> 15;
        h = h.wrapping_mul(0x2c1b_3c6d);
        h ^= h >> 12;
        let a = (h & 0xffff) as f32 / 65535.0;
        let b = ((h >> 16) & 0xffff) as f32 / 65535.0;
        Self::scaled(0.945 + 0.12 * a, 0.92 + 0.16 * b)
    }

    /// Pelvis-centre height above the hip joints.
    pub fn pelvis_above_hips(&self) -> f32 {
        0.035 * self.scale
    }

    /// Standing height (sole to top of the head).
    pub fn height(&self) -> f32 {
        self.hip_height + self.pelvis_above_hips() + self.spine + self.chest + self.neck + self.head_up + 0.115 * self.scale
    }
}
