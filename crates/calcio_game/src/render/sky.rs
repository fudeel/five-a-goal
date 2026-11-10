//! Dusk sky dome (unlit vertex-colour gradient).

use super::mesh::{hex, mix, Mat, MeshBuilder, Rgba};
use calcio_core::math::v3;

/// Colour of the sky at a given elevation (-1 = down, 0 = horizon, 1 = zenith).
pub fn sky_color(el: f32) -> Rgba {
    let ground = hex(0x0c0f14);
    let haze = hex(0x3a3340);
    let glow = hex(0xe39467);
    let violet = hex(0x6c5a8e);
    let blue = hex(0x26386a);
    let zenith = hex(0x0a1430);
    if el < -0.02 {
        mix(haze, ground, ((-el - 0.02) / 0.15).min(1.0))
    } else if el < 0.05 {
        mix(haze, glow, (el + 0.02) / 0.07)
    } else if el < 0.16 {
        mix(glow, violet, (el - 0.05) / 0.11)
    } else if el < 0.42 {
        mix(violet, blue, (el - 0.16) / 0.26)
    } else {
        mix(blue, zenith, ((el - 0.42) / 0.58).min(1.0))
    }
}

/// Builds the dome geometry (radius ~380 m around the pitch).
pub fn build() -> MeshBuilder {
    let mut b = MeshBuilder::new();
    let segs = 48;
    let rings = 28;
    let r = 380.0;
    let el_min = -0.25f32;
    let m = Mat::unlit([1.0; 4]);
    for j in 0..rings {
        let e0 = el_min + (1.0 - el_min) * j as f32 / rings as f32;
        let e1 = el_min + (1.0 - el_min) * (j + 1) as f32 / rings as f32;
        let (c0, c1) = (sky_color(e0), sky_color(e1));
        let (a0, a1) = (e0.asin(), e1.asin());
        for i in 0..segs {
            let t0 = i as f32 / segs as f32 * std::f32::consts::TAU;
            let t1 = (i + 1) as f32 / segs as f32 * std::f32::consts::TAU;
            let p = |t: f32, a: f32| v3(t.cos() * a.cos() * r, a.sin() * r, t.sin() * a.cos() * r);
            b.quad_ex(
                [p(t0, a0), p(t1, a0), p(t1, a1), p(t0, a1)],
                [v3(0.0, -1.0, 0.0); 4],
                [c0, c0, c1, c1],
                &m,
            );
        }
    }
    b
}
