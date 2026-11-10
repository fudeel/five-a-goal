//! CPU lighting baked into vertex colours: hemisphere ambient + directional
//! lights (floodlight key/fill) + Blinn-Phong specular, with a soft tone curve.

use super::mesh::{Rgba, Vtx};
use calcio_core::math::{v3, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct DirLight {
    /// Unit direction pointing *towards* the light.
    pub dir: Vec3,
    /// Linear colour × intensity.
    pub color: [f32; 3],
}

#[derive(Clone, Debug)]
pub struct Lighting {
    pub lights: Vec<DirLight>,
    /// Ambient from above (linear).
    pub sky: [f32; 3],
    /// Ambient bounced from the ground (linear).
    pub ground: [f32; 3],
    /// Camera position (for specular).
    pub eye: Vec3,
    pub exposure: f32,
    pub shininess: f32,
}

#[inline]
fn to_linear(c: f32) -> f32 {
    c * c
}
#[inline]
fn to_srgb(c: f32) -> f32 {
    c.max(0.0).sqrt()
}

impl Lighting {
    /// Evening match under floodlights: warm key light, cool fill, dusky ambient.
    pub fn floodlit(eye: Vec3) -> Self {
        Self {
            lights: vec![
                DirLight {
                    dir: v3(-0.45, 0.8, 0.40).normalize_or_zero(),
                    color: [1.25, 1.18, 1.05],
                },
                DirLight {
                    dir: v3(0.55, 0.7, -0.45).normalize_or_zero(),
                    color: [0.42, 0.46, 0.55],
                },
            ],
            sky: [0.30, 0.34, 0.46],
            ground: [0.12, 0.16, 0.10],
            eye,
            exposure: 1.35,
            shininess: 40.0,
        }
    }

    /// Main light direction (used for shadow directions).
    pub fn key_dir(&self) -> Vec3 {
        self.lights.first().map(|l| l.dir).unwrap_or(Vec3::Y)
    }

    /// Lit sRGB colour for a vertex.
    pub fn shade(&self, v: &Vtx) -> Rgba {
        if v.emit >= 1.0 {
            return v.c;
        }
        let alb = [to_linear(v.c[0]), to_linear(v.c[1]), to_linear(v.c[2])];
        let view = (self.eye - v.p).normalize_or(Vec3::Y);
        let mut n = v.n;
        if v.two_sided && n.dot(view) < 0.0 {
            n = -n;
        }
        let hemi_t = 0.5 + 0.5 * n.y;
        let mut diff = [
            self.ground[0] + (self.sky[0] - self.ground[0]) * hemi_t,
            self.ground[1] + (self.sky[1] - self.ground[1]) * hemi_t,
            self.ground[2] + (self.sky[2] - self.ground[2]) * hemi_t,
        ];
        let mut spec = [0.0f32; 3];
        for l in &self.lights {
            let ndl = n.dot(l.dir);
            let ndl = if v.two_sided { ndl.abs() * 0.85 } else { ndl };
            if ndl > 0.0 {
                for k in 0..3 {
                    diff[k] += l.color[k] * ndl;
                }
                if v.spec > 0.0 {
                    let h = (l.dir + view).normalize_or(n);
                    let s = n.dot(h).max(0.0).powf(self.shininess) * v.spec;
                    for k in 0..3 {
                        spec[k] += l.color[k] * s;
                    }
                }
            }
        }
        let mut out = [0.0f32; 4];
        for k in 0..3 {
            let lin = (alb[k] * diff[k] + spec[k]) * self.exposure;
            // Soft shoulder so highlights don't clip harshly.
            let mapped = 1.0 - (-lin).exp();
            let lit = to_srgb(mapped);
            out[k] = lit + (v.c[k] - lit) * v.emit;
        }
        out[3] = v.c[3];
        out
    }
}
