//! Footballer rendering: the character model on the animated skeleton,
//! floodlight shadows and the human-player marker.

use super::light::Lighting;
use super::mesh::{hex, with_alpha, Mat, MeshBuilder};
use super::anim::Skeleton;
use super::character::Look;
use calcio_core::math::v3;
use calcio_core::player::{Player, PlayerSpec};

pub struct PlayerRenderer;

impl PlayerRenderer {
    pub fn build(&self, b: &mut MeshBuilder, p: &Player, s: &Skeleton) {
        super::character::build(b, s, &Look::of(p));
    }

    /// Floodlight shadows: a pool at the feet plus a stretched shadow per light.
    pub fn shadows(&self, b: &mut MeshBuilder, p: &Player, spec: &PlayerSpec, s: &Skeleton, light: &Lighting) {
        let m = Mat::unlit([0.0; 4]);
        let y0 = super::arena::LINE_Y + 0.006;
        // Contact shadow under each foot plus a soft pool under the pelvis.
        for i in 0..2 {
            let f = s.ankle[i];
            let h = (f.y - s.body.ankle_height).max(0.0);
            let a = 0.35 / (1.0 + 6.0 * h);
            let c = s.heel[i].lerp(s.toe[i], 0.45);
            let along = (s.toe[i] - s.heel[i]).flat().normalize_or(p.forward());
            let across = v3(-along.z, 0.0, along.x);
            b.ellipse(v3(c.x, y0 + 0.0005, c.z), along * 0.17, across * 0.08, 12, [0.0, 0.0, 0.0, a], [0.0, 0.0, 0.0, 0.0], &m);
        }
        let pc = s.pelvis.o;
        b.ellipse(
            v3(pc.x, y0, pc.z),
            v3(0.34, 0.0, 0.0),
            v3(0.0, 0.0, 0.34),
            16,
            [0.0, 0.0, 0.0, 0.42],
            [0.0, 0.0, 0.0, 0.0],
            &m,
        );
        for (i, l) in light.lights.iter().enumerate() {
            let flat = v3(-l.dir.x, 0.0, -l.dir.z);
            let fl = flat.length();
            if fl < 1e-3 || l.dir.y < 0.1 {
                continue;
            }
            let along = flat / fl;
            let across = v3(-along.z, 0.0, along.x);
            let len = spec.height * fl / l.dir.y;
            let strength = if i == 0 { 0.34 } else { 0.18 };
            let c = v3(pc.x, 0.0, pc.z) + along * (len * 0.5);
            b.ellipse(
                v3(c.x, y0 + 0.001 * (i + 1) as f32, c.z),
                along * (len * 0.5 + 0.15),
                across * 0.26,
                18,
                [0.0, 0.0, 0.0, strength],
                [0.0, 0.0, 0.0, 0.0],
                &m,
            );
        }
    }

    /// Ring on the ground and a pointer above the head of the human-controlled player.
    pub fn marker(&self, b: &mut MeshBuilder, p: &Player, time: f32) {
        let pulse = 0.85 + 0.15 * (time * 5.0).sin();
        let ring = Mat::unlit(with_alpha(hex(0xffe14a), 0.85 * pulse));
        let c = v3(p.pos.x, super::arena::LINE_Y + 0.012, p.pos.z);
        b.ring_sector(c, 0.42, 0.5, 0.0, std::f32::consts::TAU, 40, &ring);
        // Facing notch.
        let f = p.forward();
        let s = v3(-f.z, 0.0, f.x);
        b.triangle(
            c + f * 0.68,
            c + f * 0.52 + s * 0.09,
            c + f * 0.52 - s * 0.09,
            &ring,
        );
        // Pointer above the head.
        let top = p.pos + v3(0.0, 2.25 + 0.05 * (time * 4.0).sin(), 0.0);
        let arrow = Mat::unlit(hex(0xffe14a));
        let w = 0.16;
        b.triangle(
            top - v3(0.0, 0.2, 0.0),
            top + v3(w, 0.05, 0.0),
            top + v3(-w, 0.05, 0.0),
            &arrow,
        );
        b.triangle(
            top - v3(0.0, 0.2, 0.0),
            top + v3(0.0, 0.05, w),
            top + v3(0.0, 0.05, -w),
            &arrow,
        );
    }
}
