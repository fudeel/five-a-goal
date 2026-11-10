//! Ball rendering: classic 32-panel ball (12 pentagons, 20 hexagons) built from a
//! truncated icosahedron projected on the sphere, rotating with the simulated
//! orientation, plus soft floodlight shadows.

use super::light::Lighting;
use super::mesh::{hex, Mat, MeshBuilder, Rgba, SphereTemplate, Vtx};
use calcio_core::math::{v3, Quat, Vec3};

/// One panel in ball-local unit coordinates.
struct Panel {
    verts: Vec<Vec3>,
    indices: Vec<u16>,
    color: Rgba,
}

pub struct BallRenderer {
    panels: Vec<Panel>,
    core: SphereTemplate,
}

fn ico() -> (Vec<Vec3>, Vec<[usize; 3]>) {
    let t = (1.0 + 5.0f32.sqrt()) * 0.5;
    let v: Vec<Vec3> = [
        v3(-1.0, t, 0.0),
        v3(1.0, t, 0.0),
        v3(-1.0, -t, 0.0),
        v3(1.0, -t, 0.0),
        v3(0.0, -1.0, t),
        v3(0.0, 1.0, t),
        v3(0.0, -1.0, -t),
        v3(0.0, 1.0, -t),
        v3(t, 0.0, -1.0),
        v3(t, 0.0, 1.0),
        v3(-t, 0.0, -1.0),
        v3(-t, 0.0, 1.0),
    ]
    .iter()
    .map(|p| p.normalize_or_zero())
    .collect();
    let f = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    (v, f)
}

/// Builds a curved panel from boundary points (unit vectors, ordered).
fn panel(boundary: &[Vec3], color: Rgba, inset: f32, k: usize) -> Panel {
    let center = boundary
        .iter()
        .fold(Vec3::ZERO, |a, b| a + *b)
        .normalize_or(Vec3::Y);
    let ring: Vec<Vec3> = boundary
        .iter()
        .map(|p| center.lerp(*p, inset).normalize_or(*p))
        .collect();
    let mut verts = Vec::new();
    let mut indices = Vec::new();
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        // Subdivide triangle (center, a, b) into k² triangles.
        let base = verts.len() as u16;
        let mut row_start = Vec::new();
        for r in 0..=k {
            row_start.push(verts.len() as u16 - base);
            let t = r as f32 / k as f32;
            let pa = center.lerp(a, t);
            let pb = center.lerp(b, t);
            for c in 0..=r {
                let u = if r == 0 { 0.0 } else { c as f32 / r as f32 };
                verts.push(pa.lerp(pb, u).normalize_or(center));
            }
        }
        for r in 0..k {
            let s0 = row_start[r];
            let s1 = row_start[r + 1];
            for c in 0..=r as u16 {
                indices.extend_from_slice(&[base + s0 + c, base + s1 + c, base + s1 + c + 1]);
                if c < r as u16 {
                    indices.extend_from_slice(&[
                        base + s0 + c,
                        base + s1 + c + 1,
                        base + s0 + c + 1,
                    ]);
                }
            }
        }
    }
    Panel {
        verts,
        indices,
        color,
    }
}

impl BallRenderer {
    pub fn new() -> Self {
        let (v, faces) = ico();
        let white = hex(0xf4f4f0);
        let black = hex(0x1b1b1f);
        let inset = 0.955;
        let k = 3;
        let mut panels = Vec::new();
        // Pentagons: around each icosahedron vertex, points 1/3 along its edges.
        for (i, vi) in v.iter().enumerate() {
            let mut nbrs: Vec<Vec3> = faces
                .iter()
                .filter(|f| f.contains(&i))
                .flat_map(|f| f.iter().copied().filter(|&j| j != i).collect::<Vec<_>>())
                .map(|j| v[j])
                .collect();
            nbrs.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap());
            nbrs.dedup_by(|a, b| a.distance(*b) < 1e-5);
            // Order neighbours around the vertex.
            let u = vi.any_perpendicular();
            let w = vi.cross(u);
            nbrs.sort_by(|a, b| {
                let aa = (*a - *vi).dot(w).atan2((*a - *vi).dot(u));
                let bb = (*b - *vi).dot(w).atan2((*b - *vi).dot(u));
                aa.partial_cmp(&bb).unwrap()
            });
            let boundary: Vec<Vec3> = nbrs
                .iter()
                .map(|n| (*vi + (*n - *vi) * (1.0 / 3.0)).normalize_or_zero())
                .collect();
            panels.push(panel(&boundary, black, inset, k));
        }
        // Hexagons: one per icosahedron face.
        for f in &faces {
            let (a, b, c) = (v[f[0]], v[f[1]], v[f[2]]);
            let third = |p: Vec3, q: Vec3, t: f32| (p + (q - p) * t).normalize_or_zero();
            let boundary = [
                third(a, b, 1.0 / 3.0),
                third(a, b, 2.0 / 3.0),
                third(b, c, 1.0 / 3.0),
                third(b, c, 2.0 / 3.0),
                third(c, a, 1.0 / 3.0),
                third(c, a, 2.0 / 3.0),
            ];
            panels.push(panel(&boundary, white, inset, k));
        }
        Self {
            panels,
            core: SphereTemplate::icosphere(2),
        }
    }

    /// Adds the ball at `pos` with orientation `rot`.
    pub fn build(&self, b: &mut MeshBuilder, pos: Vec3, rot: Quat, radius: f32) {
        let shiny = Mat::shiny([1.0; 4], 0.55);
        // Dark core shows through the seams between panels.
        let seam = hex(0x3a3a3e);
        b.sphere_template(
            &self.core,
            pos,
            radius * 0.985,
            rot,
            &Mat::matte(seam),
            |_| seam,
        );
        for p in &self.panels {
            let verts: Vec<Vtx> = p
                .verts
                .iter()
                .map(|v| {
                    let n = rot.rotate(*v);
                    Vtx {
                        p: pos + n * radius,
                        n,
                        c: p.color,
                        spec: shiny.spec,
                        emit: 0.0,
                        two_sided: false,
                    }
                })
                .collect();
            b.raw(&verts, &p.indices);
        }
    }

    /// Soft floodlight shadows on the turf, one per light, fading with height.
    pub fn shadows(&self, b: &mut MeshBuilder, pos: Vec3, radius: f32, light: &Lighting) {
        let h = (pos.y - radius).max(0.0);
        let m = Mat::unlit([0.0; 4]);
        for (i, l) in light.lights.iter().enumerate() {
            let strength = if i == 0 { 0.5 } else { 0.25 };
            let d = l.dir;
            let off = if d.y > 0.05 {
                v3(-d.x, 0.0, -d.z) * (pos.y / d.y)
            } else {
                Vec3::ZERO
            };
            let c = v3(
                pos.x + off.x,
                super::arena::LINE_Y + 0.004 + i as f32 * 0.001,
                pos.z + off.z,
            );
            let stretch = 1.0 + 0.5 * v3(d.x, 0.0, d.z).length() / d.y.max(0.2);
            let r = radius * (1.05 + 0.25 * h);
            let alpha = strength / (1.0 + 0.7 * h);
            let along = v3(-d.x, 0.0, -d.z).normalize_or(Vec3::X);
            let across = v3(-along.z, 0.0, along.x);
            b.ellipse(
                c,
                along * (r * stretch),
                across * r,
                18,
                [0.0, 0.0, 0.0, alpha],
                [0.0, 0.0, 0.0, 0.0],
                &m,
            );
        }
    }
}
