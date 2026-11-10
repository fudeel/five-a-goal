//! Geometry builder. Primitives are generated in world space with normals and
//! material data, then "baked" (lit on the CPU) into macroquad meshes.
//!
//! macroquad clamps a single draw to its batch capacity (10 000 vertices /
//! 5 000 indices by default), so geometry is split into chunks of at most
//! [`MAX_VERTS`] vertices and [`MAX_INDICES`] indices. Each primitive is added
//! as a unit, so a primitive never straddles two chunks.

use super::light::Lighting;
use calcio_core::math::{v3, Quat, Vec3};
use macroquad::prelude::{draw_mesh, Color, Mesh, Vertex};

/// Linear-ish RGBA in 0..1 (authored in sRGB, converted when lit).
pub type Rgba = [f32; 4];

pub const MAX_VERTS: usize = 4000;
pub const MAX_INDICES: usize = 3996;

/// sRGB hex colour → [`Rgba`].
pub const fn hex(rgb: u32) -> Rgba {
    [
        ((rgb >> 16) & 0xff) as f32 / 255.0,
        ((rgb >> 8) & 0xff) as f32 / 255.0,
        (rgb & 0xff) as f32 / 255.0,
        1.0,
    ]
}

pub fn with_alpha(c: Rgba, a: f32) -> Rgba {
    [c[0], c[1], c[2], a]
}

pub fn mix(a: Rgba, b: Rgba, t: f32) -> Rgba {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
        a[3] + (b[3] - a[3]) * t,
    ]
}

pub fn scale(c: Rgba, s: f32) -> Rgba {
    [c[0] * s, c[1] * s, c[2] * s, c[3]]
}

/// Surface material.
#[derive(Clone, Copy, Debug)]
pub struct Mat {
    pub color: Rgba,
    /// Specular strength (0 = matte).
    pub spec: f32,
    /// 0 = fully lit, 1 = unlit / emissive (colour shown as is).
    pub emit: f32,
    /// Light both faces (thin surfaces: nets, boards seen from behind).
    pub two_sided: bool,
}

impl Mat {
    pub const fn matte(color: Rgba) -> Mat {
        Mat {
            color,
            spec: 0.0,
            emit: 0.0,
            two_sided: false,
        }
    }
    pub const fn shiny(color: Rgba, spec: f32) -> Mat {
        Mat {
            color,
            spec,
            emit: 0.0,
            two_sided: false,
        }
    }
    pub const fn unlit(color: Rgba) -> Mat {
        Mat {
            color,
            spec: 0.0,
            emit: 1.0,
            two_sided: false,
        }
    }
    pub const fn thin(color: Rgba) -> Mat {
        Mat {
            color,
            spec: 0.0,
            emit: 0.0,
            two_sided: true,
        }
    }
    pub fn with_color(self, color: Rgba) -> Mat {
        Mat { color, ..self }
    }
}

/// Elliptical cross-section for lofted surfaces: centre, unit side axis `u`,
/// unit front axis `w`, half widths `a` (along u) and `b` (along w).
#[derive(Clone, Copy, Debug)]
pub struct Ring {
    pub c: Vec3,
    pub u: Vec3,
    pub w: Vec3,
    pub a: f32,
    pub b: f32,
}

/// Vertex before lighting.
#[derive(Clone, Copy, Debug)]
pub struct Vtx {
    pub p: Vec3,
    pub n: Vec3,
    pub c: Rgba,
    pub spec: f32,
    pub emit: f32,
    pub two_sided: bool,
}

#[derive(Default, Clone)]
struct Chunk {
    v: Vec<Vtx>,
    i: Vec<u16>,
}

/// Collects primitives into draw-safe chunks.
#[derive(Clone)]
pub struct MeshBuilder {
    chunks: Vec<Chunk>,
}

impl Default for MeshBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl MeshBuilder {
    pub fn new() -> Self {
        Self {
            chunks: vec![Chunk::default()],
        }
    }

    pub fn vertex_count(&self) -> usize {
        self.chunks.iter().map(|c| c.v.len()).sum()
    }

    /// Makes room for a primitive of `nv` vertices / `ni` indices and returns the
    /// base vertex index inside the current chunk.
    fn start(&mut self, nv: usize, ni: usize) -> u16 {
        assert!(
            nv <= MAX_VERTS && ni <= MAX_INDICES,
            "primitive too large for one chunk"
        );
        let c = self.chunks.last().unwrap();
        if c.v.len() + nv > MAX_VERTS || c.i.len() + ni > MAX_INDICES {
            self.chunks.push(Chunk::default());
        }
        self.chunks.last().unwrap().v.len() as u16
    }

    #[inline]
    fn cur(&mut self) -> &mut Chunk {
        self.chunks.last_mut().unwrap()
    }

    #[inline]
    fn push_v(&mut self, p: Vec3, n: Vec3, c: Rgba, m: &Mat) {
        self.cur().v.push(Vtx {
            p,
            n,
            c,
            spec: m.spec,
            emit: m.emit,
            two_sided: m.two_sided,
        });
    }

    #[inline]
    fn push_tri(&mut self, base: u16, a: u16, b: u16, c: u16) {
        let ch = self.cur();
        ch.i.push(base + a);
        ch.i.push(base + b);
        ch.i.push(base + c);
    }

    /// Appends raw geometry (indices local to `verts`).
    pub fn raw(&mut self, verts: &[Vtx], indices: &[u16]) {
        let base = self.start(verts.len(), indices.len());
        let ch = self.cur();
        ch.v.extend_from_slice(verts);
        ch.i.extend(indices.iter().map(|i| base + *i));
    }

    // ------------------------------------------------------------------
    // Flat primitives
    // ------------------------------------------------------------------

    pub fn triangle(&mut self, p0: Vec3, p1: Vec3, p2: Vec3, m: &Mat) {
        let n = (p1 - p0).cross(p2 - p0).normalize_or(Vec3::Y);
        let base = self.start(3, 3);
        for p in [p0, p1, p2] {
            self.push_v(p, n, m.color, m);
        }
        self.push_tri(base, 0, 1, 2);
    }

    /// Quad p0→p1→p2→p3 (any winding; no culling is used).
    pub fn quad(&mut self, p: [Vec3; 4], m: &Mat) {
        let n = (p[1] - p[0]).cross(p[3] - p[0]).normalize_or(Vec3::Y);
        self.quad_ex(p, [n; 4], [m.color; 4], m);
    }

    /// Horizontal quad with an upward normal.
    pub fn ground_quad(&mut self, p: [Vec3; 4], m: &Mat) {
        self.quad_ex(p, [Vec3::Y; 4], [m.color; 4], m);
    }

    /// Quad with per-corner normals and colours.
    pub fn quad_ex(&mut self, p: [Vec3; 4], n: [Vec3; 4], c: [Rgba; 4], m: &Mat) {
        let base = self.start(4, 6);
        for k in 0..4 {
            self.push_v(p[k], n[k], c[k], m);
        }
        self.push_tri(base, 0, 1, 2);
        self.push_tri(base, 0, 2, 3);
    }

    /// Oriented box: `axes` are unit vectors, `half` the half extents along them.
    pub fn oriented_box(&mut self, center: Vec3, axes: [Vec3; 3], half: Vec3, m: &Mat) {
        let h = [half.x, half.y, half.z];
        for f in 0..3 {
            for s in [-1.0f32, 1.0] {
                let n = axes[f] * s;
                let u = axes[(f + 1) % 3] * h[(f + 1) % 3];
                let w = axes[(f + 2) % 3] * h[(f + 2) % 3];
                let c = center + n * h[f];
                self.quad_ex(
                    [c - u - w, c + u - w, c + u + w, c - u + w],
                    [n; 4],
                    [m.color; 4],
                    m,
                );
            }
        }
    }

    /// Axis-aligned box from min/max corners.
    pub fn cuboid(&mut self, min: Vec3, max: Vec3, m: &Mat) {
        self.oriented_box(
            (min + max) * 0.5,
            [Vec3::X, Vec3::Y, Vec3::Z],
            (max - min) * 0.5,
            m,
        );
    }

    // ------------------------------------------------------------------
    // Round primitives
    // ------------------------------------------------------------------

    /// Cylinder (optionally tapered) from `a` to `b`.
    pub fn cylinder(
        &mut self,
        a: Vec3,
        b: Vec3,
        ra: f32,
        rb: f32,
        segs: usize,
        m: &Mat,
        caps: bool,
    ) {
        let axis_v = b - a;
        let len = axis_v.length();
        if len < 1e-6 {
            return;
        }
        let axis = axis_v / len;
        let u = axis.any_perpendicular();
        let w = axis.cross(u);
        let slope = (ra - rb) / len;
        let nv = (segs + 1) * 2 + if caps { 2 * (segs + 2) } else { 0 };
        let ni = segs * 6 + if caps { 2 * segs * 3 } else { 0 };
        let base = self.start(nv, ni);
        for i in 0..=segs {
            let t = i as f32 / segs as f32 * std::f32::consts::TAU;
            let radial = u * t.cos() + w * t.sin();
            let n = (radial + axis * slope).normalize_or(radial);
            self.push_v(a + radial * ra, n, m.color, m);
            self.push_v(b + radial * rb, n, m.color, m);
        }
        for i in 0..segs as u16 {
            let k = i * 2;
            self.push_tri(base, k, k + 1, k + 3);
            self.push_tri(base, k, k + 3, k + 2);
        }
        if caps {
            for (center, r, n) in [(a, ra, -axis), (b, rb, axis)] {
                let cb = self.cur().v.len() as u16 - base;
                self.push_v(center, n, m.color, m);
                for i in 0..=segs {
                    let t = i as f32 / segs as f32 * std::f32::consts::TAU;
                    self.push_v(center + (u * t.cos() + w * t.sin()) * r, n, m.color, m);
                }
                for i in 0..segs as u16 {
                    self.push_tri(base, cb, cb + 1 + i, cb + 2 + i);
                }
            }
        }
    }

    /// Capsule from `a` to `b` (a sphere when `a == b`).
    pub fn capsule(&mut self, a: Vec3, b: Vec3, r: f32, segs: usize, rings: usize, m: &Mat) {
        let d = b - a;
        let axis = d.normalize_or(Vec3::Y);
        let u = axis.any_perpendicular();
        let w = axis.cross(u);
        let rows = 2 * (rings + 1);
        let cols = segs + 1;
        let base = self.start(rows * cols, (rows - 1) * segs * 6);
        let half_pi = std::f32::consts::FRAC_PI_2;
        for row in 0..rows {
            let (center, phi) = if row <= rings {
                (a, -half_pi + row as f32 / rings as f32 * half_pi)
            } else {
                (b, (row - rings - 1) as f32 / rings as f32 * half_pi)
            };
            let (sp, cp) = phi.sin_cos();
            for i in 0..cols {
                let t = i as f32 / segs as f32 * std::f32::consts::TAU;
                let n = (u * t.cos() + w * t.sin()) * cp + axis * sp;
                self.push_v(center + n * r, n, m.color, m);
            }
        }
        for row in 0..(rows - 1) as u16 {
            for i in 0..segs as u16 {
                let k0 = row * cols as u16 + i;
                let k1 = k0 + cols as u16;
                self.push_tri(base, k0, k1, k1 + 1);
                self.push_tri(base, k0, k1 + 1, k0 + 1);
            }
        }
    }

    pub fn sphere(&mut self, c: Vec3, r: f32, segs: usize, rings: usize, m: &Mat) {
        self.capsule(c, c, r, segs, rings, m);
    }

    /// Sphere from a unit-sphere template, rotated by `rot`, with per-vertex colour.
    pub fn sphere_template(
        &mut self,
        t: &SphereTemplate,
        center: Vec3,
        radius: f32,
        rot: Quat,
        m: &Mat,
        mut color_of: impl FnMut(usize) -> Rgba,
    ) {
        let base = self.start(t.verts.len(), t.indices.len());
        for (k, p) in t.verts.iter().enumerate() {
            let n = rot.rotate(*p);
            self.push_v(center + n * radius, n, color_of(k), m);
        }
        let ch = self.cur();
        ch.i.extend(t.indices.iter().map(|i| base + *i));
    }

    // ------------------------------------------------------------------
    // Lofted surfaces (bodies)
    // ------------------------------------------------------------------

    /// Surface through a sequence of elliptical rings, with smooth normals.
    /// `colors` (one per ring) override the material colour when given.
    pub fn loft(&mut self, rings: &[Ring], segs: usize, m: &Mat, colors: Option<&[Rgba]>, caps: (bool, bool)) {
        if rings.len() < 2 {
            return;
        }
        let cols = segs + 1;
        let nv = rings.len() * cols + if caps.0 { cols + 1 } else { 0 } + if caps.1 { cols + 1 } else { 0 };
        let ni = (rings.len() - 1) * segs * 6 + (caps.0 as usize + caps.1 as usize) * segs * 3;
        let base = self.start(nv, ni);
        let point = |r: &Ring, cs: f32, sn: f32| r.c + r.u * (r.a * cs) + r.w * (r.b * sn);
        let last = rings.len() - 1;
        for (k, r) in rings.iter().enumerate() {
            let c = colors.map(|c| c[k]).unwrap_or(m.color);
            let (ra, rb) = (&rings[k.saturating_sub(1)], &rings[(k + 1).min(last)]);
            for i in 0..cols {
                let t = i as f32 / segs as f32 * std::f32::consts::TAU;
                let (sn, cs) = t.sin_cos();
                let p = point(r, cs, sn);
                // In-plane (ellipse gradient) normal, then tilt it by the surface slope.
                let n_r = (r.u * (cs / r.a.max(1e-4)) + r.w * (sn / r.b.max(1e-4))).normalize_or(r.u);
                let along = point(rb, cs, sn) - point(ra, cs, sn);
                let around = r.w * (r.b * cs) - r.u * (r.a * sn);
                let mut n = if around.length_sq() > 1e-10 && along.length_sq() > 1e-10 {
                    around.cross(along).normalize_or(n_r)
                } else {
                    // Pole of a closed shape: point along the axis.
                    let axis = (rb.c - ra.c).normalize_or(n_r);
                    if k == 0 { -axis } else { axis }
                };
                if n.dot(n_r) < 0.0 && around.length_sq() > 1e-10 {
                    n = -n;
                }
                self.push_v(p, n, c, m);
            }
        }
        for k in 0..(rings.len() - 1) as u16 {
            for i in 0..segs as u16 {
                let a = k * cols as u16 + i;
                let b2 = a + cols as u16;
                self.push_tri(base, a, b2, b2 + 1);
                self.push_tri(base, a, b2 + 1, a + 1);
            }
        }
        for (end, on) in [(0usize, caps.0), (rings.len() - 1, caps.1)] {
            if !on {
                continue;
            }
            let r = &rings[end];
            let other = if end == 0 { &rings[1] } else { &rings[rings.len() - 2] };
            let n = (r.c - other.c).normalize_or(r.u.cross(r.w));
            let c = colors.map(|c| c[end]).unwrap_or(m.color);
            let cb = self.cur().v.len() as u16 - base;
            self.push_v(r.c + n * (0.35 * r.a.min(r.b)), n, c, m);
            for i in 0..cols {
                let t = i as f32 / segs as f32 * std::f32::consts::TAU;
                let p = r.c + r.u * (r.a * t.cos()) + r.w * (r.b * t.sin());
                let nn = (n * 0.7 + (p - r.c).normalize_or(n) * 0.3).normalize_or(n);
                self.push_v(p, nn, c, m);
            }
            for i in 0..segs as u16 {
                self.push_tri(base, cb, cb + 1 + i, cb + 2 + i);
            }
        }
    }

    /// Ellipsoid with semi-axes along the given (unit) axes.
    pub fn ellipsoid(&mut self, c: Vec3, axes: [Vec3; 3], r: Vec3, segs: usize, rings: usize, m: &Mat) {
        let mut rs = Vec::with_capacity(rings + 1);
        for k in 0..=rings {
            let phi = -std::f32::consts::FRAC_PI_2 + std::f32::consts::PI * k as f32 / rings as f32;
            let (sp, cp) = phi.sin_cos();
            rs.push(Ring { c: c + axes[1] * (r.y * sp), u: axes[0], w: axes[2], a: (r.x * cp).max(1e-4), b: (r.z * cp).max(1e-4) });
        }
        self.loft(&rs, segs, m, None, (false, false));
    }

    // ------------------------------------------------------------------
    // Decals and thin things
    // ------------------------------------------------------------------

    /// Elliptical disc with a radial colour gradient (shadows, glows).
    /// `ax`/`az` are the (scaled) semi-axes.
    pub fn ellipse(
        &mut self,
        c: Vec3,
        ax: Vec3,
        az: Vec3,
        segs: usize,
        inner: Rgba,
        outer: Rgba,
        m: &Mat,
    ) {
        let mut n = ax.cross(az).normalize_or(Vec3::Y);
        if n.y < 0.0 {
            n = -n;
        }
        let base = self.start(segs + 2, segs * 3);
        self.push_v(c, n, inner, m);
        for i in 0..=segs {
            let t = i as f32 / segs as f32 * std::f32::consts::TAU;
            self.push_v(c + ax * t.cos() + az * t.sin(), n, outer, m);
        }
        for i in 0..segs as u16 {
            self.push_tri(base, 0, 1 + i, 2 + i);
        }
    }

    /// Flat ring sector on a horizontal plane at height `y`, from heading `a0` to `a1`.
    #[allow(clippy::too_many_arguments)]
    pub fn ring_sector(
        &mut self,
        c: Vec3,
        r_in: f32,
        r_out: f32,
        a0: f32,
        a1: f32,
        segs: usize,
        m: &Mat,
    ) {
        let base = self.start((segs + 1) * 2, segs * 6);
        for i in 0..=segs {
            let a = a0 + (a1 - a0) * i as f32 / segs as f32;
            let d = Vec3::from_heading(a);
            self.push_v(c + d * r_in, Vec3::Y, m.color, m);
            self.push_v(c + d * r_out, Vec3::Y, m.color, m);
        }
        for i in 0..segs as u16 {
            let k = i * 2;
            self.push_tri(base, k, k + 1, k + 3);
            self.push_tri(base, k, k + 3, k + 2);
        }
    }

    /// Camera-facing ribbon from `a` to `b` (net strands, thin lines).
    pub fn ribbon(&mut self, a: Vec3, b: Vec3, width: f32, eye: Vec3, m: &Mat) {
        let mid = (a + b) * 0.5;
        let to_eye = (eye - mid).normalize_or(Vec3::Y);
        let side = (b - a).cross(to_eye).normalize_or(Vec3::X) * (width * 0.5);
        let n = (to_eye + Vec3::Y * 0.6).normalize_or(Vec3::Y);
        self.quad_ex(
            [a - side, a + side, b + side, b - side],
            [n; 4],
            [m.color; 4],
            m,
        );
    }

    /// Ribbon with explicit per-end colours.
    pub fn ribbon_ex(
        &mut self,
        a: Vec3,
        b: Vec3,
        width: f32,
        eye: Vec3,
        ca: Rgba,
        cb: Rgba,
        m: &Mat,
    ) {
        let mid = (a + b) * 0.5;
        let to_eye = (eye - mid).normalize_or(Vec3::Y);
        let side = (b - a).cross(to_eye).normalize_or(Vec3::X) * (width * 0.5);
        let n = (to_eye + Vec3::Y * 0.6).normalize_or(Vec3::Y);
        self.quad_ex(
            [a - side, a + side, b + side, b - side],
            [n; 4],
            [ca, ca, cb, cb],
            m,
        );
    }

    /// Flat strip on the ground (line markings) from `a` to `b`, width `w`, at height `y`.
    pub fn ground_strip(&mut self, a: Vec3, b: Vec3, w: f32, y: f32, m: &Mat) {
        let dir = (b - a).flat().normalize_or(Vec3::X);
        let side = v3(-dir.z, 0.0, dir.x) * (w * 0.5);
        let a = v3(a.x, y, a.z);
        let b = v3(b.x, y, b.z);
        self.quad_ex(
            [a - side, a + side, b + side, b - side],
            [Vec3::Y; 4],
            [m.color; 4],
            m,
        );
    }

    // ------------------------------------------------------------------
    // Baking
    // ------------------------------------------------------------------

    /// Lights every vertex and converts to macroquad meshes.
    pub fn bake(self, light: &Lighting) -> Vec<Mesh> {
        let mut out = Vec::with_capacity(self.chunks.len());
        for ch in self.chunks {
            if ch.i.is_empty() {
                continue;
            }
            let vertices =
                ch.v.iter()
                    .map(|v| {
                        let c = light.shade(v);
                        Vertex::new(
                            v.p.x,
                            v.p.y,
                            v.p.z,
                            0.0,
                            0.0,
                            Color::new(c[0], c[1], c[2], c[3]),
                        )
                    })
                    .collect();
            out.push(Mesh {
                vertices,
                indices: ch.i,
                texture: None,
            });
        }
        out
    }

    /// Bakes and draws immediately (dynamic geometry).
    pub fn draw(self, light: &Lighting) {
        for m in self.bake(light) {
            draw_mesh(&m);
        }
    }
}

/// Draws pre-baked meshes.
pub fn draw_meshes(meshes: &[Mesh]) {
    for m in meshes {
        draw_mesh(m);
    }
}

/// A unit icosphere, generated once and reused (e.g. for the ball).
#[derive(Clone, Debug)]
pub struct SphereTemplate {
    pub verts: Vec<Vec3>,
    pub indices: Vec<u16>,
}

impl SphereTemplate {
    /// Subdivided icosahedron; level 3 → 642 vertices, 1280 triangles.
    pub fn icosphere(level: u32) -> Self {
        let t = (1.0 + 5.0f32.sqrt()) * 0.5;
        let mut verts: Vec<Vec3> = [
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
        .map(|v| v.normalize_or_zero())
        .collect();
        let mut faces: Vec<[u16; 3]> = vec![
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
        for _ in 0..level {
            let mut cache = std::collections::HashMap::new();
            let mut mid = |a: u16, b: u16, verts: &mut Vec<Vec3>| -> u16 {
                let key = if a < b { (a, b) } else { (b, a) };
                *cache.entry(key).or_insert_with(|| {
                    let p = ((verts[a as usize] + verts[b as usize]) * 0.5).normalize_or_zero();
                    verts.push(p);
                    (verts.len() - 1) as u16
                })
            };
            let mut nf = Vec::with_capacity(faces.len() * 4);
            for f in &faces {
                let a = mid(f[0], f[1], &mut verts);
                let b = mid(f[1], f[2], &mut verts);
                let c = mid(f[2], f[0], &mut verts);
                nf.push([f[0], a, c]);
                nf.push([f[1], b, a]);
                nf.push([f[2], c, b]);
                nf.push([a, b, c]);
            }
            faces = nf;
        }
        let indices = faces.iter().flat_map(|f| f.iter().copied()).collect();
        Self { verts, indices }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn icosphere_sizes() {
        let t = SphereTemplate::icosphere(3);
        assert_eq!(t.verts.len(), 642);
        assert_eq!(t.indices.len(), 1280 * 3);
        assert!(t.verts.iter().all(|v| (v.length() - 1.0).abs() < 1e-5));
    }

    #[test]
    fn chunks_respect_limits() {
        let mut b = MeshBuilder::new();
        for i in 0..400 {
            b.sphere(
                v3(i as f32, 0.0, 0.0),
                0.5,
                16,
                6,
                &Mat::matte(hex(0xffffff)),
            );
        }
        let light = Lighting::floodlit(v3(0.0, 10.0, 20.0));
        let meshes = b.bake(&light);
        assert!(meshes.len() > 1);
        for m in &meshes {
            assert!(m.vertices.len() <= MAX_VERTS);
            assert!(m.indices.len() <= MAX_INDICES);
            assert!(m.indices.iter().all(|&i| (i as usize) < m.vertices.len()));
        }
    }

    #[test]
    fn cylinder_with_caps_indices_valid() {
        let mut b = MeshBuilder::new();
        b.cylinder(
            Vec3::ZERO,
            v3(0.0, 2.0, 0.0),
            0.1,
            0.05,
            12,
            &Mat::matte(hex(0xffffff)),
            true,
        );
        let meshes = b.bake(&Lighting::floodlit(v3(0.0, 10.0, 20.0)));
        for m in &meshes {
            assert!(m.indices.iter().all(|&i| (i as usize) < m.vertices.len()));
        }
    }
}
