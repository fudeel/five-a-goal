//! Static arena geometry: turf, line markings, rebound boards, cage, goal frames,
//! floodlights, stands. Built once and baked at start-up.

use super::mesh::{hex, mix, scale, with_alpha, Mat, MeshBuilder, Rgba};
use calcio_core::math::{v3, wrap_angle, Rng, Vec3};
use calcio_core::pitch::{FramePart, Pitch, Side};

/// Lines sit slightly above the turf to avoid z-fighting at broadcast distances.
pub const LINE_Y: f32 = 0.022;

fn hash2(ix: i32, iz: i32) -> f32 {
    let mut h = (ix as u32).wrapping_mul(0x8da6_b343) ^ (iz as u32).wrapping_mul(0xd816_3841);
    h ^= h >> 13;
    h = h.wrapping_mul(0x85eb_ca6b);
    h ^= h >> 16;
    (h & 0xffff) as f32 / 65535.0
}

/// Smooth value noise in 0..1.
fn value_noise(x: f32, z: f32) -> f32 {
    let (fx, fz) = (x.floor(), z.floor());
    let (ix, iz) = (fx as i32, fz as i32);
    let (tx, tz) = (x - fx, z - fz);
    let (sx, sz) = (tx * tx * (3.0 - 2.0 * tx), tz * tz * (3.0 - 2.0 * tz));
    let a = hash2(ix, iz);
    let b = hash2(ix + 1, iz);
    let c = hash2(ix, iz + 1);
    let d = hash2(ix + 1, iz + 1);
    let ab = a + (b - a) * sx;
    let cd = c + (d - c) * sx;
    ab + (cd - ab) * sz
}

/// Base turf colour of a cell: mowing stripes inside the lines, darker run-off outside.
fn turf_base(p: &Pitch, x: f32, z: f32, stripe: usize) -> Rgba {
    let inside = x.abs() <= p.half_length() && z.abs() <= p.half_width();
    if inside {
        if stripe % 2 == 0 {
            hex(0x3f8a3c)
        } else {
            hex(0x367c34)
        }
    } else {
        hex(0x2f6b2f)
    }
}

/// Continuous detail on top of the base colour: noise and wear near the goal mouths.
fn turf_detail(p: &Pitch, base: Rgba, x: f32, z: f32) -> Rgba {
    let mut c = base;
    let n = value_noise(x * 0.6, z * 0.6) * 0.6 + value_noise(x * 2.3, z * 2.3) * 0.4;
    c = scale(c, 0.92 + 0.16 * n);
    // Wear: goal mouths and the centre spot are a bit paler/browner.
    let wear_spots = [
        v3(p.half_length() - 1.2, 0.0, 0.0),
        v3(-p.half_length() + 1.2, 0.0, 0.0),
        Vec3::ZERO,
    ];
    for w in wear_spots {
        let d = ((x - w.x).powi(2) + (z - w.z).powi(2)).sqrt();
        let k = (1.0 - d / 2.6).max(0.0) * 0.35 * (0.6 + 0.4 * n);
        c = mix(c, hex(0x7f8a4a), k);
    }
    c
}

fn build_turf(b: &mut MeshBuilder, p: &Pitch) {
    let m = Mat::matte([1.0; 4]);
    let cell = 0.5;
    let bx = p.board_x();
    let bz = p.board_z();
    let nx = (2.0 * bx / cell).round() as i32;
    let nz = (2.0 * bz / cell).round() as i32;
    let stripe_w = 4.0;
    for i in 0..nx {
        let x0 = -bx + i as f32 * cell;
        let x1 = x0 + cell;
        let stripe = ((x0 + 0.5 * cell + p.half_length()) / stripe_w)
            .floor()
            .max(0.0) as usize;
        for j in 0..nz {
            let z0 = -bz + j as f32 * cell;
            let z1 = z0 + cell;
            let (cx, cz) = (0.5 * (x0 + x1), 0.5 * (z0 + z1));
            let base = turf_base(p, cx, cz, stripe);
            let corner = |x: f32, z: f32| turf_detail(p, base, x, z);
            b.quad_ex(
                [
                    v3(x0, 0.0, z0),
                    v3(x1, 0.0, z0),
                    v3(x1, 0.0, z1),
                    v3(x0, 0.0, z1),
                ],
                [Vec3::Y; 4],
                [
                    corner(x0, z0),
                    corner(x1, z0),
                    corner(x1, z1),
                    corner(x0, z1),
                ],
                &m,
            );
        }
    }
    // Surrounding concrete apron.
    let apron = Mat::matte(hex(0x2a2c30));
    let (ax, az) = (70.0, 60.0);
    let y = -0.01;
    b.ground_quad(
        [
            v3(-ax, y, -az),
            v3(ax, y, -az),
            v3(ax, y, -bz),
            v3(-ax, y, -bz),
        ],
        &apron,
    );
    b.ground_quad(
        [v3(-ax, y, bz), v3(ax, y, bz), v3(ax, y, az), v3(-ax, y, az)],
        &apron,
    );
    b.ground_quad(
        [
            v3(-ax, y, -bz),
            v3(-bx, y, -bz),
            v3(-bx, y, bz),
            v3(-ax, y, bz),
        ],
        &apron,
    );
    b.ground_quad(
        [v3(bx, y, -bz), v3(ax, y, -bz), v3(ax, y, bz), v3(bx, y, bz)],
        &apron,
    );
}

fn arc(b: &mut MeshBuilder, c: Vec3, r: f32, w: f32, from: Vec3, to: Vec3, m: &Mat) {
    let a0 = from.heading();
    let d = wrap_angle(to.heading() - a0);
    let segs = ((d.abs() * r) / 0.25).ceil().max(6.0) as usize;
    b.ring_sector(
        v3(c.x, LINE_Y, c.z),
        r - w * 0.5,
        r + w * 0.5,
        a0,
        a0 + d,
        segs,
        m,
    );
}

fn build_lines(b: &mut MeshBuilder, p: &Pitch) {
    let m = Mat::matte(hex(0xf4f4ee));
    let w = p.spec.line_width;
    let (hl, hw) = (p.half_length(), p.half_width());
    let y = LINE_Y;
    // Touchlines and goal lines (extend by half a line width so corners are square).
    for s in [-1.0, 1.0] {
        b.ground_strip(
            v3(-hl - w * 0.5, 0.0, s * hw),
            v3(hl + w * 0.5, 0.0, s * hw),
            w,
            y,
            &m,
        );
        b.ground_strip(v3(s * hl, 0.0, -hw), v3(s * hl, 0.0, hw), w, y, &m);
    }
    // Halfway line, centre circle and spot.
    b.ground_strip(v3(0.0, 0.0, -hw), v3(0.0, 0.0, hw), w, y, &m);
    let r = p.spec.centre_circle_radius;
    b.ring_sector(
        v3(0.0, y, 0.0),
        r - w * 0.5,
        r + w * 0.5,
        0.0,
        std::f32::consts::TAU,
        72,
        &m,
    );
    b.ellipse(
        v3(0.0, y, 0.0),
        v3(0.12, 0.0, 0.0),
        v3(0.0, 0.0, 0.12),
        16,
        m.color,
        m.color,
        &m,
    );
    // Penalty areas, penalty marks.
    let pr = p.spec.penalty_radius;
    let half_line = p.goal.width * 0.5 + 2.0 * p.goal.post_radius;
    for side in Side::BOTH {
        let s = side.sign();
        let gx = s * hl;
        for t in [-1.0, 1.0] {
            let c = v3(gx, 0.0, t * half_line);
            arc(b, c, pr, w, v3(-s, 0.0, 0.0), v3(0.0, 0.0, t), &m);
        }
        let lx = gx - s * pr;
        b.ground_strip(v3(lx, 0.0, -half_line), v3(lx, 0.0, half_line), w, y, &m);
        for d in [p.spec.penalty_spot, p.spec.second_penalty_spot] {
            let c = v3(gx - s * d, y, 0.0);
            b.ellipse(
                c,
                v3(0.09, 0.0, 0.0),
                v3(0.0, 0.0, 0.09),
                14,
                m.color,
                m.color,
                &m,
            );
        }
        // Small marks on the touchline 5 m from the corners (futsal corner distance marks).
        for t in [-1.0, 1.0] {
            let x = gx - s * 5.0;
            b.ground_strip(v3(x, 0.0, t * hw), v3(x, 0.0, t * (hw - 0.4)), w, y, &m);
        }
    }
}

fn build_boards(b: &mut MeshBuilder, p: &Pitch, rng: &mut Rng) {
    let (bx, bz) = (p.board_x(), p.board_z());
    let h = p.arena.board_height;
    let th = 0.08;
    let panel_a = Mat::shiny(hex(0x18335f), 0.25);
    let panel_b = Mat::shiny(hex(0x1f3f74), 0.25);
    let band = Mat::shiny(hex(0xe2a52c), 0.3);
    let band2 = Mat::matte(hex(0xf0f0f0));
    let rail = Mat::shiny(hex(0xdedede), 0.5);
    // (start, end, inward normal)
    let walls = [
        (v3(-bx, 0.0, -bz), v3(bx, 0.0, -bz), Vec3::Z),
        (v3(-bx, 0.0, bz), v3(bx, 0.0, bz), -Vec3::Z),
        (v3(-bx, 0.0, -bz), v3(-bx, 0.0, bz), Vec3::X),
        (v3(bx, 0.0, -bz), v3(bx, 0.0, bz), -Vec3::X),
    ];
    for (a, e, n) in walls {
        let len = a.distance(e);
        let dir = (e - a) / len;
        let panels = (len / 2.4).round().max(1.0) as usize;
        let pw = len / panels as f32;
        for k in 0..panels {
            let c0 = a + dir * (k as f32 * pw);
            let center = c0 + dir * (pw * 0.5) - n * (th * 0.5) + v3(0.0, h * 0.5, 0.0);
            let mat = if k % 2 == 0 { panel_a } else { panel_b };
            let shade = 0.94 + 0.12 * rng.f32();
            b.oriented_box(
                center,
                [dir, Vec3::Y, n],
                v3(pw * 0.5 - 0.01, h * 0.5, th * 0.5),
                &mat.with_color(scale(mat.color, shade)),
            );
            // Decorative bands on the pitch-facing side.
            let face = center + n * (th * 0.5 + 0.004);
            let bc = face + v3(0.0, 0.12, 0.0);
            b.oriented_box(
                bc,
                [dir, Vec3::Y, n],
                v3(pw * 0.5 - 0.06, 0.07, 0.003),
                &band,
            );
            if k % 3 == 1 {
                let bc2 = face + v3(0.0, -0.18, 0.0);
                b.oriented_box(bc2, [dir, Vec3::Y, n], v3(pw * 0.32, 0.05, 0.003), &band2);
            }
        }
        // Rounded top rail.
        let off = -n * (th * 0.5) + v3(0.0, h + 0.03, 0.0);
        b.cylinder(a + off, e + off, 0.05, 0.05, 10, &rail, false);
    }
}

fn build_cage(b: &mut MeshBuilder, p: &Pitch, eye: Vec3) {
    let (bx, bz) = (p.board_x(), p.board_z());
    let h0 = p.arena.board_height + 0.05;
    let h1 = p.arena.cage_height;
    let pole = Mat::shiny(hex(0x3b3f45), 0.3);
    let mesh = Mat::unlit(with_alpha(hex(0x9aa3ad), 0.22));
    // Far side and both ends only: the near side would sit between camera and pitch.
    let walls = [
        (v3(-bx, 0.0, -bz), v3(bx, 0.0, -bz)),
        (v3(-bx, 0.0, -bz), v3(-bx, 0.0, bz)),
        (v3(bx, 0.0, -bz), v3(bx, 0.0, bz)),
    ];
    for (a, e) in walls {
        let len = a.distance(e);
        let dir = (e - a) / len;
        let poles = (len / 4.3).round().max(1.0) as usize;
        for k in 0..=poles {
            let base = a + dir * (len * k as f32 / poles as f32);
            b.cylinder(
                base,
                base + v3(0.0, h1 + 0.1, 0.0),
                0.05,
                0.05,
                8,
                &pole,
                false,
            );
        }
        let top = v3(0.0, h1, 0.0);
        b.cylinder(a + top, e + top, 0.04, 0.04, 8, &pole, false);
        // Netting strands.
        let spacing = 0.3;
        let rows = ((h1 - h0) / spacing) as usize;
        for r in 0..=rows {
            let y = v3(0.0, h0 + r as f32 * spacing, 0.0);
            let segs = (len / 6.0).ceil() as usize;
            for s in 0..segs {
                let p0 = a + dir * (len * s as f32 / segs as f32) + y;
                let p1 = a + dir * (len * (s + 1) as f32 / segs as f32) + y;
                b.ribbon(p0, p1, 0.012, eye, &mesh);
            }
        }
        let cols = (len / spacing) as usize;
        for c in 0..=cols {
            let base = a + dir * (c as f32 * spacing);
            b.ribbon(
                base + v3(0.0, h0, 0.0),
                base + v3(0.0, h1, 0.0),
                0.012,
                eye,
                &mesh,
            );
        }
    }
}

/// Goal frames: white posts and crossbar, grey support bars.
fn build_goals(b: &mut MeshBuilder, p: &Pitch) {
    let frame = Mat::shiny(hex(0xf7f7f7), 0.9);
    let support = Mat::shiny(hex(0x9aa0a6), 0.4);
    for side in Side::BOTH {
        for c in p.goal_colliders(side) {
            match c.part {
                FramePart::Post => {
                    // Posts go slightly above the crossbar centre so the corner is closed.
                    b.cylinder(
                        c.a,
                        c.b + v3(0.0, c.radius, 0.0),
                        c.radius,
                        c.radius,
                        18,
                        &frame,
                        true,
                    )
                }
                FramePart::Crossbar => b.cylinder(c.a, c.b, c.radius, c.radius, 18, &frame, false),
                FramePart::Support => b.cylinder(c.a, c.b, c.radius, c.radius, 8, &support, true),
            }
        }
        // Corner joints.
        let f = p.frame(side);
        for s in [-1.0, 1.0] {
            b.sphere(f.post_top(s), p.goal.post_radius * 1.02, 14, 5, &frame);
        }
    }
}

fn build_floodlights(b: &mut MeshBuilder, p: &Pitch) {
    let mast = Mat::shiny(hex(0x5b6168), 0.3);
    let head = Mat::shiny(hex(0x2a2d31), 0.2);
    let lamp = Mat::unlit(hex(0xfff6dc));
    let glow_in = with_alpha(hex(0xfff2cc), 0.55);
    let glow_out = with_alpha(hex(0xffe6b0), 0.0);
    let gx = p.board_x() + 5.0;
    let gz = p.board_z() + 5.5;
    let h = 19.0;
    for sx in [-1.0, 1.0] {
        for sz in [-1.0, 1.0] {
            let base = v3(sx * gx, 0.0, sz * gz);
            let top = base + v3(0.0, h, 0.0);
            b.cylinder(base, top, 0.32, 0.18, 12, &mast, false);
            // Lamp head facing the centre of the pitch.
            let to_c = (Vec3::ZERO - top).normalize_or(-Vec3::Y);
            let right = Vec3::Y.cross(to_c).normalize_or(Vec3::X);
            let up = to_c.cross(right).normalize_or(Vec3::Y);
            let hc = top + v3(0.0, 0.8, 0.0);
            b.oriented_box(hc, [right, up, to_c], v3(1.7, 0.9, 0.18), &head);
            for i in 0..4 {
                for j in 0..2 {
                    let off = right * (-1.2 + i as f32 * 0.8) + up * (-0.4 + j as f32 * 0.8);
                    let lc = hc + off + to_c * 0.19;
                    b.oriented_box(lc, [right, up, to_c], v3(0.32, 0.3, 0.01), &lamp);
                }
            }
            // Soft glow sprite.
            b.ellipse(
                hc + to_c * 0.3,
                right * 4.2,
                up * 3.0,
                24,
                glow_in,
                glow_out,
                &Mat::unlit([0.0; 4]),
            );
        }
    }
}

/// One seated spectator: torso, head, a hint of arms. `facing` points at the pitch.
fn spectator(b: &mut MeshBuilder, base: Vec3, facing: Vec3, rng: &mut Rng) {
    let shirts = [
        hex(0xa8322d),
        hex(0x1f4f9f),
        hex(0xd8d8d8),
        hex(0x2a2a2a),
        hex(0xc8962c),
        hex(0x2f6f3c),
        hex(0x6a4a8a),
        hex(0xb8662a),
        hex(0x5a6a7a),
    ];
    let skins = [hex(0xe8bf9a), hex(0xcf9a72), hex(0x9c6646), hex(0x5f3c2a)];
    let hairs = [
        hex(0x1a1410),
        hex(0x3a2a1a),
        hex(0x6a4a2a),
        hex(0xb89a6a),
        hex(0x2a2a2a),
    ];
    let dim = 0.62 + 0.2 * rng.f32();
    let shirt = Mat::matte(scale(shirts[rng.index(shirts.len())], dim));
    let skin = Mat::matte(scale(skins[rng.index(skins.len())], dim));
    let hair = Mat::matte(hairs[rng.index(hairs.len())]);
    let side = Vec3::Y.cross(facing).normalize_or(Vec3::X);
    let tall = rng.range(0.42, 0.52);
    let lean = facing * rng.range(-0.04, 0.06);
    let axes = [side, Vec3::Y, facing];
    b.oriented_box(
        base + v3(0.0, tall * 0.5, 0.0) + lean,
        axes,
        v3(0.17, tall * 0.5, 0.11),
        &shirt,
    );
    let hc = base + v3(0.0, tall + 0.12, 0.0) + lean * 1.5;
    b.oriented_box(hc, axes, v3(0.085, 0.1, 0.09), &skin);
    b.oriented_box(
        hc + v3(0.0, 0.08, -0.0) - facing * 0.01,
        axes,
        v3(0.09, 0.035, 0.095),
        &hair,
    );
    if rng.chance(0.12) {
        // Arm raised (celebrating / waving).
        let s = if rng.chance(0.5) { 1.0 } else { -1.0 };
        let sh = base + v3(0.0, tall * 0.9, 0.0) + side * (0.19 * s);
        b.cylinder(
            sh,
            sh + v3(0.0, 0.42, 0.0) + side * (0.06 * s),
            0.045,
            0.04,
            6,
            &shirt,
            true,
        );
    }
}

fn build_stands(b: &mut MeshBuilder, p: &Pitch, rng: &mut Rng) {
    let concrete = Mat::matte(hex(0x34373c));
    let edge = Mat::matte(hex(0x44484e));
    let seat_cols = [hex(0x16294a), hex(0x1b3158)];
    let bz = p.board_z();
    let bx = p.board_x();
    let step_h = 0.42;
    let step_d = 0.85;

    // Far-side main stand.
    let steps = 8;
    let z0 = -(bz + 2.2);
    let xlen = bx + 1.0;
    for k in 0..steps {
        let y0 = k as f32 * step_h;
        let za = z0 - k as f32 * step_d;
        b.cuboid(
            v3(-xlen, 0.0, za - step_d),
            v3(xlen, y0 + step_h, za),
            &concrete,
        );
        b.cuboid(
            v3(-xlen, y0 + step_h - 0.04, za - 0.06),
            v3(xlen, y0 + step_h + 0.01, za),
            &edge,
        );
        let seat = Mat::matte(seat_cols[k % seat_cols.len()]);
        b.cuboid(
            v3(-xlen, y0 + step_h, za - 0.42),
            v3(xlen, y0 + step_h + 0.05, za - 0.12),
            &seat,
        );
        let mut x = -xlen + 0.4;
        while x < xlen - 0.4 {
            if rng.chance(0.62) {
                let base = v3(x + rng.range(-0.06, 0.06), y0 + step_h + 0.05, za - 0.3);
                spectator(b, base, Vec3::Z, rng);
            }
            x += rng.range(0.5, 0.68);
        }
    }
    let zb = z0 - steps as f32 * step_d;
    let top = steps as f32 * step_h;
    b.cuboid(
        v3(-xlen - 0.3, 0.0, zb - 0.3),
        v3(xlen + 0.3, top + 1.2, zb),
        &edge,
    );
    for s in [-1.0, 1.0] {
        // Side walls of the stand.
        b.cuboid(
            v3(s * xlen - 0.15, 0.0, zb),
            v3(s * xlen + 0.15, top + 0.6, z0),
            &edge,
        );
    }

    // Small end stands behind each goal (4 rows, partially filled).
    for s in [-1.0, 1.0] {
        let x0 = s * (bx + 2.4);
        let zl = bz - 3.0;
        for k in 0..4 {
            let y0 = k as f32 * step_h;
            let xa = x0 + s * k as f32 * step_d;
            let (xmin, xmax) = if s > 0.0 {
                (xa, xa + step_d)
            } else {
                (xa - step_d, xa)
            };
            b.cuboid(v3(xmin, 0.0, -zl), v3(xmax, y0 + step_h, zl), &concrete);
            let mut z = -zl + 0.4;
            while z < zl - 0.4 {
                if rng.chance(0.45) {
                    let base = v3(0.5 * (xmin + xmax) + s * 0.1, y0 + step_h, z);
                    spectator(b, base, v3(-s, 0.0, 0.0), rng);
                }
                z += rng.range(0.5, 0.7);
            }
        }
    }
}

/// Builds all static arena geometry. `eye` orients thin camera-facing strands.
pub fn build(p: &Pitch, eye: Vec3) -> MeshBuilder {
    let mut rng = Rng::new(2026);
    let mut b = MeshBuilder::new();
    build_turf(&mut b, p);
    build_lines(&mut b, p);
    build_boards(&mut b, p, &mut rng);
    build_goals(&mut b, p);
    build_stands(&mut b, p, &mut rng);
    build_floodlights(&mut b, p);
    build_cage(&mut b, p, eye);
    b
}
