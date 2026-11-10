//! Realistic footballer model, built every frame on the animated skeleton:
//! lofted torso and limbs with anatomical profiles, a sculpted head with hair
//! styles, kit details, boots and shirt numbers.

use super::anim::rig::Frame;
use super::anim::Skeleton;
use super::mesh::{hex, scale, Mat, MeshBuilder, Rgba, Ring};
use calcio_core::math::{v3, Vec3};
use calcio_core::player::{Player, Role};

/// Kit colours for one team.
#[derive(Clone, Copy, Debug)]
pub struct Kit {
    pub shirt: Rgba,
    pub trim: Rgba,
    pub shorts: Rgba,
    pub socks: Rgba,
    pub number: Rgba,
    pub gk_shirt: Rgba,
    pub gk_shorts: Rgba,
    pub gk_number: Rgba,
}

pub const KITS: [Kit; 2] = [
    Kit {
        shirt: hex(0xc62f2a),
        trim: hex(0xf2f2f2),
        shorts: hex(0xf2f2f2),
        socks: hex(0xc62f2a),
        number: hex(0xf7f7f7),
        gk_shirt: hex(0xb8d936),
        gk_shorts: hex(0x1d1d1f),
        gk_number: hex(0x1d1d1f),
    },
    Kit {
        shirt: hex(0x1f5bbf),
        trim: hex(0xf2c230),
        shorts: hex(0x14233f),
        socks: hex(0xf2f2f2),
        number: hex(0xf2c230),
        gk_shirt: hex(0xf0822a),
        gk_shorts: hex(0x1d1d1f),
        gk_number: hex(0x1d1d1f),
    },
];

const SKINS: [u32; 7] = [0xf3d3b6, 0xe9bf98, 0xd7a37a, 0xbb8257, 0x9a6440, 0x76492d, 0x553420];
const HAIRS: [u32; 7] = [0x15110d, 0x2b1f16, 0x4a3221, 0x6d4c2e, 0xa8824f, 0xd2b27a, 0x0d0d0d];
const BOOTS: [(u32, u32); 6] =
    [(0x141414, 0xf2f2f2), (0xf2f2f2, 0x141414), (0xff6a1a, 0x141414), (0x2a7fff, 0xf2f2f2), (0xc8ff2e, 0x141414), (0xe0233a, 0xf2f2f2)];

fn pick(id: usize, salt: usize, n: usize) -> usize {
    let mut h = (id as u32).wrapping_mul(0x9E37_79B1) ^ (salt as u32).wrapping_mul(0x85EB_CA6B);
    h ^= h >> 15;
    h = h.wrapping_mul(0x2c1b_3c6d);
    h ^= h >> 13;
    (h as usize) % n
}

/// Everything that makes one player look like himself.
#[derive(Clone, Copy, Debug)]
pub struct Look {
    pub shirt: Rgba,
    pub trim: Rgba,
    pub shorts: Rgba,
    pub socks: Rgba,
    pub number_color: Rgba,
    pub skin: Rgba,
    pub hair: Rgba,
    pub hair_style: usize,
    pub boots: Rgba,
    pub boots_accent: Rgba,
    pub gloves: Option<Rgba>,
    pub number: u8,
}

impl Look {
    pub fn of(p: &Player) -> Look {
        let kit = KITS[p.team.min(1)];
        let gk = p.role == Role::Goalkeeper;
        let (boots, accent) = BOOTS[pick(p.id, 3, BOOTS.len())];
        Look {
            shirt: if gk { kit.gk_shirt } else { kit.shirt },
            trim: kit.trim,
            shorts: if gk { kit.gk_shorts } else { kit.shorts },
            socks: if gk { kit.gk_shorts } else { kit.socks },
            number_color: if gk { kit.gk_number } else { kit.number },
            skin: hex(SKINS[pick(p.id, 1, SKINS.len())]),
            hair: hex(HAIRS[pick(p.id, 2, HAIRS.len())]),
            hair_style: pick(p.id, 4, 5),
            boots: hex(boots),
            boots_accent: hex(accent),
            gloves: if gk { Some(hex(0xf4f4f4)) } else { None },
            number: p.number,
        }
    }
}

/// Rings along a bone from `a` to `b`. Profile entries: (t, half side, half front, front offset).
fn bone_rings(a: Vec3, b: Vec3, front: Vec3, profile: &[(f32, f32, f32, f32)], k: f32, bw: f32) -> Vec<Ring> {
    let axis = (b - a).normalize_or(Vec3::Y);
    let w = (front - axis * front.dot(axis)).normalize_or(axis.any_perpendicular());
    let u = axis.cross(w).normalize_or(Vec3::X);
    profile
        .iter()
        .map(|&(t, side, fr, off)| Ring { c: a.lerp(b, t) + w * (off * k), u, w, a: side * k * bw, b: fr * k * bw })
        .collect()
}

/// Rings in a frame: entries (y, half width, half depth, z offset).
fn frame_rings(f: &Frame, profile: &[(f32, f32, f32, f32)], k: f32, bw: f32) -> Vec<Ring> {
    profile
        .iter()
        .map(|&(y, a, b, z)| Ring { c: f.point(v3(0.0, y * k, z * k)), u: f.x, w: f.z, a: a * k * bw, b: b * k * bw })
        .collect()
}

const TORSO_LOW: [(f32, f32, f32, f32); 5] = [
    (-0.045, 0.188, 0.134, -0.002),
    (-0.01, 0.182, 0.128, 0.0),
    (0.07, 0.164, 0.116, 0.0),
    (0.17, 0.152, 0.108, 0.004),
    (0.26, 0.156, 0.111, 0.008),
];
const TORSO_HIGH: [(f32, f32, f32, f32); 6] = [
    (0.05, 0.166, 0.119, 0.012),
    (0.12, 0.176, 0.126, 0.014),
    (0.185, 0.174, 0.117, 0.006),
    (0.228, 0.152, 0.092, -0.012),
    (0.252, 0.1, 0.07, -0.02),
    (0.264, 0.064, 0.058, -0.022),
];

const HEAD: [(f32, f32, f32, f32); 11] = [
    (-0.122, 0.012, 0.012, 0.055),
    (-0.112, 0.03, 0.026, 0.052),
    (-0.095, 0.05, 0.05, 0.035),
    (-0.07, 0.062, 0.075, 0.018),
    (-0.035, 0.069, 0.09, 0.008),
    (0.0, 0.074, 0.097, 0.002),
    (0.04, 0.077, 0.099, -0.004),
    (0.075, 0.073, 0.093, -0.01),
    (0.1, 0.06, 0.077, -0.014),
    (0.117, 0.036, 0.048, -0.018),
    (0.125, 0.004, 0.004, -0.02),
];

/// Stroke font for shirt numbers, unit box (x right, y up).
fn digit_strokes(d: u8) -> &'static [&'static [(f32, f32)]] {
    match d {
        0 => &[&[(0.18, 0.0), (0.82, 0.0), (1.0, 0.16), (1.0, 0.84), (0.82, 1.0), (0.18, 1.0), (0.0, 0.84), (0.0, 0.16), (0.18, 0.0)]],
        1 => &[&[(0.25, 0.78), (0.62, 1.0), (0.62, 0.0)]],
        2 => &[&[(0.0, 0.82), (0.16, 1.0), (0.84, 1.0), (1.0, 0.82), (1.0, 0.62), (0.0, 0.0), (1.0, 0.0)]],
        3 => &[
            &[(0.0, 0.84), (0.16, 1.0), (0.84, 1.0), (1.0, 0.84), (1.0, 0.64), (0.82, 0.52), (0.35, 0.52)],
            &[(0.82, 0.52), (1.0, 0.4), (1.0, 0.16), (0.84, 0.0), (0.16, 0.0), (0.0, 0.16)],
        ],
        4 => &[&[(0.75, 0.0), (0.75, 1.0), (0.0, 0.32), (1.0, 0.32)]],
        5 => &[&[(1.0, 1.0), (0.06, 1.0), (0.0, 0.56), (0.75, 0.6), (1.0, 0.42), (1.0, 0.16), (0.84, 0.0), (0.16, 0.0), (0.0, 0.16)]],
        6 => &[&[(0.9, 1.0), (0.3, 1.0), (0.0, 0.62), (0.0, 0.16), (0.16, 0.0), (0.84, 0.0), (1.0, 0.16), (1.0, 0.45), (0.84, 0.6), (0.16, 0.6), (0.0, 0.45)]],
        7 => &[&[(0.0, 1.0), (1.0, 1.0), (0.35, 0.0)]],
        8 => &[&[
            (0.16, 0.53),
            (0.0, 0.68),
            (0.0, 0.85),
            (0.16, 1.0),
            (0.84, 1.0),
            (1.0, 0.85),
            (1.0, 0.68),
            (0.84, 0.53),
            (0.16, 0.53),
            (0.0, 0.38),
            (0.0, 0.16),
            (0.16, 0.0),
            (0.84, 0.0),
            (1.0, 0.16),
            (1.0, 0.38),
            (0.84, 0.53),
        ]],
        _ => &[&[(0.1, 0.0), (0.7, 0.0), (1.0, 0.4), (1.0, 0.84), (0.84, 1.0), (0.16, 1.0), (0.0, 0.84), (0.0, 0.56), (0.16, 0.4), (0.84, 0.4), (1.0, 0.56)]],
    }
}

/// Half width / depth / z-centre of the upper torso at chest height `y` (unscaled).
fn torso_at(y: f32) -> (f32, f32, f32) {
    let t = &TORSO_HIGH;
    if y <= t[0].0 {
        return (t[0].1, t[0].2, t[0].3);
    }
    for w in t.windows(2) {
        if y <= w[1].0 {
            let k = (y - w[0].0) / (w[1].0 - w[0].0);
            return (w[0].1 + (w[1].1 - w[0].1) * k, w[0].2 + (w[1].2 - w[0].2) * k, w[0].3 + (w[1].3 - w[0].3) * k);
        }
    }
    let l = t[t.len() - 1];
    (l.1, l.2, l.3)
}

/// Shirt number wrapped around the curved back.
fn shirt_number(b: &mut MeshBuilder, c: &Frame, number: u8, color: Rgba, k: f32, bw: f32) {
    let digits: Vec<u8> = if number >= 10 { vec![number / 10, number % 10] } else { vec![number] };
    let (h, w, gap, stroke) = (0.17, 0.095, 0.03, 0.024);
    let total = digits.len() as f32 * w + (digits.len() as f32 - 1.0) * gap;
    let y0 = 0.02;
    let m = Mat::matte(color);
    for (di, d) in digits.iter().enumerate() {
        let x0 = -total * 0.5 + di as f32 * (w + gap);
        for poly in digit_strokes(*d) {
            for seg in poly.windows(2) {
                let (p0, p1) = (seg[0], seg[1]);
                let n = ((p1.0 - p0.0).abs() * w + (p1.1 - p0.1).abs() * h).max(0.01) / 0.02;
                let n = n.ceil() as usize;
                for j in 0..n {
                    let ta = j as f32 / n as f32;
                    let tb = (j + 1) as f32 / n as f32;
                    // Seen from behind, the player's right is the viewer's right: no mirroring.
                    let pa = (x0 + (p0.0 + (p1.0 - p0.0) * ta) * w, y0 + (p0.1 + (p1.1 - p0.1) * ta) * h);
                    let pb = (x0 + (p0.0 + (p1.0 - p0.0) * tb) * w, y0 + (p0.1 + (p1.1 - p0.1) * tb) * h);
                    let on_back = |x: f32, y: f32| {
                        let (a, bb, zc) = torso_at(y);
                        let (a, bb) = (a * bw, bb * bw);
                        let q = (1.0 - (x / a).powi(2)).max(0.05).sqrt();
                        let z = zc - bb * q - 0.004;
                        let nrm = (c.x * (x / (a * a)) + c.z * (-bb * q / (bb * bb))).normalize_or(-c.z);
                        (c.point(v3(x * k, y * k, z * k)), nrm)
                    };
                    let (qa, na) = on_back(pa.0, pa.1);
                    let (qb, _) = on_back(pb.0, pb.1);
                    let along = (qb - qa).normalize_or(c.y);
                    let side = along.cross(na).normalize_or(c.x) * (stroke * 0.5 * k);
                    b.quad_ex([qa - side, qa + side, qb + side, qb - side], [na; 4], [color; 4], &m);
                }
            }
        }
    }
}

/// Hair cap for `style` following the head profile.
fn hair(b: &mut MeshBuilder, hf: &Frame, style: usize, color: Rgba, k: f32) {
    // (thickness, extra crown height, lowest y at the back, front hairline height)
    let (th, top, low, line) = match style {
        0 => (0.005, 0.0, -0.055, 0.07),  // buzz cut
        1 => (0.013, 0.006, -0.05, 0.075), // short crop
        2 => (0.024, 0.018, -0.045, 0.08), // textured
        3 => (0.045, 0.04, -0.035, 0.078), // afro
        _ => (0.018, 0.01, -0.095, 0.08),  // longer at the back
    };
    let mut rings = Vec::new();
    for &(y, a, bd, cz) in HEAD.iter() {
        if y < low {
            continue;
        }
        let lift = top * ((y - low) / (0.125 - low)).powi(2);
        let back = cz - bd - th;
        // Front edge: full above the hairline, receding to behind the ears below it.
        let full_front = cz + bd + th * 0.6;
        let k_line = ((y - (line - 0.11)) / 0.11).clamp(0.0, 1.0);
        let front = back + (full_front - back) * (0.32 + 0.68 * k_line * k_line);
        let a = a + th;
        rings.push(Ring {
            c: hf.point(v3(0.0, (y + lift) * k, (0.5 * (front + back)) * k)),
            u: hf.x,
            w: hf.z,
            a: a * k,
            b: (0.5 * (front - back)).max(0.004) * k,
        });
    }
    b.loft(&rings, 16, &Mat::matte(color), None, (true, true));
}

/// Builds the whole player model.
pub fn build(b: &mut MeshBuilder, s: &Skeleton, look: &Look) {
    let k = s.body.scale;
    let bw = s.body.build;
    let shirt = Mat::shiny(look.shirt, 0.1);
    let trim = Mat::matte(look.trim);
    let shorts = Mat::shiny(look.shorts, 0.08);
    let socks = Mat::matte(look.socks);
    let skin = Mat::shiny(look.skin, 0.07);
    let boots = Mat::shiny(look.boots, 0.45);
    let seg = 14;

    // --- Torso (shirt), lower part in the spine frame, upper in the chest frame.
    let mut torso = frame_rings(&s.spine, &TORSO_LOW, k, bw);
    torso.extend(frame_rings(&s.chest, &TORSO_HIGH, k, bw));
    b.loft(&torso, seg + 2, &shirt, None, (false, false));
    // Collar.
    let collar = frame_rings(&s.chest, &[(0.25, 0.07, 0.064, -0.02), (0.272, 0.066, 0.06, -0.022)], k, bw);
    b.loft(&collar, seg, &trim, None, (false, false));
    shirt_number(b, &s.chest, look.number, look.number_color, k, bw);

    // --- Shorts around the pelvis.
    let pelvis_shorts = vec![
        Ring { c: s.spine.point(v3(0.0, 0.06 * k, 0.0)), u: s.spine.x, w: s.spine.z, a: 0.15 * k * bw, b: 0.106 * k * bw },
        Ring { c: s.pelvis.point(v3(0.0, -0.015 * k, -0.005 * k)), u: s.pelvis.x, w: s.pelvis.z, a: 0.164 * k * bw, b: 0.112 * k * bw },
        Ring { c: s.pelvis.point(v3(0.0, -0.07 * k, -0.01 * k)), u: s.pelvis.x, w: s.pelvis.z, a: 0.158 * k * bw, b: 0.104 * k * bw },
    ];
    b.loft(&pelvis_shorts, seg, &shorts, None, (false, true));

    for i in 0..2 {
        let (hip, knee, ankle) = (s.hip[i], s.knee[i], s.ankle[i]);
        let front = s.knee_pole[i];
        // Shorts leg.
        let sl = bone_rings(hip, knee, front, &[(-0.05, 0.096, 0.1, 0.0), (0.2, 0.094, 0.097, 0.0), (0.42, 0.092, 0.093, 0.004)], k, bw);
        b.loft(&sl, 12, &shorts, None, (false, false));
        // Thigh.
        let th = bone_rings(
            hip,
            knee,
            front,
            &[(0.3, 0.082, 0.088, 0.004), (0.55, 0.074, 0.081, 0.008), (0.8, 0.061, 0.067, 0.006), (0.96, 0.05, 0.055, 0.0), (1.0, 0.048, 0.052, 0.0)],
            k,
            bw,
        );
        b.loft(&th, 12, &skin, None, (false, false));
        b.sphere(knee + front * (0.008 * k), 0.043 * k * bw, 10, 4, &skin);
        // Calf and sock with shin guard; band at the top of the sock.
        let shin = bone_rings(
            knee,
            ankle,
            front,
            &[
                (0.0, 0.048, 0.05, 0.0),
                (0.075, 0.05, 0.052, 0.0),
                (0.09, 0.055, 0.058, 0.002),
                (0.13, 0.054, 0.058, 0.002),
                (0.3, 0.055, 0.062, -0.006),
                (0.5, 0.05, 0.056, -0.004),
                (0.75, 0.038, 0.045, 0.0),
                (0.95, 0.032, 0.037, 0.0),
                (1.0, 0.031, 0.035, 0.0),
            ],
            k,
            bw,
        );
        let cols = [look.skin, look.skin, look.trim, look.socks, look.socks, look.socks, look.socks, look.socks, look.socks];
        b.loft(&shin, 12, &socks, Some(&cols), (false, false));
        // Boot: heel counter in the accent colour, then the upper, then the sole.
        let f = &s.foot[i];
        let boot = |z: f32, hw: f32, hh: f32, cy: f32| Ring {
            c: f.point(v3(0.0, cy * k, z * k)),
            u: f.x,
            w: f.y,
            a: hw * k * bw.sqrt(),
            b: hh * k,
        };
        let rear = [boot(-0.072, 0.028, 0.03, -0.042), boot(-0.05, 0.04, 0.048, -0.032), boot(-0.012, 0.043, 0.054, -0.03)];
        b.loft(&rear, 12, &Mat::shiny(look.boots_accent, 0.4), None, (true, false));
        let front_part = [
            boot(-0.016, 0.043, 0.054, -0.03),
            boot(0.05, 0.047, 0.046, -0.04),
            boot(0.11, 0.048, 0.036, -0.047),
            boot(0.16, 0.043, 0.027, -0.053),
            boot(0.198, 0.026, 0.017, -0.058),
        ];
        b.loft(&front_part, 12, &boots, None, (false, true));
        b.oriented_box(
            f.point(v3(0.0, -s.body.ankle_height + 0.006 * k, 0.062 * k)),
            [f.x, f.y, f.z],
            v3(0.044 * k, 0.006 * k, 0.135 * k),
            &Mat::matte(scale(look.boots_accent, 0.85)),
        );
    }

    // --- Arms.
    for i in 0..2 {
        let (sh, el, wr) = (s.shoulder[i], s.elbow[i], s.wrist[i]);
        let front = -s.elbow_pole[i];
        let side = if i == 0 { -1.0 } else { 1.0 };
        let delt = sh - s.chest.x * (side * 0.012 * k);
        b.ellipsoid(delt, [s.chest.x, s.chest.y, s.chest.z], v3(0.054 * k * bw, 0.05 * k, 0.058 * k * bw), 12, 6, &shirt);
        let sleeve = bone_rings(sh, el, front, &[(0.0, 0.058, 0.06, 0.0), (0.25, 0.056, 0.058, 0.0), (0.48, 0.054, 0.056, 0.0)], k, bw);
        b.loft(&sleeve, 12, &shirt, None, (false, false));
        let cuff = bone_rings(sh, el, front, &[(0.46, 0.055, 0.057, 0.0), (0.52, 0.054, 0.056, 0.0)], k, bw);
        b.loft(&cuff, 12, &trim, None, (false, false));
        let upper = bone_rings(
            sh,
            el,
            front,
            &[(0.42, 0.046, 0.05, 0.004), (0.62, 0.044, 0.048, 0.006), (0.85, 0.038, 0.039, 0.0), (1.0, 0.035, 0.036, 0.0)],
            k,
            bw,
        );
        b.loft(&upper, 10, &skin, None, (false, false));
        b.sphere(el, 0.035 * k * bw, 8, 3, &skin);
        let fore = bone_rings(
            el,
            wr,
            front,
            &[(0.0, 0.036, 0.038, 0.0), (0.2, 0.042, 0.039, 0.0), (0.5, 0.036, 0.032, 0.0), (0.85, 0.028, 0.024, 0.0), (1.0, 0.026, 0.021, 0.0)],
            k,
            bw,
        );
        b.loft(&fore, 10, &skin, None, (false, false));
        // Hand: palm faces the body, so its width runs along the arm's front axis.
        let dir = (wr - el).normalize_or(-Vec3::Y);
        let g = if look.gloves.is_some() { 1.3 } else { 1.0 };
        let hand_m = look.gloves.map(Mat::matte).unwrap_or(skin);
        let w_ax = (front - dir * front.dot(dir)).normalize_or(dir.any_perpendicular());
        let u_ax = dir.cross(w_ax).normalize_or(Vec3::X);
        let hl = 0.17 * k;
        let hand: Vec<Ring> = [(0.0, 0.016, 0.026), (0.3, 0.019, 0.04), (0.62, 0.017, 0.042), (0.86, 0.013, 0.035), (1.0, 0.009, 0.018)]
            .iter()
            .map(|&(t, a, bb)| Ring { c: wr + dir * (t * hl * g), u: u_ax, w: w_ax, a: a * k * g, b: bb * k * g })
            .collect();
        b.loft(&hand, 10, &hand_m, None, (true, true));
        let thumb_base = wr + dir * (0.04 * k) + w_ax * (0.025 * k * g);
        b.capsule(thumb_base, thumb_base + dir * (0.05 * k * g) + w_ax * (0.02 * k), 0.012 * k * g, 6, 2, &hand_m);
    }

    // --- Neck and head.
    let hf = &s.head;
    let neck = [
        Ring { c: s.neck - s.chest.y * (0.03 * k), u: s.chest.x, w: s.chest.z, a: 0.054 * k * bw, b: 0.056 * k * bw },
        Ring { c: hf.point(v3(0.0, -0.075 * k, -0.012 * k)), u: hf.x, w: hf.z, a: 0.05 * k * bw, b: 0.053 * k * bw },
    ];
    b.loft(&neck, 12, &skin, None, (false, false));
    let head = frame_rings(hf, &HEAD, k, 1.0);
    b.loft(&head, 16, &skin, None, (false, false));
    let feature = Mat::shiny(scale(look.skin, 0.93), 0.07);
    b.capsule(hf.point(v3(0.0, 0.034 * k, 0.097 * k)), hf.point(v3(0.0, -0.004 * k, 0.111 * k)), 0.011 * k, 6, 2, &feature);
    b.sphere(hf.point(v3(0.0, -0.011 * k, 0.104 * k)), 0.016 * k, 8, 3, &feature);
    b.capsule(hf.point(v3(-0.019 * k, -0.05 * k, 0.094 * k)), hf.point(v3(0.019 * k, -0.05 * k, 0.094 * k)), 0.006 * k, 6, 2, &Mat::matte(scale(look.skin, 0.8)));
    for sx in [-1.0f32, 1.0] {
        b.ellipsoid(hf.point(v3(sx * 0.075 * k, 0.004 * k, -0.008 * k)), [hf.x, hf.y, hf.z], v3(0.012 * k, 0.03 * k, 0.02 * k), 8, 5, &feature);
        b.sphere(hf.point(v3(sx * 0.031 * k, 0.028 * k, 0.088 * k)), 0.0085 * k, 6, 3, &Mat::shiny(hex(0x24170f), 0.6));
        b.capsule(
            hf.point(v3(sx * 0.047 * k, 0.047 * k, 0.09 * k)),
            hf.point(v3(sx * 0.016 * k, 0.051 * k, 0.097 * k)),
            0.006 * k,
            5,
            2,
            &Mat::matte(look.hair),
        );
    }
    hair(b, hf, look.hair_style, look.hair, k);
}
