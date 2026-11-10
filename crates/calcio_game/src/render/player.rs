//! Footballer rendering: a procedural joint skeleton dressed with capsules,
//! animated from the simulation (run cycle from distance travelled, kick and
//! tackle swings), plus floodlight shadows and the human-player marker.

use super::light::Lighting;
use super::mesh::{hex, scale, with_alpha, Mat, MeshBuilder, Rgba};
use calcio_core::math::{clamp, v3, Quat, Vec3};
use calcio_core::player::{Player, PlayerSpec, Role};

/// Kit colours for one team.
#[derive(Clone, Copy, Debug)]
pub struct Kit {
    pub shirt: Rgba,
    pub trim: Rgba,
    pub shorts: Rgba,
    pub socks: Rgba,
    pub gk_shirt: Rgba,
    pub gk_shorts: Rgba,
}

pub const KITS: [Kit; 2] = [
    Kit {
        shirt: hex(0xc62f2a),
        trim: hex(0xf2f2f2),
        shorts: hex(0xf2f2f2),
        socks: hex(0xc62f2a),
        gk_shirt: hex(0xb8d936),
        gk_shorts: hex(0x1d1d1f),
    },
    Kit {
        shirt: hex(0x1f5bbf),
        trim: hex(0xf2c230),
        shorts: hex(0x14233f),
        socks: hex(0xf2f2f2),
        gk_shirt: hex(0xf0822a),
        gk_shorts: hex(0x1d1d1f),
    },
];

const SKINS: [u32; 6] = [0xf3d0b0, 0xe7b98f, 0xcf9a6e, 0xa86f47, 0x7c4c30, 0x5a3622];
const HAIRS: [u32; 6] = [0x17120e, 0x2e2118, 0x553a22, 0x8a6538, 0xc9a66b, 0x101010];

fn pick(id: usize, salt: usize, n: usize) -> usize {
    let mut h = (id as u32).wrapping_mul(0x9E37_79B1) ^ (salt as u32).wrapping_mul(0x85EB_CA6B);
    h ^= h >> 15;
    (h as usize) % n
}

/// Joint positions of one pose (world space).
struct Skeleton {
    pelvis: Vec3,
    chest: Vec3,
    neck: Vec3,
    head: Vec3,
    hips: [Vec3; 2],
    knees: [Vec3; 2],
    ankles: [Vec3; 2],
    toes: [Vec3; 2],
    shoulders: [Vec3; 2],
    elbows: [Vec3; 2],
    hands: [Vec3; 2],
    fwd: Vec3,
    side: Vec3,
}

const THIGH: f32 = 0.45;
const SHIN: f32 = 0.44;
const UPPER_ARM: f32 = 0.29;
const FOREARM: f32 = 0.27;

fn limb(down: Vec3, fwd: Vec3, angle: f32) -> Vec3 {
    down * angle.cos() + fwd * angle.sin()
}

fn pose(p: &Player, spec: &PlayerSpec) -> Skeleton {
    let fwd = p.forward();
    let side = v3(-fwd.z, 0.0, fwd.x);
    let down = v3(0.0, -1.0, 0.0);
    let speed = p.speed();
    let a = clamp(speed / spec.sprint_speed, 0.0, 1.0);
    // One full cycle (two steps) every ~2.3 m at speed, shorter strides when slow.
    let cycle = 1.2 + 1.2 * a;
    let phase = p.stride / cycle * std::f32::consts::TAU;
    let swing = 0.1 + 0.52 * a;
    let lean = 0.06 + 0.22 * a;

    // Kick and tackle overrides for the right leg (index 1).
    let kick_w = if p.since_kick < 0.4 {
        1.0 - p.since_kick / 0.4
    } else {
        0.0
    };
    let kick_t = clamp(p.since_kick / 0.28, 0.0, 1.0);
    let tackle_w = if p.since_tackle < 0.45 {
        1.0 - p.since_tackle / 0.45
    } else {
        0.0
    };

    let pelvis0 = v3(0.0, 0.97, 0.0);
    let mut hips = [Vec3::ZERO; 2];
    let mut knees = [Vec3::ZERO; 2];
    let mut ankles = [Vec3::ZERO; 2];
    for i in 0..2 {
        let ph = phase + i as f32 * std::f32::consts::PI;
        let mut alpha = swing * ph.sin();
        let mut beta = 0.12 + a * 1.25 * ph.cos().max(0.0) + 0.1 * (1.0 - a);
        if i == 1 && kick_w > 0.0 {
            let k_alpha = 0.25 + 1.15 * (kick_t * std::f32::consts::PI).sin();
            alpha = alpha * (1.0 - kick_w) + k_alpha * kick_w;
            beta = beta * (1.0 - kick_w) + 0.12 * kick_w;
        }
        if i == 1 && tackle_w > 0.0 {
            alpha = alpha * (1.0 - tackle_w) + 1.05 * tackle_w;
            beta = beta * (1.0 - tackle_w) + 0.05 * tackle_w;
        }
        let s = if i == 0 { -1.0 } else { 1.0 };
        let hip = pelvis0 + side * (0.1 * s);
        let knee = hip + limb(down, fwd, alpha) * THIGH;
        let ankle = knee + limb(down, fwd, alpha - beta) * SHIN;
        hips[i] = hip;
        knees[i] = knee;
        ankles[i] = ankle;
    }
    // Keep the lowest foot on the ground (natural bobbing).
    let lowest = ankles[0].y.min(ankles[1].y);
    let lift = 0.07 - lowest;
    let base = p.pos + v3(0.0, lift, 0.0);
    let tackle_drop = 0.18 * tackle_w;
    let pelvis = base + pelvis0 - v3(0.0, tackle_drop, 0.0);
    let to_world = |l: Vec3| base + l - v3(0.0, tackle_drop, 0.0);
    let hips = [to_world(hips[0]), to_world(hips[1])];
    let knees = [to_world(knees[0]), to_world(knees[1])];
    let ankles = [to_world(ankles[0]), to_world(ankles[1])];
    let toes = [
        ankles[0] + fwd * 0.17 - v3(0.0, 0.03, 0.0),
        ankles[1] + fwd * 0.17 - v3(0.0, 0.03, 0.0),
    ];

    let torso_dir =
        (Vec3::Y + fwd * (lean + 0.25 * tackle_w - 0.15 * kick_w)).normalize_or(Vec3::Y);
    let chest = pelvis + torso_dir * 0.5;
    let neck = chest + torso_dir * 0.1;
    let head = neck + (torso_dir * 0.6 + Vec3::Y * 0.4).normalize_or(Vec3::Y) * 0.15;

    let mut shoulders = [Vec3::ZERO; 2];
    let mut elbows = [Vec3::ZERO; 2];
    let mut hands = [Vec3::ZERO; 2];
    for i in 0..2 {
        let s = if i == 0 { -1.0 } else { 1.0 };
        let ph = phase + i as f32 * std::f32::consts::PI;
        // Arms swing opposite to the legs.
        let arm_a = -0.7 * swing * ph.sin() * (0.4 + 0.6 * a);
        let elbow_bend = 0.25 + 1.1 * a;
        let sh = chest - torso_dir * 0.04 + side * (0.2 * s);
        let out = side * (0.12 * s);
        let up_dir = (limb(down, fwd, arm_a) + out).normalize_or(down);
        let el = sh + up_dir * UPPER_ARM;
        let fo_dir = (limb(down, fwd, arm_a + elbow_bend) + out * 0.5).normalize_or(down);
        shoulders[i] = sh;
        elbows[i] = el;
        hands[i] = el + fo_dir * FOREARM;
    }
    let mut s = Skeleton {
        pelvis,
        chest,
        neck,
        head,
        hips,
        knees,
        ankles,
        toes,
        shoulders,
        elbows,
        hands,
        fwd,
        side,
    };
    keeper_pose(p, &mut s);
    s
}

/// Goalkeeper overrides: arms holding the ball, or a full-stretch dive.
fn keeper_pose(p: &Player, s: &mut Skeleton) {
    if p.role != Role::Goalkeeper {
        return;
    }
    if p.holding_ball {
        let hold = calcio_core::keeper::hold_point(p);
        for i in 0..2 {
            let sgn = if i == 0 { -1.0 } else { 1.0 };
            s.hands[i] = hold + s.side * (0.12 * sgn) - s.fwd * 0.02;
            s.elbows[i] =
                s.shoulders[i].lerp(s.hands[i], 0.5) + s.side * (0.08 * sgn) - v3(0.0, 0.08, 0.0);
        }
        return;
    }
    if p.down <= 0.0 {
        return;
    }
    // Arms stretched above the head, then the whole body rolls towards the dive.
    let up = (s.head - s.pelvis).normalize_or(Vec3::Y);
    for i in 0..2 {
        s.elbows[i] = s.shoulders[i] + up * 0.3 + s.fwd * 0.05;
        s.hands[i] = s.shoulders[i] + up * 0.58 + s.fwd * 0.08;
    }
    let sign = if p.dive_dir.dot(s.side) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let angle = clamp(p.down, 0.0, 1.0) * 1.32 * sign;
    let q = Quat::from_axis_angle(s.fwd, angle);
    let lift = if p.dive_timer > 0.0 {
        let k = 1.0 - p.dive_timer / 0.55;
        v3(
            0.0,
            (p.dive_height * 0.45).min(0.9) * (k * std::f32::consts::PI).sin(),
            0.0,
        )
    } else {
        Vec3::ZERO
    };
    let pivot = p.pos;
    let rot = |j: Vec3| pivot + q.rotate(j - pivot) + lift;
    s.pelvis = rot(s.pelvis);
    s.chest = rot(s.chest);
    s.neck = rot(s.neck);
    s.head = rot(s.head);
    for i in 0..2 {
        s.hips[i] = rot(s.hips[i]);
        s.knees[i] = rot(s.knees[i]);
        s.ankles[i] = rot(s.ankles[i]);
        s.toes[i] = rot(s.toes[i]);
        s.shoulders[i] = rot(s.shoulders[i]);
        s.elbows[i] = rot(s.elbows[i]);
        s.hands[i] = rot(s.hands[i]);
    }
    s.side = q.rotate(s.side);
}

/// Debug: (pelvis, chest, head, left ankle, right ankle) of a pose.
pub fn debug_pose(p: &Player, spec: &PlayerSpec) -> [Vec3; 5] {
    let s = pose(p, spec);
    [s.pelvis, s.chest, s.head, s.ankles[0], s.ankles[1]]
}

pub struct PlayerRenderer;

impl PlayerRenderer {
    pub fn build(&self, b: &mut MeshBuilder, p: &Player, spec: &PlayerSpec) {
        let kit = KITS[p.team.min(1)];
        let gk = p.role == Role::Goalkeeper;
        let shirt_c = if gk { kit.gk_shirt } else { kit.shirt };
        let shorts_c = if gk { kit.gk_shorts } else { kit.shorts };
        let socks_c = if gk { kit.gk_shorts } else { kit.socks };
        let skin_c = hex(SKINS[pick(p.id, 1, SKINS.len())]);
        let hair_c = hex(HAIRS[pick(p.id, 2, HAIRS.len())]);
        let boots_c = if pick(p.id, 3, 3) == 0 {
            hex(0xf2f2f2)
        } else {
            hex(0x151515)
        };

        let shirt = Mat::shiny(shirt_c, 0.08);
        let trim = Mat::matte(kit.trim);
        let shorts = Mat::shiny(shorts_c, 0.05);
        let socks = Mat::matte(socks_c);
        let skin = Mat::shiny(skin_c, 0.06);
        let hair = Mat::matte(hair_c);
        let boots = Mat::shiny(boots_c, 0.4);
        let gloves = Mat::matte(hex(0xeeeeee));

        let s = pose(p, spec);
        let seg = 9;
        let shoulder_axis = (s.shoulders[1] - s.shoulders[0]).normalize_or(s.side) * 0.17;
        // Torso and shorts.
        b.capsule(
            s.pelvis + v3(0.0, 0.1, 0.0),
            s.chest,
            0.165,
            seg + 1,
            3,
            &shirt,
        );
        b.capsule(
            s.chest - shoulder_axis,
            s.chest + shoulder_axis,
            0.085,
            seg,
            2,
            &shirt,
        );
        b.capsule(s.hips[0], s.hips[1], 0.13, seg, 2, &shorts);
        // Collar trim.
        b.capsule(
            s.neck - s.fwd * 0.02,
            s.neck - s.fwd * 0.02,
            0.075,
            seg,
            2,
            &trim,
        );
        // Legs.
        for i in 0..2 {
            let mid = s.hips[i].lerp(s.knees[i], 0.55);
            b.capsule(s.hips[i], mid, 0.088, seg, 2, &shorts);
            b.capsule(mid, s.knees[i], 0.068, seg, 2, &skin);
            let sock_top = s.knees[i].lerp(s.ankles[i], 0.12);
            b.capsule(sock_top, s.ankles[i], 0.058, seg, 2, &socks);
            b.capsule(
                s.ankles[i] - v3(0.0, 0.02, 0.0),
                s.toes[i],
                0.05,
                7,
                2,
                &boots,
            );
        }
        // Arms.
        for i in 0..2 {
            let sleeve = s.shoulders[i].lerp(s.elbows[i], 0.55);
            b.capsule(s.shoulders[i], sleeve, 0.062, seg, 2, &shirt);
            b.capsule(sleeve, s.elbows[i], 0.048, 7, 2, &skin);
            b.capsule(s.elbows[i], s.hands[i], 0.042, 7, 2, &skin);
            let hand_m = if gk { &gloves } else { &skin };
            b.sphere(s.hands[i], if gk { 0.06 } else { 0.045 }, 7, 3, hand_m);
        }
        // Neck, head, hair.
        b.capsule(s.neck, s.head - v3(0.0, 0.05, 0.0), 0.05, 7, 2, &skin);
        b.sphere(s.head, 0.11, 12, 4, &skin);
        let cap = s.head + v3(0.0, 0.03, 0.0) - s.fwd * 0.02;
        b.sphere(cap, 0.104, 12, 4, &hair);
        // Nose hint so the facing direction reads at distance.
        b.sphere(
            s.head + s.fwd * 0.1,
            0.022,
            6,
            2,
            &Mat::matte(scale(skin_c, 0.92)),
        );
    }

    /// Floodlight shadows: a pool at the feet plus a stretched shadow per light.
    pub fn shadows(&self, b: &mut MeshBuilder, p: &Player, spec: &PlayerSpec, light: &Lighting) {
        let m = Mat::unlit([0.0; 4]);
        let y0 = super::arena::LINE_Y + 0.006;
        b.ellipse(
            v3(p.pos.x, y0, p.pos.z),
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
            let c = p.pos + along * (len * 0.5);
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
