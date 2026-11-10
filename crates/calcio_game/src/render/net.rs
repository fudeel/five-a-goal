//! Goal net rendering: every string is a thin, lit, camera-facing ribbon.

use super::mesh::{hex, with_alpha, Mat, MeshBuilder};
use calcio_core::math::Vec3;
use calcio_core::net::GoalNet;

pub fn build(b: &mut MeshBuilder, net: &GoalNet, eye: Vec3) {
    let m = Mat::thin(with_alpha(hex(0xf2f2ee), 0.92));
    for l in &net.links {
        let a = net.pos[l.a as usize];
        let c = net.pos[l.b as usize];
        b.ribbon(a, c, 0.014, eye, &m);
    }
}
