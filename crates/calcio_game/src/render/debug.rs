//! F1 debug overlay in 3D: colliders, keeper hands, AI targets, predicted ball path.

use calcio_core::keeper;
use calcio_core::Match;
use macroquad::prelude::{draw_line_3d, vec3, Color};

fn line(a: calcio_core::math::Vec3, b: calcio_core::math::Vec3, c: Color) {
    draw_line_3d(vec3(a.x, a.y, a.z), vec3(b.x, b.y, b.z), c);
}

pub fn draw(m: &Match) {
    let w = &m.world;
    let yellow = Color::new(1.0, 0.9, 0.2, 1.0);
    let cyan = Color::new(0.3, 0.9, 1.0, 1.0);
    // Goal frame colliders.
    for (_, c) in &w.frame_colliders {
        line(c.a, c.b, yellow);
    }
    // Bodies and keeper hands.
    for p in &w.players {
        let body = p.body(&w.player_spec);
        let col = if p.team == 0 {
            Color::new(1.0, 0.4, 0.4, 1.0)
        } else {
            Color::new(0.4, 0.6, 1.0, 1.0)
        };
        line(body.a, body.b, col);
        if p.role == calcio_core::Role::Goalkeeper {
            let (a, b, _) = keeper::hands_zone(p, &w.keeper_spec);
            line(a, b, cyan);
        }
        // AI target.
        if !p.human {
            let t = m.ai.brains[p.id].target;
            line(
                p.pos + calcio_core::math::v3(0.0, 0.05, 0.0),
                t + calcio_core::math::v3(0.0, 0.05, 0.0),
                col,
            );
        }
    }
    // Predicted ball path.
    let pts = &m.ai.pred.points;
    for i in 1..pts.len() {
        line(pts[i - 1], pts[i], Color::new(1.0, 1.0, 1.0, 0.8));
    }
}
