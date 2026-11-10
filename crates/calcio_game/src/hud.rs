//! Minimal heads-up display: kick power bar and debug text.

use crate::render::camera::CameraRig;
use calcio_core::kick::KickKind;
use calcio_core::math::v3;
use calcio_core::World;
use macroquad::prelude::*;

/// Power bar above the human player while a kick is charging.
pub fn power_bar(world: &World, cam: &CameraRig, charge: Option<(KickKind, f32)>) {
    let (Some(id), Some((kind, power))) = (world.human_player(), charge) else {
        return;
    };
    let p = &world.players[id];
    let Some((x, y)) = cam.project(p.pos + v3(0.0, 2.55, 0.0), screen_width(), screen_height())
    else {
        return;
    };
    let (w, h) = (90.0, 11.0);
    let (bx, by) = (x - w * 0.5, y - h * 0.5);
    let col = match kind {
        KickKind::Shot => Color::from_rgba(240, 80, 60, 255),
        KickKind::Pass => Color::from_rgba(80, 200, 120, 255),
        KickKind::Lob => Color::from_rgba(90, 160, 240, 255),
        KickKind::Cross => Color::from_rgba(250, 170, 60, 255),
        KickKind::Throw | KickKind::Roll => Color::from_rgba(230, 230, 230, 255),
    };
    draw_rectangle(
        bx - 2.0,
        by - 2.0,
        w + 4.0,
        h + 4.0,
        Color::from_rgba(0, 0, 0, 170),
    );
    draw_rectangle(bx, by, w * power, h, col);
    draw_rectangle_lines(
        bx - 2.0,
        by - 2.0,
        w + 4.0,
        h + 4.0,
        1.5,
        Color::from_rgba(255, 255, 255, 200),
    );
    let label = match kind {
        KickKind::Shot => "SHOT",
        KickKind::Pass => "PASS",
        KickKind::Lob => "LOB",
        KickKind::Cross => "CROSS",
        KickKind::Throw => "THROW",
        KickKind::Roll => "ROLL",
    };
    draw_text(label, bx, by - 6.0, 18.0, WHITE);
}

/// Small help line at the bottom of the screen.
pub fn controls_help() {
    let s = "WASD move  Shift sprint  J shoot  K pass  L lob  I cross (hold)  Q/E curl  Space tackle  Esc pause  F1 debug";
    draw_text(
        s,
        16.0,
        screen_height() - 16.0,
        18.0,
        Color::from_rgba(220, 220, 220, 200),
    );
}

fn centered(text: &str, y: f32, size: f32, color: Color) {
    let d = measure_text(text, None, size as u16, 1.0);
    let x = screen_width() * 0.5 - d.width * 0.5;
    draw_text(text, x + 2.0, y + 2.0, size, Color::from_rgba(0, 0, 0, 160));
    draw_text(text, x, y, size, color);
}

/// Score, clock and phase banners.
pub fn scoreboard(m: &calcio_core::Match, paused: bool) {
    use calcio_core::Phase;
    let st = &m.state;
    let (w, h) = (420.0, 44.0);
    let x = screen_width() * 0.5 - w * 0.5;
    draw_rectangle(x, 12.0, w, h, Color::from_rgba(8, 12, 22, 210));
    draw_rectangle(x, 12.0, 8.0, h, Color::from_rgba(198, 47, 42, 255));
    draw_rectangle(
        x + w - 8.0,
        12.0,
        8.0,
        h,
        Color::from_rgba(31, 91, 191, 255),
    );
    let half = if st.half == 1 { "1st" } else { "2nd" };
    let text = format!(
        "HOME  {}  -  {}  AWAY     {}  {}",
        st.score[0],
        st.score[1],
        half,
        m.clock_text()
    );
    let d = measure_text(&text, None, 26, 1.0);
    draw_text(
        &text,
        screen_width() * 0.5 - d.width * 0.5,
        12.0 + h * 0.5 + 9.0,
        26.0,
        WHITE,
    );

    let mid = screen_height() * 0.32;
    match st.phase {
        Phase::Goal { team, .. } => {
            let who = if team == 0 { "HOME" } else { "AWAY" };
            centered("GOAL!", mid, 96.0, Color::from_rgba(255, 225, 74, 255));
            centered(&format!("{who} scores"), mid + 48.0, 32.0, WHITE);
        }
        Phase::HalfTime { .. } => centered("HALF TIME", mid, 64.0, WHITE),
        Phase::FullTime => {
            centered("FULL TIME", mid, 72.0, WHITE);
            centered(
                &format!("{} - {}", st.score[0], st.score[1]),
                mid + 52.0,
                44.0,
                WHITE,
            );
            centered("press Enter for a new match", mid + 96.0, 26.0, LIGHTGRAY);
        }
        Phase::Kickoff { .. } => {
            if let Some(t) = st.kickoff_taker {
                if m.world.players[t].human {
                    centered("Your kick-off: pass with K", mid + 20.0, 28.0, WHITE);
                }
            }
        }
        Phase::Playing => {}
    }
    if paused {
        centered("PAUSED", screen_height() * 0.5, 64.0, WHITE);
        centered(
            "Esc to resume   Backspace to quit to setup",
            screen_height() * 0.5 + 40.0,
            24.0,
            LIGHTGRAY,
        );
    }
}

/// F1 debug text.
pub fn debug_text(m: &calcio_core::Match) {
    let w = &m.world;
    let b = &w.ball;
    let lines = [
        format!("FPS {}   phase {:?}", get_fps(), m.state.phase),
        format!(
            "ball {:.1} m/s  spin {:.0} rad/s  height {:.2} m",
            b.speed(),
            b.spin.length(),
            b.height(&w.ball_spec)
        ),
        format!(
            "owner {:?}  last touch {:?}",
            w.possession.owner, w.possession.last_touch
        ),
        format!(
            "chasers {:?}  team phase {:?}",
            m.ai.chaser, m.ai.team_phase
        ),
        format!("nets asleep {} {}", w.nets[0].sleeping, w.nets[1].sleeping),
    ];
    let h = 22.0 * lines.len() as f32 + 12.0;
    draw_rectangle(10.0, 64.0, 560.0, h, Color::from_rgba(0, 0, 0, 160));
    for (i, l) in lines.iter().enumerate() {
        draw_text(l, 18.0, 84.0 + 22.0 * i as f32, 20.0, WHITE);
    }
}
