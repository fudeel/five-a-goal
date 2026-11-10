//! CalcioFive game client: window, game loop, rendering and input.

mod hud;
mod input;
mod render;
mod setup;

use calcio_core::{FixedClock, GameEvent, Match, MatchConfig, Phase, PHYSICS_DT};
use input::HumanInput;
use macroquad::prelude::*;
use render::camera::{CamTarget, CameraRig};
use render::anim::Animator;
use render::mesh::draw_meshes;
use render::scene::Scene;
use setup::Setup;

fn window_conf() -> Conf {
    Conf {
        window_title: "CalcioFive".to_owned(),
        window_width: 1600,
        window_height: 900,
        high_dpi: true,
        sample_count: 4,
        ..Default::default()
    }
}

enum Screen {
    Setup,
    Playing,
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut setup = Setup::default();
    // A quiet AI-vs-AI match plays behind the setup menu.
    let backdrop = || {
        Match::new(MatchConfig {
            human_team: None,
            half_length: 600.0,
            ..MatchConfig::default()
        })
    };
    let mut m = backdrop();
    let mut screen = Screen::Setup;
    let mut clock = FixedClock::new(PHYSICS_DT);
    let mut cam = CameraRig::broadcast();
    cam.snap(m.world.ball.pos);
    let scene = Scene::new(&m.world, &cam);
    let mut human = HumanInput::default();
    let mut animator = Animator::new();
    let mut paused = false;
    let mut debug = false;

    loop {
        if is_key_pressed(KeyCode::F1) {
            debug = !debug;
        }
        let frame_dt = get_frame_time();
        match screen {
            Screen::Setup => {
                if let Some(cfg) = setup.update() {
                    m = Match::new(cfg);
                    cam.snap(m.world.ball.pos);
                    human.cancel();
                    paused = false;
                    screen = Screen::Playing;
                }
            }
            Screen::Playing => {
                if is_key_pressed(KeyCode::Escape) {
                    paused = !paused;
                }
                let back = (paused && is_key_pressed(KeyCode::Backspace))
                    || (m.state.phase == Phase::FullTime && is_key_pressed(KeyCode::Enter));
                if back {
                    m = backdrop();
                    screen = Screen::Setup;
                    paused = false;
                }
            }
        }

        let simulate = !paused;
        if simulate {
            if matches!(screen, Screen::Playing) {
                if let Some(team) = m.config.human_team {
                    let attack = m.attack_side(team);
                    human.update(&mut m.world, &cam, attack, frame_dt);
                }
            }
            let steps = clock.advance(frame_dt);
            for _ in 0..steps {
                m.step(PHYSICS_DT);
            }
        } else {
            clock.advance(0.0);
        }
        let (events, _) = m.drain_events();
        for e in &events {
            if let GameEvent::FrameHit { speed, .. } = e {
                cam.kick_shake(speed * 0.006);
            }
        }
        let goal_side = match m.state.phase {
            Phase::Goal { team, .. } => Some(m.attack_side(team)),
            _ => None,
        };
        let human_pos = m.world.human_player().map(|id| m.world.players[id].pos);
        cam.update(
            &CamTarget {
                ball: m.world.ball.pos,
                ball_vel: m.world.ball.vel,
                human: human_pos,
                goal: goal_side,
            },
            frame_dt,
        );

        // 3D
        clear_background(Color::from_rgba(10, 14, 24, 255));
        set_camera(&cam.to_mq());
        draw_meshes(&scene.sky);
        draw_meshes(&scene.arena);
        animator.celebrate = match m.state.phase {
            Phase::Goal { team, .. } => Some(team),
            _ => None,
        };
        animator.update(&m.world, if paused { 0.0 } else { frame_dt.min(0.1) });
        let light = scene.lighting(&cam);
        scene.dynamic(&m.world, &animator, &light).draw(&light);
        if debug {
            render::debug::draw(&m);
        }

        // 2D
        set_default_camera();
        match screen {
            Screen::Setup => setup.draw(),
            Screen::Playing => {
                hud::power_bar(&m.world, &cam, human.charge());
                hud::scoreboard(&m, paused);
                hud::controls_help();
            }
        }
        if debug {
            hud::debug_text(&m);
        } else {
            draw_text(&format!("FPS {}", get_fps()), 16.0, 28.0, 20.0, LIGHTGRAY);
        }

        next_frame().await;
    }
}
