//! Headless soak tests: full AI-vs-AI matches for every team size.
//!
//! `cargo test -p calcio_core --test soak` runs a quick version;
//! `cargo test -p calcio_core --release --test soak -- --ignored` runs the long one.

use calcio_core::{GameEvent, Match, MatchConfig, Phase, PHYSICS_DT};

struct Report {
    goals: u32,
    kicks: u32,
    max_idle: f32,
}

fn play(size: usize, half: f32, seed: u64) -> Report {
    let mut m = Match::new(MatchConfig {
        team_size: size,
        half_length: half,
        human_team: None,
        seed,
        ..MatchConfig::default()
    });
    let mut kicks = 0;
    let mut idle = 0.0f32;
    let mut max_idle = 0.0f32;
    let mut steps = 0usize;
    while m.is_running() {
        m.step(PHYSICS_DT);
        steps += 1;
        assert!(
            steps < (half as usize * 2 + 120) * 240,
            "size {size} seed {seed}: match never ends ({:?})",
            m.state.phase
        );
        let w = &m.world;
        assert!(w.ball.is_finite(), "size {size} seed {seed}: ball NaN");
        assert!(
            w.pitch.inside_arena(w.ball.pos, 0.05),
            "size {size} seed {seed}: ball escaped {:?}",
            w.ball.pos
        );
        for p in &w.players {
            assert!(
                p.pos.is_finite() && w.pitch.inside_arena(p.pos, 0.05),
                "player escaped {:?}",
                p.pos
            );
        }
        assert!(w.nets.iter().all(|n| n.is_finite()), "net NaN");
        if m.state.phase == Phase::Playing && w.ball.speed() < 0.3 && w.possession.owner.is_none() {
            idle += PHYSICS_DT;
        } else {
            idle = 0.0;
        }
        max_idle = max_idle.max(idle);
        for e in m.drain_events().0 {
            if matches!(e, GameEvent::Kick { .. }) {
                kicks += 1;
            }
        }
    }
    Report {
        goals: m.state.score[0] + m.state.score[1],
        kicks,
        max_idle,
    }
}

#[test]
fn quick_soak_every_team_size() {
    for size in 1..=5 {
        let r = play(size, 40.0, 11 + size as u64);
        assert!(r.kicks > 5, "size {size}: only {} kicks", r.kicks);
        assert!(
            r.max_idle < 14.0,
            "size {size}: ball dead for {:.1}s",
            r.max_idle
        );
    }
}

#[test]
#[ignore]
fn long_soak_every_team_size() {
    for size in 1..=5 {
        let mut goals = 0;
        for seed in 0..4 {
            let r = play(size, 180.0, seed * 977 + 3);
            assert!(
                r.max_idle < 14.0,
                "size {size} seed {seed}: ball dead for {:.1}s",
                r.max_idle
            );
            goals += r.goals;
        }
        assert!(goals > 0, "size {size}: no goals in four full matches");
    }
}
