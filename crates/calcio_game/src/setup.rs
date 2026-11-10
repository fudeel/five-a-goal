//! Minimal pre-match setup screen (keyboard): team size, position, match length,
//! difficulty. The polished menus come in a later phase.

use calcio_core::formation;
use calcio_core::player::Role;
use calcio_core::{Difficulty, MatchConfig};
use macroquad::prelude::*;

const HALF_LENGTHS: [f32; 4] = [60.0, 120.0, 180.0, 300.0];
const DIFFICULTIES: [Difficulty; 3] = [Difficulty::Easy, Difficulty::Normal, Difficulty::Hard];

pub struct Setup {
    pub team_size: usize,
    /// Index into the outfield positions of the current line-up.
    pub position: usize,
    pub length: usize,
    pub difficulty: usize,
    row: usize,
}

impl Default for Setup {
    fn default() -> Self {
        Self {
            team_size: 5,
            position: usize::MAX,
            length: 2,
            difficulty: 1,
            row: 0,
        }
    }
}

fn role_name(r: Role) -> &'static str {
    match r {
        Role::Goalkeeper => "Goalkeeper",
        Role::Defender => "Defender",
        Role::Winger => "Winger",
        Role::Pivot => "Pivot",
    }
}

impl Setup {
    /// Line-up indices the human can pick (outfield players only).
    fn positions(&self) -> Vec<usize> {
        formation::lineup(self.team_size)
            .iter()
            .enumerate()
            .filter(|(_, s)| s.role != Role::Goalkeeper)
            .map(|(i, _)| i)
            .collect()
    }

    fn clamp_position(&mut self) {
        let n = self.positions().len();
        if self.position >= n {
            self.position = n - 1; // default: the most advanced player
        }
    }

    pub fn config(&self) -> MatchConfig {
        let slots = self.positions();
        MatchConfig {
            team_size: self.team_size,
            half_length: HALF_LENGTHS[self.length],
            human_team: Some(0),
            human_slot: slots[self.position.min(slots.len() - 1)],
            difficulty: DIFFICULTIES[self.difficulty],
            seed: (get_time() * 1000.0) as u64 ^ 0x5EED,
        }
    }

    /// Handles input; returns a config when the player starts the match.
    pub fn update(&mut self) -> Option<MatchConfig> {
        self.clamp_position();
        let rows = 5;
        if is_key_pressed(KeyCode::Up) || is_key_pressed(KeyCode::W) {
            self.row = (self.row + rows - 1) % rows;
        }
        if is_key_pressed(KeyCode::Down) || is_key_pressed(KeyCode::S) {
            self.row = (self.row + 1) % rows;
        }
        let left = is_key_pressed(KeyCode::Left) || is_key_pressed(KeyCode::A);
        let right = is_key_pressed(KeyCode::Right) || is_key_pressed(KeyCode::D);
        let step = |v: usize, n: usize| -> usize {
            if left {
                (v + n - 1) % n
            } else if right {
                (v + 1) % n
            } else {
                v
            }
        };
        match self.row {
            0 => {
                let ns = step(self.team_size - 1, 5) + 1;
                if ns != self.team_size {
                    self.team_size = ns;
                    self.position = usize::MAX;
                    self.clamp_position();
                }
            }
            1 => self.position = step(self.position, self.positions().len()),
            2 => self.length = step(self.length, HALF_LENGTHS.len()),
            3 => self.difficulty = step(self.difficulty, DIFFICULTIES.len()),
            _ => {}
        }
        if is_key_pressed(KeyCode::Enter) || (self.row == 4 && is_key_pressed(KeyCode::Space)) {
            return Some(self.config());
        }
        None
    }

    pub fn draw(&self) {
        let (sw, sh) = (screen_width(), screen_height());
        let (w, h) = (560.0, 420.0);
        let (x, y) = (sw * 0.5 - w * 0.5, sh * 0.5 - h * 0.5);
        draw_rectangle(x, y, w, h, Color::from_rgba(8, 12, 22, 225));
        draw_rectangle(x, y, w, 6.0, Color::from_rgba(255, 225, 74, 255));
        let title = "CalcioFive";
        let d = measure_text(title, None, 56, 1.0);
        draw_text(title, sw * 0.5 - d.width * 0.5, y + 66.0, 56.0, WHITE);
        let lineup = formation::lineup(self.team_size);
        let slots = self.positions();
        let pos_idx = slots[self.position.min(slots.len() - 1)];
        let diff = ["Easy", "Normal", "Hard"][self.difficulty];
        let rows = [
            ("Match", format!("{0} v {0}", self.team_size)),
            (
                "You play",
                format!(
                    "{} (#{})",
                    role_name(lineup[pos_idx].role),
                    lineup[pos_idx].number
                ),
            ),
            (
                "Half length",
                format!("{} min", HALF_LENGTHS[self.length] as u32 / 60),
            ),
            ("Difficulty", diff.to_string()),
            ("", "Start match".to_string()),
        ];
        for (i, (label, value)) in rows.iter().enumerate() {
            let ry = y + 130.0 + i as f32 * 52.0;
            let sel = i == self.row;
            if sel {
                draw_rectangle(
                    x + 24.0,
                    ry - 32.0,
                    w - 48.0,
                    44.0,
                    Color::from_rgba(255, 225, 74, 40),
                );
            }
            let c = if sel {
                Color::from_rgba(255, 225, 74, 255)
            } else {
                WHITE
            };
            if label.is_empty() {
                let d = measure_text(value, None, 32, 1.0);
                draw_text(value, sw * 0.5 - d.width * 0.5, ry, 32.0, c);
            } else {
                draw_text(label, x + 48.0, ry, 28.0, LIGHTGRAY);
                let v = format!("<  {value}  >");
                let d = measure_text(&v, None, 28, 1.0);
                draw_text(&v, x + w - 48.0 - d.width, ry, 28.0, c);
            }
        }
        let hint = "Up/Down select   Left/Right change   Enter start";
        let d = measure_text(hint, None, 20, 1.0);
        draw_text(hint, sw * 0.5 - d.width * 0.5, y + h - 22.0, 20.0, GRAY);
    }
}
