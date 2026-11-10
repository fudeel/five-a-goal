//! Keyboard controls for the human footballer.
//!
//! * WASD / arrows: move (relative to the camera); Shift: sprint
//! * J shoot, K pass, L lob — hold to charge, release to kick
//! * Q / E held while releasing: curl left / right
//! * Space: standing tackle

use crate::render::camera::CameraRig;
use calcio_core::kick::{self, KickKind, KickRequest};
use calcio_core::math::{clamp, Vec3};
use calcio_core::pitch::Side;
use calcio_core::World;
use macroquad::prelude::*;

/// Seconds of holding for a full-power kick.
pub const CHARGE_TIME: f32 = 0.9;

#[derive(Default)]
pub struct HumanInput {
    charging: Option<(KickKind, f32)>,
}

fn key_for(kind: KickKind) -> KeyCode {
    match kind {
        KickKind::Shot => KeyCode::J,
        KickKind::Pass => KeyCode::K,
        KickKind::Lob => KeyCode::L,
    }
}

impl HumanInput {
    /// Current charge (kind, power 0..1) for the HUD.
    pub fn charge(&self) -> Option<(KickKind, f32)> {
        self.charging
            .map(|(k, t)| (k, clamp(t / CHARGE_TIME, 0.0, 1.0)))
    }

    pub fn cancel(&mut self) {
        self.charging = None;
    }

    /// Reads the keyboard and writes the intents of the human player.
    /// `attack` is the goal the human's team is attacking.
    pub fn update(&mut self, world: &mut World, cam: &CameraRig, attack: Side, frame_dt: f32) {
        let Some(id) = world.human_player() else {
            return;
        };
        let mut sx = 0.0f32;
        let mut sy = 0.0f32;
        if is_key_down(KeyCode::W) || is_key_down(KeyCode::Up) {
            sy += 1.0;
        }
        if is_key_down(KeyCode::S) || is_key_down(KeyCode::Down) {
            sy -= 1.0;
        }
        if is_key_down(KeyCode::D) || is_key_down(KeyCode::Right) {
            sx += 1.0;
        }
        if is_key_down(KeyCode::A) || is_key_down(KeyCode::Left) {
            sx -= 1.0;
        }
        let (right, up) = cam.ground_axes();
        let mv = (right * sx + up * sy).normalize_or_zero();
        let sprint = is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift);
        let spec = world.player_spec.clone();
        {
            let p = &mut world.players[id];
            p.sprint = sprint;
            p.desired = mv
                * if sprint {
                    spec.sprint_speed
                } else {
                    spec.jog_speed
                };
            p.dribble = true;
        }

        // Kick charging.
        if self.charging.is_none() {
            for kind in [KickKind::Shot, KickKind::Pass, KickKind::Lob] {
                if is_key_pressed(key_for(kind)) {
                    self.charging = Some((kind, 0.0));
                    break;
                }
            }
        }
        if let Some((kind, t)) = self.charging {
            let t = t + frame_dt;
            let released = !is_key_down(key_for(kind));
            if released || t > CHARGE_TIME * 1.6 {
                self.charging = None;
                let power = clamp(t / CHARGE_TIME, 0.08, 1.0);
                let mut curl = 0.0;
                if is_key_down(KeyCode::Q) {
                    curl += 1.0;
                }
                if is_key_down(KeyCode::E) {
                    curl -= 1.0;
                }
                let p = &world.players[id];
                let dir = if mv != Vec3::ZERO { mv } else { p.forward() };
                let mut req = KickRequest::new(kind, dir, power).with_curl(curl);
                match kind {
                    KickKind::Shot => {
                        // Aim across the mouth with the lateral part of the input.
                        let to_goal = (world.pitch.goal_centre(attack) - p.pos)
                            .flat()
                            .normalize_or(Vec3::X);
                        let side_axis = to_goal.cross(Vec3::Y).normalize_or(Vec3::Z);
                        let mut lateral = mv.dot(side_axis) * 1.1;
                        if mv == Vec3::ZERO {
                            // No input: aim for the far post.
                            lateral = if p.pos.z > 0.0 { -0.6 } else { 0.6 } * side_axis.z.signum();
                        }
                        // goal_target's lateral runs along +z.
                        let lat_z = lateral * side_axis.z.signum();
                        let height = 0.35 + 1.25 * power * power;
                        req =
                            req.with_target(kick::goal_target(&world.pitch, attack, lat_z, height));
                        req.dir = to_goal;
                    }
                    KickKind::Lob => {
                        let mates: Vec<_> =
                            world.players.iter().filter(|m| m.team == p.team).collect();
                        if let Some(t) =
                            kick::auto_target(p, &mates, dir, world.kick_spec.assist_cone * 0.8)
                        {
                            req = req.with_target(t);
                        }
                    }
                    KickKind::Pass => {}
                }
                world.players[id].pending_kick = Some(req);
            } else {
                self.charging = Some((kind, t));
            }
        }

        if is_key_pressed(KeyCode::Space) {
            world.players[id].tackle_request = true;
        }
    }
}
