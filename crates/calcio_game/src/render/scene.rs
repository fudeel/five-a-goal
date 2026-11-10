//! Everything drawn in 3D for one frame, independent of the window (so it can
//! be reused by tools and tests).

use super::arena;
use super::ball::BallRenderer;
use super::camera::CameraRig;
use super::light::Lighting;
use super::mesh::MeshBuilder;
use super::net;
use super::player::PlayerRenderer;
use super::sky;
use calcio_core::World;
use macroquad::prelude::Mesh;

/// Static meshes (built once) and renderers for dynamic objects.
pub struct Scene {
    pub sky: Vec<Mesh>,
    pub arena: Vec<Mesh>,
    pub ball: BallRenderer,
    pub players: PlayerRenderer,
}

impl Scene {
    pub fn new(world: &World, cam: &CameraRig) -> Self {
        let light = Lighting::floodlit(cam.position);
        Self {
            sky: sky::build().bake(&light),
            arena: arena::build(&world.pitch, cam.position).bake(&light),
            ball: BallRenderer::new(),
            players: PlayerRenderer,
        }
    }

    /// Lighting for this frame.
    pub fn lighting(&self, cam: &CameraRig) -> Lighting {
        Lighting::floodlit(cam.position)
    }

    /// Builds all dynamic geometry for the current world state.
    pub fn dynamic(&self, world: &World, light: &Lighting) -> MeshBuilder {
        let mut b = MeshBuilder::new();
        let r = world.ball_spec.radius;
        let ps = &world.player_spec;
        // Translucent ground decals first, then solid geometry.
        for p in &world.players {
            self.players.shadows(&mut b, p, ps, light);
        }
        self.ball.shadows(&mut b, world.ball.pos, r, light);
        for p in world.players.iter().filter(|p| p.human) {
            self.players.marker(&mut b, p, world.time as f32);
        }
        for p in &world.players {
            self.players.build(&mut b, p, ps);
        }
        self.ball
            .build(&mut b, world.ball.pos, world.ball.orient, r);
        for n in &world.nets {
            net::build(&mut b, n, light.eye);
        }
        b
    }
}
