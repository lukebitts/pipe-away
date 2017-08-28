#![allow(unused_imports)]
#![allow(dead_code)]

extern crate amethyst;
extern crate cgmath;
extern crate ncollide;
extern crate nalgebra;
extern crate aabb_tree;
extern crate rayon;
extern crate num;

use amethyst::{Application, State, Trans, Event, WindowEvent, VirtualKeyCode, ElementState,
               MouseButton};
use amethyst::asset_manager::AssetManager;
use amethyst::config::Config;
use amethyst::ecs::{World, Component, VecStorage, System, Fetch, Join, WriteStorage, ReadStorage, Entity};
use amethyst::ecs::resources::{Camera, InputHandler, Projection, ScreenDimensions};
use amethyst::gfx_device::DisplayConfig;
use amethyst::renderer::Layer;
use amethyst::renderer::pass::{Clear, DrawFlat};
use amethyst::renderer::{Pipeline, VertexPosNormal};
use amethyst::ecs::components::{Mesh, LocalTransform, Texture, Transform};
use amethyst::ecs::systems::TransformSystem;

mod line;
use line::*;

mod physics;
use physics::*;

mod spawn;
use spawn::*;

mod duration_utils;
use duration_utils::*;

struct PipeAway;

impl State for PipeAway {
    fn on_start(&mut self, world: &mut World, assets: &mut AssetManager, pipe: &mut Pipeline) {
        let layer = Layer::new(
            "main",
            vec![
                Clear::new([0.0, 0.0, 0.0, 1.0]),
                DrawFlat::new("main", "main"),
            ],
        );
        pipe.layers.push(layer);

        {
            let dim = world.read_resource::<ScreenDimensions>();
            let mut camera = world.write_resource::<Camera>();
            let aspect_ratio = dim.aspect_ratio;
            let eye = [0., 0., 0.1];
            let target = [0., 0., 0.];
            let up = [0., 1., 0.];

            // Get an Orthographic projection
            let proj = Projection::Orthographic {
                left: -1.0 * aspect_ratio,
                right: 1.0 * aspect_ratio,
                bottom: -1.0,
                top: 1.0,
                near: 0.0,
                far: 1.0,
            };

            let proj = Projection::Orthographic {
                left: 0.0,
                right: dim.w,
                bottom: 0.0,
                top: dim.h,
                near: 0.0,
                far: 1.0,
            };

            camera.proj = proj;
            camera.eye = eye;
            camera.target = target;
            camera.up = up;
        }

        //world.add_resource(aabb_tree::AabbTree::<Entity>::new());

        world.add_resource(InputHandler::new());

        assets.register_asset::<Mesh>();
        assets.register_asset::<Texture>();
        assets.load_asset_from_data::<Texture, [f32; 4]>("white", [1.0, 1.0, 1.0, 1.0]);
        let square_verts = gen_rectangle(1.0, 1.0);
        assets.load_asset_from_data::<Mesh, Vec<VertexPosNormal>>("square", square_verts);
        let square = assets
            .create_renderable("square", "white", "white", "white", 1.0)
            .unwrap();

        world.add_resource(square);
        
    }

    fn update(&mut self, _: &mut World, _: &mut AssetManager, _: &mut Pipeline) -> Trans {
        Trans::None
    }

    fn handle_events(
        &mut self,
        events: &[WindowEvent],
        world: &mut World,
        _: &mut AssetManager,
        _: &mut Pipeline,
    ) -> Trans {

        {
            let mut input = world.write_resource::<InputHandler>();
            input.update(events);
        }

        for e in events {
            match **e {
                Event::MouseInput(ElementState::Pressed, MouseButton::Left) => {
                    world.create_entity().with(LineEvent::Start).build();
                }
                Event::MouseInput(ElementState::Released, MouseButton::Left) => {
                    world.create_entity().with(LineEvent::End).build();
                }
                Event::MouseLeft => {
                    world.create_entity().with(LineEvent::Cancel).build();
                }
                Event::KeyboardInput(_, _, Some(VirtualKeyCode::S)) => {
                    world.create_entity().with(SpawnEvent::Start).build();
                }
                Event::KeyboardInput(_, _, Some(VirtualKeyCode::D)) => {
                    world.create_entity().with(SpawnEvent::Stop).build();
                }
                Event::KeyboardInput(_, _, Some(VirtualKeyCode::Escape)) => return Trans::Quit,
                Event::Closed => return Trans::Quit,
                _ => (),
            }
        }
        Trans::None
    }
}

fn main() {
    let pipe_away = PipeAway;

    let path = format!("{}/Config.ron", env!("CARGO_MANIFEST_DIR"));
    let cfg = DisplayConfig::load(path);
    let mut game = Application::build(pipe_away, cfg)
        .register::<LineEvent>()
        .register::<PhysicsComponent>()
        .register::<PhysicsInit>()
        .register::<SpawnEvent>()
        .register::<Transform>()
        .register::<LocalTransform>()
        .with::<SpawnSystem>(SpawnSystem::new(), "spawn_system", &[])
        .with::<PhysicsSystem>(PhysicsSystem::new(), "physics_system", &[])
        .with::<LineSystem>(LineSystem::new(), "line_system", &[])
        .with::<TransformSystem>(TransformSystem::new(), "transform_system", &["physics_system"])
        .done();
    game.run();
}

fn gen_rectangle(w: f32, h: f32) -> Vec<VertexPosNormal> {
    let data: Vec<VertexPosNormal> = vec![
        VertexPosNormal {
            pos: [-w / 2., -h / 2., 0.],
            normal: [0., 0., 1.],
            tex_coord: [0., 0.],
        },
        VertexPosNormal {
            pos: [w / 2., -h / 2., 0.],
            normal: [0., 0., 1.],
            tex_coord: [1., 0.],
        },
        VertexPosNormal {
            pos: [w / 2., h / 2., 0.],
            normal: [0., 0., 1.],
            tex_coord: [1., 1.],
        },
        VertexPosNormal {
            pos: [w / 2., h / 2., 0.],
            normal: [0., 0., 1.],
            tex_coord: [1., 1.],
        },
        VertexPosNormal {
            pos: [-w / 2., h / 2., 0.],
            normal: [0., 0., 1.],
            tex_coord: [1., 1.],
        },
        VertexPosNormal {
            pos: [-w / 2., -h / 2., 0.],
            normal: [0., 0., 1.],
            tex_coord: [1., 1.],
        },
    ];
    data
}

