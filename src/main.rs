//#![allow(unused_imports)]
//#![allow(dead_code)]

extern crate amethyst;
extern crate cgmath;
extern crate futures;
extern crate rayon;

use amethyst::assets::{AssetFuture, BoxedErr};
use amethyst::{Application, Engine, State, Trans};
use amethyst::event::{Event, WindowEvent, KeyboardInput, VirtualKeyCode, MouseButton, ElementState};
use amethyst::config::Config;
use amethyst::renderer::{pass, Config as DisplayConfig, Pipeline, Camera, Projection, Stage, Mesh,
                         Texture, MaterialBuilder};
use amethyst::ecs::rendering::{MeshComponent, MaterialComponent, Factory};
use amethyst::renderer::vertex::PosNormTex;
use amethyst::ecs::transform::{Transform, LocalTransform, TransformSystem, Child,
                               Init as InitTransform};
use amethyst::input::InputHandler;
use futures::{IntoFuture, Future};
use cgmath::{Vector2, Rad};

mod line;
use line::*;

mod physics;
use physics::*;

mod spawn;
use spawn::*;

mod duration_utils;
use duration_utils::*;

struct PipeAway;

fn load_proc_asset<T, F>(engine: &mut Engine, f: F) -> AssetFuture<T::Item>
where
    T: IntoFuture<Error = BoxedErr>,
    T::Future: 'static,
    F: FnOnce(&mut Engine) -> T,
{
    let future = f(engine).into_future();
    let future: Box<Future<Item = T::Item, Error = BoxedErr>> = Box::new(future);
    AssetFuture(future.shared())
}

impl State for PipeAway {
    fn on_start(&mut self, engine: &mut Engine) {
        let verts = gen_rectangle(1.0, 1.0);
        let mesh = Mesh::build(verts);
        let tex = Texture::from_color_val([0.0, 0.0, 1.0, 1.0]);
        let mtl = MaterialBuilder::new().with_albedo(tex);

        let mesh = load_proc_asset(engine, move |engine| {
            let factory = engine.world.read_resource::<Factory>();
            factory.create_mesh(mesh).map(MeshComponent::new).map_err(
                BoxedErr::new,
            )
        });

        let mtl = load_proc_asset(engine, move |engine| {
            let factory = engine.world.read_resource::<Factory>();
            factory
                .create_material(mtl)
                .map(MaterialComponent)
                .map_err(BoxedErr::new)
        });

        let camera = {
            struct ScreenDimensions {
                //aspect_ratio: f32,
                w: f32,
                h: f32,
            }

            let dim = ScreenDimensions {
                //aspect_ratio: 768.0 / 1024.0,
                w: 1900.0,
                h: 1000.0,
            };

            let proj = Projection::orthographic(0.0, dim.w, dim.h, 0.0);

            Camera {
                eye: [0., 0., 0.1].into(),
                proj: proj.into(),
                forward: [0.0, 0.0, -1.0].into(),
                right: [1.0, 0.0, 0.0].into(),
                up: [0.0, 1.0, 0.0].into(),
            }
        };

        /*{
            let mut local = LocalTransform::default();
            local.translation = [400.0, 768.0 - 550.0, 0.0];
            local.scale[0] = 300.0;
            local.scale[1] = 20.0;

            let mut body = Body::new(Shape::rect(Vector2::new(150.0, 10.0)));
            body.set_static();
            body.set_orient(&mut local, Rad(0.0));

            engine.world
            .create_entity()
            .with(Transform::default())
            .with(local)
            .with(body)
            .with(mesh.clone())
            .with(mtl.clone())
            .build();
        }

        {
            let mut local = LocalTransform::default();
            local.translation = [500.0, 768.0 - 100.0, 0.0];
            local.scale[0] = 10.0;
            local.scale[1] = 10.0;

            let mut body = Body::new(Shape::Circle{ radius: 10.0 });

            engine.world
            .create_entity()
            .with(Transform::default())
            .with(local)
            .with(body)
            .with(mesh.clone())
            .with(mtl.clone())
            .build();
        }*/

        engine.world.add_resource(InputHandler::new());
        engine.world.add_resource(mesh);
        engine.world.add_resource(mtl);
        engine.world.add_resource(camera);

    }

    fn update(&mut self, _: &mut Engine) -> Trans {
        Trans::None
    }

    fn handle_event(&mut self, engine: &mut Engine, event: Event) -> Trans {
        match event {
            Event::WindowEvent { event, .. } => {
                match event {
                    WindowEvent::MouseInput {
                        state: ElementState::Pressed,
                        button: MouseButton::Left,
                        ..
                    } => {
                        engine.world.create_entity().with(LineEvent::Start).build();
                    }
                    WindowEvent::MouseInput {
                        state: ElementState::Released,
                        button: MouseButton::Left,
                        ..
                    } => {
                        engine.world.create_entity().with(LineEvent::End).build();
                    }
                    WindowEvent::MouseLeft { .. } => {
                        engine.world.create_entity().with(LineEvent::Cancel).build();
                    }
                    WindowEvent::KeyboardInput {
                        input: KeyboardInput {
                            state: ElementState::Released,
                            virtual_keycode: Some(VirtualKeyCode::S),
                            ..
                        },
                        ..
                    } => {
                        engine.world.create_entity().with(SpawnEvent::Start).build();
                    }
                    WindowEvent::KeyboardInput {
                        input: KeyboardInput {
                            state: ElementState::Released,
                            virtual_keycode: Some(VirtualKeyCode::D),
                            ..
                        },
                        ..
                    } => {
                        engine.world.create_entity().with(SpawnEvent::Stop).build();
                    }
                    WindowEvent::KeyboardInput {
                        input: KeyboardInput {
                            state: ElementState::Released,
                            virtual_keycode: Some(VirtualKeyCode::Q),
                            ..
                        },
                        ..
                    } => {
                        engine.world.create_entity().with(SpawnEvent::RemoveDynamic).build();
                    }
                    WindowEvent::KeyboardInput {
                        input: KeyboardInput {
                            state: ElementState::Released,
                            virtual_keycode: Some(VirtualKeyCode::X),
                            ..
                        },
                        ..
                    } => {
                        engine.world.create_entity().with(SpawnEvent::RemoveAll).build();
                    }
                    WindowEvent::KeyboardInput {
                        input: KeyboardInput {
                            state: ElementState::Pressed,
                            virtual_keycode: Some(VirtualKeyCode::Escape),
                            ..
                        },
                        ..
                    } |
                    WindowEvent::Closed => return Trans::Quit,
                    _ => (),
                }
            }
            _ => (),
        }

        Trans::None
    }
}

fn main() {
    let pipe_away = PipeAway;

    let path = format!("{}/Config.ron", env!("CARGO_MANIFEST_DIR"));
    let cfg = DisplayConfig::load(path);
    let mut game = Application::build(pipe_away)
        .expect("app")
        .with_renderer(
            Pipeline::build().with_stage(
                Stage::with_backbuffer()
                    .clear_target([0.00196, 0.23726, 0.21765, 1.0], 1.0)
                    .with_model_pass(pass::DrawFlat::<PosNormTex>::new()),
            ),
            Some(cfg),
        )
        .expect("renderer")
        .register::<LineEvent>()
        .register::<Body>()
        .register::<SpawnEvent>()
        .register::<Transform>()
        .register::<LocalTransform>()
        .register::<Child>()
        .register::<InitTransform>()
        .with::<SpawnSystem>(SpawnSystem::new(), "spawn_system", &[])
        .with::<PhysicsSystem>(PhysicsSystem::new(), "physics_system", &[])
        .with::<LineSystem>(LineSystem::new(), "line_system", &[])
        .with::<TransformSystem>(
            TransformSystem::new(),
            "transform_system",
            &["physics_system"],
        )
        .build()
        .expect("game");
    game.run();
}

fn gen_rectangle(w: f32, h: f32) -> Vec<PosNormTex> {
    let data: Vec<PosNormTex> = vec![
        PosNormTex {
            a_position: [-w / 2., -h / 2., 0.],
            a_normal: [0., 0., 1.],
            a_tex_coord: [0., 0.],
        },
        PosNormTex {
            a_position: [w / 2., -h / 2., 0.],
            a_normal: [0., 0., 1.],
            a_tex_coord: [1., 0.],
        },
        PosNormTex {
            a_position: [w / 2., h / 2., 0.],
            a_normal: [0., 0., 1.],
            a_tex_coord: [1., 1.],
        },
        PosNormTex {
            a_position: [w / 2., h / 2., 0.],
            a_normal: [0., 0., 1.],
            a_tex_coord: [1., 1.],
        },
        PosNormTex {
            a_position: [-w / 2., h / 2., 0.],
            a_normal: [0., 0., 1.],
            a_tex_coord: [1., 1.],
        },
        PosNormTex {
            a_position: [-w / 2., -h / 2., 0.],
            a_normal: [0., 0., 1.],
            a_tex_coord: [1., 1.],
        },
    ];
    data
}
