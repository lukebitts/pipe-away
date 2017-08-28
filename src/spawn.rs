use amethyst::ecs::{Component, VecStorage, System, Fetch, FetchMut, Join, WriteStorage};
use amethyst::ecs::components::{LocalTransform, Transform, Renderable, Child};
use amethyst::ecs::resources::{InputHandler, Time};
use amethyst::ecs::{Entities, LazyUpdate};
use amethyst::asset_manager::AssetManager;
use cgmath;
use cgmath::{MetricSpace, Rotation3};
use std::f32::consts::PI;
use std::time::Duration;

#[derive(Copy, Clone)]
pub enum SpawnEvent {
    Start,
    Stop,
}

impl Component for SpawnEvent {
    type Storage = VecStorage<SpawnEvent>;
}

pub struct SpawnSystem {
    current_state: SpawnEvent,
    delta: Duration,
}

impl SpawnSystem {
    pub fn new() -> Self {
        SpawnSystem {
            current_state: SpawnEvent::Stop,
            delta: Duration::from_millis(0),
        }
    }
}

impl<'a> System<'a> for SpawnSystem {
    type SystemData = (
        WriteStorage<'a, SpawnEvent>, 
        Fetch<'a, Renderable>,
        Fetch<'a, Time>,
        Entities<'a>, 
        Fetch<'a, LazyUpdate>,
    );

    fn run(&mut self, (mut events, square, time, entities, lazy): Self::SystemData) {

        if let Some(evt) = events.join().last() {
            self.current_state = *evt;
        }
        events.clear();

        self.delta += time.delta_time;

        if let SpawnEvent::Start = self.current_state {
            if self.delta > Duration::from_millis(100) {
                self.delta = Duration::from_millis(0);

                {
                    let mut line_transform = LocalTransform::default();
                    line_transform.translation[0] = 300.0;
                    line_transform.translation[1] = 568.0;
                    line_transform.scale[0] = 35.0;
                    line_transform.scale[1] = 35.0;

                    let line_entity = entities.create();
                    lazy.insert(line_entity, square.clone());
                    line_transform.aabb = ((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0));

                    lazy.insert(line_entity, line_transform);
                    lazy.insert(line_entity, Transform::default());
                }

                let mut line_transform = LocalTransform::default();
                line_transform.translation[0] = 50.0;
                line_transform.translation[1] = 768.0 - 50.0;
                line_transform.scale[0] = 50.0;
                line_transform.scale[1] = 50.0;

                let line_entity = entities.create();
                lazy.insert(line_entity, square.clone());

                //let mut spatial = super::Spatial::new();
                //spatial.transform = line_transform;
                line_transform.aabb = ((-25.0, -25.0, -5.0), (25.0, 25.0, 5.0));

                lazy.insert(line_entity, line_transform);
                lazy.insert(line_entity, Transform::default());

                /*let cuboid = super::Ball2::new(10.0);
                let mut body = super::RigidBody::new_dynamic(cuboid, 1.0, 0.2, 0.3);
                body.set_translation(::nalgebra::Translation2::new(50.0, 768.0 - 50.0));

                let handle = {
                    world.add_rigid_body(body)
                };

                lazy.insert(line_entity, super::PhysicsComponent(handle));*/
                lazy.insert(line_entity, super::PhysicsComponent::new_dynamic(50.0));
            }
        }
    }
}