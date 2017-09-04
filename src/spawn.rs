use amethyst::ecs::{Component, VecStorage, System, Fetch, ParJoin, Join, WriteStorage, ReadStorage};
use amethyst::ecs::transform::{LocalTransform, Transform};
use amethyst::ecs::rendering::{MeshComponent, MaterialComponent};
use amethyst::timing::Time;
use amethyst::ecs::{Entities, LazyUpdate};
use amethyst::assets::AssetFuture;
use super::physics::{Body, Shape};
use std::time::Duration;
use cgmath::Rad;
use rayon::iter::ParallelIterator;

#[derive(Copy, Clone)]
pub enum SpawnEvent {
    Start,
    Stop,
    Run,
    RemoveStatic,
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
    type SystemData = (WriteStorage<'a, SpawnEvent>,
     ReadStorage<'a, Body>,
     Fetch<'a, AssetFuture<MeshComponent>>,
     Fetch<'a, AssetFuture<MaterialComponent>>,
     Fetch<'a, Time>,
     Entities<'a>,
     Fetch<'a, LazyUpdate>);

    fn run(&mut self, (mut events, bodies, square, material, time, entities, lazy): Self::SystemData) {
        if let Some(evt) = events.join().last() {
            self.current_state = *evt;
        }
        events.clear();

        self.delta += time.delta_time;
        let wait_time = Duration::from_millis(5);

        if let SpawnEvent::Stop = self.current_state {
            self.delta = Duration::from_millis(0);
        } else {
            match self.current_state {
                SpawnEvent::Start => {
                    self.current_state = SpawnEvent::Run;
                    self.delta += wait_time;
                },
                SpawnEvent::RemoveStatic => {
                    self.current_state = SpawnEvent::Run;

                    (&bodies).par_join().filter(|body| body.inv_mass == 0.0).for_each(|_|{
                        //delete them
                    });
                },
                _ => ()
            }
            
            if let SpawnEvent::Run = self.current_state {
                if self.delta >= wait_time {
                    self.delta -= wait_time;

                    let mut line_transform = LocalTransform::default();
                    line_transform.translation[0] = 512.0;
                    line_transform.translation[1] = 768.0 - 50.0;
                    line_transform.scale[0] = 10.0;
                    line_transform.scale[1] = 10.0;

                    let mut body = Body::new(Shape::Circle { radius: 10.0 });
                    body.set_orient(&mut line_transform, Rad(0.0));

                    let line_entity = entities.create();
                    lazy.insert(line_entity, Transform::default());
                    lazy.insert(line_entity, line_transform);
                    lazy.insert(line_entity, body);
                    lazy.insert(line_entity, square.clone());
                    lazy.insert(line_entity, material.clone());
                }
            }
        }
    }
}
