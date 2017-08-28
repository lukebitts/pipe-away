use amethyst::ecs::{Component, VecStorage, System, Fetch, FetchMut, Join, WriteStorage};
use amethyst::ecs::components::{LocalTransform, Transform, Renderable, Child};
use amethyst::ecs::resources::{InputHandler};
use amethyst::ecs::{Entities, LazyUpdate};
use amethyst::asset_manager::AssetManager;
use cgmath;
use cgmath::{MetricSpace, Rotation3, Vector2};
use std::f32::consts::PI;

pub enum LineEvent {
    Start,
    End,
    Cancel,
}

impl Component for LineEvent {
    type Storage = VecStorage<LineEvent>;
}

enum LineState {
    Waiting,
    Dragging(cgmath::Vector2<i32>),
}

pub struct LineSystem {
    line_state: LineState,
}

impl LineSystem {
    pub fn new() -> Self {
        LineSystem { line_state: LineState::Waiting }
    }
}

impl<'a> System<'a> for LineSystem {
    type SystemData = (
        WriteStorage<'a, LineEvent>, 
        Fetch<'a, InputHandler>,
        Entities<'a>, 
        Fetch<'a, LazyUpdate>,
        Fetch<'a, Renderable>,
    );

    fn run(&mut self, (mut line_events, input, entities, lazy, square): Self::SystemData) {

        for line_event in line_events.join() {
            match *line_event {
                LineEvent::Start => {
                    match self.line_state {
                        LineState::Waiting => {
                            if let Some((x, y)) = input.mouse_position() {
                                self.line_state = LineState::Dragging(cgmath::Vector2::new(x, y))
                            } else {
                                //debug_assert!(false, "mouse has no position")
                                //This happens if you click the screen without moving your mouse inside it first
                            }
                        }
                        LineState::Dragging(_) => debug_assert!(false, "invalid state"),
                    };
                }
                LineEvent::End => {
                    match self.line_state {
                        LineState::Waiting => {
                            // After the mouse leaves the screen we might receive a
                            // LineEvent::End without a LineEvent::Start
                        }
                        LineState::Dragging(line_start) => {
                            if let Some((x, y)) = input.mouse_position() {
                                self.line_state = LineState::Waiting;
                                
                                let line_start = cgmath::Point2::new(line_start.x as f32, line_start.y as f32);
                                let line_end = cgmath::Point2::new(x as f32, y as f32);

                                let distance = line_start.distance(line_end);

                                let angle = {
                                    let delta_x = line_end.x - line_start.x;
                                    let delta_y = line_end.y - line_start.y;

                                    delta_y.atan2(delta_x) * -180.0 / PI
                                };

                                if distance > 50.0 {
                                    let (line_x, line_y) = (
                                        (line_start.x + line_end.x) / 2.0 , 
                                        768.0 - (line_start.y + line_end.y) / 2.0
                                    );

                                    let mut line_transform = LocalTransform::default();
                                    line_transform.translation[0] = line_x;
                                    line_transform.translation[1] = line_y;
                                    line_transform.rotation = cgmath::Quaternion::from_axis_angle(cgmath::Vector3::new(0f32, 0f32, 1f32), cgmath::Deg(angle)).into();
                                    line_transform.scale[0] = distance;
                                    line_transform.scale[1] = 10.0;

                                    let line_entity = entities.create();
                                    lazy.insert(line_entity, square.clone());

                                    //let mut line_spatial = super::Spatial::new();
                                    //line_spatial.transform = line_transform;
                                    //line_spatial.aabb = ((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0));

                                    lazy.insert(line_entity, line_transform);
                                    lazy.insert(line_entity, Transform::default());


                                    //let mut parent_transform = LocalTransform::default();
                                    //parent_transform.translation[0] = line_x;
                                    //parent_transform.translation[1] = line_y;
                                    //parent_transform.rotation = cgmath::Quaternion::from_axis_angle(cgmath::Vector3::new(0f32, 0f32, 1f32), cgmath::Deg(angle)).into();

                                    //let parent_entity = entities.create();
                                    //let mut parent_spatial = super::Spatial::new();
                                    //parent_spatial.transform = parent_transform;
                                    //parent_spatial.aabb = ((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0));

                                    //lazy.insert(parent_entity, parent_transform);
                                    //lazy.insert(parent_entity, Transform::default());

                                    //lazy.insert(line_entity, Child::new(parent_entity));

                                    lazy.insert(line_entity, super::PhysicsComponent::new_static(Vector2::new(50.0, 50.0)));
                                }

                            } else {
                                debug_assert!(false, "mouse has no position")
                            }
                        }
                    };
                }
                LineEvent::Cancel => self.line_state = LineState::Waiting,
            };
        }

        line_events.clear()

    }
}
