use amethyst::ecs::{Component, VecStorage, System, Fetch, Join, WriteStorage};
use amethyst::ecs::transform::{LocalTransform, Transform};
use amethyst::input::InputHandler;
use amethyst::assets::AssetFuture;
use amethyst::ecs::rendering::{MeshComponent, MaterialComponent};
use amethyst::ecs::{Entities, LazyUpdate};
use cgmath::{MetricSpace, Rotation3, Vector2, Point2, Quaternion, Vector3, Deg};
use std::f64::consts::PI;
use super::physics::{Body, Shape};

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
    Dragging(Vector2<i32>),
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
    type SystemData = (WriteStorage<'a, LineEvent>,
     Fetch<'a, InputHandler>,
     Entities<'a>,
     Fetch<'a, LazyUpdate>,
     Fetch<'a, AssetFuture<MeshComponent>>,
     Fetch<'a, AssetFuture<MaterialComponent>>);

    fn run(
        &mut self,
        (mut line_events, input, entities, lazy, square, material): Self::SystemData,
    ) {

        for line_event in line_events.join() {
            match *line_event {
                LineEvent::Start => {
                    match self.line_state {
                        LineState::Waiting => {
                            if let Some((x, y)) = input.mouse_position() {
                                self.line_state =
                                    LineState::Dragging(Vector2::new(x as i32, y as i32))
                            } else {
                                //This happens if you click the screen without
                                //moving your mouse inside it first
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

                                let line_start =
                                    Point2::new(line_start.x as f64, line_start.y as f64);
                                let line_end = Point2::new(x as f64, y as f64);

                                let distance = line_start.distance(line_end);

                                let angle = {
                                    let delta_x = line_end.x - line_start.x;
                                    let delta_y = line_end.y - line_start.y;

                                    delta_y.atan2(delta_x) * -180.0 / PI
                                };
                                if distance > 50.0 {
                                    let (line_x, line_y) = (
                                        (line_start.x + line_end.x) / 2.0,
                                        1000.0 -
                                            (line_start.y + line_end.y) / 2.0,
                                    );

                                    let mut line_transform = LocalTransform::default();
                                    line_transform.translation[0] = line_x as f32;
                                    line_transform.translation[1] = line_y as f32;
                                    /*line_transform.rotation = Quaternion::from_axis_angle(
                                        Vector3::new(0f32, 0f32, 1f32),
                                        Deg(angle),
                                    ).into();*/
                                    line_transform.scale[0] = distance as f32;
                                    line_transform.scale[1] = 10.0;

                                    let line_entity = entities.create();
                                    
                                    let mut body = Body::new(
                                        Shape::rect(Vector2::new(distance / 2.0, 5.0))
                                    );
                                    body.static_friction = 0.2;
                                    body.set_static();
                                    body.set_orient(&mut line_transform, Deg(angle));
                                    lazy.insert(line_entity, square.clone());
                                    lazy.insert(line_entity, material.clone());
                                    lazy.insert(line_entity, line_transform);
                                    lazy.insert(line_entity, Transform::default());
                                    lazy.insert(line_entity, body,);
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
