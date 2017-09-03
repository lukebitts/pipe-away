use std::time::Duration;
use super::{Body, PhysicsInit, FRAME_TIME};
use amethyst::ecs::transform::{LocalTransform, Transform};
use amethyst::timing::Time;
use amethyst::ecs::{System, Fetch, Join, WriteStorage, ReadStorage, Entities};
use cgmath::{Vector2, Vector3, Quaternion, Rotation3};
use super::super::DurationUtils;
use super::collision::{collides, calculate_impulse, positional_correct};

pub struct PhysicsSystem {
    elapsed: Duration,
    iterations: u32,
}

impl PhysicsSystem {
    pub fn new() -> PhysicsSystem {
        PhysicsSystem {
            elapsed: Duration::from_millis(0),
            iterations: 10,
        }
    }
}

impl<'a> System<'a> for PhysicsSystem {
    type SystemData = (WriteStorage<'a, Body>,
     WriteStorage<'a, LocalTransform>,
     WriteStorage<'a, PhysicsInit>,
     ReadStorage<'a, Transform>,
     Entities<'a>,
     Fetch<'a, Time>);
    fn run(
        &mut self,
        (mut physics, mut locals, mut _inits, _globals, entities, time): Self::SystemData,
    ) {
        let delta = time.delta_time * 1;
        let time = delta.as_delta32();

        self.elapsed += delta;

        while self.elapsed >= Duration::from_millis((FRAME_TIME * 1000.0) as u64) {
            self.elapsed -= Duration::from_millis((FRAME_TIME * 1000.0) as u64);

            let bodies: Vec<_> = (&mut physics, &mut locals, &*entities)
                .join()
                .map(|(_, _, e)| e)
                .collect();

            // Generate collision info
            let mut contact_builders = Vec::new();
            for i in 0..bodies.len() {
                let a = bodies[i];

                for &b in bodies.iter().skip(i + 1) {
                    let physic1 = physics.get(a).expect("expected component");
                    let physic2 = physics.get(b).expect("expected component");

                    if physic1.inv_mass == 0.0 && physic2.inv_mass == 0.0 {
                        continue;
                    }

                    let local1 = locals.get(a).expect("expected component");
                    let local2 = locals.get(b).expect("expected component");

                    if let Some(builder) = collides((a, physic1, local1), (b, physic2, local2)) {
                        contact_builders.push(builder);
                    }
                }
            }

            // Integrate forces
            for &body in &bodies {
                physics
                    .get_mut(body)
                    .expect("expected component")
                    .integrate_forces(time);
            }
            //}

            // Initialize collision
            // Solve collisions
            let mut contacts = Vec::new();
            for _ in 0..self.iterations {
                for builder in &contact_builders {
                    let changes = {
                        let a = (
                            physics.get(builder.pair.0).unwrap(),
                            locals.get(builder.pair.0).unwrap(),
                        );
                        let b = (
                            physics.get(builder.pair.1).unwrap(),
                            locals.get(builder.pair.1).unwrap(),
                        );

                        let contact = builder.build(a, b);
                        contacts.push(contact);

                        calculate_impulse(&contacts.last().unwrap(), (a.0.clone(), &a.1), (
                            b.0.clone(),
                            &b.1,
                        ))
                    };

                    *physics.get_mut(builder.pair.0).unwrap() = changes.0;
                    *physics.get_mut(builder.pair.1).unwrap() = changes.1;
                }
            }

            // Integrate velocities
            for &body in &bodies {
                let local = locals.get_mut(body).expect("expected component");
                physics
                    .get_mut(body)
                    .expect("expected component")
                    .integrate_velocity(time, local);
            }
            // Correct positions
            for contact in &mut contacts {
                let (pos1, pos2) = {
                    let a = (
                        physics.get(contact.pair.0).unwrap(),
                        locals.get(contact.pair.0).unwrap(),
                    );
                    let b = (
                        physics.get(contact.pair.1).unwrap(),
                        locals.get(contact.pair.1).unwrap(),
                    );
                    positional_correct(contact, a, b)
                };

                locals.get_mut(contact.pair.0).unwrap().translation =
                    [pos1.x as f32, pos1.y as f32, 0.0];
                locals.get_mut(contact.pair.1).unwrap().translation =
                    [pos2.x as f32, pos2.y as f32, 0.0];
            }

            // Clear all forces
            for &body in &bodies {
                let physic = physics.get_mut(body).expect("expected component");
                physic.force = Vector2::new(0.0, 0.0);
                physic.torque = 0.0;

                //let local = locals.get_mut(body).expect("expected component");
                //local.rotation = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), physic.orient).into();
            }
        }
    }
}
