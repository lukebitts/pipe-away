//! Physics processor based on `nphysics3d`

use std::ops::{Deref, DerefMut};
use std::time::Duration;
use super::{PhysicsComponent, PhysicsInit, PhysicsShape};
use amethyst::ecs::components::{LocalTransform, Transform};
use amethyst::ecs::resources::Time;
use amethyst::ecs::{Component, Entity, VecStorage, System, Fetch, FetchMut, ParJoin, Join, WriteStorage, ReadStorage, AabbStorage, Entities};
use cgmath::{Quaternion, Vector3, Vector2, Rotation3, Deg, Rad};
use super::Real;
use std::f32;
use rayon::iter::ParallelIterator;
use super::super::DurationUtils;
use super::super::{ForceGenerator, Gravity, Drag, Spring};
use cgmath::{MetricSpace, InnerSpace};

struct CollisionPair {
    pub pair: (Entity, Option<Entity>),
    pub restitution: Real,
    pub penetration: Real,
    pub contact_normal: Vector2<Real>

    // Pagina 114
}

impl CollisionPair {
    pub fn resolve<'a>(&self, delta: Duration, physics: &mut WriteStorage<'a, PhysicsComponent>, locals: &mut WriteStorage<'a, LocalTransform>) {
        self.resolve_velocity(delta, physics);
        self.resolve_interpenetration(delta, physics, locals)
    }

    pub fn calculate_separating_velocity<'a>(&self, physics: &mut WriteStorage<'a, PhysicsComponent>) -> Real {
        let mut relative_velocity = physics.get(self.pair.0).unwrap().velocity;

        if let Some(other_entity) = self.pair.1 {
            relative_velocity -= physics.get(other_entity).unwrap().velocity;
        }

        //Dot(v1,v2)=(dx1*dx2)+(dy1*dy2)
        fn dot(v1: Vector2<Real>, v2: Vector2<Real>) -> Real {
            v1.x * v2.x + v1.y * v2.y
        }

        dot(relative_velocity, self.contact_normal)
    }

    fn resolve_velocity<'a>(&self, delta: Duration, physics: &mut WriteStorage<'a, PhysicsComponent>) {
        let delta = delta.as_delta32();

        let separating_velocity = self.calculate_separating_velocity(physics);

        if separating_velocity > 0.0 {
            return
        }

        let new_sep_velocity = -separating_velocity * self.restitution;
        let delta_velocity = new_sep_velocity - separating_velocity;

        let mut total_inverse_mass = physics.get(self.pair.0).unwrap().inverse_mass();
        if let Some(other_entity) = self.pair.1 {
            total_inverse_mass += physics.get(other_entity).unwrap().inverse_mass();
        }

        if total_inverse_mass <= 0.0 { return }

        let impulse = delta_velocity / total_inverse_mass;
        let impulse_per_inverse_mass = self.contact_normal * impulse;

        let inverse_mass = physics.get(self.pair.0).unwrap().inverse_mass();
        physics.get_mut(self.pair.0).unwrap().velocity += impulse_per_inverse_mass * inverse_mass;

        if let Some(other_entity) = self.pair.1 {
            let inverse_mass = physics.get(other_entity).unwrap().inverse_mass();
            physics.get_mut(other_entity).unwrap().velocity += impulse_per_inverse_mass * -inverse_mass;
        }
    }

    fn resolve_interpenetration<'a>(&self, delta: Duration, physics: &mut WriteStorage<'a, PhysicsComponent>, locals: &mut WriteStorage<'a, LocalTransform>) {
        if self.penetration <= 0.0 { return }
        
        let p1_inverse_mass = physics.get(self.pair.0).unwrap().inverse_mass();
        let mut total_inverse_mass = p1_inverse_mass;
        if let Some(other_entity) = self.pair.1 {
            total_inverse_mass += physics.get(other_entity).unwrap().inverse_mass();
        }

        if total_inverse_mass <= 0.0 { return }

        let move_per_inverse_mass = self.contact_normal * (-self.penetration / total_inverse_mass);

        let translation = locals.get_mut(self.pair.0).unwrap().translation;
        let mut translation2d = Vector2::new(translation[0], translation[1]);
        translation2d += move_per_inverse_mass * p1_inverse_mass;

        locals.get_mut(self.pair.0).unwrap().translation = [translation2d.x, translation2d.y, 0.0];

        if let Some(other_entity) = self.pair.1 {
            let p2_inverse_mass = physics.get(other_entity).unwrap().inverse_mass();
            let translation = locals.get_mut(other_entity).unwrap().translation;
            let mut translation2d = Vector2::new(translation[0], translation[1]);
            translation2d += -move_per_inverse_mass * p2_inverse_mass;

            locals.get_mut(other_entity).unwrap().translation = [translation2d.x, translation2d.y, 0.0];
        }
    }
}

/// Handles physics processing for ECS
pub struct PhysicsSystem {
    elapsed: Duration,
    force_generators: Vec<Box<ForceGenerator + Send + Sync>>
}

impl PhysicsSystem {
    pub fn new() -> PhysicsSystem {
        PhysicsSystem {
            elapsed: Duration::from_millis(0),
            force_generators: vec![],
        }
    }
}

pub fn apply_generators<'a>(delta: Duration, local: &mut LocalTransform) {

    /*let generators = replace(&mut self.force_generators, vec![]);

    for generator in &generators {
        generator.update_force(self, local, delta);
    }

    let _ = replace(&mut self.force_generators, generators);*/
}

impl<'a> System<'a> for PhysicsSystem {
    type SystemData = (
        WriteStorage<'a, PhysicsComponent>,
        WriteStorage<'a, LocalTransform>,
        WriteStorage<'a, PhysicsInit>,
        ReadStorage<'a, Transform>,
        Entities<'a>,
        Fetch<'a, Time>
    );
    fn run(&mut self, (mut physics, mut locals, mut inits, globals, entities, time): Self::SystemData) {
        
        let delta = time.delta_time * 1;
        let time = delta.as_delta32();

        self.elapsed += delta;

        /*let mut init_entities = Vec::new();
        for (_, _, entity) in (&mut physics, !&inits, &*entities).join() {
            init_entities.push(entity);

            self.force_generators.push(Box::new(Gravity { target: entity, value: Vector2::new(0.0, -50.0) }));
            //self.force_generators.push(Box::new(Spring { target: entity, other: entity, spring_constant: 0.5, rest_length: 30.0 }));
            self.force_generators.push(Box::new(Drag { target: entity, k1: 0.005, k2: 0.01 }));
        }
        for e in init_entities {
            inits.insert(e, PhysicsInit);
        }

        for generator in &mut self.force_generators {
            generator.update_force(delta, &mut physics, &mut locals);
        }*/

        for (physic, mut local, entity) in (&mut physics, &mut locals, &*entities).join() {

            //physic.apply_generators(delta, &mut local);

            //pagina 84

            physic.add_force(Vector2::new(0.0, -5000.0));

            {
                let mut force = physic.velocity.clone();

                let mut drag_coeff = force.magnitude();
                drag_coeff = 0.005 * drag_coeff + 0.01 * drag_coeff * drag_coeff;

                force.normalize();
                force *= -drag_coeff;
                physic.add_force(force);
            }

            let pos = Vector3::new(physic.velocity.x * time, physic.velocity.y * time, 0.0);

            local.translation[0] += pos.x;
            local.translation[1] += pos.y;
            local.translation[2] = 0.0;

            let mut resulting_acc = physic.acceleration;
            resulting_acc += physic.force_accum * physic.inverse_mass;

            physic.velocity += resulting_acc * time;
            //physic.velocity *= physic.damping.powf(time);
            physic.clear_acumulator();
        };

        // Collision detection
        let collision_pairs : Vec<CollisionPair> = {
            let &(_, aabb_storage) = &globals.open();
            let physical_entities = (&physics, &locals, &*entities)
                .join()
                .map(|(_, _, entity)| entity)
                .collect::<Vec<_>>();

            aabb_storage
                .collision_pairs(&physical_entities)
                .iter()
                .filter_map(|&(e1, e2)| {

                    match (physics.get(e1), physics.get(e2)) {
                        (Some(ref p1), Some(ref p2)) => {
                            match (&p1.shape, &p2.shape) {
                                (&PhysicsShape::Circle(r1), &PhysicsShape::Circle(r2)) => {
                                    let l1 = locals.get(e1).expect("");
                                    let l2 = locals.get(e2).expect("");

                                    let position1 = Vector2::new(l1.translation[0], l1.translation[1]);
                                    let position2 = Vector2::new(l2.translation[0], l2.translation[1]);

                                    let distance = position1.distance(position2);

                                    if distance < (r1 + r2) {
                                        return Some(
                                            CollisionPair{
                                                pair: (e1, Some(e2)), 
                                                restitution: 0.5, 
                                                contact_normal: -(position1 - position2).normalize(),
                                                penetration: (r1 + r2 - distance) / (r1 + r2),
                                            })
                                    }
                                }
                                (&PhysicsShape::Circle(r), &PhysicsShape::Rectangle(h)) => {
                                    let l1 = locals.get(e1).expect("");
                                    let l2 = locals.get(e2).expect("");

                                    let position1 = Vector2::new(l1.translation[0], l1.translation[1]);
                                    let position2 = Vector2::new(l2.translation[0], l2.translation[1]);

                                    
                                }
                                (&PhysicsShape::Rectangle(h), &PhysicsShape::Circle(r)) => {

                                }
                                (&PhysicsShape::Rectangle(h1), &PhysicsShape::Rectangle(h2)) => {

                                }
                            }
                        },
                        _ => unreachable!("not every entity has a physics component")
                    }

                    None
                })
                .collect()
        };

        for pair in collision_pairs {
            for _ in 0..4 {
                pair.resolve(delta, &mut physics, &mut locals);
            }
        }

        /*for (physic, local) in (&mut physics, &mut locals).par_join() {
            let pos = Vector3::new(physic.velocity.x * time, physic.velocity.y * time, 0.0);

            local.translation[0] += pos.x;
            local.translation[1] += pos.y;
            local.translation[2] = 0.0;

            let mut resulting_acc = physic.acceleration;
            resulting_acc += physic.force_accum * physic.inverse_mass;

            physic.velocity += resulting_acc * time;
            physic.velocity *= physic.damping.powf(time);
        }*/


        /*let (_, g) = (&globals).open();

        //let k = g.1.query;

        let collision_pairs : Vec<(Entity, Entity)> = {
            let &(_, aabb_storage) = &globals.open();
            let physical_entities = (&physics, &locals, &*entities)
                .join()
                .map(|(_, _, entity)| entity)
                .collect::<Vec<_>>();

            aabb_storage
                .collision_pairs(&physical_entities)
                .iter()
                .filter_map(|&(e1, e2)| {

                    match (physics.get(e1), physics.get(e2)) {
                        (Some(ref p1), Some(ref p2)) => {
                            match (&p1.shape, &p2.shape) {
                                (&PhysicsShape::Circle(r1), &PhysicsShape::Circle(r2)) => {
                                    
                                }
                                (&PhysicsShape::Circle(r), &PhysicsShape::Rectangle(h)) |
                                (&PhysicsShape::Rectangle(h), &PhysicsShape::Circle(r)) => {

                                }
                                (&PhysicsShape::Rectangle(h1), &PhysicsShape::Rectangle(h2)) => {

                                }
                            }
                        },
                        _ => unreachable!("not every entity has a physics component")
                    }

                    Some((e1, e2))
                })
                .collect()
        };

        for (entity1, entity2) in collision_pairs { }*/

        /*for (physic, local) in (&mut physics, &mut locals).join() {
            let body = handle.borrow();
            let position = body.position();

            // Update translation
            if position.translation.vector.x != local.translation[0] {
                local.translation[0] = position.translation.vector.x;
            }
            if position.translation.vector.y != local.translation[1] {
                local.translation[1] = position.translation.vector.y;
            }
            if local.translation[2] != 0.0 {
                local.translation[2] = 0.0;
            }

            let rot = position.rotation.to_rotation_matrix();
            let angle = rot[(1,0)].atan2(rot[(0,0)]);

            let res = Quaternion::from_axis_angle(Vector3::new(0f32, 0.0, 1.0), Rad(angle)).into();

            if res != local.rotation {
                local.rotation = res;
            }

            // Scaling is not supported yet. See
            // https://github.com/sebcrozet/ncollide/issues/139
            // local.scale[0] = position.scale[0];
            // local.scale[1] = position.scale[1];
            // local.scale[2] = position.scale[2];
        }*/
    }
}
