use super::{collision, float_cmp, cross_vectors, cross_real_vector, Real, Body, Shape, Manifold, Vec2, FRAME_TIME};
use super::super::DurationUtils;
use amethyst::ecs::{System, Fetch, Join, ParJoin, WriteStorage, ReadStorage, Entities};
use amethyst::ecs::transform::{LocalTransform, Init};
use amethyst::timing::Time;
use rayon::iter::ParallelIterator;
use rayon::prelude::*;
use std::sync::Mutex;
use cgmath::{dot, InnerSpace};

pub struct PhysicsSystem {
    acumulator: Real,
}

impl PhysicsSystem {
    pub fn new() -> Self {
        PhysicsSystem {
            acumulator: 0.0
        }
    }

    // Manifold::ApplyImpulse
    fn apply_impulse(&mut self, m: &Manifold, 
        (mut body_a, local_a): (Body, &LocalTransform), 
        (mut body_b, local_b): (Body, &LocalTransform)
    ) -> (Body, Body) {
        //let (i_a, i_b) = m.pair;
        //let (body_a, body_b) = self.get_two_mut(i_a, i_b);

        if float_cmp(body_a.inv_mass + body_b.inv_mass, 0.0) {
            //InfiniteMassCorrection
            body_a.velocity = Vec2::new(0.0, 0.0);
            body_b.velocity = Vec2::new(0.0, 0.0);

            return (body_a, body_b);
        }

        for contact in &m.contacts {
            let ra = contact - Vec2::new(local_a.translation[0], local_a.translation[1]);
            let rb = contact - Vec2::new(local_b.translation[0], local_b.translation[1]);
            
            let (inv_mass_sum, j, impulse) = {
                let rv = body_b.velocity + cross_real_vector(body_b.angular_velocity, rb) -
                         body_a.velocity - cross_real_vector(body_a.angular_velocity, ra);

                let contact_vel = dot(rv, m.normal);
                if contact_vel > 0.0 {
                    return (body_a, body_b)
                }

                let ra_cross_n = cross_vectors(ra, m.normal);
                let rb_cross_n = cross_vectors(rb, m.normal);
                
                let inv_mass_sum = 
                    body_a.inv_mass + body_b.inv_mass +
                    ra_cross_n.powi(2) * body_a.inv_inertia +
                    rb_cross_n.powi(2) * body_b.inv_inertia;
                
                let j = -(1.0 + m.e) * contact_vel / inv_mass_sum / m.contacts.len() as f32;
                (inv_mass_sum, j, m.normal * j)
            };

            body_a.apply_impulse(-impulse, ra);
            body_b.apply_impulse( impulse, rb);

            let tangent_impulse = {
                // rv = B->velocity + Cross( B->angularVelocity, rb ) -
                //      A->velocity - Cross( A->angularVelocity, ra );
                let rv = body_b.velocity + cross_real_vector(body_b.angular_velocity, rb) -
                         body_a.velocity - cross_real_vector(body_a.angular_velocity, ra);

                // Vec2 t = rv - (normal * Dot( rv, normal ));
                // t.Normalize( );
                //let unsafe_rv = UnsafeVec2::new(rv.x, rv.y);
                //let unsafe_normal = UnsafeVec2::new(m.normal.x, m.normal.y);
                
                let mut t = rv - (m.normal * dot(rv, m.normal));
                let len_t = t.magnitude();
                //if len_t is too small we can't normalize the vector, since it would divide by zero
                if !float_cmp(len_t, 0.0) {
                    t = t.normalize();
                }

                //let t = Vec2::new(unsafe_t.x, unsafe_t.y);

                let jt = -dot(rv, t) / inv_mass_sum / m.contacts.len() as f32;

                if float_cmp(jt, 0.0) {
                    return (body_a, body_b)
                }

                if jt.abs() < j * m.sf {
                    t * jt
                } else {
                    t * -j * m.df
                }
            };

            body_a.apply_impulse(-tangent_impulse, ra);
            body_b.apply_impulse( tangent_impulse, rb);
        }
        (body_a, body_b)
    }

    // Manifold::PositionalCorrect
    fn positional_correct(&mut self, m: &Manifold,
        (body_a, mut local_a): (&Body, LocalTransform), 
        (body_b, mut local_b): (&Body, LocalTransform)
    ) -> (LocalTransform, LocalTransform) {
        let k_slop = 0.05;
        let percent = 0.4;

        //let unsafe_normal = UnsafeVec2::new(m.normal.x, m.normal.y);
        let correction = ((m.penetration - k_slop).max(0.0) / (body_a.inv_mass + body_b.inv_mass)) * m.normal * percent;
        //let correction = Vec2::new(unsafe_correction.x, unsafe_correction.y);

        let pos_a = Vec2::new(local_a.translation[0], local_a.translation[1]) - correction * body_a.inv_mass;
        let pos_b = Vec2::new(local_b.translation[0], local_b.translation[1]) + correction * body_b.inv_mass;

        local_a.translation = [pos_a.x, pos_a.y, 0.0];
        local_b.translation = [pos_b.x, pos_b.y, 0.0];

        (local_a, local_b)
    }
}

impl<'a> System<'a> for PhysicsSystem {
    type SystemData = (
        WriteStorage<'a, Body>,
        WriteStorage<'a, LocalTransform>,
        ReadStorage<'a, Init>,
        Entities<'a>,
        Fetch<'a, Time>,
    );
    fn run(&mut self, (mut bodies, mut locals, inits, entities, time): Self::SystemData) {
        let delta = time.delta_time.as_delta32();
        self.acumulator += delta;
        
        while self.acumulator >= FRAME_TIME {
            self.acumulator -= FRAME_TIME;
        
            let contact_data = {
                let data = Mutex::new(Vec::new());
                
                (&*entities, &bodies, &locals).par_join()
                .for_each(|(i_a, body_a, local_a)|{
                    let mut ret = Vec::new();
                    for (i_b, body_b, local_b) in (&*entities, &bodies, &locals).join().filter(|&(i, _, _)| i.id() > i_a.id()) {
                        if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
                            continue
                        }
                        match (&body_a.shape, &body_b.shape) {
                            (&Shape::Circle { radius: r1 }, &Shape::Circle { radius: r2 }) => {
                                if let Some(manifold_data) = collision::circle_circle(
                                    (i_a, r1, local_a),
                                    (i_b, r2, local_b)
                                ) {
                                    ret.push(manifold_data);
                                }
                            }
                            (&Shape::Circle { radius }, &Shape::Polygon { ref orientation, ref vertices }) => {
                                if let Some(manifold_data) = collision::circle_polygon(
                                    (i_a, radius, local_a),
                                    (i_b, orientation, vertices, local_b)
                                ) {
                                    ret.push(manifold_data);
                                }
                            }
                            (&Shape::Polygon { ref orientation, ref vertices }, &Shape::Circle { radius }) => {
                                if let Some(manifold_data) = collision::circle_polygon(
                                    (i_b, radius, local_b),
                                    (i_a, orientation, vertices, local_a)
                                ) {
                                    ret.push(manifold_data);
                                }
                            }
                            _ => unimplemented!()
                        }
                    }
                    let mut v = data.lock().unwrap();
                    v.extend(ret);
                });

                data.into_inner().unwrap()
            };

            (&mut bodies, &locals).par_join().for_each(|(body, _)|{
                body.integrate_forces(FRAME_TIME);
            });
            
            let contacts : Vec<_> = contact_data.par_iter().map(|data|{
                data.initialize(
                    FRAME_TIME, 
                    (bodies.get(data.pair.0).unwrap(), locals.get(data.pair.0).unwrap()), 
                    (bodies.get(data.pair.1).unwrap(), locals.get(data.pair.1).unwrap())
                )
            }).collect();

            //TODO: When performance becomes important, see if changing this to a multithreaded version is better
            let iterations = 10;
            for _ in 0..iterations {
                for contact in &contacts { 
                    let changes = {
                        let a = (bodies.get(contact.pair.0).unwrap().clone(), locals.get(contact.pair.0).unwrap());
                        let b = (bodies.get(contact.pair.1).unwrap().clone(), locals.get(contact.pair.1).unwrap());
                        self.apply_impulse(&contact, a, b)
                    };
                    
                    *bodies.get_mut(contact.pair.0).unwrap() = changes.0;
                    *bodies.get_mut(contact.pair.1).unwrap() = changes.1;
                }
            }

            (&mut bodies, &mut locals).par_join().for_each(|(body, local)|{
                body.integrate_velocity(local, FRAME_TIME);
            });

            for contact in &contacts {
                let changes = {
                    let a = (bodies.get(contact.pair.0).unwrap(), locals.get(contact.pair.0).unwrap().clone());
                    let b = (bodies.get(contact.pair.1).unwrap(), locals.get(contact.pair.1).unwrap().clone());
                    self.positional_correct(&contact, a, b)
                };
                
                *locals.get_mut(contact.pair.0).unwrap() = changes.0;
                *locals.get_mut(contact.pair.1).unwrap() = changes.1;
            }

            (&mut bodies, &mut locals).par_join().for_each(|(body, _)|{
                body.force = Vec2::new(0.0, 0.0);
                body.torque = 0.0;
            });

            println!("Body count: {}", (&mut bodies, &mut locals).join().count());
        }
    }

    
}
