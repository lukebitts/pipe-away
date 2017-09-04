use super::{collision, float_cmp, cross_vectors, cross_real_vector, Body, Shape, Manifold, Vec2, UnsafeVec2, FRAME_TIME};
use super::super::DurationUtils;
use amethyst::ecs::{System, Fetch, Join, ParJoin, JoinParIter, WriteStorage, ReadStorage, Entities};
use amethyst::ecs::transform::{LocalTransform, Init};
use amethyst::timing::Time;
use rayon::iter::ParallelIterator;
use rayon::prelude::*;
use std::sync::Mutex;
use std::time::Duration;
use noisy_float::types::{n32, N32};
use cgmath::{dot, InnerSpace};

pub struct PhysicsSystem {
    acumulator: N32,
}

impl PhysicsSystem {
    pub fn new() -> Self {
        PhysicsSystem {
            acumulator: n32(0.0)
        }
    }

    // Manifold::ApplyImpulse
    fn apply_impulse(&mut self, m: &Manifold, 
        (mut body_a, local_a): (Body, &LocalTransform), 
        (mut body_b, local_b): (Body, &LocalTransform)
    ) -> (Body, Body) {
        let (i_a, i_b) = m.pair;
        //let (body_a, body_b) = self.get_two_mut(i_a, i_b);

        if float_cmp(body_a.inv_mass + body_b.inv_mass, n32(0.0)) {
            //InfiniteMassCorrection
            body_a.velocity = Vec2::new(n32(0.0), n32(0.0));
            body_b.velocity = Vec2::new(n32(0.0), n32(0.0));

            return (body_a, body_b);
        }

        for contact in &m.contacts {
            let ra = contact - Vec2::new(n32(local_a.translation[0]), n32(local_a.translation[1]));
            let rb = contact - Vec2::new(n32(local_b.translation[0]), n32(local_b.translation[1]));
            
            let (inv_mass_sum, j, impulse) = {
                let rv = body_b.velocity + cross_real_vector(body_b.angular_velocity, rb) -
                         body_a.velocity - cross_real_vector(body_a.angular_velocity, ra);

                let unsafe_rv = UnsafeVec2::new(rv.x.raw(), rv.y.raw());
                let unsafe_normal = UnsafeVec2::new(m.normal.x.raw(), m.normal.y.raw());

                let contact_vel = n32(dot(unsafe_rv, unsafe_normal));
                if contact_vel > 0.0 {
                    return (body_a, body_b)
                }

                let ra_cross_n = cross_vectors(ra, m.normal);
                let rb_cross_n = cross_vectors(rb, m.normal);
                
                let inv_mass_sum = 
                    body_a.inv_mass + body_b.inv_mass +
                    n32(ra_cross_n.raw().powi(2)) * body_a.inv_inertia +
                    n32(rb_cross_n.raw().powi(2)) * body_b.inv_inertia;
                
                let j = -(n32(1.0) + m.e) * contact_vel / inv_mass_sum / n32(m.contacts.len() as f32);
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
                let unsafe_rv = UnsafeVec2::new(rv.x.raw(), rv.y.raw());
                let unsafe_normal = UnsafeVec2::new(m.normal.x.raw(), m.normal.y.raw());
                
                let mut unsafe_t = unsafe_rv - (unsafe_normal * dot(unsafe_rv, unsafe_normal));
                let len_t = unsafe_t.magnitude();
                //if len_t is too small we can't normalize the vector, since it would divide by zero
                if !float_cmp(n32(len_t), n32(0.0)) {
                    unsafe_t = unsafe_t.normalize();
                }
                let t = Vec2::new(n32(unsafe_t.x), n32(unsafe_t.y));

                let unsafe_rv_dot_unsafe_t = dot(unsafe_rv, unsafe_t);
                let jt = -n32(unsafe_rv_dot_unsafe_t) / inv_mass_sum / n32(m.contacts.len() as f32);

                if float_cmp(jt, n32(0.0)) {
                    return (body_a, body_b)
                }

                if n32(jt.raw().abs()) < j * m.sf {
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

        let unsafe_normal = UnsafeVec2::new(m.normal.x.raw(), m.normal.y.raw());
        let unsafe_correction = ((m.penetration - k_slop).raw().max(0.0) / (body_a.inv_mass.raw() + body_b.inv_mass.raw())) * unsafe_normal * percent;
        let correction = Vec2::new(n32(unsafe_correction.x), n32(unsafe_correction.y));

        let pos_a = Vec2::new(n32(local_a.translation[0]), n32(local_a.translation[1])) - correction * body_a.inv_mass;
        let pos_b = Vec2::new(n32(local_b.translation[0]), n32(local_b.translation[1])) + correction * body_b.inv_mass;

        local_a.translation = [pos_a.x.raw(), pos_a.y.raw(), 0.0];
        local_b.translation = [pos_b.x.raw(), pos_b.y.raw(), 0.0];

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
        let delta = n32(time.delta_time.as_delta32());
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
                                    (i_a, r1, body_a, local_a),
                                    (i_b, r2, body_b, local_b)
                                ) {
                                    ret.push(manifold_data);
                                }
                            }
                        }
                    }
                    let mut v = data.lock().unwrap();
                    v.extend(ret);
                });

                data.into_inner().unwrap()
            };

            (&mut bodies, &locals).par_join().for_each(|(body, _)|{
                body.integrate_forces(n32(FRAME_TIME));
            });
            
            let contacts : Vec<_> = contact_data.par_iter().map(|data|{
                data.initialize(
                    n32(FRAME_TIME), 
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
                body.integrate_velocity(local, n32(FRAME_TIME));
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
                body.force = Vec2::new(n32(0.0), n32(0.0));
                body.torque = n32(0.0);
            });

            println!("Body count: {}", (&mut bodies, &mut locals).join().count());
        }
    }

    
}
