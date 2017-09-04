use super::{Vec2, Real};
//use super::scene::BodyIndex;
use super::Body;
use super::{GRAVITY, EPSILON};
use noisy_float::types::n32;
use amethyst::ecs::Entity;
use amethyst::ecs::transform::LocalTransform;

#[derive(Debug)]
pub struct ManifoldData {
    pub pair: (Entity, Entity),
    pub penetration: Real,
    pub normal: Vec2,
    pub contacts: Vec<Vec2>,
}

#[derive(Debug)]
pub struct Manifold {
    pub pair: (Entity, Entity),
    pub penetration: Real,
    pub normal: Vec2,
    pub contacts: Vec<Vec2>,
    pub e: Real,
    pub df: Real,
    pub sf: Real,
}

impl ManifoldData {
    // Manifold::Initialize
    pub fn initialize(&self, delta: Real, (body_a, local_a): (&Body, &LocalTransform), (body_b, local_b): (&Body, &LocalTransform)) -> Manifold {
        // Calculate average restitution
        let mut e = n32(body_a.restitution.raw().min(body_b.restitution.raw()));

        // Calculate static and dynamic friction
        let sf = n32(body_a.static_friction.raw().powi(2).sqrt());
        let df = n32(body_a.dynamic_friction.raw().powi(2).sqrt());

        for contact in &self.contacts {
            let ra = contact - Vec2::new(n32(local_a.translation[0]), n32(local_a.translation[1]));
            let rb = contact - Vec2::new(n32(local_b.translation[0]), n32(local_b.translation[1]));

            let rv = body_b.velocity + cross_real_vector(body_b.angular_velocity, rb) -
                     body_a.velocity - cross_real_vector(body_a.angular_velocity, ra);

            if len_sqr(rv) < len_sqr(Vec2::new(n32(GRAVITY[0]) * delta, n32(GRAVITY[1]) * delta)) + EPSILON {
                e = n32(0.0);
            }
        }
        
        Manifold {
            pair: self.pair,
            penetration: self.penetration,
            normal: self.normal,
            contacts: self.contacts.clone(),
            e: e,
            df: df,
            sf: sf
        }
    }
}

// LenSqr
fn len_sqr(v: Vec2) -> Real {
    n32(v.x.raw().powi(2) + v.y.raw().powi(2))
}

// CircletoCircle
pub fn circle_circle(
        (i_a, radius_a, body_a, local_a): (Entity, Real, &Body, &LocalTransform), 
        (i_b, radius_b, body_b, local_b): (Entity, Real, &Body, &LocalTransform)) 
    -> Option<ManifoldData> {
    // Calculate translational vector, which is normal

    let pos_a = Vec2::new(n32(local_a.translation[0]), n32(local_a.translation[1]));
    let pos_b = Vec2::new(n32(local_b.translation[0]), n32(local_b.translation[1]));

    let normal = pos_b - pos_a;
    let dist_sqr = len_sqr(normal);
    let radius = radius_a + radius_b;

    if dist_sqr >= n32(radius.raw().powi(2)) {
        return None
    }
    let distance = n32(dist_sqr.raw().sqrt());

    if distance == 0.0 {
        Some(ManifoldData {
            pair: (i_a, i_b),
            penetration: radius_a,
            normal: Vec2::new(n32(1.0), n32(0.0)),
            contacts: vec![pos_a]
        })
    } else {
        let normal_over_distance = normal / distance;
        Some(ManifoldData {
            pair: (i_a, i_b),
            penetration: radius - distance,
            normal: normal_over_distance,
            contacts: vec![normal_over_distance * radius_a + pos_a],
        })
    }
}