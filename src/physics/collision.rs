use super::{Body, Real, Shape, LocalTransform, cross_vectors, FRAME_TIME, GRAVITY};
use amethyst::ecs::Entity;
use cgmath::{Vector2, InnerSpace, dot};

static EPSILON: Real = 0.0001;

fn real_cmp(a: Real, b: Real) -> bool {
    (a - b).abs() <= EPSILON
}

fn cross_real_vec(a: Real, v: Vector2<Real>) -> Vector2<Real> {
    Vector2::new(-a * v.y, a * v.x)
}

pub struct ManifoldBuilder {
    pub pair: (Entity, Entity),
    pub penetration: Real,
    pub normal: Vector2<Real>,
    pub contacts: Vec<Vector2<Real>>,
}

impl ManifoldBuilder {
    pub fn build(
        &self,
        (physic1, local1): (&Body, &LocalTransform),
        (physic2, local2): (&Body, &LocalTransform),
    ) -> Manifold {
        let mut e = physic1.restitution.min(physic2.restitution);
        let sf = (physic1.static_friction * physic1.static_friction).sqrt();
        let df = (physic1.dynamic_friction * physic1.dynamic_friction).sqrt();

        for contact in &self.contacts {
            let ra = contact -
                Vector2::new(local1.translation[0] as Real, local1.translation[1] as Real);
            let rb = contact -
                Vector2::new(local2.translation[0] as Real, local2.translation[1] as Real);

            let rv = physic2.velocity + cross_real_vec(physic2.angular_velocity, rb) -
                physic1.velocity -
                cross_real_vec(physic1.angular_velocity, ra);

            if len_sqr(rv) < len_sqr(FRAME_TIME * Vector2::new(GRAVITY[0], GRAVITY[1])) + EPSILON {
                e = 0.0;
            }
        }

        Manifold {
            pair: self.pair,
            penetration: self.penetration,
            normal: self.normal,
            contacts: self.contacts.clone(),
            e: e,
            df: df,
            sf: sf,
        }
    }
}

#[derive(Debug)]
pub struct Manifold {
    pub pair: (Entity, Entity),
    pub penetration: Real,
    pub normal: Vector2<Real>,
    pub contacts: Vec<Vector2<Real>>,
    pub e: Real, // Mixed restitution
    pub df: Real, // Mixed dynamic friction
    pub sf: Real, // Mixed static friction*/
}

pub fn positional_correct(
    manifold: &Manifold,
    a: (&Body, &LocalTransform),
    b: (&Body, &LocalTransform),
) -> (Vector2<Real>, Vector2<Real>) {
    let k_slop = 0.05;
    let percent = 0.4;
    let correction = ((manifold.penetration - k_slop).max(0.0) / (a.0.inv_mass + b.0.inv_mass)) *
        manifold.normal *
        percent;

    let position1 = Vector2::new(a.1.translation[0] as Real, a.1.translation[1] as Real) -
        correction * a.0.inv_mass;
    let position2 = Vector2::new(b.1.translation[0] as Real, b.1.translation[1] as Real) +
        correction * b.0.inv_mass;

    (position1, position2)
}

fn infinite_mass_correction(a: &mut Body, b: &mut Body) {
    a.velocity = [0.0, 0.0].into();
    b.velocity = [0.0, 0.0].into();
}

pub fn calculate_impulse(
    manifold: &Manifold,
    (mut physic1, local1): (Body, &LocalTransform),
    (mut physic2, local2): (Body, &LocalTransform),
) -> (Body, Body) {
    // Early out and positional correct if both objects have infinite mass
    if real_cmp(physic1.inv_mass + physic2.inv_mass, 0.0) {
        infinite_mass_correction(&mut physic1, &mut physic2);
        return (physic1, physic2);
    }

    let contact_count = manifold.contacts.len();
    for contact in &manifold.contacts {
        let ra = contact -
            Vector2::new(local1.translation[0] as Real, local1.translation[1] as Real);
        let rb = contact -
            Vector2::new(local2.translation[0] as Real, local2.translation[1] as Real);
        let mut rv = physic2.velocity + cross_real_vec(physic2.angular_velocity, rb) -
            physic1.velocity - cross_real_vec(physic1.angular_velocity, ra);

        let contact_vel = dot(rv, manifold.normal);

        if contact_vel > 0.0 {
            return (physic1, physic2);
        }

        let ra_cross_n = cross_vectors(ra, manifold.normal);
        let rb_cross_n = cross_vectors(rb, manifold.normal);
        let inv_mass_sum = physic1.inv_mass + physic2.inv_mass +
            ra_cross_n.sqrt() * physic1.inv_inertia +
            rb_cross_n.sqrt() * physic2.inv_inertia;

        let mut j = -(1.0 + manifold.e) * contact_vel;
        j /= inv_mass_sum;
        j /= contact_count as Real;

        let impulse = manifold.normal * j;

        physic1.apply_impulse(-impulse, ra);
        physic2.apply_impulse(impulse, rb);

        rv = physic2.velocity + cross_real_vec(physic2.angular_velocity, rb) - physic1.velocity -
            cross_real_vec(physic1.angular_velocity, ra);

        let mut t = rv - (manifold.normal * dot(rv, manifold.normal));
        t = t.normalize();

        let mut jt = -dot(rv, t);
        jt /= inv_mass_sum;
        jt /= contact_count as Real;

        if real_cmp(jt, 0.0) {
            return (physic1, physic2);
        }

        let tangent_impulse = if jt.abs() < j * manifold.sf {
            t * jt
        } else {
            t * -j * manifold.df
        };

        physic1.apply_impulse(-tangent_impulse, ra);
        physic2.apply_impulse(tangent_impulse, rb);
    }

    (physic1, physic2)
}

pub fn collides(
    (e1, physic1, local1): (Entity, &Body, &LocalTransform),
    (e2, physic2, local2): (Entity, &Body, &LocalTransform),
) -> Option<ManifoldBuilder> {

    /*let mut e = physic1.restitution.min(physic2.restitution);
    let sf = (physic1.static_friction * physic1.static_friction).sqrt();
    let df = (physic1.dynamic_friction * physic1.dynamic_friction).sqrt();*/

    let data = match (&physic1.shape, &physic2.shape) {
        (&Shape::Circle { radius: r1 }, &Shape::Circle { radius: r2 }) => {
            if let Some(result) = circle_circle(
                r1,
                Vector2::new(
                    local1.translation[0] as Real,
                    local1.translation[1] as Real,
                ),
                r2,
                Vector2::new(
                    local2.translation[0] as Real,
                    local2.translation[1] as Real,
                ),
            )
            {
                result
            } else {
                return None;
            }
        }
        (&Shape::Circle { .. }, &Shape::Polygon { .. }) => return None,
        (&Shape::Polygon { .. }, &Shape::Circle { .. }) => return None,
        (&Shape::Polygon { .. }, &Shape::Polygon { .. }) => return None,
    };

    /*for contact in &data.contacts {
        let ra = contact - Vector2::new(local1.translation[0], local1.translation[1]);
        let rb = contact - Vector2::new(local2.translation[0], local2.translation[1]);

        let rv = physic2.velocity + cross_real_vec(physic2.angular_velocity, rb) -
            physic1.velocity - cross_real_vec(physic1.angular_velocity, ra);

        if len_sqr(rv) < len_sqr(FRAME_TIME * Vector2::new(GRAVITY[0], GRAVITY[1])) + EPSILON {
            e = 0.0;
        }
    }*/

    Some(ManifoldBuilder {
        pair: (e1, e2),
        penetration: data.penetration,
        normal: data.normal,
        contacts: data.contacts,
    })
}

fn len_sqr(v: Vector2<Real>) -> Real {
    v.x * v.x + v.y * v.y
}

struct CollisionData {
    pub contacts: Vec<Vector2<Real>>,
    pub penetration: Real,
    pub normal: Vector2<Real>,
}

fn circle_circle(
    radius1: Real,
    position1: Vector2<Real>,
    radius2: Real,
    position2: Vector2<Real>,
) -> Option<CollisionData> {

    let normal = position2 - position1;
    let dist_sqr = len_sqr(normal);
    let radius = radius1 + radius2;

    if dist_sqr >= radius * radius {
        return None;
    }

    let distance = dist_sqr.sqrt();

    if distance == 0.0 {
        Some(CollisionData {
            penetration: radius1,
            normal: Vector2::new(1.0, 0.0),
            contacts: vec![position1],
        })
    } else {
        let normal = normal / distance;
        Some(CollisionData {
            penetration: radius - distance,
            normal: normal,
            contacts: vec![normal * radius1 + position1],
        })
    }
}

/*
  real distance = std::sqrt( dist_sqr );

  m->contact_count = 1;

  if(distance == 0.0f)
  {
    m->penetration = A->radius;
    m->normal = Vec2( 1, 0 );
    m->contacts [0] = a->position;
  }
  else
  {
    m->penetration = radius - distance;
    m->normal = normal / distance; // Faster than using Normalized since we already performed sqrt
    m->contacts[0] = m->normal * A->radius + a->position;
  }
*/
