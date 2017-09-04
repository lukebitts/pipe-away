mod collision;
mod system;

use cgmath::{dot, Rad, Vector2, Matrix2, Quaternion, Rotation3};
use amethyst::ecs::transform::LocalTransform;
use amethyst::ecs::{Component, VecStorage};

pub use self::system::PhysicsSystem;
pub use self::collision::{Manifold, ManifoldData};

pub type Real = f32;
pub type Vec2 = Vector2<Real>;
pub type Mat2 = Matrix2<Real>;

pub static PI : Real = ::std::f32::consts::PI;
pub static GRAVITY : [Real; 2] = [0.0, -500.0];
pub static EPSILON : Real = 0.0001;
pub static FRAME_TIME: Real = 1.0/60.0;

#[derive(Clone)]
pub struct PolygonShapeVertex {
    position: Vec2,
    normal: Vec2,
}

#[derive(Clone)]
pub enum Shape {
    Circle {
        radius: Real
    },
    Polygon {
        orientation: Mat2,
        vertices: Vec<PolygonShapeVertex>,
    },
}

impl Shape {
    pub fn rect(h: Vec2) -> Shape {
        Shape::Polygon {
            orientation: Mat2::new(1.0, 0.0, 0.0, 1.0),
            vertices: vec![
                PolygonShapeVertex { position: Vec2::new(-h.x, -h.y), normal: Vec2::new( 0.0, -1.0) },
                PolygonShapeVertex { position: Vec2::new( h.x, -h.y), normal: Vec2::new( 1.0,  0.0) },
                PolygonShapeVertex { position: Vec2::new( h.x,  h.y), normal: Vec2::new( 0.0,  1.0) },
                PolygonShapeVertex { position: Vec2::new(-h.x,  h.y), normal: Vec2::new(-1.0,  0.0) },
            ],
        }
    }
}

// Equal
fn float_cmp(a: Real, b: Real) -> bool {
    (a - b).abs() <= EPSILON
}

// Cross( const Vec2& a, const Vec2& b )
pub fn cross_vectors(a: Vec2, b: Vec2) -> Real {
    a.x * b.y - a.y * b.x
}

// Cross( real a, const Vec2& v )
pub fn cross_real_vector(a: Real, v: Vec2) -> Vec2 {
    Vec2::new( -a * v.y, a * v.x )
}

pub fn dist_sqr(a: Vec2, b: Vec2) -> Real {
    let c = a - b;
    return dot( c, c );
}

struct MassData {
    pub moment_inertia: Real,
    pub inv_inertia: Real,
    pub mass: Real,
    pub inv_mass: Real,
}

// Circle::ComputeMass
fn compute_mass(shape: &mut Shape, density: Real) -> MassData {
    match shape {
        &mut Shape::Circle{ radius } => {
            let m = PI * radius * radius * density;
            let i = m * radius * radius;

            MassData {
                moment_inertia: i,
                inv_inertia: if i != 0.0 { 1.0 / i } else { 0.0 },
                mass: m,
                inv_mass: if m != 0.0 { 1.0 / m } else { 0.0 },
            }
        },
        &mut Shape::Polygon { ref mut vertices, .. } => {
            let mut c = Vec2::new(0.0, 0.0); // centroid
            let mut area = 0.0;
            let mut i = 0.0;
            let k_inv3 = 1.0 / 3.0;

            //for(uint32 i1 = 0; i1 < m_vertexCount; ++i1)
            for edge in vertices.chunks(2) {
                // Triangle vertices, third vertex implied as (0, 0)
                //Vec2 p1( m_vertices[i1] );
                let p1 = edge[0].position;
                let p2 = if edge.len() == 2 { edge[1].position } else { edge[0].position };
                
                let d = cross_vectors(p1, p2);
                let triangle_area = 0.5 * d;

                area += triangle_area;

                //let unsafe_p1 = UnsafeVec2::new(p1.x, p1.y);
                //let unsafe_p2 = UnsafeVec2::new(p2.x, p2.y);
                //let mut unsafe_c = UnsafeVec2::new(c.x, c.y);
                //unsafe_c += triangle_area * k_inv3 * (unsafe_p1 + unsafe_p2);
                //c = Vec2::new(unsafe_c.x, unsafe_c.y);
                // Use area to weight the centroid average, not just vertex position
                c += triangle_area * k_inv3 * (p1 + p2);

                let intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
                let inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
                i += (0.25 * k_inv3 * d) * (intx2 + inty2);
            }

            c *= 1.0 / area;

            // Translate vertices to centroid (make the centroid (0, 0)
            // for the polygon in model space)
            for vertex in vertices {
               vertex.position -= c;
            }

            let m = density * area;
            let i = i * density;

            MassData {
                moment_inertia: m,
                inv_inertia: if m != 0.0 { 1.0 / m } else { 0.0 },
                mass: i,
                inv_mass: if i != 0.0 { 1.0 / i } else { 0.0 },
            }
        }
    }
}

#[derive(Clone)]
pub struct Body {
    pub shape: Shape,

    pub velocity: Vec2,
    pub angular_velocity: Real,
    pub torque: Real,
    pub force: Vec2,
    pub static_friction: Real,
    pub dynamic_friction: Real,
    pub restitution: Real,

    pub moment_inertia: Real,
    pub inv_inertia: Real,
    pub mass: Real,
    pub inv_mass: Real,
}

impl Body {
    // Body::Body, Shape::Initialize
    pub fn new(mut shape: Shape) -> Self {
        let mass_data = compute_mass(&mut shape, 1.0);

        Body {
            shape: shape,

            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            torque: 0.0,
            force: Vec2::new(0.0, 0.0),
            static_friction: 0.5,
            dynamic_friction: 0.3,
            restitution: 0.2,

            moment_inertia: mass_data.moment_inertia,
            inv_inertia: mass_data.inv_inertia,
            mass: mass_data.mass,
            inv_mass: mass_data.inv_mass,
        }
    }

    // Body::ApplyForce
    pub fn apply_force(&mut self, force: Vec2) {
        self.force += force;
    }

    // Body::ApplyImpulse
    pub fn apply_impulse(&mut self, impulse: Vec2, contact_vector: Vec2) {
        self.velocity += self.inv_mass * impulse;
        self.angular_velocity += self.inv_inertia * cross_vectors(contact_vector, impulse);
    }

    // Body::SetStatic
    pub fn set_static(&mut self) {
        self.moment_inertia = 0.0;
        self.inv_inertia = 0.0;
        self.mass = 0.0;
        self.inv_mass = 0.0;
    }

    // Body::SetOrient, Circle::SetOrient
    pub fn set_orient<T: Into<Rad<Real>>>(&mut self, local: &mut LocalTransform, radians: T) {
        let radians = radians.into();
        //self.orient = radians;

        //let q : Quaternion<f32> = local.rotation.into();
        //let roll = (2.0*(q.v.x*q.v.y + q.s*q.v.z)).atan2(q.s.powi(2) + q.v.x.powi(2) - q.v.y.powi(2) - q.v.z.powi(2));
        
        //rotation format: [w, x, y, z]
        local.rotation = Quaternion::from_angle_z(Rad(radians.0)).into();

        match self.shape {
            Shape::Circle{ .. } => (),
            Shape::Polygon{ ref mut orientation, .. } => {
                fn set(radians: Rad<Real>) -> Mat2
                {
                    let c = radians.0.cos();
                    let s = radians.0.sin();
                    //m00 = c; m01 = -s;
                    //m10 = s; m11 =  c;

                    Mat2::new(c, -s, s, c)
                }

                *orientation = set(-radians);
            }
        }
    }

    // IntegrateForces
    pub fn integrate_forces(&mut self, delta: Real) {
        if self.inv_mass == 0.0 {
            return
        }
        let safe_gravity = [GRAVITY[0], GRAVITY[1]];
        self.velocity += (self.force * self.inv_mass + Vec2::from(safe_gravity)) * (delta / 2.0);
        self.angular_velocity += self.torque * self.inv_inertia * (delta / 2.0);
    }

    // IntegrateVelocity
    pub fn integrate_velocity(&mut self, local: &mut LocalTransform, delta: Real) {
        if self.inv_mass == 0.0 {
            return
        }

        let translation = Vec2::new(local.translation[0], local.translation[1]) + self.velocity * delta;

        local.translation = [translation.x, translation.y, 0.0];
        let orient = {
            let q : Quaternion<f32> = local.rotation.into();
            let roll = (2.0*(q.v.x*q.v.y + q.s*q.v.z)).atan2(q.s.powi(2) + q.v.x.powi(2) - q.v.y.powi(2) - q.v.z.powi(2));

            Rad(roll + self.angular_velocity * delta)
        };
        self.set_orient(local, orient);

        self.integrate_forces(delta);
    }
}

impl Component for Body {
    type Storage = VecStorage<Body>;
}
