use amethyst::ecs::{Component, VecStorage, NullStorage};
use amethyst::ecs::transform::LocalTransform;
use cgmath::{Vector2, Vector3, Matrix2, Rad, Quaternion, Rotation3};
use std::f32::consts::PI;

mod system;
mod collision;

pub use self::system::*;

pub type Real = f32;

pub static FRAME_TIME: Real = 1.0 / 60.0;
pub static GRAVITY: [Real; 2] = [0.0, -500.0];

#[derive(Clone)]
pub struct ShapeVertex {
    position: Vector2<Real>,
    normal: Vector2<Real>,
}

#[derive(Clone)]
pub enum Shape {
    #[allow(unused)]
    Polygon {
        orientation: Matrix2<Real>,
        vertices: Vec<ShapeVertex>,
    },
    Circle { radius: Real },
}

fn cross_vectors(a: Vector2<Real>, b: Vector2<Real>) -> Real {
    a.x * b.y - a.y * b.x
}

struct MassData {
    pub moment_inertia: Real,
    pub inv_inertia: Real,
    pub mass: Real,
    pub inv_mass: Real,
}

fn compute_mass(shape: &mut Shape, density: Real) -> MassData {
    match *shape {
        Shape::Circle { radius } => {
            let m = PI * radius * radius * density;
            let i = m * radius * radius;

            MassData {
                moment_inertia: m,
                inv_inertia: if m != 0.0 { 1.0 / m } else { 0.0 },
                mass: i,
                inv_mass: if i != 0.0 { 1.0 / i } else { 0.0 },
            }
        }
        Shape::Polygon { .. } => unimplemented!(),
    }
}

#[derive(Clone, Eq, PartialEq)]
pub enum BodyType {
    Static,
    Dynamic,
}

#[derive(Clone)]
pub struct Body {
    shape: Shape,

    velocity: Vector2<Real>,
    angular_velocity: Real,
    torque: Real,
    force: Vector2<Real>,
    static_friction: Real,
    dynamic_friction: Real,
    restitution: Real,

    moment_inertia: Real,
    inv_inertia: Real,
    mass: Real,
    inv_mass: Real,
}

impl Body {
    pub fn new(mut shape: Shape, body_type: BodyType) -> Self {
        let mass_data = compute_mass(&mut shape, 1.0);

        let mut body = Body {
            shape: shape,

            velocity: Vector2::new(0.0, 0.0),
            angular_velocity: 0.0,
            torque: 0.0,
            force: Vector2::new(0.0, 0.0),
            static_friction: 0.5,
            dynamic_friction: 0.3,
            restitution: 0.2,

            moment_inertia: mass_data.moment_inertia,
            inv_inertia: mass_data.inv_inertia,
            mass: mass_data.mass,
            inv_mass: mass_data.inv_mass,
        };

        if body_type == BodyType::Static {
            body.set_static();
        }

        body
    }

    /*pub fn apply_force(&mut self, f: Vector2<Real>) {
        self.force += f;
    }*/

    pub fn apply_impulse(&mut self, impulse: Vector2<Real>, contact_vector: Vector2<Real>) {
        self.velocity += self.inv_mass * impulse;
        self.angular_velocity += self.inv_inertia * cross_vectors(contact_vector, impulse);
    }

    pub fn set_static(&mut self) {
        self.moment_inertia = 0.0;
        self.inv_inertia = 0.0;
        self.mass = 0.0;
        self.inv_mass = 0.0;
    }

    /*pub fn set_orient(&mut self, radians: Rad<Real>) {
        self.orient = radians;
        match self.shape {
            Shape::Circle { .. } => (),
            Shape::Polygon { .. } => unimplemented!(),
        }
    }*/

    fn integrate_forces(&mut self, delta: Real) {
        if self.inv_mass == 0.0 {
            return;
        }

        let gravity = Vector2::new(GRAVITY[0], GRAVITY[1]);

        self.velocity += (self.force * self.inv_mass + gravity) * (delta / 2.0);
        self.angular_velocity += self.torque * self.inv_inertia * (delta / 2.0);
    }

    fn integrate_velocity(&mut self, delta: Real, local: &mut LocalTransform) {
        if self.inv_mass == 0.0 {
            return;
        }

        let speed = self.velocity * delta;
        let position = Vector3::new(
            local.translation[0] + speed.x as f32,
            local.translation[1] + speed.y as f32,
            0.0,
        );

        local.translation = position.into();

        //self.orient += Rad(self.angular_velocity * delta);
        //let o = self.orient;
        //self.set_orient(o);
        let q: Quaternion<_> = local.rotation.into();
        let mut roll = Rad((2.0 * (q.v.x * q.v.y + q.s * q.v.z)).atan2(
            q.s * q.s + q.v.x * q.v.x - q.v.y * q.v.y -
                q.v.z * q.v.z,
        ));
        roll += Rad(self.angular_velocity * delta);

        local.rotation = Quaternion::from_angle_z(roll).into();

        self.integrate_forces(delta);
    }
}

impl Component for Body {
    type Storage = VecStorage<Body>;
}

#[derive(Default)]
pub struct PhysicsInit;

impl Component for PhysicsInit {
    type Storage = NullStorage<PhysicsInit>;
}
