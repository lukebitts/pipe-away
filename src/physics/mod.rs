use std::ops::{Deref, DerefMut};
use amethyst::ecs::{Component, VecStorage, WriteStorage, NullStorage};
use amethyst::ecs::components::LocalTransform;
use cgmath::{Vector2};
use std::sync::RwLock;
use std::cell::RefCell;
use std::time::Duration;
use std::mem::replace;

mod physics;
mod force_generator;

pub use self::physics::*;
pub use self::force_generator::*;

// Reëxport `ncollide` shapes
//pub use ncollide::shape::*;

// Also reëxport some `nphysics3d` objects
//pub use nphysics2d::object::{RigidBody, RigidBodyHandle, Sensor, SensorHandle};

pub enum PhysicsShape {
    Rectangle(Vector2<f32>),
    Circle(f32),
}

pub enum BodyType {
    Static,
    Dynamic,
}

pub type Real = f32;

pub struct PhysicsComponent {
    shape: PhysicsShape,
    body_type: BodyType,

    force_accum: Vector2<Real>,
    pub velocity: Vector2<Real>,
    pub acceleration: Vector2<Real>,
    pub damping: Real,
    inverse_mass: Real,
}

impl PhysicsComponent {
    pub fn new_dynamic(circle_radius: f32) -> Self {
        PhysicsComponent {
            shape: PhysicsShape::Circle(circle_radius),
            body_type: BodyType::Dynamic,

            force_accum: Vector2::new(0.0, 0.0),
            velocity: Vector2::new(20.0, 0.0),
            acceleration: Vector2::new(0.0, 0.0),
            damping: 0.99,
            inverse_mass: 0.5,
        }
    }

    pub fn new_static(half_extents: Vector2<f32>) -> Self {
        PhysicsComponent {
            shape: PhysicsShape::Circle(half_extents.x),
            body_type: BodyType::Static,

            force_accum: Vector2::new(0.0, 0.0),
            velocity: Vector2::new(0.0, 0.0),
            acceleration: Vector2::new(0.0, 0.0),
            damping: 0.95,
            inverse_mass: 0.0,
        }
    }

    pub fn set_mass(&mut self, val: Real) {
        self.set_inverse_mass(1.0/val);
    }

    pub fn set_inverse_mass(&mut self, val: Real) {
        self.inverse_mass = val;
    }

    pub fn inverse_mass(&self) -> Real {
        self.inverse_mass
    }

    pub fn clear_acumulator(&mut self) {
        self.force_accum = Vector2::new(0.0, 0.0);
    }

    pub fn add_force(&mut self, force: Vector2<f32>) {
        self.force_accum += force;
    }

    pub fn has_finite_mass(&self) -> bool {
        self.inverse_mass > 0.0 
    }

    pub fn mass(&self) -> Real {
        1.0 / self.inverse_mass
    }
}

impl Component for PhysicsComponent {
    type Storage = VecStorage<PhysicsComponent>;
}

#[derive(Default)]
pub struct PhysicsInit;

impl Component for PhysicsInit {
    type Storage = NullStorage<PhysicsInit>;
}