use super::{PhysicsComponent, LocalTransform};
use std::time::Duration;
use cgmath::Vector2;
use cgmath::prelude::*;
use super::Real;
use amethyst::ecs::{Entity, WriteStorage};


pub trait ForceGenerator: Send + Sync {
    fn update_force<'a>(&mut self, delta: Duration, physics: &mut WriteStorage<'a, PhysicsComponent>, locals: &mut WriteStorage<'a, LocalTransform>);
}

pub struct Gravity {
    pub target: Entity,
    pub value: Vector2<Real>
}

impl ForceGenerator for Gravity {
    fn update_force<'a>(&mut self, delta: Duration, physics: &mut WriteStorage<'a, PhysicsComponent>, locals: &mut WriteStorage<'a, LocalTransform>) {
        // TODO check for None properly
        let physic = physics.get_mut(self.target).unwrap();
        
        if !physic.has_finite_mass() { return }

        let mass = physic.mass();
        physic.add_force(self.value * mass);
    }   
}

pub struct Drag {
    pub target: Entity,
    pub k1: Real,
    pub k2: Real,
}

impl ForceGenerator for Drag {
    fn update_force<'a>(&mut self, delta: Duration, physics: &mut WriteStorage<'a, PhysicsComponent>, locals: &mut WriteStorage<'a, LocalTransform>) {
        // TODO check for None properly
        let physic = physics.get_mut(self.target).unwrap();

        let mut force = physic.velocity.clone();

        let mut drag_coeff = force.magnitude();
        drag_coeff = self.k1 * drag_coeff + self.k2 * drag_coeff * drag_coeff;

        force.normalize();
        force *= -drag_coeff;
        physic.add_force(force);
    }
}

pub struct Spring {
    pub target: Entity,
    pub other: Entity,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ForceGenerator for Spring {
    fn update_force<'a>(&mut self, delta: Duration, physics: &mut WriteStorage<'a, PhysicsComponent>, locals: &mut WriteStorage<'a, LocalTransform>) {
        let target_position = locals.get(self.target).unwrap().translation;
        let other_position = [300.0, 568.0];//locals.get(self.other).unwrap().translation;
        
        let mut force = Vector2::new(target_position[0] - other_position[0], target_position[1] - other_position[1]);

        let mut magnitude = force.magnitude();
        magnitude = (magnitude - self.rest_length).abs();
        magnitude *= self.spring_constant;

        force = force.normalize();
        force *= -magnitude;

        physics.get_mut(self.target).unwrap().add_force(force);
    }
}