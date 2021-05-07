#include "rigidbody.h"
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include <algorithm>
#include <cmath>

Rigidbody_Particle::Rigidbody_Particle(Vec3 rel_pos, Rigidbody *owner) : rel_pos(rel_pos) {
    this->owner = owner;
    update();
}

void Rigidbody_Particle::update() {
    pos = Mat4::translate(owner->center_of_mass()) * owner->quaternion().to_mat() * rel_pos;
    velocity = owner->velocity() + cross(owner->angular_velocity(), rel_pos);
}


Rigidbody::Rigidbody(Scene_Object& obj, float particle_radius) : body(obj) {
    this->particle_radius = particle_radius;
    populate_particles();
    this->_center_of_mass = (body.bbox().min + body.bbox().max) / 2; // For now this will work
}



// For now only support rectangular meshes
void Rigidbody::populate_particles() {
    Vec3 min = body.bbox().min;
    Vec3 max = body.bbox().max;

    const float r = particle_radius;
    const Vec3 start = - (max - min) / 2;
    const Vec3 end = -start;

    for (float x = start.x + r; x <= end.x - r; x += 2*r) {
    for (float y = start.y + r; y <= end.y - r; y += 2*r) {
    for (float z = start.z + r; z <= end.z - r; z += 2*r) {
        _particles.push_back(Rigidbody_Particle(Vec3(x,y,z), this));
    }}}
}

void Rigidbody::render(const Mat4& view) {
    // TODO: Update body.pos as a function of center_of_mass and quaternion
    body.render(view);
}

Pose& Rigidbody::pose() {
    return body.pose;
}


void Rigidbody::accumulate_force(Vec3 partial_force) {
    force += partial_force;
}

void Rigidbody::accumulate_torque(Vec3 partial_torque) {
    torque += partial_torque;
}

void Rigidbody::apply_partial_updates(float dt) {

    P += force * dt;
    L += torque * dt;

    force = Vec3();
    torque = Vec3();
}

const BBox Rigidbody::bbox() {
    return body.bbox();
}

const Vec3 Rigidbody::center_of_mass() {
    return _center_of_mass;
}

const Quat Rigidbody::quaternion() {
    return _quaternion;
}

const Vec3 Rigidbody::angular_velocity() {
    return w;
}

const Vec3 Rigidbody::velocity() {
    return v;
}
