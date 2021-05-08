#include "rigidbody.h"
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include <algorithm>
#include <cmath>

Rigidbody_Particle::Rigidbody_Particle(Vec3 rel_pos, Rigidbody *owner) : rel_pos(rel_pos), mass(0.1f) {
    this->owner = owner;
    update();
}

void Rigidbody_Particle::update() {
    pos = Mat4::translate(owner->center_of_mass()) * owner->quaternion().to_mat() * rel_pos;
    velocity = owner->velocity() + cross(owner->angular_velocity(), owner->quaternion().rotate(rel_pos));
}


Rigidbody::Rigidbody(Scene_Object& obj, float particle_radius) : body(obj) {
    this->particle_radius = particle_radius;
    populate_particles();
    this->_center_of_mass = (body.bbox().min + body.bbox().max) / 2; // For now this will work
    pos_offset = obj.pose.pos - _center_of_mass;

    float Ixx = 0.f;
    float Iyy = 0.f;
    float Izz = 0.f;
    float Ixy = 0.f;
    float Ixz = 0.f;
    float Iyz = 0.f;

    M = 0.f;
    // Initialize inertia tensor
    for (Rigidbody_Particle& p : _particles) {
        M += p.mass;
        float x = p.rel_pos.x;
        float y = p.rel_pos.y;
        float z = p.rel_pos.z;
        float x2 = std::pow(x,2);
        float y2 = std::pow(y,2);
        float z2 = std::pow(z,2);

        Ixx += p.mass * (y2 + z2);
        Iyy += p.mass * (x2 + z2);
        Izz += p.mass * (x2 + y2);

        Ixy += - p.mass * x * y;
        Ixz += - p.mass * x * z;
        Iyz += - p.mass * y * z;
    }

    Mat4 _inertia_tensor = Mat4(Vec4{Ixx, Ixy, Ixz, 0.f},
                                Vec4{Ixy, Iyy, Iyz, 0.f},
                                Vec4{Ixz, Iyz, Izz, 0.f},
                                Vec4{0.f, 0.f, 0.f, 1.f});

    inv_inertia_tensor = _inertia_tensor.inverse();
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
    body.pose.pos = pos_offset + _center_of_mass;
    body.pose.euler = _quaternion.to_euler();
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

    v = P/M;
    w = inertia_tensor() * L;

    force = Vec3();
    torque = Vec3();
}


Mat4 Rigidbody::inertia_tensor() {
    // Multiply inverse inertia tensor by R(t)
    return _quaternion.to_mat() * inv_inertia_tensor * Mat4::transpose(_quaternion.to_mat());
}


void Rigidbody::update_position(float dt) {
    // Position
    _center_of_mass += v * dt;

    // Rotation - NOTE: This updates _quaternion to represent the
    // rotation _quaternion followed by dq
    float theta = (w * dt).norm();
    Quat dq = Quat(w.unit() * sinf(theta/2), cosf(theta/2));
    _quaternion = dq * _quaternion;
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

std::vector<Rigidbody_Particle>& Rigidbody::particles() {
    return _particles;
}
