#include "../lib/mathlib.h"
#include "../platform/gl.h"
#include "renderer.h"
#include "scene_rigidbody.h"
#include "kernels.h"

#include <algorithm>
#include <cmath>
#include <iostream>

Scene_Rigidbody::Scene_Rigidbody(Scene_ID id) {
    _id = id;
    snprintf(opt.name, max_name_len, "Rigidbody group %d", id);
    _bbox = BBox();
    opt.enabled = false;
}

void Scene_Rigidbody::clear() {
    // TODO: ?
}

void Scene_Rigidbody::addRigidbody(Rigidbody&& body) {
    num_bodies++;
    bodies.push_back(std::move(body));
}


BBox Scene_Rigidbody::bbox() const {
    return _bbox;
};


void Scene_Rigidbody::for_rigidbody(std::function<void(Rigidbody&)> func) {
    for (size_t i=0; i < bodies.size(); i++) {
        Rigidbody& r = bodies[i];
        func(r);
    }
}

void Scene_Rigidbody::step(float dt) {
    for (float t = 0; t < dt; t += delta_t) {
        partial_step(delta_t);
    }
}

void Scene_Rigidbody::partial_step(float dt) {
    BBox bounds;

    for_rigidbody([&bounds](Rigidbody& body){
        bounds.enclose(body.bbox());
    });

    /* 2nd and 3rd steps: Use kernel for collision detection and reaction */
    int N = bodies.size();
    float *center_of_mass   = (float*)malloc(N * 3 * sizeof(float));
    float *quaternion       = (float*)malloc(N * 4 * sizeof(float));
    float *velocity         = (float*)malloc(N * 3 * sizeof(float));
    float *angular_velocity = (float*)malloc(N * 3 * sizeof(float));
    float *box_min          = (float*)malloc(N * 3 * sizeof(float));
    float *box_max          = (float*)malloc(N * 3 * sizeof(float));
    float *box_all          = (float*)malloc(6 * sizeof(float));

    float *out_force        = (float*)malloc(N * 3 * sizeof(float));
    float *out_torque       = (float*)malloc(N * 3 * sizeof(float));

    for (size_t i=0; i < bodies.size(); i++) {
        Rigidbody& r = bodies[i];

        center_of_mass[i * 3    ] = r.center_of_mass.x;
        center_of_mass[i * 3 + 1] = r.center_of_mass.y;
        center_of_mass[i * 3 + 2] = r.center_of_mass.z;

        quaternion[i * 4    ] = r.quaternion.x;
        quaternion[i * 4 + 1] = r.quaternion.y;
        quaternion[i * 4 + 2] = r.quaternion.z;
        quaternion[i * 4 + 3] = r.quaternion.w;

        velocity[i * 3    ] = r.velocity().x;
        velocity[i * 3 + 1] = r.velocity().y;
        velocity[i * 3 + 2] = r.velocity().z;

        angular_velocity[i * 3    ] = r.angular_velocity().x;
        angular_velocity[i * 3 + 1] = r.angular_velocity().y;
        angular_velocity[i * 3 + 2] = r.angular_velocity().z;

        box_min[i * 3    ] = r.bbox().min.x;
        box_min[i * 3 + 1] = r.bbox().min.y;
        box_min[i * 3 + 2] = r.bbox().min.z;

        box_max[i * 3    ] = r.bbox().max.x;
        box_max[i * 3 + 1] = r.bbox().max.y;
        box_max[i * 3 + 2] = r.bbox().max.z;

        box_all[0] = bounds.min.x;
        box_all[1] = bounds.min.y;
        box_all[2] = bounds.min.z;

        box_all[3] = bounds.max.x;
        box_all[4] = bounds.max.y;
        box_all[5] = bounds.max.z;
    }

    Kernels::update(N, particle_radius, center_of_mass, quaternion, velocity,
                    angular_velocity, box_min, box_max, box_all, out_force, out_torque);

    for (size_t i=0; i < bodies.size(); i++) {
        Rigidbody& r = bodies[i];

        Vec3 force;
        force.x = out_force[i * 3    ];
        force.y = out_force[i * 3 + 1];
        force.z = out_force[i * 3 + 2];

        //std::cout << "Force update: " << force << "\n";

        Vec3 torque;
        torque.x = out_torque[i * 3    ];
        torque.y = out_torque[i * 3 + 1];
        torque.z = out_torque[i * 3 + 2];

        //std::cout << "Torque update: " << torque << "\n";

        r.accumulate_force(force * r.mass());
        r.accumulate_torque(torque * r.mass());
    }

    /* 3.5th step: Apply gravity */
    for_rigidbody([](Rigidbody& body){
        body.accumulate_force(Vec3{0.f, -9.8f, 0.f} * body.mass());
    });

    /* 4th step: Compute change in momenta and apply it to rigid bodies */
    for_rigidbody([dt](Rigidbody& body){
        body.apply_partial_updates(dt);
    });

    /* 5th step: Compute new center_of_mass position and quaternion */
    for_rigidbody([dt](Rigidbody& body){
        body.update_position(dt);
    });

}

void Scene_Rigidbody::render(const Mat4& view, bool depth_only, bool posed, bool particles_only) {
    for_rigidbody([&view](Rigidbody& body){
        body.render(view);
    });
}

Scene_ID Scene_Rigidbody::id() const {
    return _id;
}

void Scene_Rigidbody::set_time(float time) {
}
