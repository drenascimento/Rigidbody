#include "../lib/mathlib.h"
#include "../platform/gl.h"
#include "renderer.h"
#include "scene_rigidbody.h"

#include <algorithm>
#include <cmath>
#include <iostream>

Scene_Rigidbody::Scene_Rigidbody(Scene_ID id) {
    _id = id;
    snprintf(opt.name, max_name_len, "Rigidbody group %d", id);
    // TODO: Maybe compute bbox if needed
    _bbox = BBox();
    opt.enabled = false;
}

void Scene_Rigidbody::clear() {
    // TODO: Clear the rigid bodies vector
}

void Scene_Rigidbody::addRigidbody(Rigidbody&& body) {
    bodies.push_back(std::move(body));
}


BBox Scene_Rigidbody::bbox() const {
    return _bbox;
};


void Scene_Rigidbody::for_rigidbody(std::function<void(Rigidbody&)> func) {
    std::vector<Rigidbody> next;
    next.reserve(bodies.size());
    for (size_t i=0; i < bodies.size(); i++) {
        Rigidbody& r = bodies[i];
        func(r);
        next.emplace_back(std::move(r));
    }
    bodies = std::move(next);
}

void Scene_Rigidbody::step(float dt) {
    BBox bounds;
    /* 1st step: Compute particle values */
    // Position, velocity, relative position
    for_rigidbody([&bounds](Rigidbody& body){
        bounds.enclose(body.bbox());
        for (Rigidbody_Particle& p : body.particles()) {
            p.update();
        }
    });

    /* 2nd step: Generate grid, and place particles on grid */
    size_t width  = ceilf((bounds.max.x - bounds.min.x) / (2.f * particle_radius));
    size_t height = ceilf((bounds.max.y - bounds.min.y) / (2.f * particle_radius));
    size_t depth  = ceilf((bounds.max.z - bounds.min.z) / (2.f * particle_radius));
    std::vector<uint32_t> grid;
    // max particles in voxel is a buffer for when multiple particles fall in the same voxel.
    grid.resize(width * height * depth * max_particles_in_voxel);

    std::vector<Rigidbody> next;
    next.reserve(bodies.size());
    for (size_t i=0; i < bodies.size(); i++) {
        Rigidbody& r = bodies[i];

        for (Rigidbody_Particle& p : r.particles()) {
            size_t x, y, z;

            x = ceilf((p.pos.x - bounds.min.x) / (2.f * particle_radius));
            y = ceilf((p.pos.y - bounds.min.y) / (2.f * particle_radius));
            z = ceilf((p.pos.z - bounds.min.z) / (2.f * particle_radius));

            grid[(x * height * depth + y * depth + z) * max_particles_in_voxel] = i;
        }

        next.emplace_back(std::move(r));
    }
    bodies = std::move(next);

    /* 3rd step: Get collisions */

    /* 4th step: Compute change in momenta and apply it to rigid bodies */

    /* 5th step: Compute new center_of_mass position and quaternion */

    /* 6th step: Convert center_of_mass and quaternion and apply to body.pose() */

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
    std::cout << "set_time called\n";
}
