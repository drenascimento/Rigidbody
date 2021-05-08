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
    size_t num_particles = 0;
    /* 1st step: Compute particle values */
    // Position, velocity, relative position
    for_rigidbody([&bounds, &num_particles](Rigidbody& body){
        bounds.enclose(body.bbox());
        for (Rigidbody_Particle& p : body.particles()) {
            num_particles++;
            p.update();
        }
    });

    /* 2nd step: Generate grid, and place particles on grid */
    size_t width  = ceilf((bounds.max.x - bounds.min.x) / (2.f * particle_radius));
    size_t height = ceilf((bounds.max.y - bounds.min.y) / (2.f * particle_radius));
    size_t depth  = ceilf((bounds.max.z - bounds.min.z) / (2.f * particle_radius));

    std::vector<uint32_t> grid;
    grid.resize(width * height * depth * max_particles_in_voxel, 0);

    /* NOTE: Particles in grid are 1-indexed, 0 index indicates grid is empty at that location. */
    std::vector<Rigidbody_Particle*> particles;
    particles.push_back({});

    {
        std::vector<Rigidbody> next;
        next.reserve(bodies.size());

        size_t particle_index = 0;

        for (size_t i=0; i < bodies.size(); i++) {
            Rigidbody& r = bodies[i];

            for (Rigidbody_Particle& p : r.particles()) {
                particle_index++;
                particles.push_back(&p);

                size_t x, y, z;

                x = ceilf((p.pos.x - bounds.min.x) / (2.f * particle_radius));
                y = ceilf((p.pos.y - bounds.min.y) / (2.f * particle_radius));
                z = ceilf((p.pos.z - bounds.min.z) / (2.f * particle_radius));

                size_t index = (x * height * depth + y * depth + z) * max_particles_in_voxel;
                size_t offset = 0;

                while (grid[index + offset] != 0) offset++;

                // In the *very rare* case of overflow, just ignore additional particles
                if (offset < max_particles_in_voxel) {
                    grid[index + offset] = particle_index;
                }
            }
            next.emplace_back(std::move(r));
        }
        bodies = std::move(next);
    }

    /* 3rd step: Get collisions */
    for (size_t x = 0; x < width; x++) {
    for (size_t y = 0; y < height; y++) {
    for (size_t z = 0; z < depth; z++) {

        size_t index = (x * height * depth + y * depth + z) * max_particles_in_voxel;

        for (size_t offset = 0; offset < max_particles_in_voxel && grid[index + offset] != 0; offset++) {
            // Get all points in a block around (x,y,z)
            for (size_t xp = x-1; xp <= x+1; xp++) {
            for (size_t yp = y-1; yp <= y+1; yp++) {
            for (size_t zp = z-1; zp <= z+1; zp++) {
                if (xp < 0 || xp >= width || yp < 0 || yp >= height || zp < 0 || zp >= depth) continue;

                size_t neighbor_index = (xp * height * depth + yp * depth + zp) * max_particles_in_voxel;

                for (size_t neighbor_offset = 0; neighbor_offset < max_particles_in_voxel && grid[neighbor_index + neighbor_offset] != 0; neighbor_offset++) {

                    /* Found a pair of colliding particles! Update my body only. */

                    Rigidbody_Particle *my_particle = particles.at(grid[index + offset]);
                    Rigidbody_Particle *neighbor_particle = particles.at(grid[neighbor_index + neighbor_offset]);

                    Vec3 rel_pos_other = my_particle->pos - neighbor_particle->pos;
                    Vec3 rel_vel_other = my_particle->velocity - neighbor_particle->velocity;
                    Vec3 rel_tangential_vel = rel_vel_other - (dot(rel_vel_other,rel_pos_other.unit()) * rel_pos_other.unit());

                    Vec3 Fis = -spring_coefficient * (2.f * particle_radius - rel_pos_other.norm()) * rel_pos_other.unit();
                    Vec3 Fid = damping_coefficient * rel_vel_other;
                    Vec3 Fit = shear_coefficient * rel_tangential_vel;

                    // Apply update to my body
                    Vec3 rel_pos_to_center = my_particle->owner->quaternion().rotate(my_particle->rel_pos);
                    my_particle->owner->accumulate_force(Fis + Fid + Fit);
                    my_particle->owner->accumulate_torque(cross(rel_pos_to_center, Fis + Fid + Fit));
                }
            }}}
        }
    }}}


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
    std::cout << "set_time called\n";
}
