
#include "../geometry/util.h"
#include "../scene/renderer.h"

#include "manager.h"
#include "simulate.h"

#include <iostream>

namespace Gui {

const char* Solid_Type_Names[(int)Solid_Type::count] = {"Sphere", "Cube", "Cylinder", "Torus",
                                                        "Custom"};

Simulate::Simulate() : thread_pool(std::thread::hardware_concurrency()) {
    last_update = SDL_GetPerformanceCounter();
}

Simulate::~Simulate() {
    thread_pool.wait();
    thread_pool.stop();
}

bool Simulate::keydown(Widgets& widgets, Undo& undo, SDL_Keysym key) {
    return false;
}

void Simulate::step(Scene& scene, float dt) {
    scene.for_items([this, dt](Scene_Item& item) {
        if(item.is<Scene_Particles>()) {
            item.get<Scene_Particles>().step(scene_bvh, dt);
        }
        if(item.is<Scene_Rigidbody>()) {
            item.get<Scene_Rigidbody>().step(dt);
        }
    });
}

void Simulate::update_time() {
    last_update = SDL_GetPerformanceCounter();
}

void Simulate::update(Scene& scene, Undo& undo) {

    update_bvh(scene, undo);

    static Uint64 ufreq = SDL_GetPerformanceFrequency();
    static double freq = (double)ufreq;

    Uint64 time = SDL_GetPerformanceCounter();
    Uint64 udt = time - last_update;

    float dt = clamp((float)(udt / freq), 0.0f, 0.05f);
    last_update = time;

    scene.for_items([this, dt](Scene_Item& item) {
        if(item.is<Scene_Particles>()) {
            Scene_Particles& particles = item.get<Scene_Particles>();
            if(particles.opt.enabled) {
                particles.step(scene_bvh, dt);
            }
        }
        if(item.is<Scene_Rigidbody>()) {
            Scene_Rigidbody& rigidbody = item.get<Scene_Rigidbody>();
            if (rigidbody.opt.enabled) {
                rigidbody.step(dt);
            }
        }
    });
}

void Simulate::render(Scene_Maybe obj_opt, Widgets& widgets, Camera& cam) {

    if(!obj_opt.has_value()) return;
    Scene_Item& item = obj_opt.value();

    if(item.is<Scene_Light>()) {
        Scene_Light& light = item.get<Scene_Light>();
        if(light.is_env()) return;
    }

    Pose& pose = item.pose();
    float scale = std::min((cam.pos() - pose.pos).norm() / 5.5f, 10.0f);
    Mat4 view = cam.get_view();

    item.render(view);
    Renderer::get().outline(view, item);
    widgets.render(view, pose.pos, scale);
}

void Simulate::build_scene(Scene& scene) {

    if(!scene.has_particles()) return;

    std::mutex obj_mut;
    std::vector<PT::Object> obj_list;

    scene.for_items([&, this](Scene_Item& item) {
        if(item.is<Scene_Object>()) {
            Scene_Object& obj = item.get<Scene_Object>();
            thread_pool.enqueue([&]() {
                if(obj.is_shape()) {
                    PT::Shape shape(obj.opt.shape);
                    std::lock_guard<std::mutex> lock(obj_mut);
                    obj_list.push_back(
                        PT::Object(std::move(shape), obj.id(), 0, obj.pose.transform()));
                } else {
                    PT::Tri_Mesh mesh(obj.posed_mesh());
                    std::lock_guard<std::mutex> lock(obj_mut);
                    obj_list.push_back(
                        PT::Object(std::move(mesh), obj.id(), 0, obj.pose.transform()));
                }
            });
        } else if(item.is<Scene_Light>()) {

            Scene_Light& light = item.get<Scene_Light>();
            if(light.opt.type != Light_Type::rectangle) return;

            PT::Tri_Mesh mesh(Util::quad_mesh(light.opt.size.x, light.opt.size.y));

            std::lock_guard<std::mutex> lock(obj_mut);
            obj_list.push_back(PT::Object(std::move(mesh), light.id(), 0, light.pose.transform()));
        }
    });

    thread_pool.wait();
    scene_bvh.build(std::move(obj_list));
}

void Simulate::clear_particles(Scene& scene) {
    scene.for_items([](Scene_Item& item) {
        if(item.is<Scene_Particles>()) {
            item.get<Scene_Particles>().clear();
        }
    });
}

void Simulate::update_bvh(Scene& scene, Undo& undo) {
    if(cur_actions != undo.n_actions()) {
        build_scene(scene);
        cur_actions = undo.n_actions();
    }
}

Mode Simulate::UIsidebar(Manager& manager, Scene& scene, Undo& undo, Widgets& widgets,
                         Scene_Maybe obj_opt) {

    Mode mode = Mode::simulate;
    if(obj_opt.has_value()) {
        ImGui::Text("Object Options");
        mode = manager.item_options(undo, mode, obj_opt.value(), old_pose);
        ImGui::Separator();
    }

    update_bvh(scene, undo);

    if(ImGui::CollapsingHeader("Rigidbody")) {
        ImGui::PushID(0);

        {
            Scene_ID id;
            bool found = false;
            scene.for_items([&id, &found](Scene_Item& item) {
                if (item.is<Scene_Rigidbody>()) {
                    id = item.id();
                    found = true;
                }
            });
            if (found) {
                Scene_Rigidbody& scene_r = scene.get(id).value().get().get<Scene_Rigidbody>();
                ImGui::SliderFloat("Particle radius", &scene_r.particle_radius, 0.0001f, 2.f);
                ImGui::SliderFloat("Time step (delta)", &scene_r.delta_t, 0.00001f, 1.f);
            }
        }



        if(ImGui::Button("Add all objects")) {
            Scene_Rigidbody scene_r = Scene_Rigidbody(scene.reserve_id());
            scene.for_items([&scene_r](Scene_Item& item) {
                if (item.is<Scene_Object>()) {
                    Scene_Object& obj = item.get<Scene_Object>();
                    scene_r.addRigidbody(Rigidbody(scene_r.num_bodies, obj, scene_r.particle_radius, false));
                }
            });
            undo.add_rigidbody(std::move(scene_r));
        }
        if(obj_opt.has_value() && obj_opt.value().get().is<Scene_Object>() && ImGui::Button("Add selected as static rigidbody")) {
            Scene_ID id;
            bool found = false;
            scene.for_items([&id, &found](Scene_Item& item) {
                if (item.is<Scene_Rigidbody>()) {
                    id = item.id();
                    found = true;
                }
            });
            if (found) {
                Scene_Rigidbody& scene_r = scene.get(id).value().get().get<Scene_Rigidbody>();
                Scene_Object& obj = obj_opt.value().get().get<Scene_Object>();
                scene_r.addRigidbody(Rigidbody(scene_r.num_bodies, obj, scene_r.particle_radius, true));
            }
        }
        if(ImGui::Button("Start")) {
            scene.for_items([](Scene_Item& item) {
                if (item.is<Scene_Rigidbody>()) {
                    Scene_Rigidbody& rigidbody = item.get<Scene_Rigidbody>();
                    rigidbody.opt.enabled = true;
                }
            });
        }
        if(ImGui::Button("Stop")) {
            scene.for_items([](Scene_Item& item) {
                if (item.is<Scene_Rigidbody>()) {
                    Scene_Rigidbody& rigidbody = item.get<Scene_Rigidbody>();
                    rigidbody.opt.enabled = false;
                }
            });
        }


        ImGui::PopID();
    }

    return mode;
}

} // namespace Gui
