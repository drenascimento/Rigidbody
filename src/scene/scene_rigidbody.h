#pragma once

#include <vector>
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "object.h"
#include "pose.h"
#include "rigidbody.h"

/* A scene rigid body object holds a set of rigid bodies which can interact
 * with each other. These rigid bodies will not react to any entity not in
 * their same object. */
class Scene_Rigidbody {
public:
  Scene_Rigidbody(Scene_ID id);
  Scene_Rigidbody(Scene_Rigidbody&& src) = default;
  Scene_Rigidbody(const Scene_Rigidbody& src) = delete;
  ~Scene_Rigidbody() = default;

  void operator=(const Scene_Rigidbody& src) = delete;
  Scene_Rigidbody& operator=(Scene_Rigidbody&& src) = default;

  void clear();
  void step(float dt);
  void set_time(float time);

  void addRigidbody(Rigidbody&& body);

  BBox bbox() const;
  void render(const Mat4& view, bool depth_only = false, bool posed = true, bool particles_only = false);
  Scene_ID id() const;

  // TODO: Convert these into parameters
  static const inline float particle_radius = 0.1f;
  static const inline float spring_coefficient = 0.5f;
  static const inline float damping_coefficient = 0.5f;
  static const inline float shear_coefficient = 0.5f;

  static const inline int max_particles_in_voxel = 4;
  static const inline int max_name_len = 256;
  struct Options {
    char name[max_name_len] = {};
    bool enabled = true;
    Spectrum color = Spectrum(0.5f);
  };

  Options opt;
  Pose pose;
  Anim_Pose anim;
  size_t num_bodies = 0;
private:
  void partial_step(float dt);
  std::vector<Rigidbody> bodies;
  Scene_ID _id;
  BBox _bbox;

  void for_rigidbody(std::function<void(Rigidbody&)> func);
};
