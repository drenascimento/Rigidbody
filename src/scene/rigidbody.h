#include <vector>
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "object.h"
#include "pose.h"


class Rigidbody {

public:
  // Particle is in owning rigidbody's center_of_mass space, ie center_of_mass is (0,0,0)
  struct Rigidbody_Particle {
    Rigidbody_Particle(size_t owner_index, Vec3 rel_pos, Vec3 owner_center_of_mass, Quat owner_quat, Vec3 owner_vel, Vec3 owner_angular_vel);

    const Vec3 rel_pos;
    Vec3 pos;
    Vec3 velocity;
    const float mass;

    const size_t owner_index;

    void update(Vec3 owner_center_of_mass, Quat owner_quat, Vec3 owner_vel, Vec3 owner_angular_vel);
  };
  Rigidbody(size_t index, Scene_Object& obj, float particle_size);
  Rigidbody(Rigidbody&& src) = default;
  Rigidbody(const Rigidbody& src) = delete;
  ~Rigidbody() = default;

  std::vector<Rigidbody_Particle>& particles();
  void render(const Mat4& view);
  Pose& pose();

  void accumulate_force(Vec3 partial_force);
  void accumulate_torque(Vec3 partial_torque);

  // Call this function to flush all updates to force and torque
  void apply_partial_updates(float dt);

  // Update position and quaternion based on v and w
  void update_position(float dt);

  const BBox bbox();

  // Offset from mesh position and center of mass
  Vec3 pos_offset = Vec3();
  // Rigid body's center of mass
  Vec3 center_of_mass  = Vec3();
  // Rigid body's quaternion
  Quat quaternion = Quat();

  //const Vec3 center_of_mass();
  //const Quat quaternion();
  const Vec3 angular_velocity();
  const Vec3 velocity();
  const size_t index;

  void operator=(const Rigidbody& src) = delete;
private:
  Scene_Object& body;
  std::vector<Rigidbody_Particle> _particles;
  float particle_radius;
  // Force accumulated in a given iteration
  Vec3 force = Vec3();
  // Torque accumulated in a given iteration
  Vec3 torque = Vec3();
  void populate_particles();

  // Mass, default to 1kg
  float M = 1.f;
  // Inertia tensor about the center of mass at time 0
  Mat4 inv_inertia_tensor;
  // Inertia tensor at current time
  Mat4 inertia_tensor();


  /* Updated in the collision phase. */

  // Linear momentum
  Vec3 P = Vec3();
  // Angular momentum
  Vec3 L = Vec3();

  /* Updated in the first phase. */

  // Velocity of center of mass
  Vec3 v = Vec3();
  // Angular velocity
  Vec3 w = Vec3();
};

