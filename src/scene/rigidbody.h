#include <vector>
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "object.h"
#include "pose.h"

class Rigidbody;

// Particle is in owning rigidbody's center_of_mass space, ie center_of_mass is (0,0,0)
struct Rigidbody_Particle {
  Rigidbody_Particle(Vec3 rel_pos, Rigidbody *owner);

  const Vec3 rel_pos;
  Rigidbody *owner;
  Vec3 pos;
  Vec3 velocity;

  // TODO: Add particle methods
  void update();
};

class Rigidbody {
public:
  Rigidbody(Scene_Object& obj, float particle_size);
  Rigidbody(Rigidbody&& src) = default;
  Rigidbody(const Rigidbody& src) = delete;
  ~Rigidbody() = default;

  std::vector<Rigidbody_Particle>& particles();
  void render(const Mat4& view);
  Pose& pose();

  const BBox bbox();

  const Vec3 center_of_mass();
  const Quat quaternion();
  const Vec3 angular_velocity();
  const Vec3 velocity();

  void operator=(const Rigidbody& src) = delete;
private:
  Scene_Object& body;
  std::vector<Rigidbody_Particle> _particles;
  float particle_radius;
  void populate_particles();

  /* Never updated. */

  // Mass, default to 1kg
  float M = 1.f;
  // Inertia tensor about the center of mass. Initialize it once, recalculate at time step t by using the rotational matrix of t
  Mat4 inv_inertia_tensor;

  /* These are updated in the final step. */

  // Rigid body's center of mass
  Vec3 _center_of_mass;
  // Rigid body's quaternion
  Quat _quaternion;

  /* Updated in the collision phase. */

  // Linear momentum
  Vec3 P;
  // Angular momentum
  Vec3 L;

  /* Updated in the first phase. */

  // Velocity of center of mass
  Vec3 v;
  // Angular velocity
  Vec3 w;
};

