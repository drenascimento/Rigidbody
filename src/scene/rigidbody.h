#include <vector>
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "object.h"
#include "pose.h"


struct Rigidbody_Particle {
        Vec3 pos;
        Vec3 velocity;

        // TODO: Add particle methods
};

class Rigidbody {
public:
  Rigidbody(Scene_Object&& obj);
  Rigidbody(Rigidbody&& src) = default;
  Rigidbody(const Rigidbody& src) = delete;
  ~Rigidbody() = default;

  const std::vector<Rigidbody_Particle>& particles() const;

  Scene_Object& obj();

  void operator=(const Rigidbody& src) = delete;
  Rigidbody& operator=(Rigidbody&& src) = default;
private:
  Scene_Object body;
  std::vector<Rigidbody_Particle> _particles;

  /* Never updated. */

  // Mass
  int M;
  // Inertia tensor about the center of mass. Initialize it once, recalculate at time step t by using the rotational matrix of t
  Mat4 inv_inertia_tensor;

  /* These are updated in the final step. */

  // Rigid body's center of mass
  Vec3 center_of_mass;
  // Rigid body's quaternion
  Quat quaternion;

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

