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

class Scene_Rigidbody {
public:
  Scene_Rigidbody(Scene_ID id);
  ~Scene_Rigidbody() = default;

  void clear();
  void step(float dt);
  const std::vector<Rigidbody_Particle>& get_particles() const;

  BBox bbox() const;
  void render(const Mat4& view, bool depth_only = false, bool posed = true, bool particles_only = false);
  Scene_ID id() const;
  void set_time(float time);

  const GL::Mesh& mesh() const;
  void take_mesh(GL::Mesh&& mesh);

  // NOTE: What's this for?
  static const inline int max_name_len = 256;
  struct Options {
    char name[max_name_len] = {};
    bool enabled = false;
    // TODO: Do we care about the rest of these?
    //        Spectrum color = Spectrum(1.0f);
    //        float velocity = 25.0f;
    //        float angle = 0.0f;
    //        float scale = 0.1f;
    //        float lifetime = 15.0f;
    //        float pps = 5.0f;
  };


  // TODO: Figure out what to do with these
  Options opt;
  Pose pose;
  Anim_Pose anim;
private:
  /* Scene related variables/methods */

  Scene_ID _id;
  // FIXME: For now only 1 mesh per Scene_Rigidbody. Later we want as many meshes as necessary. Also just hardcoded cube mesh.
  GL::Mesh base_mesh;

  /* Structure */
  // FIXME: Again only 1 mesh so particles belong only to it.
  std::vector<Rigidbody_Particle> particles;


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
