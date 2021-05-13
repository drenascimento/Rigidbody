#ifndef __kernels_h__
#define __kernels_h__

class Kernels
{
public:
    static void update(int N, float particle_radius, float *center_of_mass, float *quaternion, float *velocity, float* angular_velocity,
                       float* box_min, float* box_max, float *box_all, float *out_force, float *out_torque);
};

#endif
