#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>

#define MAX_PARTICLES_IN_CELL 3
//#define SCAN_BLOCK_DIM 1024
//#include "exclusiveScan.cu_inl"

//#include <driver_functions.h>
//#include "../lib/mathlib.h"
//#include "../platform/gl.h"

__device__ float3 operator+(const float3 &a, const float3 &b) {
    return make_float3(a.x+b.x, a.y+b.y, a.z+b.z);
}

__device__ float3 operator-(const float3 &a, const float3 &b) {
    return make_float3(a.x-b.x, a.y-b.y, a.z-b.z);
}

__device__ float3 operator*(const float3&a, const float3 &b) {
    return make_float3(a.x*b.x, a.y*b.y, a.z*b.z);
}

__device__ float3 operator*(const float& a, const float3 &b) {
    return make_float3(a*b.x, a*b.y, a*b.z);
}

__device__ float norm(const float3 &a) {
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

__device__ float3 unit(const float3 &a) {
    float my_norm = norm(a);
    return make_float3(a.x/my_norm, a.y/my_norm, a.z/my_norm);
}

__device__ float dot(const float3 &a, const float3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float3 cross(const float3 &a, const float3 &b) {
    return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

__device__ float4 operator*(const float4 &a, const float4 &r) {
    return make_float4(a.y * r.z - a.z * r.y + a.x * r.w + a.w * r.x, a.z * r.x - a.x * r.z + a.y * r.w + a.w * r.y,
                           a.x * r.y - a.y * r.x + a.z * r.w + a.w * r.z, a.w * r.w - a.x * r.x - a.y * r.y - a.z * r.z);
}

__device__ float4 conjugate(const float4 &a) {
    return make_float4(-a.x, -a.y, -a.z, a.w);
}

__device__ float4 to4(const float3 &a) {
    return make_float4(a.x,a.y,a.z,0);
}

__device__ float3 rotate(const float4 &quat, const float3 &v) {
    float4 partial = (quat * to4(v)) * conjugate(quat);
    return make_float3(partial.x,partial.y,partial.z);
}

// Particles of the form [x,y,z,vx,vy,vz]
__global__ void step_kernel(int N, float *center_of_mass, float *quaternion, float *velocity, float *angular_velocity,
                            float* box_min, float* box_max, float *box_all, int *particle_indices, int *particle_owners, float particle_radius,
                            int num_particles, float *particles, int width, int height, int depth, float *out_force, float *out_torque, int *grid) {

    const int idx = threadIdx.x + blockDim.x * blockIdx.x;
    const int idy = threadIdx.y + blockDim.y * blockIdx.y;
    const int idz = threadIdx.z + blockDim.z * blockIdx.z;
    const int threadIndex = threadIdx.x + threadIdx.y * blockDim.x + threadIdx.z * blockDim.x * blockDim.y;

    /* Clean grid */

    grid[threadIndex] = 0;
    __syncthreads();

    /* All threads < N are each responsible for 1 rigidbody. */

    if (threadIndex < N) {
        // Generate particles and store in local array
        float min_x = box_min[N * 3    ];
        float min_y = box_min[N * 3 + 1];
        float min_z = box_min[N * 3 + 2];

        float max_x = box_max[N * 3    ];
        float max_y = box_max[N * 3 + 1];
        float max_z = box_max[N * 3 + 2];

        const float r = particle_radius;
        const float start_x = - (max_x - min_x) / 2;
        const float end_x = -start_x;
        const float start_y = - (max_y - min_y) / 2;
        const float end_y = -start_y;
        const float start_z = - (max_z - min_z) / 2;
        const float end_z = -start_z;

        int i = particle_indices[threadIndex];
        for (float x = start_x + r; x <= end_x - r; x += 2*r) {
        for (float y = start_y + r; y <= end_y - r; y += 2*r) {
        for (float z = start_z + r; z <= end_z - r; z += 2*r) {
            /* Instantiate particle */
            float3 owner_center_of_mass = *((float3*)(center_of_mass + threadIndex * 3));
            float4 owner_quaternion = *((float4*)(quaternion + threadIndex * 4));
            float3 owner_vel = *((float3*)(velocity + threadIndex * 3));
            float3 owner_angular_vel = *((float3*)(angular_velocity + threadIndex * 3));

            float3 rel_pos = make_float3(x,y,z);

            float3 pos = owner_center_of_mass + rotate(owner_quaternion, rel_pos);
            float3 velocity = owner_vel + cross(owner_angular_vel, rotate(owner_quaternion, rel_pos));
            // Pos (x,y,z)
            particles[i * 6    ] = pos.x;
            particles[i * 6 + 1] = pos.y;
            particles[i * 6 + 2] = pos.z;
            // Vel (x,y,z)
            particles[i * 6 + 3] = velocity.x;
            particles[i * 6 + 4] = velocity.y;
            particles[i * 6 + 5] = velocity.z;

            /* Assign particle owner as rigid body */
            particle_owners[i] = threadIndex;

            /* Also store particles on grid */
            // No need to use a lock, if we miss a particle (who cares, right?) so long as they are sufficiently
            // small, their contribution will not be missed

            size_t x, y, z;
            x = floor((pos.x - box_all[0]) / (2.f * particle_radius));
            y = floor((pos.y - box_all[1]) / (2.f * particle_radius));
            z = floor((pos.z - box_all[2]) / (2.f * particle_radius));

            size_t index = (x * height * depth + y * depth + z) * MAX_PARTICLES_IN_CELL;

            int offset = 0;
            while (offset < MAX_PARTICLES_IN_CELL && grid[index + offset] != 0) offset++;

            if (offset < MAX_PARTICLES_IN_CELL) {
                grid[index + offset] = i;
            }

            i++;
        }}}

        /* Clean out_force and out_torque */
        out_force[threadIndex] = 0;
        out_torque[threadIndex] = 0;

        /* Debugging - DELETE */
        if (!(threadIndex == N-1 && i == num_particles) || (i == particle_indices[threadIndex+1])){
            printf("violation!\n");
        }
        /* Debugging - DELETE */
    }

    __syncthreads();

    /* Collision detection and reaction - write to out_force and out_torque */

    int index = threadIndex;
    int x = idx;
    int y = idy;
    int z = idz;

    for (size_t offset = 0; offset < MAX_PARTICLES_IN_CELL && grid[index + offset] != 0; offset++) {
        // Get all points in a 3x3 block around (x,y,z)
        for (int xp = x-1; xp <= x+1; xp++) {
        for (int yp = y-1; yp <= y+1; yp++) {
        for (int zp = z-1; zp <= z+1; zp++) {
            if (xp < 0 || xp >= width || yp < 0 || yp >= height || zp < 0 || zp >= depth) continue;

            size_t neighbor_index = (xp * height * depth + yp * depth + zp) * MAX_PARTICLES_IN_CELL;

            for (size_t neighbor_offset = 0; neighbor_offset < MAX_PARTICLES_IN_CELL && grid[neighbor_index + neighbor_offset] != 0; neighbor_offset++) {

                /* Possibly found a pair of colliding particles! Update my body only. */
                float3 my_particle_pos = *((float3*)particles + 6 * (grid[index + offset]));
                float3 neighbor_particle_pos = *((float3*)particles + 6 * (grid[neighbor_index + neighbor_offset]));

                float3 my_particle_velocity = *((float3*)particles + 6 * (grid[index + offset]) + 3);
                float3 neighbor_particle_velocity = *((float3*)particles + 6 * (grid[neighbor_index + neighbor_offset]) + 3);

                if (particle_owners[grid[index + offset]] == particle_owners[grid[neighbor_index + neighbor_offset]]) continue;

                float real_spring_coefficient = 0.5f;
                float real_shear_coefficient = 0.5f;
                float real_damping_coefficient = 0.5f;

                // We want the relative pos/vel of neighbor with respect to ourselves
                float3 rel_pos_other = neighbor_particle_pos - my_particle_pos;
                float3 rel_vel_other = neighbor_particle_velocity - my_particle_velocity;
                float3 rel_tangential_vel = rel_vel_other - (dot(rel_vel_other,unit(rel_pos_other)) * unit(rel_pos_other));

                float3 Fis = -real_spring_coefficient * abs(2.f * particle_radius - norm(rel_pos_other)) * unit(rel_pos_other);
                float3 Fid = real_damping_coefficient * rel_vel_other;
                float3 Fit = real_shear_coefficient * rel_tangential_vel;

                // Write updates to out_force and out_torque. For this we will need synchronization primitives

                float3 center_of_mass_owner = *((float3 *)center_of_mass + 3 * particle_owners[grid[index + offset]]);
                float3 rel_pos_to_center = my_particle_pos - center_of_mass_owner;

                float3 total_force = Fis + Fid + Fit;
                float3 total_torque = cross(rel_pos_to_center, total_force);

                atomicAdd(out_force + 3 * (particle_owners[grid[index + offset]]    ), total_force.x);
                atomicAdd(out_force + 3 * (particle_owners[grid[index + offset]] + 1), total_force.y);
                atomicAdd(out_force + 3 * (particle_owners[grid[index + offset]] + 2), total_force.z);

                atomicAdd(out_torque + 3 * (particle_owners[grid[index + offset]]    ), total_torque.x);
                atomicAdd(out_torque + 3 * (particle_owners[grid[index + offset]] + 1), total_torque.y);
                atomicAdd(out_torque + 3 * (particle_owners[grid[index + offset]] + 2), total_torque.z);
            }
        }}}
    }

}

/* In:
 * Center of mass, size 3N
 * Quaternion, size 4N
 * Box min, size 3N
 * Box max, size 3N
 * Box all, size 6 - (x,y,z) min, (x,y,z) max
 *
 * Out:
 * outForce, size N
 * outTorque, size N */
void
update(int N, float particle_radius, float *center_of_mass, float *quaternion, float *velocity, float* angular_velocity,
       float* box_min, float* box_max, float *box_all, float *out_force, float *out_torque) {

    // compute number of blocks and threads per block
    int width  = ceilf((box_all[3] - box_all[0]) / (2.f * particle_radius));
    int height = ceilf((box_all[4] - box_all[1]) / (2.f * particle_radius));
    int depth  = ceilf((box_all[5] - box_all[2]) / (2.f * particle_radius));

    const int block_x = 16;
    const int block_y = 8;
    const int block_z = 8;

    dim3 block(block_x, block_y, block_z);
    dim3 grid((width + block_x - 1) / block_x, (height + block_y - 1) / block_y, (depth + block_z - 1) / block_z);

    int particle_indices[N];
    int num_particles = 0;
    particle_indices[0] = 0;

    for (int i=0; i<N; i++) {
        // Get particle count of ith rigid body
        float min_x = box_min[N * 3    ];
        float min_y = box_min[N * 3 + 1];
        float min_z = box_min[N * 3 + 2];

        float max_x = box_max[N * 3    ];
        float max_y = box_max[N * 3 + 1];
        float max_z = box_max[N * 3 + 2];

        const float r = particle_radius;
        const float start_x = - (max_x - min_x) / 2;
        const float end_x = -start_x;
        const float start_y = - (max_y - min_y) / 2;
        const float end_y = -start_y;
        const float start_z = - (max_z - min_z) / 2;
        const float end_z = -start_z;

        int num_particles_x = floor((end_x - r) - (start_x + r)) / (2*r) + 1;
        int num_particles_y = floor((end_y - r) - (start_y + r)) / (2*r) + 1;
        int num_particles_z = floor((end_z - r) - (start_z + r)) / (2*r) + 1;

        if (i == N-1) {
            num_particles = num_particles_x * num_particles_y * num_particles_z + particle_indices[i];
        } else {
            particle_indices[i+1] = num_particles_x * num_particles_y * num_particles_z + particle_indices[i];
        }
    }

    //const int threadsPerBlock = 512;
    //const int blocks = (N + threadsPerBlock - 1) / threadsPerBlock;

    float* device_center_of_mass;
    float* device_quaternion;
    float* device_box_min;
    float* device_box_max;
    float* device_box_all;
    float* device_velocity;
    float* device_angular_velocity;
    int* device_particle_indices;
    float* device_particles;
    int* device_particle_owners;
    int* device_grid;
    float* device_out_force;
    float* device_out_torque;

    cudaMalloc((void **) &device_center_of_mass, N * 3 * sizeof(float));
    cudaMalloc((void **) &device_quaternion, N * 4 * sizeof(float));
    cudaMalloc((void **) &device_velocity, N * 3 * sizeof(float));
    cudaMalloc((void **) &device_angular_velocity, N * 3 * sizeof(float));
    cudaMalloc((void **) &device_box_min, N * 3 * sizeof(float));
    cudaMalloc((void **) &device_box_max, N * 3 * sizeof(float));
    cudaMalloc((void **) &device_box_all, 6 * sizeof(float));
    cudaMalloc((void **) &device_particle_indices, N * sizeof(int));
    cudaMalloc((void **) &device_particles, num_particles * 6 * sizeof(float));
    cudaMalloc((void **) &device_out_force, N * 3 * sizeof(float));
    cudaMalloc((void **) &device_out_torque, N * 3 * sizeof(float));
    cudaMalloc((void **) &device_grid, width * height * depth * 3 * sizeof(int));
    cudaMalloc((void **) &device_particle_owners, num_particles * sizeof(int));

    cudaMemcpy(device_center_of_mass, center_of_mass, N * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(device_quaternion, quaternion, N * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(device_box_min, box_min, N * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(device_box_max, box_max, N * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(device_box_all, box_all, 6 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(device_particle_indices, particle_indices, N * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(device_velocity, velocity, N * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(device_angular_velocity, angular_velocity, N * 3 * sizeof(float), cudaMemcpyHostToDevice);

    step_kernel<<<grid, block>>>(N, device_center_of_mass, device_quaternion, device_velocity, device_angular_velocity, device_box_min,
                                 device_box_max, device_box_all, device_particle_indices, device_particle_owners, particle_radius,
                                 num_particles, device_particles, width, height, depth, device_out_force, device_out_torque, device_grid);

    cudaDeviceSynchronize();

    cudaMemcpy(out_force, device_out_force, N * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(out_torque, device_out_torque, N * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(device_center_of_mass);
    cudaFree(device_quaternion);
    cudaFree(device_box_min);
    cudaFree(device_box_max);
    cudaFree(device_velocity);
    cudaFree(device_angular_velocity);
    cudaFree(device_particles);
    cudaFree(device_particle_indices);
    cudaFree(device_out_force);
    cudaFree(device_out_torque);
    cudaFree(device_grid)
}
