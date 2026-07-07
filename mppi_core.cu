#include <cuda_runtime.h>
#include <math.h>
#include <stdio.h>

#define M_PI 3.14159265358979323846f

struct Obstacle {
    float x, y, r;
};

__device__ float normalize_angle_diff(float diff) {
    while (diff > M_PI) diff -= 2.0f * M_PI;
    while (diff < -M_PI) diff += 2.0f * M_PI;
    return diff;
}

__device__ void update_state_gpu(
    float* x, float* y, float* yaw, float* v, 
    float steer, float accel, float dt,
    float wheelbase
) {
    float current_x = *x;
    float current_y = *y;
    float current_yaw = *yaw;
    float current_v = *v;

    *x = current_x + current_v * cosf(current_yaw) * dt;
    *y = current_y + current_v * sinf(current_yaw) * dt;
    *yaw = current_yaw + current_v / wheelbase * tanf(steer) * dt;
    *v = current_v + accel * dt;

}

// CBF Function
__device__ float compute_cbf_cost(
    float x, float y, float yaw, 
    const Obstacle* obstacles, int num_obs,
    float vw, float vl,
    float influence_radius, float cbf_weight, float decay_rate) 
    {
    float total_barrier_cost = 0.0f;
    
    // Vehicle body key points (same footprint as the hard collision check).
    float local_x[9] = {-0.5f*vl, -0.5f*vl, -0.5f*vl,  0.0f,      0.0f,     0.0f,     0.5f*vl, 0.5f*vl, 0.5f*vl};
    float local_y[9] = {-0.5f*vw,  0.0f,     0.5f*vw,  0.5f*vw,  -0.5f*vw,  0.0f,     0.5f*vw, 0.0f,   -0.5f*vw};
    float c = cosf(yaw);
    float s = sinf(yaw);

    for(int i = 0; i < num_obs; ++i) {
        float obs_x = obstacles[i].x;
        float obs_y = obstacles[i].y;
        float obs_r = obstacles[i].r;

        // Closest approach of ANY body key point to the obstacle SURFACE.
        // Footprint-based (NOT a circumscribing circle): the old
        // robot_radius = 0.5*sqrt(vw^2+vl^2) treated a 4.8 m car as a 3 m-radius
        // disk -> a ~5 m keep-out -> avoidance needed a ~4 m swerve -> infeasible,
        // so the car just stopped. Using the real footprint keeps it feasible.
        float min_h = 1e10f;
        for(int p = 0; p < 9; ++p) {
            float gpx = (local_x[p] * c - local_y[p] * s) + x;
            float gpy = (local_x[p] * s + local_y[p] * c) + y;
            float d = sqrtf((gpx - obs_x)*(gpx - obs_x) + (gpy - obs_y)*(gpy - obs_y));
            float h = d - obs_r;
            if (h < min_h) min_h = h;
        }

        if (min_h <= 0.0f) {
            return 1000000000.0f;   // a key point is inside the obstacle -> collision
        }
        if (min_h < influence_radius) {
            total_barrier_cost += cbf_weight * expf(-decay_rate * min_h);
        }
    }

    return total_barrier_cost;
}

__device__ bool check_collision_gpu(
    float x, float y, float yaw, 
    const Obstacle* obstacles, int num_obs,
    float vw, float vl) {
    
    // key points for collision check
    float local_x[9] = {-0.5f*vl, -0.5f*vl, -0.5f*vl,  0.0f,      0.0f,     0.0f,     0.5f*vl, 0.5f*vl, 0.5f*vl};
    float local_y[9] = {-0.5f*vw,  0.0f,     0.5f*vw,  0.5f*vw,  -0.5f*vw,  0.0f,     0.5f*vw, 0.0f,   -0.5f*vw};

    // check if the key points are inside the obstacles
    for(int i = 0; i < num_obs; ++i) {

        float r_sq = obstacles[i].r * obstacles[i].r;

        for(int p = 0; p < 9; ++p) {
            float global_px = (local_x[p] * cosf(yaw) - local_y[p] * sinf(yaw)) + x;
            float global_py = (local_x[p] * sinf(yaw) + local_y[p] * cosf(yaw)) + y;

            float dist_sq = (global_px - obstacles[i].x)*(global_px - obstacles[i].x) + (global_py - obstacles[i].y)*(global_py - obstacles[i].y);

            if(dist_sq < r_sq) {
                return true; // collided
            }
        }
    }
    return false;
}

__device__ void get_nearest_waypoint_gpu(
    float x, float y,
    const float* path_points, int path_size,
    int prev_idx,
    float* ref_x, float* ref_y, float* ref_yaw, float* ref_v,
    const Obstacle* obstacles, int num_obs
) {
    float min_dist_sq = 1e10f;
    int nearest = prev_idx;
    
    // Bidirectional search: 50 backward, 200 forward
    int SEARCH_BWD = 50;
    int SEARCH_FWD = 200;
    int start_idx = (prev_idx - SEARCH_BWD > 0) ? prev_idx - SEARCH_BWD : 0;
    int end_idx = (prev_idx + SEARCH_FWD < path_size) ? prev_idx + SEARCH_FWD : path_size;

    for(int i = start_idx; i < end_idx; ++i) {
        float px = path_points[i * 4 + 0];
        float py = path_points[i * 4 + 1];
        
        float dx = x - px;
        float dy = y - py;
        float d_sq = dx*dx + dy*dy;
        
        if(d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            nearest = i;
        }
    }

    *ref_x   = path_points[nearest * 4 + 0];
    *ref_y   = path_points[nearest * 4 + 1];
    *ref_yaw = path_points[nearest * 4 + 2];
    *ref_v   = path_points[nearest * 4 + 3];
    // (reference-detour removed: it was hardcoded to the -y side, which only works
    //  for obstacles above the path. For side obstacles on EITHER side the CBF
    //  footprint gradient already pushes the car to the correct side.)
}

__global__ void mppi_rollout_kernel(
    const float* initial_state, // [x, y, yaw, v]
    const float* u_prev,        // [steer, accel] x T 
    const float* noise,         // [steer, accel] x K x T 
    const float* ref_path,      
    int path_size,
    const Obstacle* obstacles,  
    int num_obs,
    float* costs,               
    int K, int T, float dt,
    int prev_waypoint_idx,
    float param_exploration,
    float w_x, float w_y, float w_yaw, float w_v,
    float term_w_x, float term_w_y, float term_w_yaw, float term_w_v,
    float param_gamma,
    float inv_sigma_steer, // 1.0 / Sigma[0,0]
    float inv_sigma_accel,  // 1.0 / Sigma[1,1]
    float max_steer, float max_accel, float wheelbase,
    float vehicle_w, float vehicle_l,
    float influence_radius, float cbf_weight, float decay_rate
) {
    
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= K) return; 

    float x = initial_state[0];
    float y = initial_state[1];
    float yaw = initial_state[2];
    float v = initial_state[3];

    float total_cost = 0.0f;

    int exploitation_count = (int)((1.0f - param_exploration) * K);
    bool is_exploration = (k >= exploitation_count);


    int local_waypoint_idx = prev_waypoint_idx;

    for (int t = 0; t < T; ++t) {

        int idx = k * T * 2 + t * 2;
        int u_idx = t * 2;

        float n_steer = noise[idx + 0];
        float n_accel = noise[idx + 1];

        float u_prev_steer = u_prev[u_idx + 0];
        float u_prev_accel = u_prev[u_idx + 1];

        float steer, accel;

        if (is_exploration) {
            // Pure noise (Exploration)
            steer = n_steer;
            accel = n_accel;
        } else {
            // Previous plan + noise (Exploitation)
            steer = u_prev[u_idx + 0] + n_steer;
            accel = u_prev[u_idx + 1] + n_accel;
        }

        // Clamp
        if(steer > max_steer) steer = max_steer;
        if(steer < -max_steer) steer = -max_steer;
        if(accel > max_accel) accel = max_accel;
        if(accel < -max_accel) accel = -max_accel;

        // (_F)
        update_state_gpu(&x, &y, &yaw, &v, steer, accel, dt, wheelbase);

        // Stage Cost (_c): search while tracking waypoint index along the horizon
        float rx, ry, ryaw, rv;
        {
            float min_d = 1e10f;
            int SFWD = 200, SBWD = 50;
            int si = (local_waypoint_idx - SBWD > 0) ? local_waypoint_idx - SBWD : 0;
            int ei = (local_waypoint_idx + SFWD < path_size) ? local_waypoint_idx + SFWD : path_size;
            for (int i = si; i < ei; ++i) {
                float dx = x - ref_path[i*4+0];
                float dy = y - ref_path[i*4+1];
                float d = dx*dx + dy*dy;
                if (d < min_d) { min_d = d; local_waypoint_idx = i; }
            }
            rx   = ref_path[local_waypoint_idx*4+0];
            ry   = ref_path[local_waypoint_idx*4+1];
            ryaw = ref_path[local_waypoint_idx*4+2];
            rv   = ref_path[local_waypoint_idx*4+3];
        }

        float yaw_diff = normalize_angle_diff(yaw - ryaw);

        float stage_cost = w_x*(x-rx)*(x-rx) + 
                        w_y*(y-ry)*(y-ry) + 
                        w_yaw*(yaw_diff)*(yaw_diff) +
                        w_v*(v-rv)*(v-rv);

        // control barrier function
        stage_cost += compute_cbf_cost(x, y, yaw, obstacles, num_obs, vehicle_w, vehicle_l, 
                                       influence_radius, cbf_weight, decay_rate);


        float control_cost = 0.0f;
        if (!is_exploration) {
            control_cost = param_gamma * (
                u_prev_steer * steer * inv_sigma_steer + 
                u_prev_accel * accel * inv_sigma_accel
            );
        }
        
        total_cost += stage_cost + control_cost;
    }

    // Terminal Cost: local_waypoint_idx reflects the position at the end of the horizon
    float rx, ry, ryaw, rv;
    get_nearest_waypoint_gpu(x, y, ref_path, path_size, local_waypoint_idx, &rx, &ry, &ryaw, &rv, obstacles, num_obs);

    float term_yaw_diff = normalize_angle_diff(yaw - ryaw);

    float terminal_cost = term_w_x*(x-rx)*(x-rx) + 
                        term_w_y*(y-ry)*(y-ry) + 
                        term_w_yaw*(term_yaw_diff)*(term_yaw_diff) +
                        term_w_v*(v-rv)*(v-rv);
                        
    if(check_collision_gpu(x, y, yaw, obstacles, num_obs, vehicle_w, vehicle_l)) {
        terminal_cost += 1000000000.0f;
    }

    total_cost += terminal_cost;

    costs[k] = total_cost;
}



extern "C" void launch_mppi_gpu(
    const float* h_initial_state,
    const float* h_u_prev,
    const float* h_noise,
    const float* h_ref_path,
    int path_size,
    const Obstacle* h_obstacles,
    int num_obs,
    float* h_costs, 
    int K, int T, float dt,
    int prev_idx,
    float param_exploration,
    float w_x, float w_y, float w_yaw, float w_v,
    float term_w_x, float term_w_y, float term_w_yaw, float term_w_v,
    float param_gamma, float inv_sigma_steer, float inv_sigma_accel,
    float max_steer, float max_accel, float wheelbase,
    float vehicle_w_param, float vehicle_l_param, float safety_margin,
    float influence_radius_param, float cbf_weight_param, float decay_rate_param
) {
    float *d_state, *d_u, *d_noise, *d_path, *d_costs;
    Obstacle *d_obs;
    
    cudaMalloc(&d_state, 4 * sizeof(float));
    cudaMalloc(&d_u, T * 2 * sizeof(float));
    cudaMalloc(&d_noise, K * T * 2 * sizeof(float));
    cudaMalloc(&d_path, path_size * 4 * sizeof(float));
    cudaMalloc(&d_costs, K * sizeof(float));
    cudaMalloc(&d_obs, num_obs * sizeof(Obstacle));

    cudaMemcpy(d_state, h_initial_state, 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_u, h_u_prev, T * 2 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_noise, h_noise, K * T * 2 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_path, h_ref_path, path_size * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obs, h_obstacles, num_obs * sizeof(Obstacle), cudaMemcpyHostToDevice);

    int threadsPerBlock = 256;
    int blocksPerGrid = (K + threadsPerBlock - 1) / threadsPerBlock;
    
    float final_w = vehicle_w_param * safety_margin;
    float final_l = vehicle_l_param * safety_margin;


    mppi_rollout_kernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_state, d_u, d_noise, d_path, path_size, d_obs, num_obs, d_costs,
        K, T, dt, prev_idx, param_exploration,
        w_x, w_y, w_yaw, w_v,
        term_w_x, term_w_y, term_w_yaw, term_w_v,
        param_gamma, inv_sigma_steer, inv_sigma_accel,
        max_steer, max_accel, wheelbase,
        final_w, final_l, 
        influence_radius_param, cbf_weight_param, decay_rate_param
    );
    

    cudaDeviceSynchronize();

    cudaMemcpy(h_costs, d_costs, K * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_state); cudaFree(d_u); cudaFree(d_noise); 
    cudaFree(d_path); cudaFree(d_costs); cudaFree(d_obs);
}