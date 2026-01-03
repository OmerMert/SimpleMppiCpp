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
    float* ref_x, float* ref_y, float* ref_yaw, float* ref_v
) {
    float min_dist_sq = 1e10f;
    int nearest = prev_idx;
    
    int SEARCH_IDX_LEN = 200; 
    int end_idx = (prev_idx + SEARCH_IDX_LEN < path_size) ? prev_idx + SEARCH_IDX_LEN : path_size;

    for(int i = prev_idx; i < end_idx; ++i) {
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
    float vehicle_w, float vehicle_l
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


    for (int t = 0; t < T; ++t) {

        int idx = k * T * 2 + t * 2;
        int u_idx = t * 2;

        float n_steer = noise[idx + 0];
        float n_accel = noise[idx + 1];

        float u_prev_steer = u_prev[u_idx + 0];
        float u_prev_accel = u_prev[u_idx + 1];

        float steer, accel;

        if (is_exploration) {
            // Sadece gürültü (Keşif)
            steer = n_steer;
            accel = n_accel;
        } else {
            // Önceki plan + gürültü (Sömürü)
            steer = u_prev[u_idx + 0] + n_steer;
            accel = u_prev[u_idx + 1] + n_accel;
        }

        // Clamp (Sınırla)
        if(steer > max_steer) steer = max_steer;
        if(steer < -max_steer) steer = -max_steer;
        if(accel > max_accel) accel = max_accel;
        if(accel < -max_accel) accel = -max_accel;

        // Durumu Güncelle (_F)
        update_state_gpu(&x, &y, &yaw, &v, steer, accel, dt, wheelbase);

        // Stage Cost (_c)
        float rx, ry, ryaw, rv;
        get_nearest_waypoint_gpu(x, y, ref_path, path_size, prev_waypoint_idx, &rx, &ry, &ryaw, &rv);
        
        float yaw_diff = normalize_angle_diff(yaw - ryaw);

        float stage_cost = w_x*(x-rx)*(x-rx) + 
                        w_y*(y-ry)*(y-ry) + 
                        w_yaw*(yaw_diff)*(yaw_diff) +
                        w_v*(v-rv)*(v-rv);

        // Çarpışma Cezası
        if(check_collision_gpu(x, y, yaw, obstacles, num_obs, vehicle_w, vehicle_l)) {
            stage_cost += 1000000000.0f; 
        }

        float control_cost = 0.0f;
        if (!is_exploration) {
            control_cost = param_gamma * (
                u_prev_steer * steer * inv_sigma_steer + 
                u_prev_accel * accel * inv_sigma_accel
            );
        }
        
        total_cost += stage_cost + control_cost;
    }

    // Terminal Cost
    float rx, ry, ryaw, rv;
    get_nearest_waypoint_gpu(x, y, ref_path, path_size, prev_waypoint_idx, &rx, &ry, &ryaw, &rv);

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
    float vehicle_w_param, float vehicle_l_param, float safety_margin
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
        final_w, final_l
    );
    

    cudaDeviceSynchronize();

    cudaMemcpy(h_costs, d_costs, K * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_state); cudaFree(d_u); cudaFree(d_noise); 
    cudaFree(d_path); cudaFree(d_costs); cudaFree(d_obs);
}