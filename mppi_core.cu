// mppi_core.cu
#include <cuda_runtime.h>
#include <math.h>
#include <stdio.h>

// Yapılarımızı burada tekrar tanımlayalım (veya ortak bir .h dosyasından çekelim)
struct Obstacle {
    float x, y, r;
};

// --- YARDIMCI FONKSİYONLAR (GPU İÇİN __device__) ---

// En yakın nokta bulma (Basitleştirilmiş GPU versiyonu)
__device__ void get_nearest_waypoint_gpu(
    float x, float y, 
    const float* path_points, int path_size, 
    int prev_idx, 
    float* ref_x, float* ref_y, float* ref_yaw, float* ref_v
) {
    float min_dist_sq = 1e10f;
    int nearest = prev_idx;
    int search_len = 200; // Arama aralığı

    // Sınırları kontrol et
    int end_idx = (prev_idx + search_len < path_size) ? prev_idx + search_len : path_size;

    for(int i = prev_idx; i < end_idx; ++i) {
        float dx = x - path_points[i * 4 + 0]; // 0: x
        float dy = y - path_points[i * 4 + 1]; // 1: y
        float d_sq = dx*dx + dy*dy;
        if(d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            nearest = i;
        }
    }

    *ref_x = path_points[nearest * 4 + 0];
    *ref_y = path_points[nearest * 4 + 1];
    *ref_yaw = path_points[nearest * 4 + 2];
    *ref_v = path_points[nearest * 4 + 3];
}

// Çarpışma Kontrolü
__device__ bool check_collision_gpu(float x, float y, float yaw, const Obstacle* obstacles, int num_obs) {
    // Araç boyutları (Sabit kabul ediyoruz)
    float vl = 4.0f * 1.2f; // Safety margin dahil
    float vw = 3.0f * 1.2f;
    
    // Basitlik için sadece 3 noktayı kontrol edelim (Ön, Orta, Arka)
    float local_pts_x[3] = {vl/2.0f, 0.0f, -vl/2.0f};
    
    for(int i=0; i<num_obs; ++i) {
        float obs_x = obstacles[i].x;
        float obs_y = obstacles[i].y;
        float r_sq = obstacles[i].r * obstacles[i].r;

        for(int p=0; p<3; ++p) {
            // Döndür ve ötele
            float gx = x + (local_pts_x[p] * cosf(yaw));
            float gy = y + (local_pts_x[p] * sinf(yaw));
            
            float dx = gx - obs_x;
            float dy = gy - obs_y;
            
            if (dx*dx + dy*dy < r_sq) return true;
        }
    }
    return false;
}

// --- ANA GPU KERNEL ---
__global__ void mppi_rollout_kernel(
    const float* initial_state, // [x, y, yaw, v]
    const float* u_prev,        // [steer, accel] * T
    const float* noise,         // [steer, accel] * K * T
    const float* ref_path,      // [x, y, yaw, v] * PathSize
    int path_size,
    const Obstacle* obstacles,
    int num_obs,
    float* costs,               // Çıktı: Her sample için maliyet
    int K, int T, float dt,
    int prev_waypoint_idx
) {
    // Ben hangi işçiyim? (Sample Index)
    int k = blockIdx.x * blockDim.x + threadIdx.x;

    if (k >= K) return;

    // Yerel durum değişkenleri
    float x = initial_state[0];
    float y = initial_state[1];
    float yaw = initial_state[2];
    float v = initial_state[3];

    float total_cost = 0.0f;

    // Simülasyon Döngüsü (T adım)
    for (int t = 0; t < T; ++t) {
        // 1. Kontrol Girdisini Hazırla
        // noise dizini: k * (T * 2) + t * 2
        int noise_idx = k * T * 2 + t * 2;
        int u_idx = t * 2;

        float noise_steer = noise[noise_idx + 0];
        float noise_accel = noise[noise_idx + 1];

        // u + epsilon
        float steer = u_prev[u_idx + 0] + noise_steer;
        float accel = u_prev[u_idx + 1] + noise_accel;

        // Clamp (Sınırla)
        if(steer > 0.523f) steer = 0.523f;
        if(steer < -0.523f) steer = -0.523f;
        if(accel > 2.0f) accel = 2.0f;
        if(accel < -2.0f) accel = -2.0f;

        // 2. Kinematik Model (Vehicle Update)
        x += v * cosf(yaw) * dt;
        y += v * sinf(yaw) * dt;
        yaw += v / 2.5f * tanf(steer) * dt; // 2.5 = wheelbase
        v += accel * dt;

        // 3. Maliyet Hesabı (Stage Cost)
        float rx, ry, ryaw, rv;
        get_nearest_waypoint_gpu(x, y, ref_path, path_size, prev_waypoint_idx, &rx, &ry, &ryaw, &rv);
        
        // Ağırlıklar (Basitlik için sabit yazdım, parametre olarak geçilebilir)
        float cost = 50.0f*(x-rx)*(x-rx) + 50.0f*(y-ry)*(y-ry) + 1.0f*(yaw-ryaw)*(yaw-ryaw) + 20.0f*(v-rv)*(v-rv);

        // Çarpışma Cezası
        if(check_collision_gpu(x, y, yaw, obstacles, num_obs)) {
            cost += 1000000.0f;
        }

        // Kontrol Maliyeti (Control Cost - Simplified)
        // lambda(1-alpha) * u' * Sigma^-1 * v ...
        // Bu kısmı tam formüle göre eklemek gerekir, şimdilik basit tuttum.
        
        total_cost += cost;
    }

    // Sonucu global belleğe yaz
    costs[k] = total_cost;
}

// --- C++'tan Çağırılacak "Wrapper" Fonksiyon ---
extern "C" void launch_mppi_gpu_wrapper(
    const float* h_initial_state,
    const float* h_u_prev,
    const float* h_noise,
    const float* h_ref_path,
    int path_size,
    const Obstacle* h_obstacles,
    int num_obs,
    float* h_costs,
    int K, int T, float dt,
    int prev_idx
) {
    // 1. GPU Belleği Ayır (Malloc)
    // Gerçek uygulamada bunları her döngüde yapmamalı,
    // Class içinde bir kez yapıp saklamalıyız.
    // Ancak eğitim amaçlı buraya koyuyorum.
    
    float *d_initial_state, *d_u_prev, *d_noise, *d_ref_path, *d_costs;
    Obstacle *d_obstacles;

    cudaMalloc(&d_initial_state, 4 * sizeof(float));
    cudaMalloc(&d_u_prev, T * 2 * sizeof(float));
    cudaMalloc(&d_noise, K * T * 2 * sizeof(float));
    cudaMalloc(&d_ref_path, path_size * 4 * sizeof(float));
    cudaMalloc(&d_costs, K * sizeof(float));
    cudaMalloc(&d_obstacles, num_obs * sizeof(Obstacle));

    // 2. Veriyi Kopyala (Host -> Device)
    cudaMemcpy(d_initial_state, h_initial_state, 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_u_prev, h_u_prev, T * 2 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_noise, h_noise, K * T * 2 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ref_path, h_ref_path, path_size * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obstacles, h_obstacles, num_obs * sizeof(Obstacle), cudaMemcpyHostToDevice);

    // 3. Kernel'i Başlat
    int threadsPerBlock = 256;
    int blocksPerGrid = (K + threadsPerBlock - 1) / threadsPerBlock;
    
    mppi_rollout_kernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_initial_state, d_u_prev, d_noise, d_ref_path, path_size,
        d_obstacles, num_obs, d_costs, K, T, dt, prev_idx
    );
    
    // Hata kontrolü
    cudaDeviceSynchronize();

    // 4. Sonucu Geri Al (Device -> Host)
    cudaMemcpy(h_costs, d_costs, K * sizeof(float), cudaMemcpyDeviceToHost);

    // 5. Temizlik
    cudaFree(d_initial_state);
    cudaFree(d_u_prev);
    cudaFree(d_noise);
    cudaFree(d_ref_path);
    cudaFree(d_costs);
    cudaFree(d_obstacles);
}