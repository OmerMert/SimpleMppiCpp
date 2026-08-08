#include "MPPIController.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <chrono>
#include <cstdlib>

#define M_PI       3.14159265358979323846

// Per-phase breakdown of calc_control_input, printed every PROFILE_EVERY solves when
// MPPI_PHASE_PROFILE=1. Answers "how much of our solve is CPU and how much is GPU" -
// the CPU part is what BeamNG's physics threads compete with.
namespace {
using Clock = std::chrono::steady_clock;
inline double ms_since(const Clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
constexpr int PROFILE_EVERY = 200;
}  // namespace

// --- Constructor ---
MPPIController::MPPIController(
    double delta_t, double wheel_base, double max_steer_abs, double max_accel_abs,
    const MatrixXd& ref_path, int horizon_step_T, int number_of_samples_K,
    double param_exploration, double param_lambda, double param_alpha,
    const Matrix2d& sigma, const Vector4d& stage_cost_weight,
    const Vector4d& terminal_cost_weight,
    const std::vector<Obstacle>& obstacles_param,
    const float influence_radius, const float cbf_weight, const float decay_rate)
    : dt(delta_t), L(wheel_base), max_steer(max_steer_abs), max_accel(max_accel_abs),
      ref_path(ref_path), T(horizon_step_T), K(number_of_samples_K),
      param_exploration(param_exploration), param_lambda(param_lambda), param_alpha(param_alpha),
      Sigma(sigma), stage_cost_weight(stage_cost_weight), terminal_cost_weight(terminal_cost_weight), obstacles(obstacles_param),
      influence_radius(influence_radius), cbf_weight(cbf_weight), decay_rate(decay_rate)
{
    param_gamma = param_lambda * (1.0 - param_alpha);
    u_prev = MatrixXd::Zero(T, dim_u);
    
    // Cholesky decomposition of Sigma
    LLT<MatrixXd> llt(Sigma);
    if (llt.info() == NumericalIssue) {
        throw std::runtime_error("Sigma matrix is not positive!");
    }
    cholesky_L = llt.matrixL();
}


std::tuple<Control, MatrixXd> MPPIController::calc_control_input(const State& observed_x) {
    // load privious control input sequence
    MatrixXd u = u_prev;

    // set initial x value from observation
    State x0 = observed_x;

    static const bool phase_profile = [] {
        const char* e = std::getenv("MPPI_PHASE_PROFILE");
        return e && e[0] == '1';
    }();
    static double t_search = 0, t_noise = 0, t_flat = 0, t_gpu = 0, t_weight = 0, t_rest = 0;
    static int n_profiled = 0;
    auto t_phase = Clock::now();

    // get the waypoint closest to current vehicle position
    _get_nearest_waypoint(x0[0], x0[1], true);
    if (prev_waypoints_idx >= ref_path.rows()) {
        std::cerr << "[Finish] End of the reference path." << std::endl;
        throw std::out_of_range("End of the reference path.");
    }
    t_search += ms_since(t_phase); t_phase = Clock::now();

    // Noise matrix generation
    Eigen::MatrixXd epsilon = _calc_epsilon();
    t_noise += ms_since(t_phase); t_phase = Clock::now();

    // Flatten the epsilon matrix for GPU
    std::vector<float> h_noise(K * T * 2);
    for(int k=0; k<K; ++k) {
        for(int t=0; t<T; ++t) {
            h_noise[k*T*2 + t*2 + 0] = (float)epsilon.row(k*T+t)[0]; // steer
            h_noise[k*T*2 + t*2 + 1] = (float)epsilon.row(k*T+t)[1]; // accel
        }
    }

    // Flatten u_prev for GPU
    std::vector<float> h_u_prev(T * 2);
    for(int t=0; t<T; ++t) {
        h_u_prev[t*2 + 0] = (float)u_prev(t, 0);
        h_u_prev[t*2 + 1] = (float)u_prev(t, 1);
    }

    // Initial state for GPU
    float h_initial_state[4] = {(float)x0[0], (float)x0[1], (float)x0[2], (float)x0[3]};

    // Flatten reference path for GPU
    int path_rows = ref_path.rows();
    std::vector<float> h_ref_path(path_rows * 4);
    for(int i=0; i<path_rows; ++i) {
        h_ref_path[i*4+0] = (float)ref_path(i, 0);
        h_ref_path[i*4+1] = (float)ref_path(i, 1);
        h_ref_path[i*4+2] = (float)ref_path(i, 2);
        h_ref_path[i*4+3] = (float)ref_path(i, 3);
    }

    std::vector<float> h_costs(K);
    float inv_sigma_steer = 1.0f / (float)Sigma(0,0);
    float inv_sigma_accel = 1.0f / (float)Sigma(1,1);
    t_flat += ms_since(t_phase); t_phase = Clock::now();

    // Run GPU Kernel
    launch_mppi_gpu(
        h_initial_state,
        h_u_prev.data(),
        h_noise.data(),
        h_ref_path.data(),
        path_rows,
        obstacles.data(),
        obstacles.size(),
        h_costs.data(),
        K, T, (float)dt,
        prev_waypoints_idx,
        (float)param_exploration,
        // Stage Cost Weights
        (float)stage_cost_weight[0], (float)stage_cost_weight[1], (float)stage_cost_weight[2], (float)stage_cost_weight[3],
        (float)terminal_cost_weight[0], (float)terminal_cost_weight[1], (float)terminal_cost_weight[2], (float)terminal_cost_weight[3],

        param_gamma, inv_sigma_steer, inv_sigma_accel,
        (float)max_steer, (float)max_accel, (float)L,
        (float)vehicle_width, (float)vehicle_length, (float)safety_margin_rate,
        influence_radius, cbf_weight, decay_rate,
        obstacle_costmap_data.empty() ? nullptr : obstacle_costmap_data.data(),
        obs_rows, obs_cols, obs_res, obs_x_min, obs_y_min, obstacle_costmap_weight
    );

    t_gpu += ms_since(t_phase); t_phase = Clock::now();

    // Fill the cost vector from GPU output
    Eigen::VectorXd S(K);
    for(int k=0; k<K; ++k) {
        S[k] = h_costs[k];
    }

    // compute information theoretic weights for each sample
    VectorXd w = _compute_weights(S);

    // calculate w_k * epsilon_k
    MatrixXd w_epsilon = MatrixXd::Zero(T, dim_u);
    for (int t = 0; t < T; ++t) { // loop for time step t = 0 ~ T-1
        for (int k = 0; k < K; ++k) {
            w_epsilon.row(t) += w(k) * epsilon.row(k * T + t);
        }
    }
    
    t_weight += ms_since(t_phase); t_phase = Clock::now();

    // Smooth the control update along the horizon. window_size=10 is the reference
    // implementation's value (MizuhoAOKI mppi_pathtracking_obav.py:121), so our port
    // stays faithful here. A window of 3 was A/B tested: it tracks tighter (mean
    // deviation 0.359 -> 0.321 m in BeamNG) but makes the steering ~2.3x busier
    // (mean |dsteer| 0.0064 -> 0.0147 rad/step). We prefer the smoother command.
    w_epsilon = _moving_average_filter(w_epsilon, 10);
    
    // update control input sequence
    u += w_epsilon;

    // calculate optimal trajectory
    MatrixXd optimal_traj(T, dim_x);
    State x_opt = x0;
    for (int t = 0; t < T; ++t) { // loop for time step t = 0 ~ T-1
        x_opt = _F(x_opt, _g(u.row(t)));
        optimal_traj.row(t) = x_opt;
    }

    // clamp the first control input to ensure safety
    u(0, 0) = std::clamp(u(0, 0), -max_steer, max_steer);
    u(0, 1) = std::clamp(u(0, 1), -max_accel, max_accel);

    // update previous control input sequence
    u_prev.block(0, 0, T - 1, dim_u) = u.block(1, 0, T - 1, dim_u);
    u_prev.row(T - 1) = u.row(T - 1);

    t_rest += ms_since(t_phase);
    if (phase_profile && ++n_profiled % PROFILE_EVERY == 0) {
        const double n = PROFILE_EVERY;
        const double cpu = (t_search + t_noise + t_flat + t_weight + t_rest) / n;
        const double gpu = t_gpu / n;
        std::cout << "[PHASE] " << PROFILE_EVERY << " solve ort (ms): "
                  << "arama " << t_search / n << " | gurultu " << t_noise / n
                  << " | duzlestir " << t_flat / n << " | GPU " << gpu
                  << " | agirlik " << t_weight / n << " | kalan " << t_rest / n
                  << "  =>  CPU " << cpu << " (%" << 100.0 * cpu / (cpu + gpu)
                  << ")  GPU " << gpu << " (%" << 100.0 * gpu / (cpu + gpu) << ")"
                  << std::endl;
        t_search = t_noise = t_flat = t_gpu = t_weight = t_rest = 0;
    }

    return std::make_tuple(u.row(0), optimal_traj);
}

State MPPIController::_F(const State& x_t, const Control& v_t) const {
    double x = x_t[0], y = x_t[1], yaw = x_t[2], v = x_t[3];
    double steer = v_t[0], accel = v_t[1];

    State x_t_plus_1;
    x_t_plus_1[0] = x + v * std::cos(yaw) * dt;
    x_t_plus_1[1] = y + v * std::sin(yaw) * dt;
    x_t_plus_1[2] = yaw + v / L * std::tan(steer) * dt;
    x_t_plus_1[3] = v + accel * dt;
    return x_t_plus_1;
}

Control MPPIController::_g(const Control& v) const {
    Control v_clamped;
    v_clamped[0] = std::clamp(v[0], -max_steer, max_steer);
    v_clamped[1] = std::clamp(v[1], -max_accel, max_accel);
    return v_clamped;
}

// Finds the nearest waypoint on the reference path.
// Forward-only window, identical to the reference implementation (MizuhoAOKI
// _get_nearest_waypoint, SEARCH_IDX_LEN=200). A 50-step backward window was tried for
// "the car can slide backwards in BeamNG", but over 2675 logged steps the nearest index
// never moved backwards - it only cost ~25% extra search work per rollout step.
Vector4d MPPIController::_get_nearest_waypoint(double x, double y, bool update_prev_idx) {

    const int SEARCH_FWD = 200;    // waypoints to search forward

    int start_idx = std::max(0, prev_waypoints_idx);
    int end_idx = std::min(static_cast<int>(ref_path.rows()), prev_waypoints_idx + SEARCH_FWD);

    MatrixXd search_segment = ref_path.block(start_idx, 0, end_idx - start_idx, 2); 

    // Calculate distances to waypoints in the search segment
    Vector2d current_pos(x, y);
    MatrixXd diffs = search_segment.rowwise() - current_pos.transpose();
    
    // Compute squared norms
    VectorXd dist_sq = diffs.rowwise().squaredNorm();

    VectorXd::Index min_idx;
    dist_sq.minCoeff(&min_idx);
    
    int nearest_idx = min_idx + start_idx;
    // update nearest waypoint index if necessary
    if (update_prev_idx) {
        prev_waypoints_idx = nearest_idx;
    }
    
    // [ref_x, ref_y, ref_yaw, ref_v]
    return ref_path.row(nearest_idx).transpose();
}

MatrixXd MPPIController::_calc_epsilon() {
    int total_samples = K * T;
    MatrixXd epsilon(total_samples, dim_u);
    
    // Sample from standard normal distribution
    MatrixXd std_normal_noise(total_samples, dim_u);
    for (int i = 0; i < total_samples; ++i) {
        for (int j = 0; j < dim_u; ++j) {
            std_normal_noise(i, j) = std_normal_dist(generator);
        }
    }
    // Transform to match desired covariance using Cholesky factor
    for (int i = 0; i < total_samples; ++i) {
        epsilon.row(i) = (cholesky_L * std_normal_noise.row(i).transpose()).transpose();
    }
    
    return epsilon;
}

VectorXd MPPIController::_compute_weights(const VectorXd& S) const {
    VectorXd w(K);

    // calculate rho
    double rho = S.minCoeff();

    VectorXd exp_term = (-1.0 / param_lambda * (S.array() - rho)).array().exp();
    
    double eta = exp_term.sum(); 
    
    if (eta < 1e-9) { // Precision safeguard
        w.fill(1.0 / K); // Uniform weights
    } else {
        w = exp_term / eta;
    }
    return w;
}

void MPPIController::set_vehicle_footprint(double width, double length, double safety_margin) {
    if (width > 0.0)         vehicle_width = width;
    if (length > 0.0)        vehicle_length = length;
    if (safety_margin > 0.0) safety_margin_rate = safety_margin;
}

void MPPIController::set_obstacle_costmap(const std::vector<float>& data, int rows, int cols,
                                          float resolution, float x_min, float y_min, float weight) {
    if ((int)data.size() != rows * cols || rows <= 0 || cols <= 0 || weight <= 0.0f) {
        obstacle_costmap_data.clear();
        obs_rows = obs_cols = 0;
        obstacle_costmap_weight = 0.0f;
        return;
    }
    obstacle_costmap_data  = data;
    obs_rows               = rows;
    obs_cols               = cols;
    obs_res                = resolution;
    obs_x_min              = x_min;
    obs_y_min              = y_min;
    obstacle_costmap_weight = weight;
}

MatrixXd MPPIController::_moving_average_filter(const MatrixXd& xx, int window_size) const {
    
    MatrixXd xx_mean = MatrixXd::Zero(xx.rows(), xx.cols());
    int n = xx.rows();
    int half_window = window_size / 2;

    for (int d = 0; d < xx.cols(); ++d) {
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            int count = 0;
            for (int j = std::max(0, i - half_window); j <= std::min(n - 1, i + half_window); ++j) {
                sum += xx(j, d);
                count++;
            }
            if (count > 0) {
                xx_mean(i, d) = sum / count;
            }
        }
    }

    return xx_mean;
}

