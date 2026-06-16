#include "MPPIController.h"
#include <iostream>
#include <cmath>
#include <limits>

#define M_PI       3.14159265358979323846

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

    // get the waypoint closest to current vehicle position 
    _get_nearest_waypoint(x0[0], x0[1], true);
    if (prev_waypoints_idx >= ref_path.rows() - 4) {
        std::cerr << "[Finish] End of the reference path." << std::endl;
        throw std::out_of_range("End of the reference path.");
    }

    // Noise matrix generation
    Eigen::MatrixXd epsilon = _calc_epsilon(); 

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
        influence_radius, cbf_weight, decay_rate
    );

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
    
    // apply moving average filter for smoothing input sequence
    w_epsilon = _moving_average_filter(w_epsilon, 3);
    
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

double MPPIController::normalize_angle(double angle) const {
    return std::fmod(angle + 2.0 * M_PI , 2.0 * M_PI);
}

double MPPIController::_c(const State& x_t) {
    Vector4d ref = _get_nearest_waypoint(x_t[0], x_t[1]);
    
    double ref_x = ref[0], ref_y = ref[1], ref_yaw = ref[2], ref_v = ref[3];
    
    State x_normalized = x_t;
    x_normalized[2] = normalize_angle(x_t[2]);

    double cost = stage_cost_weight[0] * std::pow(x_normalized[0] - ref_x, 2) +
                  stage_cost_weight[1] * std::pow(x_normalized[1] - ref_y, 2) +
                  stage_cost_weight[2] * std::pow(x_normalized[2] - ref_yaw, 2) +
                  stage_cost_weight[3] * std::pow(x_normalized[3] - ref_v, 2);

    cost += _is_collided(x_t) * 1.0e10;

    return cost;
}

double MPPIController::_phi(const State& x_T) {
    Vector4d ref = _get_nearest_waypoint(x_T[0], x_T[1]);
    double ref_x = ref[0], ref_y = ref[1], ref_yaw = ref[2], ref_v = ref[3];
    
    State x_normalized = x_T;
    x_normalized[2] = normalize_angle(x_T[2]);
    
    double cost = terminal_cost_weight[0] * std::pow(x_normalized[0] - ref_x, 2) +
                  terminal_cost_weight[1] * std::pow(x_normalized[1] - ref_y, 2) +
                  terminal_cost_weight[2] * std::pow(x_normalized[2] - ref_yaw, 2) +
                  terminal_cost_weight[3] * std::pow(x_normalized[3] - ref_v, 2);

    cost += _is_collided(x_T) * 1.0e10;

    return cost;
}

// Finds the nearest waypoint on the reference path.
// IMPORTANT: In dynamic simulators the vehicle can slide backwards.
// Instead of searching only forward, this now also looks backward (bidirectional window).
Vector4d MPPIController::_get_nearest_waypoint(double x, double y, bool update_prev_idx) {

    const int SEARCH_FWD = 200;    // waypoints to search forward
    const int SEARCH_BWD = 50;     // waypoints to search backward
    
    int start_idx = std::max(0, prev_waypoints_idx - SEARCH_BWD);
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

double MPPIController::_is_collided(const State& x_t) {

    // vehicle shape parameters
    double vw = vehicle_width * safety_margin_rate;
    double vl = vehicle_length * safety_margin_rate;

    // get current states
    double x = x_t[0];
    double y = x_t[1];
    double yaw = x_t[2];

    // key points for collision check
    std::vector<Eigen::Vector2d> local_points = {
        {-0.5 * vl, -0.5 * vw}, {-0.5 * vl, 0.0}, {-0.5 * vl, +0.5 * vw}, 
        { 0.0,      +0.5 * vw}, { 0.0,     -0.5 * vw}, { 0.0, 0.0},       
        {+0.5 * vl, +0.5 * vw}, {+0.5 * vl, 0.0}, {+0.5 * vl, -0.5 * vw}  
    };


    // check if the key points are inside the obstacles
    for (const auto& obs : obstacles) {
        double obs_r_sq = obs.r * obs.r; 

        for (const auto& p : local_points) {
            
            double global_px = (p.x() * std::cos(yaw) - p.y() * std::sin(yaw)) + x;
            double global_py = (p.x() * std::sin(yaw) + p.y() * std::cos(yaw)) + y;

            double dist_sq = std::pow(global_px - obs.x, 2) + std::pow(global_py - obs.y, 2);

            if (dist_sq < obs_r_sq) {
                return 1.0; // collided
            }
        }
    }

    return 0.0; // not collided
}

void MPPIController::set_weights(double w_x, double w_y, double w_yaw, double w_v) {
        stage_cost_weight << w_x, w_y, w_yaw, w_v;
        terminal_cost_weight << w_x, w_y, w_yaw, w_v;
}

// --- Reset MPPI Internal State ---
void MPPIController::reset() {
    prev_waypoints_idx = 0; 
    u_prev.setZero();       
}