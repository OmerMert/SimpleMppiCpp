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
    const std::vector<Obstacle>& obstacles_param)
    : dt(delta_t), L(wheel_base), max_steer(max_steer_abs), max_accel(max_accel_abs),
      ref_path(ref_path), T(horizon_step_T), K(number_of_samples_K),
      param_exploration(param_exploration), param_lambda(param_lambda), param_alpha(param_alpha),
      Sigma(sigma), stage_cost_weight(stage_cost_weight), terminal_cost_weight(terminal_cost_weight), obstacles(obstacles_param)
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
    if (prev_waypoints_idx >= ref_path.rows() - 1) {
        std::cerr << "[ERROR] End of the reference path." << std::endl;
        throw std::out_of_range("End of the reference path.");
    }

    // prepare buffer
    VectorXd S = VectorXd::Zero(K); // state cost list
    
    // sample noise
    MatrixXd epsilon = _calc_epsilon(); // size is K*T, dim_u

    // prepare buffer of sampled control input sequence
    std::vector<MatrixXd> v(K, MatrixXd(T, dim_u)); // control input sequence with noise

    // loop for 0 ~ K-1 samples
    for (int k = 0; k < K; ++k) {

        // set initial(t=0) state x i.e. observed state of the vehicle
        State x = x0;

        // loop for time step t = 1 ~ T
        for (int t = 0; t < T; ++t) {
            
            // get control input with noise
            if (k < (1.0 - param_exploration) * K) {
                v[k].row(t) = u.row(t) + epsilon.row(k * T + t); // sampling for exploitation
            } else {
                v[k].row(t) = epsilon.row(k * T + t); //  sampling for exploitation
            }

            // update x
            Control v_clamped = _g(v[k].row(t));
            x = _F(x, v_clamped);

            // add stage cost
            S(k) += _c(x) + param_gamma * u.row(t) * Sigma.inverse() * v[k].row(t).transpose();
        }
        // add terminal cost
        S(k) += _phi(x);
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

    // update privious control input sequence (shift 1 step to the left)
    u_prev.block(0, 0, T - 1, dim_u) = u.block(1, 0, T - 1, dim_u);
    u_prev.row(T - 1) = u.row(T - 1); // Repeat last control input

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

// Finds the nearest waypoint on the reference path
Vector4d MPPIController::_get_nearest_waypoint(double x, double y, bool update_prev_idx) {
    
    const int SEARCH_IDX_LEN = 200; //[points] forward search range
    int end_idx = std::min(static_cast<int>(ref_path.rows()), prev_waypoints_idx + SEARCH_IDX_LEN);

    MatrixXd search_segment = ref_path.block(prev_waypoints_idx, 0, end_idx - prev_waypoints_idx, 2); 

    // Calculate distances to waypoints in the search segment
    Vector2d current_pos(x, y);
    MatrixXd diffs = search_segment.rowwise() - current_pos.transpose();
    
    // Compute squared norms
    VectorXd dist_sq = diffs.rowwise().squaredNorm();

    VectorXd::Index min_idx;
    dist_sq.minCoeff(&min_idx);
    
    int nearest_idx = min_idx + prev_waypoints_idx;
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