#ifndef MPPI_CONTROLLER_H
#define MPPI_CONTROLLER_H

#pragma once

#include "Eigen/Dense"
#include <vector>
#include <tuple>
#include <random>

using namespace Eigen;

// State vector: [x, y, yaw, v]
using State = Vector4d; 
// Control vector: [steer, accel]
using Control = Vector2d; 

// Obstacle structure
struct Obstacle {
    double x;
    double y;
    double r;
};


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
);

class MPPIController {
public:
    MPPIController(
        double delta_t,
        double wheel_base,
        double max_steer_abs,
        double max_accel_abs,
        const MatrixXd& ref_path,
        int horizon_step_T,
        int number_of_samples_K,
        double param_exploration,
        double param_lambda,
        double param_alpha,
        const Matrix2d& sigma,
        const Vector4d& stage_cost_weight,
        const Vector4d& terminal_cost_weight,
        const std::vector<Obstacle>& obstacles
    );

    std::tuple<Control, MatrixXd> calc_control_input(const State& observed_x);

private:

    int dim_x = 4; // dimension of system state vector
    int dim_u = 2; // dimension of control input vector
    int T; // prediction horizon
    int K; // number of sample trajectories

    // MPPI parameters
    double param_exploration; //  constant parameter of mppi
    double param_lambda; //  constant parameter of mppi
    double param_alpha; //  constant parameter of mppi
    double param_gamma; //  constant parameter of mppi
    Matrix2d Sigma; // deviation of noise
    Vector4d stage_cost_weight;
    Vector4d terminal_cost_weight;
    std::vector<Obstacle> obstacles;
    double vehicle_width = 3.0;
    double vehicle_length = 4.0;
    double safety_margin_rate = 1.2;

    // Vehicle parameters
    double dt;
    double L;
    double max_steer;
    double max_accel;
    
    // Reference path
    MatrixXd ref_path; // N x 4 [x, y, yaw, v]
    int prev_waypoints_idx = 0;

    MatrixXd u_prev; // T x dim_u

    // Random number generator
    std::default_random_engine generator;
    Matrix2d cholesky_L; 
    std::normal_distribution<double> std_normal_dist{0.0, 1.0};

    // System dynamic
    State _F(const State& x_t, const Control& v_t) const;

    // Control limitation
    Control _g(const Control& v) const;

    // Stage cost
    double _c(const State& x_t);

    // Terminal cost
    double _phi(const State& x_T);

    // Returns: [ref_x, ref_y, ref_yaw, ref_v]
    Vector4d _get_nearest_waypoint(double x, double y, bool update_prev_idx = false);

    MatrixXd _calc_epsilon();

    VectorXd _compute_weights(const VectorXd& S) const;

    MatrixXd _moving_average_filter(const MatrixXd& xx, int window_size) const;

    double normalize_angle(double angle) const;

    double _is_collided(const State& x_t);
};

#endif // MPPI_CONTROLLER_H