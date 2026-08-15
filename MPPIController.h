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

struct Obstacle {
    float x;
    float y;
    float r;
};


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
    float influence_radius_param, float cbf_weight_param, float decay_rate_param,
    const float* h_obstacle_costmap, int obs_rows, int obs_cols,
    float obs_res, float obs_x_min, float obs_y_min, float obstacle_costmap_weight
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
        const std::vector<Obstacle>& obstacles,
        const float influence_radius,
        const float cbf_weight,
        const float decay_rate
    );

    std::tuple<Control, MatrixXd> calc_control_input(const State& observed_x);

    // Returns: [ref_x, ref_y, ref_yaw, ref_v]
    Vector4d _get_nearest_waypoint(double x, double y, bool update_prev_idx = false);

    // Vehicle body box used by the collision check and the CBF. Read from config.json
    // VEHICLE_FOOTPRINT, so this MPPI and every competitor wrapper drive the same car.
    void set_vehicle_footprint(double width, double length, double safety_margin);

    // Obstacle costmap: a soft cost added on top of the analytic CBF, which it does not
    // replace; the footprint and collision logic in mppi_core.cu is untouched. Leaving
    // this uncalled, or passing weight <= 0, disables the term.
    void set_obstacle_costmap(const std::vector<float>& data, int rows, int cols,
                              float resolution, float x_min, float y_min, float weight);
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
    // Vehicle body box (etk800). These defaults are only a fallback; main.cpp overwrites
    // them from config.json VEHICLE_FOOTPRINT via set_vehicle_footprint(), which is also
    // what the competitor wrappers read, so every controller collides with the same car.
    double vehicle_width = 1.9;
    double vehicle_length = 4.5;
    double safety_margin_rate = 1.0;

    // Vehicle parameters
    double dt;
    double L;
    double max_steer;
    double max_accel;

    // CBF parameters
    float influence_radius;
    float cbf_weight;
    float decay_rate;

    // Obstacle costmap, additive to the CBF; empty or weight 0 means disabled
    std::vector<float> obstacle_costmap_data;
    int   obs_rows = 0, obs_cols = 0;
    float obs_res = 1.0f, obs_x_min = 0.0f, obs_y_min = 0.0f;
    float obstacle_costmap_weight = 0.0f;

    // Reference path
    MatrixXd ref_path; // N x 4 [x, y, yaw, v]
    int prev_waypoints_idx = 0;

    MatrixXd u_prev; // T x dim_u

    // Random number generator
    std::default_random_engine generator;
    Matrix2d cholesky_L; 
    std::normal_distribution<double> std_normal_dist{0.0, 1.0};

    // System dynamics (kinematic bicycle) used to roll out the optimal trajectory
    State _F(const State& x_t, const Control& v_t) const;

    // Clamp a control input to the actuator limits
    Control _g(const Control& v) const;

    // Draw the K x T noise samples ~ N(0, Sigma)
    MatrixXd _calc_epsilon();

    // Information-theoretic sample weights from the rollout costs
    VectorXd _compute_weights(const VectorXd& S) const;

    // Temporal smoothing of the control update
    MatrixXd _moving_average_filter(const MatrixXd& xx, int window_size) const;
};

#endif // MPPI_CONTROLLER_H