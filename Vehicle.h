#ifndef VEHICLE_H
#define VEHICLE_H

#pragma once

#include "Eigen/Dense"
#include <vector>
#include <cmath>
#include <algorithm>

using namespace Eigen;

// State vector: [x, y, yaw, v]
using State = Vector4d; 
// Control vector: [steer, accel]
using Control = Vector2d; 

class Vehicle {
public:
    /**
     * @brief 
     * @param wheel_base 
     * @param max_steer_abs
     * @param max_accel_abs
     */
    Vehicle(double wheel_base, double max_steer_abs, double max_accel_abs);

    /**
     * @brief
     * @param init_state
     */
    void reset(const State& init_state);

    /**
     * @brief
     * @param u 
     * @param delta_t 
     */
    void update(const Control& u, double delta_t);

    /**
     * @brief
     * @return 
     */
    State get_state() const;

private:
    // Vehicle parameters
    double L;           // [m]
    double max_steer;   // [rad]
    double max_accel;   // [m/s^2]

    // Current state
    State state; // [x, y, yaw, v]
};

#endif // VEHICLE_H