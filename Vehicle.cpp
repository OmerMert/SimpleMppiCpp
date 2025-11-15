#include "Vehicle.h"

Vehicle::Vehicle(double wheel_base, double max_steer_abs, double max_accel_abs)
    : L(wheel_base), max_steer(max_steer_abs), max_accel(max_accel_abs) {
    state = State::Zero();
}

void Vehicle::reset(const State& init_state) {
    state = init_state;
}

State Vehicle::get_state() const {
    return state; 
}

void Vehicle::update(const Control& u, double delta_t) {

    // keep previous states
    double x = state[0];
    double y = state[1];
    double yaw = state[2];
    double v = state[3];

    // limit control inputs
    double steer = std::clamp(u[0], -max_steer, max_steer);
    double accel = std::clamp(u[1], -max_accel, max_accel);

    // update state variables
    state[0] = x + v * std::cos(yaw) * delta_t; //new x
    state[1] = y + v * std::sin(yaw) * delta_t; //new y
    state[2] = yaw + v / L * std::tan(steer) * delta_t; //new yaw
    state[3] = v + accel * delta_t; //new v
}