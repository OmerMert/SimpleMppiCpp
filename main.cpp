#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "Eigen/Dense"

#include "Vehicle.h"
#include "MPPIController.h"
#include "UDP.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

double delta_t;
double wheel_base;
double max_steer_abs;
double max_accel_abs;
int horizon_step_T;
int number_of_samples_K;
double param_exploration;
double param_lambda;
double param_alpha;

Matrix2d sigma;
Vector4d stage_cost_weight;
Vector4d terminal_cost_weight;
std::vector<Obstacle> defined_obstacles;

void ReadConfig() {

    std::ifstream f("config.json");
    json cfg = json::parse(f);

    delta_t        = cfg["delta_t"];
    wheel_base     = cfg["wheel_base"];
    max_steer_abs  = cfg["max_steer_abs"];
    max_accel_abs  = cfg["max_accel_abs"];

    horizon_step_T     = cfg["horizon_step_T"];
    number_of_samples_K = cfg["number_of_samples_K"];

    param_exploration = cfg["param_exploration"];
    param_lambda      = cfg["param_lambda"];
    param_alpha       = cfg["param_alpha"];

    sigma << cfg["sigma"][0][0], cfg["sigma"][0][1],
             cfg["sigma"][1][0], cfg["sigma"][1][1];

    stage_cost_weight << cfg["stage_cost_weight"][0],
                         cfg["stage_cost_weight"][1],
                         cfg["stage_cost_weight"][2],
                         cfg["stage_cost_weight"][3];

    terminal_cost_weight << cfg["terminal_cost_weight"][0],
                            cfg["terminal_cost_weight"][1],
                            cfg["terminal_cost_weight"][2],
                            cfg["terminal_cost_weight"][3];

    for (auto& ob : cfg["OBSTACLES"]) {
        Obstacle o;
        o.x = ob[0];
        o.y = ob[1];
        o.r = ob[2];
        defined_obstacles.push_back(o);
    }
}


//Read CSV File
MatrixXd loadRefPath(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("can not open CSV file: " + filepath);
    }

    std::vector<std::vector<double>> data;
    std::string line;

    std::getline(file, line);

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }
        data.push_back(row);
    }
    file.close();

    if (data.empty()) {
        throw std::runtime_error("Cannot read or empty CSV.");
    }

    // Transfer data to MatrixXd
    MatrixXd matrix(data.size(), data[0].size());
    for (size_t i = 0; i < data.size(); ++i) {
        for (size_t j = 0; j < data[i].size(); ++j) {
            matrix(i, j) = data[i][j];
        }
    }
    return matrix;
}

int main() {
    std::cout << "[INFO] Starting C++ MPPI Path Tracking Simulation" << std::endl;

    // --- UDP Sender Setup ---
    SOCKET udpSocket;
    sockaddr_in destAddr;
    if (!setupUDPSender(udpSocket, destAddr)) {
        return 1;
    }
    std::cout << "[INFO] UDP sender 127.0.0.1:5005 is set." << std::endl;

    // --- Simulation settings ---
    int sim_steps = 800;  // [steps]
    std::cout << "[INFO] delta_t : " << delta_t << "[s] , sim_steps : " << sim_steps << "[steps], total_sim_time : " << delta_t * sim_steps << "[s]" << std::endl;

    // --- load the reference path ---
    MatrixXd ref_path;
    try {
        ref_path = loadRefPath("data/ovalpath.csv");
        std::cout << "[INFO] Referans path is loaded. Size: " << ref_path.rows() << "x" << ref_path.cols() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    // --- read config file ---
    ReadConfig();

    // initialize a vehicle as a control target
    Vehicle vehicle(
        wheel_base,   // wheel_base
        max_steer_abs, // max_steer_abs [rad]
        max_accel_abs // max_accel_abs [m/s^2]
    );

    vehicle.reset(Vector4d(0.0, 0.0, 0.0, 0.0)); // init_state [x[m], y[m], yaw[rad], v[m/s]]


    // initialize a mppi controller for the vehicle
    MPPIController mppi(
        delta_t * 2.0, // delta_t [s]
        wheel_base,           // wheel_base [m]
        max_steer_abs,         // max_steer_abs [rad]
        max_accel_abs,         // max_accel_abs [m/s^2]
        ref_path,      // ref_path, size is <num_of_waypoints x 2>
        horizon_step_T,            // horizon_step_T [steps]
        number_of_samples_K,           // number_of_samples_K [samples]
        param_exploration,           // param_exploration
        param_lambda,         // param_lambda
        param_alpha,          // param_alpha
        sigma, // sigma
        stage_cost_weight, // stage_cost_weight [x, y, yaw, v]
        terminal_cost_weight,  // terminal_cost_weight [x, y, yaw, v]
        defined_obstacles // obstacles
    );

    SimDataPacket packet;

    //simulation loop
    for (int i = 0; i < sim_steps; ++i) {

        // get current state of vehicle
        State current_state = vehicle.get_state();

        Control optimal_input;
        MatrixXd optimal_traj;

        try {
            // calculate input force with MPPI
            std::tie(optimal_input, optimal_traj) = mppi.calc_control_input(current_state);
        } catch (const std::out_of_range& e) {
            // the vehicle has reached the end of the reference path
            std::cout << "[ERROR] IndexError detected. Terminate simulation." << std::endl;
            break;
        }

        // print current state and input force
        double t = i * delta_t;
        printf("Time: %5.2f[s], x=%+7.3f[m], y=%+7.3f[m], yaw=%+7.3f[rad], v=%+7.3f[m/s], steer=%+6.2f[rad], accel=%+6.2f[m/s]\n",
               t, current_state[0], current_state[1], current_state[2], current_state[3], optimal_input[0], optimal_input[1]);

        // update states of vehicle
        vehicle.update(optimal_input, delta_t);

        // --- send UDP data ---
        packet.time = t;
        packet.x = current_state[0];
        packet.y = current_state[1];
        packet.yaw = current_state[2];
        packet.v = current_state[3];
        packet.steer = optimal_input[0];
        packet.accel = optimal_input[1];

        sendUDPData(udpSocket, destAddr, packet);


    }

    cleanupUDPSender(udpSocket);
    std::cout << "[INFO] Simulation is done." << std::endl;

    return 0;
}