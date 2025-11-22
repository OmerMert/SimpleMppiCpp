#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "Eigen/Dense"

#include "Vehicle.h"
#include "MPPIController.h"
#include "UDP.h"


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
    double delta_t = 0.05; // [sec]
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

    // initialize a vehicle as a control target
    Vehicle vehicle(
        2.5,   // wheel_base
        0.523, // max_steer_abs [rad]
        2.000  // max_accel_abs [m/s^2]
    );

    vehicle.reset(Vector4d(0.0, 0.0, 0.0, 0.0)); // init_state [x[m], y[m], yaw[rad], v[m/s]]

    // initialize a mppi controller for the vehicle
    MPPIController mppi(
        delta_t * 2.0, // delta_t [s]
        2.5,           // wheel_base [m]
        0.523,         // max_steer_abs [rad]
        2.000,         // max_accel_abs [m/s^2]
        ref_path,      // ref_path, size is <num_of_waypoints x 2>
        20,            // horizon_step_T [steps]
        100,           // number_of_samples_K [samples]
        0.0,           // param_exploration
        100.0,         // param_lambda
        0.98,          // param_alpha
        (Matrix2d() << 0.075, 0.0, 0.0, 2.0).finished(), // sigma
        Vector4d(50.0, 50.0, 1.0, 20.0), // stage_cost_weight [x, y, yaw, v]
        Vector4d(50.0, 50.0, 1.0, 20.0)  // terminal_cost_weight [x, y, yaw, v]
    );


    //simulation loop
    SimDataPacket packet;

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