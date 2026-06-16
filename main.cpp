#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "Eigen/Dense"

#include "Vehicle.h"
#include "MPPIController.h"
#include "CostMapGenerator.h"
#include "UDP.h"
#include <chrono>

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

//CBF Parameters
float influence_radius;
float cbf_weight;
float decay_rate;

std::string ref_path_filepath;
// Cost Map Parameters
double costmap_resolution;
double costmap_margin;
double costmap_gradient_margin;
std::string costmap_filepath;

Matrix2d sigma;
Vector4d stage_cost_weight;
Vector4d terminal_cost_weight;

void Simulate(MPPIController& mppi, Vehicle& vehicle, const std::string& mode, SOCKET serverSocket, SOCKADDR_IN destAddr, double& total_reward);

// Isaac mode: Vehicle class is not used; state comes from UDP
void SimulateIsaac(MPPIController& mppi, SOCKET serverSocket, SOCKADDR_IN destAddr);

// Parse the weights from the received UDP message
void parse_weights(const std::string& msg, double& wx, double& wy, double& wyaw, double& wv) {
    std::stringstream ss(msg);
    std::string item;
    std::getline(ss, item, ','); wx = std::stod(item);
    std::getline(ss, item, ','); wy = std::stod(item);
    std::getline(ss, item, ','); wyaw = std::stod(item);
    std::getline(ss, item, ','); wv = std::stod(item);
}

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

    influence_radius = cfg["CBF_PARAMETERS"]["influence_radius"];
    cbf_weight       = cfg["CBF_PARAMETERS"]["cbf_weight"];
    decay_rate       = cfg["CBF_PARAMETERS"]["decay_rate"];

    ref_path_filepath = cfg["REF_PATH_FILE"];

        // Cost Map config
    costmap_resolution       = cfg["COSTMAP_RESOLUTION"];
    costmap_margin           = cfg["COSTMAP_MARGIN"];
    costmap_gradient_margin  = cfg["COSTMAP_GRADIENT_MARGIN"];
    costmap_filepath         = cfg["COSTMAP_FILE"];
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

    // IMPORTANT: If the path yaw column (index 2) is in [0, 2pi], convert it to
    // [-pi, pi] to be consistent with the MPPI state yaw (atan2 output).
    // The GPU-side normalize_angle_diff expects this range.
    if (matrix.cols() >= 3) {
        const double PI_D = 3.14159265358979323846;
        for (int i = 0; i < matrix.rows(); ++i) {
            double y = matrix(i, 2);
            // [0, 2pi] -> [-pi, pi]
            while (y > PI_D)  y -= 2.0 * PI_D;
            while (y < -PI_D) y += 2.0 * PI_D;
            matrix(i, 2) = y;
        }
    }

    return matrix;
}

void computeMapBounds(const MatrixXd& ref_path, double margin,
                      double& x_min, double& x_max, double& y_min, double& y_max) {
    x_min = ref_path.col(0).minCoeff() - margin;
    x_max = ref_path.col(0).maxCoeff() + margin;
    y_min = ref_path.col(1).minCoeff() - margin;
    y_max = ref_path.col(1).maxCoeff() + margin;
}

int main(int argc, char* argv[]) {
    std::cout << "[INFO] Starting C++ MPPI Path Tracking Simulation" << std::endl;
    std::string mode = "normal"; // default mode

    //--- UDP Setup ---
    SOCKET serverSocket;
    sockaddr_in destAddr, clientAddr;

    int cpp_listen_port = 5005;
    int py_send_port = 5006;

    if (argc >= 4) {
        cpp_listen_port = std::stoi(argv[1]);
        py_send_port = std::stoi(argv[2]);
        mode = std::string(argv[3]);
    }

    if (!setupUDPSender(serverSocket, destAddr, cpp_listen_port, py_send_port)) {
        return 1;
    }
    std::cout << "[INFO] UDP sender 127.0.0.1:" << cpp_listen_port << " -> 127.0.0.1:" << py_send_port << " is set." << std::endl;
    

    if(mode == "train") {
        std::cout << "[INFO] C++ is running in TRAINING MODE." << std::endl;
    } else if (mode == "isaac") {
        std::cout << "[INFO] C++ is running in ISAAC MODE." << std::endl;
    } else {
        std::cout << "[INFO] C++ is running in NORMAL MODE." << std::endl;
    }

    // --- read config file ---
    ReadConfig();

    // --- load the reference path ---
    MatrixXd ref_path;
    try {
        ref_path = loadRefPath(ref_path_filepath);
        std::cout << "[INFO] Reference path loaded from: " << ref_path_filepath
                  << " Size: " << ref_path.rows() << "x" << ref_path.cols() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    // --- Compute map bounds from reference path ---
    double map_x_min, map_x_max, map_y_min, map_y_max;
    computeMapBounds(ref_path, costmap_margin, map_x_min, map_x_max, map_y_min, map_y_max);

        // --- Load cost map from CSV file ---
    CostMapGenerator* costmap_gen_ptr = nullptr;
    try {
        costmap_gen_ptr = new CostMapGenerator(
            costmap_filepath,
            costmap_resolution,
            map_x_min, map_y_min,
            costmap_gradient_margin
        );
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] CostMap loading failed: " << e.what() << std::endl;
        std::cerr << "[ERROR] Make sure '" << costmap_filepath << "' exists!" << std::endl;
        cleanupUDPSender(serverSocket);
        return 1;
    }
    CostMapGenerator& costmap_gen = *costmap_gen_ptr;
 
    // Get extracted obstacles for MPPI (circles derived from grid)
    const auto& obstacles = costmap_gen.getObstacles();
 
    // Get cost map with gradients for RL
    const auto& global_costmap = costmap_gen.getCostMap();

    // initialize a vehicle as a control target
    Vehicle vehicle(
        wheel_base,   // wheel_base
        max_steer_abs, // max_steer_abs [rad]
        max_accel_abs // max_accel_abs [m/s^2]
    );

    vehicle.reset(Vector4d(0.0, 0.0, 0.0, 0.0)); // init_state [x[m], y[m], yaw[rad], v[m/s]]


    // Isaac mode runs at 20 Hz (0.05 s/tick) and must match delta_t.
    // Normal simulation mode used delta_t * 2.0; corrected for Isaac.
    double mppi_dt = (mode == "isaac") ? delta_t : delta_t * 2.0;

    // initialize a mppi controller for the vehicle
    MPPIController mppi(
        mppi_dt, // delta_t [s]
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
        obstacles, // obstacles
        influence_radius,
        cbf_weight,
        decay_rate
    );

    double total_reward = 0.0;

    if(mode == "normal")
    {
        Simulate(mppi, vehicle, mode, serverSocket, destAddr, total_reward);
    }
    else if (mode == "isaac")
    {
        SimulateIsaac(mppi, serverSocket, destAddr);
    }
    else // train mode
    {
        char recvBuffer[512];
        int clientAddrLen = sizeof(clientAddr);
        int train_step = 0;

        while(true)
        {
            int bytesReceived = recvfrom(serverSocket, recvBuffer, 512, 0, (SOCKADDR*)&clientAddr, &clientAddrLen);
            if (bytesReceived > 0) {
                recvBuffer[bytesReceived] = '\0';
                std::string msg(recvBuffer);
                
                double w_x, w_y, w_yaw, w_v;
                parse_weights(msg, w_x, w_y, w_yaw, w_v);

                mppi.set_weights(w_x, w_y, w_yaw, w_v);
                mppi.reset(); 
                vehicle.reset(Vector4d(0.0, 0.0, 0.0, 0.0)); 
                total_reward = 0.0;

                Simulate(mppi, vehicle, mode, serverSocket, destAddr, total_reward);

                // Send cost map + reward to Python
                sendTrainResponse(
                    serverSocket, destAddr,
                    global_costmap,
                    costmap_gen.getRows(), costmap_gen.getCols(),
                    (float)costmap_resolution,
                    (float)map_x_min, (float)map_y_min,
                    (float)total_reward
                );
                
                train_step++;
                std::cout << train_step << ":Computed weights: [" << w_x << ", " << w_y << ", " << w_yaw << ", " << w_v << "] " << "Total Reward: " << total_reward << std::endl;
            }
        }
    }

    if (costmap_gen_ptr) delete costmap_gen_ptr;
    closesocket(serverSocket);
    WSACleanup();
    return 0;
}


void Simulate(MPPIController& mppi, Vehicle& vehicle, const std::string& mode, SOCKET serverSocket, SOCKADDR_IN destAddr, double& total_reward) {

    int sim_steps = 800;
    
    bool crashed = false;
    SimDataPacket packet;

    //simulation loop (1 Episode)
    for (int i = 0; i < sim_steps; ++i) {


        // get current state of vehicle
        State current_state = vehicle.get_state();

        Control optimal_input;
        MatrixXd optimal_traj;

        try {
            // calculate input force with MPPI
            std::tie(optimal_input, optimal_traj) = mppi.calc_control_input(current_state);
            total_reward -= 1.0; // Step penalty
        } catch (const std::out_of_range& e) {
            total_reward += 5000.0; // Finish bonus
            break;
        }

        // update states of vehicle
        vehicle.update(optimal_input, delta_t);

        State new_state = vehicle.get_state();

        // REWARD FUNCTION
        Vector4d ref = mppi._get_nearest_waypoint(new_state[0], new_state[1]);
        double dist_to_path = std::sqrt(std::pow(new_state[0] - ref[0], 2) + std::pow(new_state[1] - ref[1], 2));
        
        total_reward -= dist_to_path / 5.0;
        total_reward += new_state[3]; 

        // Collision check
        if (mppi._is_collided(new_state) > 0.0) {
            total_reward -= 10000.0;
            crashed = true;
            break; 
        }

        double t = i * delta_t;
        if(mode == "normal") {
            // --- send UDP data ---
            packet.time = t;
            packet.x = current_state[0];
            packet.y = current_state[1];
            packet.yaw = current_state[2];
            packet.v = current_state[3];
            packet.steer = optimal_input[0];
            packet.accel = optimal_input[1];

            sendUDPData(serverSocket, destAddr, packet);
        }


    }


}


// ==========================================================================
// Isaac integration simulation loop
// ==========================================================================
// Flow:
//   1) Wait for StatePacket from Python bridge (real vehicle state)
//   2) Compute optimal control with MPPI
//   3) Send ControlPacket back to bridge
//   4) Repeat
// Note: the internal Vehicle class is NOT used in this mode - real physics run in Isaac
// ==========================================================================
void SimulateIsaac(MPPIController& mppi, SOCKET serverSocket, SOCKADDR_IN destAddr) {
    std::cout << "[Isaac] Waiting for first state from bridge..." << std::endl;

    StatePacket state_pkt;
    ControlPacket ctrl_pkt;

    // Block until first state arrives (waits while Isaac stabilizes)
    if (!receiveStatePacket(serverSocket, state_pkt, 0)) {
        std::cerr << "[Isaac] Failed to receive first state." << std::endl;
        return;
    }
    std::cout << "[Isaac] Connected. Starting control loop..." << std::endl;

    int step = 0;
    int max_steps = 5000; // safety limit

    while (step < max_steps) {
        // 1) State -> MPPI state vektoru
        State x0;
        x0 << state_pkt.x, state_pkt.y, state_pkt.yaw, state_pkt.v;

        if (!state_pkt.valid) {
            std::cout << "[Isaac] Bridge sent stop signal (valid=0). Exiting." << std::endl;
            break;
        }

        // 2) MPPI'yi cozdur
        Control u_opt;
        MatrixXd traj;
        try {
            std::tie(u_opt, traj) = mppi.calc_control_input(x0);
        } catch (const std::out_of_range&) {
            std::cout << "[Isaac] End of path - completed!" << std::endl;
            // reset=1: send "done" signal to Python bridge
            ctrl_pkt.time = state_pkt.time;
            ctrl_pkt.steer = 0.0;
            ctrl_pkt.accel = -max_accel_abs;
            ctrl_pkt.reset = 1;
            sendControlPacket(serverSocket, destAddr, ctrl_pkt);
            break;
        }

        // 3) ControlPacket hazirla ve gonder
        ctrl_pkt.time = state_pkt.time;
        ctrl_pkt.steer = u_opt[0];
        ctrl_pkt.accel = u_opt[1];
        ctrl_pkt.reset = 0;
        sendControlPacket(serverSocket, destAddr, ctrl_pkt);

        // Periyodik log
        if (step % 20 == 0) {
            std::cout << "[" << step << "] pos=(" << state_pkt.x << "," << state_pkt.y
                      << ") yaw=" << state_pkt.yaw
                      << " v=" << state_pkt.v
                      << " | u=(" << u_opt[0] << "," << u_opt[1] << ")"
                      << std::endl;
        }

        // 4) Wait for next state (5-second timeout)
        if (!receiveStatePacket(serverSocket, state_pkt, 5000)) {
            std::cerr << "[Isaac] State timeout - did the bridge stop?" << std::endl;
            break;
        }

        step++;
    }

    std::cout << "[Isaac] Loop finished. Total steps: " << step << std::endl;
}