#ifndef UDP_H
#define UDP_H

#pragma once

#define WIN32_LEAN_AND_MEAN
#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdint>
#include <vector>

// ---- Packet structures (must be byte-identical to the Python side) ----
// We use pragma pack(1) to guarantee the byte layout.
#pragma pack(push, 1)

// Legacy: state sent from MPPI to the visualizer in normal mode
struct SimDataPacket {
    double time;
    double x;
    double y;
    double yaw;
    double v;
    double steer;
    double accel;
};

// C++ -> Python (control command sent to BeamNG)
// Python struct format: "dddi" (3 doubles + 1 int32) = 28 bytes
struct ControlPacket {
    double time;
    double steer;        // radians, [-max_steer, +max_steer]
    double accel;        // m/s^2, negative = braking request
    int32_t reset;       // 0 = continue, 1 = reset scenario
};

// Python -> C++ (real vehicle state from BeamNG)
// Python struct format: "dddddi" (5 doubles + 1 int32) = 44 bytes
struct StatePacket {
    double time;
    double x;
    double y;
    double yaw;          // radians, world frame
    double v;            // forward speed (m/s)
    int32_t valid;       // 0 = not ready yet, 1 = usable
};

#pragma pack(pop)

// ---- Functions ----

// Basic UDP setup
bool setupUDPSender(SOCKET& outSocket, sockaddr_in& outDestAddr,
                    int listenPort, int sendPort);

// Legacy API (for visualizer)
void sendUDPData(SOCKET sock, const sockaddr_in& destAddr,
                 const SimDataPacket& packet);

// BeamNG mode: bidirectional functions
void sendControlPacket(SOCKET sock, const sockaddr_in& destAddr,
                       const ControlPacket& pkt);

// Blocking wait for StatePacket. timeout_ms=0 -> wait indefinitely.
// Returns: true = packet received, false = timeout
bool receiveStatePacket(SOCKET sock, StatePacket& outPkt, int timeout_ms);

 
/**
 * @brief Send global cost map + reward to Python RL agent.
 * Binary layout: [rows:i32][cols:i32][res:f32][x_min:f32][y_min:f32][reward:f32][data:f32*rows*cols]
 */
void sendTrainResponse(SOCKET sock, const sockaddr_in& destAddr,
                       const std::vector<float>& costmap,
                       int rows, int cols,
                       float resolution, float x_min, float y_min,
                       float reward);
                       
void cleanupUDPSender(SOCKET sock);

#endif // UDP_H
