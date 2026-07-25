#ifndef UDP_H
#define UDP_H

#pragma once

#define WIN32_LEAN_AND_MEAN
#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdint>

// ---- Packet structures (must be byte-identical to the Python side) ----
// We use pragma pack(1) to guarantee the byte layout.
#pragma pack(push, 1)

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

// C++ -> Python: send a control command to BeamNG
void sendControlPacket(SOCKET sock, const sockaddr_in& destAddr,
                       const ControlPacket& pkt);

// Blocking wait for StatePacket. timeout_ms=0 -> wait indefinitely.
// Returns: true = packet received, false = timeout
bool receiveStatePacket(SOCKET sock, StatePacket& outPkt, int timeout_ms);

void cleanupUDPSender(SOCKET sock);

#endif // UDP_H
