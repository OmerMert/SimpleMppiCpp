#ifndef UDP_H
#define UDP_H

#pragma once

#define WIN32_LEAN_AND_MEAN
#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>

// Data packet structure to send via UDP
struct SimDataPacket {
    double time;
    double x;
    double y;
    double yaw;
    double v;
    double steer;
    double accel;
};
bool setupUDPSender(SOCKET& outSocket, sockaddr_in& outDestAddr);
void sendUDPData(SOCKET sock, const sockaddr_in& destAddr, const SimDataPacket& packet);
void cleanupUDPSender(SOCKET sock);

#endif // UDP_H