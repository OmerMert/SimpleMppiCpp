#include "UDP.h"
#include <cstring>

bool setupUDPSender(SOCKET& outSocket, sockaddr_in& outDestAddr,
                    int listenPort, int sendPort) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "[ERROR] WSAStartup failed." << std::endl;
        return false;
    }

    outSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (outSocket == INVALID_SOCKET) {
        std::cerr << "[ERROR] UDP socket cannot created: "
                  << WSAGetLastError() << std::endl;
        WSACleanup();
        return false;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(listenPort);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    if (bind(outSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "[ERROR] bind failed: " << WSAGetLastError() << std::endl;
        closesocket(outSocket);
        WSACleanup();
        return false;
    }

    outDestAddr.sin_family = AF_INET;
    outDestAddr.sin_port = htons(sendPort);
    inet_pton(AF_INET, "127.0.0.1", &outDestAddr.sin_addr);

    return true;
}

void sendControlPacket(SOCKET sock, const sockaddr_in& destAddr,
                       const ControlPacket& pkt) {
    sendto(sock, (const char*)&pkt, sizeof(ControlPacket), 0,
           (SOCKADDR*)&destAddr, sizeof(destAddr));
}

bool receiveStatePacket(SOCKET sock, StatePacket& outPkt, int timeout_ms) {
    // Set timeout
    if (timeout_ms > 0) {
        DWORD tv = (DWORD)timeout_ms;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                   (const char*)&tv, sizeof(tv));
    } else {
        // timeout_ms=0 -> blocking mode (wait indefinitely)
        DWORD tv = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                   (const char*)&tv, sizeof(tv));
    }

    sockaddr_in srcAddr;
    int srcLen = sizeof(srcAddr);
    int n = recvfrom(sock, (char*)&outPkt, sizeof(StatePacket), 0,
                     (SOCKADDR*)&srcAddr, &srcLen);

    if (n == SOCKET_ERROR) {
        int err = WSAGetLastError();
        if (err == WSAETIMEDOUT) {
            return false; // timeout, expected condition
        }
        std::cerr << "[WARN] recvfrom error: " << err << std::endl;
        return false;
    }

    if (n != sizeof(StatePacket)) {
        std::cerr << "[WARN] Unexpected packet size: " << n
                  << " (expected " << sizeof(StatePacket) << ")" << std::endl;
        return false;
    }

    return true;
}

void cleanupUDPSender(SOCKET sock) {
    closesocket(sock);
    WSACleanup();
}
