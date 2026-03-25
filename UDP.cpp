#include "UDP.h"

bool setupUDPSender(SOCKET& outSocket, sockaddr_in& outDestAddr, int listenPort ,int sendPort) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "[ERROR] WSAStartup failed." << std::endl;
        return false;
    }

    outSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (outSocket == INVALID_SOCKET) {
        std::cerr << "[ERROR] UDP socket cannot created: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return false;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(listenPort);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    bind(outSocket, (SOCKADDR*)&serverAddr, sizeof(serverAddr));

    outDestAddr.sin_family = AF_INET;
    outDestAddr.sin_port = htons(sendPort);
    inet_pton(AF_INET, "127.0.0.1", &outDestAddr.sin_addr);


    return true;
}

void sendUDPData(SOCKET sock, const sockaddr_in& destAddr, const SimDataPacket& packet) {
    sendto(sock, (const char*)&packet, sizeof(SimDataPacket), 0,
           (SOCKADDR*)&destAddr, sizeof(destAddr));
}

void cleanupUDPSender(SOCKET sock) {
    closesocket(sock);
    WSACleanup();
}