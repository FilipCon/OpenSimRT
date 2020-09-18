#pragma comment(lib, "Ws2_32.lib")
#pragma once

// #include <WS2tcpip.h>
#include <iostream>
#include <string>
#include <system_error>
#include <winsock2.h>

class NetworkInitializer {
 public:
    NetworkInitializer() {
        int ret = WSAStartup(MAKEWORD(2, 2), &data);
        if (ret != 0)
            throw std::system_error(WSAGetLastError(), std::system_category(),
                                    "WSAStartup Failed");
    }
    ~NetworkInitializer() { WSACleanup(); }

 private:
    WSAData data;
};

class UDPSocket {
 public:
    UDPSocket() {
        m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO::IPPROTO_UDP);
        if (m_socket == INVALID_SOCKET)
            throw std::system_error(WSAGetLastError(), std::system_category(),
                                    "Error opening socket");
    }
    ~UDPSocket() { closesocket(m_socket); }

    sockaddr_in RecvFrom(char* buffer, int len, int flags = 0) {
        sockaddr_in from;
        int size = sizeof(from);
        if (recvfrom(m_socket, buffer, len, flags,
                     reinterpret_cast<SOCKADDR*>(&from), &size) < 0) {
            throw std::system_error(WSAGetLastError(), std::system_category(),
                                    "recvfrom failed");
        }
        return from;
    }
    void Bind(unsigned short port) {
        sockaddr_in add;
        add.sin_family = AF_INET;
        add.sin_addr.s_addr = htonl(INADDR_ANY);
        add.sin_port = htons(port);

        if (bind(m_socket, reinterpret_cast<SOCKADDR*>(&add), sizeof(add))) {
            throw std::system_error(WSAGetLastError(), std::system_category(),
                                    "Bind failed");
        }
    }

 private:
    NetworkInitializer m_networkInitializer;
    SOCKET m_socket;
};