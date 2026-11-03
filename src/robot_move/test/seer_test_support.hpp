#ifndef ROBOT_MOVE_SEER_TEST_SUPPORT_HPP
#define ROBOT_MOVE_SEER_TEST_SUPPORT_HPP
#include "robot_move/impl/agv/seer/seer_transport.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
namespace robot_move::seer
{
struct TransportTestAccess
{
    static int& statusSocket(Transport& t)
    {
        return t.sockets_[0];
    }
    static int& controlSocket(Transport& t)
    {
        return t.sockets_[1];
    }
    static int& navigationSocket(Transport& t)
    {
        return t.sockets_[2];
    }
    static int& doSocket(Transport& t)
    {
        return t.sockets_[3];
    }
    static void release(Transport& t)
    {
        t.sockets_.fill(-1);
    }
    static bool connect(Transport& t, const std::string& ip, int port, int& fd)
    {
        return t.connectPort(ip, port, fd);
    }
    static bool sendAll(Transport& t, int fd, const std::vector<uint8_t>& data)
    {
        return t.sendAll(fd, data, std::chrono::steady_clock::now() + std::chrono::seconds(1));
    }
    static bool request(Transport& t, int& fd, uint16_t api, const std::string& body,
                        std::string& reply)
    {
        const bool ok = t.request(ApiChannel::Status, api, body, reply);
        if (t.sockets_[0] < 0)
            fd = -1;
        return ok;
    }
    static std::vector<uint8_t> pack(Transport& t, uint16_t api, const std::string& body)
    {
        return t.packFrame(api, body);
    }
    static void sequence(Transport& t, uint16_t value)
    {
        t.sequence_ = value;
    }
};
} // namespace robot_move::seer
using robot_move::seer::ApiChannel;
using robot_move::seer::Transport;
using robot_move::seer::TransportTestAccess;
struct TransportLoan
{
    Transport& transport;
    TransportLoan(Transport& t, int fd) : transport(t)
    {
        TransportTestAccess::statusSocket(t) = fd;
    }
    ~TransportLoan()
    {
        TransportTestAccess::release(transport);
    }
};
inline void require(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

inline void readAll(int fd, uint8_t* data, size_t size)
{
    size_t offset = 0;
    while (offset < size)
    {
        const ssize_t count = recv(fd, data + offset, size - offset, 0);
        require(count > 0, "mock server received an incomplete request");
        offset += static_cast<size_t>(count);
    }
}

inline std::vector<uint8_t> readRequest(int fd)
{
    std::vector<uint8_t> frame(16);
    readAll(fd, frame.data(), 16);
    const uint32_t length = (uint32_t(frame[4]) << 24) | (uint32_t(frame[5]) << 16) |
                            (uint32_t(frame[6]) << 8) | frame[7];
    require(length <= 4 * 1024 * 1024, "mock request length too large");
    frame.resize(16 + length);
    readAll(fd, frame.data() + 16, length);
    return frame;
}

inline std::vector<uint8_t> response(const std::vector<uint8_t>& request, const std::string& body)
{
    auto frame = request;
    frame.resize(16);
    const uint16_t api = ((uint16_t(frame[8]) << 8) | frame[9]) + 10000;
    frame[8] = api >> 8;
    frame[9] = api & 0xFF;
    for (size_t i = 0; i < 4; ++i)
        frame[4 + i] = (body.size() >> (24 - i * 8)) & 0xFF;
    frame.insert(frame.end(), body.begin(), body.end());
    return frame;
}

inline void writeAll(int fd, const uint8_t* data, size_t size)
{
    size_t offset = 0;
    while (offset < size)
    {
        const ssize_t count = send(fd, data + offset, size - offset, MSG_NOSIGNAL);
        // Clients intentionally close on bad headers and timeout in these tests.
        if (count <= 0)
            return;
        offset += static_cast<size_t>(count);
    }
}

inline void withServer(const std::function<void(int)>& server,
                       const std::function<void(int&)>& client)
{
    int sockets[2];
    require(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0, "socketpair failed");
    timeval timeout{3, 0};
    setsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sockets[1], SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    std::exception_ptr server_error;
    std::thread worker(
        [&]
        {
            try
            {
                server(sockets[1]);
            }
            catch (...)
            {
                server_error = std::current_exception();
            }
            close(sockets[1]);
        });
    std::exception_ptr client_error;
    try
    {
        client(sockets[0]);
    }
    catch (...)
    {
        client_error = std::current_exception();
    }
    if (sockets[0] >= 0)
        close(sockets[0]);
    worker.join();
    if (client_error)
        std::rethrow_exception(client_error);
    if (server_error)
        std::rethrow_exception(server_error);
}

#endif
