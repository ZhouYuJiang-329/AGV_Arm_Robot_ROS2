#include "seer_test_support.hpp"
int main()
{
    try
    {
        {
            Transport t;
            const std::vector<uint8_t> expected{0x5A, 1,    0, 1, 0, 0, 0, 0,
                                                3,    0xEC, 0, 0, 0, 0, 0, 0};
            require(TransportTestAccess::pack(t, 1004, "") == expected,
                    "empty frame encoding differs");
            auto frame = TransportTestAccess::pack(t, 1020, R"({"simple":true})");
            require(frame.size() == 31 && frame[3] == 2 && frame[7] == 15 && frame[9] == 0xFC,
                    "JSON frame encoding differs");
            frame = TransportTestAccess::pack(t, 2010, std::string(256, 'x'));
            require(frame[6] == 1 && frame[7] == 0 && frame.size() == 272,
                    "body length byte order differs");
            TransportTestAccess::sequence(t, 65535);
            frame = TransportTestAccess::pack(t, 1004, "");
            require(frame[2] == 0 && frame[3] == 0, "sequence rollover differs");
        }
        // Validate nonblocking TCP connect and reconnect against loopback only.
        {
            int listener = socket(AF_INET, SOCK_STREAM, 0);
            require(listener >= 0, "listener creation failed");
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            require(bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
                    "loopback bind failed");
            socklen_t length = sizeof(address);
            require(getsockname(listener, reinterpret_cast<sockaddr*>(&address), &length) == 0,
                    "listener address missing");
            const int port = ntohs(address.sin_port);
            // A bound socket that is not listening must fail and clear the new fd.
            {
                Transport transport;
                int& fd = TransportTestAccess::controlSocket(transport);
                require(!TransportTestAccess::connect(transport, "127.0.0.1", port, fd) && fd == -1,
                        "refused TCP connection was accepted");
            }
            require(listen(listener, 2) == 0, "loopback listen failed");
            timeval timeout{2, 0};
            setsockopt(listener, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            std::exception_ptr error;
            std::thread peer(
                [&]
                {
                    try
                    {
                        for (int i = 0; i < 2; ++i)
                        {
                            const int fd = accept(listener, nullptr, nullptr);
                            require(fd >= 0, "loopback accept failed");
                            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
                            uint8_t byte;
                            const auto count = recv(fd, &byte, 1, 0);
                            close(fd);
                            require(count == 0,
                                    "reconnect/destruction did not release TCP connection");
                        }
                    }
                    catch (...)
                    {
                        error = std::current_exception();
                    }
                });
            bool connected = false;
            {
                Transport transport;
                int& fd = TransportTestAccess::controlSocket(transport);
                connected = TransportTestAccess::connect(transport, "127.0.0.1", port, fd) &&
                            TransportTestAccess::connect(transport, "127.0.0.1", port, fd);
            }
            peer.join();
            close(listener);
            require(connected, "loopback connect/reconnect failed");
            if (error)
                std::rethrow_exception(error);
        }

        // Consecutive frames in one write must not be consumed as one JSON body.
        withServer(
            [](int fd)
            {
                auto req = readRequest(fd);
                auto bytes = response(req, "{}");
                ++req[3];
                auto second = response(req, "{}");
                bytes.insert(bytes.end(), second.begin(), second.end());
                writeAll(fd, bytes.data(), bytes.size());
                readRequest(fd);
            },
            [](int& fd)
            {
                Transport transport;
                TransportLoan loan(transport, fd);
                std::string reply;
                require(TransportTestAccess::request(transport, fd, 1004, "", reply),
                        "first frame failed");
                require(TransportTestAccess::request(transport, fd, 1004, "", reply),
                        "coalesced frame lost");
            });

        for (int scenario = 0; scenario < 11; ++scenario)
        {
            if (scenario == 8 || scenario == 9)
                continue; // JSON validation belongs to Client.
            withServer(
                [scenario](int fd)
                {
                    auto bytes = response(readRequest(fd), "{}");
                    switch (scenario)
                    {
                    case 0:
                        bytes[0] = 0;
                        break;
                    case 1:
                        bytes[1] = 2;
                        break;
                    case 2:
                        bytes[3] ^= 1;
                        break;
                    case 3:
                        bytes[9] ^= 1;
                        break;
                    case 4:
                        bytes[4] = 1;
                        break; // Oversized body.
                    case 5:
                        bytes[14] = 1;
                        break; // Unsupported compression.
                    case 6:
                        bytes.resize(5);
                        break;
                    case 7:
                        bytes.resize(17);
                        break;
                    case 8:
                        bytes[16] = '[';
                        break;
                    case 9:
                        bytes[16] = '\0';
                        break;
                    case 10:
                        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
                        return;
                    }
                    writeAll(fd, bytes.data(), bytes.size());
                },
                [](int& fd)
                {
                    Transport transport;
                    TransportLoan loan(transport, fd);
                    std::string reply = "stale";
                    require(!TransportTestAccess::request(transport, fd, 1004, "", reply),
                            "bad response accepted");
                    require(fd == -1 && reply.empty(), "failed transport not invalidated");
                });
        }

        // A slow drip must not restart the deadline for every received byte.
        withServer(
            [](int fd)
            {
                const auto bytes = response(readRequest(fd), "{}");
                for (uint8_t byte : bytes)
                {
                    writeAll(fd, &byte, 1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            },
            [](int& fd)
            {
                Transport transport;
                TransportLoan loan(transport, fd);
                std::string reply;
                const auto start = std::chrono::steady_clock::now();
                require(!TransportTestAccess::request(transport, fd, 1004, "", reply),
                        "slow fragments bypassed transaction timeout");
                require(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1700),
                        "timeout was reset for each fragment");
            });

        // Concurrent calls share a connection: only one transaction may be in flight.
        withServer(
            [](int fd)
            {
                for (int i = 0; i < 2; ++i)
                {
                    const auto req = readRequest(fd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    uint8_t byte;
                    require(recv(fd, &byte, 1, MSG_PEEK | MSG_DONTWAIT) < 0,
                            "another request arrived before the current reply");
                    const auto bytes = response(req, "{}");
                    writeAll(fd, bytes.data(), bytes.size());
                }
            },
            [](int& fd)
            {
                Transport transport;
                TransportLoan loan(transport, fd);
                bool first = false, second = false;
                std::thread worker(
                    [&]
                    {
                        std::string reply;
                        first = TransportTestAccess::request(transport, fd, 1004, "", reply);
                    });
                std::string reply;
                second = TransportTestAccess::request(transport, fd, 1004, "", reply);
                worker.join();
                require(first && second, "concurrent requests failed");
            });

        // A small socket buffer forces partial sends while the peer drains it.
        withServer(
            [](int fd)
            {
                std::vector<uint8_t> bytes(2 * 1024 * 1024);
                readAll(fd, bytes.data(), bytes.size());
                require(bytes.front() == 0xA5 && bytes.back() == 0xA5,
                        "partial send corrupted data");
            },
            [](int& fd)
            {
                int size = 1024;
                setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
                Transport transport;
                TransportLoan loan(transport, fd);
                require(TransportTestAccess::sendAll(transport, fd,
                                                     std::vector<uint8_t>(2 * 1024 * 1024, 0xA5)),
                        "partial send was not completed");
            });
        std::cout << "PASS: Seer transport contract\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
