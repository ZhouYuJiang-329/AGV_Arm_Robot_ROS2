#include <fstream>
#include <sstream>
#include <cmath>
#include "../../robot_move/test/seer_test_support.hpp"
#include "lidar/impl/seer/seer_lidar_source.hpp"
#include "lidar/processing/laser_scan_builder.hpp"

using lidar::ReadStatus;
using lidar::seer::SeerLidarSource;

template<typename Check>
void query(const std::string& body, Check check)
{
    withServer([&](int fd)
    {
        auto request = readRequest(fd);
        require(request.size() == 16 && request[8] == 3 && request[9] == 0xF1,
                "source must send zero-body 1009");
        auto frame = response(request, body);
        // Deliberately fragmented header and body.
        writeAll(fd, frame.data(), 5);
        writeAll(fd, frame.data() + 5, frame.size() - 5);
    }, [&](int& fd)
    {
        auto transport = std::make_unique<Transport>();
        TransportTestAccess::statusSocket(*transport) = fd;
        fd = -1;  // source owns this descriptor
        SeerLidarSource source("127.0.0.1", std::move(transport));
        check(source.readBatch(lidar::Clock::now() + std::chrono::milliseconds(200)));
    });
}

int main()
{
    try
    {
        std::ifstream input(std::string(FIXTURE_DIR) + "/seer_lasers.json");
        require(input.good(), "missing fixture");
        std::ostringstream contents;
        contents << input.rdbuf();
        query(contents.str(), [](const auto& result)
        {
            require(result.status == ReadStatus::Ok && result.batch.scans.size() == 2,
                    "multi-device fixture rejected");
            const auto& front = result.batch.scans[0];
            require(front.name == "laser_front" && front.id == 3 &&
                    front.source_stamp == 9007199254740993ULL,
                    "device identity or uint64 timestamp lost");
            require(!front.beams[0].angle && !front.beams[0].distance &&
                    !front.beams[0].valid, "missing fields replaced with defaults");
            require(front.beams[4].valid.value_or(false) &&
                    std::abs(*front.beams[4].angle / lidar::kRadiansPerDegree + 124.6) < 1e-8,
                    "degree conversion failed");
            require(front.install.upside == true && front.install.yaw == -47,
                    "raw install info missing");
            const auto scan = lidar::buildLaserScan(front, lidar::ScanConfig{"front"});
            require(scan.valid_points == 2 && scan.rejected_points == 6,
                    "real fixture conversion dropped/created measurements");
        });
        // Exercise public connect -> Transport TCP -> API parse without control servers.
        {
            int listener = socket(AF_INET, SOCK_STREAM, 0);
            require(listener >= 0, "TCP listener failed");
            int reuse = 1;
            setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            address.sin_port = htons(19204);
            require(bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0 &&
                    listen(listener, 2) == 0, "test requires loopback port 19204");
            timeval timeout{2, 0};
            setsockopt(listener, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            std::exception_ptr server_error;
            std::thread peer([&]
            {
                int fd = -1;
                try
                {
                    fd = accept(listener, nullptr, nullptr);
                    require(fd >= 0, "TCP accept failed");
                    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
                    // API error leaves frame alignment intact for a subsequent request.
                    for (const auto& body : {std::string(R"({"ret_code":7})"), contents.str()})
                    {
                        auto req = readRequest(fd);
                        auto frame = response(req, body);
                        writeAll(fd, frame.data(), frame.size());
                    }
                }
                catch (...) { server_error = std::current_exception(); }
                if (fd >= 0) close(fd);
            });
            bool connected, api_error = false, parsed = false;
            {
                SeerLidarSource source("127.0.0.1");
                connected = source.connect();
                if (connected)
                {
                    api_error = source.readBatch(lidar::Clock::now() + std::chrono::seconds(1)).status ==
                                ReadStatus::ApiError;
                    parsed = source.readBatch(lidar::Clock::now() + std::chrono::seconds(1)).batch.scans.size() == 2;
                }
            }
            peer.join();
            close(listener);
            if (server_error) std::rethrow_exception(server_error);
            require(connected && api_error && parsed, "TCP source connection/error recovery failed");
        }
        query(R"({"lasers":[]})", [](const auto& r)
        { require(r.status == ReadStatus::Ok && r.batch.scans.empty(), "empty batch rejected"); });
        query(R"({"ret_code":7,"err_msg":"unsupported"})", [](const auto& r)
        { require(r.status == ReadStatus::ApiError, "API failure not classified"); });
        for (const auto& body : {"", "{", "{} extra", "[]", "{}",
             "{\"ret_code\":\"0\",\"lasers\":[]}", "{\"lasers\":[3]}"})
            query(body, [](const auto& r)
            { require(r.status == ReadStatus::ProtocolError, "bad response accepted"); });
        // Wrong response type is a protocol error, not a timeout or API error.
        withServer([](int fd)
        {
            auto bytes = response(readRequest(fd), R"({"lasers":[]})");
            bytes[9] ^= 1;
            writeAll(fd, bytes.data(), bytes.size());
        }, [](int& fd)
        {
            auto transport = std::make_unique<Transport>();
            TransportTestAccess::statusSocket(*transport) = fd;
            fd = -1;
            SeerLidarSource source("127.0.0.1", std::move(transport));
            require(source.readBatch(lidar::Clock::now() + std::chrono::seconds(1)).status ==
                    ReadStatus::ProtocolError, "wrong frame error classification");
        });
        query(R"({"lasers":[{"device_info":{"device_name":"one","min_range":0,"max_range":10},
              "header":{"data_nsec":"-1"},"beams":[{"angle":0,"dist":1,"valid":true}]}]})",
            [](const auto& r)
        {
            require(r.status == ReadStatus::Ok && !r.batch.scans[0].source_stamp &&
                    r.batch.scans[0].beams[0].angle == 0, "timestamp or zero-angle handling");
        });
        // A delayed server must respect the single request deadline.
        withServer([](int fd)
        {
            readRequest(fd);
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
        }, [](int& fd)
        {
            auto transport = std::make_unique<Transport>();
            TransportTestAccess::statusSocket(*transport) = fd;
            fd = -1;
            SeerLidarSource source("127.0.0.1", std::move(transport));
            const auto r = source.readBatch(lidar::Clock::now() + std::chrono::milliseconds(20));
            require(r.status == ReadStatus::Timeout, "timeout not classified");
        });
        SeerLidarSource disconnected("127.0.0.1");
        require(disconnected.readBatch(lidar::Clock::now() + std::chrono::seconds(1)).status ==
                ReadStatus::ConnectionError, "disconnected source accepted");
        std::cout << "seer_lidar_source_test passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
