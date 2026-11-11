#include <atomic>
#include <iostream>
#include "../../robot_move/test/seer_test_support.hpp"
#include "lidar/nodes/lidar_node.hpp"

using namespace std::chrono_literals;

/// Loopback-only controller: closes its first connection after three replies.
class Controller
{
public:
    Controller()
    {
        listener_ = socket(AF_INET, SOCK_STREAM, 0);
        require(listener_ >= 0, "listener failed");
        int reuse = 1;
        setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(19204);
        if (bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) ||
            listen(listener_, 2))
        {
            close(listener_);
            throw std::runtime_error("test requires loopback port 19204");
        }
        timeval timeout{0, 200000};
        setsockopt(listener_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        thread_ = std::thread([this] { run(); });
    }
    ~Controller()
    {
        stop_ = true;
        thread_.join();
        close(listener_);
    }
    std::atomic<size_t> connections{0};
    std::atomic<bool> protocol_ok{true};

private:
    void run()
    {
        uint64_t stamp = 1;
        while (!stop_)
        {
            int peer = accept(listener_, nullptr, nullptr);
            if (peer < 0) continue;
            const auto session = ++connections;
            timeval timeout{0, 200000};
            setsockopt(peer, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            try
            {
                for (size_t frame = 0; !stop_; ++frame)
                {
                    auto request = readRequest(peer);
                    if (request.size() != 16 || request[8] != 3 || request[9] != 0xF1)
                    {
                        protocol_ok = false;
                        break;
                    }
                    std::string devices;
                    for (const auto* name : {"laser_front", "laser_back"})
                    {
                        if (!devices.empty()) devices += ',';
                        devices += std::string("{\"device_info\":{\"device_name\":\"") + name +
                            "\",\"min_range\":0.1,\"max_range\":40,\"scan_freq\":30}," +
                            "\"header\":{\"data_nsec\":\"" + std::to_string(stamp++) +
                            "\"},\"beams\":[{\"angle\":0,\"dist\":2,\"valid\":true}]}";
                    }
                    const auto bytes = response(request, "{\"lasers\":[" + devices + "]}");
                    writeAll(peer, bytes.data(), 8);
                    writeAll(peer, bytes.data() + 8, bytes.size() - 8);
                    if (session == 1 && frame == 2) break;
                }
            }
            catch (const std::exception&) { /* EOF or read timeout ends this session. */ }
            close(peer);
        }
    }
    int listener_ = -1;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        Controller controller;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("connection", "127.0.0.1"),
                                     rclcpp::Parameter("request_rate_hz", 10.0)});
        auto observer = std::make_shared<rclcpp::Node>("lidar_integration_observer");
        size_t front = 0, back = 0, heartbeat_count = 0;
        auto on_front = observer->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_front", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::LaserScan& scan)
            {
                require(scan.header.frame_id == "laser_front_link" && scan.ranges.size() == 541 &&
                        scan.ranges[270] == 2 && scan.header.stamp.sec > 0,
                        "end-to-end front geometry or time invalid");
                ++front;
            });
        auto on_back = observer->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_back", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::LaserScan& scan)
            {
                require(scan.header.frame_id == "laser_back_link" && scan.ranges[270] == 2,
                        "end-to-end back scan invalid");
                ++back;
            });
        auto on_heartbeat = observer->create_subscription<std_msgs::msg::Header>(
            "radar_heartbeat", 10, [&](const std_msgs::msg::Header&) { ++heartbeat_count; });
        auto node = std::make_shared<lidar::LidarNode>(options);
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(observer);
        const auto end = lidar::Clock::now() + 2500ms;
        while (lidar::Clock::now() < end)
        {
            executor.spin_some();
            std::this_thread::sleep_for(2ms);
        }
        executor.remove_node(node);
        node.reset();
        require(front >= 5 && back >= 5 && heartbeat_count >= 5 &&
                controller.connections >= 2 && controller.protocol_ok,
                "real TCP -> source -> ROS chain did not recover");
        rclcpp::shutdown();
        std::cout << "lidar_integration_test passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
