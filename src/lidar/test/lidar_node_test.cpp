#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include "lidar/nodes/lidar_node.hpp"

using namespace std::chrono_literals;
void require(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}

struct Control
{
    std::atomic<int> mode{0}, connects{0}, disconnects{0}, reads{0};
    std::atomic<int> delay_ms{0};
};

class FakeSource : public lidar::LidarSource
{
public:
    explicit FakeSource(std::shared_ptr<Control> control) : control_(std::move(control)) {}
    bool connect() override { ++control_->connects; return true; }
    void disconnect() override { ++control_->disconnects; }
    lidar::ReadResult readBatch(lidar::Deadline deadline) override
    {
        ++control_->reads;
        const int mode = control_->mode;
        const auto started = lidar::Clock::now();
        std::this_thread::sleep_until(std::min(deadline, started +
                                             std::chrono::milliseconds(control_->delay_ms.load())));
        lidar::ReadResult result;
        result.batch.request_started = started;
        result.batch.received = lidar::Clock::now();
        if (mode == 2)
        {
            result.status = lidar::ReadStatus::ConnectionError;
            result.error = "injected connection loss";
            return result;
        }
        for (auto name : {"laser_front", "laser_back"})
        {
            if (mode == 3 && std::string(name) == "laser_back") continue;
            lidar::DeviceScan scan;
            scan.name = name;
            scan.source_stamp = mode == 1 ? stamp_ : ++stamp_;
            if (mode == 4) { stamp_ = 1; scan.source_stamp = 1; }
            if (mode == 5) scan.source_stamp.reset();
            scan.range_min = 0.1;
            scan.range_max = 40;
            scan.beams.push_back({0.0, 1.0, 2.0, true, false});
            // Per-device stable timestamp in duplicate mode.
            if (mode == 1) scan.source_stamp = std::string(name) == "laser_front" ? front_ : back_;
            else if (std::string(name) == "laser_front") front_ = scan.source_stamp.value_or(0);
            else back_ = scan.source_stamp.value_or(0);
            result.batch.scans.push_back(scan);
        }
        return result;
    }
private:
    std::shared_ptr<Control> control_;
    uint64_t stamp_ = 100, front_ = 0, back_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto control = std::make_shared<Control>();
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("request_rate_hz", 20.0),
                                     rclcpp::Parameter("stale_timeout_sec", 0.2)});
        auto node = std::make_shared<lidar::LidarNode>(options, std::make_unique<FakeSource>(control));
        auto observer = std::make_shared<rclcpp::Node>("lidar_test_observer");
        size_t front = 0, back = 0, heartbeats = 0, callbacks = 0;
        diagnostic_msgs::msg::DiagnosticArray last;
        auto scan_front = observer->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_front", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::LaserScan& msg)
            { require(msg.header.frame_id == "laser_front_link", "wrong frame"); ++front; });
        auto scan_back = observer->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_back", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::LaserScan&) { ++back; });
        auto heartbeat = observer->create_subscription<std_msgs::msg::Header>(
            "radar_heartbeat", 10, [&](const std_msgs::msg::Header&) { ++heartbeats; });
        auto diag = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray& msg) { last = msg; });
        auto timer = observer->create_wall_timer(10ms, [&] { ++callbacks; });
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(observer);
        auto spin = [&](auto duration)
        {
            const auto end = lidar::Clock::now() + duration;
            while (lidar::Clock::now() < end)
            {
                executor.spin_some();
                std::this_thread::sleep_for(2ms);
            }
        };
        spin(700ms);
        require(front > 0 && back > 0 && heartbeats > 0, "no published scans/heartbeat");
        control->mode = 1;
        spin(150ms);  // drain in-flight work
        const auto frozen_front = front, frozen_heartbeats = heartbeats;
        spin(450ms);
        require(front == frozen_front && heartbeats == frozen_heartbeats,
                "duplicate source frames refreshed output/heartbeat");
        require(last.status.size() == 3 && last.status[1].level == 2,
                "stale data did not report ERROR");
        control->mode = 3;
        spin(450ms);
        require(front > frozen_front && last.status[2].level == 2,
                "missing back device not diagnosed independently");
        const auto missing_heartbeat = heartbeats;
        spin(150ms);
        require(heartbeats == missing_heartbeat, "missing device allowed heartbeat");
        control->mode = 2;
        spin(150ms);
        require(control->disconnects > 0, "connection error did not close source");
        control->mode = 0;
        spin(800ms);
        require(control->connects >= 2 && heartbeats > missing_heartbeat,
                "reconnect did not resume output");
        control->mode = 4;
        spin(350ms);
        require(last.status[1].level >= 1, "source clock reset not diagnosed");
        control->mode = 5;
        spin(350ms);
        require(last.status[1].level == 2, "missing timestamp accepted as fresh");
        control->mode = 0;
        control->delay_ms = 300;
        const auto previous_callbacks = callbacks;
        spin(450ms);
        require(callbacks > previous_callbacks + 20, "slow I/O blocked ROS executor");
        executor.remove_node(node);
        const auto before = lidar::Clock::now();
        node.reset();
        require(lidar::Clock::now() - before < 1200ms, "worker shutdown exceeded I/O bound");

        // Validate configuration before constructing any worker/source connection.
        rclcpp::NodeOptions invalid;
        invalid.parameter_overrides({rclcpp::Parameter("devices", std::vector<std::string>{})});
        bool rejected = false;
        try { auto bad = std::make_shared<lidar::LidarNode>(invalid, std::make_unique<FakeSource>(control)); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "empty device list accepted");
        rclcpp::shutdown();
        std::cout << "lidar_node_test passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
