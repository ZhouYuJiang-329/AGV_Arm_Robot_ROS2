#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "lidar/interface/lidar_source.hpp"
#include "lidar/processing/laser_scan_builder.hpp"

namespace lidar
{
/// Owns acquisition and reconnection. Inject a source for hardware-free tests.
class LidarNode : public rclcpp::Node
{
public:
    explicit LidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                       std::unique_ptr<LidarSource> source = nullptr);
    ~LidarNode() override;

private:
    struct Device
    {
        std::string name;
        ScanConfig config;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
    };
    struct DeviceState
    {
        std::optional<uint64_t> stamp;
        std::optional<uint64_t> anchor_stamp;
        Deadline anchor_time{};
        bool clock_fault = false;
        std::optional<int> id;
        Deadline updated{};
        size_t frames = 0, duplicates = 0, resets = 0, valid = 0, rejected = 0,
               outside = 0, collisions = 0;
        size_t heartbeat_frame = 0;
        bool present = false;
        std::string error = "waiting for first frame";
        InstallInfo install;
    };
    void run();
    void accept(const ScanBatch& batch, const rclcpp::Time& received);
    void diagnostics();
    bool waitUntil(Deadline deadline);

    std::unique_ptr<LidarSource> source_;
    std::vector<Device> devices_;
    std::vector<DeviceState> states_;
    std::mutex state_mutex_;
    std::string phase_ = "Connecting", error_;
    size_t errors_ = 0, connections_ = 0;
    double request_ms_ = 0;
    Deadline started_ = Clock::now();
    std::chrono::milliseconds timeout_{1000};
    Clock::duration period_;
    double stale_seconds_ = 1;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_pub_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    std::atomic<bool> stop_{false};
    std::mutex wait_mutex_;
    std::condition_variable wake_;
    std::thread worker_;
};
}  // namespace lidar
