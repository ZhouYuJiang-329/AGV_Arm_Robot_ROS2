#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "robot_move/odometry/integrator.hpp"
namespace robot_move::odometry
{
class OdometryNode : public rclcpp::Node
{
public:
  explicit OdometryNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<seer::VelocitySource> source = nullptr);
  ~OdometryNode() override;

private:
  using Clock = std::chrono::steady_clock;
  void run();
  void diagnostics();
  bool wait(Clock::time_point deadline);
  std::unique_ptr<seer::VelocitySource> source_;
  std::unique_ptr<Integrator> integrator_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
  double rate_, stale_, offset_x_, offset_y_, offset_yaw_;
  double pose_linear_, pose_yaw_, velocity_linear_, velocity_yaw_, gap_variance_;
  double variance_x_, variance_y_, variance_yaw_;
  std::chrono::milliseconds timeout_{1000};
  std::mutex mutex_, wait_mutex_;
  std::condition_variable wake_;
  std::atomic<bool> stop_{false};
  std::thread worker_;
  Clock::time_point started_ = Clock::now(), updated_{};
  uint64_t frames_ = 0, connections_ = 0, errors_ = 0, skipped_ = 0;
  double request_ms_ = 0, interval_ = 0;
  std::string error_ = "waiting for feedback";
  bool degraded_ = false;
};
}
