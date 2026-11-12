#pragma once
#include "robot_move/navigation/worker.hpp"
#include "robot_move/impl/agv/seer/seer_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
namespace robot_move::navigation
{
class Node : public rclcpp::Node
{
public:
  explicit Node(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    Worker::Send sender = {});
  ~Node() override;

private:
  std::unique_ptr<seer::Client> client_;
  std::unique_ptr<Worker> worker_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scans_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr localization_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool connected_ = false;
  int64_t command_barrier_ = 0;
  std::string address_;
  void diagnostics();
};
}
