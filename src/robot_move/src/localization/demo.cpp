#include "robot_move/localization/node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
using namespace robot_move::localization;
using Clock = std::chrono::steady_clock;
SE2 odomPose(double seconds)
{
  return {0.6 * std::sin(seconds * 0.12),
    0.3 * (1 - std::cos(seconds * 0.12)), wrap(seconds * 0.12)};
}
// An explicitly synthetic source: contains no Transport, Client or network access.
class DemoSource final : public robot_move::seer::LocalizationSource
{
public:
  explicit DemoSource(Clock::time_point start)
  : start_(start) {}
  bool connect() override {return true;}
  void disconnect() override {}
  bool read(
    Sample & sample, robot_move::seer::Deadline,
    const std::function<void()> & received) override
  {
    const double seconds = std::chrono::duration<double>(Clock::now() - start_).count();
    const bool corrected = static_cast<int>(seconds / 20) % 2;
    SE2 map_odom{-0.56 + (corrected ? 0.8 : 0), 5.5, -0.63 + (corrected ? 0.4 : 0)};
    sample.pose = compose(compose(map_odom, odomPose(seconds)),
        inverse({0, 0, 1.5707963267948966}));
    sample.map = "synthetic_demo";
    sample.md5 = "synthetic_not_controller_map";
    sample.status = 1; sample.confidence = 0.95; sample.method = 0;
    received();
    return true;
  }

private:
  Clock::time_point start_;
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto start = Clock::now();
    auto odom = std::make_shared<rclcpp::Node>("synthetic_demo_odometry");
    auto publisher = odom->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf2_ros::TransformBroadcaster broadcaster(*odom);
    auto timer = odom->create_wall_timer(std::chrono::milliseconds(20), [&] {
          const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
          auto p = odomPose(seconds);
          nav_msgs::msg::Odometry message;
          message.header.stamp = odom->now(); message.header.frame_id = "odom";
          message.child_frame_id = "base_link";
          message.pose.pose.position.x = p.x; message.pose.pose.position.y = p.y;
          message.pose.pose.orientation.z = std::sin(p.angle / 2);
          message.pose.pose.orientation.w = std::cos(p.angle / 2);
          const double dx = 0.072 * std::cos(seconds * 0.12);
          const double dy = 0.036 * std::sin(seconds * 0.12);
          message.twist.twist.linear.x = std::cos(p.angle) * dx + std::sin(p.angle) * dy;
          message.twist.twist.linear.y = -std::sin(p.angle) * dx + std::cos(p.angle) * dy;
          message.twist.twist.angular.z = 0.12;
          publisher->publish(message);
          geometry_msgs::msg::TransformStamped tf;
          tf.header = message.header; tf.child_frame_id = "base_link";
          tf.transform.translation.x = p.x; tf.transform.translation.y = p.y;
          tf.transform.rotation = message.pose.pose.orientation;
          broadcaster.sendTransform(tf);
      });
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("expected_map", "synthetic_demo"),
      rclcpp::Parameter("request_rate_hz", 10.0),
      rclcpp::Parameter("base_yaw_in_seer_rad", 1.5707963267948966),
      rclcpp::Parameter("map_alignment_confirmed", false),
      rclcpp::Parameter("base_transform_confirmed", false)});
    auto localization = std::make_shared<LocalizationNode>(options,
        std::make_unique<DemoSource>(start));
    RCLCPP_WARN(odom->get_logger(),
      "SYNTHETIC DISPLAY ONLY. No controller connection. Correction changes every 20 seconds.");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(odom); executor.add_node(localization); executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("synthetic_demo"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
