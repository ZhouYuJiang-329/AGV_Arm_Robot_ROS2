#include "robot_move/localization/node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <iostream>
using namespace std::chrono_literals;
using namespace robot_move::localization;
namespace
{
class Source : public robot_move::seer::LocalizationSource
{
public:
  bool connect() override {return true;}
  void disconnect() override {}
  bool read(
    Sample & sample, robot_move::seer::Deadline end,
    const std::function<void()> & received) override
  {
    std::this_thread::sleep_until(std::min(end, std::chrono::steady_clock::now() + 60ms));
    received(); sample.map = "floor"; sample.status = 1; sample.confidence = 0.9;
    return true;
  }
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("expected_map", "floor"),
        rclcpp::Parameter("request_rate_hz", 2.0),
        rclcpp::Parameter("tf_publish_mode", "bounded_odom_hold")});
    auto localizer = std::make_shared<LocalizationNode>(options, std::make_unique<Source>());
    auto io = std::make_shared<rclcpp::Node>("scan_filter_timing_io");
    tf2_ros::Buffer buffer(io->get_clock());
    buffer.setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
        io->get_node_base_interface(), io->get_node_timers_interface()));
    geometry_msgs::msg::TransformStamped mount;
    mount.header.frame_id = "base_link"; mount.child_frame_id = "laser_front_link";
    mount.transform.rotation.w = 1; mount.transform.translation.z = -0.24;
    buffer.setTransform(mount, "fixture_mount", true);
    // Same two target frames, 50ms scan tolerance and 300ms wait as Jazzy ObstacleLayer.
    tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> filter(buffer, "map", 50, io, 300ms);
    filter.setTargetFrames({"map", "laser_front_link"});
    filter.setTolerance(rclcpp::Duration::from_seconds(0.05));
    size_t sent = 0, accepted = 0;
    filter.registerCallback([&](sensor_msgs::msg::LaserScan::ConstSharedPtr) {++accepted;});
    auto tf = io->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    auto observed = io->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100,
        [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
          for (const auto & t : msg->transforms) {
            buffer.setTransform(t, "fixture", false);
          }
      });
    auto odom = io->create_wall_timer(200ms, [&] {
          geometry_msgs::msg::TransformStamped t;
          t.header.frame_id = "odom"; t.child_frame_id = "base_link";
          t.header.stamp = io->now(); t.transform.rotation.w = 1;
          tf2_msgs::msg::TFMessage message; message.transforms.push_back(t); tf->publish(message);
      });
    bool scanning = false;
    auto scan = io->create_wall_timer(200ms, [&] {
          if (!scanning) {return;}
          auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
          msg->header.frame_id = "laser_front_link";
          msg->header.stamp = io->now() - rclcpp::Duration::from_seconds(0.01);
          msg->range_min = .01; msg->range_max = 10; msg->angle_increment = .1;
          msg->ranges = {1}; ++sent; filter.add(msg);
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(io); executor.add_node(localizer);
    auto spin = [&](auto duration) {
        auto end = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < end) {
          executor.spin_some(); std::this_thread::sleep_for(1ms);
        }
      };
    spin(2500ms); scanning = true; spin(4000ms); scanning = false; spin(700ms);
    std::cout << "original-stamp scans=" << sent << " transformed=" << accepted << '\n';
    if (sent < 15 || accepted != sent) {
      std::cerr << "fresh 5Hz scans lost while waiting for 2Hz localization TF support\n";
      result = 1;
    }
    filter.clear(); executor.remove_node(localizer); localizer.reset();
  }
  rclcpp::shutdown(); return result;
}
