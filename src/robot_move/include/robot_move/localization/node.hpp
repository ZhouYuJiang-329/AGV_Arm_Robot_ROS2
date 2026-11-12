#pragma once
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include "robot_move/impl/agv/seer/seer_localization_source.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <thread>
namespace robot_move::localization
{
class LocalizationNode : public rclcpp::Node
{
public:
  explicit LocalizationNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<seer::LocalizationSource> source = nullptr);
  ~LocalizationNode() override;

private:
  using Clock = std::chrono::steady_clock;
  struct Observation
  {
    Sample sample;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Clock::time_point received;
    double latency = 0;
    bool ok = false;
    uint64_t sequence = 0;
  };
  void run();
  void tick();
  void transforms(const tf2_msgs::msg::TFMessage &, const rclcpp::MessageInfo &, bool);
  bool wait(double seconds);
  Gate gate_;
  SE2 map_alignment_, base_offset_;
  double rate_, max_age_, max_latency_, max_span_, odom_age_, timeout_sec_;
  bool assumptions_;
  std::string tf_publish_mode_;
  std::unique_ptr<seer::LocalizationSource> source_;
  tf2_ros::Buffer buffer_;
  std::deque<rclcpp::Time> odom_stamps_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_, static_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex mutex_, wait_mutex_;
  std::condition_variable wake_;
  std::optional<Observation> mailbox_, pending_, accepted_;
  std::optional<rclcpp::Time> last_sample_, last_clock_, last_tf_stamp_;
  Clock::time_point started_ = Clock::now(), last_received_{};
  bool conflict_ = false, fresh_ = false;
  double accepted_time_gap_ = -1;
  std::string pending_reason_, accepted_message_ = "receive_time_approximation";
  std::string error_ = "waiting_for_localization", map_;
  double latency_ = -1, time_gap_ = -1, confidence_ = -1;
  int status_ = -1;
  uint64_t updates_ = 0, attempts_ = 0, failures_ = 0;
};
}
