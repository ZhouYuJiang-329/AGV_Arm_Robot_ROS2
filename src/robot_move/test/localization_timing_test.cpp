#include "robot_move/localization/node.hpp"
#include <deque>
#include <set>
#include <iostream>
using namespace std::chrono_literals;
using namespace robot_move::localization;
namespace
{
void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}
struct Control
{
  std::mutex mutex;
  std::condition_variable ready;
  std::deque<int> samples;
  std::atomic<int> received{0};
  void push(int mode)
  {
    std::lock_guard<std::mutex> lock(mutex); samples.push_back(mode); ready.notify_all();
  }
};
class Source : public robot_move::seer::LocalizationSource
{
public:
  explicit Source(std::shared_ptr<Control> control)
  : control_(control) {}
  bool connect() override {return true;}
  void disconnect() override {}
  bool read(
    Sample & s, robot_move::seer::Deadline end,
    const std::function<void()> & received) override
  {
    std::unique_lock<std::mutex> lock(control_->mutex);
    if (!control_->ready.wait_until(lock, end, [&] {return !control_->samples.empty();})) {
      return false;
    }
    int mode = control_->samples.front(); control_->samples.pop_front(); lock.unlock();
    received(); s.pose = {2, 3, 0.1}; s.map = mode == 2 ? "wrong" : "floor";
    s.status = 1; s.confidence = 0.9; ++control_->received;
    return mode != 1;
  }

private:
  std::shared_ptr<Control> control_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto c = std::make_shared<Control>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("expected_map", "floor"),
        rclcpp::Parameter("request_rate_hz", 10.0),
        rclcpp::Parameter("max_sample_age_sec", 0.6),
        // This source waits for test-controlled tokens inside read().
        rclcpp::Parameter("max_query_latency_sec", 1.0),
        rclcpp::Parameter("max_odom_age_sec", 0.25),
        rclcpp::Parameter("max_odom_bracket_sec", 0.3),
        rclcpp::Parameter("tf_publish_mode", "bounded_odom_hold")});
    auto localizer = std::make_shared<LocalizationNode>(options, std::make_unique<Source>(c));
    auto io = std::make_shared<rclcpp::Node>("localization_timing_io");
    auto tf = io->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    std::map<std::string, std::string> fields;
    diagnostic_msgs::msg::DiagnosticStatus status;
    rclcpp::Time diagnostic_stamp(0, 0, RCL_ROS_TIME);
    auto diagnostics = io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10, [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        status = msg->status.at(0); fields.clear(); diagnostic_stamp = msg->header.stamp;
        for (const auto & value : status.values) {
          fields[value.key] = value.value;
        }
      });
    size_t poses = 0; std::vector<rclcpp::Time> support;
    std::set<int64_t> odometry_sample_stamps;
    rclcpp::Time pose_stamp(0, 0, RCL_ROS_TIME);
    auto pose_sub = io->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/seer/localization/base_pose", 10, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) {
        ++poses; pose_stamp = p->header.stamp;
      });
    auto map_sub = io->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100,
        [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
          for (const auto & t : msg->transforms) {
            if (t.child_frame_id == "odom") {
              require(odometry_sample_stamps.count(rclcpp::Time(t.header.stamp).nanoseconds()) == 1,
                "TF support does not match a received odometry sample");
              require(rclcpp::Time(t.header.stamp) <= io->now(), "TF support is future-stamped");
              support.emplace_back(t.header.stamp);
            }
          }
      });
    bool odom_enabled = true;
    auto odom = io->create_wall_timer(20ms, [&] {
          if (!odom_enabled) {return;}
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = io->now();
          odometry_sample_stamps.insert(rclcpp::Time(t.header.stamp).nanoseconds());
          t.header.frame_id = "odom"; t.child_frame_id = "base_link";
          t.transform.rotation.w = 1;
          tf2_msgs::msg::TFMessage message; message.transforms.push_back(t); tf->publish(message);
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(io); executor.add_node(localizer);
    auto spin = [&](auto duration) {
        auto end = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < end) {
          executor.spin_some(); std::this_thread::sleep_for(1ms);
        }
      };
    c->push(0); spin(1150ms);
    require(poses == 0 && support.empty(), "unaccepted localization produced TF support");
    // Start a new accepted measurement after the deliberate one-second discovery gate.
    c->push(0);
    auto initial_deadline = std::chrono::steady_clock::now() + 1200ms;
    while ((poses == 0 || status.level != 1) &&
      std::chrono::steady_clock::now() < initial_deadline) {spin(5ms);}
    require(poses >= 1 && status.level == 1, "no initial accepted measurement");
    auto accepted_stamp = pose_stamp; auto count = poses;
    odom_enabled = false; c->push(0);
    auto end = std::chrono::steady_clock::now() + 200ms;
    while (c->received < 3 && std::chrono::steady_clock::now() < end) {spin(2ms);}
    spin(70ms);
    require(status.level == 1 && fields["tf_available"] == "true" &&
      status.message == "receive_time_approximation",
      "normal interpolation wait invalidated a still-fresh accepted localization");
    require(fields["pending_reason"] == "waiting_for_odom_bracket", "missing pending status");
    require(poses == count && pose_stamp == accepted_stamp, "pending sample republished a pose");
    const double underlying_stamp = diagnostic_stamp.seconds() -
      std::stod(fields.at("response_age_sec"));
    require(std::abs(underlying_stamp - accepted_stamp.seconds()) < 0.003,
      "diagnostic timer or pending query refreshed accepted sample age");
    // Fresh odometry can support a bounded correction without claiming a new pose.
    odom_enabled = true; spin(100ms);
    require(poses == count + 1, "pending interpolation did not complete");
    accepted_stamp = pose_stamp; count = poses; auto transforms = support.size();
    spin(100ms);
    require(support.size() > transforms + 1 && poses == count,
      "accepted correction does not support subsequent fresh odometry stamps");
    for (size_t i = 1; i < support.size(); ++i) {
      require(support[i] > support[i - 1], "TF support stamps must strictly increase");
    }
    require(support.back() > accepted_stamp, "no odometry-supported TF interval");
    spin(550ms); transforms = support.size(); spin(100ms);
    require(status.level == 2 && support.size() == transforms,
      "continued odometry kept expired localization alive");
    auto recover = [&] {
        const auto before = poses; c->push(0);
        auto deadline = std::chrono::steady_clock::now() + 1800ms;
        while ((poses == before || status.level != 1) &&
          std::chrono::steady_clock::now() < deadline) {spin(5ms);}
        require(poses > before && status.level == 1, "fresh recovery failed");
      };
    recover();
    odom_enabled = false; spin(300ms);
    require(status.level == 2, "stale odometry left localization execution-ready");
    odom_enabled = true; spin(60ms); recover();
    c->push(2); spin(180ms);
    require(status.level == 2 && status.message == "map_mismatch", "wrong map hidden by held pose");
    recover(); c->push(1); spin(180ms);
    require(status.level == 2, "query failure hidden by held pose");
    executor.remove_node(localizer); localizer.reset();
    std::cout << "accepted/pending localization, source-age expiry and bounded TF support passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
