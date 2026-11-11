#include "robot_move/odometry/odometry_node.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <map>
#include <cmath>
#include <iostream>
using namespace std::chrono_literals;
using namespace robot_move::odometry;
void require(bool ok, const char * message) {if (!ok) {throw std::runtime_error(message);}}
struct Control { std::atomic<int> delay{0}, mode{0}, reads{0}, connects{0}, disconnects{0}; };
class Fake : public robot_move::seer::VelocitySource
{
public:
  explicit Fake(std::shared_ptr<Control> c)
  : c_(c) {}
  bool connect() override {++c_->connects; return true;}
  void disconnect() override {++c_->disconnects;}
  bool read(Velocity & v, robot_move::seer::Deadline deadline) override
  {
    ++c_->reads;
    std::this_thread::sleep_until(std::min(deadline,
      std::chrono::steady_clock::now() + std::chrono::milliseconds(c_->delay.load())));
    v = {0.1, 0.2, 0.3};
    return c_->mode == 0;
  }

private:
  std::shared_ptr<Control> c_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto c = std::make_shared<Control>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("base_transform_confirmed", true),
        rclcpp::Parameter("request_rate_hz", 20.0), rclcpp::Parameter("stale_timeout_sec", 0.25)});
    auto node = std::make_shared<OdometryNode>(options, std::make_unique<Fake>(c));
    auto observer = std::make_shared<rclcpp::Node>("odometry_test_observer");
    std::map<int64_t, nav_msgs::msg::Odometry> odoms;
    std::map<int64_t, geometry_msgs::msg::TransformStamped> transforms;
    diagnostic_msgs::msg::DiagnosticArray last;
    auto sub = observer->create_subscription<nav_msgs::msg::Odometry>("/odom", 100,
        [&](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          odoms[rclcpp::Time(m->header.stamp).nanoseconds()] = *m;
                                                                                                                });
    auto tf = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100,
        [&](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
          for (auto & t:m->transforms) {
            transforms[rclcpp::Time(t.header.stamp).nanoseconds()] = t;
          }
                                                                                                                                                      });
    auto diag = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
      10,
        [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {last = *m;});
    size_t callbacks = 0;
    auto timer = node->create_wall_timer(10ms, [&] {++callbacks;});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node); executor.add_node(observer);
    auto spin = [&](auto duration) {
        const auto end = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < end) {
          executor.spin_some(); std::this_thread::sleep_for(2ms);
        }
      };
    spin(650ms);
    require(odoms.size() > 3, "no odometry");
    size_t matched = 0;
    for (auto & pair:odoms) {
      auto found = transforms.find(pair.first); if (found == transforms.end()) {continue;}
      auto & m = pair.second; auto & t = found->second;
      require(m.header.frame_id == "odom" && m.child_frame_id == "base_link" &&
                t.header.frame_id == m.header.frame_id && t.child_frame_id == m.child_frame_id,
        "wrong frames");
      require(t.transform.translation.x == m.pose.pose.position.x &&
        t.transform.translation.y == m.pose.pose.position.y &&
                t.transform.rotation == m.pose.pose.orientation, "TF pose mismatch");
      require(std::abs(std::hypot(m.pose.pose.orientation.z,
        m.pose.pose.orientation.w) - 1) < 1e-12, "not unit quaternion");
      require(m.twist.twist.linear.y == 0.2 && m.pose.covariance[14] == 1e6,
        "twist/covariance wrong"); ++matched;
    }
    require(matched > 2, "no matching stamps");
    c->mode = 1; spin(150ms);
    auto count = odoms.size(); auto before = odoms.rbegin()->second;
    spin(400ms);
    require(odoms.size() == count, "stale data republished");
    require(!last.status.empty() && last.status[0].level == 2, "missing stale diagnostic");
    c->mode = 0; spin(1300ms);
    require(c->connects >= 2 && odoms.size() > count, "recovery failed");
    auto after = odoms.upper_bound(rclcpp::Time(before.header.stamp).nanoseconds())->second;
    require(after.pose.pose.position.x == before.pose.pose.position.x &&
      after.pose.pose.position.y == before.pose.pose.position.y,
            "reconnect integrated unknown period or reset pose");
    require(after.pose.covariance[0] > before.pose.covariance[0], "gap did not degrade covariance");
    c->delay = 500; auto previous = callbacks; spin(600ms);
    require(callbacks > previous + 30, "slow I/O blocked executor");
    executor.remove_node(node); timer.reset();
    auto start = std::chrono::steady_clock::now(); node.reset();
    require(std::chrono::steady_clock::now() - start < 1200ms && c->disconnects > 0,
      "shutdown unbounded");
        // publish_tf=false must create no TF publisher.
    options.parameter_overrides({rclcpp::Parameter("base_transform_confirmed", true),
        rclcpp::Parameter("publish_tf", false)});
    auto no_tf = std::make_shared<OdometryNode>(options, std::make_unique<Fake>(c));
    require(no_tf->count_publishers("/tf") == 0, "publish_tf=false ignored");
    no_tf.reset();
    c->delay = 0;
    options.parameter_overrides({rclcpp::Parameter("base_transform_confirmed", true),
        rclcpp::Parameter("publish_tf", false), rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("request_rate_hz", 20.0)});
    auto simulated = std::make_shared<OdometryNode>(options, std::make_unique<Fake>(c));
    executor.add_node(simulated);
    auto clock = observer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    auto clock_spin = [&](double seconds, bool advance, auto duration) {
        const auto end = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < end) {
          rosgraph_msgs::msg::Clock message;
          message.clock = rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
          clock->publish(message); executor.spin_some();
          std::this_thread::sleep_for(5ms);
          if (advance) {seconds += 0.01;}
        }
      };
    odoms.clear();
    clock_spin(100, true, 400ms);
    require(!odoms.empty(), "simulated clock did not produce feedback");
    clock_spin(110, false, 200ms);   // drain last advancing sample, then pause
    const auto frozen_count = odoms.size();
    const auto frozen_pose = odoms.rbegin()->second.pose.pose;
    clock_spin(110, false, 250ms);
    require(odoms.size() == frozen_count, "paused clock refreshed odometry");
    clock_spin(50, false, 250ms);
    require(odoms.size() == frozen_count, "backward clock published a fresh sample");
    clock_spin(51, true, 250ms);
    auto resumed = odoms.lower_bound(51000000000LL);
    require(resumed != odoms.end() && resumed->first < 100000000000LL, "clock recovery missing");
    require(resumed->second.pose.pose.position.x == frozen_pose.position.x &&
                resumed->second.pose.pose.position.y == frozen_pose.position.y,
      "clock jump integrated a gap");
    executor.remove_node(simulated); simulated.reset();
    rclcpp::shutdown(); std::cout << "odometry_node_test passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
  }
}
