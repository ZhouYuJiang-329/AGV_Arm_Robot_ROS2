#include "robot_move/localization/node.hpp"
#include <iostream>
using namespace std::chrono_literals;
using namespace robot_move::localization;
void require(bool b, const char * m) {if (!b) {throw std::runtime_error(m);}}
struct Control {std::atomic<int> delay{0}, mode{0}, reads{0}; std::atomic<bool> closed{false};};
class Fake : public robot_move::seer::LocalizationSource
{
public:
  explicit Fake(std::shared_ptr<Control> c)
  : c_(c) {}
  bool connect() override {return true;}
  void disconnect() override {c_->closed = true;}
  bool read(
    Sample & s, robot_move::seer::Deadline deadline,
    const std::function<void()> & received) override
  {
    ++c_->reads;
    std::this_thread::sleep_until(std::min(deadline,
      std::chrono::steady_clock::now() + std::chrono::milliseconds(c_->delay.load())));
    received(); s.pose = {c_->mode == 4 ? 5.0 : 2.0, 3, 1.0};
    s.map = c_->mode == 2 ? "other" : "floor";
    s.status = c_->mode == 3 ? 2 : 1; s.confidence = 0.9;
    return c_->mode != 1;
  }

private:
  std::shared_ptr<Control> c_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto c = std::make_shared<Control>();
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("expected_map", "floor"),
        rclcpp::Parameter("request_rate_hz", 10.0),
        rclcpp::Parameter("max_sample_age_sec", 0.3),
        rclcpp::Parameter("max_query_latency_sec", 0.2),
        rclcpp::Parameter("max_odom_bracket_sec", 0.1)});
    auto node = std::make_shared<LocalizationNode>(opts, std::make_unique<Fake>(c));
    auto observer = std::make_shared<rclcpp::Node>("localization_observer");
    auto tf = observer->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    std::vector<geometry_msgs::msg::TransformStamped> maps;
    auto sub = observer->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/seer/localization/base_pose", 100, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) {
        poses.push_back(*p);
      });
    auto tsub = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100,
        [&](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
          for (auto & t:m->transforms) {
            if (t.child_frame_id == "odom") {
              maps.push_back(t);
            }
          }
      });
    diagnostic_msgs::msg::DiagnosticStatus last;
    auto diag = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10, [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr a) {
        if (!a->status.empty()) {last = a->status[0];}
      });
    size_t callbacks = 0; bool odom_enabled = false;
    auto timer = node->create_wall_timer(10ms, [&] {++callbacks;});
    auto odom_timer = observer->create_wall_timer(20ms, [&] {
          if (!odom_enabled) {return;}
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = observer->now(); t.header.frame_id = "odom";
          t.child_frame_id = "base_link";
          t.transform.translation.x = 1; t.transform.translation.y = -1;
          t.transform.rotation.z = std::sin(0.4); t.transform.rotation.w = std::cos(0.4);
          tf2_msgs::msg::TFMessage m; m.transforms.push_back(t); tf->publish(m);
      });
    rclcpp::executors::SingleThreadedExecutor ex; ex.add_node(node); ex.add_node(observer);
    auto spin = [&](auto duration) {
        auto end = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < end) {
          ex.spin_some(); std::this_thread::sleep_for(2ms);
        }
      };
    spin(1300ms); require(poses.empty(), "published without TF");
    odom_enabled = true; spin(600ms); require(poses.size() > 2, "TF recovery failed");
    require(!maps.empty(), "no map odom TF");
    auto t = maps.back(); auto p = poses.back();
    SE2 correction{t.transform.translation.x, t.transform.translation.y,
      2 * std::atan2(t.transform.rotation.z, t.transform.rotation.w)};
    require(near(compose(correction, {1, -1, 0.8}), {2, 3, 1}, 1e-8, 1e-8), "wrong SE2 TF");
    require(t.header.stamp == p.header.stamp && p.header.frame_id == "map", "stamp/frame mismatch");
    c->mode = 2; spin(350ms); auto count = poses.size(); spin(250ms);
    require(poses.size() == count && last.level == 2, "map change accepted");
    c->mode = 3; spin(350ms); require(poses.size() == count, "relocating accepted");
    c->mode = 0; spin(350ms); require(poses.size() > count, "quality recovery failed");
    c->mode = 4; spin(600ms);
    require(poses.back().pose.position.x == 5, "stable relocation correction not recovered");
    c->mode = 0; spin(600ms);
    require(poses.back().pose.position.x == 2, "relocation return not recovered");
    odom_enabled = false; spin(600ms); count = poses.size(); spin(300ms);
    require(poses.size() == count && last.level == 2, "stale odom accepted");
    odom_enabled = true; spin(600ms); require(poses.size() > count, "odom stale recovery failed");
    c->delay = 500; spin(300ms); auto callback_start = callbacks; count = poses.size(); spin(600ms);
    require(callbacks > callback_start + 30, "slow query blocked executor");
    require(poses.size() == count, "slow query accepted");
    c->delay = 0; c->mode = 1; spin(600ms); count = poses.size(); spin(600ms);
    require(poses.size() == count && last.level == 2, "failed query republished");
    c->mode = 0; spin(1600ms); require(poses.size() > count, "reconnect recovery failed");
    // Foreign odom parent must latch, including when geometry equals our own.
    tf2_msgs::msg::TFMessage foreign; foreign.transforms.push_back(maps.back());
    tf->publish(foreign); spin(300ms); count = poses.size(); spin(300ms);
    require(poses.size() == count && last.level == 2, "competing owner not blocked");
    c->delay = 500; spin(150ms); auto start = std::chrono::steady_clock::now();
    ex.remove_node(node); timer.reset(); node.reset();
    require(std::chrono::steady_clock::now() - start < 1200ms && c->closed, "unbounded shutdown");
    std::cout << "localization ROS TF/quality/time/failure/worker lifecycle passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
