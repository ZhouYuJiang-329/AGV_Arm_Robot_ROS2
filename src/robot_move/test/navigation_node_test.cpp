#include "robot_move/navigation/node.hpp"
#include "seer_test_support.hpp"
#include <atomic>
using namespace std::chrono_literals;
using namespace robot_move::navigation;
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("connection", "127.0.0.1"),
        rclcpp::Parameter("motion_enabled", true),
        rclcpp::Parameter("input_freshness_contract_verified", true),
        rclcpp::Parameter("required_scan_topics", std::vector<std::string>{"/test_scan"}),
        rclcpp::Parameter("required_tf_edges", std::vector<std::string>{"map->odom"})});
    std::atomic<int> motion{0}, zeros{0};
    auto guard = std::make_shared<Node>(options, [&](Velocity v, int duration, Time) {
          require(duration == 800, "finite duration");
          if (v.x != 0) {++motion;} else {++zeros;} return true;
      });
    auto io = std::make_shared<rclcpp::Node>("navigation_test_io");
    auto scans = io->create_publisher<sensor_msgs::msg::LaserScan>("/test_scan",
      rclcpp::SensorDataQoS());
    auto odom = io->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    auto tf = io->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    auto localization = io->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
      10);
    auto commands = io->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    auto recover = io->create_client<std_srvs::srv::Trigger>("/navigation_guard/recover");
    std::string state; bool confirmed = false;
    auto diagnostics =
      io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10,
        [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
          for (const auto & s : m->status) {
            if (s.name == "/navigation_guard") {
              for (const auto & v : s.values) {
                if (v.key == "state") {state = v.value;}
                if (v.key == "stopped_confirmed") {confirmed = v.value == "true";}
              }
            }
          }
      });
    bool scan_enabled = true, odom_enabled = true, tf_enabled = true, localization_valid = true;
    auto timer = io->create_wall_timer(40ms, [&] {
          const auto stamp = io->now();
          if (scan_enabled) {
            sensor_msgs::msg::LaserScan s; s.header.stamp = stamp; s.header.frame_id = "laser";
            s.angle_increment = .1; s.range_min = .1; s.range_max = 10; s.ranges = {1};
            scans->publish(s);
          }
          if (odom_enabled) {
            nav_msgs::msg::Odometry o; o.header.stamp = stamp; o.header.frame_id = "odom";
            o.child_frame_id = "base_link"; o.pose.pose.orientation.w = 1;
            for (int i : {0, 7, 35}) {
              o.pose.covariance[i] = .01; o.twist.covariance[i] = .01;
            }
            odom->publish(o);
          }
          if (tf_enabled) {
            tf2_msgs::msg::TFMessage m; geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stamp; t.header.frame_id = "map"; t.child_frame_id = "odom";
            t.transform.rotation.w = 1; m.transforms.push_back(t); tf->publish(m);
          }
          diagnostic_msgs::msg::DiagnosticArray a; a.header.stamp = stamp;
          diagnostic_msgs::msg::DiagnosticStatus s; s.name = "/seer_localization/map_to_odom";
          s.level = 1;
          s.message = "receive_time_approximation";
          for (auto field : {std::pair{"coordinate_status", "configured_confirmed"},
            {"reloc_status", "1.000000"}, {"tf_available", "true"}, {"confidence", "0.900000"},
            {"response_age_sec", "0.000000"}})
          {
            diagnostic_msgs::msg::KeyValue v; v.key = field.first; v.value = field.second;
            s.values.push_back(v);
          }
          if (!localization_valid) {s.values.clear();} a.status.push_back(s);
          localization->publish(a);
    });
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(guard);
    executor.add_node(io);
    auto spin = [&](auto duration, bool command = false) {
        auto end = Clock::now() + duration; auto next = Clock::now();
        while (Clock::now() < end) {
          if (command && Clock::now() >= next) {
            geometry_msgs::msg::Twist v; v.linear.x = .05; commands->publish(v);
            next = Clock::now() + 50ms;
          }
          executor.spin_some(); std::this_thread::sleep_for(2ms);
        }
      };
    auto reset = [&] {
        spin(800ms); require(confirmed, "fresh zero feedback confirmation");
        auto response =
          recover->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        require(executor.spin_until_future_complete(response,
        1s) == rclcpp::FutureReturnCode::SUCCESS &&
        response.get()->success, "recovery service");
        int before = motion; spin(150ms); require(motion == before, "recovery itself moved");
      };
    spin(700ms); require(state == "Ready" && motion == 0 && zeros == 0, "startup no command");
    spin(350ms, true); require(motion > 2 && zeros == 0, "continuous normal commands");
    spin(650ms); require(state == "StopRequested" && zeros == 1, "watchdog zero"); reset();
    for (int fault = 0; fault < 4; ++fault) {
      spin(150ms, true); auto before = zeros.load();
      if (fault == 0) {scan_enabled = false;} if (fault == 1) {odom_enabled = false;}
      if (fault == 2) {tf_enabled = false;} if (fault == 3) {localization_valid = false;}
      spin(1000ms, true);
      require(state == "Fault" && zeros == before + 1, "required input fault stop");
      if (fault == 1) {require(!confirmed, "missing feedback claimed stopped");}
      scan_enabled = odom_enabled = tf_enabled = localization_valid = true; reset();
    }
    executor.remove_node(guard); guard.reset(); rclcpp::shutdown();
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
  }
  return 0;
}
