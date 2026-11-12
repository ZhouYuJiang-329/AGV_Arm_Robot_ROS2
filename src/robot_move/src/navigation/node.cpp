#include "robot_move/navigation/node.hpp"
#include <algorithm>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2_ros/qos.hpp>
namespace robot_move::navigation
{
namespace
{
bool finite(double v) {return std::isfinite(v);}
bool quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return finite(norm) && std::abs(norm - 1) < 1e-3;
}
}
Node::Node(const rclcpp::NodeOptions & options, Worker::Send sender)
: rclcpp::Node("navigation_guard", options)
{
  rcl_interfaces::msg::ParameterDescriptor d; d.read_only = true;
  auto real = [&](const std::string & n, double v) {return declare_parameter<double>(n, v, d);};
  Config c;
  c.command_timeout = real("cmd_vel_timeout", c.command_timeout);
  c.linear_limit = real("max_linear_speed", c.linear_limit);
  c.angular_limit = real("max_angular_speed", c.angular_limit);
  c.stopped_linear = real("stopped_linear_threshold", c.stopped_linear);
  c.stopped_angular = real("stopped_angular_threshold", c.stopped_angular);
  c.stopped_hold = real("stopped_hold_sec", c.stopped_hold);
  c.duration_ms = declare_parameter<int>("controller_duration_ms", c.duration_ms, d);
  c.request_ms = declare_parameter<int>("request_timeout_ms", c.request_ms, d);
  c.stop_attempts = declare_parameter<int>("stop_attempts", c.stop_attempts, d);
  c.motion_enabled = declare_parameter<bool>("motion_enabled", false, d);
  c.freshness_verified = declare_parameter<bool>("input_freshness_contract_verified", false, d);
  address_ = declare_parameter<std::string>("connection", "", d);
  if (get_parameter("use_sim_time").as_bool())
  {throw std::invalid_argument("real navigation guard requires use_sim_time=false");}
  if (address_.empty()) {throw std::invalid_argument("connection is required");}
  c.validate(); client_ = std::make_unique<seer::Client>();
  const bool injected = static_cast<bool>(sender);
  if (!sender) {
    sender = [this](Velocity v, int duration, Time deadline) {
        if (!connected_) {return false;}
        return client_->setVelocity(v.x, v.y, v.w, duration, deadline);
      };
  }
  worker_ = std::make_unique<Worker>(c, std::move(sender));
  auto & g = worker_->guard;
  auto topics = declare_parameter<std::vector<std::string>>("required_scan_topics",
      {"/scan_front", "/scan_back"}, d);
  if (topics.empty()) {throw std::invalid_argument("at least one navigation lidar is required");}
  auto scan_age = real("scan_timeout", 0.6);
  for (const auto & topic : topics) {
    g.require(topic, scan_age);
    scans_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this, topic](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        bool valid = !m->header.frame_id.empty() && !m->ranges.empty() &&
        finite(m->angle_increment) && m->angle_increment > 0 &&
        finite(m->range_min) && finite(m->range_max) && m->range_min >= 0 &&
        m->range_max > m->range_min && std::any_of(m->ranges.begin(), m->ranges.end(),
        [&](float r) {return finite(r) && r >= m->range_min && r <= m->range_max;});
        std::lock_guard<std::mutex> lock(worker_->mutex);
        worker_->guard.input(topic, rclcpp::Time(m->header.stamp).seconds(), now().seconds(),
          valid, Clock::now()); worker_->wake();
      }));
  }
  g.require("odom", real("odom_timeout", 0.6));
  odom_ = create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto & v = m->twist.twist; const auto & p = m->pose.pose;
        bool valid = m->header.frame_id == "odom" && m->child_frame_id == "base_link" &&
        finite(v.linear.x) && finite(v.linear.y) && finite(v.angular.z) &&
        finite(p.position.x) && finite(p.position.y) && quaternion(p.orientation);
        for (int i : {0, 7, 35}) {
          valid &= finite(m->twist.covariance[i]) && m->twist.covariance[i] > 0 &&
          finite(m->pose.covariance[i]) && m->pose.covariance[i] > 0;
        }
        const auto t = Clock::now(); std::lock_guard<std::mutex> lock(worker_->mutex);
        bool accepted = worker_->guard.input("odom", rclcpp::Time(m->header.stamp).seconds(),
        now().seconds(), valid, t);
        if (accepted || !valid) {
          worker_->guard.feedback(std::hypot(v.linear.x, v.linear.y), v.angular.z, t,
          accepted && valid);
        }
        worker_->wake();
    });
  const auto tf_age = real("tf_timeout", 0.8);
  auto pairs = declare_parameter<std::vector<std::string>>("required_tf_edges",
      {"map->odom", "odom->base_link"}, d);
  if (pairs.empty()) {throw std::invalid_argument("navigation requires dynamic TF edges");}
  for (const auto & edge : pairs) {
    g.require("tf:" + edge, tf_age);
  }
  tf_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicListenerQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(worker_->mutex);
        for (const auto & t : m->transforms) {
          auto name = "tf:" + t.header.frame_id + "->" + t.child_frame_id;
          if (!worker_->guard.inputs.count(name)) {continue;}
          const auto & p = t.transform.translation;
          worker_->guard.input(name, rclcpp::Time(t.header.stamp).seconds(), now().seconds(),
          finite(p.x) && finite(p.y) && finite(p.z) && quaternion(t.transform.rotation),
          Clock::now());
        }
        worker_->wake();
    });
  // Select the actual localization provider explicitly. Empty means TF-only
  // localization (e.g. AMCL); Seer default requires strict status/quality fields.
  auto localization_name = declare_parameter<std::string>("localization_diagnostic",
      "/seer_localization/map_to_odom", d);
  if (!localization_name.empty()) {
    g.require("localization", real("localization_timeout", 0.8));
    localization_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10,
        [this, localization_name](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
          for (const auto & s : m->status) {
            if (s.name != localization_name) {continue;}
            std::map<std::string, std::string> fields;
            for (const auto & v : s.values) {
              fields[v.key] = v.value;
            }
            bool valid = s.level <= 1 && fields["coordinate_status"] == "configured_confirmed" &&
            fields["reloc_status"] == "1.000000" && fields["tf_available"] == "true" &&
            (s.message == "receive_time_approximation" ||
            s.message == "persistent_correction_accepted");
            try {
              size_t used = 0; const auto text = fields.at("confidence");
              const double confidence = std::stod(text, &used);
              valid &= used == text.size() && finite(confidence) && confidence >= 0.5 &&
              confidence <= 1;
            // Timer diagnostics must not renew the age of the underlying pose.
              const auto age_text = fields.at("response_age_sec");
              double age = std::stod(age_text, &used);
              valid &= used == age_text.size() && finite(age) && age >= 0;
              const double stamp = rclcpp::Time(m->header.stamp).seconds();
              std::lock_guard<std::mutex> lock(worker_->mutex);
              worker_->guard.input("localization", stamp - age, now().seconds(), valid,
              Clock::now());
            } catch (...) {
              std::lock_guard<std::mutex> lock(worker_->mutex);
              worker_->guard.input("localization", 0, now().seconds(), false, Clock::now());
            }
            worker_->wake();
          }
      });
  }
  commands_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m, const rclcpp::MessageInfo & info) {
        std::lock_guard<std::mutex> lock(worker_->mutex);
        worker_->guard.command({m->linear.x, m->linear.y, m->angular.z},
        m->linear.z == 0 && m->angular.x == 0 && m->angular.y == 0 &&
        info.get_rmw_message_info().source_timestamp > command_barrier_ &&
        now().nanoseconds() >= info.get_rmw_message_info().source_timestamp &&
        (now().nanoseconds() - info.get_rmw_message_info().source_timestamp) * 1e-9 <
        worker_->guard.config.command_timeout, Clock::now());
        worker_->wake();
    });
  recovery_ = create_service<std_srvs::srv::Trigger>("~/recover",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(worker_->mutex);
        response->success = worker_->guard.recover(Clock::now());
        if (response->success) {command_barrier_ = now().nanoseconds();}
        response->message = response->success ? "Ready; send a NEW command" :
        "requires healthy verified inputs, zero ACK and fresh sustained stopped feedback";
    });
  diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {diagnostics();});
  // Startup connection is bounded by the existing four one-second port deadlines.
  // It happens before spin, never inside an executor callback or the stop worker.
  if (!injected && c.motion_enabled && c.freshness_verified) {
    connected_ = client_->connect(address_);
  }
  worker_->start();
}
Node::~Node() {if (worker_) {worker_->shutdown();}}
void Node::diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray message; message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus s; s.name = "/navigation_guard";
  s.hardware_id = "seer_2010";
  std::lock_guard<std::mutex> lock(worker_->mutex); auto & g = worker_->guard;
  const auto t = Clock::now();
  s.level = g.state == "Fault" ? 2 : g.state == "Active" || g.state == "Ready" ? 0 : 1;
  s.message = g.state + ": " + g.reason;
  auto add = [&](const std::string & key, const std::string & v) {
      diagnostic_msgs::msg::KeyValue entry; entry.key = key; entry.value = v;
      s.values.push_back(entry);
    };
  add("state", g.state); add("stop_reason", g.reason);
  add("valid_command_age_sec",
      g.last_command ? std::to_string(seconds(t, *g.last_command)) : "never");
  add("zero_sent", g.zero_sent ? "true" : "false"); add("zero_ack", g.zero_ack ? "true" : "false");
  add("zero_attempts", std::to_string(g.zero_count));
  add("controller_duration_ms", std::to_string(g.config.duration_ms));
  add("controller_expiry_stop", "UNVERIFIED_ON_TARGET_FIRMWARE");
  add("motion_enabled", g.config.motion_enabled ? "true" : "false");
  add("input_freshness_contract_verified", g.config.freshness_verified ? "true" : "false");
  add("stopped_confirmed", g.confirmed(t) ? "true" : "false");
  add("recovery_conditions",
      "healthy verified inputs + zero ACK + sustained fresh stopped feedback + new command");
  for (const auto & i : g.inputs) {
    add("input:" + i.first,
        i.second.valid && t < i.second.expires ? "healthy" : i.second.reason + "/expired");
    add("input_age:" + i.first, i.second.last_stamp > 0 ?
      std::to_string(i.second.limit - seconds(i.second.expires, t)) : "never");
  }
  message.status.push_back(s); diagnostics_->publish(message);
}
}
