#include "robot_move/odometry/odometry_node.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
namespace robot_move::odometry
{
OdometryNode::OdometryNode(
  const rclcpp::NodeOptions & options,
  std::unique_ptr<seer::VelocitySource> source)
: Node("seer_odometry", options), source_(std::move(source))
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  auto real = [&](const std::string & name, double initial) {
      double v = declare_parameter<double>(name, initial, descriptor);
      if (!std::isfinite(v)) {throw std::invalid_argument(name + " must be finite");}
      return v;
    };
  const auto address = declare_parameter<std::string>("connection", "", descriptor);
  const bool confirmed = declare_parameter<bool>("base_transform_confirmed", false, descriptor);
  if (!confirmed) {
    throw std::invalid_argument(
        "confirm measured Seer-to-base transform with base_transform_confirmed");
  }
  offset_x_ = real("base_x_in_seer", 0);
  offset_y_ = real("base_y_in_seer", 0);
  offset_yaw_ = real("base_yaw_in_seer", 0);
  rate_ = real("request_rate_hz", 5);
  stale_ = real("stale_timeout_sec", 1);
  const double max_interval = real("max_integration_interval_sec", 0.6);
  const int timeout = declare_parameter<int>("request_timeout_ms", 1000, descriptor);
  pose_linear_ = real("initial_position_variance", 0.01);
  pose_yaw_ = real("initial_yaw_variance", 0.01);
  velocity_linear_ = real("linear_velocity_variance", 0.0025);
  velocity_yaw_ = real("angular_velocity_variance", 0.0025);
  gap_variance_ = real("gap_variance", 100);
  if (rate_ <= 0 || rate_ > 100 || stale_ <= 0 || timeout < 1 || timeout > 1000 ||
    pose_linear_ <= 0 || pose_yaw_ <= 0 || velocity_linear_ <= 0 || velocity_yaw_ <= 0 ||
    gap_variance_ <= 0 || (!source_ && address.empty()))
  {
    throw std::invalid_argument("invalid odometry timing, variance or connection");
  }
  timeout_ = std::chrono::milliseconds(timeout);
  integrator_ = std::make_unique<Integrator>(std::min(max_interval, stale_));
  variance_x_ = variance_y_ = pose_linear_;
  variance_yaw_ = pose_yaw_;
  if (!source_) {source_ = std::make_unique<seer::SeerVelocitySource>(address);}
  if (declare_parameter<bool>("publish_tf", true, descriptor)) {
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(250), [this] {diagnostics();});
  worker_ = std::thread([this] {run();});
}
OdometryNode::~OdometryNode()
{
  stop_ = true;
  wake_.notify_all();
  timer_->cancel();
  if (worker_.joinable()) {worker_.join();}
}
bool OdometryNode::wait(Clock::time_point deadline)
{
  std::unique_lock<std::mutex> lock(wait_mutex_);
  wake_.wait_until(lock, deadline, [this] {return stop_.load();});
  return stop_;
}
void OdometryNode::run()
{
  bool connected = false;
  std::optional<rclcpp::Time> last_ros;
  auto backoff = std::chrono::milliseconds(500);
  const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 /
      rate_));
  auto mark_gap = [&](const std::string & reason) {
      integrator_->gap();
      variance_x_ += gap_variance_;
      variance_y_ += gap_variance_;
      variance_yaw_ += gap_variance_;
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = reason;
      degraded_ = true;
      skipped_ = integrator_->skipped;
      interval_ = 0;
      ++errors_;
    };
  while (!stop_ && rclcpp::ok(get_node_base_interface()->get_context())) {
    const auto cycle = Clock::now();
    seer::Velocity velocity;
    bool ok = false;
    std::string failure = "connection/query failed; unobserved displacement unrecoverable";
    auto received = cycle;
    auto ros_stamp = now();
    auto request_start = cycle;
    try {
      if (!connected) {
        connected = source_->connect();
        if (connected) {std::lock_guard<std::mutex> lock(mutex_); ++connections_;}
      }
      if (connected && !stop_) {
        request_start = Clock::now();
        ok = source_->read(velocity, request_start + timeout_);
        received = Clock::now();
        ros_stamp = now();
        std::lock_guard<std::mutex> lock(mutex_);
        request_ms_ = std::chrono::duration<double, std::milli>(received - request_start).count();
      }
    } catch (const std::exception & error) {
      failure = error.what();
    }
    if (stop_) {break;}
    if (ok && std::chrono::duration<double>(received - request_start).count() >= stale_)
    {ok = false; failure = "response latency exceeds stale threshold";}
    if (ok && (ros_stamp.nanoseconds() <= 0 || (last_ros && ros_stamp <= *last_ros))) {
      mark_gap("ROS clock paused/backward; integration baseline discarded");
      last_ros = ros_stamp;
    } else if (ok) {
      last_ros = ros_stamp;
      velocity = toBase(velocity, offset_x_, offset_y_, offset_yaw_);
      const auto previous_skips = integrator_->skipped;
      ok = integrator_->accept(velocity,
          std::chrono::duration<double>(received.time_since_epoch()).count());
      if (integrator_->skipped != previous_skips) {
        variance_x_ += gap_variance_;
        variance_y_ += gap_variance_;
        variance_yaw_ += gap_variance_;
      }
      if (ok) {
        const double dt = integrator_->dt;
                // Conservative diagonal envelope: add correlated velocity error
                // and heading-induced position uncertainty, never shrink on recovery.
        const double speed2 = velocity.vx * velocity.vx + velocity.vy * velocity.vy;
        variance_x_ = std::pow(std::sqrt(variance_x_) + dt *
            std::sqrt(velocity_linear_ + speed2 * variance_yaw_), 2);
        variance_y_ = std::pow(std::sqrt(variance_y_) + dt *
            std::sqrt(velocity_linear_ + speed2 * variance_yaw_), 2);
        variance_yaw_ = std::pow(std::sqrt(variance_yaw_) + dt * std::sqrt(velocity_yaw_), 2);
        nav_msgs::msg::Odometry message;
        message.header.stamp = ros_stamp;
        message.header.frame_id = "odom";
        message.child_frame_id = "base_link";
        message.pose.pose.position.x = integrator_->x;
        message.pose.pose.position.y = integrator_->y;
        message.pose.pose.orientation.z = std::sin(integrator_->yaw / 2);
        message.pose.pose.orientation.w = std::cos(integrator_->yaw / 2);
        message.twist.twist.linear.x = velocity.vx;
        message.twist.twist.linear.y = velocity.vy;
        message.twist.twist.angular.z = velocity.w;
        const double pose[] = {variance_x_, variance_y_, 1e6, 1e6, 1e6, variance_yaw_};
        const double twist[] = {velocity_linear_, velocity_linear_, 1e6, 1e6, 1e6, velocity_yaw_};
        for (int i = 0; i < 6; ++i) {
          message.pose.covariance[i * 7] = pose[i];
          message.twist.covariance[i * 7] = twist[i];
        }
        publisher_->publish(message);
        if (tf_) {
          geometry_msgs::msg::TransformStamped transform;
          transform.header = message.header;
          transform.child_frame_id = message.child_frame_id;
          transform.transform.translation.x = message.pose.pose.position.x;
          transform.transform.translation.y = message.pose.pose.position.y;
          transform.transform.rotation = message.pose.pose.orientation;
          tf_->sendTransform(transform);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        updated_ = received;
        ++frames_;
        skipped_ = integrator_->skipped;
        interval_ = dt;
        degraded_ = integrator_->degraded;
        error_.clear();
      }
      backoff = std::chrono::milliseconds(500);
    }
    if (!ok) {
      mark_gap(failure);
      source_->disconnect();
      connected = false;
      if (wait(Clock::now() + backoff)) {break;}
      backoff = std::min(backoff * 2, std::chrono::milliseconds(5000));
    } else {
      const auto periods = (Clock::now() - cycle) / period + 1;
      if (wait(cycle + period * periods)) {break;}
    }
  }
  source_->disconnect();
}
void OdometryNode::diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_fully_qualified_name()) + "/estimated_odometry";
  status.hardware_id = "seer_1005";
  std::lock_guard<std::mutex> lock(mutex_);
  const double age = frames_ ? std::chrono::duration<double>(Clock::now() - updated_).count() : -1;
  status.level = age < 0 || age >= stale_ || !error_.empty() ? 2 : 1;
  status.message = !error_.empty() ? error_ : degraded_ ?
    "estimated odometry degraded: lost displacement unrecoverable" :
    "estimated odometry: receive-time approximation";
  auto add = [&](const std::string & key, const std::string & v) {
      diagnostic_msgs::msg::KeyValue entry; entry.key = key; entry.value = v;
      status.values.push_back(entry);
    };
  auto number = [&](const std::string & key, double v) {add(key, std::to_string(v));};
  add("source", "1005 actual vx/vy/w; estimated odometry");
  add("time_mode", "steady receive intervals; ROS receive stamps");
  add("device_sample_age", "unknown; no verified sample timestamp; duplicates undetectable");
  number("response_age_sec", age);
  number("valid_response_hz",
      frames_ / std::max(1e-6, std::chrono::duration<double>(Clock::now() - started_).count()));
  number("request_ms", request_ms_);
  number("integration_interval_sec", interval_);
  number("skipped_integrations", skipped_);
  number("reconnections", connections_ ? connections_ - 1 : 0);
  number("errors", errors_);
  add("quality_degraded", degraded_ ? "true" : "false");
  message.status.push_back(status);
  diagnostics_->publish(message);
}
}
