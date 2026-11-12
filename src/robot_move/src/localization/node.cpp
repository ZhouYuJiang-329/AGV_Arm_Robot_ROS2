#include "robot_move/localization/node.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_ros/qos.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <algorithm>
#include <cstring>
#include <stdexcept>
namespace robot_move::localization
{
LocalizationNode::LocalizationNode(
  const rclcpp::NodeOptions & options,
  std::unique_ptr<seer::LocalizationSource> source)
: Node("seer_localization", options), source_(std::move(source)), buffer_(get_clock())
{
  rcl_interfaces::msg::ParameterDescriptor d; d.read_only = true;
  auto real = [&](const std::string & n, double initial) {
      auto v = declare_parameter<double>(n, initial, d);
      if (!std::isfinite(v)) {throw std::invalid_argument(n + " must be finite");} return v;
    };
  auto text = [&](const std::string & n, const std::string & v) {
      return declare_parameter<std::string>(n, v, d);
    };
  auto address = text("connection", "");
  tf_publish_mode_ = text("tf_publish_mode", "measurement_time");
  if (tf_publish_mode_ != "measurement_time" && tf_publish_mode_ != "bounded_odom_hold") {
    throw std::invalid_argument("tf_publish_mode must be measurement_time or bounded_odom_hold");
  }
  if (text("stamp_mode", "receive_time") != "receive_time") {
    throw std::invalid_argument("only receive_time is verified for this API");
  }
  assumptions_ = !declare_parameter<bool>("map_alignment_confirmed", false, d);
  assumptions_ |= !declare_parameter<bool>("base_transform_confirmed", false, d);
  map_alignment_ = {real("map_x_in_ros_m", 0), real("map_y_in_ros_m", 0),
    real("map_yaw_in_ros_rad", 0)};
  base_offset_ = {real("base_x_in_seer_m", 0), real("base_y_in_seer_m", 0),
    real("base_yaw_in_seer_rad", 0)};
  gate_.expected_map = text("expected_map", "");
  gate_.expected_md5 = text("expected_map_md5", "");
  gate_.min_confidence = real("min_confidence", 0.5);
  gate_.require_confidence = declare_parameter<bool>("require_confidence", true, d);
  gate_.jump_m = real("max_correction_translation_m", 0.5);
  gate_.jump_rad = real("max_correction_rotation_rad", 0.5);
  gate_.stable_m = real("recovery_translation_m", 0.1);
  gate_.stable_rad = real("recovery_rotation_rad", 0.1);
  gate_.recovery_samples = declare_parameter<int>("jump_recovery_samples", 3, d);
  rate_ = real("request_rate_hz", 2);
  max_age_ = real("max_sample_age_sec", 0.8);
  max_latency_ = real("max_query_latency_sec", 0.6);
  max_span_ = real("max_odom_bracket_sec", 0.4);
  odom_age_ = real("max_odom_age_sec", 0.5);
  timeout_sec_ = real("request_timeout_sec", 1);
  if (rate_ <= 0 || rate_ > 20 || max_age_ <= 0 || max_latency_ <= 0 ||
    max_span_ <= 0 || odom_age_ <= 0 || timeout_sec_ <= 0 || timeout_sec_ > 1 ||
    gate_.expected_map.empty() || (!source_ && address.empty()) || gate_.min_confidence < 0 ||
    gate_.min_confidence > 1 || gate_.jump_m <= 0 || gate_.jump_rad <= 0 ||
    gate_.stable_m <= 0 || gate_.stable_rad <= 0 || gate_.recovery_samples < 0)
  {throw std::invalid_argument("invalid localization map/connection/timing/quality parameters");}
  if (get_parameter("use_sim_time").as_bool()) {
    throw std::invalid_argument("hardware localization requires use_sim_time=false");
  }
  if (!source_) {source_ = std::make_unique<seer::SeerLocalizationSource>(address);}
  tf_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicBroadcasterQoS());
  pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/seer/localization/base_pose", 10);
  diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicListenerQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m, const rclcpp::MessageInfo & i) {
        transforms(*m, i, false);
    });
  static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
    tf2_ros::StaticListenerQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m, const rclcpp::MessageInfo & i) {
        transforms(*m, i, true);
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {tick();});
  worker_ = std::thread([this] {run();});
}
LocalizationNode::~LocalizationNode()
{
  stop_ = true; wake_.notify_all(); timer_->cancel();
  if (worker_.joinable()) {worker_.join();}
}
bool LocalizationNode::wait(double seconds)
{
  std::unique_lock<std::mutex> lock(wait_mutex_);
  wake_.wait_for(lock, std::chrono::duration<double>(seconds), [this] {return stop_.load();});
  return stop_;
}
void LocalizationNode::run()
{
  bool connected = false; uint64_t seq = 0; double backoff = 0.5;
  while (!stop_ && rclcpp::ok(get_node_base_interface()->get_context())) {
    auto cycle = Clock::now(); Observation o; o.sequence = ++seq;
    try {
      if (!connected) {connected = source_->connect();}
      auto start = Clock::now();
      if (connected && !stop_) {
        auto deadline = start + std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(timeout_sec_));
        o.ok = source_->read(o.sample, deadline, [&] {
              o.received = Clock::now(); o.stamp = now();
          });
        o.latency = std::chrono::duration<double>(Clock::now() - start).count();
      }
    } catch (const std::exception &) {
      o.ok = false;
    }
    if (stop_) {break;}
    {std::lock_guard<std::mutex> lock(mutex_); mailbox_ = o;}
    if (!o.ok) {
      source_->disconnect(); connected = false;
      if (wait(backoff)) {break;} backoff = std::min(5.0, backoff * 2);
    } else {
      backoff = 0.5;
      double elapsed = std::chrono::duration<double>(Clock::now() - cycle).count();
      // Skip missed cycles; never burst queries to catch up.
      if (wait((std::floor(elapsed * rate_) + 1) / rate_ - elapsed)) {break;}
    }
  }
  source_->disconnect();
}
void LocalizationNode::transforms(
  const tf2_msgs::msg::TFMessage & msg,
  const rclcpp::MessageInfo & info, bool is_static)
{
  auto own = tf_->get_gid();
  if (std::memcmp(own.data, info.get_rmw_message_info().publisher_gid.data,
    RMW_GID_STORAGE_SIZE) == 0) {return;}
  for (const auto & t : msg.transforms) {
    if (t.child_frame_id == "odom") {
      conflict_ = true; error_ = "competing_odom_parent_publisher_restart_required";
    }
    if (t.header.frame_id != "odom" || t.child_frame_id != "base_link" || is_static) {continue;}
    const auto & q = t.transform.rotation;
    double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-5 ||
      !std::isfinite(t.transform.translation.x) || !std::isfinite(t.transform.translation.y) ||
      !std::isfinite(t.transform.translation.z)) {continue;}
    rclcpp::Time stamp(t.header.stamp, get_clock()->get_clock_type());
    if (stamp > now() + rclcpp::Duration::from_seconds(0.05)) {continue;}
    if (!odom_stamps_.empty() && stamp <= odom_stamps_.back()) {continue;}
    if (buffer_.setTransform(t, "observed_odometry", false)) {
      odom_stamps_.push_back(stamp);
      while (odom_stamps_.size() > 300) {odom_stamps_.pop_front();}
    }
  }
}
void LocalizationNode::tick()
{
  auto current = now();
  if (last_clock_ && current < *last_clock_) {
    pending_.reset(); accepted_.reset(); last_sample_.reset(); last_tf_stamp_.reset();
    odom_stamps_.clear(); buffer_.clear();
    gate_.interrupt(); fresh_ = false; pending_reason_.clear(); error_ = "clock_backward";
    // Discard observations that straddle the clock jump.
    std::lock_guard<std::mutex> lock(mutex_); mailbox_.reset();
  }
  last_clock_ = current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mailbox_) {
      ++attempts_; latency_ = mailbox_->latency;
      if (!mailbox_->ok) {
        ++failures_; pending_.reset(); pending_reason_.clear(); fresh_ = false;
        gate_.interrupt(); error_ = "query_failed_or_status_map_changed_during_query";
      } else {pending_ = *mailbox_; last_received_ = mailbox_->received;}
      mailbox_.reset();
    }
  }
  // Allow discovery before first publication; observed foreign parent latches a fault.
  if (pending_ && std::chrono::duration<double>(Clock::now() - started_).count() >= 1) {
    auto & o = *pending_;
    status_ = o.sample.status; map_ = o.sample.map;
    confidence_ = o.sample.confidence.value_or(-1);
    std::string reason = gate_.quality(o.sample);
    if (conflict_) {reason = "competing_odom_parent_publisher_restart_required";}
    double age = (current - o.stamp).seconds();
    double steady_age = std::chrono::duration<double>(Clock::now() - o.received).count();
    if (reason.empty()) {reason = timing(age, o.latency, 0, max_age_, max_latency_, max_span_);}
    if (reason.empty() && steady_age > max_age_) {reason = "stale_sample";}
    if (reason.empty() && (o.stamp.nanoseconds() <= 0 ||
      (last_sample_ && o.stamp <= *last_sample_))) {reason = "sample_time_not_increasing";}
    bool retry = false;
    time_gap_ = -1;
    if (reason.empty()) {
      auto upper = std::lower_bound(odom_stamps_.begin(), odom_stamps_.end(), o.stamp);
      if (upper == odom_stamps_.end() || (upper == odom_stamps_.begin() && *upper != o.stamp)) {
        reason = "waiting_for_odom_bracket"; retry = true;
      } else {
        auto lower = *upper == o.stamp ? upper : std::prev(upper);
        time_gap_ = (*upper - *lower).seconds();
        reason = timing(age, o.latency, time_gap_, max_age_, max_latency_, max_span_);
        double latest_age = (current - odom_stamps_.back()).seconds();
        if (latest_age < 0 || latest_age > odom_age_) {reason = "odom_stale_or_future";}
        if (reason.empty()) {
          try {
            auto t = buffer_.lookupTransform("odom", "base_link", o.stamp);
            const auto & q = t.transform.rotation;
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
            if (std::abs(roll) > 0.01 || std::abs(pitch) > 0.01 ||
              std::abs(t.transform.translation.z) > 0.01)
            {
              reason = "odom_not_planar";
            } else {
              SE2 odom_base{t.transform.translation.x, t.transform.translation.y, yaw};
              SE2 map_base = compose(compose(map_alignment_, o.sample.pose), base_offset_);
              SE2 correction = compose(map_base, inverse(odom_base));
              reason = gate_.update(correction);
              if (reason.empty() || reason == "persistent_correction_accepted") {
                geometry_msgs::msg::PoseStamped p;
                p.header.frame_id = "map"; p.header.stamp = o.stamp;
                p.pose.position.x = map_base.x; p.pose.position.y = map_base.y;
                p.pose.orientation.z = std::sin(map_base.angle / 2);
                p.pose.orientation.w = std::cos(map_base.angle / 2);
                geometry_msgs::msg::TransformStamped tf;
                tf.header = p.header; tf.child_frame_id = "odom";
                tf.transform.translation.x = correction.x;
                tf.transform.translation.y = correction.y;
                tf.transform.rotation.z = std::sin(correction.angle / 2);
                tf.transform.rotation.w = std::cos(correction.angle / 2);
                if (tf_publish_mode_ == "measurement_time") {
                  tf2_msgs::msg::TFMessage message; message.transforms.push_back(tf);
                  tf_->publish(message); last_tf_stamp_ = o.stamp;
                }
                pose_->publish(p);
                ++updates_; fresh_ = true; last_sample_ = o.stamp;
                accepted_ = o; accepted_time_gap_ = time_gap_;
                accepted_message_ = reason.empty() ? "receive_time_approximation" : reason;
              }
            }
          } catch (const tf2::TransformException &) {
            reason = "tf_lookup_failed"; retry = true;
          }
        }
      }
    }
    pending_reason_ = retry ? reason : "";
    // Waiting for a newer interpolation endpoint is normal. It must not replace
    // the accepted measurement or extend its original lifetime.
    const bool waiting_with_accepted = retry && reason == "waiting_for_odom_bracket" &&
      accepted_ && fresh_;
    if (!waiting_with_accepted) {error_ = reason;}
    if (!reason.empty() && reason != "persistent_correction_accepted" && !waiting_with_accepted) {
      fresh_ = false;
      if (!retry && reason != "jump_quarantined") {gate_.interrupt();}
    }
    if (!retry) {pending_.reset();}
  }
  const double age = accepted_ ? (current - accepted_->stamp).seconds() : -1;
  const double steady_age = accepted_ ?
    std::chrono::duration<double>(Clock::now() - accepted_->received).count() : -1;
  const double odom_age = odom_stamps_.empty() ? -1 : (current - odom_stamps_.back()).seconds();
  const bool source_fresh = accepted_ && age >= 0 && age < max_age_ &&
    steady_age >= 0 && steady_age < max_age_;
  const bool odom_fresh = odom_age >= 0 && odom_age < odom_age_;
  const bool usable = fresh_ && !conflict_ && source_fresh && odom_fresh;
  if (tf_publish_mode_ == "bounded_odom_hold" && usable && gate_.accepted &&
    (!last_tf_stamp_ || odom_stamps_.back() > *last_tf_stamp_))
  {
    // map->odom is an estimated correction, not a new absolute pose measurement.
    // Support only a received odometry timestamp, never now() or a future stamp.
    // Source freshness above still expires at the ORIGINAL accepted pose time.
    const auto & correction = *gate_.accepted;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map"; tf.child_frame_id = "odom";
    tf.header.stamp = odom_stamps_.back();
    tf.transform.translation.x = correction.x; tf.transform.translation.y = correction.y;
    tf.transform.rotation.z = std::sin(correction.angle / 2);
    tf.transform.rotation.w = std::cos(correction.angle / 2);
    tf2_msgs::msg::TFMessage message; message.transforms.push_back(tf); tf_->publish(message);
    last_tf_stamp_ = odom_stamps_.back();
  }
  diagnostic_msgs::msg::DiagnosticStatus s;
  s.name = "/seer_localization/map_to_odom"; s.hardware_id = "seer_read_only";
  s.level = usable ? 1 : 2;
  s.message = conflict_ ? "competing_odom_parent_publisher_restart_required" :
    usable ? accepted_message_ : !fresh_ && !error_.empty() ? error_ :
    !source_fresh ? "stale_localization_no_republish" : "odom_stale_or_future";
  auto add = [&](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue v; v.key = key; v.value = value; s.values.push_back(v);
    };
  auto number = [&](const std::string & k, double v) {add(k, std::to_string(v));};
  add("time_mode", "1004 receive time; internal sample age unknown; create_on ignored");
  add("coordinate_status", assumptions_ ? "UNVERIFIED_ASSUMPTION" : "configured_confirmed");
  add("map", usable ? accepted_->sample.map : map_);
  number("reloc_status", usable ? accepted_->sample.status : status_);
  number("confidence", usable ? accepted_->sample.confidence.value_or(-1) : confidence_);
  add("pending_reason", pending_reason_);
  add("tf_publish_mode", tf_publish_mode_);
  number("tf_support_stamp_sec", last_tf_stamp_ ? last_tf_stamp_->seconds() : -1);
  number("accepted_pose_stamp_sec", accepted_ ? accepted_->stamp.seconds() : -1);
  number("query_pose_receive_age_sec", last_received_ == Clock::time_point{} ? -1 :
    std::chrono::duration<double>(Clock::now() - last_received_).count());
  number("latest_odom_age_sec", odom_stamps_.empty() ? -1 :
    (current - odom_stamps_.back()).seconds());
  // Exact-time tf2 interpolation: the returned transform is stamped at the pose time.
  number("pose_odom_stamp_difference_sec", usable ? 0 : -1);
  number("response_age_sec", age);
  number("query_bundle_latency_sec", usable ? accepted_->latency : latency_);
  number("odom_bracket_sec", usable ? accepted_time_gap_ : time_gap_);
  add("tf_available", usable ? "true" : "false");
  number("updates", updates_); number("queries", attempts_); number("query_failures", failures_);
  number("jump_pending_samples", gate_.count);
  add("device_sample_age", "unknown; repeated cached poses cannot be detected");
  diagnostic_msgs::msg::DiagnosticArray a; a.header.stamp = current; a.status.push_back(s);
  diagnostics_->publish(a);
}
}
