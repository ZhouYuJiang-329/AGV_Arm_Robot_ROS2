#pragma once
#include <cmath>
#include <optional>
#include <string>
#include "robot_move/impl/agv/seer/seer_types.hpp"
namespace robot_move::localization
{
using SE2 = seer::Pose;
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
inline bool finite(const SE2 & p)
{return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.angle);}
inline SE2 compose(const SE2 & a, const SE2 & b)
{
  return {a.x + std::cos(a.angle) * b.x - std::sin(a.angle) * b.y,
    a.y + std::sin(a.angle) * b.x + std::cos(a.angle) * b.y, wrap(a.angle + b.angle)};
}
inline SE2 inverse(const SE2 & a)
{
  return {-std::cos(a.angle) * a.x - std::sin(a.angle) * a.y,
    std::sin(a.angle) * a.x - std::cos(a.angle) * a.y, wrap(-a.angle)};
}
inline bool near(const SE2 & a, const SE2 & b, double meters, double radians)
{
  return std::hypot(a.x - b.x, a.y - b.y) <= meters &&
         std::abs(wrap(a.angle - b.angle)) <= radians;
}
struct Sample
{
  SE2 pose;
  std::optional<double> confidence;
  std::optional<int> method;
  int status = -1;
  std::string map, md5;
};
// Quality gate operates on map->odom corrections, never writes odometry.
struct Gate
{
  std::string expected_map, expected_md5;
  double min_confidence = 0.5, jump_m = 0.5, jump_rad = 0.5;
  double stable_m = 0.1, stable_rad = 0.1;
  int recovery_samples = 3;  // 0: latch until process restart after inspection.
  bool require_confidence = true;
  std::optional<SE2> accepted, pending;
  std::optional<std::string> active_md5;
  int count = 0;
  void interrupt() {pending.reset(); count = 0;}
  std::string quality(const Sample & s)
  {
    if (!finite(s.pose)) {return "invalid_pose";}
    if (s.status != 1) {return "localization_not_success";}
    if (s.map.empty() || s.map != expected_map ||
      (!expected_md5.empty() && s.md5 != expected_md5)) {return "map_mismatch";}
    if ((require_confidence && !s.confidence) || (s.confidence &&
      (!std::isfinite(*s.confidence) || *s.confidence < min_confidence || *s.confidence > 1)))
    {return "confidence_missing_or_low";}
    if (s.method && *s.method == 3) {return "odometry_only_localization";}
    if (active_md5 && *active_md5 != s.md5) {return "map_content_changed_restart_required";}
    active_md5 = s.md5;
    return "";
  }
  std::string update(const SE2 & correction)
  {
    if (!finite(correction)) {interrupt(); return "invalid_transform";}
    if (!accepted || near(*accepted, correction, jump_m, jump_rad)) {
      accepted = correction; interrupt(); return "";
    }
    if (!pending || !near(*pending, correction, stable_m, stable_rad)) {
      pending = correction; count = 1;
    } else {++count;}
    if (recovery_samples > 0 && count >= recovery_samples) {
      accepted = correction; interrupt(); return "persistent_correction_accepted";
    }
    return "jump_quarantined";
  }
};
// Receive-time approximation: these bounds constrain transport/ROS alignment,
// not unknown internal controller localization latency.
inline std::string timing(
  double age, double latency, double span,
  double max_age, double max_latency, double max_span)
{
  if (!std::isfinite(age) || age < 0) {return "clock_backward";}
  if (age > max_age) {return "stale_sample";}
  if (!std::isfinite(latency) || latency < 0 || latency > max_latency)
  {return "query_latency_exceeded";}
  if (!std::isfinite(span) || span < 0 || span > max_span) {return "odom_time_gap";}
  return "";
}
}
