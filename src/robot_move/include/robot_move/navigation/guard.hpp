#pragma once
#include <chrono>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>

namespace robot_move::navigation
{
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
inline double seconds(Time a, Time b) {return std::chrono::duration<double>(a - b).count();}
struct Velocity {double x = 0, y = 0, w = 0;};
struct Config
{
  double command_timeout = 0.5, linear_limit = 0.15, angular_limit = 0.2;
  double stopped_linear = 0.01, stopped_angular = 0.02, stopped_hold = 0.5;
  int duration_ms = 800, request_ms = 150, stop_attempts = 2;
  bool motion_enabled = false, freshness_verified = false;
  void validate() const;
};
struct Input
{
  double limit = 1, last_stamp = 0;
  Time expires{};
  bool valid = false;
  std::string reason = "missing";
};
// Caller serializes access. No ROS clock participates in watchdog/hold durations.
class Guard
{
public:
  explicit Guard(Config config);
  Config config;
  std::map<std::string, Input> inputs;
  // CommandedStop is an ordinary acknowledged zero with live input supervision.
  // Fault/StopRequested remain latched and require explicit recovery.
  std::string state = "WaitingForInputs", reason = "startup";
  std::optional<Time> last_command;
  bool zero_ack = false, zero_sent = false;
  unsigned zero_count = 0;
  void require(const std::string & name, double limit);
  bool input(const std::string & name, double stamp, double ros_now, bool valid, Time now);
  void feedback(double linear, double angular, Time now, bool valid);
  bool healthy(Time now) const;
  bool confirmed(Time now) const;
  void command(Velocity value, bool valid, Time now);
  void stop(const std::string & cause, bool fault, Time now);
  bool recover(Time now);
  void tick(Time now);
  std::optional<Velocity> next(Time now);
  void result(Velocity value, bool ok, Time now);

private:
  std::optional<Velocity> pending_;
  std::optional<Time> low_since_;
  Time stopped_at_{};
  bool zero_pending_ = false;
};
}
