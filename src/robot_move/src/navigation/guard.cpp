#include "robot_move/navigation/guard.hpp"
namespace robot_move::navigation
{
void Config::validate() const
{
  for (double v : {command_timeout, linear_limit, angular_limit, stopped_linear,
      stopped_angular, stopped_hold})
  {
    if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("invalid guard thresholds");}
  }
  if (request_ms < 1 || request_ms > 1000 || duration_ms < 1 || duration_ms > 5000 ||
    duration_ms <= command_timeout * 1000 + request_ms + 20 ||
    stop_attempts < 1 || stop_attempts > 3)
  {throw std::invalid_argument("duration must exceed cmd timeout + request deadline + 20ms");}
}
Guard::Guard(Config c)
: config(c) {config.validate();}
void Guard::require(const std::string & name, double limit)
{
  if (name.empty() || !std::isfinite(limit) || limit <= 0 || inputs.count(name))
  {throw std::invalid_argument("invalid/duplicate required input");}
  inputs[name].limit = limit;
}
bool Guard::input(const std::string & name, double stamp, double ros_now, bool valid, Time now)
{
  auto & i = inputs.at(name);
  const double age = ros_now - stamp;
  if (!valid || !std::isfinite(stamp) || !std::isfinite(age) || stamp <= 0 ||
    age < 0 || age >= i.limit)
  {
    i.valid = false; i.reason = "invalid/stale/future"; return false;
  }
  if (stamp <= i.last_stamp) {return false;}
  if (name == "odom" && (!i.valid || now >= i.expires)) {low_since_.reset();}
  i.last_stamp = stamp; i.valid = true; i.reason = "fresh";
  i.expires = now + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(i.limit - age));
  return true;
}
bool Guard::healthy(Time now) const
{
  if (!config.motion_enabled || !config.freshness_verified || inputs.empty()) {return false;}
  for (const auto & item : inputs) {
    if (!item.second.valid || now >= item.second.expires) {return false;}
  }
  return true;
}
void Guard::feedback(double linear, double angular, Time now, bool valid)
{
  if (!valid || !std::isfinite(linear) || !std::isfinite(angular) ||
    linear > config.stopped_linear || std::abs(angular) > config.stopped_angular)
  {
    low_since_.reset(); return;
  }
  if (now >= stopped_at_ && !low_since_) {low_since_ = now;}
}
bool Guard::confirmed(Time now) const
{
  auto i = inputs.find("odom");
  return state != "Active" && config.freshness_verified && low_since_ &&
         seconds(now, *low_since_) >= config.stopped_hold && i != inputs.end() &&
         i->second.valid && now < i->second.expires;
}
void Guard::stop(const std::string & cause, bool fault, Time now)
{
  pending_.reset();
  if (state != "Fault" && state != "StopRequested") {low_since_.reset(); stopped_at_ = now;}
  if (state != "Fault") {state = fault ? "Fault" : "StopRequested"; reason = cause;}
  // One bounded stop episode; repeated zeros/bad commands cannot reset retry budget.
  if (!zero_pending_ && !zero_sent) {zero_pending_ = true; zero_count = 0; zero_ack = false;}
}
void Guard::command(Velocity v, bool valid, Time now)
{
  valid = valid && std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.w) &&
    std::hypot(v.x, v.y) <= config.linear_limit && std::abs(v.w) <= config.angular_limit;
  if (!valid) {stop("invalid_command", true, now); return;}
  if (v.x == 0 && v.y == 0 && v.w == 0) {
    // An idle upstream zero is not a request to take control of the chassis.
    // API2010 can cancel an existing controller task even at zero velocity.
    // After accepted motion, keep the normal stop/ACK/recovery contract.
    if (!last_command && (state == "WaitingForInputs" || state == "Ready")) {return;}
    last_command = now; stop("explicit_zero", false, now); return;
  }
  if (state == "Fault" || state == "StopRequested" || !healthy(now)) {return;}
  last_command = now; pending_ = v; state = "Active";
  reason = "new_command"; low_since_.reset(); zero_sent = zero_ack = false; zero_count = 0;
}
void Guard::tick(Time now)
{
  if (state == "Active") {
    if (!healthy(now)) {
      std::string cause = "required_input_unhealthy";
      for (const auto & item : inputs) {
        if (!item.second.valid || now >= item.second.expires) {cause += ":" + item.first; break;}
      }
      stop(cause, true, now);
    } else if (last_command && seconds(now, *last_command) >= config.command_timeout)
    {stop("command_timeout", false, now);}
  } else if (state == "WaitingForInputs" || state == "Ready") {
    state = healthy(now) ? "Ready" : "WaitingForInputs";
  }
}
bool Guard::recover(Time now)
{
  if (!healthy(now) || !zero_ack || !confirmed(now) || zero_pending_) {return false;}
  state = "Ready"; reason = "recovered_requires_new_command"; pending_.reset();
  zero_sent = false; last_command.reset(); return true;
}
std::optional<Velocity> Guard::next(Time now)
{
  tick(now);
  if (zero_pending_) {zero_pending_ = false; zero_sent = true; ++zero_count; return Velocity{};}
  auto value = pending_; pending_.reset(); return value;
}
void Guard::result(Velocity v, bool ok, Time now)
{
  if (v.x == 0 && v.y == 0 && v.w == 0) {
    zero_ack = ok;
    if (!ok) {
      state = "Fault";
      if (reason.find("zero_request_failed") == std::string::npos) {
        reason += ";zero_request_failed";
      }
      if (zero_count < static_cast<unsigned>(config.stop_attempts)) {zero_pending_ = true;}
    }
  } else if (!ok) {stop("control_request_failed", true, now);}
}
}
