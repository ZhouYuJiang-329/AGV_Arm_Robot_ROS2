#include "robot_move/odometry/integrator.hpp"
#include <cmath>
#include <stdexcept>
namespace robot_move::odometry
{
Velocity toBase(Velocity v, double x, double y, double yaw)
{
    // v(base origin) = v(Seer origin) + omega cross r, then rotate axes.
  const double vx = v.vx - v.w * y, vy = v.vy + v.w * x;
  return {std::cos(yaw) * vx + std::sin(yaw) * vy,
    -std::sin(yaw) * vx + std::cos(yaw) * vy, v.w};
}
Integrator::Integrator(double interval)
: max_interval_(interval)
{
  if (!std::isfinite(interval) || interval <= 0) {throw std::invalid_argument("max interval");}
}
void Integrator::gap()
{
  time_.reset();
  sample_.reset();
  dt = 0;
  ++skipped;
  degraded = true;
}
bool Integrator::accept(Velocity v, double time, std::optional<uint64_t> sample)
{
  if (!std::isfinite(time) || !std::isfinite(v.vx) || !std::isfinite(v.vy) || !std::isfinite(v.w))
  {gap(); return false;}
  if (sample && sample_ && *sample == *sample_) {++duplicates; return false;}
  const bool reset = sample && sample_ && *sample < *sample_;
  dt = time_ ? time - *time_ : 0;
  if (reset || (time_ && (dt <= 0 || dt > max_interval_))) {gap();}
  if (time_) {
        // Average endpoint body velocities; exact SE(2) exponential for that
        // constant twist over the receive interval. Small-angle sinc is stable.
    const double vx = (previous_.vx + v.vx) / 2, vy = (previous_.vy + v.vy) / 2;
    const double angle = (previous_.w + v.w) / 2 * dt;
    const double half = angle / 2;
    const double sinc = std::abs(half) < 1e-6 ? 1 - half * half / 6 : std::sin(half) / half;
    const double c = std::cos(yaw + half), s = std::sin(yaw + half);
    x += dt * sinc * (vx * c - vy * s);
    y += dt * sinc * (vx * s + vy * c);
    yaw = std::remainder(yaw + angle, 2 * std::acos(-1.0));
  }
  time_ = time;
  sample_ = sample;
  previous_ = v;
  return true;
}
}
