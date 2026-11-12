#pragma once
#include <cstdint>
#include <optional>
#include "robot_move/impl/agv/seer/seer_velocity_source.hpp"
namespace robot_move::odometry
{
using seer::Velocity;
// Position of base origin in Seer coordinates; yaw is base orientation in Seer.
Velocity toBase(Velocity source, double x, double y, double yaw);
// Inverse of toBase, including the reference-point angular velocity term.
Velocity toSeer(Velocity base, double x, double y, double yaw);
class Integrator
{
public:
  explicit Integrator(double max_interval);
    // Optional sample id is for future verified device clocks and fake sources.
    // Current API 1005 supplies none. Equal values are not evidence of duplication.
  bool accept(
    Velocity velocity, double steady_seconds,
    std::optional<uint64_t> sample_id = std::nullopt);
  void gap();
  double x = 0, y = 0, yaw = 0, dt = 0;
  uint64_t skipped = 0, duplicates = 0;
  bool degraded = false;

private:
  double max_interval_;
  std::optional<double> time_;
  std::optional<uint64_t> sample_;
  Velocity previous_;
};
}
