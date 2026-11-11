#pragma once
#include <memory>
#include <string>
#include "robot_move/impl/agv/seer/seer_transport.hpp"

namespace robot_move::seer
{
struct Velocity { double vx = 0, vy = 0, w = 0; };
// Synchronous, exclusively owned by the acquisition thread. No device sample clock
// has been verified for API 1005; receive time is supplied by the caller.
class VelocitySource
{
public:
  virtual ~VelocitySource() = default;
  virtual bool connect() = 0;
  virtual bool read(Velocity & value, Deadline deadline) = 0;
  virtual void disconnect() = 0;
};
class SeerVelocitySource final : public VelocitySource
{
public:
  explicit SeerVelocitySource(std::string address);
  bool connect() override;
  bool read(Velocity & value, Deadline deadline) override;
  void disconnect() override;
  static bool parse(const std::string & body, Velocity & value);

private:
  friend struct VelocitySourceTestAccess;
  std::string address_;
  Transport transport_;
};
}
