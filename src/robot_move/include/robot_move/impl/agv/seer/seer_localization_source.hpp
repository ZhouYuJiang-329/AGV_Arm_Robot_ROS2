#pragma once
#include "robot_move/impl/agv/seer/seer_transport.hpp"
#include "robot_move/localization/core.hpp"
#include <functional>
namespace robot_move::seer
{
class LocalizationSource
{
public:
  virtual ~LocalizationSource() = default;
  virtual bool connect() = 0;
  // on_pose_received is called immediately after 1004 completes, before post-checks.
  virtual bool read(
    localization::Sample &, Deadline,
    const std::function<void()> & on_pose_received) = 0;
  virtual void disconnect() = 0;
};
class SeerLocalizationSource final : public LocalizationSource
{
public:
  explicit SeerLocalizationSource(std::string address)
  : address_(std::move(address)) {}
  bool connect() override {return transport_.connect(address_, {ApiChannel::Status});}
  void disconnect() override {transport_.disconnect();}
  bool read(localization::Sample &, Deadline, const std::function<void()> &) override;
  static bool parsePose(const std::string &, localization::Sample &);
  static bool parseStatus(const std::string &, int &);
  static bool parseMap(const std::string &, std::string &, std::string &);

private:
  friend struct LocalizationSourceTestAccess;
  std::string address_;
  Transport transport_;
};
}
