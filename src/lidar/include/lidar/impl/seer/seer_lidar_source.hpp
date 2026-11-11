#pragma once

#include <memory>
#include <string>
#include "lidar/interface/lidar_source.hpp"
#include "robot_move/impl/agv/seer/seer_transport.hpp"

namespace lidar::seer
{
class SeerLidarSource final : public LidarSource
{
public:
    explicit SeerLidarSource(std::string address);
    /// Inject an already connected transport for local socket tests.
    SeerLidarSource(std::string address,
                   std::unique_ptr<robot_move::seer::Transport> transport);
    bool connect() override;
    ReadResult readBatch(Deadline deadline) override;
    void disconnect() override;

private:
    std::string address_;
    std::unique_ptr<robot_move::seer::Transport> transport_;
};
}  // namespace lidar::seer
