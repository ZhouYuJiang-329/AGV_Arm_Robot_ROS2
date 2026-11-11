#pragma once

#include "lidar/data/lidar_types.hpp"

namespace lidar
{
/// Synchronous, single-owner source. All I/O is bounded; no internal worker threads.
class LidarSource
{
public:
    virtual ~LidarSource() = default;
    virtual bool connect() = 0;
    virtual ReadResult readBatch(Deadline deadline) = 0;
    virtual void disconnect() = 0;
};
}  // namespace lidar
