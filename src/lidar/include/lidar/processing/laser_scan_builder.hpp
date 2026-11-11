#pragma once

#include <cstddef>
#include <string>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "lidar/data/lidar_types.hpp"

namespace lidar
{
struct ScanConfig
{
    std::string frame_id;
    double angle_min = -135 * kRadiansPerDegree;
    double angle_max = 135 * kRadiansPerDegree;
    double angle_increment = 0.5 * kRadiansPerDegree;
};

struct BuiltScan
{
    sensor_msgs::msg::LaserScan message;
    size_t valid_points = 0, rejected_points = 0, outside_points = 0, collisions = 0;
};

/// Throws invalid_argument for invalid geometry; no I/O or clock access.
size_t validateScanConfig(const ScanConfig& config);
BuiltScan buildLaserScan(const DeviceScan& source, const ScanConfig& config);
}  // namespace lidar
