#include "lidar/processing/laser_scan_builder.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace lidar
{
size_t validateScanConfig(const ScanConfig& config)
{
    if (config.frame_id.empty() || config.frame_id.front() == '/' ||
        !std::isfinite(config.angle_min) || !std::isfinite(config.angle_max) ||
        !std::isfinite(config.angle_increment) || config.angle_increment <= 0 ||
        config.angle_max <= config.angle_min ||
        std::abs(config.angle_min) > std::numeric_limits<float>::max() ||
        std::abs(config.angle_max) > std::numeric_limits<float>::max() ||
        config.angle_increment > std::numeric_limits<float>::max() ||
        static_cast<float>(config.angle_increment) == 0)
        throw std::invalid_argument("invalid scan frame or angle grid");
    const double steps = (config.angle_max - config.angle_min) / config.angle_increment;
    if (!std::isfinite(steps) || steps < 1 || steps > 99999 ||
        std::abs(steps - std::round(steps)) > 1e-6)
        throw std::invalid_argument("scan grid must have 2..100000 aligned bins");
    return static_cast<size_t>(std::llround(steps)) + 1;
}

BuiltScan buildLaserScan(const DeviceScan& source, const ScanConfig& config)
{
    const auto count = validateScanConfig(config);
    if (!std::isfinite(source.range_min) || !std::isfinite(source.range_max) ||
        source.range_min < 0 || source.range_max <= source.range_min ||
        source.range_max > std::numeric_limits<float>::max())
        throw std::invalid_argument("invalid source range");
    BuiltScan output;
    auto& scan = output.message;
    scan.header.frame_id = config.frame_id;
    scan.angle_min = static_cast<float>(config.angle_min);
    scan.angle_increment = static_cast<float>(config.angle_increment);
    scan.angle_max = scan.angle_min + static_cast<float>(count - 1) * scan.angle_increment;
    scan.range_min = static_cast<float>(source.range_min);
    scan.range_max = static_cast<float>(source.range_max);
    // Sensor nominal period only; not the TCP request period or per-beam timing.
    if (std::isfinite(source.scan_frequency) &&
        source.scan_frequency > 1.0 / std::numeric_limits<float>::max())
        scan.scan_time = static_cast<float>(1.0 / source.scan_frequency);
    scan.time_increment = 0;
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    scan.ranges.assign(count, nan);
    scan.intensities.assign(count, nan);
    for (const auto& beam : source.beams)
    {
        if (!beam.valid.value_or(false) || beam.is_virtual || !beam.angle || !beam.distance ||
            !std::isfinite(*beam.angle) || !std::isfinite(*beam.distance) ||
            *beam.distance < source.range_min || *beam.distance > source.range_max)
        {
            ++output.rejected_points;
            continue;
        }
        if (*beam.angle < config.angle_min - 1e-9 || *beam.angle > config.angle_max + 1e-9)
        {
            ++output.outside_points;
            continue;
        }
        const auto index = static_cast<size_t>(std::llround(
            (*beam.angle - config.angle_min) / config.angle_increment));
        if (index >= count)
        {
            ++output.outside_points;
            continue;
        }
        if (std::isfinite(scan.ranges[index]))
            ++output.collisions;
        else
            ++output.valid_points;
        if (!std::isfinite(scan.ranges[index]) || *beam.distance < scan.ranges[index])
        {
            scan.ranges[index] = static_cast<float>(*beam.distance);
            scan.intensities[index] = beam.intensity && std::isfinite(*beam.intensity) &&
                                     std::abs(*beam.intensity) <= std::numeric_limits<float>::max()
                                         ? static_cast<float>(*beam.intensity) : nan;
        }
    }
    return output;
}
}  // namespace lidar
