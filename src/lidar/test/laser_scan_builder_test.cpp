#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "lidar/processing/laser_scan_builder.hpp"

void require(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}

int main()
{
    try
    {
        lidar::ScanConfig config{"laser_front_link", -1, 1, 0.5};
        lidar::DeviceScan source;
        source.range_min = 0.1;
        source.range_max = 40;
        source.scan_frequency = 30;
        source.beams = {
            {0.0, 3.0, 10.0, true, false},
            {0.1, 2.0, 20.0, true, false},  // collision, nearest wins
            {0.2, 4.0, 99.0, true, false},
            {-1.0, 1.0, std::nullopt, true, false},
            {1.0, 5.0, 30.0, true, false},
            {std::nullopt, 1.0, 1.0, true, false},
            {0.5, std::nullopt, 1.0, true, false},
            {0.5, 0.0, 1.0, true, false},
            {0.5, 1.0, 1.0, std::nullopt, false},
            {0.5, 1.0, 1.0, true, true},
            {0.5, std::numeric_limits<double>::infinity(), 1.0, true, false},
            {1.001, 1.0, 1.0, true, false},  // outside: do not clamp to edge
        };
        const auto result = lidar::buildLaserScan(source, config);
        const auto& scan = result.message;
        require(scan.ranges.size() == 5 && scan.intensities.size() == 5,
                "grid dimensions incorrect");
        require(scan.ranges[0] == 1 && scan.ranges[2] == 2 && scan.ranges[4] == 5,
                "geometry or nearest-distance handling incorrect");
        require(scan.intensities[2] == 20 && std::isnan(scan.intensities[0]),
                "intensity does not match chosen distance");
        require(std::isnan(scan.ranges[1]) && std::isnan(scan.ranges[3]),
                "invalid measurements fabricated");
        require(result.valid_points == 3 && result.collisions == 2 &&
                result.outside_points == 1 && result.rejected_points == 6,
                "conversion counters incorrect");
        require(scan.angle_max == scan.angle_min + 4 * scan.angle_increment &&
                scan.time_increment == 0, "scan geometry/timing inconsistent");
        for (auto invalid : {lidar::ScanConfig{"", -1, 1, 0.5},
                             lidar::ScanConfig{"laser", -1, 1, 0},
                             lidar::ScanConfig{"laser", -1, 1, 1e-12},
                             lidar::ScanConfig{"laser", -1, 1, 0.3}})
        {
            bool rejected = false;
            try { lidar::validateScanConfig(invalid); }
            catch (const std::invalid_argument&) { rejected = true; }
            require(rejected, "invalid config accepted");
        }
        std::cout << "laser_scan_builder_test passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
