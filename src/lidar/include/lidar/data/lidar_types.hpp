#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lidar
{
using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;
constexpr double kRadiansPerDegree = 0.017453292519943295;

struct Beam
{
    std::optional<double> angle;  // rad, in the source scanner frame
    std::optional<double> distance;  // m
    std::optional<double> intensity;
    std::optional<bool> valid;
    bool is_virtual = false;
};

struct InstallInfo
{
    std::optional<double> x, y, z, yaw;  // raw source values; yaw units unverified
    std::optional<bool> upside;
};

struct DeviceScan
{
    std::string name;
    std::optional<int> id;
    std::optional<uint64_t> source_stamp;
    std::string source_frame;
    double range_min = 0;
    double range_max = 0;
    double scan_frequency = 0;
    InstallInfo install;
    std::vector<Beam> beams;
};

struct ScanBatch
{
    std::vector<DeviceScan> scans;
    Deadline request_started, received;
};

enum class ReadStatus { Ok, Timeout, ConnectionError, ProtocolError, ApiError };

struct ReadResult
{
    ReadStatus status = ReadStatus::Ok;
    std::string error;
    ScanBatch batch;
};
}  // namespace lidar
