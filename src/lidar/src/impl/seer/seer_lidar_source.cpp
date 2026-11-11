#include "lidar/impl/seer/seer_lidar_source.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>
#include "robot_move/interface/cJSON.h"

namespace lidar::seer
{
namespace
{
const cJSON* field(const cJSON* object, const char* key)
{
    return cJSON_GetObjectItemCaseSensitive(object, key);
}

std::optional<double> number(const cJSON* object, const char* key)
{
    const auto* value = field(object, key);
    if (cJSON_IsNumber(value) && std::isfinite(value->valuedouble))
        return value->valuedouble;
    return std::nullopt;
}

std::optional<bool> boolean(const cJSON* object, const char* key)
{
    const auto* value = field(object, key);
    if (cJSON_IsBool(value))
        return cJSON_IsTrue(value);
    return std::nullopt;
}

std::string string(const cJSON* object, const char* key)
{
    const auto* value = field(object, key);
    return cJSON_IsString(value) ? value->valuestring : "";
}

std::optional<uint64_t> timestamp(const cJSON* header)
{
    const auto text = string(header, "data_nsec");
    uint64_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || parsed.ec != std::errc() || parsed.ptr != text.data() + text.size())
        return std::nullopt;
    return value;
}
}  // namespace

SeerLidarSource::SeerLidarSource(std::string address)
    : SeerLidarSource(std::move(address), std::make_unique<robot_move::seer::Transport>())
{
}

SeerLidarSource::SeerLidarSource(
    std::string address, std::unique_ptr<robot_move::seer::Transport> transport)
    : address_(std::move(address)), transport_(std::move(transport))
{
    if (!transport_)
        throw std::invalid_argument("SeerLidarSource requires a transport");
}

bool SeerLidarSource::connect()
{
    return transport_->connect(address_, {robot_move::seer::ApiChannel::Status});
}

void SeerLidarSource::disconnect()
{
    transport_->disconnect();
}

ReadResult SeerLidarSource::readBatch(Deadline deadline)
{
    ReadResult result;
    result.batch.request_started = Clock::now();
    std::string raw;
    if (!transport_->request(robot_move::seer::ApiChannel::Status, 1009, "", raw, deadline))
    {
        using Error = robot_move::seer::Transport::Error;
        const auto error = transport_->lastError();
        result.status = error == Error::Timeout ? ReadStatus::Timeout :
                        error == Error::Protocol ? ReadStatus::ProtocolError :
                        ReadStatus::ConnectionError;
        result.error = "1009 transport request failed";
        return result;
    }
    result.batch.received = Clock::now();
    auto malformed = [&](const std::string& reason)
    {
        transport_->closeChannel(robot_move::seer::ApiChannel::Status);
        result.status = ReadStatus::ProtocolError;
        result.error = reason;
        result.batch.scans.clear();
        return result;
    };
    if (raw.find('\0') != std::string::npos)
        return malformed("embedded NUL in JSON");
    std::unique_ptr<cJSON, decltype(&cJSON_Delete)> json(
        cJSON_ParseWithOpts(raw.c_str(), nullptr, true), cJSON_Delete);
    if (!cJSON_IsObject(json.get()))
        return malformed("response must be a complete JSON object");
    const auto* code = field(json.get(), "ret_code");
    if (code && (!number(json.get(), "ret_code")))
        return malformed("ret_code must be numeric");
    if (code && code->valuedouble != 0)
    {
        result.status = ReadStatus::ApiError;
        result.error = "1009 API error: " + std::to_string(code->valuedouble) + " " +
                       string(json.get(), "err_msg");
        return result;
    }
    const auto* lasers = field(json.get(), "lasers");
    if (!cJSON_IsArray(lasers))
        return malformed("lasers must be an array");
    std::set<std::string> names;
    for (const cJSON* laser = lasers->child; laser; laser = laser->next)
    {
        if (!cJSON_IsObject(laser))
            return malformed("laser entry must be an object");
        const auto* info = field(laser, "device_info");
        const auto* header = field(laser, "header");
        const auto* beams = field(laser, "beams");
        DeviceScan scan;
        scan.name = string(info, "device_name");
        const auto id = number(info, "id");
        if (id && *id >= 0 && *id <= std::numeric_limits<int>::max() && std::floor(*id) == *id)
            scan.id = static_cast<int>(*id);
        auto min = number(info, "min_range"), max = number(info, "max_range");
        if (scan.name.empty() || !names.insert(scan.name).second || !min || !max ||
            *min < 0 || *max <= *min || !cJSON_IsArray(beams))
            return malformed("invalid device identity, range or beams");
        scan.range_min = *min;
        scan.range_max = *max;
        scan.scan_frequency = number(info, "scan_freq").value_or(0);
        scan.source_stamp = timestamp(header);
        scan.source_frame = string(header, "frame_id");
        const auto* install = field(laser, "install_info");
        scan.install = {number(install, "x"), number(install, "y"), number(install, "z"),
                        number(install, "yaw"), boolean(install, "upside")};
        for (const cJSON* beam = beams->child; beam; beam = beam->next)
        {
            Beam value;
            value.angle = number(beam, "angle");
            if (value.angle)
                *value.angle *= kRadiansPerDegree;
            value.distance = number(beam, "dist");
            value.intensity = number(beam, "rssi");
            value.valid = boolean(beam, "valid");
            // Unknown virtual-point flag types cannot become real measurements.
            const auto* virtual_flag = field(beam, "is_virtual");
            value.is_virtual = virtual_flag && !boolean(beam, "is_virtual").has_value()
                                   ? true : boolean(beam, "is_virtual").value_or(false);
            scan.beams.push_back(value);
        }
        result.batch.scans.push_back(std::move(scan));
    }
    return result;
}
}  // namespace lidar::seer
