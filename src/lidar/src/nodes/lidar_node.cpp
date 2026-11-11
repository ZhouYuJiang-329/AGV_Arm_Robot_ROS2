#include "lidar/nodes/lidar_node.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "lidar/impl/seer/seer_lidar_source.hpp"

namespace lidar
{
namespace
{
using Status = diagnostic_msgs::msg::DiagnosticStatus;
void value(Status& status, const std::string& key, const std::string& text)
{
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = text;
    status.values.push_back(std::move(entry));
}
}  // namespace

LidarNode::LidarNode(const rclcpp::NodeOptions& options, std::unique_ptr<LidarSource> source)
    : Node("lidar_node", options), source_(std::move(source))
{
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    const auto address = declare_parameter<std::string>("connection", "", startup);
    const auto rate = declare_parameter<double>("request_rate_hz", 5.0, startup);
    const auto timeout = declare_parameter<int>("request_timeout_ms", 1000, startup);
    stale_seconds_ = declare_parameter<double>("stale_timeout_sec", 1.0, startup);
    const auto stamp_mode = declare_parameter<std::string>("stamp_mode", "receive_time", startup);
    if ((!source_ && address.empty()) || !std::isfinite(rate) || rate <= 0 || rate > 100 ||
        timeout <= 0 || timeout > 1000 || !std::isfinite(stale_seconds_) || stale_seconds_ <= 0 ||
        stamp_mode != "receive_time")
        throw std::invalid_argument("invalid lidar connection, timing or stamp_mode");
    timeout_ = std::chrono::milliseconds(timeout);
    period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / rate));
    auto names = declare_parameter<std::vector<std::string>>(
        "devices", {"laser_front", "laser_back"}, startup);
    std::set<std::string> unique_names, topics, frames;
    for (const auto& name : names)
    {
        if (name.empty() || name.find('.') != std::string::npos || !unique_names.insert(name).second)
            throw std::invalid_argument("invalid or duplicate lidar device name");
        const auto prefix = "device." + name + ".";
        if (!declare_parameter<bool>(prefix + "enabled", true, startup))
            continue;
        const auto label = name == "laser_front" ? "front" : name == "laser_back" ? "back" : name;
        const auto topic = declare_parameter<std::string>(prefix + "topic", "/scan_" + label, startup);
        ScanConfig config;
        config.frame_id = declare_parameter<std::string>(prefix + "frame_id", name + "_link", startup);
        config.angle_min = declare_parameter<double>(prefix + "angle_min_deg", -135.0, startup) *
                           kRadiansPerDegree;
        config.angle_max = declare_parameter<double>(prefix + "angle_max_deg", 135.0, startup) *
                           kRadiansPerDegree;
        config.angle_increment = declare_parameter<double>(prefix + "angle_increment_deg", 0.5,
                                                            startup) * kRadiansPerDegree;
        validateScanConfig(config);
        auto publisher = create_publisher<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS());
        if (!topics.insert(publisher->get_topic_name()).second || !frames.insert(config.frame_id).second)
            throw std::invalid_argument("duplicate lidar output topic or frame");
        devices_.push_back({name, config, publisher});
    }
    if (devices_.empty())
        throw std::invalid_argument("at least one lidar device must be enabled");
    states_.resize(devices_.size());
    if (!source_)
        source_ = std::make_unique<seer::SeerLidarSource>(address);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::Header>("radar_heartbeat", 10);
    diagnostics_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this] { diagnostics(); });
    worker_ = std::thread([this] { run(); });
}

LidarNode::~LidarNode()
{
    stop_ = true;
    wake_.notify_all();
    if (diagnostics_timer_)
        diagnostics_timer_->cancel();
    if (worker_.joinable())
        worker_.join();
}

bool LidarNode::waitUntil(Deadline deadline)
{
    std::unique_lock<std::mutex> lock(wait_mutex_);
    wake_.wait_until(lock, deadline, [this] { return stop_.load(); });
    return stop_.load();
}

void LidarNode::run()
{
    bool connected = false;
    auto backoff = std::chrono::milliseconds(500);
    while (!stop_ && rclcpp::ok(get_node_base_interface()->get_context()))
    {
        ReadResult result;
        const auto cycle_started = Clock::now();
        try
        {
            if (!connected)
            {
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    phase_ = "Connecting";
                }
                connected = source_->connect();
                if (connected)
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    ++connections_;
                }
            }
            if (!connected)
            {
                result.status = ReadStatus::ConnectionError;
                result.error = "status connection failed";
            }
            else if (!stop_)
            {
                result = source_->readBatch(Clock::now() + timeout_);
                const auto received = now();
                if (result.status == ReadStatus::Ok && !stop_)
                {
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        phase_ = "Running";
                        error_.clear();
                        request_ms_ = std::chrono::duration<double, std::milli>(
                            result.batch.received - result.batch.request_started).count();
                    }
                    accept(result.batch, received);
                    backoff = std::chrono::milliseconds(500);
                }
            }
        }
        catch (const std::exception& error)
        {
            result.status = ReadStatus::ProtocolError;
            result.error = error.what();
        }
        if (stop_)
            break;
        if (result.status != ReadStatus::Ok)
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                phase_ = "Backoff";
                error_ = result.error;
                ++errors_;
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Lidar: %s", result.error.c_str());
            if (result.status != ReadStatus::ApiError)
            {
                source_->disconnect();
                connected = false;
            }
            if (waitUntil(Clock::now() + backoff))
                break;
            backoff = std::min(backoff * 2, std::chrono::milliseconds(5000));
        }
        else
        {
            // Skip missed periods rather than emitting catch-up requests.
            const auto elapsed = Clock::now() - cycle_started;
            const auto periods = elapsed / period_ + 1;
            if (waitUntil(cycle_started + period_ * periods))
                break;
        }
    }
    source_->disconnect();
    std::lock_guard<std::mutex> lock(state_mutex_);
    phase_ = "Stopping";
}

void LidarNode::accept(const ScanBatch& batch, const rclcpp::Time& received)
{
    for (size_t i = 0; i < devices_.size(); ++i)
    {
        const auto& device = devices_[i];
        const auto found = std::find_if(batch.scans.begin(), batch.scans.end(),
            [&](const DeviceScan& scan) { return scan.name == device.name; });
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto& state = states_[i];
            state.present = found != batch.scans.end();
            if (!state.present || !found->source_stamp || found->beams.empty())
            {
                state.error = !state.present ? "device missing from batch" :
                    !found->source_stamp ? "missing/invalid source timestamp" : "empty beams";
                continue;
            }
            if (state.stamp == found->source_stamp)
            {
                ++state.duplicates;
                continue;
            }
            if (state.stamp && *found->source_stamp < *state.stamp)
                ++state.resets;
        }
        auto built = buildLaserScan(*found, device.config);
        built.message.header.stamp = received;
        if (stop_)
            return;
        device.publisher->publish(built.message);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto& state = states_[i];
            state.stamp = found->source_stamp;
            state.updated = batch.received;
            state.install = found->install;
            state.id = found->id;
            state.valid = built.valid_points;
            state.rejected = built.rejected_points;
            state.outside = built.outside_points;
            state.collisions = built.collisions;
            state.error.clear();
            ++state.frames;
        }
    }
    bool heartbeat = true;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto& state : states_)
            heartbeat = heartbeat && state.present && state.error.empty() && state.frames > 0 &&
                state.frames > state.heartbeat_frame &&
                std::chrono::duration<double>(Clock::now() - state.updated).count() < stale_seconds_;
        if (heartbeat)
            for (auto& state : states_)
                state.heartbeat_frame = state.frames;
    }
    if (heartbeat && !stop_)
    {
        std_msgs::msg::Header message;
        message.stamp = received;
        heartbeat_pub_->publish(message);
    }
}

void LidarNode::diagnostics()
{
    std::vector<DeviceState> states;
    std::string phase, error;
    size_t errors, connections;
    double request_ms;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        states = states_;
        phase = phase_;
        error = error_;
        errors = errors_;
        connections = connections_;
        request_ms = request_ms_;
    }
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    Status transport;
    transport.name = get_fully_qualified_name() + std::string("/acquisition");
    transport.hardware_id = "seer_status_api";
    transport.level = phase == "Running" ? Status::OK : Status::ERROR;
    transport.message = error.empty() ? phase : phase + ": " + error;
    value(transport, "request_ms", std::to_string(request_ms));
    value(transport, "errors", std::to_string(errors));
    value(transport, "connections", std::to_string(connections));
    value(transport, "stamp_mode", "receive_time (network latency included)");
    message.status.push_back(transport);
    for (size_t i = 0; i < states.size(); ++i)
    {
        const auto& state = states[i];
        const double age = state.frames ?
            std::chrono::duration<double>(Clock::now() - state.updated).count() : -1;
        Status status;
        status.name = std::string(get_fully_qualified_name()) + "/" + devices_[i].name;
        status.hardware_id = devices_[i].name;
        const bool stale = age < 0 || age >= stale_seconds_;
        status.level = stale || !state.error.empty() ? Status::ERROR :
            state.valid == 0 || state.outside > 0 || state.resets > 0 ? Status::WARN : Status::OK;
        status.message = !state.error.empty() ? state.error : stale ? "no fresh source frame" :
            state.valid == 0 ? "all measurements invalid" : state.outside > 0 ? "points outside grid" :
            state.resets > 0 ? "source clock reset observed" : "fresh scan";
        value(status, "age_sec", std::to_string(age));
        value(status, "frames", std::to_string(state.frames));
        value(status, "average_hz", std::to_string(state.frames /
            std::max(1e-6, std::chrono::duration<double>(Clock::now() - started_).count())));
        value(status, "duplicates", std::to_string(state.duplicates));
        value(status, "clock_resets", std::to_string(state.resets));
        value(status, "valid_points", std::to_string(state.valid));
        value(status, "rejected_points", std::to_string(state.rejected));
        value(status, "outside_points", std::to_string(state.outside));
        value(status, "collisions", std::to_string(state.collisions));
        value(status, "source_stamp", state.stamp ? std::to_string(*state.stamp) : "missing");
        if (state.id) value(status, "device_id", std::to_string(*state.id));
        if (state.install.x) value(status, "install_x_raw", std::to_string(*state.install.x));
        if (state.install.y) value(status, "install_y_raw", std::to_string(*state.install.y));
        if (state.install.z) value(status, "install_z_raw", std::to_string(*state.install.z));
        if (state.install.yaw) value(status, "install_yaw_raw", std::to_string(*state.install.yaw));
        if (state.install.upside) value(status, "install_upside_raw", *state.install.upside ? "true" : "false");
        message.status.push_back(std::move(status));
    }
    diagnostics_pub_->publish(message);
}
}  // namespace lidar
