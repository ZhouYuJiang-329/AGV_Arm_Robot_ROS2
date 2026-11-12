#include "agv_hardware/config.hpp"
#include <arpa/inet.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace agv_hardware
{
namespace
{
std::string trim(const std::string& value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return "";
    return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}
} // namespace
bool loadChassisConfig(const std::unordered_map<std::string, std::string>& parameters,
                       AgvConfig& config, std::string& error)
{
    AgvConfig candidate = config;
    error.clear();
    auto read = [&](const std::string& key, auto parse)
    {
        const auto item = parameters.find(key);
        if (item == parameters.end())
            return;
        try
        {
            parse(trim(item->second));
        }
        catch (const std::exception&)
        {
            throw std::invalid_argument("无效硬件参数: " + key);
        }
    };
    auto boolean = [](std::string text)
    {
        std::transform(text.begin(), text.end(), text.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (text == "true")
            return true;
        if (text == "false")
            return false;
        throw std::invalid_argument("bool");
    };
    auto nonnegative = [](const std::string& text)
    {
        size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value) || value < 0)
            throw std::invalid_argument("speed");
        return value;
    };
    try
    {
        read("chassis_ip", [&](const std::string& text) { candidate.chassis_ip = text; });
        in_addr address{};
        if (inet_pton(AF_INET, candidate.chassis_ip.c_str(), &address) != 1)
            throw std::invalid_argument("无效硬件参数: chassis_ip（需要IPv4地址）");
        read("agv_max_linear_speed",
             [&](const std::string& s) { candidate.agv_max_linear_speed = nonnegative(s); });
        read("agv_max_angular_speed",
             [&](const std::string& s) { candidate.agv_max_angular_speed = nonnegative(s); });
        read("external_navigation",
             [&](const std::string& s) { candidate.external_navigation = boolean(s); });
        read("agv_init_set_do",
             [&](const std::string& s) { candidate.agv_init_set_do = boolean(s); });
        read("agv_init_do_status",
             [&](const std::string& s) { candidate.agv_init_do_status = boolean(s); });
        read("agv_init_relocate",
             [&](const std::string& s) { candidate.agv_init_relocate = boolean(s); });
        read("agv_init_do_id",
             [&](const std::string& s)
             {
                 if (s.empty() || !std::all_of(s.begin(), s.end(),
                                               [](unsigned char c) { return std::isdigit(c); }))
                     throw std::invalid_argument("id");
                 size_t consumed = 0;
                 const auto value = std::stoll(s, &consumed);
                 if (consumed != s.size() || value > std::numeric_limits<int>::max())
                     throw std::invalid_argument("id");
                 candidate.agv_init_do_id = static_cast<int>(value);
             });
        config = candidate;
        return true;
    }
    catch (const std::exception& e)
    {
        error = e.what();
        return false;
    }
}
} // namespace agv_hardware
