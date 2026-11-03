#include "robot_move/impl/agv/seer/seer_client.hpp"
#include "robot_move/impl/agv/seer/seer_transport.hpp"
#include "robot_move/interface/cJSON.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>

namespace robot_move::seer
{
namespace
{
// 用cJSON对应的释放函数管理对象和序列化文本，避免失败分支泄漏。
using JsonPtr = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;
using TextPtr = std::unique_ptr<char, decltype(&cJSON_free)>;
/**
 * @brief 将有限数值字段和可选布尔字段序列化为紧凑JSON。
 * @param numbers 数值字段的键值列表；任一值非有限或分配失败时拒绝生成正文。
 * @param boolean_key 可选布尔字段名，为空指针时不添加该字段。
 * @param boolean_value 指定布尔字段的值。
 * @return 成功返回JSON字符串，失败返回nullopt；局部RAII自动回收对象和序列化缓冲区。
 */
std::optional<std::string> payload(std::initializer_list<std::pair<const char*, double>> numbers,
                                   const char* boolean_key = nullptr, bool boolean_value = false)
{
    JsonPtr object(cJSON_CreateObject(), cJSON_Delete);
    if (!object)
        return std::nullopt;
    for (const auto& field : numbers)
    {
        if (!std::isfinite(field.second) ||
            !cJSON_AddNumberToObject(object.get(), field.first, field.second))
            return std::nullopt;
    }
    if (boolean_key && !cJSON_AddBoolToObject(object.get(), boolean_key, boolean_value))
        return std::nullopt;
    TextPtr text(cJSON_PrintUnformatted(object.get()), cJSON_free);
    if (!text)
        return std::nullopt;
    return std::string(text.get());
}
/**
 * @brief 从JSON对象提取一个有限数值，不进行字符串到数字的隐式转换。
 * @param object 要读取的对象，可以为空；无对应字段时读取失败。
 * @param key 大小写敏感的字段名。
 * @param result 成功时写入数值，失败时保持原值。
 * @return 字段存在、类型为数字且数值有限时返回true。
 */
bool number(const cJSON* object, const char* key, double& result)
{
    const auto* item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble))
        return false;
    result = item->valuedouble;
    return true;
}
} // namespace

/** @brief 内部响应包装；对象离开作用域时自动释放解析树。 */
struct Client::Json
{
    JsonPtr value{nullptr, cJSON_Delete};
};

/** @brief 默认组合真实传输层，构造阶段不连接网络或发送命令。 */
Client::Client() : Client(std::make_unique<Transport>()) {}
/** @brief 接管传输对象，拒绝空依赖，确保后续API调用始终具有传输入口。 */
Client::Client(std::unique_ptr<Transport> transport) : transport_(std::move(transport))
{
    if (!transport_)
        throw std::invalid_argument("Seer Client requires a transport");
}
/** @brief 由独占指针销毁Transport并释放连接，外部线程须在此之前停止使用本客户端。 */
Client::~Client() = default;
/** @brief 仅委托传输层建立连接，不混入DO输出和重定位策略。 */
bool Client::connect(const std::string& ip)
{
    return transport_->connect(ip);
}
/** @brief 等待当前串行调用完成后关闭传输层，避免在JSON校验中途释放连接。 */
void Client::disconnect()
{
    std::lock_guard<std::timed_mutex> lock(mutex_);
    transport_->disconnect();
}

/** @brief 使用统一截止时间串行完成传输、严格JSON解析和API错误校验。 */
bool Client::call(ApiChannel channel, uint16_t api, const std::string& body, Json& result,
                  Deadline deadline)
{
    // 客户端锁等待和传输层共用这个时限，不给下一层重新增加完整的一秒。
    deadline = std::min(deadline, std::chrono::steady_clock::now() + std::chrono::seconds(1));
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    if (!lock.try_lock_until(deadline))
        return false;
    std::string response;
    if (!transport_->request(channel, api, body, response, deadline))
        return false;
    if (response.empty())
        return true; // 空正文可作命令确认，具体查询仍须校验自己的字段。
    // 文本内嵌NUL会让C字符串解析提前结束，因此使该通道失效。
    if (response.find('\0') != std::string::npos)
    {
        transport_->closeChannel(channel);
        return false;
    }
    // 要求整个响应都是一个JSON对象，不能接受对象后还有未解析的内容。
    JsonPtr parsed(cJSON_ParseWithOpts(response.c_str(), nullptr, true), cJSON_Delete);
    if (!cJSON_IsObject(parsed.get()))
    {
        transport_->closeChannel(channel);
        return false;
    }
    // 完整的API错误不是TCP帧错误，返回失败但保留连接以供后续调用。
    const auto* code = cJSON_GetObjectItemCaseSensitive(parsed.get(), "ret_code");
    if (code && (!cJSON_IsNumber(code) || code->valuedouble != 0))
    {
        std::cerr << "[Seer] API " << api << " 返回错误: " << response << std::endl;
        return false;
    }
    result.value = std::move(parsed);
    return true;
}

/** @brief 查询位姿并先解析到临时对象，三个字段均有效后才一起更新输出。 */
bool Client::queryPose(Pose& pose, Deadline deadline)
{
    Json result;
    if (!call(ApiChannel::Status, 1004, "", result, deadline))
        return false;
    Pose candidate;
    if (!number(result.value.get(), "x", candidate.x) ||
        !number(result.value.get(), "y", candidate.y) ||
        !number(result.value.get(), "angle", candidate.angle))
        return false;
    pose = candidate;
    return true;
}
/** @brief 封装vx/vy/w/duration并调用速度API，成功只表示该次命令应答无错。 */
bool Client::setVelocity(double vx, double vy, double w, int duration, Deadline deadline)
{
    const auto body = payload({{"vx", vx}, {"vy", vy}, {"w", w}, {"duration", duration}});
    Json result;
    return body && call(ApiChannel::Control, 2010, *body, result, deadline);
}
/** @brief 将距离和方向速度映射为平移API，不在客户端等待导航任务完成。 */
bool Client::translate(double dist, double vx, double vy, Deadline deadline)
{
    const auto body = payload({{"dist", dist}, {"vx", vx}, {"vy", vy}});
    Json result;
    return body && call(ApiChannel::Navigation, 3055, *body, result, deadline);
}
/** @brief 将转角和角速度映射为旋转API，方向及运动策略由上层决定。 */
bool Client::turn(double angle, double vw, Deadline deadline)
{
    const auto body = payload({{"angle", angle}, {"vw", vw}});
    Json result;
    return body && call(ApiChannel::Navigation, 3056, *body, result, deadline);
}
/** @brief 查询简化导航状态，只有0至6范围内的整数才转换为枚举并更新输出。 */
bool Client::queryNavigationStatus(NavigationStatus& status, Deadline deadline)
{
    Json result;
    if (!call(ApiChannel::Status, 1020, R"({"simple":true})", result, deadline))
        return false;
    double value;
    if (!number(result.value.get(), "task_status", value) || value < 0 || value > 6 ||
        std::floor(value) != value)
        return false;
    status = static_cast<NavigationStatus>(static_cast<int>(value));
    return true;
}
/** @brief 将目标位姿和搜索半径映射为x/y/angle/length，避免把朝向字段误写为theta。 */
bool Client::requestRelocation(const Pose& target, double radius, Deadline deadline)
{
    const auto body =
        payload({{"x", target.x}, {"y", target.y}, {"angle", target.angle}, {"length", radius}});
    Json result;
    return body && call(ApiChannel::Control, 2002, *body, result, deadline);
}
/** @brief 查询定位状态，只接受0至3的整数，保留旧版待确认状态供策略层判断。 */
bool Client::queryRelocationStatus(RelocationStatus& status, Deadline deadline)
{
    Json result;
    if (!call(ApiChannel::Status, 1021, "", result, deadline))
        return false;
    double value;
    if (!number(result.value.get(), "reloc_status", value) || value < 0 || value > 3 ||
        std::floor(value) != value)
        return false;
    status = static_cast<RelocationStatus>(static_cast<int>(value));
    return true;
}
/** @brief 发送无正文的旧版定位确认请求；是否需要确认由AGV层决定。 */
bool Client::confirmRelocation(Deadline deadline)
{
    Json result;
    return call(ApiChannel::Control, 2003, "", result, deadline);
}
/** @brief 拒绝负DO编号后构造id/status请求，电平含义不扩展为具体电气动作。 */
bool Client::setDigitalOutput(int id, bool status, Deadline deadline)
{
    if (id < 0)
        return false;
    const auto body = payload({{"id", id}}, "status", status);
    Json result;
    return body && call(ApiChannel::Other, 6001, *body, result, deadline);
}
} // namespace robot_move::seer
