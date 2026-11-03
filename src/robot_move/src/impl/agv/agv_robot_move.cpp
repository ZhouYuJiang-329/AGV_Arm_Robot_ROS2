#include "robot_move/impl/agv/agv_robot_move.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

using robot_move::seer::Client;
using robot_move::seer::Pose;
using robot_move::seer::RelocationStatus;

/** @brief 默认构造真实仙工客户端，连接与初始化动作留到显式调用时执行。 */
AGVRobotMove::AGVRobotMove() : AGVRobotMove(std::make_unique<Client>()) {}
/** @brief 接管注入客户端的所有权，拒绝空指针，保证后续业务调用都有有效依赖。 */
AGVRobotMove::AGVRobotMove(std::unique_ptr<robot_move::seer::IClient> client)
    : client_(std::move(client))
{
    if (!client_)
        throw std::invalid_argument("AGVRobotMove requires a Seer client");
}
/** @brief 销毁时复用关闭流程；调用者应在对象销毁前先回收外部任务线程。 */
AGVRobotMove::~AGVRobotMove()
{
    moveshutdown();
}
/** @brief 请求结束本地导航/定位等待；不发送机器人停车或任务取消命令。 */
void AGVRobotMove::cancelPendingOperations()
{
    operations_canceled_.store(true);
}

/** @brief 先校验再原子更新两项应用层限速，错误配置不会覆盖原值。 */
bool AGVRobotMove::setSpeedLimits(double max_linear_speed, double max_angular_speed)
{
    if (!std::isfinite(max_linear_speed) || max_linear_speed < 0 ||
        !std::isfinite(max_angular_speed) || max_angular_speed < 0)
    {
        std::cerr << "[AGV] 速度上限必须是非负有限数值（0表示未配置）" << std::endl;
        return false;
    }
    std::lock_guard<std::mutex> lock(limits_mutex_);
    max_linear_speed_ = max_linear_speed;
    max_angular_speed_ = max_angular_speed;
    return true;
}

/** @brief 先拒绝非有限速度和合成溢出，再用配置快照检查合成线速度及角速度上限。 */
bool AGVRobotMove::validateVelocity(double vx, double vy, double wz)
{
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz))
    {
        std::cerr << "[AGV] 速度参数必须是有限数值" << std::endl;
        return false;
    }
    const double linear_speed = std::hypot(vx, vy);
    if (!std::isfinite(linear_speed))
    {
        std::cerr << "[AGV] 合成线速度超出数值范围" << std::endl;
        return false;
    }
    std::lock_guard<std::mutex> lock(limits_mutex_);
    if (max_linear_speed_ > 0 && linear_speed > max_linear_speed_)
    {
        std::cerr << "[AGV] 合成线速度超过配置上限 " << max_linear_speed_ << " m/s" << std::endl;
        return false;
    }
    if (max_angular_speed_ > 0 && std::abs(wz) > max_angular_speed_)
    {
        std::cerr << "[AGV] 角速度超过配置上限 " << max_angular_speed_ << " rad/s" << std::endl;
        return false;
    }
    return true;
}

/** @brief 先接收完整的类型化位姿，成功后再统一写入调用者的三个输出参数。 */
bool AGVRobotMove::getpose(double& x, double& y, double& theta)
{
    // 查询失败时不修改调用者原有的位姿数据。
    Pose pose;
    if (!client_->queryPose(pose))
        return false;
    x = pose.x;
    y = pose.y;
    theta = pose.angle;
    return true;
}

/** @brief 获取当前位置作为重定位目标，等待请求被接受后再等待定位真正成功。 */
bool AGVRobotMove::relocate()
{
    Pose pose;
    if (!client_->queryPose(pose) || !client_->requestRelocation(pose, 2))
        return false;
    return waitForRelocationComplete();
}

/** @brief 在统一截止时间内轮询定位；旧版待确认状态只确认一次，最终仍需成功状态。 */
bool AGVRobotMove::waitForRelocationComplete(std::chrono::milliseconds timeout)
{
    constexpr auto POLL_INTERVAL = std::chrono::milliseconds(100);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    // 旧版状态可能连续多次处于待确认阶段，避免重复发送确认请求。
    bool confirmation_sent = false;
    while (!operations_canceled_.load() && std::chrono::steady_clock::now() < deadline)
    {
        RelocationStatus status;
        if (!client_->queryRelocationStatus(status, deadline))
            return false;
        if (operations_canceled_.load())
            return false;
        if (std::chrono::steady_clock::now() >= deadline)
            break;
        if (status == RelocationStatus::Success)
            return true;
        if (status == RelocationStatus::Completed && !confirmation_sent)
        {
            if (!client_->confirmRelocation(deadline))
                return false;
            confirmation_sent = true;
        }
        std::this_thread::sleep_until(
            std::min(deadline, std::chrono::steady_clock::now() + POLL_INTERVAL));
    }
    if (!operations_canceled_.load())
        std::cerr << "[AGV] 等待重定位完成超时" << std::endl;
    return false;
}

/** @brief 在统一截止时间内轮询导航；仅完成算成功，失败/取消立即退出，其余状态继续等待。 */
bool AGVRobotMove::waitForNavigationComplete(std::chrono::milliseconds timeout)
{
    constexpr auto POLL_INTERVAL = std::chrono::milliseconds(100);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::cout << "[AGV] 等待导航任务完成..." << std::endl;
    while (!operations_canceled_.load() && std::chrono::steady_clock::now() < deadline)
    {
        NavigationStatus status;
        if (!client_->queryNavigationStatus(status, deadline))
            return false;
        // 已到截止时间时，即使本次查询返回完成，也不能把过期等待判为成功。
        if (operations_canceled_.load())
            return false;
        if (std::chrono::steady_clock::now() >= deadline)
            break;
        switch (status)
        {
        case NavigationStatus::Completed:
            std::cout << "[AGV] 导航完成" << std::endl;
            return true;
        case NavigationStatus::Failed:
            std::cerr << "[AGV] 导航任务失败" << std::endl;
            return false;
        case NavigationStatus::Canceled:
            std::cerr << "[AGV] 导航任务已取消" << std::endl;
            return false;
        case NavigationStatus::None:
        case NavigationStatus::Waiting:
        case NavigationStatus::Running:
        case NavigationStatus::Suspended:
            // 无任务不能代表完成；暂停状态也不重置整体等待时限。
            break;
        }
        std::this_thread::sleep_until(
            std::min(deadline, std::chrono::steady_clock::now() + POLL_INTERVAL));
    }
    // 这里只结束本地等待，不取消机器人端可能仍在执行的任务。
    if (operations_canceled_.load())
        return false;
    std::cerr << "[AGV] 等待导航任务完成超时" << std::endl;
    return false;
}

/** @brief 校验DO编号后委托客户端设置电平，电气用途不在移动控制层推定。 */
bool AGVRobotMove::setDigitalOutput(int id, bool status)
{
    if (id < 0)
        return false;
    return client_->setDigitalOutput(id, status);
}
/** @brief 先清理旧连接和取消状态，再委托客户端连接；失败后再次清理资源。 */
bool AGVRobotMove::connect(const std::string& ip)
{
    moveshutdown();
    operations_canceled_.store(false);
    if (client_->connect(ip))
        return true;
    moveshutdown();
    return false;
}

/** @brief 保存通过校验的初始化动作配置，不立即对设备执行任何动作。 */
bool AGVRobotMove::setInitOptions(const InitOptions& options)
{
    if (options.do_id < 0)
    {
        std::cerr << "[AGV] 初始化DO编号必须是非负整数" << std::endl;
        return false;
    }
    std::lock_guard<std::mutex> lock(init_options_mutex_);
    init_options_ = options;
    return true;
}

/** @brief 使用同一份配置快照按顺序执行DO和重定位，任一步失败都关闭连接。 */
bool AGVRobotMove::initializeActions()
{
    // 仅在复制配置时持锁，执行网络请求和等待时不占用配置锁。
    InitOptions options;
    {
        std::lock_guard<std::mutex> lock(init_options_mutex_);
        options = init_options_;
    }
    if (options.set_do && !setDigitalOutput(options.do_id, options.do_status))
    {
        std::cerr << "[AGV] 初始化失败：DO设置未成功" << std::endl;
        moveshutdown();
        return false;
    }
    if (options.relocate && !relocate())
    {
        std::cerr << "[AGV] 初始化失败：重定位未成功" << std::endl;
        moveshutdown();
        return false;
    }
    return true;
}

/** @brief 组合连接与初始化动作；短路求值保证连接失败后不会执行设备动作。 */
bool AGVRobotMove::init(const std::string& seer_ip)
{
    return connect(seer_ip) && initializeActions();
}

/** @brief 参数有效才下发持续速度；等待命令应答但不等待运动结束，失败记录日志。 */
void AGVRobotMove::setVelocity(double vx, double vy, double wz)
{
    if (!validateVelocity(vx, vy, wz))
        return;
    if (!client_->setVelocity(vx, vy, wz, 0))
        std::cerr << "[AGV] 速度指令未被接受" << std::endl;
}

/** @brief 校验距离、方向速度与限速，再依次完成平移请求和任务完成等待。 */
bool AGVRobotMove::movebydistance(double dist, double vx, double vy)
{
    if (!std::isfinite(dist) || dist <= 0)
    {
        std::cerr << "[AGV] 平移距离必须是正的有限数值，方向由速度正负决定" << std::endl;
        return false;
    }
    if (!validateVelocity(vx, vy, 0))
        return false;
    if (vx == 0 && vy == 0)
    {
        std::cerr << "[AGV] 平移速度不能同时为零" << std::endl;
        return false;
    }
    std::cout << "[AGV] 直线运动距离: " << dist << "m" << std::endl;

    if (!client_->translate(dist, vx, vy))
        return false;
    return waitForNavigationComplete();
}

/** @brief 校验转角、角速度与限速，再依次完成旋转请求和任务完成等待。 */
bool AGVRobotMove::rotatebyangle(double angle, double vw)
{
    if (!std::isfinite(angle) || angle <= 0)
    {
        std::cerr << "[AGV] 转角必须是正的有限数值，方向由角速度正负决定" << std::endl;
        return false;
    }
    if (!validateVelocity(0, 0, vw))
        return false;
    if (vw == 0)
    {
        std::cerr << "[AGV] 转动角速度不能为零" << std::endl;
        return false;
    }
    std::cout << "[AGV] 转动角度: " << angle << "rad" << std::endl;

    if (!client_->turn(angle, vw))
        return false;
    return waitForNavigationComplete();
}

/** @brief 保留父类周期接口；AGV没有需要在此重复刷新的协议数据。 */
void AGVRobotMove::send_loop() {}
/** @brief 结束本地等待并释放客户端连接；此处不发送或确认机器人停车命令。 */
void AGVRobotMove::moveshutdown()
{
    cancelPendingOperations();
    client_->disconnect();
}
