#include "robot_move/nodes/universal_robot_driver.hpp"

#include <chrono>
#include <functional>
#include <thread>
#include <stdexcept>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "robot_move/factory/robot_move_factory.hpp"
#include "robot_move/impl/agv/agv_robot_move.hpp"

/**
 * @class UniversalRobotDriver
 * @brief 通用机器人驱动节点类，负责机器人的控制和状态管理
 *
 * 该类实现了一个ROS2节点，用于控制不同类型的机器人，包括AGV等。
 * 它提供了速度控制、扫描器配置、距离移动、角度旋转等功能，并定期发布机器人姿态信息。
 */

/**
 * @brief 构造函数
 * @param node_name 节点名称
 *
 * 初始化机器人驱动节点，包括：
 * 1. 声明并获取机器人类型参数
 * 2. 根据机器人类型获取默认连接端口
 * 3. 创建机器人实例
 * 4. 初始化机器人硬件
 * 5. 创建各种订阅者和发布者
 * 6. 创建定时器用于定期发送控制命令和发布姿态信息
 */
UniversalRobotDriver::UniversalRobotDriver(const std::string& node_name)
    : UniversalRobotDriver(node_name, nullptr)
{
}

UniversalRobotDriver::UniversalRobotDriver(const std::string& node_name, std::unique_ptr<RobotMove> robot)
    : Node(node_name), robot_(std::move(robot))
{
    RCLCPP_INFO(this->get_logger(),"[%s]节点启动!", node_name.c_str());
    // 声明机器人类型参数，默认为"agv"
    this->declare_parameter<std::string>("robot_type", "agv");
    std::string type = this->get_parameter("robot_type").as_string();
    // 根据机器人类型获取默认连接端口
    std::string port = this->declare_parameter<std::string>(
        "connection", robot_move::defaultConnectionForType(type));
    RCLCPP_INFO(this->get_logger(), "正在初始化类型: [%s], 端口: [%s]", type.c_str(), port.c_str());

    if (!robot_)
    {
        // 创建机器人实例
        robot_ = robot_move::createRobotMove(type);
        if (!robot_)
        {
            throw std::invalid_argument("未知机器人类型: " + type);
        }
        // Validate AGV limits before connecting to hardware. Zero means unconfigured.
        if (type == "agv")
        {
            const double linear_limit = this->declare_parameter<double>("agv_max_linear_speed", 0.0);
            const double angular_limit = this->declare_parameter<double>("agv_max_angular_speed", 0.0);
            rcl_interfaces::msg::ParameterDescriptor startup_only;
            startup_only.read_only = true;
            AGVRobotMove::InitOptions options;
            options.set_do = this->declare_parameter<bool>("agv_init_set_do", true, startup_only);
            options.do_status = this->declare_parameter<bool>("agv_init_do_status", true, startup_only);
            options.relocate = this->declare_parameter<bool>("agv_init_relocate", true, startup_only);
            const auto do_id = this->declare_parameter<int64_t>("agv_init_do_id", 0, startup_only);
            if (do_id < 0 || do_id > std::numeric_limits<int>::max())
                throw std::invalid_argument("agv_init_do_id 必须是非负int范围内的整数");
            options.do_id = static_cast<int>(do_id);
            auto* agv = dynamic_cast<AGVRobotMove*>(robot_.get());
            if (!agv || !agv->setSpeedLimits(linear_limit, angular_limit) || !agv->setInitOptions(options))
            {
                RCLCPP_FATAL(this->get_logger(), "AGV速度上限配置无效，停止初始化");
                throw std::invalid_argument("AGV速度上限配置无效");
            }
        }
        // 初始化机器人硬件
        if (!robot_->init(port))
        {
            RCLCPP_FATAL(this->get_logger(), "硬件初始化失败! Port: %s", port.c_str());
            throw std::runtime_error("硬件初始化失败: " + port);
        }
    }
    RCLCPP_INFO(this->get_logger(), "硬件初始化成功!");

    // 创建速度控制订阅者
    robot_move_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        type == "agv" ? "legacy_cmd_vel" : "cmd_vel", 1,
        std::bind(&UniversalRobotDriver::cmdVelCallback, this, std::placeholders::_1));

    // 创建扫描器配置订阅者
    scan_config_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "scan_config", 10,
        std::bind(&UniversalRobotDriver::scanconfigCallback, this, std::placeholders::_1));

    // 创建扫描器控制订阅者
    scannerctrl_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "scan_ctrl", 10,
        std::bind(&UniversalRobotDriver::scannerctrlCallback, this, std::placeholders::_1));

    // 创建距离移动订阅者
    robot_distance_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "robot_distance_move", 10,
        std::bind(&UniversalRobotDriver::stepMoveCallback, this, std::placeholders::_1));

    // 创建旋转移动订阅者
    robot_rotate_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "robot_rotate_move", 10,
        std::bind(&UniversalRobotDriver::stepRotateCallback, this, std::placeholders::_1));

    // 创建姿态发布者
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("agv_pose", 10);

    // 创建定时器，每20毫秒执行一次timerCallback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&UniversalRobotDriver::timerCallback, this)
    );
}

/**
 * @brief 析构函数
 *
 * 节点关闭时，停止机器人运动并关闭硬件连接
 */
UniversalRobotDriver::~UniversalRobotDriver()
{
    shutting_down_.store(true);
    if (timer_) timer_->cancel();
    if (robot_) robot_->cancelPendingOperations();
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (task_thread_.joinable()) task_thread_.join();
    }
    std::lock_guard<std::mutex> lock(robot_mutex_);
    if(robot_)
    {
        RCLCPP_INFO(this->get_logger(), "节点关闭，停止机器人...");
        // 设置速度为0
        robot_->setVelocity(0.0, 0.0, 0.0);
        // 关闭机器人运动
        robot_->moveshutdown();
    }
}

void UniversalRobotDriver::startTask(std::function<void(RobotMove&)> task)
{
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (shutting_down_.load() || !robot_) return;
    if (task_running_.load())
    {
        RCLCPP_WARN(this->get_logger(), "已有底盘任务执行中，拒绝重复任务");
        return;
    }
    if (task_thread_.joinable()) task_thread_.join();
    task_running_.store(true);
    try
    {
        task_thread_ = std::thread([this, task]()
        {
            try
            {
                std::lock_guard<std::mutex> lock(robot_mutex_);
                if (!shutting_down_.load()) task(*robot_);
            }
            catch (const std::exception& error)
            {
                RCLCPP_ERROR(this->get_logger(), "底盘任务异常: %s", error.what());
            }
            catch (...)
            {
                RCLCPP_ERROR(this->get_logger(), "底盘任务发生未知异常");
            }
            task_running_.store(false);
        });
    }
    catch (...)
    {
        task_running_.store(false);
        throw;
    }
}

/**
 * @brief 速度控制回调函数
 * @param msg 速度控制消息
 *
 * 处理cmd_vel话题的消息，设置机器人的线速度和角速度
 */
void UniversalRobotDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (shutting_down_.load()) return;
    std::unique_lock<std::mutex> lock(robot_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
        RCLCPP_WARN(this->get_logger(), "底盘任务执行中，当前指令未执行");
        return;
    }
    if (robot_)
    {
        // 设置机器人速度：x方向线速度，y方向线速度，z方向角速度
        robot_->setVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
    }
}

/**
 * @brief 扫描器配置回调函数
 * @param msg 扫描器配置消息
 *
 * 处理scan_config话题的消息，配置扫描器参数
 */
void UniversalRobotDriver::scanconfigCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (shutting_down_.load()) return;
    std::unique_lock<std::mutex> lock(robot_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
        RCLCPP_WARN(this->get_logger(), "底盘任务执行中，当前指令未执行");
        return;
    }
    if (robot_)
    {
        // 设置扫描器配置参数
        robot_->setScannerConfig(msg->linear.x, msg->linear.y, msg->angular.z);
    }
}

/**
 * @brief 扫描器控制回调函数
 * @param msg 空消息
 *
 * 处理scan_ctrl话题的消息，触发扫描器控制操作
 * 在新线程中执行扫描器控制，避免阻塞主线程
 */
void UniversalRobotDriver::scannerctrlCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg; // 忽略消息参数
    if (!robot_) return;

    RCLCPP_INFO(this->get_logger(), "收到扫查器触发指令，开始执行 scannercontrol...");
    startTask([](RobotMove& robot) { robot.scannercontrol(); });
}

/**
 * @brief 距离移动回调函数
 * @param msg 距离移动消息
 *
 * 处理robot_distance_move话题的消息，执行指定距离的移动
 * 在新线程中执行移动操作，避免阻塞主线程
 */
void UniversalRobotDriver::stepMoveCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!robot_) return;

    // 从消息中提取移动距离和速度参数
    double dist = msg->linear.z;  // 移动距离
    double vx = msg->linear.x;    // x方向速度
    double vy = msg->linear.y;    // y方向速度

    RCLCPP_INFO(this->get_logger(), "StepMove: dist=%.2f, vx=%.2f, vy=%.2f", dist, vx, vy);
    startTask([this, dist, vx, vy](RobotMove& robot)
    {
        if (!robot.movebydistance(dist, vx, vy))
            RCLCPP_WARN(this->get_logger(), "底盘任务未成功完成");
    });
}

/**
 * @brief 旋转回调函数
 * @param msg 旋转消息
 *
 * 处理robot_rotate_move话题的消息，执行指定角度的旋转
 * 在新线程中执行旋转操作，避免阻塞主线程
 */
void UniversalRobotDriver::stepRotateCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!robot_) return;

    // 从消息中提取旋转角度和角速度参数
    double angle = msg->angular.z;  // 旋转角度
    double vw = msg->angular.y;     // 角速度

    RCLCPP_INFO(this->get_logger(), "StepRotate: angle=%.2f, vw=%.2f", angle, vw);
    startTask([this, angle, vw](RobotMove& robot)
    {
        if (!robot.rotatebyangle(angle, vw))
            RCLCPP_WARN(this->get_logger(), "底盘任务未成功完成");
    });
}

/**
 * @brief 定时器回调函数
 *
 * 定期执行以下操作：
 * 1. 发送控制命令到机器人
 * 2. 每5个周期发布一次机器人姿态信息
 */
void UniversalRobotDriver::timerCallback()
{
    if (shutting_down_.load()) return;
    std::unique_lock<std::mutex> lock(robot_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    if (robot_)
    {
        // 发送控制命令到机器人
        robot_->send_loop();
        // 每5个周期发布一次姿态信息
        if (++pose_loop_count_ >= 5)
        {
            pose_loop_count_ = 0;
            double x = 0, y = 0, angle = 0;
            // 获取机器人姿态
            if (robot_->getpose(x, y, angle))
            {
                // 创建并发布姿态消息
                auto msg = geometry_msgs::msg::Pose2D();
                msg.x = x;
                msg.y = y;
                msg.theta = angle;
                pose_pub_->publish(msg);
            }
        }
    }
}

