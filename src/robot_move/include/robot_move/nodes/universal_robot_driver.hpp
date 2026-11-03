#ifndef UNIVERSAL_ROBOT_DRIVER_HPP
#define UNIVERSAL_ROBOT_DRIVER_HPP

#include <atomic>
#include <functional>
#include <thread>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include "robot_move/interface/robot_move_base.hpp"

class UniversalRobotDriver : public rclcpp::Node
{
private:
    std::unique_ptr<RobotMove> robot_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_move_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_distance_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_rotate_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr scan_config_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr scannerctrl_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex robot_mutex_;
    std::mutex task_mutex_;
    std::thread task_thread_;
    std::atomic<bool> task_running_{false};
    std::atomic<bool> shutting_down_{false};
    int pose_loop_count_ = 0;

    void startTask(std::function<void(RobotMove&)> task);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void scanconfigCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void scannerctrlCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void stepMoveCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void stepRotateCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timerCallback();

public:
    explicit UniversalRobotDriver(const std::string& node_name);
    // Supply an already-initialized driver for embedding or hardware-free tests.
    UniversalRobotDriver(const std::string& node_name, std::unique_ptr<RobotMove> robot);
    ~UniversalRobotDriver() override;

private:
    friend struct UniversalRobotDriverTestAccess;
};

#endif
