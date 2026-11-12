#pragma once

#include <string>
#include <unordered_map>

namespace agv_hardware {

/**
 * @brief AGV 硬件接口配置参数
 * 
 * 包含所有可配置的参数：机器人网络配置、WebSocket 配置、控制模式等
 */
struct AgvConfig {
    // ==================== 机器人网络配置 ====================
    /// @brief 机器人控制器 IP 地址
    std::string robot_ip = "192.168.1.158";
    
    /// @brief 底盘控制 IP 地址
    std::string chassis_ip = "192.168.1.109";
    
    /// 合成线速度上限（m/s）和角速度上限（rad/s）；0表示未配置。
    double agv_max_linear_speed = 0.0;
    double agv_max_angular_speed = 0.0;
    /// 初始化动作保持旧默认值，可通过硬件参数关闭。
    bool external_navigation = false;
    bool agv_init_set_do = true;
    int agv_init_do_id = 0;
    bool agv_init_do_status = true;
    bool agv_init_relocate = true;

    /// @brief 本地控制机 IP 地址
    std::string local_ip = "192.168.1.100";
    
    // ==================== WebSocket 配置 ====================
    /// @brief 上位机 WebSocket 服务器 URI
    std::string server_uri = "ws://192.168.1.108:9100";
    
    /// @brief 本地 WebSocket 服务器端口（用于接收本地控制指令）
    int local_server_port = 9001;
    
    /// @brief WebSocket 重连间隔（毫秒）
    int reconnect_interval_ms = 5000;
    
    // ==================== 控制模式配置 ====================
    /// @brief 实时模式开关（true=实时模式，false=非实时模式）
    bool rt_mode = false;
    
    /// @brief 默认运动速度（mm/s）
    double default_speed = 300.0;
    
    /// @brief 在线调速比例
    double speed_ratio = 0.2;
    
    /// @brief 实时模式网络容差（毫秒）
    double rt_network_tolerance_ms = 20.0;
    
    /// @brief 非实时模式转弯区大小（mm）
    double nrt_zone = 50.0;
};

/** @brief 严格解析底盘硬件参数；失败时不修改config，并通过error返回原因。 */
bool loadChassisConfig(const std::unordered_map<std::string, std::string>& parameters,
                       AgvConfig& config, std::string& error);

} // namespace agv_hardware
