#ifndef ROBOT_MOVE_SEER_CLIENT_HPP
#define ROBOT_MOVE_SEER_CLIENT_HPP

#include <memory>
#include <mutex>
#include <string>
#include "robot_move/impl/agv/seer/seer_types.hpp"

namespace robot_move::seer
{
class Transport;
/**
 * @brief AGV策略层依赖的类型化客户端接口，不暴露JSON、字节或socket。
 * @details 操作返回true仅表示本次调用成功；移动/重定位命令成功不代表任务已经完成。
 *          查询失败时应保持输出参数不变。deadline均为绝对截止时间，真实Client还会
 *          应用单次1秒时限；Deadline::max()不会关闭这个默认时限。
 */
class IClient
{
  public:
    /** @brief 支持通过接口指针销毁真实或测试客户端，释放具体实现持有的资源。 */
    virtual ~IClient() = default;
    /**
     * @brief 建立客户端连接，不执行DO设置、重定位等初始化动作。
     * @param ip 机器人连接地址，真实Client使用IPv4及仙工固定端口。
     * @return 连接成功返回true，失败返回false。
     * @note 连接/重连不能与其他操作并发执行。
     */
    virtual bool connect(const std::string& ip) = 0;
    /** @brief 释放客户端连接；不表示机器人已经停车，也不发送业务取消命令。 */
    virtual void disconnect() = 0;
    /**
     * @brief 查询世界坐标系中的机器人位姿。
     * @param pose 成功时写入位置（m）与朝向（rad），失败时保持原值。
     * @param deadline 调用方的绝对截止时间。
     * @return 调用成功且位姿字段完整有效返回true，否则返回false。
     */
    virtual bool queryPose(Pose& pose, Deadline deadline = Deadline::max()) = 0;
    /**
     * @brief 下发一次开环速度，等待命令应答，不等待运动结束。
     * @param vx X方向速度，m/s，正值向前。
     * @param vy Y方向速度，m/s，正值向左。
     * @param w 角速度，rad/s，正值逆时针。
     * @param duration 持续时间，ms；0表示持续保持速度，正值为有限请求时长（配置项文档范围0–5000）；到期制动和固件支持待目标设备验证。
     * @param deadline 调用方的绝对截止时间。
     * @return 本次速度请求成功应答返回true，否则返回false。
     * @note 应用层限速由AGVRobotMove处理，真实Client只负责协议字段和应答。
     */
    virtual bool setVelocity(double vx, double vy, double w, int duration,
                             Deadline deadline = Deadline::max()) = 0;
    /**
     * @brief 下发定距离平移请求，只等待请求应答。
     * @param dist 移动距离绝对值，m；正数要求由调用方校验。
     * @param vx X方向速度，m/s，正值向前，负值向后。
     * @param vy Y方向速度，m/s，正值向左，负值向右。
     * @param deadline 调用方的绝对截止时间。
     * @return 请求被接受返回true，否则返回false；任务完成须另行查询。
     */
    virtual bool translate(double dist, double vx, double vy,
                           Deadline deadline = Deadline::max()) = 0;
    /**
     * @brief 下发定角度旋转请求，只等待请求应答。
     * @param angle 转角绝对值，rad；正数要求由调用方校验。
     * @param vw 角速度，rad/s，正值逆时针，负值顺时针。
     * @param deadline 调用方的绝对截止时间。
     * @return 请求被接受返回true，否则返回false；任务完成须另行查询。
     */
    virtual bool turn(double angle, double vw, Deadline deadline = Deadline::max()) = 0;
    /**
     * @brief 查询一次导航任务状态，不执行轮询等待。
     * @param status 成功时写入有效状态枚举，失败时保持原值。
     * @param deadline 调用方的绝对截止时间。
     * @return 请求成功且状态字段有效返回true，否则返回false。
     * @note 成功查询到Failed/Canceled仍返回true，由策略层解释具体任务结果。
     */
    virtual bool queryNavigationStatus(NavigationStatus& status,
                                       Deadline deadline = Deadline::max()) = 0;
    /**
     * @brief 以指定世界坐标位姿为目标发起重定位，不等待定位完成。
     * @param target 搜索中心位姿，位置单位m、朝向单位rad。
     * @param radius 搜索区域半径，m。
     * @param deadline 调用方的绝对截止时间。
     * @return 重定位请求成功应答返回true，否则返回false。
     */
    virtual bool requestRelocation(const Pose& target, double radius,
                                   Deadline deadline = Deadline::max()) = 0;
    /**
     * @brief 查询一次定位状态，不自动确认旧版待确认状态。
     * @param status 成功时写入有效状态枚举，失败时保持原值。
     * @param deadline 调用方的绝对截止时间。
     * @return 查询成功且字段有效返回true，否则返回false。
     */
    virtual bool queryRelocationStatus(RelocationStatus& status,
                                       Deadline deadline = Deadline::max()) = 0;
    /**
     * @brief 发送旧版定位确认请求，只等待命令应答。
     * @param deadline 调用方的绝对截止时间。
     * @return 确认请求成功应答返回true，否则返回false。
     * @note 何时需要确认由AGV策略层根据定位状态决定，应答成功后仍需检查定位结果。
     */
    virtual bool confirmRelocation(Deadline deadline = Deadline::max()) = 0;
    /**
     * @brief 设置指定数字输出，不预设其电气用途。
     * @param id 非负DO编号。
     * @param status true为高电平，false为低电平。
     * @param deadline 调用方的绝对截止时间。
     * @return 编号有效且请求成功应答返回true，否则返回false。
     */
    virtual bool setDigitalOutput(int id, bool status, Deadline deadline = Deadline::max()) = 0;
};
/**
 * @brief 将类型化操作映射为仙工API，通过Transport收发并解析JSON响应。
 * @note 不包含任务等待、初始化策略或应用层限速；销毁前调用方须回收使用它的线程。
 */
class Client final : public IClient
{
  public:
    /** @brief 创建并独占持有真实Transport，构造时不连接设备。 */
    Client();
    /**
     * @brief 接管指定Transport的所有权，用于组合已有传输对象或本地模拟连接。
     * @param transport 非空传输对象。
     * @throws std::invalid_argument 传输对象为空时抛出。
     */
    explicit Client(std::unique_ptr<Transport> transport);
    /** @brief 释放持有的Transport，由其析构关闭连接，不发送业务命令。 */
    ~Client() override;
    /** @brief 委托Transport建立四端口连接；地址和返回值约定见IClient::connect。 */
    bool connect(const std::string& ip) override;
    /** @brief 与客户端调用串行地关闭连接，等待当前应答解析结束后再释放资源。 */
    void disconnect() override;
    /** @brief 用1004查询位姿，字段完整有效才更新pose；参数约定见IClient::queryPose。 */
    bool queryPose(Pose& pose, Deadline deadline = Deadline::max()) override;
    /** @brief 将速度和持续时间封装为2010请求；参数约定见IClient::setVelocity。 */
    bool setVelocity(double vx, double vy, double w, int duration,
                     Deadline deadline = Deadline::max()) override;
    /** @brief 用3055下发平移请求；参数及应答含义见IClient::translate。 */
    bool translate(double dist, double vx, double vy, Deadline deadline = Deadline::max()) override;
    /** @brief 用3056下发旋转请求；参数及应答含义见IClient::turn。 */
    bool turn(double angle, double vw, Deadline deadline = Deadline::max()) override;
    /** @brief 用1020查询简化导航状态，拒绝缺失、非整数或越界状态；其余约定见IClient。 */
    bool queryNavigationStatus(NavigationStatus& status,
                               Deadline deadline = Deadline::max()) override;
    /** @brief 用2002发送目标x/y/angle和搜索半径length；参数约定见IClient::requestRelocation。 */
    bool requestRelocation(const Pose& target, double radius,
                           Deadline deadline = Deadline::max()) override;
    /** @brief 用1021查询定位状态，保留旧版Completed语义；其余约定见IClient。 */
    bool queryRelocationStatus(RelocationStatus& status,
                               Deadline deadline = Deadline::max()) override;
    /** @brief 发送无正文的2003确认请求，不等待定位状态变化；截止时间约定见IClient。 */
    bool confirmRelocation(Deadline deadline = Deadline::max()) override;
    /** @brief 校验DO编号并用6001设置电平；参数及返回值约定见IClient::setDigitalOutput。 */
    bool setDigitalOutput(int id, bool status, Deadline deadline = Deadline::max()) override;

  private:
    /// 独占持有的传输层，负责socket、帧和收发，不向策略层暴露。
    std::unique_ptr<Transport> transport_;
    // 将响应JSON校验与异常通道关闭放在同一次串行调用内，避免其他请求插入。
    std::timed_mutex mutex_;
    /// 已解析响应的内部包装，定义留在cpp中，避免头文件依赖cJSON。
    struct Json;
    /**
     * @brief 统一执行单次调用，检查响应JSON和ret_code。
     * @param channel 请求通道。
     * @param api 请求编号。
     * @param body 请求正文，无正文时传空字符串。
     * @param result 成功且响应非空时接管解析后的对象，空应答不填充。
     * @param deadline 调用方的绝对截止时间，与单次1秒时限取更早者并传给Transport。
     * @return 传输成功且响应无API错误返回true，否则返回false。
     * @note 空应答可成功；非法JSON/非对象/内嵌NUL关闭对应通道，完整的API错误保留连接。
     */
    bool call(ApiChannel channel, uint16_t api, const std::string& body, Json& result,
              Deadline deadline);
};
} // namespace robot_move::seer
#endif
