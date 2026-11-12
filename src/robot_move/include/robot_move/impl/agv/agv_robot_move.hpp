#ifndef AGV_ROBOT_MOVE_HPP
#define AGV_ROBOT_MOVE_HPP

#include <atomic>
#include <cstdint>
#include <chrono>
#include <mutex>
#include <string>
#include <memory>
#include "robot_move/impl/agv/seer/seer_client.hpp"

#include "robot_move/interface/robot_move_base.hpp"

#ifndef SEERCONTROLIP
#define SEERCONTROLIP "10.42.0.114" // AGV IP
#endif

/** @brief AGV移动控制与任务策略，通过类型化仙工客户端执行具体操作。 */
class AGVRobotMove : public RobotMove
{
  public:
    /** @brief init()使用的动作配置；全部禁用时，初始化阶段仅建立连接。 */
    struct InitOptions
    {
        /// 是否在连接成功后设置数字输出。
        bool set_do = true;
        /// 数字输出编号；具体用途取决于机器人接线。
        int do_id = 0;
        /// 数字输出电平：true为高，false为低。
        bool do_status = true;
        /// 是否在DO步骤之后按当前位置发起重定位。
        bool relocate = true;
    };

  private:
    friend struct AGVRobotMoveTestAccess;
    std::unique_ptr<robot_move::seer::IClient> client_;
    std::atomic<bool> operations_canceled_{false};
    std::mutex limits_mutex_;
    double max_linear_speed_ = 0.0;
    double max_angular_speed_ = 0.0;
    std::mutex init_options_mutex_;
    InitOptions init_options_;
    using NavigationStatus = robot_move::seer::NavigationStatus;
    /**
     * @brief 检查速度数值及已配置的应用层上限，不下发命令。
     * @param vx X方向速度，m/s。
     * @param vy Y方向速度，m/s。
     * @param wz 角速度，rad/s。
     * @return 全部有限，且合成线速度和角速度绝对值未超限时返回true。
     * @note 零速度合法；上限为0表示该项未配置限制。
     */
    bool validateVelocity(double vx, double vy, double wz);
    /**
     * @brief 获取初始化配置快照，依次执行启用的DO设置和重定位。
     * @return 所有启用步骤成功返回true；任一步骤失败则关闭连接并返回false。
     * @note 不建立连接，也不回滚已经执行的动作。
     */
    bool initializeActions();
    /**
     * @brief 轮询定位状态，兼容旧版需要确认的定位流程。
     * @param timeout 本地状态等待的总时限，默认60秒。
     * @return 定位成功返回true；查询/确认失败、本地取消或超时返回false。
     * @note 旧版“完成待确认”状态只确认一次，之后仍须等待“成功”；超时不自动取消重定位。
     */
    bool waitForRelocationComplete(std::chrono::milliseconds timeout = std::chrono::seconds(60));
    /**
     * @brief 轮询导航状态，等待当前移动任务明确完成。
     * @param timeout 本地状态等待的总时限，默认5分钟。
     * @return 仅“完成”返回true；失败、取消、查询失败、本地取消或超时返回false。
     * @note 无任务、等待、运行和暂停均继续等待；暂停不重置时限，超时不自动停车。
     */
    bool waitForNavigationComplete(std::chrono::milliseconds timeout = std::chrono::minutes(5));

  public:
    /** @brief 创建真实仙工客户端；构造本身不连接设备或执行初始化动作。 */
    AGVRobotMove();
    /**
     * @brief 注入类型化客户端，用于测试或替换底层实现。
     * @param client 转移给本对象独占持有的非空客户端。
     * @throws std::invalid_argument 客户端为空时抛出。
     */
    explicit AGVRobotMove(std::unique_ptr<robot_move::seer::IClient> client);
    /**
     * @brief 请求结束本地等待并断开客户端连接。
     * @note 不发送停车命令；外部调用方须先结束并回收使用本对象的线程。
     */
    ~AGVRobotMove() override;
    /**
     * @brief 设置本地取消标志，使导航和定位等待尽快返回失败。
     * @note 不发送机器人取消指令；标志持续到下一次connect()/init()，当前通信仍受截止时间约束。
     */
    void cancelPendingOperations() override;

    /**
     * @brief 更新后续运动命令使用的应用层速度上限。
     * @param max_linear_speed 合成线速度上限，m/s；0表示未配置。
     * @param max_angular_speed 角速度绝对值上限，rad/s；0表示未配置。
     * @return 两项都是非负有限数值时返回true；否则返回false并保留原配置。
     * @note 不改变已经下发的速度，也不自动截断超限命令。
     */
    bool setSpeedLimits(double max_linear_speed, double max_angular_speed);
    /**
     * @brief 保存下次init()使用的初始化动作配置，不立即操作设备。
     * @param options 是否执行DO/重定位、DO编号和电平。
     * @return DO编号非负时保存并返回true，否则返回false且保留原配置。
     */
    bool setInitOptions(const InitOptions& options);

    /**
     * @brief 释放旧连接并建立新连接，同时清除本地取消标志。
     * @param seer_ip 机器人IPv4地址；兼容带端口后缀，但真实客户端仍使用仙工固定端口。
     * @return 真实客户端的四个服务端口全部连接成功时返回true，失败时清理连接并返回false。
     * @note 此阶段不发送DO或重定位命令；不能与其他驱动操作并发执行。
     */
    bool connect(const std::string& seer_ip);
    /**
     * @brief 设置指定数字输出并等待命令应答。
     * @param id 非负DO编号。
     * @param status true为高电平，false为低电平。
     * @return 编号有效且客户端接受该请求时返回true，否则返回false。
     * @note DO的电气用途由现场接线决定，不预设为抱闸控制。
     */
    bool setDigitalOutput(int id, bool status);
    /**
     * @brief 查询当前位置，以其为目标、2米为搜索半径发起重定位并等待结果。
     * @return 位姿查询、请求下发和定位完成判断全部成功时返回true，否则返回false。
     * @note 定位状态等待默认60秒；不会自行建立连接。
     */
    bool relocate();

    /**
     * @brief 建立连接，再执行当前配置中启用的初始化动作。
     * @param seer_ip 机器人连接地址，含义与connect()相同。
     * @return 连接及启用的动作全部成功时返回true，失败时清理连接并返回false。
     * @note 默认先设置DO0高电平再重定位；两个动作都禁用时仅连接，不能与其他操作并发执行。
     */
    bool init(const std::string& seer_ip) override;
    /**
     * @brief 校验并下发开环速度，等待本次命令的应答。
     * @param vx 前后速度，m/s，正值向前。
     * @param vy 左右速度，m/s，正值向左。
     * @param wz 角速度，rad/s，正值逆时针。
     * @note 零速度允许；duration固定为800ms；目标固件到期行为待实机验证。非法参数不发送，
     *       不会自动停止此前的运动；客户端失败仅记录日志，本接口不返回执行结果。
     */
    void setVelocity(double vx, double vy, double wz) override;
    /** @brief AGV通过单次请求下发指令，无需周期刷新；该兼容接口为空操作。 */
    void send_loop() override;
    /**
     * @brief 设置本地取消标志并断开客户端全部连接，可重复调用。
     * @note 不发送停车或导航取消命令，关闭连接不等于确认机器人已经停下。
     */
    void moveshutdown() override;

    /**
     * @brief 按给定速度方向平移指定距离，并阻塞等待任务结果。
     * @param dist 正的有限距离，m。
     * @param vx X方向有限速度，m/s，正值向前。
     * @param vy Y方向有限速度，m/s，正值向左；与vx不能同时为0。
     * @return 参数有效、请求被接受且任务完成时返回true，其余情况返回false。
     * @note 应用合成线速度上限；任务等待默认5分钟，超时不自动取消机器人任务。
     */
    bool movebydistance(double dist, double vx, double vy) override;
    /**
     * @brief 按给定角速度方向旋转指定角度，并阻塞等待任务结果。
     * @param angle 正的有限转角，rad。
     * @param vw 非零有限角速度，rad/s，正值逆时针，负值顺时针。
     * @return 参数有效、请求被接受且任务完成时返回true，其余情况返回false。
     * @note 应用角速度绝对值上限；任务等待默认5分钟，超时不自动取消机器人任务。
     */
    bool rotatebyangle(double angle, double vw) override;
    /**
     * @brief 查询机器人在世界坐标系中的当前位姿。
     * @param x 成功时写入X坐标，m。
     * @param y 成功时写入Y坐标，m。
     * @param theta 成功时写入朝向角，rad，对应客户端的Pose.angle。
     * @return 查询成功返回true；失败返回false且三个输出参数均保持不变。
     */
    bool getpose(double& x, double& y, double& theta) override;
};

#endif
