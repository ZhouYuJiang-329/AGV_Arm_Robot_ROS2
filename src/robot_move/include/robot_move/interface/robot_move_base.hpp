#ifndef ROBOT_MOVE_BASE_HPP
#define ROBOT_MOVE_BASE_HPP

#include <string>

/** @brief 底盘驱动的统一接口，提供连接、速度控制和可选的任务操作。 */
class RobotMove
{
public:
    /** @brief 创建驱动基类；实际连接由派生类的 init() 建立。 */
    RobotMove() = default;
    /** @brief 支持通过基类指针销毁派生驱动，正确释放派生类持有的资源。 */
    virtual ~RobotMove() = default;
    /** @brief 禁止复制构造，避免两个驱动对象共同持有同一连接资源。 */
    RobotMove(const RobotMove&) = delete;
    /** @brief 禁止复制赋值，避免覆盖或重复释放已有连接。 */
    RobotMove& operator=(const RobotMove&) = delete;
    /** @brief 禁止移动构造，保持驱动对象及其并发操作的归属稳定。 */
    RobotMove(RobotMove&&) = delete;
    /** @brief 禁止移动赋值，避免转移正在被其他操作使用的驱动资源。 */
    RobotMove& operator=(RobotMove&&) = delete;

    /**
     * @brief 建立连接并执行具体底盘定义的初始化流程。
     * @param connection_str 连接地址；AGV使用机器人IP，串口底盘使用设备路径。
     * @return 初始化成功返回true，失败返回false。
     * @note 初始化可能包含设备动作，具体流程由派生类及其配置决定。
     */
    virtual bool init(const std::string& connection_str) = 0;
    /**
     * @brief 设置底盘的目标线速度和角速度。
     * @param vx X方向线速度；AGV中单位为m/s，正值向前。
     * @param vy Y方向线速度；AGV中单位为m/s，正值向左，部分底盘不支持此方向。
     * @param wz 旋转角速度；AGV中单位为rad/s，正值逆时针。
     * @note 下发方式、限速规则和失败反馈由具体底盘实现决定。
     */
    virtual void setVelocity(double vx, double vy, double wz) = 0;
    /** @brief 执行一次周期发送；需要持续刷新协议数据的底盘由节点定时调用。 */
    virtual void send_loop() = 0;
    /**
     * @brief 执行具体底盘的关闭流程并释放连接。
     * @note 不等同于统一的硬件急停；AGV实现仅关闭连接，其他底盘可包含停机序列。
     */
    virtual void moveshutdown() = 0;

    /**
     * @brief 请求结束本地阻塞操作，便于调用方回收任务线程。
     * @note 基类默认不处理；此接口不负责向机器人发送停车或任务取消命令。
     */
    virtual void cancelPendingOperations() {}

    /**
     * @brief 可选的扫描器配置接口，不支持的底盘默认不处理。
     * @details 参数依次为扫描速度、扫描距离和扫描精度，单位及编码由具体实现约定。
     */
    virtual void setScannerConfig(double /*speed*/, double /*distance*/, double /*precision*/) {}
    /** @brief 可选的扫描器触发接口，不支持的底盘默认不处理。 */
    virtual void scannercontrol() {}

    /**
     * @brief 可选的定距离平移接口，不支持的底盘默认返回false。
     * @details 参数依次为距离dist、X方向速度vx、Y方向速度vy。
     *          AGV要求距离为正数（m），速度单位为m/s，方向由速度正负决定。
     * @return AGV实现等待任务完成后返回结果；其他实现按各自能力处理。
     */
    virtual bool movebydistance(double /*dist*/, double /*vx*/, double /*vy*/) { return false; }
    /**
     * @brief 可选的定角度旋转接口，不支持的底盘默认返回false。
     * @details 参数依次为转角angle和角速度vw；AGV要求转角为正数（rad），
     *          方向由角速度正负决定（rad/s，正值逆时针）。
     * @return AGV实现等待任务完成后返回结果；其他实现按各自能力处理。
     */
    virtual bool rotatebyangle(double /*angle*/, double /*vw*/) { return false; }
    /**
     * @brief 可选的位姿查询接口，不支持的底盘默认返回false且不修改输出。
     * @details 三个引用参数依次接收x、y和theta；AGV返回世界坐标系位置（m）和朝向（rad）。
     * @return 查询成功返回true，失败返回false。
     */
    virtual bool getpose(double& /*x*/, double& /*y*/, double& /*theta*/) { return false; }
};

#endif
