#ifndef ROBOT_MOVE_SEER_TYPES_HPP
#define ROBOT_MOVE_SEER_TYPES_HPP

#include <chrono>
#include <cstddef>

namespace robot_move::seer
{
/// 单调时钟上的绝对截止时间，逐层传递时不重新计算完整超时。
/// Deadline::max()表示调用方不额外限制；真实客户端/传输层仍有单次事务时限。
using Deadline = std::chrono::steady_clock::time_point;
/** @brief 机器人在世界坐标系中的二维位姿，供客户端与移动策略层交换。 */
struct Pose
{
    /// 世界坐标系X坐标，m。
    double x = 0;
    /// 世界坐标系Y坐标，m。
    double y = 0;
    /// 朝向角，rad。
    double angle = 0;
};
/** @brief API服务通道；枚举顺序对应Transport内部连接数组，数值本身不是TCP端口号。 */
enum class ApiChannel : std::size_t
{
    /// 状态查询通道，TCP端口19204。
    Status,
    /// 速度控制、重定位等控制通道，TCP端口19205。
    Control,
    /// 平移、旋转等导航任务通道，TCP端口19206。
    Navigation,
    /// DO等其他操作通道，TCP端口19210。
    Other
};
/** @brief 导航查询返回的任务状态，保持仙工协议的原始数值语义。 */
enum class NavigationStatus
{
    /// 当前无任务，不能当成任务已经完成。
    None = 0,
    /// 任务等待中。
    Waiting = 1,
    /// 任务执行中。
    Running = 2,
    /// 任务暂停，是否继续等待由移动策略层决定。
    Suspended = 3,
    /// 任务已完成。
    Completed = 4,
    /// 任务执行失败。
    Failed = 5,
    /// 任务已取消。
    Canceled = 6
};
/** @brief 定位查询返回的状态；Completed是旧版“完成但尚未确认”，不等同于Success。 */
enum class RelocationStatus
{
    /// 定位初始化中。
    Init = 0,
    /// 定位成功。
    Success = 1,
    /// 正在重定位。
    Relocating = 2,
    /// 旧版重定位完成，仍需确认并等待Success。
    Completed = 3
};
} // namespace robot_move::seer
#endif
