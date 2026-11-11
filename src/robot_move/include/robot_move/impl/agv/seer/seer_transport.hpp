#ifndef ROBOT_MOVE_SEER_TRANSPORT_HPP
#define ROBOT_MOVE_SEER_TRANSPORT_HPP

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include "robot_move/impl/agv/seer/seer_types.hpp"

namespace robot_move::seer
{
/**
 * @brief 独占管理仙工TCP连接及报文事务，不解释JSON内容或机器人任务状态。
 * @note 请求通过同一把事务锁串行执行；对象销毁前调用方须结束并回收使用它的线程。
 */
class Transport
{
  public:
    /// 最近事务失败类型；供独占使用本对象的数据源区分恢复和诊断原因。
    enum class Error { None, NotConnected, Timeout, Protocol, Io };
    /** @brief 创建未连接的传输对象，四个socket句柄均初始化为无效状态。 */
    Transport() = default;
    /** @brief 关闭所有连接，不发送业务命令。 */
    ~Transport();
    /** @brief 禁止复制连接句柄，避免多个对象重复释放同一socket。 */
    Transport(const Transport&) = delete;
    /** @brief 禁止复制赋值，保持连接资源的独占所有权。 */
    Transport& operator=(const Transport&) = delete;

    /**
     * @brief 先断开旧连接，再连接状态、控制、导航和其他四个固定服务端口。
     * @param ip 机器人IPv4地址；兼容带端口后缀，但后缀不改变固定端口映射。
     * @return 四个端口全部成功返回true；任一失败则清理全部连接并返回false。
     * @note 每个端口连接尝试上限1秒；不发送DO、重定位等业务请求，不能与其他调用并发。
     */
    bool connect(const std::string& ip);
    /** @brief 仅连接指定通道；空/非法集合拒绝，重复通道只连接一次。 */
    bool connect(const std::string& ip, const std::vector<ApiChannel>& channels);
    /** @brief 获取最近事务结果；并发调用者应自行关联请求与结果。 */
    Error lastError();
    /** @brief 与事务串行地关闭全部连接并清空句柄，可重复调用，不发送停车指令。 */
    void disconnect();
    /**
     * @brief 关闭指定通道并将对应句柄置为无效，其他通道保持不变。
     * @param channel 要关闭的有效API通道，例如Client遇到非法JSON时使其失效。
     */
    void closeChannel(ApiChannel channel);
    /**
     * @brief 执行一次完整的报文请求/响应，只校验帧结构，不判断业务是否成功。
     * @param channel 使用的有效服务通道。
     * @param api 仙工请求编号，由Client选择，例如1004。
     * @param body 已序列化的正文；无正文请求传空字符串。
     * @param response 输出完整响应正文；进入函数时清空，无正文响应成功时为空。
     * @param deadline 调用方的绝对截止时间，实际取它与单次1秒时限中更早者。
     * @return 收齐并匹配响应头/正文返回true；连接无效、超时或帧错误返回false。
     * @note 锁等待、完整发送和完整接收共用截止时间。开始通信后的收发/帧错误会关闭
     *       选中通道，避免残包串入下一次请求；仅等待锁超时不会关闭通道。
     *       正文限制4MiB，压缩响应暂不支持；JSON与ret_code由Client检查。
     */
    bool request(ApiChannel channel, uint16_t api, const std::string& body, std::string& response,
                 Deadline deadline = Deadline::max());

  private:
    friend struct TransportTestAccess;
    /// 按ApiChannel顺序保存独占持有的socket；-1表示未连接或已失效。
    std::array<int, 4> sockets_{{-1, -1, -1, -1}};
    /// 全部通道共用的16位请求序号，递增后允许自然回绕。
    uint16_t sequence_ = 0;
    Error last_error_ = Error::None;
    /// 保护序号分配、完整请求/响应及连接关闭，避免并发串读。
    std::timed_mutex mutex_;
    /**
     * @brief 用非阻塞TCP连接指定IPv4端口，替换fd中原有的连接。
     * @param ip IPv4字符串。
     * @param port 目标TCP端口。
     * @param fd 成功时接收新句柄，失败时关闭新句柄并置为-1。
     * @return 地址有效且1秒内连接完成返回true，其余返回false。
     */
    bool connectPort(const std::string& ip, int port, int& fd);
    /**
     * @brief 将请求编号与正文编码为16字节头部加正文字节数组。
     * @param type 请求API编号。
     * @param json 正文原始字节，不在此检查是否为合法JSON。
     * @return 大端编码的完整请求帧，保留区填0，空正文也可正常编码。
     * @note 分配并递增请求序号；由已持有事务锁的request()调用。
     */
    std::vector<uint8_t> packFrame(uint16_t type, const std::string& json);
    /**
     * @brief 循环发送完整字节数组，处理部分发送及可重试的临时中断。
     * @param fd 已连接socket。
     * @param data 要发送的全部字节。
     * @param deadline 本次事务共享的绝对截止时间。
     * @return 全部发送完成返回true；超时、连接异常或不可恢复错误返回false。
     * @note 不自行关闭fd；request()负责失败清理，发送时抑制SIGPIPE。
     */
    bool sendAll(int fd, const std::vector<uint8_t>& data, Deadline deadline);
    /**
     * @brief 循环接收恰好size个字节，避免将TCP半包当作完整响应。
     * @param fd 已连接socket。
     * @param buffer 至少可容纳size个字节的输出缓冲区。
     * @param size 需要接收的字节数，为0时直接成功。
     * @param deadline 本次事务共享的绝对截止时间，不随分片到达而延长。
     * @return 收齐返回true；对端关闭、超时或不可恢复错误返回false。
     * @note 不追加字符串结束符，不自行关闭fd；按明确长度处理原始字节。
     */
    bool receiveExact(int fd, uint8_t* buffer, size_t size, Deadline deadline);
};
} // namespace robot_move::seer
#endif
