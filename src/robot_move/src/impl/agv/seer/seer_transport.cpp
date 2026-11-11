#include "robot_move/impl/agv/seer/seer_transport.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <iostream>

namespace robot_move::seer
{
namespace
{
/**
 * @brief 在绝对截止时间前等待socket出现指定事件。
 * @param sock_fd 已创建的socket。
 * @param events 期望的poll事件，发送用POLLOUT，接收用POLLIN。
 * @param deadline 当前操作共享的绝对截止时间。
 * @return 指定事件就绪返回true；超时或不可恢复错误返回false。
 * @note 信号中断后用剩余时间重试，不重新开始一段完整超时。
 */
bool waitForSocket(int sock_fd, short events, std::chrono::steady_clock::time_point deadline)
{
    while (true)
    {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - std::chrono::steady_clock::now())
                             .count();
        if (remaining <= 0)
        {
            errno = ETIMEDOUT;
            return false;
        }
        pollfd descriptor{sock_fd, events, 0};
        int result = poll(&descriptor, 1, static_cast<int>(remaining));
        if (result > 0)
            return (descriptor.revents & events) != 0;
        if (result == 0)
        {
            errno = ETIMEDOUT;
            return false;
        }
        if (errno != EINTR)
            return false;
    }
}
} // namespace

/** @brief 清理旧句柄后执行非阻塞连接，用就绪事件和SO_ERROR共同判断是否真正连通。 */
bool Transport::connectPort(const std::string& ip, int port, int& sock_fd)
{
    if (sock_fd >= 0)
        close(sock_fd);
    sock_fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (sock_fd < 0)
        return false;
    // 连接失败后立即清空句柄，避免后续关闭时误操作已经被系统复用的描述符。
    auto fail = [&]()
    {
        close(sock_fd);
        sock_fd = -1;
        return false;
    };
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
        return fail();

    if (::connect(sock_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        if (errno != EINPROGRESS)
            return fail();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        if (!waitForSocket(sock_fd, POLLOUT, deadline))
            return fail();
        int error = 0;
        socklen_t size = sizeof(error);
        if (getsockopt(sock_fd, SOL_SOCKET, SO_ERROR, &error, &size) < 0 || error != 0)
            return fail();
    }
    return true;
}

/** @brief 按仙工格式生成16字节头部，再追加正文；所有多字节整数使用大端序。 */
std::vector<uint8_t> Transport::packFrame(uint16_t type, const std::string& json)
{
    constexpr size_t HEADER_SIZE = 16;
    constexpr uint8_t SYNC = 0x5A;
    constexpr uint8_t VERSION = 0x01; // RBK3.4协议版本；RBK3.5使用0x02。

    // 先创建全零头部，包含必须保留的6个字节。
    std::vector<uint8_t> frame(HEADER_SIZE, 0);
    frame.reserve(HEADER_SIZE + json.size());

    // 将16位整数拆成两个字节，高位在前。
    auto writeU16 = [&frame](size_t offset, uint16_t value)
    {
        frame[offset] = (value >> 8) & 0xFF;
        frame[offset + 1] = value & 0xFF;
    };

    // 将32位整数拆成四个字节，高位在前。
    auto writeU32 = [&frame](size_t offset, uint32_t value)
    {
        frame[offset] = (value >> 24) & 0xFF;
        frame[offset + 1] = (value >> 16) & 0xFF;
        frame[offset + 2] = (value >> 8) & 0xFF;
        frame[offset + 3] = value & 0xFF;
    };

    frame[0] = SYNC;
    frame[1] = VERSION;
    writeU16(2, ++sequence_);                        // 请求序号。
    writeU32(4, static_cast<uint32_t>(json.size())); // 正文字节数。
    writeU16(8, type);                               // API请求编号。

    // 空正文直接保留16字节头部，不会访问越界元素。
    frame.insert(frame.end(), json.begin(), json.end());
    return frame;
}

/** @brief 循环发送剩余字节，统一处理部分发送、临时不可写及信号中断。 */
bool Transport::sendAll(int sock_fd, const std::vector<uint8_t>& data, Deadline deadline)
{
    size_t sent = 0;
    while (sent < data.size())
    {
        if (!waitForSocket(sock_fd, POLLOUT, deadline))
            return false;
        ssize_t count =
            send(sock_fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
            continue;
        if (count <= 0)
            return false;
        sent += static_cast<size_t>(count);
    }
    return true;
}

/** @brief 循环收齐指定长度，既不多取下一帧，也不把半包交给上层。 */
bool Transport::receiveExact(int sock_fd, uint8_t* buffer, size_t size, Deadline deadline)
{
    size_t received = 0;
    while (received < size)
    {
        if (!waitForSocket(sock_fd, POLLIN, deadline))
            return false;
        ssize_t count = recv(sock_fd, buffer + received, size - received, MSG_DONTWAIT);
        if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
            continue;
        if (count <= 0)
            return false;
        received += static_cast<size_t>(count);
    }
    return true;
}

/** @brief 串行组包、完整发送、按长度接收及匹配响应；失败时按帧完整性清理连接。 */
bool Transport::request(ApiChannel channel, uint16_t api, const std::string& request_json,
                        std::string& response_json, Deadline deadline)
{
    response_json.clear();
    // 锁等待也受上层任务剩余时间约束，不能在排队后重新获得完整的事务时限。
    deadline = std::min(deadline, std::chrono::steady_clock::now() + std::chrono::seconds(1));
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    if (!lock.try_lock_until(deadline))
        return false;
    last_error_ = Error::None;
    if (std::chrono::steady_clock::now() >= deadline)
    {
        last_error_ = Error::Timeout;
        std::cerr << "[AGV] API " << api << " 等待请求锁超时" << std::endl;
        return false;
    }
    int& sock_fd = sockets_.at(static_cast<size_t>(channel));
    if (sock_fd < 0)
    {
        last_error_ = Error::NotConnected;
        return false;
    }

    // 收发中断或帧校验失败后，连接中的边界不再可信；关闭它以免迟到响应串入下一次请求。
    auto fail = [&](const char* reason, Error error = Error::Io)
    {
        last_error_ = error == Error::Io && (errno == ETIMEDOUT || std::chrono::steady_clock::now() >= deadline)
                          ? Error::Timeout : error;
        std::cerr << "[AGV] API " << api << " 请求失败: " << reason << std::endl;
        close(sock_fd);
        sock_fd = -1;
        return false;
    };
    const auto frame = packFrame(api, request_json);
    errno = 0;
    if (!sendAll(sock_fd, frame, deadline))
        return fail("发送超时或连接异常");

    // 先收齐固定头部，再依据头中的长度接收正文。
    std::array<uint8_t, 16> header{};
    if (!receiveExact(sock_fd, header.data(), header.size(), deadline))
        return fail("响应头不完整、超时或连接关闭");
    // 从大端头部提取序号、类型及正文长度，与本次请求逐项匹配。
    const uint16_t sequence = (uint16_t(header[2]) << 8) | header[3];
    const uint16_t expected_sequence = (uint16_t(frame[2]) << 8) | frame[3];
    const uint16_t type = (uint16_t(header[8]) << 8) | header[9];
    const uint32_t length = (uint32_t(header[4]) << 24) | (uint32_t(header[5]) << 16) |
                            (uint32_t(header[6]) << 8) | header[7];
    if (header[0] != 0x5A || header[1] != frame[1] || sequence != expected_sequence ||
        type != api + 10000)
        return fail("响应同步头、版本、序号或编号不匹配", Error::Protocol);
    // 保留区第5字节标识压缩格式；当前只支持未压缩数据。
    if (header[14] != 0)
        return fail("暂不支持压缩响应", Error::Protocol);
    constexpr uint32_t MAX_RESPONSE_SIZE = 4 * 1024 * 1024;
    if (length > MAX_RESPONSE_SIZE)
        return fail("响应正文超过长度限制", Error::Protocol);

    std::vector<uint8_t> body(length);
    if (!receiveExact(sock_fd, body.data(), body.size(), deadline))
        return fail("响应正文不完整、超时或连接关闭");
    // 正文作为原始字节字符串交给Client，不在传输层解释JSON或API错误码。
    if (!body.empty())
        response_json.assign(body.begin(), body.end());
    return true;
}

/** @brief 统一释放所有socket，析构不发送机器人业务指令。 */
Transport::~Transport()
{
    disconnect();
}

/** @brief 建立四端口连接，任一端口失败则关闭本轮已建立的连接。 */
bool Transport::connect(const std::string& address)
{
    return connect(address, {ApiChannel::Other, ApiChannel::Status,
                             ApiChannel::Control, ApiChannel::Navigation});
}

bool Transport::connect(const std::string& address, const std::vector<ApiChannel>& channels)
{
    disconnect();
    if (channels.empty())
        return false;
    for (auto channel : channels)
        if (static_cast<size_t>(channel) >= sockets_.size())
            return false;
    const auto ip = address.substr(0, address.find(':'));
    // 通道数组按状态、控制、导航、其他排列；连接尝试顺序保持既有行为。
    constexpr std::array<int, 4> ports{{19204, 19205, 19206, 19210}};
    bool success = true;
    std::array<bool, 4> visited{};
    for (ApiChannel channel : channels)
    {
        const auto i = static_cast<size_t>(channel);
        if (visited[i])
            continue;
        visited[i] = true;
        const bool connected = connectPort(ip, ports[i], sockets_[i]);
        if (!connected)
            std::cerr << "[Seer] 连接端口失败: " << ports[i] << std::endl;
        success = connected && success;
    }
    if (!success)
        disconnect();
    return success;
}

Transport::Error Transport::lastError()
{
    std::lock_guard<std::timed_mutex> lock(mutex_);
    return last_error_;
}

/** @brief 在事务锁保护下关闭全部通道并清空句柄，重复调用不会误关复用后的描述符。 */
void Transport::disconnect()
{
    std::lock_guard<std::timed_mutex> lock(mutex_);
    for (int& fd : sockets_)
    {
        if (fd >= 0)
            close(fd);
        fd = -1;
    }
}

/** @brief 仅使指定通道失效，供客户端在发现非法JSON时清理对应连接。 */
void Transport::closeChannel(ApiChannel channel)
{
    std::lock_guard<std::timed_mutex> lock(mutex_);
    int& fd = sockets_.at(static_cast<size_t>(channel));
    if (fd >= 0)
        close(fd);
    fd = -1;
}
} // namespace robot_move::seer
