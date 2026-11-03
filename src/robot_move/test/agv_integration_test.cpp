#include "seer_test_support.hpp"
#include "robot_move/impl/agv/agv_robot_move.hpp"
#include "robot_move/impl/agv/seer/seer_client.hpp"
using namespace robot_move::seer;
struct AGVRobotMoveTestAccess
{
    static bool initializeActions(AGVRobotMove& robot)
    {
        return robot.initializeActions();
    }
};
int main()
{
    try
    {
        withServer(
            [](int fd)
            {
                const auto req = readRequest(fd);
                require(req[8] == 0x17 && req[9] == 0x71, "expected DO command");
                const auto bytes = response(req, "{}");
                writeAll(fd, bytes.data(), bytes.size());
            },
            [](int& do_fd)
            {
                withServer(
                    [](int fd)
                    {
                        for (int step = 0; step < 4; ++step)
                        {
                            const auto req = readRequest(fd);
                            require(((int(req[8]) << 8) | req[9]) == (step == 0 ? 1004 : 1020),
                                    "unexpected status API");
                            const std::string body = step == 0
                                                         ? R"({"x":1,"y":2,"angle":0.5})"
                                                         : "{\"task_status\":" +
                                                               std::to_string(step == 1   ? 0
                                                                              : step == 2 ? 2
                                                                                          : 4) +
                                                               "}";
                            const auto bytes = response(req, body);
                            writeAll(fd, bytes.data(), bytes.size());
                        }
                        uint8_t byte;
                        require(recv(fd, &byte, 1, 0) == 0, "rejected movement still polled");
                    },
                    [&](int& status_fd)
                    {
                        withServer(
                            [](int fd)
                            {
                                for (int step = 0; step < 2; ++step)
                                {
                                    const auto req = readRequest(fd);
                                    require(req[8] == 0x0B && req[9] == 0xEF,
                                            "expected translate command");
                                    const auto bytes =
                                        response(req, step == 0 ? "{}" : R"({"ret_code":42})");
                                    writeAll(fd, bytes.data(), bytes.size());
                                }
                            },
                            [&](int& navigation_fd)
                            {
                                auto transport = std::make_unique<Transport>();
                                TransportTestAccess::doSocket(*transport) = do_fd;
                                TransportTestAccess::statusSocket(*transport) = status_fd;
                                TransportTestAccess::navigationSocket(*transport) = navigation_fd;
                                do_fd = status_fd = navigation_fd =
                                    -1; // Driver owns these mock connections.
                                std::unique_ptr<RobotMove> owner = std::make_unique<AGVRobotMove>(
                                    std::make_unique<Client>(std::move(transport)));
                                auto& robot = static_cast<AGVRobotMove&>(*owner);
                                AGVRobotMove::InitOptions options;
                                options.set_do = false;
                                options.relocate = false;
                                require(robot.setInitOptions(options) &&
                                            AGVRobotMoveTestAccess::initializeActions(robot),
                                        "disabled init actions failed");
                                require(robot.setDigitalOutput(9, false), "DO failed");
                                double x = 0, y = 0, angle = 0;
                                require(robot.getpose(x, y, angle) && x == 1 && y == 2 &&
                                            angle == .5,
                                        "pose failed");
                                require(robot.movebydistance(1, -.1, 0), "movement failed");
                                require(!robot.movebydistance(1, -.1, 0),
                                        "rejected movement succeeded");
                            });
                    });
            });
        std::cout << "PASS: real AGV -> Client -> Transport -> local server\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
