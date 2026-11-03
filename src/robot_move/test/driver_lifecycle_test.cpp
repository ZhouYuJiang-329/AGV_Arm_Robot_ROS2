#include "robot_move/factory/robot_move_factory.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <type_traits>

void check(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
size_t descriptorCount()
{
    size_t count = 0;
    for (const auto& entry : std::filesystem::directory_iterator("/proc/self/fd"))
    {
        (void)entry;
        ++count;
    }
    return count;
}
int main()
{
    try
    {
        static_assert(std::has_virtual_destructor<RobotMove>::value, "base destructor must be virtual");
        static_assert(!std::is_copy_constructible<RobotMove>::value, "connections must not be copied");
        check(!robot_move::createRobotMove("unknown"), "unknown type accepted");
        for (const std::string type : {"air", "mag", "duct"})
        {
            const auto before = descriptorCount();
            {
                auto robot = robot_move::createRobotMove(type);
                check(!robot->init("/dev/null"), "non-TTY initialization succeeded");
                int sentinel = open("/dev/null", O_RDWR);
                check(sentinel >= 0, "sentinel open failed");
                robot->moveshutdown();
                robot->moveshutdown();
                check(fcntl(sentinel, F_GETFD) >= 0, "shutdown closed a reused descriptor");
                robot.reset();
                check(fcntl(sentinel, F_GETFD) >= 0, "destructor closed a reused descriptor");
                close(sentinel);
            }
            check(descriptorCount() == before, "failed initialization leaked descriptors");

            int master = posix_openpt(O_RDWR | O_NOCTTY);
            check(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0, "PTY creation failed");
            const std::string port = ptsname(master);
            const auto count = descriptorCount();
            {
                auto robot = robot_move::createRobotMove(type);
                check(robot->init(port), "mock serial initialization failed");
                check(descriptorCount() == count + 1, "serial connection missing");
                check(robot->init(port), "serial reinitialization failed");
                check(descriptorCount() == count + 1, "reinitialization leaked serial connection");
            }
            check(descriptorCount() == count, "polymorphic destruction leaked serial connection");
            close(master);
        }
        auto robot = robot_move::createRobotMove("agv");
        const auto before = descriptorCount();
        check(!robot->init("invalid-ip"), "invalid IP accepted");
        robot->moveshutdown();
        robot->moveshutdown();
        robot.reset();
        check(descriptorCount() == before, "AGV failed initialization leaked descriptors");
        std::cout << "PASS: factory, polymorphic destruction, cleanup and reinitialization\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
