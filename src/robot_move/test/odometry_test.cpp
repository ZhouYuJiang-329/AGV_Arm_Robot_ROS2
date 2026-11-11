#include "seer_test_support.hpp"
#include "robot_move/odometry/integrator.hpp"
#include <cmath>
namespace robot_move::seer
{
struct VelocitySourceTestAccess
{
  static void socket(SeerVelocitySource & source, int fd)
  {TransportTestAccess::statusSocket(source.transport_) = fd;}
};
}
using namespace robot_move::odometry;
using robot_move::seer::SeerVelocitySource;
void near(double actual, double expected)
{
  require(std::abs(actual - expected) < 1e-9, "numeric mismatch");
}
int main()
{
  try {
    robot_move::seer::Velocity v{9, 9, 9};
    for (const auto & bad : {"{}", "[]", "{\"vx\":0,\"vy\":0}",
        "{\"vx\":true,\"vy\":0,\"w\":0}", "{\"vx\":1e999,\"vy\":0,\"w\":0}",
        "{\"vx\":0,\"vy\":null,\"w\":0}", "{\"r_vx\":0,\"r_vy\":0,\"r_w\":0}",
        "{\"vx\":0,\"vy\":0,\"w\":0,\"ret_code\":1}",
        "{\"vx\":0,\"vy\":0,\"w\":0} trailing"})
    {
      require(!SeerVelocitySource::parse(bad, v), "invalid actual feedback accepted");
      near(v.vx, 9);
    }
    require(SeerVelocitySource::parse("{\"vx\":0,\"vy\":1,\"w\":0}", v), "valid feedback rejected");
    auto transformed = toBase({1, 2, 3}, 2, 1, std::acos(-1.0) / 2);
    near(transformed.vx, 8); near(transformed.vy, 2); near(transformed.w, 3);
    for (const auto & velocity : {Velocity{0, 0, 0}, Velocity{1, 0, 0}, Velocity{0, 1, 0},
        Velocity{0, 0, 1}, Velocity{1, 1, 1}})
    {
      Integrator i(2);
      require(i.accept(velocity, 100), "first rejected"); near(i.x, 0);
      require(i.accept(velocity, 101), "second rejected");
      if (velocity.w == 0) {near(i.x, velocity.vx); near(i.y, velocity.vy);} else {
        near(i.x, velocity.vx * std::sin(1.0) - velocity.vy * (1 - std::cos(1.0)));
        near(i.y, velocity.vx * (1 - std::cos(1.0)) + velocity.vy * std::sin(1.0));
      }
      near(std::hypot(std::sin(i.yaw / 2), std::cos(i.yaw / 2)), 1);
    }
    Integrator i(2);
    i.accept({1, 0, 0}, 0, 10); i.accept({1, 0, 0}, 1, 11); near(i.x, 1);
    require(!i.accept({1, 0, 0}, 1.5, 11), "duplicate accepted"); near(i.x, 1);
    i.accept({1, 0, 0}, 2, 9); near(i.x, 1); require(i.degraded, "clock reset not degraded");
    i.accept({1, 0, 0}, 1); near(i.x, 1); // receive time reversal
    i.accept({1, 0, 0}, 10); near(i.x, 1); // long gap
    i.accept({1, 0, 0}, 11); near(i.x, 2);
    i.gap(); i.accept({1, 0, 0}, 100); near(i.x, 2);
    i.accept({1, 0, 0}, 101); near(i.x, 3);
    require(!i.accept({NAN, 0, 0}, 102), "NaN accepted");
    Integrator turn(2);
    for (int n = 0; n < 20; ++n) {
      turn.accept({0, 0, 1}, n);
    }
    near(turn.yaw, std::remainder(19.0, 2 * std::acos(-1.0)));
        // Real source + transport, fragmented responses, malformed data and timeout.
    withServer([](int fd) {
        auto request = readRequest(fd);
        require(request.size() == 16 && request[8] == 3 && request[9] == 237, "not empty API 1005");
        auto reply = response(request, "{\"vx\":1,\"vy\":2,\"w\":3}");
        writeAll(fd, reply.data(), 5); writeAll(fd, reply.data() + 5, reply.size() - 5);
        request = readRequest(fd);
        reply = response(request, "{\"vx\":1,\"vy\":2}");
        writeAll(fd, reply.data(), reply.size());
        }, [](int & fd) {
        SeerVelocitySource source("unused");
        robot_move::seer::VelocitySourceTestAccess::socket(source, fd); fd = -1;
        Velocity value;
        require(source.read(value, robot_move::seer::Deadline::max()), "local source failed");
        near(value.vy, 2);
        require(!source.read(value, robot_move::seer::Deadline::max()),
        "partial response accepted"); near(value.w, 3);
        });
    withServer([](int fd) {
        readRequest(fd); std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }, [](int & fd) {
        SeerVelocitySource source("unused");
        robot_move::seer::VelocitySourceTestAccess::socket(source, fd); fd = -1;
        Velocity value;
        const auto start = std::chrono::steady_clock::now();
        require(!source.read(value, start + std::chrono::milliseconds(40)), "timeout accepted");
        require(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(130),
        "deadline exceeded");
        });
    withServer([](int fd) {readRequest(fd);}, [](int & fd) {
        SeerVelocitySource source("unused");
        robot_move::seer::VelocitySourceTestAccess::socket(source, fd); fd = -1;
        Velocity value;
        require(!source.read(value, robot_move::seer::Deadline::max()), "disconnect accepted");
        });
    std::cout << "odometry_test passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
