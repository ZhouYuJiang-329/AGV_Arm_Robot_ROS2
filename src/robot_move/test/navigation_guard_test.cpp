#include "robot_move/navigation/worker.hpp"
#include "robot_move/impl/agv/seer/seer_client.hpp"
#include "seer_test_support.hpp"
#include <atomic>
using namespace robot_move::navigation;
using namespace std::chrono_literals;
namespace
{
Time at(double s)
{
  return Time{} + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
}
Config config() {Config c; c.motion_enabled = c.freshness_verified = true; return c;}
void health(Guard & g, double t, double ros = 100)
{
  for (const auto & item : g.inputs) {
    g.input(item.first, ros + t, ros + t, true, at(t));
  }
}
Guard ready()
{
  Guard g(config());
  for (auto name : {"scan_front", "scan_back", "odom", "tf:map->odom"}) {
    g.require(name, 0.6);
  }
  health(g, 0); g.tick(at(0)); return g;
}
void core()
{
  Guard disabled(Config{});
  disabled.require("odom", 1);
  disabled.input("odom", 100, 100, true, at(0));
  disabled.command({.1, 0, 0}, true, at(0));
  require(!disabled.next(at(0)) && !disabled.confirmed(at(1)), "default commissioning gates");
  disabled.command({}, true, at(.1));
  require(!disabled.next(at(.1)) && !disabled.last_command,
    "disabled idle zero must not issue API2010 or acquire a shutdown stop obligation");
  auto idle = ready();
  idle.command({}, true, at(.1));
  require(!idle.next(at(.1)) && idle.state == "Ready" && !idle.last_command,
    "idle zero before any motion must not latch navigation or cancel another task");
  auto g = ready();
  require(g.state == "Ready" && !g.next(at(0)), "startup must not send motion/zero");
  for (int i = 0; i < 10; ++i) {
    double t = i * 0.1; health(g, t); g.command({0.1, 0, 0}, true, at(t));
    auto v = g.next(at(t)); require(v && v->x == .1, "normal fresh stream");
    g.result(*v, true, at(t));
  }
  health(g, 1.4); auto zero = g.next(at(1.4));
  require(zero && zero->x == 0 && g.state == "StopRequested", "command interruption must zero");
  require(!g.confirmed(at(1.4)), "request is not stop confirmation");
  g.result(*zero, true, at(1.4)); require(!g.confirmed(at(1.4)), "ACK is not stop confirmation");
  health(g, 1.5); g.feedback(0, 0, at(1.5), true);
  health(g, 2); g.feedback(0, 0, at(2), true);
  require(g.confirmed(at(2)), "fresh low measured speed sustained");
  g.command({.1, 0, 0}, true, at(2)); require(!g.next(at(2)), "requires explicit recovery");
  require(g.recover(at(2)), "healthy stopped recovery");
  require(!g.next(at(2)), "recovery must discard cached target");
  require(!g.confirmed(at(2.7)), "stale measured zero cannot confirm stopped");
  for (const auto & name : {"scan_front", "scan_back", "odom", "tf:map->odom"}) {
    auto h = ready(); h.command({.1, 0, 0}, true, at(0)); h.next(at(0));
    health(h, .4); h.inputs.at(name).valid = false;
    auto v = h.next(at(.41)); require(v && v->x == 0 && h.state == "Fault", "input failure zero");
    health(h, .5); require(!h.next(at(.5)), "input recovery cannot resurrect speed");
  }
  auto h = ready(); h.command({.1, 0, 0}, true, at(0));
  h.command({NAN, 0, 0}, true, at(.1)); auto v = h.next(at(.1));
  require(v && v->x == 0 && h.state == "Fault", "invalid cancels queued nonzero");
  require(h.last_command == at(0), "invalid must not refresh watchdog");
  h.result(*v, false, at(.1)); require(h.next(at(.2)).has_value(), "bounded stop retry");
  h.result({}, false, at(.2)); require(!h.next(at(.3)), "no infinite zeros");
  require(!h.recover(at(.3)), "no recovery without ACK");
  auto f = ready(); f.command({.1, 0, 0}, true, at(0)); f.next(at(0));
  f.result({.1, 0, 0}, false, at(.01));
  require(f.next(at(.02))->x == 0 && f.state == "Fault", "control failure stop");
  auto stale = ready();
  require(!stale.input("odom", 99, 100, true, at(.1)), "old stamp rejected");
  require(!stale.input("odom", 101, 100, true, at(.1)), "future stamp rejected");
  auto repeat = ready();
  require(!repeat.input("odom", 100, 100.5, true, at(.5)), "duplicates rejected");
  require(!repeat.healthy(at(.61)), "duplicate cannot extend freshness");
  auto explicit_zero = ready(); explicit_zero.command({.1, 0, 0}, true, at(0));
  explicit_zero.command({}, true, at(.1));
  require(explicit_zero.last_command == at(.1), "zero command receive age recorded");
  require(explicit_zero.next(at(.1))->x == 0, "zero priority");
  explicit_zero.feedback(0, 0, at(.2), true); explicit_zero.command({}, true, at(.3));
  health(explicit_zero, .7); explicit_zero.feedback(0, 0, at(.7), true);
  // A gap in feedback must restart the confirmation hold interval.
  require(!explicit_zero.confirmed(at(.7)), "feedback gap restarts hold");
  bool rejected = false; try {
    auto c = config(); c.duration_ms = 0; Guard invalid(c);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "duration zero forbidden");
}
void commanded_zero()
{
  auto g = ready();
  g.command({.04, 0, 0}, true, at(0));
  g.result(*g.next(at(0)), true, at(0));
  g.command({}, true, at(.1));
  g.command({.05, 0, 0}, true, at(.11));
  auto zero = g.next(at(.12));
  require(zero && zero->x == 0 && g.state == "CommandedStop",
    "ordinary Nav2 zero must stop without latching recovery");
  g.command({.05, 0, 0}, true, at(.13));
  require(!g.next(at(.13)), "motion must wait for the outstanding zero ACK");
  g.result(*zero, true, at(.14));
  require(!g.next(at(.15)), "zero ACK must not replay a queued motion target");
  g.command({.05, 0, 0}, true, at(.2));
  auto next = g.next(at(.2));
  require(next && next->x == .05 && g.state == "Active",
    "a new healthy Nav2 command must resume after an acknowledged ordinary zero");
  g.result(*next, true, at(.2));
  g.command({}, true, at(.3)); zero = g.next(at(.3));
  g.result(*zero, true, at(.3));
  for (double t : {.4, .8, 1.2}) {
    health(g, t); g.feedback(0, 0, at(t), true);
    require(!g.next(at(t)) && g.state == "CommandedStop",
      "an acknowledged commanded stop must remain idle without repeated API zeros");
  }
  require(g.confirmed(at(1.2)), "ordinary commanded stop still needs fresh stop feedback");
  health(g, 1.3); g.command({.04, 0, 0}, true, at(1.3));
  require(g.next(at(1.3)).has_value(), "a later goal may send a new command after idle");
  health(g, 1.81); zero = g.next(at(1.81));
  require(zero && g.state == "StopRequested", "moving command timeout must still latch");
  g.result(*zero, true, at(1.81));
  health(g, 1.9); g.command({}, true, at(1.9));
  g.command({.04, 0, 0}, true, at(1.91));
  require(!g.next(at(1.91)) && g.state == "StopRequested",
    "normal zeros and new commands cannot clear a latched timeout");

  for (const auto & name : {"scan_front", "scan_back", "odom", "tf:map->odom"}) {
    auto fault = ready();
    fault.command({.04, 0, 0}, true, at(0)); fault.next(at(0));
    fault.command({}, true, at(.1));
    fault.result(*fault.next(at(.1)), true, at(.1));
    fault.inputs.at(name).valid = false; fault.next(at(.2));
    require(fault.state == "Fault", "input loss during commanded stop must still latch");
    health(fault, .3); fault.command({}, true, at(.3));
    fault.command({.04, 0, 0}, true, at(.31));
    require(!fault.next(at(.31)) && fault.state == "Fault",
      "input recovery cannot restart a faulted commanded stop");
  }
  auto failed = ready();
  failed.command({.04, 0, 0}, true, at(0)); failed.next(at(0));
  failed.command({}, true, at(.1)); zero = failed.next(at(.1));
  failed.result(*zero, false, at(.1));
  failed.command({.04, 0, 0}, true, at(.11));
  zero = failed.next(at(.12));
  require(zero && zero->x == 0 && failed.state == "Fault", "zero failure must fault and retry");
  failed.result(*zero, false, at(.12));
  failed.command({}, true, at(.13));
  require(!failed.next(at(.14)), "repeated upstream zeros cannot reset failed-stop retry budget");
}
void worker()
{
  std::mutex log_mutex; std::vector<Velocity> sent; std::atomic<bool> entered{false};
  Worker w(config(), [&](Velocity v, int duration, Time deadline) {
      require(duration == 800, "finite duration field");
      {std::lock_guard<std::mutex> lock(log_mutex); sent.push_back(v);}
      if (v.x != 0) {entered = true; std::this_thread::sleep_until(deadline); return false;}
      return true;
    });
  {std::lock_guard<std::mutex> lock(w.mutex); w.guard.require("odom", 1);
    auto t = Clock::now(); w.guard.input("odom", 100, 100, true, t);
    w.guard.command({.1, 0, 0}, true, t);}
  w.start();
  auto until = Clock::now() + 1s;
  while (!entered && Clock::now() < until) {std::this_thread::sleep_for(1ms);}
  require(entered, "worker dispatched"); auto start = Clock::now();
  {std::lock_guard<std::mutex> lock(w.mutex);
    w.guard.command({.12, 0, 0}, true, Clock::now());
    w.guard.stop("input_fault", true, Clock::now());}
  require(Clock::now() - start < 50ms, "slow I/O must not hold state lock");
  w.shutdown(); require(Clock::now() - start < 400ms, "bounded exit");
  require(sent.size() == 2 && sent.back().x == 0, "stop priority discards pending target");
  Worker throwing(config(), [](Velocity, int, Time) -> bool {throw std::runtime_error("network");});
  {auto t = Clock::now(); throwing.guard.require("odom", 1);
    throwing.guard.input("odom", 100, 100, true, t); throwing.guard.command({.1, 0, 0}, true, t);}
  throwing.start(); throwing.shutdown();
}
void protocol()
{
  int fds[2]; require(socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0, "socketpair");
  auto transport = std::make_unique<Transport>();
  TransportTestAccess::controlSocket(*transport) = fds[0];
  robot_move::seer::Client client(std::move(transport));
  auto server = std::async(std::launch::async, [&] {
        auto request = readRequest(fds[1]); std::string body(request.begin() + 16, request.end());
        require(body.find("\"duration\":800") != std::string::npos,
        "wire duration in milliseconds");
        require(request[8] == 7 && request[9] == 0xda, "2010 velocity API");
        auto reply = response(request, "{\"ret_code\":0}");
        send(fds[1], reply.data(), reply.size(), MSG_NOSIGNAL); close(fds[1]);
  });
  require(client.setVelocity(.1, 0, 0, 800, Clock::now() + 150ms), "finite request ACK");
  server.get();
}
}
int main()
{
  try {
    core(); commanded_zero(); worker(); protocol();
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 1;
  }
  return 0;
}
