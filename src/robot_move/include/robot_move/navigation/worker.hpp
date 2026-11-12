#pragma once
#include "robot_move/navigation/guard.hpp"
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
namespace robot_move::navigation
{
// Exclusive owner of the navigation control channel. No task/pose queries share it.
class Worker
{
public:
  using Send = std::function<bool(Velocity, int, Time)>;
  Worker(Config config, Send send)
  : guard(config), send_(std::move(send)) {}
  ~Worker() {shutdown();}
  Guard guard;
  std::mutex mutex;
  void start() {thread_ = std::thread([this] {run();});}
  void wake() {wake_.notify_all();}
  void shutdown()
  {
    {std::lock_guard<std::mutex> lock(mutex); closing_ = true;}
    wake();
    if (thread_.joinable()) {thread_.join();}
  }

private:
  Send send_;
  std::condition_variable wake_;
  std::thread thread_;
  bool closing_ = false;
  void run()
  {
    std::unique_lock<std::mutex> lock(mutex);
    bool shutdown_requested = false;
    while (true) {
      if (closing_ && !shutdown_requested) {
        // Do not cancel somebody else's task if this driver never sent motion.
        if (guard.last_command) {guard.stop("node_shutdown", false, Clock::now());}
        shutdown_requested = true;
      }
      auto command = guard.next(Clock::now());
      if (!command) {
        if (closing_) {break;}
        wake_.wait_for(lock, std::chrono::milliseconds(10)); continue;
      }
      const auto config = guard.config;
      lock.unlock();
      bool ok = false;
      try {
        ok = send_(*command, config.duration_ms,
          Clock::now() + std::chrono::milliseconds(config.request_ms));
      } catch (...) {ok = false;}
      lock.lock();
      guard.result(*command, ok, Clock::now());
    }
  }
};
}
