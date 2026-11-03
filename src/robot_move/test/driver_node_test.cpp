#include "robot_move/nodes/universal_robot_driver.hpp"
#include <condition_variable>
#include <chrono>
#include <iostream>
#include <stdexcept>

struct State
{
    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false;
    bool canceled = false;
    bool destroyed = false;
    int rotations = 0;
    int velocity_commands = 0;
    int shutdowns = 0;
};
class FakeDriver : public RobotMove
{
public:
    explicit FakeDriver(std::shared_ptr<State> state) : state_(std::move(state)) {}
    ~FakeDriver() override { state_->destroyed = true; }
    bool init(const std::string&) override { throw std::runtime_error("injected driver must not be reinitialized"); }
    void setVelocity(double, double, double) override { ++state_->velocity_commands; }
    void send_loop() override {}
    void moveshutdown() override { ++state_->shutdowns; }
    void cancelPendingOperations() override
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->canceled = true;
        state_->condition.notify_all();
    }
    bool movebydistance(double, double, double) override
    {
        std::unique_lock<std::mutex> lock(state_->mutex);
        state_->entered = true;
        state_->condition.notify_all();
        state_->condition.wait(lock, [&] { return state_->canceled; });
        return false;
    }
    bool rotatebyangle(double, double) override { ++state_->rotations; return true; }
private:
    std::shared_ptr<State> state_;
};
struct UniversalRobotDriverTestAccess
{
    static void move(UniversalRobotDriver& node)
    {
        auto message = std::make_shared<geometry_msgs::msg::Twist>();
        message->linear.z = 1;
        message->linear.x = 0.1;
        node.stepMoveCallback(message);
    }
    static void whileBusy(UniversalRobotDriver& node)
    {
        node.stepRotateCallback(std::make_shared<geometry_msgs::msg::Twist>());
        node.cmdVelCallback(std::make_shared<geometry_msgs::msg::Twist>());
        node.timerCallback();
    }
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto state = std::make_shared<State>();
        auto node = std::make_unique<UniversalRobotDriver>("driver_lifecycle_test", std::make_unique<FakeDriver>(state));
        UniversalRobotDriverTestAccess::move(*node);
        bool entered;
        {
            std::unique_lock<std::mutex> lock(state->mutex);
            entered = state->condition.wait_for(lock, std::chrono::seconds(2), [&] { return state->entered; });
        }
        if (!entered) throw std::runtime_error("worker did not start");
        const auto start = std::chrono::steady_clock::now();
        UniversalRobotDriverTestAccess::whileBusy(*node);
        if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(500))
            throw std::runtime_error("callbacks blocked behind a long task");
        node.reset();
        if (!state->canceled || !state->destroyed || state->rotations != 0 ||
            state->velocity_commands != 1 || state->shutdowns != 1)
            throw std::runtime_error("worker cancellation/join/cleanup semantics incorrect");
        std::cout << "PASS: owned worker, busy rejection, nonblocking callbacks and destruction\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}
