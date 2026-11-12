#include "agv_hardware/agv_hardware.hpp"
#include <pluginlib/class_loader.hpp>
#include <hardware_interface/component_parser.hpp>
#include <fstream>
#include <sstream>
#include <condition_variable>
#include <future>
#include <iostream>
#include <stdexcept>
using namespace robot_move::seer;
using Return = hardware_interface::CallbackReturn;
namespace agv_hardware
{
struct HardwareTestAccess
{
    static void local(AgvHardwareInterface& h)
    {
        h.config_.server_uri = "ws://127.0.0.1:1";
        h.config_.local_server_port = 0;
        h.config_.reconnect_interval_ms = 10;
    }
    static void command(AgvHardwareInterface& h, const std::string& text)
    {
        h.handleCommand(text);
    }
    static bool accepting(AgvHardwareInterface& h)
    {
        return h.accept_commands_.load();
    }
};
} // namespace agv_hardware
using agv_hardware::HardwareTestAccess;
void check(bool value, const char* message)
{
    if (!value)
        throw std::runtime_error(message);
}
struct MockState
{
    bool connect_ok = true, do_ok = true;
    int connects = 0, disconnects = 0, do_count = 0, relocates = 0, velocities = 0;
    int do_id = -1;
    bool do_status = false;
    std::string ip;
    std::array<double, 3> speed{};
    std::mutex mutex;
    std::condition_variable cv;
    bool block = false, entered = false, release = false;
};
class FakeClient : public IClient
{
  public:
    explicit FakeClient(std::shared_ptr<MockState> state) : s(std::move(state)) {}
    bool connect(const std::string& ip) override
    {
        ++s->connects;
        s->ip = ip;
        return s->connect_ok;
    }
    void disconnect() override
    {
        ++s->disconnects;
    }
    bool queryPose(Pose& p, Deadline) override
    {
        p = {1, 2, .5};
        return true;
    }
    bool setVelocity(double x, double y, double w, int duration, Deadline) override
    {
        check(duration == 800, "finite duration missing");
        std::unique_lock<std::mutex> lock(s->mutex);
        ++s->velocities;
        s->speed = {x, y, w};
        s->entered = true;
        s->cv.notify_all();
        if (s->block)
            s->cv.wait(lock, [&] { return s->release; });
        return true;
    }
    bool translate(double, double, double, Deadline) override
    {
        return false;
    }
    bool turn(double, double, Deadline) override
    {
        return false;
    }
    bool queryNavigationStatus(NavigationStatus&, Deadline) override
    {
        return false;
    }
    bool requestRelocation(const Pose&, double, Deadline) override
    {
        ++s->relocates;
        return true;
    }
    bool queryRelocationStatus(RelocationStatus& status, Deadline) override
    {
        status = RelocationStatus::Success;
        return true;
    }
    bool confirmRelocation(Deadline) override
    {
        return true;
    }
    bool setDigitalOutput(int id, bool status, Deadline) override
    {
        ++s->do_count;
        s->do_id = id;
        s->do_status = status;
        return s->do_ok;
    }

  private:
    std::shared_ptr<MockState> s;
};
class TestHardware : public agv_hardware::AgvHardwareInterface
{
  public:
    std::shared_ptr<MockState> state = std::make_shared<MockState>();
    bool real = false, connect_ok = true, activate_ok = true, throw_activate = false;
    int arm_connects = 0, arm_activates = 0, arm_deactivates = 0, arm_disconnects = 0;

  protected:
    std::unique_ptr<AGVRobotMove> createChassis() override
    {
        if (real)
            return std::make_unique<AGVRobotMove>();
        return std::make_unique<AGVRobotMove>(std::make_unique<FakeClient>(state));
    }
    bool connectArm() override
    {
        ++arm_connects;
        return connect_ok;
    }
    bool activateArm() override
    {
        ++arm_activates;
        if (throw_activate)
            throw std::runtime_error("injected activation exception");
        return activate_ok;
    }
    void deactivateArm() override
    {
        ++arm_deactivates;
    }
    void disconnectArm() override
    {
        ++arm_disconnects;
    }
};
const std::string chassis =
    R"({"header":{},"payload":{"chassis":{"command":"velocity_move","parameters":{"vx":0.1,"vy":0.2,"wz":-0.3}}}})";
const std::string agv =
    R"({"header":{},"payload":{"agv":{"command":"move","data":{"vx":0.2,"vy":0,"wz":0.1}}}})";
hardware_interface::HardwareInfo info(bool actions = false)
{
    hardware_interface::HardwareInfo i;
    i.name = "test_agv";
    i.type = "system";
    i.hardware_parameters = {{"chassis_ip", "127.0.0.1"},
                             {"agv_max_linear_speed", "0.5"},
                             {"agv_max_angular_speed", "0.5"},
                             {"agv_init_set_do", actions ? "true" : "false"},
                             {"agv_init_relocate", actions ? "true" : "false"},
                             {"agv_init_do_id", "7"},
                             {"agv_init_do_status", "false"}};
    return i;
}
void initialize(TestHardware& h, bool actions = false)
{
    check(h.on_init(info(actions)) == Return::SUCCESS, "on_init failed");
    HardwareTestAccess::local(h);
}
void simulatedTests()
{
    rclcpp_lifecycle::State previous;
    {
        TestHardware h;
        auto invalid = info();
        invalid.hardware_parameters["agv_max_linear_speed"] = "nan";
        check(h.on_init(invalid) == Return::ERROR && h.state->connects == 0,
              "invalid config connected");
    }
    {
        TestHardware h;
        initialize(h, true);
        check(h.state->connects == 0 && h.state->do_count == 0, "on_init performed network/action");
        HardwareTestAccess::command(h, chassis);
        check(h.state->velocities == 0, "unconfigured command accepted");
        check(h.on_configure(previous) == Return::SUCCESS, "configure failed");
        check(h.state->connects == 1 && h.state->ip == "127.0.0.1" && h.state->do_count == 1 &&
                  h.state->do_id == 7 && !h.state->do_status && h.state->relocates == 1 &&
                  h.arm_connects == 1,
              "configuration not applied");
        HardwareTestAccess::command(h, chassis);
        check(h.state->velocities == 0, "inactive command accepted");
        check(h.on_activate(previous) == Return::SUCCESS, "activate failed");
        HardwareTestAccess::command(h, chassis);
        check(h.state->velocities == 1 && h.state->speed == std::array<double, 3>{.1, .2, -.3},
              "chassis not sent once");
        HardwareTestAccess::command(h, agv);
        check(h.state->velocities == 2, "agv alias not sent once");
        HardwareTestAccess::command(
            h, R"({"header":{},"payload":{"agv":{"command":"move","data":{"vx":2}}}})");
        HardwareTestAccess::command(
            h, R"({"header":{},"payload":{"agv":{"command":"move","data":{"vx":"bad"}}}})");
        HardwareTestAccess::command(
            h,
            R"({"header":{},"payload":{"chassis":{"command":"velocity_move","parameters":{"vx":null}}}})");
        HardwareTestAccess::command(h, "bad json");
        check(h.state->velocities == 2, "invalid command sent");
        const int before = h.state->disconnects;
        check(h.on_deactivate(previous) == Return::SUCCESS, "deactivate failed");
        HardwareTestAccess::command(h, chassis);
        check(h.state->velocities == 2 && h.state->disconnects == before,
              "deactivation disconnected or accepted commands");
        check(h.on_activate(previous) == Return::SUCCESS, "reactivation failed");
        HardwareTestAccess::command(h, chassis);
        check(h.state->velocities == 3 && h.state->connects == 1 && h.state->do_count == 1,
              "reactivation reran init");
        check(h.on_cleanup(previous) == Return::SUCCESS, "cleanup failed");
        check(!HardwareTestAccess::accepting(h) && h.state->disconnects > before,
              "cleanup leaked chassis");
        check(h.on_cleanup(previous) == Return::SUCCESS, "repeated cleanup failed");
        check(h.on_configure(previous) == Return::SUCCESS && h.state->connects == 2 &&
                  h.state->do_count == 2,
              "reconfigure failed");
        check(h.on_shutdown(previous) == Return::SUCCESS, "shutdown failed");
    }
    {
        TestHardware h;
        auto external = info(true);
        external.hardware_parameters["external_navigation"] = "true";
        check(h.on_init(external) == Return::SUCCESS, "external on_init");
        HardwareTestAccess::local(h);
        check(h.on_configure(previous) == Return::SUCCESS, "external configure");
        check(h.on_activate(previous) == Return::SUCCESS, "external activate");
        HardwareTestAccess::command(h, chassis);
        HardwareTestAccess::command(h, agv);
        check(h.state->connects == 0 && h.state->do_count == 0 && h.state->relocates == 0 &&
              h.state->velocities == 0, "navigation mode bypassed exclusive chassis owner");
        check(h.on_shutdown(previous) == Return::SUCCESS, "external shutdown");
    }
    for (int failure = 0; failure < 3; ++failure)
    {
        TestHardware h;
        initialize(h, true);
        if (failure == 0)
            h.state->connect_ok = false;
        if (failure == 1)
            h.state->do_ok = false;
        if (failure == 2)
            h.connect_ok = false;
        int before = h.state->disconnects;
        check(h.on_configure(previous) == Return::ERROR, "configure failure ignored");
        check(h.state->disconnects > before && !HardwareTestAccess::accepting(h),
              "configure failure leaked resources");
        check(failure == 2 || h.arm_connects == 0, "arm connected after chassis failure");
    }
    {
        TestHardware h;
        initialize(h);
        check(h.on_configure(previous) == Return::SUCCESS && h.state->do_count == 0 &&
                  h.state->relocates == 0,
              "disabled actions ran");
        h.activate_ok = false;
        check(h.on_activate(previous) == Return::ERROR && !HardwareTestAccess::accepting(h) &&
                  h.arm_deactivates == 1,
              "activation failure not rolled back");
        h.activate_ok = true;
        check(h.on_activate(previous) == Return::SUCCESS, "retry activation failed");
        h.state->block = true;
        auto command =
            std::async(std::launch::async, [&] { HardwareTestAccess::command(h, chassis); });
        {
            std::unique_lock<std::mutex> lock(h.state->mutex);
            check(h.state->cv.wait_for(lock, std::chrono::seconds(2),
                                       [&] { return h.state->entered; }),
                  "command never entered");
        }
        auto cleanup = std::async(std::launch::async, [&] { return h.on_cleanup(previous); });
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (HardwareTestAccess::accepting(h) && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        const bool disabled = !HardwareTestAccess::accepting(h);
        const bool waited =
            cleanup.wait_for(std::chrono::milliseconds(30)) == std::future_status::timeout;
        {
            std::lock_guard<std::mutex> lock(h.state->mutex);
            h.state->release = true;
            h.state->cv.notify_all();
        }
        command.get();
        check(cleanup.get() == Return::SUCCESS && disabled && waited,
              "cleanup did not drain in-flight request");
    }
    {
        TestHardware h;
        initialize(h);
        check(h.on_configure(previous) == Return::SUCCESS, "exception setup failed");
        h.throw_activate = true;
        check(h.on_activate(previous) == Return::ERROR && h.arm_deactivates == 1 &&
                  !HardwareTestAccess::accepting(h),
              "activation exception not rolled back");
    }
    // 未显式cleanup的活跃对象也必须由析构回收连接和回调线程。
    std::shared_ptr<MockState> retained;
    int disconnects_before_destruction = 0;
    {
        TestHardware h;
        initialize(h);
        retained = h.state;
        check(h.on_configure(previous) == Return::SUCCESS &&
                  h.on_activate(previous) == Return::SUCCESS,
              "destructor setup failed");
        disconnects_before_destruction = retained->disconnects;
    }
    check(retained->disconnects > disconnects_before_destruction, "destructor leaked chassis");
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface", "hardware_interface::SystemInterface");
    auto plugin = loader.createSharedInstance("agv_hardware/AgvHardwareInterface");
    check(bool(plugin), "plugin not loadable");
}
void modelTest(const std::string& path)
{
    std::ifstream input(path);
    std::stringstream xml;
    xml << input.rdbuf();
    const auto hardware = hardware_interface::parse_control_resources_from_urdf(xml.str());
    check(hardware.size() == 1 &&
              hardware[0].hardware_plugin_name == "agv_hardware/AgvHardwareInterface",
          "wrong hardware model");
    agv_hardware::AgvConfig config;
    std::string error;
    check(agv_hardware::loadChassisConfig(hardware[0].hardware_parameters, config, error),
          "model parameters rejected");
    check(config.chassis_ip == "127.0.0.1" && config.agv_max_linear_speed == .45 &&
              config.agv_max_angular_speed == .6 && !config.agv_init_set_do &&
              config.agv_init_do_id == 3 && !config.agv_init_do_status && !config.agv_init_relocate,
          "model parameters not propagated");
}
void realTest()
{
    TestHardware h;
    h.real = true;
    initialize(h);
    rclcpp_lifecycle::State previous;
    check(h.on_configure(previous) == Return::SUCCESS, "loopback configure failed");
    check(h.on_activate(previous) == Return::SUCCESS, "loopback activate failed");
    HardwareTestAccess::command(h, chassis);
    HardwareTestAccess::command(h, agv);     // Mock returns an API error on this request.
    HardwareTestAccess::command(h, chassis); // A complete API error must preserve the stream.
    check(h.on_deactivate(previous) == Return::SUCCESS, "loopback deactivate failed");
    HardwareTestAccess::command(h, chassis);
    check(h.on_cleanup(previous) == Return::SUCCESS, "loopback cleanup failed");
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        if (argc > 2 && std::string(argv[1]) == "--model")
            modelTest(argv[2]);
        else if (argc > 1 && std::string(argv[1]) == "--real")
            realTest();
        else
            simulatedTests();
        std::cout << "PASS: chassis hardware integration\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
