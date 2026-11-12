#include "robot_move/impl/agv/agv_robot_move.hpp"
#include <algorithm>
#include <vector>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
using namespace robot_move::seer;
struct AGVRobotMoveTestAccess
{
    static bool navigation(AGVRobotMove& robot, std::chrono::milliseconds timeout)
    {
        return robot.waitForNavigationComplete(timeout);
    }
    static bool relocation(AGVRobotMove& robot, std::chrono::milliseconds timeout)
    {
        return robot.waitForRelocationComplete(timeout);
    }
};
void check(bool value, const char* message)
{
    if (!value)
        throw std::runtime_error(message);
}
struct State
{
    std::vector<std::string> calls;
    bool connect_ok = true, command_ok = true, pose_ok = true, status_ok = true, confirm_ok = true,
         do_ok = true;
    Pose pose{1, 2, 0.5};
    std::deque<NavigationStatus> navigation;
    std::deque<RelocationStatus> relocation;
    NavigationStatus last_nav = NavigationStatus::Completed;
    RelocationStatus last_reloc = RelocationStatus::Success;
    std::vector<double> args;
    int duration = -1, do_id = -1;
    bool do_status = false;
    Deadline last_deadline = Deadline::max();
    std::function<void(Deadline)> delay;
    std::atomic<bool> queried{false};
    bool destroyed = false;
};
class FakeClient : public IClient
{
  public:
    explicit FakeClient(std::shared_ptr<State> state) : s(std::move(state)) {}
    ~FakeClient() override
    {
        s->destroyed = true;
    }
    bool connect(const std::string&) override
    {
        s->calls.push_back("connect");
        return s->connect_ok;
    }
    void disconnect() override
    {
        s->calls.push_back("disconnect");
    }
    bool queryPose(Pose& pose, Deadline) override
    {
        s->calls.push_back("pose");
        if (!s->pose_ok)
            return false;
        pose = s->pose;
        return true;
    }
    bool setVelocity(double x, double y, double w, int duration, Deadline) override
    {
        s->calls.push_back("velocity");
        s->args = {x, y, w};
        s->duration = duration;
        return s->command_ok;
    }
    bool translate(double dist, double x, double y, Deadline) override
    {
        s->calls.push_back("translate");
        s->args = {dist, x, y};
        return s->command_ok;
    }
    bool turn(double angle, double w, Deadline) override
    {
        s->calls.push_back("turn");
        s->args = {angle, w};
        return s->command_ok;
    }
    bool queryNavigationStatus(NavigationStatus& status, Deadline deadline) override
    {
        s->calls.push_back("navigation");
        s->last_deadline = deadline;
        s->queried = true;
        if (s->delay)
            s->delay(deadline);
        if (!s->status_ok)
            return false;
        if (!s->navigation.empty())
        {
            s->last_nav = s->navigation.front();
            s->navigation.pop_front();
        }
        status = s->last_nav;
        return true;
    }
    bool requestRelocation(const Pose& pose, double radius, Deadline) override
    {
        s->calls.push_back("relocate");
        s->args = {pose.x, pose.y, pose.angle, radius};
        return s->command_ok;
    }
    bool queryRelocationStatus(RelocationStatus& status, Deadline deadline) override
    {
        s->calls.push_back("relocation");
        s->last_deadline = deadline;
        s->queried = true;
        if (s->delay)
            s->delay(deadline);
        if (!s->status_ok)
            return false;
        if (!s->relocation.empty())
        {
            s->last_reloc = s->relocation.front();
            s->relocation.pop_front();
        }
        status = s->last_reloc;
        return true;
    }
    bool confirmRelocation(Deadline deadline) override
    {
        s->calls.push_back("confirm");
        s->last_deadline = deadline;
        return s->confirm_ok;
    }
    bool setDigitalOutput(int id, bool status, Deadline) override
    {
        s->calls.push_back("do");
        s->do_id = id;
        s->do_status = status;
        return s->do_ok;
    }

  private:
    std::shared_ptr<State> s;
};
struct Scenario
{
    std::shared_ptr<State> state = std::make_shared<State>();
    AGVRobotMove robot{std::make_unique<FakeClient>(state)};
};
void rejected(const std::function<void(AGVRobotMove&)>& action, double linear = 0,
              double angular = 0)
{
    Scenario test;
    check(test.robot.setSpeedLimits(linear, angular), "test limits invalid");
    action(test.robot);
    check(test.state->calls.empty(), "invalid input reached client");
}
int main()
{
    try
    {
        {
            auto state = std::make_shared<State>();
            {
                std::unique_ptr<RobotMove> owner =
                    std::make_unique<AGVRobotMove>(std::make_unique<FakeClient>(state));
            }
            check(state->destroyed && state->calls == std::vector<std::string>{"disconnect"},
                  "injected client was not disconnected and destroyed");
        }
        bool invalid = false;
        try
        {
            AGVRobotMove robot(nullptr);
        }
        catch (const std::invalid_argument&)
        {
            invalid = true;
        }
        check(invalid, "null client accepted");
        for (bool set_do : {false, true})
            for (bool relocate : {false, true})
            {
                Scenario t;
                AGVRobotMove::InitOptions options;
                options.set_do = set_do;
                options.relocate = relocate;
                options.do_id = 9;
                options.do_status = false;
                check(t.robot.setInitOptions(options), "valid init options rejected");
                check(t.robot.init("test"), "init failed");
                std::vector<std::string> expected{"disconnect", "connect"};
                if (set_do)
                    expected.push_back("do");
                if (relocate)
                    expected.insert(expected.end(), {"pose", "relocate", "relocation"});
                check(t.state->calls == expected, "init action order differs");
                if (set_do)
                    check(t.state->do_id == 9 && !t.state->do_status, "DO config differs");
            }
        {
            Scenario t;
            check(t.robot.init("test"), "default init failed");
            check(t.state->calls == std::vector<std::string>{"disconnect", "connect", "do", "pose",
                                                             "relocate", "relocation"},
                  "defaults differ");
            check(t.state->do_id == 0 && t.state->do_status, "default DO differs");
            check(t.state->args == std::vector<double>{1, 2, 0.5, 2},
                  "relocation target/radius differs");
        }
        {
            Scenario t;
            AGVRobotMove::InitOptions valid;
            valid.set_do = false;
            valid.relocate = false;
            check(t.robot.setInitOptions(valid), "valid init options rejected");
            auto invalid = valid;
            invalid.do_id = -1;
            invalid.set_do = true;
            check(!t.robot.setInitOptions(invalid), "negative id accepted");
            check(t.robot.init("test") && t.state->calls.size() == 2,
                  "invalid update changed configuration");
        }
        for (int failure = 0; failure < 5; ++failure)
        {
            Scenario t;
            if (failure == 0)
                t.state->connect_ok = false;
            if (failure == 1)
                t.state->do_ok = false;
            if (failure == 2)
                t.state->pose_ok = false;
            if (failure == 3)
                t.state->command_ok = false;
            if (failure == 4)
                t.state->status_ok = false;
            check(!t.robot.init("test") && t.state->calls.back() == "disconnect",
                  "failed init did not clean up");
            if (failure == 0)
                check(t.state->calls.size() == 3, "actions ran after failed connection");
            if (failure == 1)
                check(t.state->calls.size() == 4, "relocation ran after failed DO");
        }
        for (double bad :
             {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity()})
        {
            rejected([&](AGVRobotMove& r) { r.setVelocity(bad, 0, 0); });
            rejected([&](AGVRobotMove& r) { r.setVelocity(0, bad, 0); });
            rejected([&](AGVRobotMove& r) { r.setVelocity(0, 0, bad); });
            rejected([&](AGVRobotMove& r)
                     { check(!r.movebydistance(bad, .1, 0), "bad distance accepted"); });
            rejected([&](AGVRobotMove& r)
                     { check(!r.movebydistance(1, bad, 0), "bad vx accepted"); });
            rejected([&](AGVRobotMove& r)
                     { check(!r.movebydistance(1, 0, bad), "bad vy accepted"); });
            rejected([&](AGVRobotMove& r)
                     { check(!r.rotatebyangle(bad, .1), "bad angle accepted"); });
            rejected([&](AGVRobotMove& r) { check(!r.rotatebyangle(1, bad), "bad vw accepted"); });
            Scenario t;
            check(!t.robot.setSpeedLimits(bad, 0) && !t.robot.setSpeedLimits(0, bad),
                  "bad limit accepted");
        }
        for (double bad : {0., -0., -1.})
        {
            rejected([&](AGVRobotMove& r)
                     { check(!r.movebydistance(bad, .1, 0), "nonpositive distance accepted"); });
            rejected([&](AGVRobotMove& r)
                     { check(!r.rotatebyangle(bad, .1), "nonpositive angle accepted"); });
        }
        rejected([](AGVRobotMove& r)
                 { check(!r.movebydistance(1, 0, -0.), "zero translation accepted"); });
        rejected([](AGVRobotMove& r)
                 { check(!r.rotatebyangle(1, -0.), "zero rotation accepted"); });
        rejected([](AGVRobotMove& r)
                 { check(!r.setDigitalOutput(-1, true), "negative DO accepted"); });
        rejected(
            [](AGVRobotMove& r)
            {
                r.setVelocity(std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max(), 0);
            });
        rejected([](AGVRobotMove& r) { r.setVelocity(.4, .4, 0); }, .5, .2);
        rejected([](AGVRobotMove& r) { r.setVelocity(-.6, 0, 0); }, .5, .2);
        rejected([](AGVRobotMove& r) { r.setVelocity(0, 0, -.3); }, .5, .2);
        rejected([](AGVRobotMove& r)
                 { check(!r.movebydistance(1, .4, .4), "combined cap ignored"); }, .5, .2);
        rejected([](AGVRobotMove& r) { check(!r.rotatebyangle(1, -.3), "angular cap ignored"); },
                 .5, .2);
        rejected(
            [](AGVRobotMove& r)
            {
                check(!r.setSpeedLimits(-1, 0) && !r.setSpeedLimits(0, -1), "bad config accepted");
                r.setVelocity(.6, 0, 0);
            },
            .5, .2);
        {
            Scenario t;
            check(t.robot.setSpeedLimits(.5, .2), "limits rejected");
            t.robot.setVelocity(0, 0, 0);
            t.robot.setVelocity(-.3, -.4, -.2);
            check(t.state->calls == std::vector<std::string>{"velocity", "velocity"} &&
                      t.state->duration == 800,
                  "valid velocity not sent");
            check(t.state->args == std::vector<double>{-.3, -.4, -.2}, "direction changed");
        }
        for (int direction : {-1, 1})
            for (bool turn : {false, true})
            {
                Scenario t;
                check(t.robot.setSpeedLimits(.1, .1), "limits rejected");
                check(turn ? t.robot.rotatebyangle(1, direction * .1)
                           : t.robot.movebydistance(1, direction * .1, 0),
                      "movement failed");
                check(t.state->calls ==
                          std::vector<std::string>{turn ? "turn" : "translate", "navigation"},
                      "movement did not wait");
                check(t.state->args[1] == direction * .1, "movement sign changed");
            }
        for (bool turn : {false, true})
        {
            Scenario t;
            t.state->command_ok = false;
            check(!(turn ? t.robot.rotatebyangle(1, .1) : t.robot.movebydistance(1, .1, 0)),
                  "rejected motion succeeded");
            check(t.state->calls.size() == 1, "rejected motion started polling");
        }
        {
            Scenario t;
            t.state->navigation = {NavigationStatus::None, NavigationStatus::Waiting,
                                   NavigationStatus::Running, NavigationStatus::Suspended,
                                   NavigationStatus::Completed};
            check(AGVRobotMoveTestAccess::navigation(t.robot, std::chrono::seconds(1)),
                  "pending state was not waited");
            check(t.state->calls.size() == 5, "NONE incorrectly reported success");
        }
        for (auto terminal : {NavigationStatus::Failed, NavigationStatus::Canceled})
        {
            Scenario t;
            t.state->last_nav = terminal;
            check(!AGVRobotMoveTestAccess::navigation(t.robot, std::chrono::seconds(1)) &&
                      t.state->calls.size() == 1,
                  "terminal state ignored");
        }
        {
            Scenario t;
            t.state->relocation = {RelocationStatus::Init, RelocationStatus::Relocating,
                                   RelocationStatus::Completed, RelocationStatus::Completed,
                                   RelocationStatus::Success};
            check(t.robot.relocate(), "legacy relocation failed");
            check(std::count(t.state->calls.begin(), t.state->calls.end(), "confirm") == 1,
                  "confirmation repeated");
        }
        {
            Scenario t;
            t.state->last_reloc = RelocationStatus::Completed;
            t.state->confirm_ok = false;
            check(!t.robot.relocate(), "failed confirmation accepted");
        }
        {
            Scenario t;
            double x = 7, y = 8, a = 9;
            t.state->pose_ok = false;
            check(!t.robot.getpose(x, y, a) && x == 7 && y == 8 && a == 9,
                  "failed query changed outputs");
        }
        {
            Scenario t;
            double x, y, a;
            check(t.robot.getpose(x, y, a) && x == 1 && y == 2 && a == .5, "pose mapping differs");
        }
        for (bool reloc : {false, true})
            for (int condition = 0; condition < 3; ++condition)
            {
                Scenario t;
                if (reloc)
                    t.state->last_reloc = condition == 0   ? RelocationStatus::Init
                                          : condition == 1 ? RelocationStatus::Relocating
                                                           : RelocationStatus::Completed;
                else
                    t.state->last_nav = condition == 0   ? NavigationStatus::None
                                        : condition == 1 ? NavigationStatus::Running
                                                         : NavigationStatus::Suspended;
                const auto start = std::chrono::steady_clock::now();
                check(!(reloc ? AGVRobotMoveTestAccess::relocation(t.robot,
                                                                   std::chrono::milliseconds(180))
                              : AGVRobotMoveTestAccess::navigation(t.robot,
                                                                   std::chrono::milliseconds(180))),
                      "nonterminal state succeeded");
                check(std::chrono::steady_clock::now() - start >= std::chrono::milliseconds(140),
                      "wait ended too early");
                check(t.state->last_deadline != Deadline::max(),
                      "task deadline not passed to client");
            }
        for (bool reloc : {false, true})
        {
            Scenario t;
            t.state->delay = [](Deadline d)
            { std::this_thread::sleep_until(d + std::chrono::milliseconds(5)); };
            check(
                !(reloc
                      ? AGVRobotMoveTestAccess::relocation(t.robot, std::chrono::milliseconds(60))
                      : AGVRobotMoveTestAccess::navigation(t.robot, std::chrono::milliseconds(60))),
                "late completion accepted");
        }
        for (bool reloc : {false, true})
        {
            Scenario t;
            t.state->last_nav = NavigationStatus::Running;
            t.state->last_reloc = RelocationStatus::Relocating;
            bool result = true;
            std::thread worker(
                [&]
                {
                    result =
                        reloc
                            ? AGVRobotMoveTestAccess::relocation(t.robot, std::chrono::seconds(2))
                            : AGVRobotMoveTestAccess::navigation(t.robot, std::chrono::seconds(2));
                });
            while (!t.state->queried.load())
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            t.robot.cancelPendingOperations();
            worker.join();
            check(!result, "local cancel ignored");
            t.state->last_nav = NavigationStatus::Completed;
            check(t.robot.connect("test") &&
                      AGVRobotMoveTestAccess::navigation(t.robot, std::chrono::seconds(1)),
                  "connect did not clear local cancel");
        }
        std::cout << "PASS: AGV policies through typed fake client\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
