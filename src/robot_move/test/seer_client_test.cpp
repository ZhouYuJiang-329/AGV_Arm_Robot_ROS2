#include "seer_test_support.hpp"
#include "robot_move/impl/agv/seer/seer_client.hpp"
#include "robot_move/interface/cJSON.h"
using namespace robot_move::seer;

void withClient(ApiChannel channel, const std::function<void(int)>& server,
                const std::function<void(Client&)>& action)
{
    withServer(server,
               [&](int& fd)
               {
                   auto transport = std::make_unique<Transport>();
                   switch (channel)
                   {
                   case ApiChannel::Status:
                       TransportTestAccess::statusSocket(*transport) = fd;
                       break;
                   case ApiChannel::Control:
                       TransportTestAccess::controlSocket(*transport) = fd;
                       break;
                   case ApiChannel::Navigation:
                       TransportTestAccess::navigationSocket(*transport) = fd;
                       break;
                   case ApiChannel::Other:
                       TransportTestAccess::doSocket(*transport) = fd;
                       break;
                   }
                   fd = -1; // Ownership transferred; Client destruction closes the connection.
                   Client client(std::move(transport));
                   action(client);
               });
}
void mapping(ApiChannel channel, int api, const std::string& expected_body,
             const std::string& reply, const std::function<bool(Client&)>& operation)
{
    withClient(
        channel,
        [&](int fd)
        {
            const auto req = readRequest(fd);
            require(((int(req[8]) << 8) | req[9]) == api, "API number mismatch");
            const std::string body(req.begin() + 16, req.end());
            if (expected_body.empty())
                require(body.empty(), "request must have no body");
            else
            {
                cJSON* actual = cJSON_Parse(body.c_str());
                cJSON* expected = cJSON_Parse(expected_body.c_str());
                const bool same = cJSON_Compare(actual, expected, true);
                cJSON_Delete(actual);
                cJSON_Delete(expected);
                require(same, "API request fields mismatch");
            }
            const auto bytes = response(req, reply);
            for (uint8_t byte : bytes)
                writeAll(fd, &byte, 1);
        },
        [&](Client& client) { require(operation(client), "typed API operation failed"); });
}
int main()
{
    try
    {
        bool rejected = false;
        try
        {
            Client invalid(nullptr);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "null transport accepted");
        mapping(ApiChannel::Status, 1004, "", R"({"x":1.2,"y":0.5,"angle":1.57})",
                [](Client& c)
                {
                    Pose pose;
                    return c.queryPose(pose) && pose.x == 1.2 && pose.y == 0.5 &&
                           pose.angle == 1.57;
                });
        mapping(ApiChannel::Control, 2010, R"({"vx":-0.3,"vy":0.4,"w":-0.2,"duration":0})", "{}",
                [](Client& c) { return c.setVelocity(-0.3, 0.4, -0.2, 0); });
        mapping(ApiChannel::Navigation, 3055, R"({"dist":1,"vx":-0.1,"vy":0})", "",
                [](Client& c) { return c.translate(1, -0.1, 0); });
        mapping(ApiChannel::Navigation, 3056, R"({"angle":1.57,"vw":-0.2})", "{}",
                [](Client& c) { return c.turn(1.57, -0.2); });
        mapping(ApiChannel::Control, 2002, R"({"x":1,"y":2,"angle":0.5,"length":2})", "{}",
                [](Client& c) { return c.requestRelocation(Pose{1, 2, 0.5}, 2); });
        mapping(ApiChannel::Control, 2003, "", "", [](Client& c) { return c.confirmRelocation(); });
        mapping(ApiChannel::Other, 6001, R"({"id":9,"status":false})", "{}",
                [](Client& c) { return c.setDigitalOutput(9, false); });
        for (int value = 0; value <= 6; ++value)
            mapping(ApiChannel::Status, 1020, R"({"simple":true})",
                    "{\"task_status\":" + std::to_string(value) + "}",
                    [value](Client& c)
                    {
                        NavigationStatus s;
                        return c.queryNavigationStatus(s) && int(s) == value;
                    });
        for (int value = 0; value <= 3; ++value)
            mapping(ApiChannel::Status, 1021, "",
                    "{\"reloc_status\":" + std::to_string(value) + "}",
                    [value](Client& c)
                    {
                        RelocationStatus s;
                        return c.queryRelocationStatus(s) && int(s) == value;
                    });

        // API errors retain the stream; zero, absent and empty acknowledgment succeed.
        withClient(
            ApiChannel::Control,
            [](int fd)
            {
                for (const auto& body :
                     {R"({"ret_code":42})", R"({"ret_code":"0"})", R"({"ret_code":0})", "{}", ""})
                {
                    const auto bytes = response(readRequest(fd), body);
                    writeAll(fd, bytes.data(), bytes.size());
                }
            },
            [](Client& c)
            {
                for (int i = 0; i < 5; ++i)
                    require(c.confirmRelocation() == (i >= 2),
                            "ret_code/empty ack handling changed");
            });
        for (const std::string& bad : {std::string("[}"), std::string("[]"),
                                       std::string("{}trailing"), std::string("{\0}", 3)})
            withClient(
                ApiChannel::Control,
                [&](int fd)
                {
                    const auto bytes = response(readRequest(fd), bad);
                    writeAll(fd, bytes.data(), bytes.size());
                    uint8_t byte;
                    require(recv(fd, &byte, 1, 0) == 0, "malformed JSON did not close channel");
                },
                [](Client& c) { require(!c.confirmRelocation(), "malformed JSON accepted"); });
        for (const auto& bad :
             {"{}", "", R"({"x":1,"y":2,"angle":"bad"})", R"({"x":1,"y":2,"angle":1e999})"})
            withClient(
                ApiChannel::Status,
                [&](int fd)
                {
                    for (const auto& body :
                         {std::string(bad), std::string(R"({"x":3,"y":4,"angle":5})")})
                    {
                        const auto bytes = response(readRequest(fd), body);
                        writeAll(fd, bytes.data(), bytes.size());
                    }
                },
                [](Client& c)
                {
                    Pose p{7, 8, 9};
                    require(!c.queryPose(p) && p.x == 7 && p.y == 8 && p.angle == 9,
                            "failed pose changed output");
                    require(c.queryPose(p) && p.x == 3, "field error unexpectedly closed channel");
                });
        for (bool relocation : {false, true})
            for (const auto& value : {"null", "true", "\"1\"", "-1", "1.5", "7"})
                withClient(
                    ApiChannel::Status,
                    [&](int fd)
                    {
                        const std::string key = relocation ? "reloc_status" : "task_status";
                        const auto bytes =
                            response(readRequest(fd), "{\"" + key + "\":" + value + "}");
                        writeAll(fd, bytes.data(), bytes.size());
                    },
                    [&](Client& c)
                    {
                        NavigationStatus nav = NavigationStatus::Canceled;
                        RelocationStatus reloc = RelocationStatus::Completed;
                        require(!(relocation ? c.queryRelocationStatus(reloc)
                                             : c.queryNavigationStatus(nav)),
                                "invalid status accepted");
                        require(nav == NavigationStatus::Canceled &&
                                    reloc == RelocationStatus::Completed,
                                "failed status changed output");
                    });
        for (bool relocation : {false, true})
            for (const auto& body : {"{}", ""})
                withClient(
                    ApiChannel::Status,
                    [&](int fd)
                    {
                        const auto bytes = response(readRequest(fd), body);
                        writeAll(fd, bytes.data(), bytes.size());
                    },
                    [&](Client& c)
                    {
                        NavigationStatus nav = NavigationStatus::Canceled;
                        RelocationStatus reloc = RelocationStatus::Completed;
                        require(!(relocation ? c.queryRelocationStatus(reloc)
                                             : c.queryNavigationStatus(nav)),
                                "missing query field accepted");
                        require(nav == NavigationStatus::Canceled &&
                                    reloc == RelocationStatus::Completed,
                                "missing field changed output");
                    });
        // A caller's remaining time includes the client's serialization lock.
        std::promise<void> started;
        withClient(
            ApiChannel::Control,
            [&](int fd)
            {
                const auto req = readRequest(fd);
                started.set_value();
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
                const auto bytes = response(req, "{}");
                writeAll(fd, bytes.data(), bytes.size());
            },
            [&](Client& c)
            {
                bool first = false;
                std::thread worker([&] { first = c.confirmRelocation(); });
                started.get_future().wait();
                const auto start = std::chrono::steady_clock::now();
                const bool second = c.confirmRelocation(start + std::chrono::milliseconds(80));
                const auto elapsed = std::chrono::steady_clock::now() - start;
                worker.join();
                require(first && !second && elapsed < std::chrono::milliseconds(250),
                        "client lock exceeded deadline");
            });
        withClient(
            ApiChannel::Status,
            [](int fd)
            {
                const auto req = readRequest(fd);
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
                const auto bytes = response(req, R"({"task_status":4})");
                writeAll(fd, bytes.data(), bytes.size());
            },
            [](Client& c)
            {
                NavigationStatus status = NavigationStatus::Running;
                const auto start = std::chrono::steady_clock::now();
                require(!c.queryNavigationStatus(status, start + std::chrono::milliseconds(80)),
                        "late completion accepted");
                require(status == NavigationStatus::Running &&
                            std::chrono::steady_clock::now() - start <
                                std::chrono::milliseconds(250),
                        "network deadline not propagated");
            });
        std::cout << "PASS: typed Seer API mapping and response semantics\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
