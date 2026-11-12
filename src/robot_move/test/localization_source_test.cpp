#include "seer_test_support.hpp"
#include "robot_move/impl/agv/seer/seer_localization_source.hpp"
namespace robot_move::seer
{
struct LocalizationSourceTestAccess
{
  static void socket(SeerLocalizationSource & s, int fd)
  {
    TransportTestAccess::statusSocket(s.transport_) = fd;
  }
};
}
using namespace robot_move::seer;
int main()
{
  try {
    for (int mode = 0; mode < 4; ++mode) {
      withServer([&](int fd) {
          int apis[] = {1000, 1021, 1004, 1021, 1000};
          for (int i = 0; i < 5; ++i) {
            auto req = readRequest(fd);
            require(req.size() == 16 && ((int(req[8]) << 8) | req[9]) == apis[i],
            "not read-only bracket");
            std::string body;
            if (apis[i] == 1000) {
              body = (mode == 1 &&
              i == 4) ? "{\"current_map\":\"other\"}" : "{\"current_map\":\"floor\"}";
            } else if (apis[i] == 1021) {
              body = (mode == 2 && i == 3) ? "{\"reloc_status\":2}" : "{\"reloc_status\":1}";
            } else {body = "{\"x\":1,\"y\":2,\"angle\":0,\"confidence\":0.9}";}
            if (mode == 3 && i == 0) {
              std::this_thread::sleep_for(std::chrono::milliseconds(100)); return;
            }
            auto reply = response(req, body);
            writeAll(fd, reply.data(), 5); writeAll(fd, reply.data() + 5, reply.size() - 5);
          }
        }, [&](int & fd) {
          SeerLocalizationSource source("unused"); LocalizationSourceTestAccess::socket(source, fd);
          fd = -1;
          robot_move::localization::Sample s; s.pose.x = 99; bool received = false;
          auto start = std::chrono::steady_clock::now();
          bool ok = source.read(s, start + std::chrono::milliseconds(mode == 3 ? 40 : 1000), [&]{
            received = true;
                                                                                                                 });
          require(ok == (mode == 0), "mixed map/status or timeout accepted");
          require(received == (mode != 3), "pose receive callback incorrect");
          require(s.pose.x == (mode == 0 ? 1 : 99), "failure modified sample");
          if (mode == 3) {
            require(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(90),
            "deadline ignored");
          }
        });
    }
    std::cout << "localization source read-only bracket and deadline passed\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';return 1;
  }
}
