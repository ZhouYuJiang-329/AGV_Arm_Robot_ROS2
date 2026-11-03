#include "agv_hardware/config.hpp"
#include <iostream>
#include <stdexcept>
using namespace agv_hardware;
void check(bool value, const char* message)
{
    if (!value)
        throw std::runtime_error(message);
}
int main()
{
    try
    {
        AgvConfig config;
        std::string error;
        check(loadChassisConfig({}, config, error), "defaults rejected");
        check(config.chassis_ip == "192.168.1.109" && config.agv_init_set_do &&
                  config.agv_init_relocate,
              "defaults changed");
        check(loadChassisConfig({{"chassis_ip", "127.0.0.1"},
                                 {"agv_max_linear_speed", " 0.5 "},
                                 {"agv_max_angular_speed", "1e-1"},
                                 {"agv_init_set_do", "False"},
                                 {"agv_init_relocate", "false"},
                                 {"agv_init_do_id", "2147483647"},
                                 {"agv_init_do_status", "false"}},
                                config, error),
              "valid override rejected");
        check(config.chassis_ip == "127.0.0.1" && config.agv_max_linear_speed == .5 &&
                  config.agv_max_angular_speed == .1 && !config.agv_init_set_do &&
                  !config.agv_init_relocate && !config.agv_init_do_status &&
                  config.agv_init_do_id == 2147483647,
              "override ignored");
        for (const auto* key : {"agv_max_linear_speed", "agv_max_angular_speed"})
            for (const auto* value : {"-1", "nan", "inf", "1e9999", "0.2m", "", "true"})
            {
                check(
                    !loadChassisConfig({{key, value}, {"chassis_ip", "127.0.0.2"}}, config, error),
                    "bad speed accepted");
                check(config.chassis_ip == "127.0.0.1" && config.agv_max_linear_speed == .5 &&
                          !error.empty(),
                      "partial update on failure");
            }
        for (const auto* key : {"agv_init_set_do", "agv_init_do_status", "agv_init_relocate"})
            for (const auto* value : {"1", "0", "", "yes", "falsex"})
                check(!loadChassisConfig({{key, value}}, config, error), "bad bool accepted");
        for (const auto* value : {"-1", "1.0", "1x", "", "2147483648", "9999999999999999999999"})
            check(!loadChassisConfig({{"agv_init_do_id", value}}, config, error),
                  "bad id accepted");
        for (const auto* value : {"", "host", "127.0.0.1:19205", "999.0.0.1"})
            check(!loadChassisConfig({{"chassis_ip", value}}, config, error), "bad ip accepted");
        check(loadChassisConfig({{"unrelated", "ignored"}}, config, error) && error.empty(),
              "unknown parameters rejected");
        std::cout << "PASS: chassis configuration\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
