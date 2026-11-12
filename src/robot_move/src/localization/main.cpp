#include "robot_move/localization/node.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_move::localization::LocalizationNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("seer_localization"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
