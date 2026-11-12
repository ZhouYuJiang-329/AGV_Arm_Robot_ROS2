#include "robot_move/navigation/node.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<robot_move::navigation::Node>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("navigation_guard"), "%s", error.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
