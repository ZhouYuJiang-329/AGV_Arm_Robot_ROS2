#include "robot_move/odometry/odometry_node.hpp"
#include <iostream>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<robot_move::odometry::OdometryNode>());
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; result = 1;
  }
  rclcpp::shutdown();
  return result;
}
