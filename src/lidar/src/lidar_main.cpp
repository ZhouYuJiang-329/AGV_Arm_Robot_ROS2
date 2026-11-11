#include <exception>
#include <iostream>
#include "lidar/nodes/lidar_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        rclcpp::spin(std::make_shared<lidar::LidarNode>());
    }
    catch (const std::exception& error)
    {
        std::cerr << "lidar_node: " << error.what() << std::endl;
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
