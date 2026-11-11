cmake_minimum_required(VERSION 3.8)
project(lidar)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(robot_move REQUIRED)
find_package(Threads REQUIRED)

add_library(lidar_impl
  src/impl/seer/seer_lidar_source.cpp
  src/processing/laser_scan_builder.cpp
  src/nodes/lidar_node.cpp)
target_compile_features(lidar_impl PUBLIC cxx_std_17)
set_target_properties(lidar_impl PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(lidar_impl PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(lidar_impl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lidar_impl rclcpp rcl_interfaces sensor_msgs std_msgs diagnostic_msgs)
target_link_libraries(lidar_impl robot_move::robot_move_impl Threads::Threads)
add_executable(lidar_node src/lidar_main.cpp)
target_link_libraries(lidar_node lidar_impl)
install(TARGETS lidar_impl EXPORT lidarTargets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS lidar_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_targets(lidarTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rcl_interfaces sensor_msgs std_msgs diagnostic_msgs robot_move Threads)

if(BUILD_TESTING)
  enable_testing()
  foreach(test_name seer_lidar_source_test laser_scan_builder_test lidar_node_test lidar_integration_test)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} lidar_impl)
    target_compile_definitions(${test_name} PRIVATE FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 30 ENVIRONMENT "ROS_DOMAIN_ID=217")
  endforeach()
endif()
ament_package()
