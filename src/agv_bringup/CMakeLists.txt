cmake_minimum_required(VERSION 3.8)
project(agv_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
add_library(agv_obstacle_layer SHARED src/measured_ray_obstacle_layer.cpp)
target_compile_features(agv_obstacle_layer PUBLIC cxx_std_17)
target_include_directories(agv_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(agv_obstacle_layer nav2_costmap_2d pluginlib rclcpp sensor_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d agv_costmap_plugins.xml)
install(TARGETS agv_obstacle_layer LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_python_install_package(agv_bringup_sim)
install(PROGRAMS scripts/agv_simulation_adapter scripts/agv_simulation_ready scripts/agv_static_planning scripts/agv_navigation_execution_gate DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(costmap_clearing_test test/costmap_clearing_test.cpp)
  target_compile_features(costmap_clearing_test PRIVATE cxx_std_17)
  target_link_libraries(costmap_clearing_test agv_obstacle_layer)
  ament_target_dependencies(costmap_clearing_test nav2_costmap_2d sensor_msgs)
  add_test(NAME costmap_clearing_test COMMAND costmap_clearing_test)
  set_tests_properties(costmap_clearing_test PROPERTIES TIMEOUT 10 ENVIRONMENT "ROS_DOMAIN_ID=211")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME agv_simulation_assets_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_simulation_assets.py ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME agv_real_navigation_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_real_navigation.py)
  set_tests_properties(agv_real_navigation_test PROPERTIES TIMEOUT 60 ENVIRONMENT "ROS_DOMAIN_ID=214")
  set_tests_properties(agv_simulation_assets_test PROPERTIES TIMEOUT 15)
endif()

ament_package()
