cmake_minimum_required(VERSION 3.8)
project(robot_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(robot_move_impl
  src/cJSON.c
  src/factory/robot_move_factory.cpp
  src/impl/air/air_robot_move.cpp
  src/impl/duct/duct_robot_move.cpp
  src/impl/mag/mag_robot_move.cpp
  src/impl/agv/agv_robot_move.cpp
  src/impl/agv/seer/seer_client.cpp
  src/impl/agv/seer/seer_velocity_source.cpp
  src/impl/agv/seer/seer_localization_source.cpp
  src/odometry/integrator.cpp
  src/impl/agv/seer/seer_transport.cpp
)
target_compile_features(robot_move_impl PUBLIC cxx_std_17)
set_target_properties(robot_move_impl PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(robot_move_impl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_move_impl
  Threads::Threads
)

install(
  TARGETS robot_move_impl
  EXPORT robot_move_implTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(robot_move_implTargets HAS_LIBRARY_TARGET)
ament_export_libraries(robot_move_impl)
ament_export_dependencies(Threads)

add_executable(robot_move_node
  src/robot_move_node.cpp
  src/nodes/universal_robot_driver.cpp
)
ament_target_dependencies(robot_move_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  std_msgs
)
target_link_libraries(robot_move_node
  robot_move_impl
)
install(TARGETS robot_move_node
  DESTINATION lib/${PROJECT_NAME}
)

add_library(seer_odometry_ros src/odometry/odometry_node.cpp)
ament_target_dependencies(seer_odometry_ros rclcpp rcl_interfaces nav_msgs diagnostic_msgs tf2_ros)
target_link_libraries(seer_odometry_ros robot_move_impl)
add_executable(seer_odometry_node src/odometry/odometry_main.cpp)
target_link_libraries(seer_odometry_node seer_odometry_ros)
install(TARGETS seer_odometry_ros ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS seer_odometry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

add_executable(seer_localization_inspect src/localization/inspect.cpp)
target_link_libraries(seer_localization_inspect robot_move_impl)
install(TARGETS seer_localization_inspect DESTINATION lib/${PROJECT_NAME})
add_library(seer_localization_ros src/localization/node.cpp)
ament_target_dependencies(seer_localization_ros rclcpp rcl_interfaces geometry_msgs diagnostic_msgs tf2 tf2_ros tf2_msgs)
target_link_libraries(seer_localization_ros robot_move_impl)
add_executable(seer_localization_node src/localization/main.cpp)
target_link_libraries(seer_localization_node seer_localization_ros)
install(TARGETS seer_localization_ros ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS seer_localization_node DESTINATION lib/${PROJECT_NAME})
add_executable(seer_localization_demo src/localization/demo.cpp)
ament_target_dependencies(seer_localization_demo nav_msgs)
target_link_libraries(seer_localization_demo seer_localization_ros)
install(TARGETS seer_localization_demo DESTINATION lib/${PROJECT_NAME})

add_library(navigation_guard_core src/navigation/guard.cpp)
target_compile_features(navigation_guard_core PUBLIC cxx_std_17)
target_include_directories(navigation_guard_core PUBLIC include)
target_link_libraries(navigation_guard_core Threads::Threads)
add_library(navigation_guard_ros src/navigation/node.cpp)
ament_target_dependencies(navigation_guard_ros rclcpp rcl_interfaces geometry_msgs sensor_msgs
  nav_msgs tf2_msgs tf2_ros diagnostic_msgs std_srvs)
target_link_libraries(navigation_guard_ros navigation_guard_core robot_move_impl)
add_executable(navigation_guard_node src/navigation/main.cpp)
target_link_libraries(navigation_guard_node navigation_guard_ros)
install(TARGETS navigation_guard_core navigation_guard_ros ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS navigation_guard_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(navigation_node_test test/navigation_node_test.cpp)
  target_link_libraries(navigation_node_test navigation_guard_ros)
  add_test(NAME navigation_node_test COMMAND navigation_node_test)
  set_tests_properties(navigation_node_test PROPERTIES TIMEOUT 30 ENVIRONMENT "ROS_DOMAIN_ID=217")
  add_executable(navigation_guard_test test/navigation_guard_test.cpp)
  target_link_libraries(navigation_guard_test navigation_guard_core robot_move_impl)
  add_test(NAME navigation_guard_test COMMAND navigation_guard_test)
  set_tests_properties(navigation_guard_test PROPERTIES TIMEOUT 15)
  add_executable(localization_source_test test/localization_source_test.cpp)
  target_link_libraries(localization_source_test robot_move_impl)
  add_test(NAME localization_source_test COMMAND localization_source_test)
  set_tests_properties(localization_source_test PROPERTIES TIMEOUT 15)
  add_executable(localization_test test/localization_test.cpp)
  target_link_libraries(localization_test robot_move_impl)
  add_test(NAME localization_test COMMAND localization_test)
  add_executable(localization_node_test test/localization_node_test.cpp)
  target_link_libraries(localization_node_test seer_localization_ros)
  add_test(NAME localization_node_test COMMAND localization_node_test)
  set_tests_properties(localization_test localization_node_test PROPERTIES TIMEOUT 30 ENVIRONMENT "ROS_DOMAIN_ID=219")
  find_package(tf2_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  add_executable(odometry_test test/odometry_test.cpp)
  target_link_libraries(odometry_test robot_move_impl)
  add_test(NAME odometry_test COMMAND odometry_test)
  add_executable(odometry_node_test test/odometry_node_test.cpp)
  target_link_libraries(odometry_node_test seer_odometry_ros)
  ament_target_dependencies(odometry_node_test tf2_msgs rosgraph_msgs)
  add_test(NAME odometry_node_test COMMAND odometry_node_test)
  set_tests_properties(odometry_test odometry_node_test PROPERTIES TIMEOUT 30 ENVIRONMENT "ROS_DOMAIN_ID=218")
  foreach(test_name seer_transport_test seer_client_test agv_robot_move_test agv_integration_test)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} robot_move_impl)
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 30)
  endforeach()
  add_executable(driver_lifecycle_test test/driver_lifecycle_test.cpp)
  target_link_libraries(driver_lifecycle_test robot_move_impl)
  add_test(NAME driver_lifecycle_test COMMAND driver_lifecycle_test)
  set_tests_properties(driver_lifecycle_test PROPERTIES TIMEOUT 15)

  add_executable(driver_node_test
    test/driver_node_test.cpp
    src/nodes/universal_robot_driver.cpp
  )
  ament_target_dependencies(driver_node_test rclcpp rcl_interfaces geometry_msgs std_msgs)
  target_link_libraries(driver_node_test robot_move_impl)
  add_test(NAME driver_node_test COMMAND driver_node_test)
  set_tests_properties(driver_node_test PROPERTIES TIMEOUT 15)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
