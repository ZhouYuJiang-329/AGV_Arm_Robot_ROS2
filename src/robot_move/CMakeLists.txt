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
find_package(Threads REQUIRED)

add_library(robot_move_impl
  src/cJSON.c
  src/factory/robot_move_factory.cpp
  src/impl/air/air_robot_move.cpp
  src/impl/duct/duct_robot_move.cpp
  src/impl/mag/mag_robot_move.cpp
  src/impl/agv/agv_robot_move.cpp
  src/impl/agv/seer/seer_client.cpp
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

if(BUILD_TESTING)
  enable_testing()
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
