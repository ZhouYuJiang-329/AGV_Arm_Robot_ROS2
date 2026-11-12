cmake_minimum_required(VERSION 3.8)
project(agv_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(agv_bringup_sim)
install(PROGRAMS scripts/agv_simulation_adapter scripts/agv_simulation_ready scripts/agv_static_planning scripts/agv_navigation_execution_gate DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME agv_simulation_assets_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_simulation_assets.py ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME agv_real_navigation_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_real_navigation.py)
  set_tests_properties(agv_real_navigation_test PROPERTIES TIMEOUT 60 ENVIRONMENT "ROS_DOMAIN_ID=214")
  set_tests_properties(agv_simulation_assets_test PROPERTIES TIMEOUT 15)
endif()

ament_package()
