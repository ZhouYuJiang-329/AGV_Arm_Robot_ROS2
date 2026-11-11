cmake_minimum_required(VERSION 3.8)
project(agv_description)

# 必要的 ament 工具
find_package(ament_cmake REQUIRED)

# 将资源安装到 share/<package>
install(
  DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME front_lidar_mount_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lidar_mount.py ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(front_lidar_mount_test PROPERTIES TIMEOUT 15)
endif()

ament_package()

