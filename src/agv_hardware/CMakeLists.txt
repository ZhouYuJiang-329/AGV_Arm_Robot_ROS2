cmake_minimum_required(VERSION 3.12)
project(agv_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(agv_protocol REQUIRED)
find_package(agv_bridge REQUIRED)
find_package(robot_move REQUIRED)

# ============ OpenSSL 配置（先注释掉，使用非TLS版本）============
# find_package(OpenSSL REQUIRED)
# message(STATUS "OpenSSL version: ${OPENSSL_VERSION}")

# 查找 Boost（robot_controller 可能需要）
find_package(Boost REQUIRED COMPONENTS system thread)

# 添加 external 子目录，里面定义了 eigen target 
add_subdirectory(external)

# 添加 lib 子目录，里面定义了 xCoreSDK / xMateModel 等 target
add_subdirectory(lib)

# 添加头文件路径
include_directories(include)
include_directories(/usr/include/jsoncpp)

# 定义一个库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/agv_hardware_interface.cpp
  src/robot_controller.cpp
  src/config.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 添加编译定义
target_compile_definitions(${PROJECT_NAME} PRIVATE
  # ASIO_STANDALONE  # 不再需要，已迁移到 agv_protocol
  # BOOST_ERROR_CODE_HEADER_ONLY
)

# ROS2依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  hardware_interface
  pluginlib
  jsoncpp
  agv_bridge
  robot_move
)

# 普通CMake target依赖
target_link_libraries(${PROJECT_NAME}
  agv_protocol::agv_protocol
  robot_move::robot_move_impl
  eigen
  Rokae::Rokae
  jsoncpp_lib
  # ${OPENSSL_LIBRARIES}  # 暂时注释，使用非TLS版本
)

# 添加必要的库
target_link_libraries(${PROJECT_NAME}
  pthread
)

# 添加包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/include/jsoncpp
  # ${OPENSSL_INCLUDE_DIR}  # 暂时注释
)

# 添加 agv_protocol 的包含目录
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  ${agv_protocol_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(hardware_interface agv_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  eigen
  Rokae::Rokae
  jsoncpp
  agv_protocol
  agv_bridge
  robot_move
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(chassis_config_test test/chassis_config_test.cpp src/config.cpp)
  target_compile_features(chassis_config_test PRIVATE cxx_std_17)
  add_test(NAME chassis_config_test COMMAND chassis_config_test)

  add_executable(chassis_hardware_test test/chassis_hardware_test.cpp)
  target_link_libraries(chassis_hardware_test ${PROJECT_NAME})
  ament_target_dependencies(chassis_hardware_test rclcpp rclcpp_lifecycle std_msgs geometry_msgs
    tf2_ros tf2_geometry_msgs hardware_interface pluginlib agv_protocol agv_bridge robot_move)
  add_test(NAME chassis_hardware_test COMMAND chassis_hardware_test)
  add_test(NAME chassis_loopback_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/chassis_loopback_test.py $<TARGET_FILE:chassis_hardware_test>)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)
  add_test(NAME chassis_model_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/chassis_model_test.py $<TARGET_FILE:chassis_hardware_test>
    ${CMAKE_CURRENT_SOURCE_DIR}/.. ${XACRO_EXECUTABLE})
  set_tests_properties(chassis_model_test chassis_config_test chassis_hardware_test chassis_loopback_test
    PROPERTIES TIMEOUT 30)
endif()

ament_package()
