# AGV_ROS2_Control_Manager

**双机械臂移动 AGV（含超声/无损检测扫查）ROS 2 控制系统工作区**

本仓库是一个 ROS 2 工作区（`src/` 下为各功能包），面向一套「移动底盘 AGV + 升降机构 + 回转云台 + 左右双机械臂」的复合机器人。整体采用 `ros2_control` 统一驱动硬件（珞石 Rokae 协作机械臂 + 仙工 Seer AGV 底盘），通过 WebSocket / 自定义协议与上位机通信，并由视觉 + 深度学习（SAM2）规划机械臂末端扫查路径，用于超声 / 风电叶片等无损检测扫查作业。

> 本文件是对 `src/` 下每个文件夹的分析与说明。机器人涉及左右臂两套逻辑（`arm1` / `arm2`），当前实际动作的主要是**右臂 `arm2`**（相机、路径规划、工具扫查均挂在 `arm2` 链路上），代码中保留的 `arm1` 链路与文档存在部分历史残留。

---

## 1. 目录总览

| 目录 | 类型 / 语言 | 一句话职责 | 自带文档 |
|------|-------------|-----------|---------|
| `agv_protocol` | C++ ament_cmake | 底层 WebSocket 客户端/服务器封装 + 统一 JSON 命令/状态解析的协议库 | — |
| `agv_bridge` | C++ ament_cmake | 中介者(Mediator)模式：底盘 `cmd_vel` 下发、机器人状态回传、雷达/相机心跳探测 | — |
| `agv_hardware` | C++ ament_cmake | **核心**：`ros2_control` 的 `SystemInterface` 硬件接口，桥接机械臂/底盘/上位 WebSocket | `READEME.md`(拼写如此)、`ROS2.md`(通信协议) |
| `agv_description` | ament_cmake(纯资源) | 机器人 URDF/xacro 模型 + mesh + ros2_control 接口声明 | — |
| `agv_bringup` | ament_cmake(纯资源) | 整机主启动编排（`agv.launch.xml`） | — |
| `agv_moveit_config` | ament_cmake(纯资源) | MoveIt 2 运动规划配置（两条臂 + 云台底座） | — |
| `moveit2_calibration` | vendored 第三方包 | 上游 `moveit_calibration` 整目录拷贝，供**手眼标定开发**使用（非运行时） | `README.md` |
| `camera` | C++ ament_cmake | 奥比中光 Astra-s RGB-D 相机 ROS 2 驱动（彩色/深度图 + CameraInfo + 静态 TF） | `README.md` |
| `robot_move` | C++ ament_cmake | 通用底盘驱动节点：工厂 + 策略封装 AGV/气吸附/磁吸附/涵道四类底盘 | — |
| `startup` | Python(CMake+ament) | 独立 WebSocket 服务，远程一键启停整套 launch | `README.md` |
| `pathplannernode` | 外层容器（非包） | 仅存放 `src/config/params.yaml`、点云结果 `result/`、ROI 示例 `roi.png` | — |
| └ `pathplannernode/src/pathplanner` | **Python ament_python** | SAM2 视觉 + 点云 → 机械臂 6D 扫查路径规划节点 | `ROS_README.md`、`Python_README.md` |
| `AGENTS.md` | — | 本仓库给 AI 助手的协作约定（结构 / 构建 / 风格） | — |

> 说明：colcon 会**递归**下钻子目录查找 `package.xml`。因此嵌套在 `pathplannernode/src/` 下的 `pathplanner`、以及 `moveit2_calibration` 下的三个 `moveit_calibration_*` 子包，都会连同直接位于 `src/<包名>/` 下的包一起被自动发现。**`src/` 下共 13 个可构建包**（已用 `colcon list` 验证）。

---

## 2. 机器人本体构成

以 `agv_description/urdf/agv.xacro` 为准的整机运动学（`base_link` 仅含升降 + 回转 + 双机械臂，**不含移动轮**，真实移动底盘由 Seer 控制器驱动，不在本模型内）：

```
world ──► base_link（移动底盘，mass≈40.7kg）
    └─ lifiting_joint [prismatic, 升降 0~0.5 m]
        └─ lifting_link
            └─ revolute_joint [revolute, ±3.14 rad, 回转云台]
                └─ revolute_link
                    ├─ arm1_base_joint(fixed, 朝向 +x)
                    │   └─ arm1_joint1..6 ×revolute → arm1_tool_link
                    └─ arm2_cartesionbase_link(fixed, 镜像 -x)
                        └─ arm2_joint1..6 ×revolute → arm2_tool_link
                            └─ arm2_toolscan_link（末端扫查工具端，挂相机/扫查器）
```

- **机械臂**：珞石 Rokae xMate 六轴协作机械臂 ×2（互为镜像），6+6 共 12 个旋转关节。
- **AGV 底座**（此处指控制器分组 `agvBase`）：`lifiting_joint`（升降）+ `revolute_joint`（回转），非移动轮。
- 关节限位统一 effort 1000 / velocity 1.0 / ±3.0 rad；mesh 全部来自 `agv_description/meshes/`（STL）。
- `agv_yuan.xacro` 为去掉右臂（arm2）的旧版单臂原型，当前不启用；**启用的是 `agv.xacro`**。

---

## 3. 系统架构与数据流

```
                            上位机 / 浏览器
                                 │
        ┌────────────────────────┼───────────────────────────┐
        │  WebSocket JSON 协议    │ (ws://192.168.1.108:9100,  见 ROS2.md)
        │                        │  本地启动服务 ws://:9001)
        ▼                        ▼
   [startup]               [agv_hardware: AgvHardwareInterface]   ← ros2_control SystemInterface 插件
   远程启停 launch            依赖 agv_protocol(WS+解析) / agv_bridge(中介)
                                 │  │  │
        controller_manager ◄──────┤  │  ├─ RobotController ──► Rokae xMate 臂 (TCP, xCoreSDK, .158)
   (agv.launch.xml @1000Hz)      │  │  ├─ AGVRobotMove   ──► 仙工 AGV 底盘 (TCP 4端口, SEFA帧)
   joint_state_broadcaster        │  │  └─ 话题: cmd_arm_joint / cmd_arm_cartesian / read上报
   arm1/arm2/agvBase 控制器       │  │
   (JointTrajectoryController)   │  └─ BridgeManager ──► /cmd_vel, robot_status, 心跳订阅
        ▲                        ▼
        │                [agv_moveit_config: move_group]     规划 arm1 / arm2 / agvBase
        │                        │
        └────────────────────────┴── [camera: Astra RGB-D] ─► color|depth/image_raw + arm2_tool_link→camera TF

   [robot_move: UniversalRobotDriver]  ← /cmd_vel、scan_config… → AGV/气/磁/涵道底盘, 发布 /agv_pose
   [pathplanner: SAM2] 订阅 color/depth/click → /path_planner/path_3d、/path_planner/cartesian_path (arm2 基座系)
```

运行主入口为 `ros2 launch agv_bringup agv.launch.xml`，其内部依次拉起 robot_state_publisher → controller_manager(加载 `ros2_controllers.yaml`) → spawn 4 个 controller → include MoveIt `move_group.launch.py` → rviz2。相机、pathplanner、robot_move、startup 多为独立启动。

---

## 4. 功能包详解

### 4.1 `agv_protocol` — WebSocket 通信与命令解析协议包

**定位**：最底层工具库，为上位通信提供 WebSocket 客户端/服务器封装与统一 JSON 指令/状态解析，**不依赖工作区任何其它包**（依赖 jsoncpp / websocketpp / Boost.ASIO）。

**核心符号**（`include/agv_protocol/`）：
- `command_parser.hpp` — `MessageHeader{robot_id, msg_type}`（`robot_id` 如 `mobile_dual_arm_robot`；`msg_type` = check / command / query / status_update）、`ArmCommand`、`CameraCommand`、`CommandData`；`CommandParser` 负责 `parse()` 入向命令，`buildStatusMessage / buildPosturePositionsMessage / buildCommandResponse / buildCheckResponse` 构建出向消息。
- `websocket_client.hpp` — `WebSocketClientWrapper`，主动连上位机，支持断线重连与消息/连接回调。
- `websocket_server.hpp` — `WebSocketServerWrapper`，本地监听入向指令。

**对外接口**：无自有 ROS topic/service，以库 + 回调形式被 `agv_hardware` 调用。

### 4.2 `agv_bridge` — 中介者模式的底盘消息分发包

**定位**：单例中介（`BridgeManager::getInstance()`），在硬件接口与 ROS 话题间解耦；负责底盘速度下发、状态回传、设备心跳在线探测与电量维护。

**对外 ROS 接口**：
- Publisher：`cmd_vel`（`geometry_msgs/Twist`）、`robot_status`（`std_msgs/String`）
- Subscriber：`radar_heartbeat`、`camera_heartbeat`（`std_msgs/Header`）→ 2s 内有心跳判定在线
- 查询方法：`isRadarOnline() / isCameraOnline() / getBatteryLevel()`

**依赖**：rclcpp / geometry_msgs / std_msgs；被 `agv_hardware`（自检、底盘命令）调用，无第三方 SDK。

### 4.3 `agv_hardware` — 硬件接口与机械臂/底盘控制器（核心包）

**定位**：实现 `hardware_interface::SystemInterface`（插件 `agv_hardware/AgvHardwareInterface`），把「控制器命令 → 底层硬件」打通：珞石 xMate 臂控制、仙工 AGV 底盘 TCP 控制、与上位机双向 WebSocket，是 `agv_description` / `agv_bringup` 装配链的核心。

**核心文件**：
| 文件 | 内容 |
|------|------|
| `include/agv_hardware/agv_hardware.hpp` | `AgvHardwareInterface`，实现 `on_init/on_configure/on_activate/on_deactivate/read/write`，聚合 `RobotController`、WS 客户端/服务器、`CommandParser`、`AGVRobotMove` |
| `src/agv_hardware_interface.cpp` | 生命周期逻辑：`on_configure` 连机械臂(`robot_ip`)、`read()` 上报关节/位姿并周期性 `buildStatusMessage` 上报上位、`write()` 在 RT 模式下把 `joint_commands_` 下发实时核；`handleCommand()` 分发 check/急停/机械臂/相机/底盘命令 |
| `include/agv_hardware/robot_controller.hpp` + `src/robot_controller.cpp` | `RobotController`，封装 Rokae SDK：`connect / activate / deactivate`（RT 或 NRT 模式）、关节与笛卡尔 moveAppend/moveStart 指令队列 |
| `include/robot_gotarget.hpp` | 底盘抽象基类 `RobotMove`(`init/setVelocity/send_loop/moveshutdown/movebydistance/rotatebyangle/getpose`) 及 `AGVRobotMove`（仙工 SEFA 控制器，TCP 四端口）等派生 |
| `agv_hardware_interface.xml` | pluginlib 插件描述 |
| `ROS2.md` | 与上位机的完整 WebSocket JSON 协议文档 |
| `websocket_server.py` / `AGVWebSocketServerFixed.cs` | 配套的上位机**模拟服务器**参考实现（端口 9000，非本包运行时依赖） |

**对外 ROS 接口**：
- Publisher：`arm_joint_states`、`arm_cartesian_pose`（`Float64MultiArray`）、`arm_operation_state`（`Int32`）、`/click_point`（`PointStamped`）
- Subscriber：`cmd_arm_joint`（6 关节角）、`cmd_arm_cartesian`（6 维笛卡尔位姿）、`/path_planner/cartesian_path`（`PoseArray`）
- controller ↔ hardware：URDF `*.ros2_control.xacro` 声明该插件 + 12 机械臂关节/升降/回转的 position 接口；`agv_bringup/config/ros2_controllers.yaml` 声明 `arm1_controller` / `arm2_controller` / `agvBase_controller`（均为 `JointTrajectoryController`）+ `joint_state_broadcaster`，由 `agv.launch.xml` 装配。

**通信 / 第三方**：
- 机械臂：Rokae **xCoreSDK**（`include/rokae/`，动态库在 `lib/Linux/{x86_64,aarch64}`），TCP 到控制器；RT 模式走 `RtControllerMode::jointPosition`，NRT 模式走 `moveAppend + moveStart` 指令队列。
- 底盘：仙工 Seer 自定义帧 `0x5A 0x01 + seq + len + type + JSON`（cJSON 组包），TCP 端口 19210(抱闸)/19204(查询)/19205(控制)/19206(导航)；命令 type 如 6001(松刹车)、2010(setVelocity)、3055/3056(平动/转动) 等。
- Eigen 以 `external/Eigen` 头文件-only 形式内置。

### 4.4 `agv_description` — 机器人模型描述包

**定位**：URDF/xacro 描述包（纯资源）。顶层 `urdf/agv_robot.urdf.xacro` 只 include `agv.xacro`（运动学）+ `agv_description.ros2_control.xacro`（ros2_control 接口）；`meshes/` 存放整机 STL；`rviz/urdf_config.rviz` 可视化配置。

**配套文件**：
- `urdf/agv_description.ros2_control.xacro` — 声明 `agv_hardware/AgvHardwareInterface` 插件与各关节 position 接口。
- `config/joint_names_agv_description.yaml` — 控制器所需关节名清单。
- `launch/display.launch.xml` — robot_state_publisher + joint_state_publisher_gui + rviz2 纯可视化（不含控制）。
- `launch/gazebo.launch` — ROS1-gazebo 遗留文件，非 ROS 2 有效路径。

### 4.5 `agv_bringup` — 整机启动编排包

**定位**：运行时主入口（`ros2 launch agv_bringup agv.launch.xml`）。

- `launch/agv.launch.xml`：robot_state_publisher → controller_manager（参数 `config/ros2_controllers.yaml`，update_rate **1000 Hz**）→ spawn `joint_state_broadcaster` / `agvBase_controller` / `arm1_controller` / `arm2_controller` → include `agv_moveit_config` 的 `move_group.launch.py` → rviz2。相机节点被注释（需单独 `ros2 launch camera camera_with_tf.launch.py`）。
- `config/ros2_controllers.yaml`：三个 `joint_trajectory_controller`（`agvBase`=升降+回转，`arm1`/`arm2`=各 6 关节）+ 一个 `JointStateBroadcaster`。

### 4.6 `agv_moveit_config` — MoveIt 2 配置包

**定位**：MoveIt Setup Assistant 自动生成的配置包，为两条臂 + 云台底座做运动规划与执行。

- `config/agv_description.srdf`：规划组 `arm1`（arm1_joint1..6 + 工具）、`arm2`（arm2_joint1..6 + 工具）、`agvBase`（virtual_joint + 升降 + 回转）；含 `arm1_home / arm1_pos1 / arm2_home / arm2_pos2 / agvBase_pos1 / pos2` 等 group_state 与大量碰撞规避表。
- `config/kinematics.yaml`：arm1/arm2 共用 KDL 运动学插件。
- `config/moveit_controllers.yaml`：`MoveItSimpleControllerManager`，把规划结果送到 `arm1/arm2/agvBase_controller` 的 `FollowJointTrajectory` action 执行。
- `config/ros2_controllers.yaml` + `agv_description.ros2_control.xacro`（FakeSystem）：MoveIt 侧**假硬件/模拟测试**配置（update_rate 100 Hz），便于不上真机试规划。
- `launch/move_group.launch.py`：被 `agv_bringup` include，为本包实际被用入口；另有 demo/rsp/moveit_rviz/spawn_controllers/warehouse_db/setup_assistant 等自动生成 launch。

### 4.7 `moveit2_calibration` — vendored 第三方手眼标定包

**定位**：上游 `ros-planning/moveit_calibration` 的**整目录拷贝**（作者签名、LICENSE、CI、README 均为上游），不是自研代码。包含三个子包：

- `moveit_calibration_plugins` — 手眼标定插件：标定靶（`HandEyeTarget/Aruco`、`HandEyeTarget/Charuco`）与求解器（`HandEyeSolverDefault`，OpenCV 实现）。
- `moveit_calibration_gui` — Rviz 手眼标定工作流插件。
- `moveit_calibration_demos` — Gazebo 演示（上游 Panda 机械臂，非本机器人）。

**使用方式**：仅作为 **Eye-in-Hand 手眼标定开发工具**，运行时**不依赖**它；相机→右臂末端的标定结果被回填到 `camera/config/camera_tf.yaml`。

### 4.8 `camera` — Astra RGB-D 相机 ROS 2 驱动

**定位**：基于奥比中光 OpenNI2 SDK 的 Astra-s RGB-D 相机驱动，分层设计：`interface/camera_interface.hpp`（抽象）→ `astras/astras_camera.hpp`（`AstraSCamera` 实现）→ `nodes/astras_node.cpp`（`AstraSNode`）。

**发布话题 / TF**：
- `color/image_raw`（+`color/camera_info`，rgb8）
- `depth/image_raw`（+`depth/camera_info`，16UC1）
- IR 关闭，**不发布点云**（仅图像）
- 静态 TF：`config/camera_tf.yaml` 参数化父系。当前生效值 **`parent_frame_id: arm2_tool_link`**（配置注释历史写 arm1，存在文档/配置不一致）；`tf_translation/rotation` 由 moveit_calibration 手眼标定得到，即相机挂在右臂末端做 Eye-in-Hand。

**启动**：`ros2 launch camera camera_with_tf.launch.py`（被 `agv_bringup` 以注释形式引用）。依赖 OpenCV / cv_bridge / image_transport / tf2_ros 及随包固定 SDK（`sdk/`）。

### 4.9 `robot_move` — 通用底盘运动驱动包

**定位**：把「不同运动底盘/机器人的底层通信协议」统一抽象为 `RobotMove` 接口，由单个节点 `UniversalRobotDriver` 依据参数 `robot_type` 运行时挑选底盘——典型**工厂 + 策略**模式。

**目录结构**：`interface/robot_move_base.hpp`(抽象基类 `RobotMove`) → `factory/robot_move_factory.hpp/.cpp`(`createRobotMove(type)` / `defaultConnectionForType(type)`) → `impl/`(四个实现) → `nodes/universal_robot_driver.hpp/.cpp`。

**四种被控对象**（`impl/`）：
| 实现 | 对象 | 通信 | 备注 |
|------|------|------|------|
| `agv/AGVRobotMove` | 仙工 Seer AGV 底盘 | TCP 4 端口(19210/19204/19205/19206)，`0x5A 0x01` SEFA 帧 + cJSON | 重定位/速度/定长平移/定角旋转/松刹车 |
| `air/AirRobotMove` | 气吸附(负压爬壁)机器人 | 串口 SBUS 16 通道帧 | `ch[0]`转向、`ch[1]`前进、吸附风扇 PWM |
| `duct/DuctRobotMove` | 涵道/风管扫查机器人 | 串口 SBUS 帧 | 步进电机 + 探伤扫查器电源/转速控制 |
| `mag/MagRobotMove` | 磁吸附爬壁机器人 | 串口 115200 8E1 | VAL_LOW/MID/HIGH 数字信号映射 |

**对外话题**：
- Subscriber：`cmd_vel`(Twist→setVelocity)、`scan_config`(Twist：speed/distance/precision→setScannerConfig)、`scan_ctrl`(Empty→触发扫查)、`robot_distance_move`(Twist：距离/速度→movebydistance)、`robot_rotate_move`(Twist：角度/角速度→rotatebyangle)
- Publisher：`agv_pose`(Pose2D，20ms 定时器周期读取)
- 无 service/action。入口主程序 `src/robot_move_node.cpp`。

### 4.10 `startup` — WebSocket 远程启停服务

**定位**：独立 Python WebSocket 服务器（`src/agv_startup_websocket.py`，端口 **9001**），接收 JSON 指令用 `ros2 launch` 启停 `agv_bringup/agv.launch.xml`，供上位远程一键上下电整套系统。

- 下行请求：`{"target":"arm_left_start","command":"true"|"false"}`
- 上行响应（type）：`connection_ack / command_ack(status: started/stopped/already_running/...) / error`
- 脚本：`start_agv_server.sh`(启动)、`test_websocket_client.py`、`test_server.sh`
- 依赖 rclpy / ros2launch / python3-websockets；运行时需先有 `install/setup.bash` 环境。

### 4.11 `pathplannernode`（外层）与嵌套的 `pathplanner` 包

**外层 `pathplannernode/` 不是 ROS 包**，只承载：
- `src/config/params.yaml` — ROS 参数文件（`scan_mode`、`spacing`、`shrink_factor`、`interpolation_points`、`attention_method`、`interactive_mode`、`enable_visualization` 等）
- `result/`（点云结果导出）、`roi.png`（示例 ROI 掩码）

**真正的 ROS 包 `pathplannernode/src/pathplanner`（ament_python）** 是一个基于 **SAM2 分割 + 点云 + 几何注意力** 的机械臂末端探测/扫查路径规划包：
- 模型：`sam2model/sam2.1_l.pt`，经 Ultralytics `SAM` 加载推理。
- 算法模块（`pathplanner/`）：`attention_path_ros2.py`(主类 `InteractiveSegmentationROS2`) + 点云处理 `pointcloud_processing.py`、路径生成 `path_generation.py`(栅格/旋转矩形/平滑轮廓三类)、路径优化 `path_optimization.py`、局部坐标系 `local_coordinate.py`、注意力热力图 `attention_computation.py`、可视化 `visualization.py`。
- ROS 节点 `pathplanner_ros/pathplanner_ros2_node.py`：
  - Subscriber：`color/image_raw`、`depth/image_raw`、`/click_point`(触发管线)、TF
  - Publisher：`/path_planner/path_3d`(PointCloud2，含 x/y/z/rx/ry/rz)、`/path_planner/cartesian_path`(PoseArray，坐标轴转换到 **arm2 基座系**，含 `arm2_tool_link→arm2_toolscan_link` 工具补偿) —— 供机械臂执行扫查。
- `srv/PlanPath.srv`：预留的离线/正式 API 服务定义（当前未接线为 service）。
- launch：`launch/ros2_pathplanner_launch.py`（`ros2 launch pathplanner ros2_pathplanner_launch.py`）。
- 依赖：numpy / opencv / open3d / scipy / ultralytics / torch 等。

---

## 5. 构建、测试与运行

> 命令在**工作区根目录**执行。

```bash
# 全量构建
colcon build --symlink-install

# 单包迭代
colcon build --symlink-install --packages-select agv_hardware

# 叠加环境
source install/setup.bash

# 运行整机
ros2 launch agv_bringup agv.launch.xml
ros2 topic pub --once /cmd_arm_joint std_msgs/msg/Float64MultiArray "{data:[1.047,-1.570,-2.0943,-2.6179,-1.8849,0.1745]}"


# 相机 / 路径规划 / 底盘驱动 / 启动服务
ros2 launch camera camera_with_tf.launch.py
ros2 launch pathplanner ros2_pathplanner_launch.py      # 见下方嵌套包注意事项
ros2 run robot_move robot_move_node --ros-args -p robot_type:=agv
ros2 run startup agv_startup_websocket.py

# 测试
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

**嵌套包说明**：colcon 会递归发现子目录中的包——`src/` 下共 13 个可构建包（含嵌套的 `pathplanner`、`moveit2_calibration` 的 `moveit_calibration_demos/gui/plugins`），在根目录一条 `colcon build --symlink-install` 即可全部构建。`pathplanner` 为 ament_python 包，构建本身不耗资源，但其**运行依赖** numpy / opencv-python / open3d / ultralytics / torch 等 Python 库需另行准备。

---

## 6. WebSocket JSON 通信协议（速览）

完整协议见 `src/agv_hardware/ROS2.md`。统一信封：

```json
{ "header": { "robot_id": "mobile_dual_arm_robot", "msg_type": "command|check|query|status_update" },
  "payload": { ... } }
```

- **command payload**：`arms[]`（arm_id / command：cartesian_move|joint_move / parameters / speed）、`agv` 或 `chassis`（`emergency_stop`、`move`/`velocity_move` + vx/vy/wz）、`camera`。
- **check payload**：各设备（radar/camera/agv…）→ `{"command":"check"}`。
- **出向 status_update**：`arms[i]` 带 `pos_type`（joint：joint1..6 或 cartesian：x,y,z,rx,ry,rz）；response：`result: "xxx_successed"` 等。

---

## 7. 坐标系 / TF 要点

- `base_link`（底盘）→ 升降 → 回转 → 左/右臂基座 → 机械臂末端（`arm2_tool_link`）→ `arm2_toolscan_link`（扫查工具端）。
- 相机静态 TF：`arm2_tool_link → camera_color_frame / camera_depth_frame`（Eye-in-Hand，由 `camera_tf.yaml` 参数化）。
- pathplanner 输出：先算 `arm2_cartesionbase_link → camera_color_frame` 手眼复合，再叠加 `arm2_tool_link → arm2_toolscan_link` 工具补偿，得到机械臂基座系可执行位姿。
- 移动底盘位姿由 `robot_move` 以 `agv_pose`（Pose2D）周期性发布。

---

## 8. 已知事项与注意事项

1. **控制器分组**：`agvBase_controller` 驱动的是升降 + 回转两个关节，不是移动轮；移动 AGV 底盘由 Seer 控制器通过 TCP 单独驱动。
2. **arm1 / arm2 文档历史残留**：多处注释/README（如相机 TF、pathplanner 旧模型）写的是 `arm1`，实际生效链路为 `arm2`（`arm2_tool_link` / `arm2_cartesionbase_link`），改动时请以代码为准。
3. **底盘驱动代码重复**：`robot_move` 与 `agv_hardware` 内各含一套 Seer 底盘驱动（`AGVRobotMove` 等），协议一致但维护需留意同步。
4. **moveit2_calibration 为第三方拷贝**：不要当作本工作区自研代码提交修改，仅作手眼标定工具；三个子包仍用原名 `moveit_calibration_*`。
5. **SDK / 模型均为受管工件**：`agv_hardware/lib`（Rokae SDK）、`camera/sdk`（OpenNI2）、`pathplanner/sam2model/sam2.1_l.pt` 为大文件/二进制产物，替换需注明来源与平台。
6. **路径规划安全**：涉及机械臂运动、扫查执行时需确认坐标系与工具补偿，遵守机器人安全规范；launch 请勿提交机器特定 IP。
7. **startup 与 agv_hardware 的本地 WS 端口同为 9001**：分属不同进程、不同时机运行，避免同时占用冲突。
8. 存在 `_rokae_log_` / `result` / `roi.png` 等运行产物与参考图像，提交时注意是否纳入版本控制。

---

## 9. 相关文档索引

| 文档 | 位置 | 内容 |
|------|------|------|
| 协作约定 | `src/AGENTS.md` | 结构 / 构建命令 / 代码风格 / 提交规范 |
| 通信协议 | `src/agv_hardware/ROS2.md` | 与上位机 WS JSON 完整协议 |
| 硬件包速览 | `src/agv_hardware/READEME.md` | 下位机职责 / 流程 |
| 相机说明 | `src/camera/README.md` | Astra-s 驱动说明 |
| 启动服务 | `src/startup/README.md` | WS 启停服务使用 |
| 手眼标定(第三方) | `src/moveit2_calibration/README.md` | 上游项目文档 |
| pathplanner | `src/pathplannernode/src/pathplanner/ROS_README.md`、`Python_README.md` | ROS 节点与算法工程说明 |
| **跨仓库接口契约** | `docs/AGV_PAUT_集成契约.md` | 本仓库（机器人平台）与 `PAUT_ros2`（超声感控）之间的话题/TF/启动约定 |
| **话题/服务/动作清单** | `docs/话题清单.md` | 本仓库全部 ROS 通信接口（话题/服务/动作）静态清单 |
