# Gazebo Harmonic 环境搭建与验收

日期：2026-11-03。工作区使用 Ubuntu 24.04 x86_64、ROS 2 Jazzy。

**最终状态：环境及官方示例验收通过。后续已按用户确认的简化全向方案跑通本项目Gazebo导航，包含雷达、里程计、AMCL、Nav2、目标执行、绕障和取消；详见[全向导航使用说明](AGV全向底盘Gazebo导航使用说明.md)。真实舵轮动力学尚未建模。**

本记录用于先确认环境可用，再接入本项目底盘与导航；所有运动验证只针对官方仿真模型，不连接仙工、机械臂或其他真实设备。

## 1. 基线与验证状态

| 项目 | 当前证据 |
|---|---|
| 系统 | Ubuntu 24.04.3 LTS，x86_64 |
| ROS | `/opt/ros/jazzy` |
| GPU | NVIDIA GeForce GTX 1650，4096 MiB；驱动580.126.09 |
| 图形 | X11可访问；glxinfo显示NVIDIA直接渲染，OpenGL 4.6 |
| Gazebo | 已安装，`gz sim --versions`返回8.15.0（Harmonic系列） |
| 空场景 | 已实际启动GUI，能显示地面和网格 |
| ROS时钟 | 收到20条递增的`/clock`消息，初次验收通过 |
| 官方差速小车 | 配套库更新后原版示例通过：前进约0.4m、旋转约0.8rad、停止，里程计/实际模型位姿/车轮反馈/TF均有证据；检查脚本退出码0 |
| 本项目静态模型 | 已在Gazebo实际生成agv_preview，21个link、20个固定关节、35个网格引用；无真实硬件声明 |
| Nav2初始入口 | 已验证5个核心生命周期节点以unconfigured状态启动；未进行导航目标执行 |
| 本项目底盘运动类型 | 后续已确认为舵轮式；用户确认先用简化全向模型跑通导航，差速示例仍仅作环境检查 |

官方支持的Jazzy/Gazebo组合见[Gazebo安装说明](https://gazebosim.org/docs/harmonic/ros_installation/)，控制桥接见[gz_ros2_control说明](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)。

## 2. 安装依赖

由用户在本机终端执行sudo命令；自动化会话没有可用的免密sudo权限。

```bash
sudo apt-get update
sudo apt-get install \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-ros2-control-demos \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  mesa-utils
```

Nav2和SLAM依赖为后续开发准备；安装成功本身不代表导航已经配置或通过验收。

检查命令：

```bash
source /opt/ros/jazzy/setup.bash
gz --commands
gz sim --versions
glxinfo -B
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix gz_ros2_control_demos
```

本次安装后检测到：ros_gz 1.0.24、gz_ros2_control 1.2.20、Nav2 1.3.13、slam_toolbox 2.8.5。具体Debian修订以`dpkg-query -W`为准。

## 3. 隔离验证环境

下述命令所在的每个终端均先设置：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=187
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=agv_gazebo_validation_zyj
```

此处固定域号和Gazebo分区用于隔离环境验收。运行检查脚本时，两边必须一致。其他程序若已经占用该验证域，需要先协调，不能把测试速度指令发送给真实驱动。

验证不需要加载本工程的真实硬件启动文件。

## 4. 空场景与ROS时钟

终端A启动空场景：

```bash
ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:='-r -v 3 empty.sdf' on_exit_shutdown:=true
```

终端B启动单向Gazebo到ROS的时钟桥接：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

终端C检查：

```bash
ros2 topic echo /clock --once
ros2 topic hz /clock
```

本次还通过rclpy订阅采集20个样本，检查时间单调且实际递增；结果记录在`/tmp/agv-gazebo-validation/empty-clock-result.json`。

完成后分别Ctrl+C结束A/B，确认进程退出，再运行下一个示例，避免同一分区内出现多个world或时钟发布源。

## 5. 官方移动机器人示例

使用系统包提供的原版启动文件，不修改`/opt/ros`下的示例源码：

```bash
ros2 launch gz_ros2_control_demos diff_drive_example.launch.py
```

示例自身启动Gazebo、时钟桥接、模型生成、RobotStatePublisher及控制器，无需再单独启动第4节的桥接。

预期存在：

```text
硬件：gz_ros2_control/GazeboSimSystem
控制器：joint_state_broadcaster（active）
        diff_drive_base_controller（active）
命令：/diff_drive_base_controller/cmd_vel，TwistStamped
反馈：/diff_drive_base_controller/odom，Odometry
      /joint_states
      /tf中的odom → chassis
```

另开一个配置了第3节环境的终端，在工作区根目录执行：

```bash
/usr/bin/python3 tools/check_gazebo_mobile_demo.py
```

脚本会先核查仿真硬件和控制器状态，然后测试前进、转动及零速度停止；同时比较ROS里程计和Gazebo模型实际位姿，避免仅凭命令已发送或车轮转动判定小车已移动。

结果默认写入：`/tmp/agv-gazebo-validation/mobile-result.json`。任意必需项缺失会返回非零退出码，并记录失败原因。

已在未启动模拟器的条件下执行检查脚本：它因`/controller_manager/list_hardware_components`服务不可用而正确返回失败，没有进入运动阶段；记录为`no-simulator-guard-result.json`。这验证了缺少执行端时不会误报通过，正式运动验收仍待控制库更新后进行。

官方示例是差速车，不代表本项目实际底盘已经确认是差速，也不代表自定义模型已经接入。

## 6. 初次官方示例崩溃的排查

首次复现的调用栈落在`gz_ros2_control::GazeboSimSystem::initSim()`，加载首个车轮状态接口时发生段错误，随后controller_manager服务不可用。

当时实际安装版本：

```text
gz_ros2_control：1.2.20（2026-10构建）
hardware_interface / controller_manager：4.42.2（2026-01构建）
diff_drive_controller：4.36.0（2026-01构建）
仓库候选：ros2_control 4.48.1、ros2_controllers 4.42.1
```

已检查动态库路径，加载的是`/opt/ros/jazzy`库，并非本工程覆盖。随后将控制库同步到仓库配套版本。第一次重测继续暴露diagnostic_updater旧版缺少新版controller_manager构造函数符号的问题；经7个相关库的ldd -r检查确认，补齐diagnostic_updater后依赖检查及同一官方示例均通过。最终验收使用系统库，没有使用临时覆盖库。

控制库更新命令：

```bash
sudo apt-get install --only-upgrade \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager ros-jazzy-controller-manager-msgs \
  ros-jazzy-controller-interface ros-jazzy-hardware-interface \
  ros-jazzy-joint-limits ros-jazzy-realtime-tools \
  ros-jazzy-transmission-interface ros-jazzy-control-msgs \
  ros-jazzy-control-toolbox ros-jazzy-ros2controlcli \
  ros-jazzy-ros2-control-cmake ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller
```

该控制库命令试算为15项更新、4项新增、无删除。还需同步诊断库：

```bash
sudo apt-get install --only-upgrade ros-jazzy-diagnostic-updater
```

诊断库从4.2.6更新至4.2.7。最终组合：ros2_control核心4.48.1、diff_drive_controller等控制器4.42.1、gz_ros2_control 1.2.20、diagnostic_updater 4.2.7。

已重新编译本工程依赖硬件接口的插件。在隔离构建目录和正常工作区build/install中均完成相关构建及功能回归：8包构建成功，robot_move的6组功能测试和agv_hardware的4组接入测试通过。既有deprecated on_init等编译警告仍存在；这些结果不代表全部格式lint通过。

## 7. 当前验收文件

临时运行记录统一保存在`/tmp/agv-gazebo-validation/`，主要包括：

- `empty-world.log`：空场景启动及GUI日志。
- `empty-clock-bridge.log`：时钟桥接日志。
- `empty-clock-result.json`：时钟采样结果。
- `official-diff-drive.log`：首次版本混用时的崩溃日志。
- `official-diff-drive-final.log`：库版本对齐后的官方小车日志。
- `mobile-result.json`：小车运动、反馈与TF验收结果（脚本实际通过后才算完成）。

临时目录可能随系统清理消失，重要结论和后续进展需同步回本文件。本次最终记录区分环境通过、静态模型导入和完整导航能力，不能将三者混为一谈。


## 8. 最终运动与图形证据

- 空场景：20条ROS时钟样本连续递增；GUI实际显示地面与网格。
- 官方小车：里程计前进距离0.39995m，Gazebo模型实际前进距离0.39991m；转角分别为0.79978rad和0.79967rad。
- 零速度后，里程计线/角速度均小于0.03阈值，实际测量接近零。
- `/joint_states`包含左右轮，`/tf`包含`odom → chassis`。
- `mobile-result.json`的passed为true，检查脚本正常退出（退出码0）。
- 已实际查看Gazebo画面，确认原版黄色小车和本项目CAD模型均能渲染。截图暂存为`official-diff-drive.png`和`project-preview.png`。
- 日志仍有Qt绑定及EGL探测警告，实际NVIDIA图形显示和运动验收均通过；未因此改动系统显卡驱动。

## 9. 已开始的项目接入

### 9.1 独立的静态模型预览入口

代码：`src/agv_bringup/launch/agv_gazebo_preview.launch.py`。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zyj/Code/ROS2/AGV_new/AGV_ROS2_Control_Manager
source install/setup.bash
export ROS_DOMAIN_ID=187
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=agv_gazebo_validation_zyj
ros2 launch agv_bringup agv_gazebo_preview.launch.py
```

该入口只复用`agv_description/urdf/agv.xacro`的几何，不包含真实硬件ros2_control文件。为了首次检查几何，预览副本将所有关节固定在零位，并设为静态模型；源URDF和实机模型不变。

CAD中部分joint和link使用相同名字，URDF可表示，但固定关节转换为SDF frame时出现重名。预览副本为joint添加`preview_fixed_`前缀，保留所有link名字，解决了实际导入失败。

`z`默认1.0仅是预览高度，尚未标定轮子接地；`gui:=false`可仅运行服务器。此入口还没有轮子运动、雷达或里程计，不能用于导航验收。

运行时已核验：模型`agv_preview`存在，21个link、20个fixed joint、35个mesh引用均有效；发布的URDF无ros2_control定义，运行节点不含真实硬件接口。

### 9.2 Nav2启动入口

代码：`src/agv_bringup/launch/agv_navigation.launch.py`。

```bash
ros2 launch agv_bringup agv_navigation.launch.py \
  map:=/实际路径/地图.yaml \
  params_file:=/实际路径/AGV的Nav2参数.yaml \
  use_sim_time:=true autostart:=false
```

上面的地图和参数文件需按实际机器人准备。入口没有默认套用TurtleBot轮距、footprint、限速或运动模型，也不创建底盘驱动。默认不激活生命周期节点；确认输入完整后才能配置和激活。

本次仅使用官方示例地图与参数进行**未激活状态的启动检查**，命名空间为`agv_nav_smoke`。AMCL、map_server、controller_server、planner_server、bt_navigator均成功响应状态查询并保持unconfigured；随后已关闭这些检查进程。该检查不代表这份示例参数适合AGV，也不代表导航成功。

证据：`navigation-launch-smoke-result.json`。

### 9.3 只读导航输入检查

代码：`tools/check_agv_navigation_inputs.py`。

```bash
/usr/bin/python3 tools/check_agv_navigation_inputs.py --timeout 8
```

默认核验递增时钟、`/odom`、`/scan`、`map → odom → base_link`及底盘到雷达的TF。话题和frame可通过参数修改；实机使用`--real-time`不要求仿真时钟。脚本不发布速度或导航目标，输入存在也不代表精度和避障质量已经通过。

在当前静态预览上，实际结果是clock通过，其余运动/雷达/定位输入缺失，脚本正确返回1；记录为`navigation-inputs-preview.json`。这正是下一阶段需要补齐的数据链路。

## 10. 下一阶段开发前提

后续已确认真实AGV为舵轮底盘，并按用户要求采用简化全向模型完成导航开发。真实迁移时仍需确认轮系参数、雷达安装坐标和连续里程计来源；没有把官方差速小车当作真实底盘实现。

本文件记录环境验收及早期静态预览阶段。后续全向仿真已完成雷达、odom/TF、地图、定位与Nav2参数并通过导航测试，使用独立的agv_navigation_sim.launch.py入口。仙工1004接口仍不能未经语义核实就当作连续局部odom。

环境验收与静态预览的结果不等同于后续完整导航测试；后续结果已单独记录在全向导航使用说明。两阶段均未连接真实设备，未自动提交Git。
