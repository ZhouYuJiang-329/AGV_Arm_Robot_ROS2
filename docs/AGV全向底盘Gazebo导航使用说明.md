# AGV简化全向底盘：Gazebo与Nav2导航

本功能用于本项目AGV导航的仿真开发。真实底盘已确认是舵轮式；本版按用户确认的目标，先使用支持vx、vy、w的理想全向运动模型，不还原每个舵轮的转向与滚动。

## 1. 一条命令启动

在工作区根目录编译并加载环境：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to agv_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch agv_bringup agv_navigation_sim.launch.py
```

启动后会打开Gazebo和RViz。启动顺序为：Gazebo场景及模型 → 时钟/里程计/雷达就绪 → AMCL与Nav2 → RViz。定位初始值与机器人出生点一致，默认自动激活导航。

该入口不加载`AgvHardwareInterface`或仙工驱动，不连接真实底盘或机械臂。ROS默认隔离在domain 187，Gazebo分区为`agv_navigation_sim`，不与原整机启动入口共用控制链路。

不要同时启动多份该仿真。测试结束在启动终端Ctrl+C，关闭整套进程后再重新启动。

### 启动选项

```bash
# 只用Gazebo窗口，不开RViz
ros2 launch agv_bringup agv_navigation_sim.launch.py rviz:=false

# 只运行服务器，不开两个可视窗口；GPU雷达仍需有效渲染环境
ros2 launch agv_bringup agv_navigation_sim.launch.py gui:=false rviz:=false

# 不自动激活Nav2，用于逐层调试
ros2 launch agv_bringup agv_navigation_sim.launch.py autostart:=false
```

`gui:=false`表示不显示Gazebo GUI，并不意味着可以在没有图形/EGL环境的任意服务器上直接运行GPU雷达。本机图形环境已单独验收。

## 2. RViz中下发目标

1. 等待Navigation 2面板显示导航节点已激活，地图及机器人出现。
2. 使用顶部`Nav2 Goal`工具，在地图白色可通行区域按下并拖动，松开时确定目标朝向。
3. 小车会规划路径并运动；绿色路径与局部代价地图可用于观察绕障。
4. 通过Navigation 2面板取消当前任务。

当前场景坐标以米为单位，范围为x/y各[-6,6]：

- 出生点：(-4,-4)，朝向0。
- 中央障碍：中心(0,0)，尺寸2×3米。
- 北侧工装：中心(-3.8,1.5)，尺寸0.8×0.8米。
- 雷达可见但不在先验地图中的箱体：中心(2.5,-2.4)，尺寸0.8×0.8米。
- 可测试目标：(3,3)及(-3,3)。

灰/黑色障碍及其膨胀区不能作为目标；机器人是约0.75×1.25米的矩形，并非零尺寸点。

## 3. 从另一个终端操作

启动文件中的domain设置只作用于其子进程，不会修改其他终端。使用CLI前先执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=agv_navigation_sim
```

发送单点目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

只读检查基础输入：

```bash
/usr/bin/python3 tools/check_agv_navigation_inputs.py --timeout 8
```

查看关键接口：

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo map base_link
ros2 action list -t
```

手动速度测试应在没有导航任务时进行；不要让手动速度与导航目标同时控制底盘。正常导航命令链为：

```text
controller_server / behavior_server
    → /cmd_vel_nav
    → velocity_smoother
    → /cmd_vel_smoothed
    → collision_monitor
    → /cmd_vel
    → 仿真适配节点（限速及0.4秒输入超时归零）
    → /sim/cmd_vel
    → ros_gz_bridge
    → Gazebo VelocityControl
```

## 4. 模型和数据链路

- 外观：复用`agv_description/meshes/base_link.STL`。
- 碰撞体：0.75×1.25×0.76米包围盒，与现有底盘网格外廓匹配；导航footprint在XY方向各增加5厘米余量。
- 运动：直接控制Gazebo模型的机体坐标系vx、vy、w；模型关闭重力，不模拟轮胎接地、舵轮转向延迟、打滑或真实质量响应。
- 雷达：720条射线，10Hz，360°，安装高度0.9米，量程0.08至12米。
- 里程计：Gazebo模型实际位姿派生的理想里程计，50Hz，非速度命令积分的伪反馈。
- TF：Gazebo提供`odom → base_link`，URDF提供`base_link → laser_link`，AMCL提供`map → odom`。
- 定位：AMCL使用OmniMotionModel，根据激光与地图定位；没有用固定map→odom变换替代定位。
- 全局规划：NavFn A*；局部规划：允许正负vy的DWB；速度平滑和碰撞监测保留在命令链路中。

场景、地图、模型及运行参数由同一份配置生成到`/tmp/agv-navigation-sim-*/`，启动日志会打印路径。包含`world.sdf`、`robot.sdf`、`robot.urdf`、`map.pgm`、`map.yaml`和`nav2.yaml`，便于核对实际生效配置。

先验地图只包含墙和已知障碍，箱体通过激光进入代价地图。此区别用于验证真实传感器链路，不靠把所有障碍预先画进地图来代替避障。

## 5. 修改位置

| 文件 | 用途 |
|---|---|
| `src/agv_bringup/launch/agv_navigation_sim.launch.py` | 整套仿真启动及输入就绪后启动Nav2/RViz |
| `src/agv_bringup/config/agv_navigation_scene.yaml` | 出生点、车体外廓、雷达高度、已知/未知障碍布局 |
| `src/agv_bringup/config/agv_nav2_sim.yaml` | AMCL、DWB、代价地图、速度平滑与碰撞监测 |
| `src/agv_bringup/config/agv_navigation_sim.rviz` | 导航可视化与目标工具 |
| `src/agv_bringup/agv_bringup_sim/assets.py` | 从统一场景生成模型、世界、地图与参数 |
| `src/agv_bringup/agv_bringup_sim/adapter.py` | 仿真速度输入校验、限速、超时停车及雷达frame规范化 |
| `src/agv_bringup/agv_bringup_sim/ready.py` | 等待时钟、里程计、雷达，避免过早启动导航和第二个GUI |
| `tools/test_agv_navigation_sim.py` | 实际导航与取消任务集成验收 |
| `src/agv_bringup/test/test_simulation_assets.py` | 场景/地图一致性与关键参数回归 |

修改布局后重建agv_bringup并重新启动；世界、地图和参数会一起重新生成。本版没有开放任意外部场景参数，因此不会默认套用未经核对的地图或真实设备配置。

## 6. 自动验收

重新启动一份仿真，保持出生点且不下发其他任务，然后在已设置第3节环境的终端执行：

```bash
/usr/bin/python3 tools/test_agv_navigation_sim.py
```

测试内容：

1. 核验仿真模型、Nav2生命周期和传感器输入。
2. 执行横移并确认朝向保持，验证vy实际生效。
3. 停止发送命令，检查速度最终归零。
4. 核验未知箱体在先验地图中为自由空间、在雷达代价地图中变为障碍。
5. 请求绕障路径并依次导航到(3,3)、(-3,3)，检查Action结果及实际位置/朝向。
6. 下发第三个目标并取消，确认返回Canceled且底盘停止。
7. 用连续实际位姿样本与墙/障碍做矩形相交检查。

默认结果：`/tmp/agv-nav-validation/navigation-result.json`。脚本只有在所有项目通过且正常退出时才算通过；它会移动仿真机器人，因此使用了固定domain和Gazebo分区检查。

快速配置回归：

```bash
ctest --test-dir build/agv_bringup --output-on-failure
```

### 联调中修复的配置问题

- 本机Nav2版本的每个激光观测源`max_obstacle_height`默认0；对0.9米高的雷达，必须显式设为2.0，否则障碍层会丢弃雷达点，防碰撞层却仍能看到障碍并制动。
- DWB的`xy_goal_tolerance`默认0.25米；它必须与到位检查的0.15米一致，否则会出现停止平移后无法满足到位条件。当前两者均为0.15米，停止速度阈值设为0.03。
- Gazebo模型生成、传感器就绪和RViz启动分阶段进行，避免并发初始化时出现世界/反馈尚未准备好就启动后续节点的情况。

## 7. 实机迁移边界

这一版验证导航应用流程、传感器接口、定位、规划、避障、任务结果和取消。它不是舵轮动力学数字孪生。

后续实机接入可以保留Nav2上层接口，通过现有仙工速度API执行vx/vy/w，但还需要确认控制权、真实速度/加速度限制、连续odom来源、雷达及坐标系、通信时效与停车语义。仙工世界定位坐标不能未经核实就直接冒充连续里程计。

本次不修改仙工驱动、机械臂SDK或真实整机启动入口。底盘真实轮系细节可以在简化导航流程跑通后逐步补入。


## 8. 本次实际验收结果（2026-11-03）

默认同时打开Gazebo与RViz的启动方式已实际运行。完整测试脚本正常退出（退出码0），结果如下：

| 项目 | 实际结果 |
|---|---|
| 横移 | 约0.20米，朝向保持 |
| 速度输入中断 | 线速度/角速度最终均为0 |
| 激光 | laser_link，720条射线，可获得障碍返回 |
| 未知箱体 | 先验地图代价值0，雷达更新后的全局代价值99 |
| 规划路径 | 440个点，长度约11.80米，绕开直接连线上的障碍 |
| 第一个目标(3,3) | Action成功；实际世界位置误差约0.095米，朝向误差约0.149弧度 |
| 第二个目标(-3,3) | Action成功；实际世界位置误差约0.179米，朝向误差约0.047弧度 |
| 第三个目标取消 | 返回Canceled，取消得到确认，最终速度为0 |
| 车体/障碍相交检查 | 2231个运动采样点中未发现矩形相交 |
| 快速配置回归 | 3项unittest全部通过 |
| 既有驱动回归 | robot_move 6组功能测试、agv_hardware 4组接入测试通过 |

Nav2的0.15米到位容差作用于AMCL定位估计；测试另外用Gazebo实际世界位置做独立检查，实际位置验收阈值为0.30米。因此不能把配置中的0.15米当作真实世界或实机精度承诺。

运行记录保存在`/tmp/agv-nav-validation/`；最终端到端结果是`navigation-result.json`，成功运行日志为`navigation-test-staged.log`及`launch-staged.log`。后续只调整了模型/地面颜色与RViz视图，已核对地图、Nav2参数和物理定义与成功测试时一致，并重新验证启动和数据链路。

取消时Nav2可能打印follow_path在halt阶段等待结果的错误日志；本次以外层任务返回Canceled、控制器取消确认和实际零速度共同判定取消结果，没有把该日志当成成功证据。

## 9. 自己建图后导航

新增`agv_mapping_sim.launch.py`可使用SLAM Toolbox和仿真雷达从零建图；建图时不启动AMCL或Nav2。步骤见[AGV仿真建图练习](AGV仿真建图练习.md)。

导航入口新增可选`map`参数：

```bash
ros2 launch agv_bringup agv_navigation_sim.launch.py map:="$PWD/maps/practice.yaml"
```

加载外部地图后可先在RViz使用`2D Pose Estimate`指定初始位置和朝向，或传入`initial_pose:="[x, y, yaw]"`自动初始化（地图坐标，米/弧度），再下发目标。不传`map`参数仍使用本说明前面的预生成地图与自动初始定位。
