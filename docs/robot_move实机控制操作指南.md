# robot_move 实机控制操作指南

本文用于通过独立的 `robot_move_node` 控制仙工 Seer AGV 底盘，使用 ROS 2 Jazzy。

## 1. 启动前准备

- 电脑能够访问底盘 IP；下文 `10.42.0.114` 是示例地址，请替换为实际地址。
- 底盘已在自身控制软件中完成定位，并处于允许运动的状态。
- 测试区域空旷，现场急停可用。
- 不要同时运行整机 `agv.launch.xml` 和独立 `robot_move_node`：整机的 `agv_hardware` 插件已经直接连接底盘。

## 2. 编译

在工作区根目录执行：

```bash
cd /home/zyj/Code/ROS2/AGV_new/AGV_ROS2_Control_Manager
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select robot_move
source install/setup.bash
```

如果工作区位于其他目录，请相应修改 `cd` 路径。

## 3. 连接实机

先检查网络连通性：

```bash
ping -c 3 10.42.0.114
```

启动底盘节点：

```bash
ros2 run robot_move robot_move_node --ros-args \
  -p robot_type:=agv \
  -p connection:=192.168.1.109 \
  -p agv_max_linear_speed:=0.1 \
  -p agv_max_angular_speed:=0.2 \
  -p agv_init_set_do:=false \
  -p agv_init_relocate:=false
```
agv_init_relocate 重定位
这组参数设置合成线速度上限为 **0.1 m/s**、角速度绝对值上限为 **0.2 rad/s**，并关闭启动时的 DO 输出和重定位。超过上限的速度指令会被拒绝，不会自动截断。

节点需要连接底盘 TCP 端口 `19204`、`19205`、`19206`、`19210`。`ping` 成功只说明网络可达，不代表这些端口全部可用。看到 `硬件初始化成功!` 后，再进行后续操作。

### 初始化动作说明

如果不显式关闭初始化动作，默认会先设置 **DO0 高电平**，再按查询到的当前位置执行重定位。DO 的物理用途由现场接线决定，是否开启这些动作应根据接线和定位流程确定。

上面的启动命令关闭了两项初始化动作，但节点仍会接收运动命令、定时查询位姿，并在退出时尝试发送零速度，因此不是全程只读模式。

## 4. 查看底盘位姿

另开一个终端，加载环境：

```bash
cd /home/zyj/Code/ROS2/AGV_new/AGV_ROS2_Control_Manager
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo /agv_pose
```

位姿消息类型为 `geometry_msgs/msg/Pose2D`，包含 `x`、`y`、`theta`。

后续控制命令也需要在已加载上述 ROS 和工作区环境的终端中执行。

## 5. 低速运动测试

沿底盘 x 正方向，以 **0.05 m/s** 持续发布速度：

```bash
ros2 topic pub --once  /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0}, angular: {z: 0.0}}"
```

`linear.x`、`linear.y` 的单位为 m/s，`angular.z` 的单位为 rad/s。首次测试应确认实际运动方向与预期一致。

## 6. 停车与退出

**先在速度发布终端按 Ctrl+C，结束非零速度发布，再明确发送零速度：**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"
```

确认底盘停止后，再在运行 `robot_move_node` 的终端按 Ctrl+C 退出。

当前驱动没有命令超时自动停车机制，不能仅靠停止话题发布来保证停车。节点正常退出时会尝试发送零速度并关闭连接，但网络异常或进程异常退出时不能依赖这一流程；紧急情况使用现场急停。

## 7. 定距移动

保持 `robot_move_node` 运行，另开终端加载环境：

```bash
cd /home/zyj/Code/ROS2/AGV_new/AGV_ROS2_Control_Manager
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果之前仍在持续发布 `cmd_vel`，先在发布终端按 Ctrl+C 结束发布，再发送零速度并确认底盘停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

定距接口为 `/robot_distance_move`，消息类型为 `geometry_msgs/msg/Twist`：

| 字段 | 含义 |
|------|------|
| `linear.z` | 移动距离，单位 m，必须大于 0 |
| `linear.x` | x 方向速度，单位 m/s，正负决定方向 |
| `linear.y` | y 方向速度，单位 m/s，正负决定方向 |

以 **0.05 m/s 沿底盘 x 正方向移动 10 cm**：

```bash
ros2 topic pub --once /robot_distance_move geometry_msgs/msg/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.1}}'
```

沿底盘 x 负方向移动 10 cm，将 `x` 改为负速度，距离仍填正数：

```bash
ros2 topic pub --once /robot_distance_move geometry_msgs/msg/Twist \
  '{linear: {x: -0.05, y: 0.0, z: 0.1}}'
```

以上两条是独立示例，等待前一个任务完成后再发送下一条。速度不能同时为零，并且仍受启动时配置的速度上限约束。

驱动将定距请求交给底盘控制器执行，并查询任务状态；完成时，节点终端输出 `导航完成`。话题发布成功只代表消息已发送，不代表实机任务执行成功。

**定距命令使用 `--once`，不要循环发布**，否则前一个任务完成后可能再次触发移动。

**定距任务执行期间，`cmd_vel`（包括零速度）可能因驱动锁被占用而被拒绝，不能用它可靠打断任务。紧急情况使用现场急停。**

## 8. 定角旋转

保持 `robot_move_node` 运行，在已加载 ROS 和工作区环境的终端中发送命令。测试前结束持续发布的 `cmd_vel`，发送零速度并确认底盘停止；具体操作见第 7 节。

旋转接口为 `/robot_rotate_move`，消息类型为 `geometry_msgs/msg/Twist`：

| 字段 | 含义 |
|------|------|
| `angular.z` | 旋转角度，单位 rad，必须大于 0 |
| `angular.y` | 角速度，单位 rad/s，正负决定旋转方向，不能为零 |

以 **0.1 rad/s 的正角速度旋转 10°**：

```bash
ros2 topic pub --once /robot_rotate_move geometry_msgs/msg/Twist \
  '{angular: {y: 0.1, z: 0.174533}}'
```

反方向旋转 10°，将角速度改为负数，角度仍填正数：

```bash
ros2 topic pub --once /robot_rotate_move geometry_msgs/msg/Twist \
  '{angular: {y: -0.1, z: 0.174533}}'
```

以上两条是独立示例，每次只发送一条，等待节点输出 `导航完成` 后再发送下一条。首次测试应确认实际旋转方向与预期一致。角速度仍受启动时设置的角速度上限约束。

角度换算公式为 `弧度 = 角度 × π / 180`：

| 角度 | 弧度（约） |
|------|------------|
| 10° | 0.174533 |
| 45° | 0.785398 |
| 90° | 1.570796 |
| 180° | 3.141593 |

**旋转命令使用 `--once`，不要循环发布**，否则任务完成后可能再次触发旋转。话题发布成功不代表实机任务执行成功，应查看节点的任务完成或失败日志。

**旋转任务执行期间，`cmd_vel`（包括零速度）可能因驱动锁被占用而被拒绝，不能用它可靠打断任务。紧急情况使用现场急停。**

## 9. 当前控制限制

- 节点只允许一个长任务执行；忙时新的长任务会被拒绝，不排队。
- 定距移动、定角旋转等长任务会占用驱动锁，此时 `cmd_vel` 可能被拒绝。因此零速度话题不能作为这些任务的可靠急停。
- 本地取消等待和关闭连接不等于取消机器人端导航任务，不能据此认定实机已经停车。

更多启动参数、定距/定角消息字段和库接口见 [robot_move 包说明](../src/robot_move/README.md)。整机接入方式见 [agv_hardware 说明](../src/agv_hardware/README.md)。
