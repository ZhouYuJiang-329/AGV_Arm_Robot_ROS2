# 在RViz中使用实景地图计算路径

本入口只运行地图服务器、NavFn A*全局规划器及起终点交互节点。输入地图、起点和终点，输出`nav_msgs/Path`。不需要Gazebo、真实车辆、激光或里程计，不发送速度指令。

## 启动

工作区根目录：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select agv_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch agv_bringup agv_static_planning.launch.py \
  map:="$PWD/maps/seer/20251120141209997_navigable.yaml"
```

默认ROS domain为188，与Gazebo导航默认187隔离。地图路径是必填参数，可替换为其他ROS地图YAML。`rviz:=false`可只启动计算节点。

## 操作

1. 等待地图显示、终端打印`Managed nodes are active`。
2. 点击顶部`2D Pose Estimate`，在白色区域点击并拖动，设置规划起点；绿色箭头标记起点。这次按钮只提供起点，不是AMCL重定位。
3. 点击顶部`2D Goal Pose`，在白色区域点击并拖动，设置终点；红色箭头标记终点。
4. 绿色线显示计算出的路径。重新选起点或终点会清除旧路径并重新计算。也可以先选终点、再选起点。

该入口没有Navigation 2执行面板，不使用`Nav2 Goal`或`NavigateToPose`。没有小车沿路径运动是正常的；本功能用于先观察算法如何选路。

起点和终点要选在远离障碍的空闲位置。灰色未知区域、黑色障碍以及膨胀后的不可通行区域不能作为有效起终点。终端提示`No path`时检查这些位置；旧路径会清除，不会把上一次成功结果当成新结果。

## 算法与限制

配置在`src/agv_bringup/config/agv_static_planning.yaml`。当前使用NavFn，`use_astar: true`，不穿越未知区域。代价地图只包含静态地图和障碍膨胀，没有实时传感器障碍层。

此处使用`robot_base_frame: map`，因为完整静态代价地图不需要跟随机器人滚动；计算请求始终`use_start: true`，起点由用户提供，所以不需要伪造机器人定位TF。

NavFn主要在二维栅格中搜索，配合膨胀代价使路径避开障碍；它不保证矩形底盘或外伸机械臂在所有朝向下都能通过，也不模拟舵轮转向过程。地图中的白色区域依据用户确认的“有效范围内空白可通行”规则生成，计算结果反映该假设。

## 接口

其他终端查看接口前，加载ROS环境并设置：

```bash
export ROS_DOMAIN_ID=188
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

- `/initialpose`：`geometry_msgs/PoseWithCovarianceStamped`，规划起点，frame为`map`。
- `/goal_pose`：`geometry_msgs/PoseStamped`，规划终点，frame为`map`。
- `/compute_path_to_pose`：Nav2计算路径Action，可直接调用并明确传入start、goal和`use_start: true`。
- `/plan`：显示用的最新规划路径，支持Transient Local，规划失败或位置改变时清空。
- `/planning_start`、`/planning_goal`：起终点箭头显示。

## 实际验证

使用转换后的仙工地图，在无Gazebo、AMCL、机器人TF和控制器的条件下，通过与RViz相同的话题提供起点(0,3)及终点(12,3)。确认先只发终点不会生成路径，随后提供起点可得到路径，路径两端对应所选位置，且全部路径采样点处于地图空闲栅格中。

本次得到1200个路径点，路径长度约12.26米。把起点改到已知障碍点后，旧路径清除、规划失败；重新提供有效起点后规划恢复。ROS图中没有`/cmd_vel`发布者。RViz已用专用配置实际启动。
