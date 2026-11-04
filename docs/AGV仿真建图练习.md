# AGV仿真建图练习

本练习使用现有Gazebo全向底盘、激光雷达和里程计，新增SLAM Toolbox在线建图。先手动探索并保存地图，再关闭建图，加载自己保存的地图执行Nav2导航。两个模式使用同一个场景和出生点。

## 1. 编译与启动建图

所有终端先进入本工作区根目录。在第一个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select agv_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch agv_bringup agv_mapping_sim.launch.py
```

会打开Gazebo和建图专用RViz。等待地图和激光出现。开始时只能看到出生点附近雷达扫描到的部分，走动后地图才逐渐补全。SLAM运行时不启动AMCL、已有地图服务器或Nav2控制器。

只保留RViz窗口可用`gui:=false`；完全不显示窗口可再加`rviz:=false`，GPU雷达仍要求有效图形环境。同一时间只运行一份建图或导航仿真；切换模式前先在启动终端Ctrl+C并等待所有子进程退出。

## 2. 键盘开车

第二个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=agv_navigation_sim
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.2 -p turn:=0.4 -p stamped:=false
```

保持键盘焦点在这个终端，而非Gazebo或RViz窗口。

| 按键 | 动作 |
|---|---|
| `i` / `,` | 前进 / 后退 |
| `j` / `l` | 原地左转 / 右转 |
| 大写`J` / 大写`L` | 向左 / 向右横移（Shift+j / Shift+l） |
| `k` | 停止 |
| Ctrl+C | 停止并退出键盘节点 |

本机的键盘节点在按键事件发生时发送速度，按住方向键依靠系统按键重复持续发送。仿真适配器在0.4秒未收到新指令时归零，因此单击只会短暂运动；长按的首次重复延迟较大时可能短暂停顿。不要用未实现的`repeat_rate`参数假设它会自动持续发送。

建图时键盘直接控制`/cmd_vel`，没有Nav2的局部避障和防碰撞节点，需人工避开墙和箱子；车体有实际尺寸。建议低速沿外围走一圈，再绕中央障碍补全遮挡区，最后回到起点附近。观察墙是否连续、是否出现重影，以及地图是否在移动后更新。

## 3. 保存扫描得到的地图

先按`k`停车，保持建图启动终端运行。第三个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=agv_navigation_sim
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/practice" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
```

成功后得到`maps/practice.yaml`和`maps/practice.pgm`。这里保存的是SLAM根据`/scan`生成的地图，不是程序预先生成的场景地图；因此场景中原来未标注的箱子也可能出现在保存的地图中。再次使用相同文件名前缀会覆盖地图；要保留旧图时使用新名字。

PGM记录占用栅格，YAML记录图像路径、原点和分辨率。移动地图时应一起移动这两个文件。它们用于地图导航，不包含继续建图所需的SLAM位姿图；需要续建时另行使用SLAM Toolbox的位姿图序列化功能。

## 4. 使用自己的地图导航

先退出键盘节点，再在第一个终端Ctrl+C关闭整套建图。确认进程结束后，重新执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agv_bringup agv_navigation_sim.launch.py \
  map:="$PWD/maps/practice.yaml"
```

本次场景仍从原出生点重新开始。加载外部地图时，AMCL不会套用预生成地图中的出生坐标，而是等待你设置初始位姿：

1. 等待RViz显示保存的地图。
2. 选择顶部`2D Pose Estimate`，在地图上点出机器人当前所在位置并拖出朝向。对于从同一出生点开始的新地图，应选建图开始时的位置。
3. 等待激光点与墙体基本重合，机器人模型出现。
4. 等待Navigation 2面板显示导航已激活，再用`Nav2 Goal`在已探索的空闲区域下发目标，观察规划和运动。

导航生命周期管理器会等待初始定位变换出现后再启动，不会要求你在固定秒数内完成操作。在初始位置尚未设置时，AMCL提示需要初始位姿是正常现象。

扫描地图坐标不保证等于Gazebo世界坐标；不要机械地把出生点的世界坐标(-4,-4)填入任意扫描地图。初始定位设置错误时先重新设置，不要立即下发远距离目标。

### 可选：自动设置初始位姿

如果已知机器人在这张地图中的出生位置，可以直接传入`initial_pose`，无需点击`2D Pose Estimate`：

```bash
ros2 launch agv_bringup agv_navigation_sim.launch.py \
  map:="$PWD/maps/practice.yaml" \
  initial_pose:="[-4.0, -4.0, 0.0]"
```

数组依次为地图坐标系中的`x`（米）、`y`（米）、`yaw`（弧度）；0弧度表示朝地图X正方向，约1.5708弧度表示朝Y正方向。参数只初始化AMCL，不改变Gazebo出生位置。上面的数值适用于本场景经核对的地图起点，不是任意地图的通用坐标。

传参后AMCL自动初始化，定位变换就绪后导航自动激活。不传参数或传空字符串，加载外部地图时仍等待手动定位；使用预生成地图时仍采用场景默认位姿。也可以为预生成地图显式指定`initial_pose`覆盖默认定位。错误长度、非数字或非有限数会在启动Gazebo之前报错；建图模式不能使用此参数。

若激光与墙未对齐，仍可用`2D Pose Estimate`纠正，并把确认后的正确位姿用于下次启动。自动初始化并不自动推算地图与世界之间的坐标关系，也不保证任意初始猜测都能定位成功。

不传`map:=`时，原有自动生成地图的导航方式保持可用，并继续自动设置该场景的初始定位：

```bash
ros2 launch agv_bringup agv_navigation_sim.launch.py
```

## 5. 数据链路与排查

```text
Gazebo激光 /scan + /odom + odom→base_link→laser_link
    → SLAM Toolbox
    → /map + map→odom
    → RViz / 地图保存工具
```

地图刚启动时为空一小段时间是正常的。若长时间不出现，可在设置了上述domain的终端检查：

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 node list
```

SLAM应为`active`；建图模式不应存在`/amcl`、`/map_server`或`/controller_server`。如果出现，先关闭另一套仿真，避免重复发布坐标变换。建图时看不到Nav2 Goal和Navigation 2面板是预期行为。

## 6. 自动验收

在全新出生点启动建图模式，关闭其他键盘控制后，可运行：

```bash
# 先按第二节设置环境和domain
/usr/bin/python3 tools/test_agv_mapping_sim.py \
  --output-directory /tmp/agv-mapping-practice-test
```

此脚本会实际移动仿真机器人，沿外围六个路点采集雷达，检查地图增长、占用/空闲栅格、车体与障碍相交、建图/导航节点互斥，并调用地图保存工具。默认输出目录为`/tmp/agv-mapping-validation`。已有同名地图时脚本拒绝覆盖，请换一个输出目录。它不是自动探索算法，仅用于当前固定场景的回归验收。


## 7. 本次实际验收（2026-11-03）

- SLAM在线更新116次，已知栅格由5369增长至53525；其中占用1375、空闲52150。
- 外围六个路点完成，车体与场景墙体/障碍相交检查未发现碰撞。
- 成功保存238×238、0.05米分辨率的PGM/YAML扫描地图。
- 重新启动导航后，确认地图服务器加载的是该扫描地图，AMCL自动初始位姿关闭，且没有SLAM节点。
- 设置初始位姿之前，定位等待节点运行，导航生命周期管理器尚未启动；设置后自动激活导航。
- 用等价于RViz的`/initialpose`消息初始化定位，再导航到世界位置(3,-4)，Action成功，实际位置误差约0.121米；随后连续10个里程计采样确认线速度/角速度均为0。
- 快速配置回归包含4项unittest，全部通过。

验收记录与练习地图在`/tmp/agv-mapping-validation/`，其中`mapping-result.json`和`saved-map-navigation-result.json`记录两阶段结果。临时目录不保证永久保存；正式练习应按上文把地图保存在工作区`maps/`或其他持久目录。实际验收使用无窗口模式运行同一仿真数据链，未连接真实设备。
