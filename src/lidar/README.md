# lidar

仙工双雷达的独立只读采集包。通过独立 TCP 19204 连接发送零正文 `1009` 请求，发布前后两台雷达的 LaserScan；不会初始化底盘、设置 DO、重定位或发送运动指令。

## 构建和启动

在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_move lidar agv_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch lidar seer_lidar.launch.py connection:=实际底盘IP
```

显式选择系统 Python 是为了避免当前机器的 Anaconda Python 缺少 ROS 构建依赖；使用正常 ROS Python 环境时可省略对应 CMake 参数。

独立 launch 支持 `connection`、`params_file` 和 `use_sim_time` 参数。默认读取 `config/seer_lidar.yaml`，实机 `use_sim_time=false`。地址必须由启动参数提供。

整机启动入口增加 `enable_lidar`（默认 false）和 `lidar_params_file`，示例：

```bash
ros2 launch agv_bringup agv.launch.xml \
  chassis_ip:=实际底盘IP enable_lidar:=true \
  agv_init_set_do:=false agv_init_relocate:=false
```

整机命令会同时启动已有底盘、机械臂等流程；初始化开关只控制已有底盘初始化动作。仅检查雷达时使用前面的独立雷达 launch。

## 代码职责

| 文件 | 职责 |
| --- | --- |
| `interface/lidar_source.hpp` | 同步连接/读取/关闭接口，读取使用绝对截止时间 |
| `data/lidar_types.hpp` | 不依赖 ROS 的扫描、可选测量字段、原始时间戳和错误类型 |
| `impl/seer/seer_lidar_source.*` | API 1009、JSON 校验、字段和角度单位解析 |
| `processing/laser_scan_builder.*` | 按真实角度映射均匀角度栅格、过滤无效数据 |
| `nodes/lidar_node.*` | 采集线程、退避重连、设备去重、状态和 ROS 发布 |

当前复用 `robot_move::robot_move_impl` 中的 Transport 和 cJSON。运动、雷达分别创建 Transport 实例；雷达仅连接状态端口，不共用运动请求锁。数据源没有内部线程，节点唯一工作线程负责所有网络操作。

## 配置与话题

| 参数 | 默认值 / 含义 |
| --- | --- |
| `connection` | 必须指定控制器 IPv4 地址 |
| `request_rate_hz` | 5.0，请求目标频率；允许 (0, 100] |
| `request_timeout_ms` | 1000，允许 1..1000 |
| `stale_timeout_sec` | 1.0，以本机单调时钟判断每设备数据新鲜度 |
| `stamp_mode` | 仅支持 `receive_time` |
| `devices` | `[laser_front, laser_back]` |
| `device.<名称>.enabled` | true；至少启用一个设备 |
| `device.<名称>.topic` | `/scan_front` 或 `/scan_back` |
| `device.<名称>.frame_id` | `laser_front_link` 或 `laser_back_link` |
| `device.<名称>.angle_min_deg` | -135.0，输出角域起点，不是设备数组起点 |
| `device.<名称>.angle_max_deg` | 135.0，输出角域终点 |
| `device.<名称>.angle_increment_deg` | 0.5，角域长度必须为该值的整数倍 |

以上采集参数在启动时设置且只读。实际设备名由返回的 `device_info.device_name` 匹配，不能按数组顺序猜前后雷达。配置中的输出范围刻意比当前元数据略宽，避免使用元数据与数组序号的错误对应关系；超出输出范围的点单独统计，按实际数据调整配置。

- `/scan_front`、`/scan_back`：`sensor_msgs/msg/LaserScan`，传感器数据 QoS，有限队列。
- `/diagnostics`：采集状态、请求耗时、错误/连接次数，以及每设备有效点、丢弃点、重复帧、时钟回退、累计平均帧率等。
- `radar_heartbeat`：`std_msgs/msg/Header`，所有启用雷达均收到新的、结构有效且仍新鲜的帧后发布。重复数据不更新心跳。

全无效测量会触发 WARN，但与设备断线分开；无设备、缺时间戳、空 beams 或数据过期会触发 ERROR。累计平均帧率不是设备硬件扫描频率，也不是瞬时频率。

## 数据和时钟约定

- 只有 `valid=true` 且角度/距离完整、有限、位于量程内的非虚拟点能成为有效测量。
- 按每束实际角度（仙工度数转为 rad）重采样；同格取最近距离及其对应强度。
- 未知距离填 NaN，不插值，不用最大量程或正无穷冒充无遮挡。缺失强度也用 NaN 表达。
- 默认输出 541 个角度格子，与原始 513 个数组元素数量可以不同；这是角度栅格转换，不是补造 28 个测距点。
- `header.stamp` 是本机收到 API 响应后记录的 ROS 时间，包含网络和请求延迟，不是已同步的首束采样时间。
- 设备 `data_nsec` 以 uint64 原样解析，每设备独立去重；不当作 Unix 时间。缺失/非法时间戳不发布新扫描。
- 检测到设备时间回退后记录时钟重置并接受新会话。诊断保留该记录，节点重启后清零。
- `scan_time` 来自有效的设备标称 `scan_freq`，`time_increment=0`；没有运动去畸变。
- 慢请求跳过错过的周期，不累积待发请求。连接失败或通信错误后按 0.5～5 s 退避重连；完整 API 错误退避重试，保留连接。
- 停止节点会唤醒等待并 join 工作线程。网络操作最多受 1 s 的连接/请求时限约束，另加有界解析与调度时间。

## TF 与导航限制

采集节点本身不发布 TF。`agv_description` 通过共用外参 YAML 和 Xacro 固定关节提供 `base_link → laser_front_link`、`base_link → laser_back_link`。整机默认关闭，可分别通过 `front_lidar_tf_owner`、`back_lidar_tf_owner` 选择 description / standalone / off；每条 TF 只有一个发布者。独立采集 launch 不自动启动 TF。

当前前后雷达 yaw 分别为 -135°、54°，roll/pitch 均为 0°；位置分别为 (-0.295,-0.550,-0.240) m 和 (0.295,0.550,-0.240) m。两路真实扫描对齐已获现场确认；高度尚未独立实测，选取墙段残差约 5 cm，不称为高精度外参标定完成。配置、启动及验证依据见 [雷达安装 TF](../../docs/前雷达安装TF.md)。

设备返回的安装 x/y/z/yaw/upside 仍仅用于诊断核对，不自动应用。不要仅根据 upside 设置 roll，也不要在扫描和 TF 中重复旋转。在 RViz 中将 Fixed Frame 设为 base_link，对照现场物体检查两路扫描；本包不提供 `/odom`、双雷达 `/scan` 融合或导航启动。

## 验证

```bash
colcon test --packages-select robot_move lidar agv_bringup --event-handlers console_direct+
colcon test-result --verbose
ros2 topic info /scan_front --verbose
ros2 topic info /scan_back --verbose
ros2 topic hz /scan_front
ros2 topic hz /scan_back
ros2 topic echo /diagnostics
```

自动测试使用本地套接字服务和注入数据源，不连接实机；lidar 测试使用 ROS_DOMAIN_ID=217 隔离现场 ROS 图。Transport 和 Source 的连接测试需要回环地址 19204 端口未被其他程序占用。这些测试按 CTest 默认串行运行；不要并行运行不同包中争用该端口的连接测试。

测试覆盖部分 TCP 帧、真实样本解析、超过 double 精确整数范围的时间戳、API 错误后的连续读取、错误响应、超时、缺字段、角度转换、无效距离、同格冲突、设备独立去重、掉线恢复、时钟重置、慢 I/O 不阻塞 ROS 回调及正常退出。

实施和验收状态见 [仙工雷达采集接入实施计划](../../docs/仙工雷达采集接入实施计划.md)；测试和现场限制在该计划中记录。
