# 仙工雷达与里程计 ROS2 接口核对

2026-11-11 实现进展：独立雷达采集已实现；1005 实际速度积分的连续估计里程计、TF和诊断现已实现并完成两组各60秒只读采集，运动精度待验收。当前实现、分层、启动和测试限制见 [连续里程计接入](连续里程计接入.md)。以下为原接口核对时的结论及设计背景。

核对日期：2026-11-05。依据本仓库 `docs/agv_docs/seer_api/` 保存的仙工 API 文档和现有代码，尚未连接实机验证。

## 结论

仙工 TCP API 提供激光点云查询，可以开发 ROS2 适配节点输出 `/scan`。它不是现成的 ROS2 雷达驱动，仍需处理角度、时间、安装坐标和无效数据。

普通状态查询中找到了实际速度反馈、编码器反馈和世界坐标定位位姿；本次未找到明确提供连续局部里程计位姿的独立查询接口。不能把 `1004` 的定位结果直接改名为 `/odom`。可先验证 `1005` 实际速度反馈，通过积分形成局部里程计，后续优先换成控制器原生连续里程计或经标定的编码器里程计。

## 查到的 API

以下普通状态查询使用 TCP `19204`。编号是请求/响应编号，不是端口。

| 用途 | 请求 / 响应 | 关键内容 | ROS2 对应关系 |
| --- | --- | --- | --- |
| 激光点云 | `1009 / 11009` | `lasers[].beams`、`device_info`、`header`、`install_info` | `/scan`，`sensor_msgs/msg/LaserScan` |
| 实际速度 | `1005 / 11005` | `vx/vy` 为机器人坐标系实际速度，m/s；`w` 为实际角速度，rad/s，逆时针为正 | `/odom.twist`，并可积分估计 `/odom.pose` |
| 世界坐标定位 | `1004 / 11004` | `x/y/angle`，单位 m、rad；`confidence`、`loc_method` | 定位来源，适合经坐标对齐后用于 `map` 系位姿 |
| 编码器 | `1018 / 11018` | `motor_encoder` 包含电机名、脉冲值；旧 `encoder` 已弃用 | 结合车型、轮径、减速比、编码器分辨率和舵角计算里程计 |

原始文档：

- [查询机器人激光点云数据](agv_docs/seer_api/查询机器人激光点云数据_-_Feishu_Docs_7abb40aeac.md)
- [查询机器人速度](agv_docs/seer_api/查询机器人速度_-_Feishu_Docs_472d693af9.md)
- [查询机器人位置](agv_docs/seer_api/查询机器人位置_-_Feishu_Docs_4ea523b587.md)
- [查询编码器脉冲值](agv_docs/seer_api/查询编码器脉冲值_-_Feishu_Docs_1f36ef8d1e.md)

`1009` 文档给出零正文请求示例，也支持可选 `return_beams3D`，默认不返回多线激光数据。二维接入先用零正文请求：

```text
5A 01 00 01 00 00 00 00 03 F1 00 00 00 00 00 00
```

这是一个 TCP 协议帧，不能把它作为 JSON 文本发送。响应需要按 16 字节头中的正文长度完整接收，不能假设一次 `recv()` 得到整包。

## 激光点云到 LaserScan

| 仙工字段 | ROS2 字段 / 处理 |
| --- | --- |
| `beams[].dist` | `ranges[]`；距离单位与量程在实机复核后按 m 输出 |
| `beams[].angle` | 文档示例为 `-90/-89.5/-89`，按度数解释并转 rad，实机复核方向 |
| `beams[].rssi` | `intensities[]`；不存在时允许整个强度数组为空 |
| `device_info.min_range/max_range` | `range_min/range_max` |
| `device_info.pub_step`、各束角度 | 核对角度间隔，生成 `angle_min/angle_max/angle_increment` |
| `device_info.scan_freq` | 核对含义和单位后用于估计 `scan_time`，不能视作 TCP 实际返回帧率 |
| `device_info.time_increment` | 确认单位和有效性后映射；未知则不能伪造逐束采样时间 |
| `header.data_nsec` | 候选采集时间，需确认时钟基准及是否对应第一束 |
| `header.pub_nsec`、顶层 `create_on` | 发布/API 上传时间，不能未经确认替代采集时间 |
| `header.frame_id` | 配置映射为 ROS TF 中的雷达坐标系，如 `laser_link`；去除前导 `/` |
| `install_info.x/y/z/yaw/upside` | 用于核对 `base_link → laser_link` 安装外参；确认角度单位、倒装含义及参考点 |

文档示例不足以确定所有单位、数据是否已做安装补偿及去畸变方式，需要抓实机样本核对。ROS2 的 LaserScan 要求角度为 rad、距离为 m，时间戳表示第一束采样时间，零角沿雷达坐标系 +X，绕 +Z 旋转。

适配时必须保留固定角度栅格。`valid=false`、缺束、虚拟点不能通过删除元素压缩数组；不能把虚拟点作为真实测距。未知/异常测量可用 NaN 表示，只有确认是有效的“量程内无回波”时才按消费者约定使用正无穷，避免错误清除障碍。`is_obstacle=false` 不等于无效测量。

若数据角度不等间距，应明确重采样策略或发布 `PointCloud2`，不能直接套用均匀 LaserScan。倒序扫描若改成角度升序，还需同步处理各束时间顺序；不能仅排序距离。多雷达先按设备名选择或分别发布，不能直接拼接数组。

ROS2 定义：[LaserScan.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg)。

## 连续里程计方案

`1005` 的 `vx/vy/w` 是实际反馈；`r_vx/r_vy/r_w` 是机器人接收到的速度指令，不能用于替代反馈。也不能用本机 `/cmd_vel` 积分冒充实测里程计。

最低可行方案是对验证过的实际速度做平面积分，保留全向底盘的 `vy`：

```text
dx/dt = vx*cos(theta) - vy*sin(theta)
dy/dt = vx*sin(theta) + vy*cos(theta)
dtheta/dt = w
```

输出 `/odom`（`nav_msgs/msg/Odometry`）：`header.frame_id=odom`，`child_frame_id=base_link`；pose 位于 odom 系，twist 位于 base_link 系。yaw 转为单位四元数，同时广播同时间戳、同位姿的动态 TF `odom → base_link`。若仙工机器人坐标原点与 base_link 不同，需转换位姿和速度，不能只改 frame 名。

速度积分属于估计里程计，会累积漂移，精度依赖反馈更新率和网络延迟。积分间隔应使用已验证的采样时间，或明确采用本机单调时钟的接收间隔近似；断线、缺字段、时间倒退、超长间隔时停止积分并报告失效，不跨丢包时段外推。恢复连接后不得在同一个 odom 系中无声重置原点。协方差需要根据误差估计，不能用全零表示完美准确。

`1004` 文档明确描述世界坐标位姿，且有不同定位模式，包括 `loc_method=3` 里程计模式。这个枚举不证明普通定位输出始终连续，也不是独立里程计查询。定位校正、重定位可能跳变，与 ROS 的 odom 连续性要求不同。

如果采用仙工定位，经确认仙工地图和 ROS map 的原点、方向与底盘参考点一致或完成对齐后，可计算：

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

此时由仙工定位适配器发布 `map → odom`。若使用 AMCL 或 slam_toolbox，则由对应定位/建图节点发布这条 TF。每条 TF 只能有一个发布者。

ROS2 定义：[Odometry.msg](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg)；坐标约定：[REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。

## 推送及 3D 建图接口的边界

- [配置机器人推送端口](agv_docs/seer_api/配置机器人推送端口_-_Feishu_Docs_e9a6692add.md)：推送端口 `19301`，配置请求 `9300`，配置响应 `19300`，可设置 `interval`（ms）、`included_fields` 或 `excluded_fields`；包含与排除不能同时设置。
- [机器人推送](agv_docs/seer_api/机器人推送_-_Feishu_Docs_837d91181d.md)：主动推送报文编号 `9301`，数据格式参考批量数据 1。是否支持推送完整 lasers、可达更新频率和带宽需要实机确认，不能仅凭可配置 interval 宣称达到某一频率。
- [实时获取 3D 扫图数据](agv_docs/seer_api/实时获取_3D_扫图数据_-_Feishu_Docs_22bf4f4272.md)：含 `Message_MapOdo` 的 `timestamp/odo_x/odo_y/odo_w/odo_vx/odo_vy/odo_vw`，但流程是启动 3D 扫图后用 `1780` 获取压缩文件列表、`1800` 下载 rawmap。它证明建图日志中存在里程计数据，不能据此认定存在可直接用于实时控制的在线里程计流。

## 本项目的接入位置

目前 `src/robot_move` 的 Seer Client 封装了 `1004` 位姿查询和运动控制，尚未封装 `1009/1005/1018`。`UniversalRobotDriver` 发布的是 `agv_pose`（Pose2D），不满足 `/odom` 消息与 TF 契约。`scan_config/scan_ctrl` 是扫查机构业务接口，不是激光 `/scan`。

建议新增独立的 `seer_sensor_bridge` 节点，使用专门的状态连接获取雷达和速度数据，发布 `/scan`、`/odom` 和所需 TF。现有 Client 使用全局互斥锁串行请求，雷达大包若共用它会占用运动指令处理时间；需要将传感器请求与运动控制的执行资源隔离，并实测控制器允许的连接数。

目标 TF 链为 `map → odom → base_link → laser_link`。雷达外参通过实测 URDF 固定关节或 static_transform_publisher 发布，不能照抄 Gazebo 中的安装高度。

实机所有相关节点使用 `use_sim_time=false`。现有 `agv_navigation.launch.py` 不创建底盘或传感器驱动，默认 `use_sim_time=true`，启动实机导航时需要显式覆盖，并提供与实际几何、量程匹配的 Nav2 参数文件。

## 实机验证顺序

1. 分别抓取 `1009/1005/1004/1018` 响应，记录固件版本、设备名、完整字段、时间戳、实际帧率与延迟；普通查询均先按各自文档发送零正文请求。
2. 雷达静止时检查量程、角度方向、倒装、外参与障碍实际位置；运动时检查采样时间和 TF 的匹配情况。
3. 验证前进、横移、旋转时速度反馈符号；确认是实际测量及其更新频率，再评估速度积分或编码器方案。
4. 配齐定位和 TF 后，用已有只读工具检查基本输入：

```bash
source install/setup.bash
python3 tools/check_agv_navigation_inputs.py --real-time --timeout 10
```

该工具同时检查 `map → odom`，因此只启动雷达和里程计、尚未启动定位时不会全部通过。通过只表示输入链路基本存在，不代表里程计精度、时间同步或导航效果通过验收。

本文为接口核对及接入设计；尚未实现 ROS2 传感器桥接节点，也未验证实机固件对上述 API 的支持情况。
