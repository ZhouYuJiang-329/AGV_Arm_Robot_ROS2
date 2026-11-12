# 整机 AGV 驱动接入

`agv_hardware` 链接 `robot_move::robot_move_impl`，持有一个新的 `AGVRobotMove`。

```text
上位机 WebSocket JSON → CommandParser → AgvHardwareInterface
    → AGVRobotMove → seer::Client → seer::Transport → 仙工控制器
```

底盘速度只由插件直接下发，不再同时转发到 `cmd_vel`。BridgeManager 的其他状态/心跳用途保留。
整机控制同一底盘时，请关闭独立 `robot_move_node`；独立节点仍可用于单独调试。

## 配置与启动

底盘配置通过 `agv.launch.xml` 参数 → xacro 的 `<hardware><param>` → `HardwareInfo.hardware_parameters` 传递。
这些是硬件参数，不是独立 `robot_move_node` 的 ROS 参数。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `chassis_ip` | 192.168.1.109 | 沿用原配置，需设为实际底盘的 IPv4 |
| `agv_max_linear_speed` | 0.0 | 合成线速度上限，m/s；0不限制 |
| `agv_max_angular_speed` | 0.0 | 角速度绝对值上限，rad/s；0不限制 |
| `agv_init_set_do` | true | 配置时是否设置DO |
| `agv_init_do_id` | 0 | 非负int范围的DO编号 |
| `agv_init_do_status` | true | DO电平；具体用途由接线决定 |
| `agv_init_relocate` | true | 配置时是否执行重定位 |

构建并加载工作区：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to agv_hardware agv_bringup
source install/setup.bash
```

例如，明确关闭初始化动作并设置速度上限：

```bash
ros2 launch agv_bringup agv.launch.xml \
  chassis_ip:=192.168.1.109 \
  agv_max_linear_speed:=0.3 agv_max_angular_speed:=0.5 \
  agv_init_set_do:=false agv_init_relocate:=false
```

此命令仍会启动现有整机流程，包括机械臂连接/激活。默认DO设置与重定位保持原有行为。
参数非法时插件初始化返回ERROR，不建立底盘连接。超限速度会被拒绝，不自动截断。

## 生命周期

- `on_init`：解析参数、创建对象，不连接底盘。
- `on_configure`：应用限速及初始化选项，初始化底盘，再连接机械臂；失败清理连接。
- `on_activate`：激活机械臂并启动回调，最后开放命令入口。
- `on_deactivate`：关闭命令入口，停止并回收回调来源，等待在途底盘请求结束；保留底盘连接。再次激活不重复DO或重定位。
- `on_cleanup/on_shutdown/析构`：回收资源、关闭连接；清理后可重新配置。

底盘网络请求只在配置及命令回调中执行，不进入 `read/write` 高频循环。
停用和断开连接仅管理资源，不表示机器人已停车；本次未增加停车或导航取消策略。

## 上位机速度命令

保留两种入口，单位为m/s、m/s、rad/s：

```json
{"header":{"msg_type":"command"},"payload":{"chassis":{"command":"velocity_move","parameters":{"vx":0.1,"vy":0,"wz":0.1}}}}
```

```json
{"header":{"msg_type":"command"},"payload":{"agv":{"command":"move","data":{"vx":0.1,"vy":0,"wz":0.1}}}}
```

`chassis.parameters`也支持原有`x/y/yaw`别名；未提供的分量为0。提供的速度字段必须为有限JSON数值。
两种入口都只调用一次新驱动，发送19205端口的2010请求（`vx/vy/w/duration=0`），不新增执行成功应答。

## 验证

```bash
colcon build --packages-up-to agv_hardware --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select robot_move agv_hardware --event-handlers console_direct+
colcon test-result --verbose
```

新增测试：`chassis_config_test`、`chassis_hardware_test`、`chassis_loopback_test`、`chassis_model_test`。
机械臂仅替换生命周期协作进行测试；本地模拟服务只监听127.0.0.1的19204/19205/19206/19210端口，需确保这些本地端口空闲。
测试覆盖真实JSON解析、驱动分层及硬件模型参数解析，并非真实整机运动验收。

## 导航模式

整机 `agv.launch.xml navigation_mode:=true` 向硬件传入 `external_navigation=true`：
插件不连接/初始化底盘并拒绝WebSocket底盘命令，由专用导航驱动独占。机械臂流程保留。
旧模式速度现为固定800ms有限duration，不具备导航状态机，控制器到期行为仍待实机验收。
见 [导航停止机制](../../docs/导航停止机制.md)。
