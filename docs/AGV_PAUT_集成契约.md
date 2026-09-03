# AGV_ROS2_Control_Manager ↔ PAUT_ros2 集成契约（胶水清单）

> 本文件把「机器人平台」与「超声相控阵(PAUT)感控」两个**独立 ROS 2 工作区/仓库**之间的对接约定固化成一份清单。
> 它们编译期互不依赖，运行期靠 ROS 话题 / DDS / UDP 拼成一套系统。**凡是两端要保持一致的约定，都以本文件为准；改动跨两端前先同步此处。**
>
> - 仓库 A（机器人平台，运动控制）：`AGV_ROS2_Control_Manager` —— `src/agv_hardware`、`src/pathplannernode/src/pathplanner`、`src/agv_bringup` 等
> - 仓库 B（探伤感控）：`PAUT_ros2` —— `src/paaut_control`（目录 `paaut_control_ros`，**构建名是 `paaut_control`**）、`src/paaut_arm_control`

---

## 1. 职责分工

| 侧 | 负责 | 边界 |
|----|------|------|
| **AGV（A）** | 移动底盘 + 升降/回转 + 双臂；机械臂底层执行（收 `cmd_arm_cartesian` 走 SDK）；发布末端位姿反馈与运行状态；SAM2 路径规划器输出末端路径模板 | 只认「末端(ee)绝对位姿」，不关心探伤/缺陷 |
| **PAUT（B）** | UDP 收探伤仪图像 → 峰值/叉形/缺陷检测 → 按超声回波做工具系微调，向 A 下发 `cmd_arm_cartesian` | 不自己动硬件，假定 A 已把臂使能并提供位姿/状态反馈 |

一句话：**A 提供“臂能按 ee 位姿走 + 报位姿/状态”，B 决定“走到哪、怎么微调”**。二者交界点全部在本清单第 3 节的话题上。

---

## 2. 进程/节点拓扑

```
探伤仪/采集机 ──UDP 12345(广播)──► [paaut_control_node]  (B)
                                        │ 发
                                        ▼
   发 paaut/detection_result、paaut/detection_defect
                                        │ 订
                                        ▼
                               [arm_control_node]  (B)  ← 闭环主脑
                                  │ 发布 cmd_arm_cartesian
                                  ▼
[agv_hardware (A) AgvHardwareInterface] ──rokae SDK──► 机械臂末端执行
      │ 发布 arm_cartesian_pose / arm_operation_state ─────────►(B 反馈闭环)
      ▲ 订 cmd_arm_cartesian
[agv_bringup: agv.launch.xml 拉起 controller_manager / 三个 JTC]
      ▲
[pathplanner (A) SAM2] ──发布 /path_planner/path_3d、/path_planner/cartesian_path──► 也发给(B)作路径模板
```

---

## 3. 话题契约（真·胶水 = 跨仓库的 4 条）

> 源码字面量一律**不带前导 `/`**（`cmd_arm_cartesian`、`arm_cartesian_pose`…），在默认根命名空间下即等价于全局 `/xxx`。PAUT 的 `docs/README.md` 写成带 `/` 仅为排版，同名。

### 3.1 跨仓库主线（A↔B 必须对齐）

| # | 话题 | 类型 | 发布端 | 订阅端 | 数据语义 | 帧假设 |
|---|------|------|--------|--------|----------|--------|
| C1 | `cmd_arm_cartesian` | `std_msgs/Float64MultiArray` | **B** `arm_control_node.cpp:188` | **A** `agv_hardware_interface.cpp:65` | 元素 6：`[x,y,z,rx,ry,rz]`，**末端 ee 绝对笛卡尔目标位姿**（一次一个点，非轨迹） | A 臂基座坐标系（绝对） |
| C2 | `arm_cartesian_pose` | `std_msgs/Float64MultiArray` | **A** `agv_hardware_interface.cpp:46`（read() 内每周期发布） | **B** `arm_control_node.cpp:181` | 元素 6：`[x,y,z,rx,ry,rz]` 当前 **ee 位姿反馈**；B 收到后置 `pose_ready_`，是后续修正的前提（`compute_and_publish_path_correction` 无它直接 return） | 与 C1 同一参考系（A 臂基座） |
| C3 | `arm_operation_state` | `std_msgs/Int32` | **A** `agv_hardware_interface.cpp:48`（`GetRobotState()` 取 `rokae::OperationState`） | **B** `arm_control_node.cpp:192` | **0=静止 / 9=运动中**；B 靠 `9→0` 沿推进状态机（PATH_MOVING→PATH_DETECTING 等），`9` 时记 `last_move_start_` 做 3s 看门狗 | 无 |
| C4 | `path_planner/cartesian_path` | `geometry_msgs/PoseArray` | **A** `pathplanner` `pathplanner_ros2_node.py`（输出话题 `path_planner/cartesian_path`） | **B** `arm_control_node.cpp:199` **且** A `agv_hardware_interface.cpp:85`（监听） | 一串**已含姿态的末端 ee 目标点列**（位置+四元数）；B 收到即从第 1 点开始逐点“移动→检测→微调” | A pathplanner 以 `arm2_cartesionbase_link` 为系；B 假定点已是 ee 系、**不再叠加变换**（见 `:244` 注释） |

### 3.2 仓库 B 内部闭环（A 可选旁听，勿当作 A 的输入）

| 话题 | 类型 | 发布 | 订阅 | 语义 |
|------|------|------|------|------|
| `paaut/detection_result` | `Float64MultiArray` | `paaut_control_node` | `arm_control_node:173` | 元素0..6 = 斜率/Ｒ²/分叉/低谷长/回波峰数，≥7 变长回波幅值 |
| `paaut/detection_defect` | `Float64MultiArray` | `paaut_control_node` | `arm_control_node:205` | 元素0=缺陷数，其后每缺陷 4 项 `[centroid_x,centroid_y,area,max_val]` |

### 3.3 仓库 A 侧其它话题（与 PAUT 无直接关系）

`cmd_arm_joint`(6关节角)、`arm_joint_states`、`cmd_vel`、`agv_pose`、`robot_status`、`/click_point`、`/path_planner/path_3d`。B 均未引用（`arm_joint_states / click_point / agv_pose / cmd_vel` 在 PAUT 源码中 0 命中）。

---

## 4. DDS / QoS / 命名约定

- **ROS_DOMAIN_ID**：两端 launch 均无 domain 设置 → 默认 **0**。必须同域才能互连；跨机时设为相同 ID 并保证组播可达。
- **namespace / remap**：两端均无 namespace、无 remap；**不要给 A 或 B 的节点套 namespace**，否则 4 条主线话题名对不上。
- **QoS**：4 条主线全部保持默认 **RELIABLE + KeepLast(10)**。C2 反馈率= A 控制环 read() 频率，B 只判断“是否收到过/值变”，不要假设固定频率；A 端发布间隔也不要低于 B 的看门狗阈值（B 内 3s）。
- **环境**：跑 A 节点的 shell 需 `source AGV…/install/setup.bash`；跑 B 节点的 shell 需先 `source AGV…/install/setup.bash` 再 `source PAUT…/install/setup.bash`（两个都 source，PATH 会叠加，两仓可同 shell 共存）。

---

## 5. 坐标/帧语义 —— 最易踩坑区

1. **C1/C2 的参考系必须同源**：两端都把 C1、C2 当作「A 臂基座（绝对）ee 位姿」。若 A 侧实际指令/反馈用的是别的系（如 `arm2_cartesionbase_link` 与臂基座偏移），B 全链路错位。**建议以真机验证：发一个已知 ee 点，对比 C2 反馈与示教器读数。**
2. **C4 点列是“末端”系**：B 收到 `path_planner/cartesian_path` 后当作 ee 目标直接执行，**不再加工具变换**。而 A 的 pathplanner 输出 frame 是 `arm2_cartesionbase_link` 且已含工具补偿 —— 请确保 pathplanner 输出的“ee 位姿 + 参考系”与 C1/C2 同一约定（当前默认一致，改动任一端坐标系时两端同改）。
3. **末端→工具外参（关键不一致项 ⚠️）**：
   | 位置 | 常量 | 值 |
   |------|------|----|
   | A `agv.xacro` 关节 `arm2_tool_link→arm2_toolscan_link` origin | `xyz rpy` | `0 -0.21 0.0748  /  -1.5708 3.14159 0`（本次提交后 y=-0.21） |
   | B `arm_control_node.cpp:34-40` `TOOL_OFFSET_*` | 注释“arm2_tool_link→arm2_toolscan_link” | `X=0, Y=-0.18, Z=0.0748, RX=-π/2, RY=π, RZ=0` |
   → **Y 差 0.03**（B 还是旧值 -0.18）。B 用这套常量做「工具尖点保持」微调，A 的 URDF 仅用于显示/TF。若真机装卡确实变了，两处必须对齐到同一实测值，否则 A 显示与 B 执行不一致。**统一前先真机重标 TOOL_OFFSET，并同步回 agv.xacro。**
4. **状态语义已对齐**：A 发的 `rokae::OperationState` 中 `idle=0, moving=9` 与 B 期望 `0 静止 / 9 运动中` 一致；注意其它枚举(如 rtControlling=2) 可能混入，B 只认 0/9 判沿，其余值不会推进状态机。

---

## 6. 状态握手时序（B 闭环如何驱动 A）

```
B arm_control_node            A agv_hardware
      │  (pose_ready_=true 需 C2 至少来过一帧)
      │  publish C1 cmd_arm_cartesian(ee)
      │──────────────────────────►  收到→走 SDK 执行(移动)
      │                             状态 idle(0)→moving(9)
      │  ◄────────────────────────── C3=9
      │  记 last_move_start_，等 9→0（3s 看门狗）
      │  ◄────────────────────────── 到位 moving(9)→idle(0)
      │  C3 下降沿 → 推进状态机（下一步/微调/检测）
      │  （微调时用 C2 反馈做工具系 rx/ry/rz 修正，保证工具尖点不动）
```

- A 只负责“收到 ee 点就走 + 回状态/位姿”；B 负责“何时发下一个点、发修正”。
- **B 缺 C2/C3 时的表现**：C2 没有 → `pose_ready_` 恒 false，修正逻辑直接 return；C3 没有 → 状态机卡在 WAITING/PATH_MOVING。启动排查先看这两条话题有没有数据。

---

## 7. 启动清单（顺序很重要）

前置：硬件上电、臂使能；各 shell 先 source 对应环境。

| 步骤 | 命令 | 说明 / 验证 |
|------|------|------|
| 1. 机械臂/整机（A） | `ros2 launch agv_bringup agv.launch.xml` | 拉起 controller_manager + 3 JTC；agv_hardware 订阅 C1、发布 C2/C3。验证：`ros2 topic hz /arm_cartesian_pose` |
| 1a. 视觉路径模板（A，可选） | `ros2 launch camera camera_with_tf.launch.py` → `ros2 launch pathplanner ros2_pathplanner_launch.py` | 需要 SAM2 自动出路径时才起；pathplanner 发 C4 |
| 2. 探伤供数（外部） | 探伤仪向本机 **UDP 12345** 打帧 | 非 ROS；`paaut_control` 只 bind + recvfrom 广播 |
| 3. 感控（B） | `ros2 launch paaut_control paaut_control_launch.py port:=12345` | 发 `paaut/detection_result`、`paaut/detection_defect` |
| 4. 臂控制闭环（B） | `ros2 launch paaut_arm_control arm_control_launch.py thin_wall:=false` | 订阅 C1 上游(自己发 C1)、C2/C3；等日志出现“位姿就绪” |

> 排错口诀：先 A 后 B；B 起不来先 `echo` C2/C3；闭环不走先看 C3 的 9→0 沿；点位偏先查第 5 节工具外参。
> 探伤/缺陷话题是 B 内部闭环，A 侧调试不必关心；A 若也要读缺陷结果可另订 `paaut/detection_defect`（当前无此依赖）。

---

## 8. 一致性敏感项清单（改动前必查，两端同步）

| 项 | 唯一事实源 | 需同步到 |
|----|-----------|---------|
| 末端→工具(toolscan)外参 | **真机实测标定** | A `agv.xacro` joint origin 与 B `TOOL_OFFSET_*`（当前 Y 值 0.21 vs 0.18 已不一致 ⚠️） |
| C1/C2 的参考系（臂基座 ee） | 真机验证 | 两端共同认知；改坐标系两端同改 |
| C4 参考系 / 是否已含工具补偿 | A pathplanner | B 的“ee 已含”假设 |
| `arm_operation_state` 值域 0/9 | A `OperationState` | B 状态机 |
| 话题名（不带 `/`） | 本文件 §3 | 勿在任一端套 namespace |
| `paaut_control` 包名（非 `paaut_control_ros`） | B package.xml | launch/文档引包名时 |
| UDP 端口 12345 / 帧头 `[h,w]int32 + int32[]` | B `udp_receiver.cpp` | 探伤仪侧配置 |

---

## 9. 风险提示（当前代码状态）

- **C4 双订阅者风险**：A 的 `agv_hardware` 与 B 的 `arm_control_node` 都订阅 `path_planner/cartesian_path`。当前 A 侧“整条轨迹下发”循环**被注释**（`agv_hardware_interface.cpp` 中只打日志不下发），因此实际执行者只有 B。若将来放开 A 侧执行，会出现**两个执行器抢跑 C4** —— 保持二选一，并在此更新职责约定。
- B 的 `arm_control_node.cpp` 有一处 `GetRobotState` 语义以 A 的 `read()` 发布频率为准；A 侧该话题在 control loop 里发布，注意别把阻塞 SDK 调用压进实时环（见 AGV README 已知事项）。
