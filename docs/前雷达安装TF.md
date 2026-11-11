# 前雷达安装 TF

前后雷达 TF 功能接入、发布唯一性及真实扫描对齐验证已通过，并获用户现场确认。当前使用值如下（位置相对 base_link，单位 m；角度单位 °）：

| 雷达 | x | y | z | roll | pitch | yaw |
| --- | --- | --- | --- | --- | --- | --- |
| 前 | -0.295 | -0.550 | -0.240 | 0 | 0 | -135 |
| 后 | 0.295 | 0.550 | -0.240 | 0 | 0 | 54 |

高度未独立实测验收，选取墙段仍约有 5 cm 残差；本次通过的是 TF 功能和现场扫描对齐，不宣称高精度外参标定完成。以下保留坐标推导、启用方法以及从临时预览到实机核对的实施记录。

## 坐标依据及尚缺信息

- `agv_robot.urdf.xacro` 包含 SolidWorks 导出的 `agv.xacro`。`base_link` 是模型根，visual/collision origin 均为 `0 0 0`、rpy 为零；没有真实地面、接地轮系或已标定的 `base_footprint`。
- `lifiting_joint` 沿 +Z 升降，支持模型 +Z 向上这一解释；没有足够资料确认模型 +X 是否实机前方、+Y 是否实机左方。ROS 目标约定为右手系：+X 前、+Y 左、+Z 上，应在实机确认 CAD 原点和轴向。
- 直接读取二进制 `meshes/base_link.STL` 的 435247 个三角面，包围盒（m）为 x=[-0.3748293,0.3748293]、y=[-0.6239985,0.6239985]、z=[-0.7545000,0.0035000]。原点接近该网格顶面，但网格最底点不能直接证明轮子接地位置；不可把 0.7545 m 当成已知原点离地高度。`agv_yuan.xacro` 也是 CAD 导出模型，没有补充地面标定。
- 旧 `gazebo.launch` 的零位移 base_link/base_footprint 关系、Gazebo 预览的悬空高度及导航仿真的雷达参数都不是实机测量依据。本实现不改这些仿真入口。

仙工截图的 x=0.55、y=-0.295、z=0.60、yaw=-47°、upside=true、minAngle=-125°、maxAngle=130°仅是控制器模型记录。必须先核对其原点、轴向、手性和雷达安装测量参考点。一般关系为：

```text
T_ROSbase_sensor = T_ROSbase_controllerBase × T_controllerBase_sensor
```

只有两个底盘坐标系已确认重合，且 sensor 指同一个扫描坐标系/光学原点时，才能直接沿用 x/y/yaw。存在底盘轴向旋转时，位置向量也要旋转，不能只加减 yaw。本实现要求直接填写最终 ROS base_link 相对外参，不自动读控制器安装信息，也不改控制器模型。

两坐标竖直轴一致、机器人处于平地时：

```text
z_relative = 光学原点离地高度 - base_link 原点离地高度
```

离地约 0.50 m 的测量点尚未确认是光学原点。该估计和未知 base_link 离地高度保存在 YAML 的 `reference_only`，不参与 TF。最初按用户要求以相对 z=0 预览，随后由用户根据模型调整到当前的 -0.24 m；它尚未经过独立高度测量。

## upside 与扫描坐标

本地资料：`docs/agv_docs/seer_api/查询机器人激光点云数据_-_Feishu_Docs_7abb40aeac.md`（离线资料未纳入 Git），来源为[仙工 1009 接口文档](https://seer-group.feishu.cn/wiki/F5UZw9yorisYgsk92sdcccdQnUd)。可见文本给出了 beams.angle 和 install_info/upside 的 JSON 示例，没有解释 upside 的准确语义，也没有说明返回角度是否已做倒装/安装补偿。对本地资料搜索未找到足以确定这些语义的说明。

现有 C++ 解析只把 beams.angle 从度转换为 rad，构建器按真实角度重采样；没有用 install_info 旋转数据。保持 `/scan_front.header.frame_id=laser_front_link`。minAngle/maxAngle 是角域，不是安装姿态；此次不调整采集角域。

待现场区分的假设：

1. API 返回传感器原生角度、未修正倒装：若物理光轴/安装轴确实对应绕局部 X 翻转 180°，才可能需要 roll=180°，应实测验证，不能由 upside 勾选直接推出。
2. API 已修正倒装：扫描 frame 应匹配修正后的虚拟扫描轴，不应再次翻转。
3. API 已补偿安装 yaw：不能在 TF 重复添加 -47°。必须先确定返回扫描相对于哪个方向；若连平移/扫描原点也已补偿，应先修正数据坐标语义，不能仅设置 yaw=0 就宣称通过。

这些是假设，代码不自动选择任何一种。roll/pitch/yaw 必须显式填写。安装外参应描述**所发布扫描数据实际使用的坐标系**，不一定等于雷达外壳的物理轴向。

## 配置和启动

配置为 `agv_description/config/lidar_mounts.yaml`。`front` 中位置单位 m、欧拉角单位度；Xacro 转为 rad，使用 URDF 固定轴 RPY，即 Rz(yaw) Ry(pitch) Rx(roll)。父/子固定为 `base_link` / `laser_front_link`，不提供容易与消息 frame 不一致的改名入口。

仓库配置已填入上述现场使用值；调整时可先复制配置，再修改 front/back 各字段。不要把参考截图未经核对地复制为 base_link 外参。缺值/非数值/非有限数值会使启用 launch 失败并指出字段。默认整机 TF 模式为 off，不会把未知量当成零。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select agv_description lidar agv_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
cp src/agv_description/config/lidar_mounts.yaml /tmp/front_lidar_mounts.yaml
# 可编辑 /tmp/front_lidar_mounts.yaml 调整外参；当前仓库值已通过扫描对齐验证

# 独立只发布前雷达 TF，不启动硬件、导航或采集
ros2 launch agv_description robot_description.launch.py \
  lidar_mounts_file:=/tmp/front_lidar_mounts.yaml

# 另一个终端：只读采集（原有双雷达话题，后雷达暂无 TF）
ros2 launch lidar seer_lidar.launch.py connection:=实际底盘IP

# 或独立查看完整模型，仍不启动 ros2_control_node 或硬件
# 先停止上述 TF launch，再运行本命令
ros2 launch agv_description robot_description.launch.py \
  model:=full front_lidar_tf_owner:=description \
  lidar_mounts_file:=/tmp/front_lidar_mounts.yaml
```

`front_lidar_tf_owner` 只有 `off`、`description`、`standalone` 三种互斥值。完整模型模式中：description 由完整 URDF 的一个固定关节发布；standalone 明确从完整模型禁用该关节，再由精简 URDF 的 robot_state_publisher 发布同一个固定关节。两者共用同一宏和同一配置，没有第二份静态 TF 数值。`model=front_only` 只允许 standalone。直接展开原完整 Xacro 时默认关闭前雷达关节，显式传入 `enable_front_lidar_tf:=true lidar_mounts_file:=...` 才启用。

整机 `agv.launch.xml` 已传递这两个参数。其默认 owner=off；集成时设置 owner=description（优先）或 standalone。整机入口会运行原有硬件流程，本任务未执行该入口。只看雷达应使用上面的独立命令。

发布模式互斥作用于这一启动链；ROS 不会阻止用户在其他终端重复启动同一 TF。整机启用 TF 后应停止独立 TF launch，检查其他 display/MoveIt/自定义 static_transform_publisher 是否也发布 laser_front_link；不要同时运行多份描述发布器。扫描节点本身不发布 TF。

## 验证与验收

```bash
colcon test --packages-select agv_description lidar agv_bringup --event-handlers console_direct+
colcon test-result --verbose
xacro src/agv_description/urdf/agv_robot.urdf.xacro \
  enable_front_lidar_tf:=true lidar_mounts_file:=/tmp/front_lidar_mounts.yaml \
  > /tmp/agv_with_front_lidar.urdf
ros2 run tf2_ros tf2_echo base_link laser_front_link
ros2 topic echo /scan_front --once --field header
ros2 topic info /tf_static --verbose
ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable
```

确认展开结果恰有一个 `laser_front_link` 和 `laser_front_mount_joint`，父 base_link、子 laser_front_link、type=fixed；默认后雷达关闭时不生成 laser_back_link；启用 back_lidar_tf_owner 后，同样检查后雷达链接及固定关节各恰有一个。自动测试还覆盖度转 rad、不同模式的关节唯一性和未知字段拒绝。`/tf_static` 可以有多个发布者，因为其他固定关节也需要发布；要逐发布者检查 child_frame_id=laser_front_link 的来源恰有一个，不能仅根据 topic publisher 总数判断冲突。可以结合 `ros2 node info` 的发布话题、`ros2 topic info --verbose` 的节点名/GID，并逐一隔离启动可疑发布器检查；检查残留缓存时应重启观察工具。

RViz：Fixed Frame 设 base_link，添加 TF、Grid、LaserScan；LaserScan Topic 选 /scan_front，Reliability 选 Best Effort。完整模型模式可添加 RobotModel。固定前雷达 frame 只能检查局部扫描，不能验证底盘外参。

机器人静止，在测量过的前方、左侧、右侧放置已知障碍，记录它们相对 CAD/base_link 原点的坐标。核对点的左右符号、前后位置及距离，避免墙面近似对称掩盖 yaw 或翻转错误。用两面方向已知且不平行的墙核验 yaw，用雷达零角标记和扫描角递增方向核验倒装；先弄清 API 补偿，再确定 roll/yaw，禁止同时在数据和 TF 中旋转同一角度。

用水平地面和尺/水平仪测量 base_link 原点离地高度、雷达光学原点离地高度，另测安装面到光学原点偏置；确认竖直轴后计算 z_relative，替换临时零值。测量倾角以验证 roll/pitch。二维扫描看起来正常不能证明高度准确；与 z 轴垂直的墙对扫描平面高度不敏感。

软件接入通过只表示构建、坐标表达、发布链和消息 frame 一致；安装外参标定通过还要求参考点/轴向、倒装补偿、现场障碍和高度测量全部有证据。历史采集断流问题仍独立存在，本次不扩展里程计、不启动导航、不发送速度、DO 或重定位。

## 本次软件验证结果（2026-11-11）

- agv_description、lidar、agv_bringup 构建成功，6 项 CTest 全部通过：描述包 1 项（内部 3 个测试）、lidar 4 项、bringup 1 项（内部 7 个测试）。描述包最后一次调整日志提示后再次构建、测试通过。
- 整机 agv.launch.xml 的 --show-args 参数解析通过，没有启动整机硬件。
- 在 ROS_DOMAIN_ID=219 使用独立临时合成配置 x=0.13 m、y=-0.21 m、z=0、roll=pitch=0°、yaw=-47°启动实际发布器，依次检查 front_only/standalone、full/description、full/standalone。每种模式只收到一个前雷达固定变换；图中 /tf_static 发布者数量分别为 1、1、2，最后一种的完整模型不包含前雷达关节，精简模型包含且只包含该关节。位移和四元数检查均通过。
- tf2_echo 实际输出 [0.130, -0.210, 0.000]，yaw=-47°≈-0.820 rad。合成数值只用于软件验证，绝不作为现场外参；临时记录为 /tmp/front_tf_*.log、/tmp/front_tf_echo.log，不作为构建依赖。测试启动的发布器均已停止。
- 本次未连接实机，也未运行 RViz 图形窗口。现场障碍位置、倒装补偿和高度验收仍待完成；二维扫描显示正常不能单独证明高度准确。

## 后雷达对角安装预览扩展（2026-11-11）

用户确认先按前后关于 base_link 原点对称、同高、朝向相反进行临时预览。当前前雷达由用户调整为 (-0.295,-0.550,-0.240) m、yaw=-135°；后雷达暂用 (0.295,0.550,-0.240) m、roll=pitch=0°、yaw=45°。这些是已授权的预览假设，不能当作测得的后雷达标定结果。两台各自独立存储，不随前雷达后续修改而自动镜像。

共用 YAML 新增 back 字段；Xacro 新增 enable_back_lidar_tf（默认 false）与 base_link → laser_back_link 固定关节。launch 和整机入口新增 back_lidar_tf_owner，默认 off，可选 description/standalone/off；每台独立选择且各自只出现一个固定关节，独立发布模式仍复用同一 YAML 和 Xacro。既有 front_only 模型名为兼容保留，使用精简雷达模型时后雷达可选 standalone 或 off，description 要求 model=full。

只读完整模型预览，不启动硬件：

```bash
ros2 launch agv_description robot_description.launch.py \
  model:=full front_lidar_tf_owner:=description back_lidar_tf_owner:=description
ros2 run tf2_ros tf2_echo base_link laser_back_link
```

RViz 添加参考帧 laser_back_link 的 Axes，并在 TF 中显示该名称。后雷达角域尚未确认，不根据前雷达255°假设生成后雷达扫描扇形。/scan_back 的 frame_id 已是 laser_back_link，未改采集数据。

agv_description 与 agv_bringup 构建、2 项 CTest 通过（内部共11个测试），新增覆盖后雷达固定关节数值、独立开关、前后9种发布模式组合的唯一性及后雷达缺失字段拒绝。

RViz 已在隔离域219启动三坐标系预览，实际 tf2_echo 确认后雷达 [0.295,0.550,-0.240] m、yaw=45°；静态消息检查确认前后各恰有一条变换。未启动导航、底盘硬件或真实采集。

## 真实扫描错开排查与相对 yaw 修正（2026-11-11）

用户报告底盘静止时同一物体出现双线。捕获前后各30帧，在 base_link 中比较选取的共同墙形轮廓：原配置前 yaw=-135°、后 yaw=45°时，三个时刻的拟合直线方向差为8.57°、8.70°、8.70°，并非只有平移。实际 TF 与配置一致，采集节点不应用 install_info 旋转；控制器诊断中前 yaw=-47°、后 yaw=142°，差189°，作为相对安装关系的辅助证据，而非直接复制控制器坐标。

仅在离线把后 yaw 增加9°到54°，保持所有 xyz、前雷达姿态不变，捕获场景回放中的方向差降至0.30°、0.12°、0.20°。先建立诊断阈值1°，旧配置回放失败；修改 back.yaw_deg=54 后同一回放通过。随后重启真实采集与 RViz，新采前后各30帧，三个时刻方向差为0.13°、0.23°、0.16°，选取墙段法向位置差仍约0.052～0.056 m。

结论：对称预览中“朝向严格相差180°”这一假设不满足当前真实扫描，后 yaw=54°是有场景数据支持的临时相对修正。尚不能证明绝对 base_link 姿态、xyz、光学原点、高度或倒装/API补偿全部正确；不为消除剩余5cm而自动改动用户确认的位置。仍须对照实测障碍和光学原点验证。采集通信仍偶发超时，不把短窗口内收到扫描称为长期稳定性通过。

现场捕获、回放检查和绘图均保存在明确用于排查的临时目录 `/tmp/agv_origin_preview/`，包括 alignment_capture.json、alignment_comparison.png、check_alignment_replay.py、alignment_after_yaw54.json、alignment_after_results.json；这些数据不作为包的自动测试或构建依赖。本轮没有改动驱动、倒装处理或控制器配置，只修改后雷达临时 yaw 并重新运行只读显示。

## 现场对齐验收结论（2026-11-11）

用户查看后 yaw=54°的真实前后扫描后确认“这下对了”。标记前后 TF 软件接入及现场扫描对齐验证通过；保留独立高度测量、约5cm位置残差、API倒装补偿语义等进一步精度核对项。外参配置以本文开头表格和 lidar_mounts.yaml 为准，前述0/45°等均为历史预览值。
