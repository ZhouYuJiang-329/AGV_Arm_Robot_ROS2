# RViz2 导航显示与话题对照表

适用环境：本项目 ROS2 Jazzy，二维 AGV 地图定位、Gazebo 导航仿真、仙工实机只读显示。根据仓库 launch / RViz / YAML 配置和本机 Jazzy 消息、插件定义核对。

RViz2 的基本流程是：**节点发布数据 → RViz 添加匹配的 Display → 选择话题 → 利用 TF 转到 Fixed Frame → 显示**。数据可以来自实机、Gazebo、录包或合成程序。RViz2负责可视化，动力学、定位和路径规划由其他节点实现。Display、Toolbar Tool、Panel分别是数据显示、交互工具和操作面板，不能混为同一种“Add显示”。

下面的话题名除“项目实际配置”外均为常用示例，命名空间、重映射及插件配置可以改变它们。不是每个系统都需要发布全部话题，也不是添加Display后系统就会自动生成数据。

## 1. 基础显示：地图、模型、雷达、里程计与定位

| 想看到的内容 | RViz Add 显示类型 | 常用话题 | ROS2 消息类型 | 要发布的关键数据 | 坐标 / 时间依赖及设置 |
| --- | --- | --- | --- | --- | --- |
| 二维占用地图 | Map | `/map` | `nav_msgs/msg/OccupancyGrid` | `header`；`info.resolution`、`width`、`height`、`origin`；`data`栅格数组 | 通常frame=`map`；RViz用Reliable、Transient Local读取已发布地图 |
| 地图增量更新 | Map 的 Update Topic | `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | 更新矩形的`x/y/width/height/data` | 依赖对应完整地图；不是另加一个Map显示；本项目静态地图服务器不需要它 |
| 机器人三维外形 | RobotModel | `/robot_description` | `std_msgs/msg/String` | URDF XML，含link/joint、mesh、visual origin | Description Source选Topic；URDF不含整车实时位姿，各link位置来自TF；mesh资源必须可找到 |
| 关节运动所需反馈 | 间接支持 RobotModel | `/joint_states` | `sensor_msgs/msg/JointState` | `header.stamp`、`name[]`、`position[]`；速度/力矩按需 | 由robot_state_publisher结合URDF生成关节TF；不是RobotModel直接用这个话题移动整车 |
| 动态坐标关系 | TF | `/tf` | `tf2_msgs/msg/TFMessage` | 每个变换的父`frame_id`、子`child_frame_id`、时间、translation、rotation | 用tf2 broadcaster发布；包括map→odom、odom→base_link、活动关节等 |
| 固定安装坐标关系 | TF，与上一行同一个Display | `/tf_static` | `tf2_msgs/msg/TFMessage` | 雷达/相机等固定安装变换 | 用static broadcaster或robot_state_publisher发布；Transient Local；不能把整车运动变换发成静态TF |
| 指定坐标系的三根轴 | Axes | 无独立数据话题 | 无 | 在Reference Frame中填写`base_link`、雷达frame等 | 从TF取位姿；Fixed Frame与目标frame必须连通 |
| 地面参考网格 | Grid | 无 | 无 | Cell Size、Plane、格数、颜色等显示参数 | RViz本地生成，不代表真实地面、地图或传感器数据 |
| 前二维雷达点 | LaserScan | `/scan_front` | `sensor_msgs/msg/LaserScan` | 角度范围/增量、距离数组、有效距离范围、扫描时间 | 实机frame=`laser_front_link`；需要到Fixed Frame的TF；通常选Best Effort |
| 后二维雷达点 | LaserScan | `/scan_back` | `sensor_msgs/msg/LaserScan` | 同上 | 实机frame=`laser_back_link`；单独Add一项，方便配置不同颜色 |
| Gazebo二维雷达 | LaserScan | `/scan` | `sensor_msgs/msg/LaserScan` | 同上 | 本项目Gazebo原始`/sim/scan_raw`经adapter转为`/scan`；查看消息实际frame，不能套用实机frame |
| 三维雷达 / 深度点云 | PointCloud2 | `/points`、`/camera/depth/points`等 | `sensor_msgs/msg/PointCloud2` | `header`、点字段描述、宽高、字节布局、点数据 | 需要点云frame对应TF；Color Transformer可选XYZ/Intensity/RGB等，取决于字段；本轮定位演示未生成 |
| 里程计位姿与历史箭头 | Odometry | `/odom` | `nav_msgs/msg/Odometry` | `header.frame_id`、`child_frame_id`、`pose`、`twist`及协方差 | 本项目odom/base_link；Keep控制保留历史数量；显示是采样箭头，不是自动规划的路线 |
| 单个定位位姿箭头 | Pose | `/seer/localization/base_pose` | `geometry_msgs/msg/PoseStamped` | 带时间和frame的位置、四元数朝向 | 本项目frame=`map`；只显示位姿，不自动生成map→odom或移动RobotModel |
| 定位位姿及协方差 | PoseWithCovariance | `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 位姿及6×6协方差矩阵 | AMCL输出；仙工confidence不是协方差，不能为了显示椭圆直接填进去 |
| 多个位姿 / 采样候选姿态 | PoseArray | 自定义，如`/candidate_poses` | `geometry_msgs/msg/PoseArray` | 一个header和多个`poses[]` | 多个姿态共享header坐标系；无连接顺序线条，需连线时使用Path或Marker |
| AMCL粒子及权重 | ParticleCloud（Nav2插件） | `/particle_cloud` | `nav2_msgs/msg/ParticleCloud` | `header`、`particles[]`中的位姿和权重 | Jazzy Nav2专用显示`nav2_rviz_plugins/ParticleCloud`；不能按旧版本示例一律选PoseArray；仙工定位不产生AMCL粒子 |

RobotModel 的依赖关系是：`URDF + joint_states → robot_state_publisher → 各link的TF`，RViz同时读取URDF和TF。车辆整体在地图中的位置来自 `map→odom→base_link`，机械臂零位预览不是实机关节反馈。

## 2. Nav2 导航过程显示：路径、代价地图、轮廓与调试图形

以下数据需对应的Nav2节点/插件已启动并激活；只有地图、里程计和RViz时不会自动出现路径或代价地图。插件的发布开关、订阅者数量、规划请求和生命周期状态也可能影响数据是否出现。

| 想看到的内容 | RViz Add 显示类型 | 常用话题 | ROS2 消息类型 | 关键数据 / 含义 | 设置与限制 |
| --- | --- | --- | --- | --- | --- |
| 全局规划路径 | Path | `/plan` | `nav_msgs/msg/Path` | `header`和有序`poses[]`；路径通常位于map | 绿色线常用于全局路径；本项目Gazebo配置已添加。Path自身不含速度和到达时间承诺 |
| 局部跟踪路径 | Path | `/local_plan`，具体以控制器插件为准 | `nav_msgs/msg/Path` | 局部控制器选中的短距离路径 | 本项目使用DWB；其他控制器未必发布同名同义话题，不出现时先检查实际发布者 |
| 历史行驶轨迹连续线 | Path | 自定义，如`/executed_path` | `nav_msgs/msg/Path` | 将历次位姿按时间整理成`poses[]` | 需要额外节点维护；RViz不会自动把`/odom`变成Path，Odometry显示可直接保留历史箭头 |
| 全局代价地图 | Map | `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 全局障碍、膨胀代价、未知区域等二维栅格表示 | Color Scheme选costmap，Alpha约0.3～0.6；通常frame=map；QoS按实际发布者匹配 |
| 局部代价地图 | Map | `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 机器人附近滚动窗口、实时障碍与膨胀 | 本项目frame=odom，RViz Fixed Frame仍可为map；本项目Gazebo配置已添加 |
| 全局代价地图增量 | 对应Map的Update Topic | `/global_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | 局部修改矩形 | 先订阅完整costmap，再处理增量；本项目`always_send_full_costmap=true`，主要按完整图显示 |
| 局部代价地图增量 | 对应Map的Update Topic | `/local_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | 同上 | 不是另一个独立的Map Display；是否产生增量取决于发布策略 |
| Nav2内部原始代价图 | 无需按普通Map添加 | `/global_costmap/costmap_raw`、`/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | Nav2原始代价编码及元数据 | 与OccupancyGrid类型不同；普通Map显示选择上一组`costmap`话题。不能把原始代价字节原样当占用概率 |
| 全局机器人轮廓 | Polygon | `/global_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 按当前位姿变换后的轮廓顶点 | 通常在map中；用于核对规划碰撞几何，与CAD外观可能不同 |
| 局部机器人轮廓 | Polygon | `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 同上 | 通常在odom中；需要TF变到map |
| 自定义区域 / 禁行边界线 | Polygon | 自定义，如`/inspection_area` | `geometry_msgs/msg/PolygonStamped` | 有序多边形顶点 | 显示一条区域边界不等于Nav2已施加禁行规则；实际限制需相应costmap filter等配置 |
| 障碍栅格集合 | GridCells | 自定义，如`/obstacle_cells` | `nav_msgs/msg/GridCells` | cell宽高与中心点集合 | 与完整OccupancyGrid不同，不是所有costmap节点都发布这种类型 |
| 目标点、文字、箭头、线、立方体、球等 | Marker | 自定义，如`/visualization_marker` | `visualization_msgs/msg/Marker` | `header`、`ns/id`、`type/action`、pose、scale、color、points/text、lifetime | 颜色alpha必须大于0；同一ns/id会更新同一对象；DELETE用于删除 |
| 多个调试图形 / 候选轨迹 | MarkerArray | 自定义，如`/visualization_marker_array` | `visualization_msgs/msg/MarkerArray` | 多个Marker，每个Marker各自有header、ID和样式 | 可画规划候选轨迹、障碍包围盒、站点标签；具体控制器调试话题依插件/发布开关而异 |
| 带交互控件的标记 | InteractiveMarkers | 自定义，如`/interactive_marker`话题前缀 | `visualization_msgs/msg/InteractiveMarkerUpdate`等 | 服务端发布交互标记，RViz回传feedback | 配置的是Topic Namespace，涉及update、初始化服务、feedback；不是只发一个普通Marker即可拖动 |

## 3. 可选传感器和状态数据

这些属于常见扩展，当前离线定位演示不生成。增加显示前先确认项目有对应数据源。

| 内容 | RViz显示 / 查看方式 | 常用话题示例 | 消息类型 | 必要数据与依赖 |
| --- | --- | --- | --- | --- |
| RGB或灰度相机图像 | Image | `/camera/image_raw` | `sensor_msgs/msg/Image` | 宽高、encoding、step、像素数据；单纯二维Image显示通常不依赖全局TF |
| 相机图像在三维环境中的投影 | Camera | `/camera/image_raw`及匹配的`camera_info` | `sensor_msgs/msg/Image` + `sensor_msgs/msg/CameraInfo` | 内外参、图像尺寸、相机光学frame及TF；具体CameraInfo名称按相机话题命名空间匹配 |
| 相机视锥 / 标定参数 | CameraInfo（本机Jazzy内置） | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | K/P/D、宽高、光学坐标系与TF |
| 深度图 | Image，或DepthCloud | `/camera/depth/image_raw`及匹配CameraInfo | `sensor_msgs/msg/Image` + 可选`CameraInfo` | 检查16UC1/32FC1的数值单位和驱动约定；转为3D时需要相机内参及TF |
| 超声 / 红外单束测距 | Range | 自定义，如`/range/front` | `sensor_msgs/msg/Range` | field_of_view、min/max_range、range、radiation_type、传感器frame |
| IMU姿态、角速度、加速度 | `ros2 topic echo`、rqt_plot；姿态可转换为Pose/TF，或安装兼容的IMU插件 | `/imu/data` | `sensor_msgs/msg/Imu` | 四元数、角速度、加速度及对应协方差；本机默认Display不能一律假定有Imu项 |
| 带时间的速度向量 | TwistStamped（本机Jazzy内置） | 自定义速度反馈话题 | `geometry_msgs/msg/TwistStamped` | header、linear/angular；消息必须真的是Stamped类型，不能直接拿Twist话题替代 |
| 系统诊断、定位质量、新鲜度 | 终端echo或rqt_robot_monitor | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | status名称、level、message、键值对；默认RViz无通用DiagnosticArray几何显示，需要文字可另转Marker |
| 导航结果 / 行为树状态 | Navigation 2 Panel、action反馈或终端 | 见后表的导航action | `nav2_msgs/action/...` | 这类是动作状态，不是一个通用的三维Display |

## 4. RViz交互、仿真时钟和控制：不是普通Display

添加数据显示通常只订阅；点击RViz工具则可能**发布输入或调用action**。只读显示时不需要使用导航目标、初始定位或速度工具。本表解释接口，不表示仙工只读节点已实现这些输入。

| 接口 / 功能 | 方向与RViz入口 | 名称示例 | 类型 | 发布 / 接收什么，以及本项目注意事项 |
| --- | --- | --- | --- | --- |
| AMCL初始位姿 | RViz发布；Toolbar的2D Pose Estimate | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | map中的估计位置、朝向和不确定性；AMCL订阅。仙工只读节点不订阅它，点击不会自动触发仙工重定位 |
| 点击三维点 | RViz发布；Publish Point工具 | `/clicked_point` | `geometry_msgs/msg/PointStamped` | 点击点的header和xyz；必须有业务订阅者才会产生后续动作。与项目某些业务中的`/click_point`不是同名话题 |
| 单点导航任务 | RViz调用；Nav2 Goal工具 / Navigation 2 Panel | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 提交目标PoseStamped、接收反馈和最终结果；不是简单发布`/goal_pose`就能替代整个Nav2 action |
| 多点导航任务 | Nav2面板 / action客户端 | `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | Jazzy接口中的一组目标poses；具体面板支持和任务方式以运行版本为准 |
| 任务反馈及状态 | Panel / action客户端；不是Add显示 | `/navigate_to_pose/_action/feedback`、`.../_action/status` | 自动生成的反馈消息、`action_msgs/msg/GoalStatusArray` | action还包含发送/取消/取结果服务；可用`ros2 action info`查看，不应只关注这些底层话题 |
| 速度控制命令 | 控制器/遥控发布，底盘/模拟器订阅 | `/cmd_vel` | 本项目`geometry_msgs/msg/Twist` | vx/vy单位m/s、wz单位rad/s；是命令，不是测量值。当前仿真显式关闭stamped cmd_vel；其他配置可能使用TwistStamped |
| 本项目仿真速度桥接 | adapter发布，Gazebo订阅 | `/sim/cmd_vel` | `geometry_msgs/msg/Twist` | `/cmd_vel`经过adapter转到仿真桥接；不是RobotModel的直接输入，当前合成定位演示不需要此话题 |
| 仿真时钟 | Gazebo/回放时钟源发布，ROS节点使用 | `/clock` | `rosgraph_msgs/msg/Clock` | 模拟时间；使用它的节点及RViz需要`use_sim_time=true`。当前合成演示用墙钟、false，不需要clock |
| 节点生命周期 | 生命周期管理器 / 工具调用服务 | `/map_server/change_state`、`.../get_state`等 | `lifecycle_msgs/srv/...` | 地图/定位/规划节点配置、激活后才具备对应功能；Add一个Map不会自动激活map_server |
| 机器人描述参数 | 启动时传给robot_state_publisher | `robot_description`参数 | 参数值为URDF字符串 | 参数和`/robot_description`话题是两种接口；模型描述节点通常同时提供参数与描述话题，RViz按Description Source读取 |

RViz不会仅因为收到`/cmd_vel`就自动计算位移；需要模拟器或真实底盘执行并产生反馈，再由odom/TF驱动显示。自定义离线演示则可直接生成odom和TF。

## 5. 坐标关系与唯一发布者

```text
map
 └─ odom                         动态：定位系统
     └─ base_link                动态：里程计系统
         ├─ laser_front_link     静态：实机前雷达安装关系
         ├─ laser_back_link      静态：实机后雷达安装关系
         ├─ camera_link          如有相机，依安装配置
         │   └─ camera_optical_frame
         └─ 机械臂 / 其他link     固定或活动关节：URDF + 关节状态
```

上图雷达frame是实机命名；Gazebo仿真以URDF和实际消息为准。相机光学坐标通常采用X向右、Y向下、Z向前，不能直接把底盘轴约定套到光学frame。

| 关系 | 表示什么 | 常见发布者 | 当前项目约定 | 检查重点 |
| --- | --- | --- | --- | --- |
| `map → odom` | 将连续但可能漂移的odom定位到地图中 | AMCL、SLAM或外部定位节点三选一 | Gazebo导航为AMCL；仙工实机为seer_localization；合成演示为注入假源的定位节点 | 同一运行系统只能有一个所有者；不要再加静态map→odom |
| `odom → base_link` | 连续局部位姿 | 轮式里程计、融合节点或模拟器 | 实机seer_odometry；Gazebo桥接；合成演示的假odom | 与`/odom`位姿、帧和时间一致，重定位不清零odom |
| `base_link → 雷达` | 传感器安装外参 | robot_state_publisher或static_transform_publisher | 实机来自lidar_mounts配置 | 不能重复发布；m/rad或配置指定单位明确，安装偏差会表现为墙面对不齐 |
| `base_link → 其他模型link` | 模型内部几何和关节位姿 | robot_state_publisher | URDF和JointState | 活动关节缺状态时相关link可能缺TF；预览零位不等于实际反馈 |

`map`、`odom`、`base_link`是**坐标系名称，不是要求存在的同名话题**。`/odom`是里程计消息话题，两者名字相似但不是一回事。TF消息同时包含平移和旋转，不能只改frame_id或简单相减x/y/yaw完成坐标转换。

关键区别：发布`/odom`不等于自动发布TF；发布Pose不等于自动产生map→odom；URDF不等于整车实时定位；静态地图不等于定位系统。

## 6. 本项目三个入口实际提供什么

| 内容 | 离线定位演示 `seer_localization_demo.launch.py` | Gazebo导航 `agv_navigation_sim.launch.py` | 仙工只读显示 `seer_localization_view.launch.py` |
| --- | --- | --- | --- |
| 所在包 | agv_bringup | agv_bringup | robot_move |
| 数据来源 | 数学合成位姿，地图仅作背景 | Gazebo传感器/里程计 + AMCL + Nav2 | 仙工1004/1021/1000，结合已有odom TF |
| 地图 | 地图服务器 `/map` | 生成的仿真场景地图，或显式指定地图 | 指定转换后的ROS地图 |
| 机器人描述 | `/localization_demo/robot_description` | `/robot_description` | 本入口不启动模型发布；已有节点提供，或独立启动 |
| 关节状态 | `/localization_demo/joint_states`，零位预览 | 以仿真模型/发布节点实际配置为准 | 本入口不提供；需要模型活动关节时沿用现有反馈 |
| odom | 合成 `/odom`，50Hz目标 | Gazebo `/sim/odom`桥接为`/odom` | 本入口不启动；独立seer_odometry提供 |
| map→odom | 正式定位计算节点 + 假定位源 | AMCL | 仙工定位节点；质量/时差不通过则停更 |
| 雷达 | 无，未伪造墙面点 | `/sim/scan_raw`→adapter→`/scan` | 本入口不启动；独立lidar提供`/scan_front`、`/scan_back` |
| 雷达安装TF | 演示仅加载CAD模型，未加入实机雷达挂载 | 仿真URDF中的传感器安装关系 | 已有安装TF；缺少时另启，避免重复 |
| 全局/局部路径 | 无 | Nav2产生`/plan`及控制器对应局部路径 | 无，未启动Nav2 |
| 代价地图/轮廓 | 无 | Nav2 costmap提供 | 无 |
| 初始位姿/导航目标 | 不使用 | AMCL初始位姿、Nav2 action | 不使用；不能由RViz直接操作仙工重定位 |
| 时钟 | wall clock，use_sim_time=false | `/clock`，use_sim_time=true | wall clock，use_sim_time=false |
| 默认ROS域 | 217，仅本机发现 | 187，仅本机发现 | 不强改域；所有参与节点统一设置 |
| 对应RViz文件 | [seer_localization_demo.rviz](../src/agv_bringup/config/seer_localization_demo.rviz) | [agv_navigation_sim.rviz](../src/agv_bringup/config/agv_navigation_sim.rviz) | [seer_localization.rviz](../src/robot_move/config/seer_localization.rviz) |

“入口提供”与“RViz默认已经添加”也不同：例如Gazebo入口运行全局costmap，但当前RViz配置默认只添加局部costmap，其他项可手动Add。关闭某个Display只停止其显示，不会自动停止发布节点或导航任务。

## 7. 数据格式与显示配置速查

| 项目 | 正确含义 / 规则 | 常见错误 |
| --- | --- | --- |
| `header.frame_id` | 此条数据的坐标参考系 | 数据在雷达系，却直接标成map，未做变换 |
| `header.stamp` | 数据对应时刻；多个数据源需使用一致时钟 | 给历史数据反复盖当前时间；ROS时间/设备未知时钟混用 |
| 二维yaw | 单位rad，绕Z轴旋转；四元数`x=y=0,z=sin(yaw/2),w=cos(yaw/2)` | 把角度数值直接填进orientation.z；全零四元数无效 |
| 长度和速度 | 通常m、m/s；角速度rad/s | 把mm当m，或把度当rad |
| LaserScan数组 | 第i束方向=`angle_min+i×angle_increment`，距离为`ranges[i]` | 把距离数组索引直接当角度；安装TF旋转错误 |
| OccupancyGrid数据 | 长度=`width×height`；索引通常`i=y×width+x`；Nav2地图常用0空闲、100占用、-1未知 | 把PNG/PGM的0～255像素直接原样当占用栅格；把YAML origin再加到定位数据上 |
| 地图origin | 栅格坐标相对header.frame_id的位姿，包含旋转 | 将图像行翻转误当世界坐标也必须翻转 |
| Path | 带header和一系列PoseStamped，有序路径 | 只填点但不填有效姿态/坐标系；将规划路线当实走轨迹 |
| Marker | 合法pose、合适scale、颜色alpha>0、管理ns/id和lifetime | alpha=0、尺度为0、反复覆盖同一ID却以为会保留历史 |
| Fixed Frame | 导航总览通常为map；仅里程计观察可用odom | 设为base_link后机器人看似不动，误以为没有里程计 |
| 地图/模型描述QoS | 常用Reliable + Transient Local | 后打开RViz读不到先前只发布过的描述/地图 |
| 雷达/点云QoS | 常用Best Effort + Volatile，以实际发布者为准 | RViz要求Reliable，而发布者只提供Best Effort，无法匹配 |
| TF时间 | 消息时刻需要有可用TF，动态TF应持续更新 | TF链存在但只有很旧的数据；消息时间落在缓存范围外 |
| 模拟时间 | Gazebo节点与RViz通常统一true；实机和本项目合成演示为false | true却没有`/clock`导致时间停在零；一半节点true、一半false |
| ROS域与发现 | 发布者、订阅者需在同一ROS_DOMAIN_ID及兼容发现配置下 | 一个终端domain=217，另一个仍在0；把仿真与实机混到同域 |

## 8. 推荐添加顺序与只读排查命令

先在目标ROS域启动相应的数据源，再打开RViz：

1. Fixed Frame设为map；Add → Map → 选择`/map`。
2. Add → TF，先确认map→odom→base_link连通；Add → Axes，Reference Frame选base_link。
3. Add → RobotModel，选择该入口实际的robot_description话题。
4. Add → Odometry，选`/odom`；需要定位箭头则Add → Pose，选仙工Pose话题，或用PoseWithCovariance选AMCL输出。
5. 实机前后雷达各Add一个LaserScan；Gazebo用`/scan`。检查安装frame和Best Effort。
6. 已启动Nav2时，再加全局/局部Path、两个costmap、footprint、AMCL粒子；没有发布者就先不加。
7. File → Save Config As保存自己的`.rviz`，以后用`rviz2 -d 文件路径`加载。

以下命令只读取接口，不发速度或导航任务：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=217  # 改成当前数据源实际使用的域

ros2 topic list -t
ros2 topic info /map --verbose
ros2 topic info /scan_front --verbose
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 interface show sensor_msgs/msg/LaserScan
ros2 interface show nav_msgs/msg/OccupancyGrid
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show tf2_msgs/msg/TFMessage
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
ros2 action list -t
ros2 action info /navigate_to_pose
```

雷达只有Best Effort发布者时，echo也可显式加`--qos-reliability best_effort`。`ros2 topic hz`是观察端收到的频率，受QoS、网络和本机负载影响，不直接等同设备真实采样频率。没有运行数据源时，topic/TF/action命令可能等待或提示不存在，这不是RViz配置文件生成数据失败。

## 9. 项目与官方依据

本项目实际名称、入口差异依据以下本地文件核对；可选话题是接口示例，不宣称当前都在运行：

- [Gazebo启动入口](../src/agv_bringup/launch/agv_navigation_sim.launch.py)、[Nav2仿真参数](../src/agv_bringup/config/agv_nav2_sim.yaml)、[仿真adapter](../src/agv_bringup/agv_bringup_sim/adapter.py)。
- [离线定位演示入口](../src/agv_bringup/launch/seer_localization_demo.launch.py)、[合成数据实现](../src/robot_move/src/localization/demo.cpp)。
- [仙工定位接入](仙工定位接入.md)、[连续里程计接入](连续里程计接入.md)、[项目总接口清单](话题清单.md)。旧总接口清单基于历史静态整理，最新定位/仿真内容优先参考本页及对应源码。

官方参考：

- [ROS2 RViz使用指南源码](https://github.com/ros2/ros2_documentation/blob/rolling/source/Developer-Tools/Visualization/RViz/RViz-User-Guide/RViz-User-Guide.rst)：Display、Fixed Frame和交互工具的概念；该页为rolling，具体插件可用性以本机Jazzy为准。
- [Nav2 Jazzy默认RViz配置](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/rviz/nav2_default_view.rviz)：Jazzy的Nav2路径、costmap、footprint、粒子显示的配置示例。
- [Nav2 Jazzy AMCL实现](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp)：AMCL位姿、粒子发布与初始位姿输入接口。

本机插件名称另核对`/opt/ros/jazzy/share/rviz_default_plugins/plugins_description.xml`，消息结构以`ros2 interface show`或已安装包的`.msg`文件为准。不同ROS/Nav2版本可能有话题类型和命名差异，不能直接照搬其他发行版截图。
