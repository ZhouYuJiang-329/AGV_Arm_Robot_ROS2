# robot_move

独立底盘驱动包。`robot_move_impl` 是可供其他包复用的 C++17 库，`robot_move_node` 是其 ROS 2 适配节点。

## 结构

- `interface/robot_move_base.hpp`：统一接口，支持多态销毁，禁止复制连接资源。
- `impl/agv/agv_robot_move`：移动参数、初始化策略和导航/定位等待流程。
- `impl/agv/seer/seer_client`：`IClient`类型化接口和真实`Client`，封装仙工API编号、JSON与业务字段解析。
- `impl/agv/seer/seer_transport`：`Transport`管理TCP连接、字节封装、完整收发及报文校验。
- `impl/agv/seer/seer_types`：位姿、状态枚举、通道和绝对截止时间。
- `impl/air`、`impl/mag`、`impl/duct`：其他底盘串口驱动。
- `factory`：按 `agv/air/mag/duct` 类型创建实例。
- `nodes`：话题适配、定时器和受管理的任务线程。

依赖方向为`AGVRobotMove → seer::IClient（真实实现Client）→ Transport → TCP`。新增类位于`robot_move::seer`命名空间，编译到同一个`robot_move_impl`库，不新增ROS包。AGV层不处理JSON或socket；Client只确认单次API调用，任务等待仍由AGV层管理。

`AGVRobotMove`默认构造真实客户端，也接受非空的`unique_ptr<robot_move::seer::IClient>`用于注入测试或其他兼容实现。Client接受由其独占持有的Transport；两种注入构造均拒绝空指针。

## 启动参数

```bash
ros2 run robot_move robot_move_node --ros-args -p robot_type:=agv -p connection:=你的机器人IP
```

将示例中的中文占位符替换为实际地址。串口底盘的 `connection` 是设备路径。未传入时使用工厂中的现有默认连接配置。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot_type` | `agv` | 底盘类型 |
| `connection` | 对应类型的现有默认值 | AGV IPv4地址或串口路径 |
| `agv_max_linear_speed` | `0.0` | AGV合成线速度上限，m/s；0表示未配置 |
| `agv_max_angular_speed` | `0.0` | AGV角速度绝对值上限，rad/s；0表示未配置 |
| `agv_init_set_do` | `true` | 初始化时是否设置DO输出 |
| `agv_init_do_id` | `0` | 初始化DO编号，非负int范围内整数 |
| `agv_init_do_status` | `true` | 初始化DO电平，true为高，false为低 |
| `agv_init_relocate` | `true` | 初始化时是否按当前位置执行重定位 |

参数在启动时读取。未知底盘类型、非法限速或初始化失败会退出并返回状态码1，不再留下一个没有可用驱动的空节点。

AGV初始化默认保留原有的DO0高电平输出和按当前位置重定位流程。四个服务端口全部连接成功，并且所有启用的初始化动作均成功，才报告初始化完成。DO是否对应抱闸取决于实际接线，因此接口命名为数字输出而不预设其物理用途。TCP连接尝试有1秒截止时间，重新初始化会先释放旧连接。

只连接、不执行初始化DO输出及重定位：

```bash
ros2 run robot_move robot_move_node --ros-args \
  -p robot_type:=agv -p connection:=你的机器人IP \
  -p agv_init_set_do:=false -p agv_init_relocate:=false
```

四个`agv_init_*`参数标记为只读，只能在启动时设置；即使关闭DO动作，DO编号仍须合法。可仅开启DO或仅开启重定位，两者都开启时始终先DO后重定位。关闭动作不会主动发送相反DO电平，也不会查询或确认重定位。

这些开关仅控制初始化动作。节点启动后的话题、定时位姿查询和原有退出处理仍会运行，并非全程只读模式。失败时清理连接，不会回滚已经设置的DO，也不会自动取消已经发出的重定位请求。

直接使用C++库时可设置`AGVRobotMove::InitOptions`并调用`setInitOptions()`，配置在下次`init()`时应用；非法配置会返回false并保留旧配置。也可独立调用：

```cpp
AGVRobotMove robot;
if (!robot.connect(ip)) { /* 连接失败 */ }
// connect()只连接，不发送DO或重定位命令。
// 需要时再独立调用，检查每一步的返回值：
// robot.setDigitalOutput(id, status);
// robot.relocate();
```

`connect()`与`init()`不可与其他操作并发执行。`setDigitalOutput()`对应6001，`relocate()`执行1004查询当前位置、2002重定位及1021状态等待；旧版需要时才调用2003确认。

## ROS接口与任务管理

订阅 `cmd_vel`、`scan_config`、`scan_ctrl`、`robot_distance_move`、`robot_rotate_move`，发布 `agv_pose`。

平移消息通过 `linear.z` 提供距离，`linear.x/y` 提供速度；旋转通过 `angular.z` 提供转角，`angular.y` 提供角速度。距离/转角必须为正，方向由速度正负表达。

节点只允许一个长任务执行，使用可join的线程管理其生命周期。新长任务在忙时直接拒绝，不累积队列；其他速度/配置回调在驱动锁被占用时记录未执行并返回，定时查询跳过本周期，不再阻塞整个ROS执行器。

退出时先请求结束本地等待，join任务线程，再执行原有的零速度与关闭连接流程。`cancelPendingOperations()`仅用于结束本地阻塞等待，不会发送导航取消或硬件急停。AGV的取消标志会持续到下次`init()`，不要把它当作可恢复的通用任务取消接口。驱动对象销毁前，外部调用者同样应结束并回收自己的调用线程。

四类驱动的析构函数保证释放连接。串口驱动析构只释放资源，原有显式`moveshutdown()`流程保留；重复关闭和初始化失败不再遗留可被重复关闭的句柄。重初始化不能与运动调用并发执行。

## 库复用

调用方CMake：

```cmake
find_package(robot_move REQUIRED)
target_link_libraries(your_target PRIVATE robot_move::robot_move_impl)
```

`package.xml`中声明`<depend>robot_move</depend>`。头文件入口为`robot_move/factory/robot_move_factory.hpp`，也可以直接包含具体驱动头。库已导出线程依赖并启用位置无关代码，可链接到共享库中。

ROS节点另有接收已初始化`unique_ptr<RobotMove>`的构造函数，供嵌入或无硬件测试使用；注入的驱动不会重新初始化，其连接和限速配置由调用者事先完成。

## 构建与测试

在工作区根目录执行：

```bash
colcon build --symlink-install --packages-select robot_move
colcon test --packages-select robot_move --ctest-args -R '^(seer_.*_test|agv_.*_test|driver_lifecycle_test|driver_node_test)$' --output-on-failure
colcon test-result --verbose
```

- `seer_transport_test`：协议帧、字节序、半包/连续帧、响应匹配、完整发送、超时、并发事务和连接生命周期。
- `seer_client_test`：所有已用API的通道/编号/请求字段、响应解析、错误码、空确认、无效字段和截止时间传播。
- `agv_robot_move_test`：通过假IClient验证参数、限速、初始化顺序、导航/定位等待、本地取消、失败清理与客户端生命周期。
- `agv_integration_test`：真实三层实现连接本地模拟服务，验证查询、DO、移动成功/失败传播及多态资源释放。
- `driver_lifecycle_test`：使用伪终端和`/dev/null`检查串口失败清理、重复初始化、重复关闭和多态析构。
- `driver_node_test`：注入假驱动检查任务互斥、回调不阻塞和退出时线程回收。

这些测试不连接真实机器人。现场运动、串口协议、电气映射仍需实机验证。当前未增加停车策略或自动重连，也未替换`agv_hardware`中的旧底盘实现。

## 分层后的错误边界

Transport只校验报文，不解析JSON。Client负责JSON和`ret_code`：无效JSON使对应通道关闭，完整的API错误响应保留连接，缺少查询字段返回失败且不部分修改输出。空正文可作为命令确认，不可作为有效位姿或状态。

Client将响应解析/通道失效包含在串行调用锁中，Transport内部也保留事务锁，避免直接使用Transport时发生交叉收发。两层共用同一绝对截止时间，不会各自重新给予完整的1秒等待。既有协议版本、超时、初始化默认值和动作策略保持。
