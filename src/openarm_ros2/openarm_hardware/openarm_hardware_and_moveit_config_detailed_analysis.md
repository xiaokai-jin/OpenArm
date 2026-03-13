# OpenArm ROS2 硬件层与 MoveIt 配置详解

本文档分两部分：
1. `openarm_hardware` 包内每个头文件与源文件的详细职责与实现逻辑。
2. `openarm_bimanual_moveit_config` 中哪些是 MoveIt Setup Assistant 生成、哪些是输入给 Assistant 的资产。

---

## 1. openarm_hardware 包总体架构

`openarm_hardware` 是一个 **ros2_control 硬件插件包**，核心目标是把底层 CAN 电机控制映射成 MoveIt / ros2_control 可用的标准接口。

### 1.1 运行链路（宏观）

- MoveIt 生成轨迹
- 轨迹发送给 `joint_trajectory_controller` / `gripper_controller`
- controller_manager 调用 `openarm_hardware::OpenArmHW::write()` 下发命令
- `OpenArmHW` 内部调用 `MotorControl` 打包协议并通过 `CANBus` 发到电机
- 电机反馈通过 `CANBus` 收回，经 `MotorControl` 解包并写入 `Motor`
- `OpenArmHW::read()` 把状态回传给 ros2_control

### 1.2 插件注册与加载

- 插件描述：`openarm_hardware.xml`
- 插件类名：`openarm_hardware/OpenArm_v10HW`
- C++ 类型：`openarm_hardware::OpenArmHW`
- 基类：`hardware_interface::SystemInterface`

在 launch + URDF/xacro 的 `<ros2_control><hardware><plugin>...</plugin></hardware>` 中配置后，controller_manager 会动态加载该插件。

---

## 2. 头文件逐一解析（定义了什么）

### 2.1 `include/openarm_hardware/openarm_hardware.hpp`

#### 作用
定义 ros2_control 硬件系统类 `OpenArmHW`，并集中放置硬件常量与控制参数。

#### 关键定义

- 全局静态配置
  - 电机型号列表 `motor_types`
  - 电机 CAN 从机 ID `can_device_ids`
  - 电机 CAN 主机 ID `can_master_ids`
  - 控制模式 `CONTROL_MODE`（MIT）
  - 关节数量 `ARM_DOF / GRIPPER_DOF / TOTAL_DOF`
  - 每关节增益 `KP[] / KD[]`
  - 安全阈值：`START_POS_TOLERANCE_RAD`、`POS_JUMP_TOLERANCE_RAD`
  - 夹爪换算参数：`GRIPPER_REFERENCE_GEAR_RADIUS_M`、`GRIPPER_GEAR_DIRECTION_MULTIPLIER`

- `OpenArmHW` 类
  - 生命周期回调：
    - `on_init()`
    - `on_configure()`
    - `on_activate()`
    - `on_deactivate()`
  - 接口导出：
    - `export_state_interfaces()`
    - `export_command_interfaces()`
  - 周期读写：
    - `read()`
    - `write()`

- 内部成员
  - `CANBus`、`MotorControl`、`Motor` 容器
  - `pos/vel/effort` 状态与命令缓存
  - `disable_torque_` 等运行参数

#### 设计特点

- 该头文件把“机械臂结构参数 + 控制参数”与“插件接口定义”放在一起，便于单文件查阅，但耦合较高。

---

### 2.2 `include/openarm_hardware/canbus.hpp`

#### 作用
对 Linux SocketCAN 提供轻量封装，屏蔽 classic CAN 与 CAN-FD 的细节差异。

#### 关键定义

- 枚举 `CANMode`
  - `CAN_MODE_CLASSIC`
  - `CAN_MODE_FD`

- 类 `CANBus`
  - 构造：打开 socket、绑定网卡、可选启用 CAN-FD
  - `send(motor_id, data)`：统一发送接口
  - `recv(out_id, out_len)`：统一接收接口
  - `whichCAN()`：当前模式

- 私有函数
  - `sendClassic()` / `sendFD()`
  - `recvClassic()` / `recvFD()`

---

### 2.3 `include/openarm_hardware/motor.hpp`

#### 作用
定义电机对象模型、协议枚举以及协议编码/解码工具函数。

#### 关键定义

- 枚举
  - `DM_Motor_Type`：DM4310/DM4340/DM8009 等型号
  - `DM_variable`：电机寄存器参数标识（例如 `CTRL_MODE`）
  - `Control_Type`：MIT、位置速度、速度、力矩位置等

- 类 `Motor`
  - 身份字段：`SlaveID`、`MasterID`、`MotorType`
  - 状态字段：位置/速度/力矩/温度
  - 目标字段：`goal_position`、`goal_velocity`、`goal_tau`
  - 参数缓存：`temp_param_dict`

- 协议工具函数
  - `double_to_uint / uint_to_double`
  - `float_to_uint8s / uint8s_to_float`
  - `data_to_uint8s / uint8s_to_uint32`

---

### 2.4 `include/openarm_hardware/motor_control.hpp`

#### 作用
封装具体协议控制逻辑，是“电机对象层”和“CAN 传输层”之间的中间控制层。

#### 关键定义

- 电机管理
  - `addMotor()`：注册 motor ID 与 `Motor*` 映射

- 电机基础控制
  - `enable()` / `disable()` / `set_zero_position()`

- 控制模式命令
  - `controlMIT()`（主流程）
  - `controlPosVel()` / `controlVel()` / `controlPosForce()`

- 数据路径
  - `sendData()`
  - `recv()`：接收并分发给 `processPacket` / `processPacketFD`

- 参数读写
  - `switchControlMode()`
  - `writeMotorParam()` / `readRIDParam()`

- 静态限幅参数
  - `Limit_Param[12][3] = {Q_MAX, DQ_MAX, TAU_MAX}`，按电机型号取值

---

### 2.5 `include/openarm_hardware/visibility_control.h`

#### 作用
定义导出符号可见性宏（Windows/Linux），保证插件类符号可被动态加载。

---

## 3. 源文件逐一解析（如何实现）

### 3.1 `src/canbus.cpp`

#### 实现要点

- 构造函数
  1. `socket(PF_CAN, SOCK_RAW, CAN_RAW)` 创建 CAN 套接字
  2. `ioctl(SIOCGIFINDEX)` 解析网卡索引
  3. 若 FD 模式：`setsockopt(... CAN_RAW_FD_FRAMES ...)`
  4. `bind()` 到具体 CAN 接口

- 发送
  - classic：构造 `can_frame`
  - FD：构造 `canfd_frame`（含 BRS）

- 接收
  - classic：读 `can_frame`
  - FD：读 `canfd_frame`

#### 注意点

- 失败策略是 `perror + exit`，属于强硬退出。
- 当前没有超时、重试、统计指标等机制。

---

### 3.2 `src/motor.cpp`

#### 实现要点

- `Motor` 构造时初始化全部状态变量
- `recv_data()` 负责把解包后的 `q/dq/tau/temp` 写入状态缓存
- getter/setter 是薄封装
- 协议工具函数完成：
  - 浮点量到整数位宽的量化
  - 字节序列转换

#### 注意点

- `uint8s_to_double()` 用 4 字节拷贝到 `double`（8 字节类型）逻辑可疑，若被使用会有精度/未定义风险；当前主链路主要用 `uint_to_double`。

---

### 3.3 `src/motor_control.cpp`

#### 实现要点

1. **控制帧打包（MIT）**
   - `controlMIT()` 根据电机型号限幅参数把 `q/dq/tau/kp/kd` 量化
   - 按协议位拼接到 8 字节
   - 发送后立即 `recv()` 刷新状态

2. **反馈帧解包**
   - `recv()` 根据 CAN 模式创建 frame
   - 调 `processPacket()` 或 `processPacketFD()`
   - 反量化成实际物理量写回 `Motor::recv_data()`

3. **控制命令**
   - `controlCmd(..., 0xFC/0xFD/0xFE)` 对应使能/失能/置零

4. **参数写入**
   - `writeMotorParam()` 通过 `0x7FF` 管理帧写寄存器

#### 注意点

- 反馈 `cmd` 当前写死 `0x11`（代码注释里也标了 someday fix）。
- 部分函数有 `...2` 版本（只发不收），用于高频场景避免阻塞。

---

### 3.4 `src/openarm_hardware.cpp`

#### 实现要点（ros2_control 适配核心）

1. `on_init()`
   - 校验 `can_device` 参数
   - 构造 `CANBus` + `MotorControl`
   - 构造所有 `Motor` 对象并注册
   - 初始化 command/state 缓冲

2. `export_state_interfaces()`
   - 每关节导出 position/velocity/effort 三个状态接口

3. `export_command_interfaces()`
   - 每关节导出 position/velocity/effort 三个命令接口

4. `on_activate()`
   - 使能所有电机
   - 把状态逐步拉向命令值（防突跳策略）

5. `read()`
   - 机械臂关节：直接读 `Motor` 缓存
   - 夹爪：做电机角度与夹爪位移换算（半径与方向）

6. `write()`
   - 先检查关节目标突跳
   - 机械臂 7 轴按 `KP/KD` MIT 控制
   - 夹爪按线位移↔电机角度换算后下发

#### 注意点

- `disable_torque_` 分支中的 `for` 循环内立即 `return`，只会执行第一个电机，逻辑存在缺陷。
- `on_configure()` 中默认会对所有轴执行 `set_zero_position()`，这会直接影响你之前遇到的“零位漂移/状态不一致”现象。

---

## 4. 构建文件如何把插件接入系统

### 4.1 `CMakeLists.txt`

- 构建共享库：`openarm_hardware`
- 源文件：`openarm_hardware.cpp + canbus.cpp + motor.cpp + motor_control.cpp`
- 依赖：`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`
- `pluginlib_export_plugin_description_file(hardware_interface openarm_hardware.xml)` 导出插件元信息

### 4.2 `openarm_hardware.xml`

声明 `openarm_hardware/OpenArm_v10HW -> openarm_hardware::OpenArmHW` 的插件映射，供 controller_manager 动态加载。

---

## 5. MoveIt 配置：哪些是 Assistant 生成，哪些是输入

## 5.1 明确证据来源

`.setup_assistant` 显示：

- 输入 URDF：`openarm_description/urdf/robot/v10.urdf.xacro`
- 输入参数：`xacro_args: bimanual:=true`
- 输出 SRDF：`config/openarm_bimanual.srdf`

这说明该 moveit_config 包是基于 `openarm_description` 生成的。

### 5.2 “传给 MoveIt Assistant 的”输入资产

主要输入来自 `openarm_description`：

- 机器人几何与关节定义（URDF/Xacro）
- 关节命名、关节限制、末端执行器拓扑
- xacro 参数（例如 `bimanual:=true`）

在 `openarm_bimanual_moveit_config` 包内，与输入关系直接相关的是：

- `.setup_assistant`（记录输入源与生成配置）
- `config/openarm_bimanual.urdf.xacro`（用于 MoveIt 侧加载描述的包装）

### 5.3 “由 MoveIt Assistant 生成的”典型产物

- `config/openarm_bimanual.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/moveit_controllers.yaml`
- `config/ros2_controllers.yaml`（初始模板通常由 Assistant 生成，后续常被手改）
- `config/initial_positions.yaml`
- `config/moveit.rviz`
- `launch/move_group.launch.py`
- `launch/moveit_rviz.launch.py`
- `launch/rsp.launch.py`
- `launch/spawn_controllers.launch.py`
- `launch/static_virtual_joint_tfs.launch.py`
- `launch/setup_assistant.launch.py`
- `launch/warehouse_db.launch.py`
- `package.xml`（描述里也写了 automatically generated）

### 5.4 明确“后续手工修改”的文件

- `launch/demo.launch.py`：明显为集成真实硬件/控制器加载顺序做过多次手改，不是 Assistant 原始模板。
- `config/ros2_controllers.yaml`：当前文件中存在重复段与注释残留，属于后续人工改动痕迹。

---

## 6. 你当前系统里最关键的实践建议

1. `openarm_hardware` 负责“硬件接口标准化”，它是 MoveIt 执行链路的桥。
2. `openarm_can`（独立包）更适合底层调试、标零、单机控制；`openarm_hardware` 是 ROS2 适配层。
3. 涉及零位策略时，优先明确：
   - 是否允许 `on_configure` 自动 `set_zero`
   - 夹爪线位移与电机角度换算方向是否与 SRDF 命名状态一致
4. `moveit_config` 中 `demo.launch.py` 与 `ros2_controllers.yaml` 建议保持最小改动并版本可追溯。

---

## 7. 附：快速定位入口

- 硬件插件入口类：`OpenArmHW`（`openarm_hardware.hpp/.cpp`）
- 协议打包入口：`MotorControl::controlMIT()`
- CAN 收发入口：`CANBus::send()` / `CANBus::recv()`
- MoveIt 语义状态：`config/openarm_bimanual.srdf` 中 `group_state`
- Assistant 元数据：`.setup_assistant`

---

如果你愿意，下一步我可以基于本文档再给你补一版“时序图”（on_init→on_activate→read/write）和“夹爪状态 open/half_closed/closed 的数值-物理含义对照表”。