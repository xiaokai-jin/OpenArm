# OpenArm Hardware Interface 说明

本文档对应以下两个文件：

- `include/openarm_hardware/openarm_hardware.hpp`
- `src/openarm_hardware.cpp`

目标：帮助你快速理解代码结构、数据流和控制算法修改入口。

---

## 1. 这个模块在系统中的位置

`OpenArmHW` 是一个 `ros2_control` 的 `SystemInterface` 插件。它位于 **控制器（Controller）** 和 **电机驱动（CAN + MotorControl）** 中间。

- 上游（Controller）负责生成控制命令：位置、速度、前馈力矩、KP、KD。
- 下游（MotorControl）负责把命令编码并发送到电机。
- `OpenArmHW` 负责做“接口绑定 + 周期读写 + 安全检查 + 量纲映射”。

简化数据流：

1. Controller 写入 command interfaces（`pos_commands_` 等）。
2. `write()` 读取这些命令并调用 `controlMIT(...)` 下发。
3. 电机返回反馈。
4. `read()` 读取反馈写入 state interfaces（`pos_states_` 等）。
5. Controller 再读取 state interfaces 做下一轮控制。

---

## 2. `openarm_hardware.hpp` 结构解释

### 2.1 全局配置常量

- `motor_types/can_device_ids/can_master_ids`：定义每个关节的电机型号和 CAN 地址。
- `ARM_DOF/GRIPPER_DOF/TOTAL_DOF`：自由度配置。
- `DEFAULT_KP/DEFAULT_KD`：默认刚度和阻尼。
- `START_POS_TOLERANCE_RAD`：激活阶段位置收敛阈值。
- `POS_JUMP_TOLERANCE_RAD`：运行期防突跳阈值。
- 夹爪相关常量用于将电机角度与夹爪开合量互相转换。

### 2.2 `OpenArmHW` 成员函数职责

- `on_init(...)`：解析参数、创建 CAN 和电机对象、初始化缓存。
- `on_configure(...)`：配置阶段执行零位设置。
- `export_state_interfaces()`：导出状态接口（position/velocity/effort）。
- `export_command_interfaces()`：导出命令接口（position/velocity/effort/stiffness/damping）。
- `on_activate(...)`：使能电机并平滑过渡到目标位置。
- `on_deactivate(...)`：停机归零并失能。
- `read(...)`：读取底层状态。
- `write(...)`：下发控制命令。

### 2.3 关键缓存变量

- 命令缓存：`pos_commands_`, `vel_commands_`, `tau_ff_commands_`, `kp_commands_`, `kd_commands_`
- 状态缓存：`pos_states_`, `vel_states_`, `tau_states_`

它们是 `ros2_control` 与硬件实现之间的共享内存桥接。

---

## 3. `openarm_hardware.cpp` 执行流程解释

## 3.1 `on_init()`

主要做 6 件事：

1. 调用父类初始化。
2. 读取 `can_device/prefix/disable_torque` 参数。
3. 构建 `CANBus` 和 `MotorControl`。
4. 按配置创建 `Motor` 实例并注册。
5. 初始化所有状态/命令数组。
6. 赋默认 KP/KD，并做一次刷新和读状态。

## 3.2 `on_configure()`

- 先 `read()`，再对每个电机 `set_zero_position()`。
- 等价于把当前姿态定义为软零位。

## 3.3 `on_activate()`

- 先 `enable()` 全部电机。
- 进入 while 循环，按步长限制逐步逼近 `pos_commands_`，避免激活瞬间跳变。
- 当全部关节误差小于 `START_POS_TOLERANCE_RAD` 后激活完成。

## 3.4 `read()`

- 机械臂 7 轴直接读取位置/速度/力矩。
- 夹爪额外进行传动半径和方向换算后写入状态。

## 3.5 `write()`

- 若 `disable_torque_ = true`，下发全零控制量。
- 否则先做位置突跳保护（大于 `POS_JUMP_TOLERANCE_RAD` 直接报错）。
- 然后调用 `motor_control_->controlMIT(...)` 下发。
- 夹爪命令需先做末端量到电机量反算。

---

## 4. 我要改控制算法，应该改哪里？

先明确你想改的是哪一层算法：

### A. 改“高层控制算法”（推荐）

例如：阻抗控制、导纳控制、笛卡尔空间控制、轨迹跟踪律等。

**推荐位置：Controller 层（不是本文件）**。

原因：

- `OpenArmHW` 的职责是硬件抽象和数据转发，不适合放复杂控制律。
- 在 Controller 层可复用 ROS2 控制框架，并更好地调参和切换算法。

你要做的是在控制器中生成并写入：

- `position`
- `velocity`
- `effort`（前馈）
- `stiffness`（KP）
- `damping`（KD）

这些会被本硬件接口透传到底层 MIT 命令。

### B. 改“驱动下发策略/保护策略”（在本文件可改）

这类改动建议在 `write()` 中做：

- 指令滤波（低通、斜坡、限幅、限速）
- 安全保护（软限位、温升降额）
- MIT 参数映射或模式切换

关键位置：

- `OpenArmHW::write(...)` 内部 `controlMIT(...)` 调用前后。

### C. 改“电机协议级控制模式”

如果想改成位置模式、速度模式或纯力矩模式，需要改：

- `motor_control.hpp/.cpp` 中的具体协议封包和控制接口；
- 再在 `write()` 中替换 `controlMIT(...)` 调用。

---

## 5. 快速改造建议（实操顺序）

1. 先在 Controller 层实现新算法，输出 `pos/vel/tau/kp/kd`。
2. 在 `write()` 前增加限幅和速度斜坡。
3. 保留 `POS_JUMP_TOLERANCE_RAD` 保护并逐步调大阈值。
4. 最后再考虑改 `MotorControl` 协议层。

---

## 6. 备注

当前代码里注释已补到 `.hpp/.cpp`，可直接按函数定位阅读。若你愿意，我可以下一步帮你：

- 给 `write()` 增加“可开关的一阶低通 + 速度限幅”模板实现；
- 或帮你生成一个最小自定义 Controller 框架（直接写 stiffness/damping/effort）。
