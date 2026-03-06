# OpenArm 控制代码实例解析 

本文档详细解析了 `openarm/src/openarm_can/examples/demo.cpp` 代码的功能、原理及其实现方式，并为扩展功能提供了参考建议。 

## 1. 代码总体概述

该 `demo.cpp` 是 **OpenArm CAN 通信库的官方示例程序**，通过系统级的方法演示了如何配置并控制 OpenArm 以及其各个组件。它涉及：
1. **CAN/CAN-FD 总线的配置与建链**。
2. **硬件设备的注册与映射**（建立软件中表示电机对象的组件：Arm 机械臂、Gripper 夹爪等与物理通信地址映射）。
3. **电机状态机控制与信息读取**（Enable / Disable、参数查询等操作）。
4. **MIT 模式的力矩/阻抗混合控制与位置获取**。

---

## 2. 详细代码解析

以下是逐行对关键逻辑与核心类库方法的解释。

### 2.1 初始化与总线建立

```cpp
// Initialize OpenArm with CAN interface and enable CAN-FD
std::cout << "Initializing OpenArm CAN..." << std::endl;
openarm::can::socket::OpenArm openarm("can0", true);  // Use CAN-FD on can0 interface
```
* **功能**：初始化 `OpenArm` 高级抽象库，内部封装了对底层的 `canbus::CANSocket` 和整个网络的 `CANDeviceCollection` 的调用。
* **参数解释**： 
   - `"can0"`：连接到硬件系统的宿主机的 CAN 网卡驱动名，通常配置于 Linux 系统的 SocketCAN 接口。
   - `true`：开启 **CAN-FD**（CAN with Flexible Data-Rate），以保证每条消息的 Payload 支持更大带宽（不仅限传统 8 Byte）或更高波特率，增强网络整体响应速度。

### 2.2 初始化机械臂电机 (Arm Motors)

```cpp
std::vector<openarm::damiao_motor::MotorType> motor_types = {
    openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310};
std::vector<uint32_t> send_can_ids = {0x01, 0x02};
std::vector<uint32_t> recv_can_ids = {0x11, 0x12};
openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);
```
* **功能**：批量向底层注机械臂组件包含的达妙（DaMiao）电机列表。
* **原理**：`init_arm_motors` 会在 `ArmComponent` 结构里注册对应的 `MotorDeviceCan` 对象。每个电机通过两个 CAN ID来唯一区分其：一个是主机控制其的 ID（`send_can_ids`），另一个是它返回数据帧的主动心跳或查询 ACK 的 ID（`recv_can_ids`）。这样便于总线区分多主/多从设备。这里采用两种类型的同款电机 `DM4310` 进行测试。

### 2.3 初始化末端夹爪 (Gripper)

```cpp
std::cout << "Initializing gripper..." << std::endl;
openarm.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);
```
* **功能**：类似以上方法，调用独立的夹爪接口方法将其绑定并向控制网络注册。同样是 DM4310 的微型驱控集成电机。

### 2.4 设置通信回调模式

```cpp
openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
```
* **功能与原理**：底层对总线数据有回调映射。`CallbackMode` 为电机响应帧分配工作状态。
   - `IGNORE` 模式用于忽略从总线中收到的不必要干扰数据（因有些阶段仅侧重写入而不准备接收如开启失能响应、周期性心跳等）。

### 2.5 伺服使能与响应等待

```cpp
openarm.enable_all();
openarm.recv_all(2000);
```
* **功能**：广播总线，并向每个设备发起配置指令将其转换为 "Enabled" 工作模式，驱动板激活功率级输出。
* **原理**：指令下发后执行 `recv_all(2000)` 表示主程序阻塞等待来自所有注册节点 CAN 收发队列 `select()` 操作最多 2 毫秒来等待电机的返回包确认，避免并发指令导致 CAN 缓冲区塞满或时序混乱。

### 2.6 查询电机固化参数 (Register Parameter)

```cpp
openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::MST_ID));
openarm.recv_all(2000);
```
* **功能**：转为 `PARAM` 模式并发出指定寄存器 ID (`RID`) 数据查询指令。
* **原理**：达妙电机具有多种只读或读写配置参数。例如：`RID::MST_ID`（主机 ID 等）。切入 `PARAM` 解析模式，让底层对帧 `recv_all` 的解码器从系统解析参数并更新 `ParamResult`，而不是常规的位置和力矩信息。

### 2.7 打印设备反馈

```cpp
for (const auto& motor : openarm.get_arm().get_motors()) {
    std::cout << "Arm Motor: " << motor.get_send_can_id() << " ID: "
              << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
              << std::endl;
}
```
* **功能**：遍历抽象的对象并打印其发送 ID 及对应的在网络中读到的数据（MST_ID），确保上下位机成功通信并确认握手建立。

### 2.8 运动控制 (状态模式：STATE)

```cpp
openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

// Control arm motors with position control
openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{2, 1, 0, 0, 0},
                                   openarm::damiao_motor::MITParam{2, 1, 0, 0, 0}});
openarm.recv_all(500);

// Control arm motors with torque control
openarm.get_arm().mit_control_all({openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
                                   openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1}});
openarm.recv_all(500);
```
* **功能**：模式变更为最常用的 `STATE` 监控模式，该模式能将每次底层回发的包解码为 `[位置, 速度, 力矩, 报错状态等]` 的结构。
* **原理与核心结构：`MITParam`**：
  MIT 控制模式是足式/手部机械系统最流行的综合型控制指令，底层算法在驱动器内执行：
  **$\tau = K_p (q_{des} - q) + K_d (\dot{q}_{des} - \dot{q}) + \tau_{ff}$**
  该库定义的 `MITParam` 数据包含 5 个元素：`{ Kp, Kd, q(位置), dq(速度), tau(前馈力矩) }`。
  * **第一段控制 (位置调节)**：下发的 `Kp=2, Kd=1, q=0, dq=0, tau=0` 意味着提供一定的刚度($K_p$)与阻尼($K_d$)，使目标回到 0 点并保持制动。
  * **第二段控制 (力矩前馈调节)**：修改为 `{0,0,0,0,0.1}`，即去掉阻抗系数，直接让底层生成额外的 `0.1 Nm` 纯扭矩。这也是遥操作经常采用的模式（做动力学补充时下发对应关节需要的摩擦和重力阻抗）。 

### 2.9 控制夹爪与信息采样

```cpp
std::cout << "Closing gripper..." << std::endl;
openarm.get_gripper().close();
openarm.recv_all(1000);

for (int i = 0; i < 10; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    openarm.refresh_all();
    openarm.recv_all(300);

    for (const auto& motor : openarm.get_arm().get_motors()) {
        std::cout << "Arm Motor: " << motor.get_send_can_id()
                  << " position: " << motor.get_position() << std::endl;
    }
    // ...
}
```
* **功能**：控制末端夹爪并开启状态采样的 Demo 循环。
* **原理**：调用高级封装的 `close()` 闭合方法；随后开启 10 轮次采样（10Hz）：下发 `refresh_all()` 指令告诉执行机构发送他们当前的观测状态（位置/速度/扭矩等）。随之用 `get_position()` 进行数据提取打印。

### 2.10 失能与退出

```cpp
openarm.disable_all();
openarm.recv_all(1000);
```
* **功能**：完成作业后对电机进行停止掉电，防止过度发热；关闭所有内部资源并在作用域抛出阶段保证下位机关断执行状态。

---

## 3. 怎样进行扩展与二次开发？

如果您需要为当前的 OpenArm 添加控制节点或实现更加复杂的高级控制场景，您可以从以下几个方向修改：

### 3.1 扩展多组网络或更多电机节点
如果系统不仅是双自由度而是一个多度系统（比如添加左臂/右臂协调）：
* 在 `std::vector` 初始化中扩充你的 `MotorType` 以及发送/接收 CAN IDs。
* 例：
  ```cpp
  // 6轴机械臂扩展
  std::vector<openarm::damiao_motor::MotorType> arm_types(6, openarm::damiao_motor::MotorType::DM4310);
  std::vector<uint32_t> arm_s_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
  std::vector<uint32_t> arm_r_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};
  openarm.init_arm_motors(arm_types, arm_s_ids, arm_r_ids);
  ```

### 3.2 自定义伺服反馈读取控制（速度、力矩获取）
当前的 demo 循环只输出了位置信息，如要进行动力反馈，需要取得速度与当前力矩：
在 `refresh_all()` 与 `recv_all()` 发起采集后：
```cpp
// 在 for 循环内读取信息
double current_vel = motor.get_velocity();
double actual_torq = motor.get_torque();
std::cout << " Vel: " << current_vel << " Torq: " << actual_torq << std::endl;
```

### 3.3 替换或配置其它类 CAN 通信接口设备模式
当前设备是在 `"can0"` 工作通过原生的 Linux SocketCAN 设备并允许了 FD (Flexible Data-rate) 功能。
如果您的设备接在另一个 USB 转换 CAN 设备（如 `can1`）或不支持 FD 数据的低端 CAN，您可以：
```cpp
// 只需要更改初始化行即可
openarm::can::socket::OpenArm openarm("can1", false); // 禁用 FD，适用传统设备
```

### 3.4 集成轨迹发生器与逆运动学
可以在下发 `MITParam` 数据前，配合 `openarm_teleop` 下预置好的 `Dynamics` （KDL）库与轨迹算法，循环下发位置进行平滑点控（`q` 参数随时间变化，结合计算补偿力给到 `tau`，组成闭环控制）。