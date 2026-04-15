// Copyright 2025 Reazon Holdings, Inc.
// Copyright 2025 Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "canbus.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motor.hpp"
#include "motor_control.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

// -----------------------------
// Hardware static configuration
// -----------------------------
// 机械臂 7 个关节的默认电机型号（与实际关节一一对应）
std::vector<DM_Motor_Type> motor_types{
    DM_Motor_Type::DM8009, DM_Motor_Type::DM8009, DM_Motor_Type::DM4340,
    DM_Motor_Type::DM4340, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310,
    DM_Motor_Type::DM4310};
// 每个关节电机在 CAN 总线上的 device id
std::vector<uint16_t> can_device_ids{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
// 每个关节对应的 master id（主控侧发送 id）
std::vector<uint16_t> can_master_ids{0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
// 当前驱动采用 MIT 控制模式
static const Control_Type CONTROL_MODE = Control_Type::MIT;
// 机械臂本体自由度
static const std::size_t ARM_DOF = 7;
// 夹爪自由度
static const std::size_t GRIPPER_DOF = 1;
// 总自由度（机械臂 + 夹爪）
static const std::size_t TOTAL_DOF = ARM_DOF + GRIPPER_DOF;
// 默认关节刚度（KP），作为命令接口初始值
static const std::array<double, TOTAL_DOF> DEFAULT_KP = {400.0, 400.0, 150.0, 350.0,
                                                 100.0,  100.0,  100.0,  8.0};   //不加重力补偿的硬kp,kd值，后续需要减小

// 默认关节阻尼（KD），作为命令接口初始值
static const std::array<double, TOTAL_DOF> DEFAULT_KD = {4.0, 4.0, 2.0, 3.2,
                                                 1.5,  1.5, 1.5, 1.2};

// // 默认关节刚度（KP），作为命令接口初始值
// static const std::array<double, TOTAL_DOF> DEFAULT_KP = {200.0, 200.0, 150.0, 55.0,
//                                                  5.0,  5.0,  5.0,  8.0};
                                                 
// // 默认关节阻尼（KD），作为命令接口初始值
// static const std::array<double, TOTAL_DOF> DEFAULT_KD = {3, 3, 2.0, 0.4,
//                                                  0.7,  0.6, 0.5, 1.2};
// 激活阶段允许的“初始位置误差”阈值（弧度）
static const double START_POS_TOLERANCE_RAD = 0.1;
// 运行阶段允许的单次位置跳变阈值（弧度）
static const double POS_JUMP_TOLERANCE_RAD = 3.1415 / 10.0;

// 是否启用夹爪硬件映射
static const bool USING_GRIPPER = true;
// 夹爪电机角度与线性开合量之间的参考半径（米）
// static const double GRIPPER_REFERENCE_GEAR_RADIUS_M = 0.00853;
static const double GRIPPER_REFERENCE_GEAR_RADIUS_M = 0.037532;
// 夹爪传动方向修正系数（-1 或 1）
static const double GRIPPER_GEAR_DIRECTION_MULTIPLIER = 1.0;
// 夹爪在状态/命令数组中的索引
static const int GRIPPER_INDEX = TOTAL_DOF - 1;

/**
 * @brief OpenArm ros2_control 硬件系统接口实现。
 *
 * 该类负责：
 * 1) 读取 URDF/ros2_control 中的硬件参数并初始化电机对象；
 * 2) 向控制器导出状态接口（position/velocity/effort）；
 * 3) 向控制器导出命令接口（position/velocity/effort/stiffness/damping）；
 * 4) 在 read()/write() 周期中完成“硬件 <-> 控制器”数据交换。
 */
class OpenArmHW : public hardware_interface::SystemInterface {
 public:
  /// 构造函数（主要初始化由 on_init 完成）
  OpenArmHW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  /// 生命周期：初始化硬件对象、解析参数、创建电机实例与缓存数组
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  /// 生命周期：配置阶段，执行零位设置等一次性动作
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  /// 导出状态接口：position / velocity / effort
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  /// 导出命令接口：position / velocity / effort / stiffness(KP) / damping(KD)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  /// 生命周期：激活阶段，电机上使能并平滑过渡到命令位置
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  /// 生命周期：停用阶段，下发安全零指令并关闭使能
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  /// 读取硬件反馈并写入状态缓存
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  /// 读取控制命令缓存并下发到电机
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // 当前实际参与控制的自由度数；如果启用夹爪会在 on_init 中 +1
  std::size_t curr_dof = ARM_DOF;
 private:
  // URDF 中的前缀参数，可用于多机械臂场景的 joint name 区分
  std::string prefix_;
  // CAN 总线通信对象
  std::unique_ptr<CANBus> canbus_;
  // 电机控制对象（对外提供 enable/disable/controlMIT 等接口）
  std::unique_ptr<MotorControl> motor_control_;

  // 命令缓存（由控制器写入，write() 使用）
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_ff_commands_;

  // 状态缓存（由 read() 写入，控制器读取）
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // 动态刚度/阻尼命令缓存（通过自定义命令接口 stiffness/damping 写入）
  std::vector<double> kp_commands_;
  std::vector<double> kd_commands_;

  // 每个关节对应一个 Motor 实例
  std::vector<std::unique_ptr<Motor>> motors_;

  // 向所有电机发送安全零指令（kp=kd=pos=vel=tau_ff=0）
  void refresh_motors();

  // 当为 true 时，write() 中不下发正常控制量，而是下发零力矩（安全/调试模式）
  bool disable_torque_;
};

}  // namespace openarm_hardware
