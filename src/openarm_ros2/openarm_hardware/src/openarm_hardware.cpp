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

#include "openarm_hardware/openarm_hardware.hpp"

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

// 默认 CAN 设备名（当前实现实际使用的是 hardware parameter 中的 can_device）
static const std::string& can_device_name = "can0";

OpenArmHW::OpenArmHW() = default;

// 初始化：读取参数、创建通信对象、构建电机列表、初始化命令/状态缓存
hardware_interface::CallbackReturn OpenArmHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  // 先执行父类初始化，确保 info_ 等基础成员被正确填充
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // 必需参数：can_device（例如 can0）
  if (info.hardware_parameters.find("can_device") ==
      info.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "No can_device parameter found");
    return CallbackReturn::ERROR;
  }

  // 可选参数：prefix（用于多机械臂关节名前缀）
  // 可选参数：disable_torque（true 时 write() 下发全零指令）
  auto it = info.hardware_parameters.find("prefix");
  if (it == info.hardware_parameters.end()) {
    prefix_ = "";
  } else {
    prefix_ = it->second;
  }

 
  it = info.hardware_parameters.find("disable_torque");
  if (it == info.hardware_parameters.end()) {
    disable_torque_ = false;
  } else {
    disable_torque_ = it->second == "true";
  }

  // 创建 CANFD 总线与电机控制器
  canbus_ = std::make_unique<CANBus>(info.hardware_parameters.at("can_device"),
                                     CAN_MODE_FD);
  motor_control_ = std::make_unique<MotorControl>(*canbus_);

  // 如果启用夹爪，则在配置末尾追加夹爪电机，并扩展自由度
  if (USING_GRIPPER) {
    motor_types.emplace_back(DM_Motor_Type::DM4310);
    can_device_ids.emplace_back(0x08);
    can_master_ids.emplace_back(0x18);
    ++curr_dof;
  }

  // 构造每个关节对应的 Motor 对象，并注册到 MotorControl
  motors_.resize(curr_dof);
  for (size_t i = 0; i < curr_dof; ++i) {
    motors_[i] = std::make_unique<Motor>(motor_types[i], can_device_ids[i],
                                         can_master_ids[i]);
    motor_control_->addMotor(*motors_[i]);
  }

  // 初始化状态/命令缓存，保证导出接口后有稳定内存地址可绑定
  pos_states_.resize(curr_dof, 0.0);
  pos_commands_.resize(curr_dof, 0.0);
  vel_states_.resize(curr_dof, 0.0);
  vel_commands_.resize(curr_dof, 0.0);
  tau_states_.resize(curr_dof, 0.0);
  tau_ff_commands_.resize(curr_dof, 0.0);

  // 初始化动态增益命令缓存（可由上层控制器通过接口实时改写）
  kp_commands_.resize(curr_dof, 0.0);
  kd_commands_.resize(curr_dof, 0.0);

  // 使用默认数组给各关节赋初值（机械臂本体）
  for (size_t i = 0; i < ARM_DOF; ++i) {
    kp_commands_[i] = DEFAULT_KP[i];
    kd_commands_[i] = DEFAULT_KD[i];
  }

  // 夹爪增益初值
  if (USING_GRIPPER) {
    kp_commands_[GRIPPER_INDEX] = DEFAULT_KP[GRIPPER_INDEX];
    kd_commands_[GRIPPER_INDEX] = DEFAULT_KD[GRIPPER_INDEX];
  }

  // 发送一次安全零指令并尝试读取一次状态，确保通信链路工作
  refresh_motors();
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

// 配置：读取当前状态并将当前机械位置设为零点
hardware_interface::CallbackReturn OpenArmHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  // 对每个电机执行“当前位置设为零位”
  for (std::size_t i = 0; i < curr_dof; ++i)
  {
    motor_control_->set_zero_position(*motors_[i]);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmHW::export_state_interfaces() {
  // 将内部状态缓存绑定到 ros2_control 的 state interface
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < curr_dof; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &tau_states_[i]));
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "Exporting state interface for joint %s",
                info_.joints[i].name.c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmHW::export_command_interfaces() {
  // 将内部命令缓存绑定到 ros2_control 的 command interface
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < curr_dof; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &tau_ff_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "stiffness", &kp_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, "damping", &kd_commands_[i]));
  }

  return command_interfaces;
}

void OpenArmHW::refresh_motors() {
  // 给每个电机发送“零控制量”作为安全刷新
  for (const auto& motor : motors_) {
    motor_control_->controlMIT(*motor, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
}

// 激活：使能电机并平滑逼近目标位置，避免上电瞬间跳变
hardware_interface::CallbackReturn OpenArmHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  bool zeroed = false;

  // 先逐个电机使能
  for (const auto& motor : motors_) {
    motor_control_->enable(*motor);
  }

  // 按最大步长逐渐贴近 pos_commands_，直到全关节误差低于阈值
  while (!zeroed) {
    bool all_zero = true;
    for (std::size_t m = 0; m < curr_dof; ++m) {
      // 当前关节位置误差
      const double diff = pos_commands_[m] - pos_states_[m];
      if (std::abs(diff) > START_POS_TOLERANCE_RAD) {
        all_zero = false;
      }

      // 限制每一轮最大移动量，防止激活阶段突跳
      const double max_step = std::min(POS_JUMP_TOLERANCE_RAD, std::abs(diff));
      double command = pos_states_[m];
      if (pos_states_[m] < pos_commands_[m]) {
        command += max_step;
      } else {
        command -= max_step;
      }
      // 用 MIT 下发过渡位置（速度和前馈设为 0）
      motor_control_->controlMIT(*motors_[m], kp_commands_[m], kd_commands_[m], command, 0.0, 0.0);
    }

    // 所有关节进入容差范围后结束激活过渡
    if (all_zero) {
      zeroed = true;
    } else {
      // 短暂等待后读取新状态，继续下一轮插补
      sleep(0.01);
      read(rclcpp::Time(0), rclcpp::Duration(0, 0));
    }
  }

  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // 先归零刷新，再下使能，避免停机瞬间残留控制量
  refresh_motors();
  for (const auto& motor : motors_) {
    motor_control_->disable(*motor);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // 机械臂关节：直接读取电机反馈
  for (size_t i = 0; i < ARM_DOF; ++i) {
    pos_states_[i] = motors_[i]->getPosition();
    vel_states_[i] = motors_[i]->getVelocity();
    tau_states_[i] = motors_[i]->getTorque();
  }

  // 夹爪：将电机角度/速度/力矩转换为夹爪侧物理量
  if (USING_GRIPPER) {
    pos_states_[GRIPPER_INDEX] = -motors_[GRIPPER_INDEX]->getPosition() *
                                 GRIPPER_REFERENCE_GEAR_RADIUS_M *
                                 GRIPPER_GEAR_DIRECTION_MULTIPLIER;
    vel_states_[GRIPPER_INDEX] = motors_[GRIPPER_INDEX]->getVelocity() *
                                 GRIPPER_REFERENCE_GEAR_RADIUS_M *
                                 GRIPPER_GEAR_DIRECTION_MULTIPLIER;
    tau_states_[GRIPPER_INDEX] = motors_[GRIPPER_INDEX]->getTorque() *
                                 GRIPPER_REFERENCE_GEAR_RADIUS_M *
                                 GRIPPER_GEAR_DIRECTION_MULTIPLIER;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (disable_torque_) {
    // 禁扭模式：每个周期下发全零指令（常用于调试/安全）
    for (size_t i = 0; i < curr_dof; ++i) {
      motor_control_->controlMIT(*motors_[i], 0.0, 0.0, 0.0, 0.0, 0.0);
      return hardware_interface::return_type::OK;
    }
  }

  // 机械臂关节写入：位置跳变保护 + MIT 命令下发
  for (size_t i = 0; i < ARM_DOF; ++i) {
    double target_position = pos_commands_[i];
    const double position_diff = pos_commands_[i] - pos_states_[i];
    if (std::abs(position_diff) > POS_JUMP_TOLERANCE_RAD) {
      target_position = pos_states_[i] +
                        (position_diff > 0.0 ? POS_JUMP_TOLERANCE_RAD
                                             : -POS_JUMP_TOLERANCE_RAD);
      RCLCPP_WARN(rclcpp::get_logger("OpenArmHW"),
                  "Position jump limited for joint %s: cmd=%f state=%f limited_cmd=%f",
                  info_.joints[i].name.c_str(), pos_commands_[i],
                  pos_states_[i], target_position);
    }

    // 关键控制下发点：
    // 若要改底层控制策略，通常优先在上层控制器改写 pos/vel/tau/kp/kd 的生成逻辑；
    // 若要改“发给驱动器的控制模式/映射”，可在此处替换 controlMIT 调用。
    motor_control_->controlMIT(*motors_[i], kp_commands_[i], kd_commands_[i],
                               target_position, vel_commands_[i],
                               tau_ff_commands_[i]);
  }

  // 夹爪写入：先做末端量 -> 电机量换算，再使用同样的 MIT 通道下发
  if (USING_GRIPPER) {
    motor_control_->controlMIT(
        *motors_[GRIPPER_INDEX], kp_commands_[GRIPPER_INDEX], kd_commands_[GRIPPER_INDEX],
        -pos_commands_[GRIPPER_INDEX] / GRIPPER_REFERENCE_GEAR_RADIUS_M *
            GRIPPER_GEAR_DIRECTION_MULTIPLIER,
        vel_commands_[GRIPPER_INDEX] / GRIPPER_REFERENCE_GEAR_RADIUS_M *
            GRIPPER_GEAR_DIRECTION_MULTIPLIER,
        tau_ff_commands_[GRIPPER_INDEX] / GRIPPER_REFERENCE_GEAR_RADIUS_M *
            GRIPPER_GEAR_DIRECTION_MULTIPLIER);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArmHW,
                       hardware_interface::SystemInterface)
