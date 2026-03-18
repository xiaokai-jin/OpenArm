// Copyright 2025 Enactic, Inc.
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

// #include <sensor_msgs/msg/joint_state.hpp>
#include <controller/diff.hpp>
#include <controller/dynamics.hpp>
#include <deque>
#include <fstream>
#include <joint_state_converter.hpp>
#include <memory>
#include <numeric>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_constants.hpp>
#include <robot_state.hpp>
#include <utility>

class Control {
    openarm::can::socket::OpenArm *openarm_;

    double Ts_;
    int role_;

    size_t arm_motor_num_;
    size_t hand_motor_num_;

    Differentiator *differentiator_;
    OpenArmJointConverter *openarmjointconverter_;
    OpenArmJGripperJointConverter *openarmgripperjointconverter_;

    std::shared_ptr<RobotSystemState> robot_state_;

    std::string arm_type_;

    Dynamics *dynamics_f_;
    Dynamics *dynamics_l_;

    double oblique_coordinates_force;
    double oblique_coordinates_position;

    // for easy logging
    // std::vector<std::pair<double, double>> velocity_log_;  // (differ_velocity, motor_velocity)
    // std::string log_file_path_ = "../data/velocity_comparison.csv";
    static constexpr int VEL_WINDOW_SIZE = 10;
    static constexpr double VIB_THRESHOLD = 0.7;  // [rad/s]
    std::deque<double> velocity_buffer_[NJOINTS];

public:
    Control(openarm::can::socket::OpenArm *arm, Dynamics *dynamics_l, Dynamics *dynamics_f,
            std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
            size_t arm_joint_num, size_t hand_motor_num);
    Control(openarm::can::socket::OpenArm *arm, Dynamics *dynamics_l, Dynamics *dynamics_f,
            std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
            std::string arm_type, size_t arm_joint_num, size_t hand_motor_num);
    ~Control();

    std::shared_ptr<RobotSystemState> response_;
    std::shared_ptr<RobotSystemState> reference_;

    std::vector<double> Dn_, Kp_, Kd_, Fc_, k_, Fv_, Fo_;

    // bool Setup(void);
    void Setstate(int state);
    void Shutdown(void);

    /**
     * @brief 动态设置阻尼、刚度、摩擦力补偿等控制器相关参数
     * @param Kp 达妙电机 MIT 模式下的被动刚度系数位置(阻抗参数)
     * @param Kd 达妙电机 MIT 模式下的被动阻尼系数速度(阻抗参数)
     * @param Fc 库仑摩擦力系数
     * @param k  双曲正切函数摩擦力形状参数，控制在低速下的过渡平滑度
     * @param Fv 粘滞摩擦力系数
     * @param Fo 静态推力/偏移残余补偿
     */
    void SetParameter(const std::vector<double> &Kp, const std::vector<double> &Kd,
                      const std::vector<double> &Fc, const std::vector<double> &k,
                      const std::vector<double> &Fv, const std::vector<double> &Fo);

    /**
     * @brief [安全策略] 插值调整初始位置
     *        在上电/算法启动时，不直接输出目标力，而是通过多个 step 线性插值，平滑移动，避免抽打/剧烈运动。
     */
    bool AdjustPosition(void);

    /**
     * @brief 双向遥操作(Bilateral)通信步进
     *        处理主臂(Leader)与从臂(Follower)的双向力与位置耦合。
     *        计算并下发重力、科里奥利力与摩擦力前馈，并通过底层 MIT 协议进行力控补偿。
     */
    bool bilateral_step();
    
    /**
     * @brief 单向控制(Unilateral)通信步进
     *        只处理单方向跟随指令(例如仅仅主臂传位置，从臂跟随，不反向传递力矩)。
     */
    bool unilateral_step();

    // NOTE! Control() class operates on "joints", while the underlying
    // classes operates on "actuators". The following functions map
    // joints to actuators.

    void ComputeJointPosition(const double *motor_position, double *joint_position);
    void ComputeJointVelocity(const double *motor_velocity, double *joint_velocity);
    void ComputeMotorTorque(const double *joint_torque, double *motor_torque);

    /**
     * @brief 计算并返回摩擦力补偿力矩
     *        使用连续滑移模型 (包含粘滞与基于 Tanh 函数的平滑化库仑摩擦力)
     */
    void ComputeFriction(const double *velocity, double *friction, size_t index);
    void ComputeGravity(const double *position, double *gravity);
    
    /**
     * @brief [安全诊断] 振动/共振检测
     *        通过缓存近几帧控制循环的速度滑动窗口，计算方差/标准差。
     *        若速度标准差突变超过安全阈值 `VIB_THRESHOLD`，则报告危险，常用于触发掉电急停。
     */
    bool DetectVibration(const double *velocity, bool *what_axis);
};
