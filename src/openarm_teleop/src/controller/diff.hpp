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
// #include "controller/global.hpp"
// #include <global.hpp>

#include <openarm_constants.hpp>

class Differentiator {
private:
    double Ts_;                            // 采样时间 (Sampling time), 控制周期，例如0.001s(1kHz)
    double velocity_z1_[NMOTORS] = {0.0};  // 上一时刻的速度缓冲 (Velocity 1 step before)
    double position_z1_[NMOTORS] = {0.0};  // 上一时刻的位置缓冲 (Position 1 step before)
    double acc_z1_[NMOTORS] = {0.0};       // 上一时刻的加速度缓冲
    double acc_[NMOTORS] = {0.0};          // 当前计算的加速度

public:
    Differentiator(double Ts) : Ts_(Ts) {}

    /**
     * @brief 对位置信号求导计算速度。
     *        如果不加滤波直接 (p - p_old)/Ts，位置传感器的量化噪声会被极度放大。
     *        这里采用了一阶低通滤波器结合差分： v = a * v_old + b * (p - p_old)
     */
    void Differentiate(const double *position, double *velocity) {
        // 低通滤波器的离散化参数公式：基于截至频率 CUTOFF_FREQUENCY
        double a = 1.0 / (1.0 + Ts_ * CUTOFF_FREQUENCY);
        double b = a * CUTOFF_FREQUENCY;

        for (int i = 0; i < NMOTORS; i++) {
            // 初始化防止第一步产生巨大阶跃 (导数爆炸)
            if (position_z1_[i] == 0.0) {
                position_z1_[i] = position[i];
            }

            // 速度 = 上次速度的衰减(低通) + 位置差分的平滑放大
            velocity[i] = velocity_z1_[i] * a + b * (position[i] - position_z1_[i]);
            
            // 更新上一时刻状态
            position_z1_[i] = position[i];
            velocity_z1_[i] = velocity[i];
        }
    }

    /**
     * @brief 基于观测器的微分计算 (带有前馈扭矩/加速度观测的模型)
     *        利用动力学中 torque/mass = 加速度的关系，结合位置差分计算速度和加速度，使得速度估算更迅速且不滞后。
     */
    void Differentiate_w_obs(const double *position, double *velocity, double *mass,
                             double *input_torque) {
        double a = 1.0 / (1.0 + Ts_ * CUTOFF_FREQUENCY);
        double b = a * CUTOFF_FREQUENCY;

        for (int i = 0; i < NMOTORS; i++) {
            if (position_z1_[i] == 0.0000000) {
                position_z1_[i] = position[i];
                acc_z1_[i] = acc_[i];
            }

            // 根据 牛顿第二定律 a = tau / M 前馈计算加速度并滤波
            acc_[i] = acc_z1_[i] * a + b * (input_torque[i] / (mass[i]));
            // 速度 = 基于位置差分的平滑速度 + 当期前馈加速度的积分效果
            velocity[i] = velocity_z1_[i] * a + b * (position[i] - position_z1_[i]) + acc_[i];
            
            // 更新状态
            position_z1_[i] = position[i];
            velocity_z1_[i] = velocity[i];
            acc_z1_[i] = acc_[i];
        }
    }
};
