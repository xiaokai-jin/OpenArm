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

#include <cstdint>
#include <cstring>
#include <map>

#include "dm_motor_constants.hpp"

namespace openarm::damiao_motor {
class Motor {
    friend class DMCANDevice;  // 允许 DMCANDevice 访问受保护成员
    friend class DMControl;

public:
    // 构造函数
    Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id);

    // 状态 getter 方法
    double get_position() const { return state_q_; }
    double get_velocity() const { return state_dq_; }
    double get_torque() const { return state_tau_; }
    int get_state_tmos() const { return state_tmos_; }
    int get_state_trotor() const { return state_trotor_; }

    // 电机属性 getter 方法
    uint32_t get_send_can_id() const { return send_can_id_; }
    uint32_t get_recv_can_id() const { return recv_can_id_; }
    MotorType get_motor_type() const { return motor_type_; }

    // 使能状态 getter 方法
    bool is_enabled() const { return enabled_; }

    // 参数方法
    double get_param(int RID) const;

    // 电机属性静态方法
    static LimitParam get_limit_param(MotorType motor_type);

protected:
    // 状态更新方法
    void update_state(double q, double dq, double tau, int tmos, int trotor);
    void set_state_tmos(int tmos);
    void set_state_trotor(int trotor);
    void set_enabled(bool enabled);
    void set_temp_param(int RID, int val);

    // 电机标识符
    uint32_t send_can_id_;
    uint32_t recv_can_id_;
    MotorType motor_type_;

    // 使能状态
    bool enabled_;

    // 当前状态
    double state_q_, state_dq_, state_tau_;
    int state_tmos_, state_trotor_;

    // 参数存储
    std::map<int, double> temp_param_dict_;
};
}  // namespace openarm::damiao_motor
