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

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>
#include <openarm/can/socket/arm_component.hpp>

namespace openarm::can::socket {

ArmComponent::ArmComponent(canbus::CANSocket& can_socket)
    : damiao_motor::DMDeviceCollection(can_socket) {}

void ArmComponent::init_motor_devices(const std::vector<damiao_motor::MotorType>& motor_types,
                                      const std::vector<canid_t>& send_can_ids,
                                      const std::vector<canid_t>& recv_can_ids, bool use_fd) {
    // 预留空间以防止 vector 重新分配内存，从而导致指向电机的引用失效
    motors_.reserve(motor_types.size());

    for (size_t i = 0; i < motor_types.size(); i++) {
        // 首先，在 vector 中创建并存储电机对象
        motors_.emplace_back(motor_types[i], send_can_ids[i], recv_can_ids[i]);
        // 然后使用存储在 vector 中的电机引用创建设备对象
        auto motor_device =
            std::make_shared<damiao_motor::DMCANDevice>(motors_.back(), CAN_SFF_MASK, use_fd);
        get_device_collection().add_device(motor_device);
    }
}

}  // namespace openarm::can::socket
