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

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <thread>

int main() {
    try {
        std::cout << "=== OpenArm CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the OpenArm API functionality" << std::endl;

        // Initialize OpenArm with CAN interface and enable CAN-FD
        std::cout << "Initializing OpenArm CAN..." << std::endl;
        openarm::can::socket::OpenArm openarm("can0", true);  // Use CAN-FD on can0 interface

        // Initialize arm motors
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM8009, 
            openarm::damiao_motor::MotorType::DM8009,
            openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4340,
            openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310
        };
        std::vector<uint32_t> send_can_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Initialize gripper
        std::cout << "Initializing gripper..." << std::endl;
        openarm.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);

        // Set callback mode to ignore and enable all motors
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        // Enable all motors
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        openarm.enable_all();
        // Allow time (10ms) for the motors to respond for slow operations like enabling
        openarm.recv_all(2000);

        // Set device mode to param and query motor id
        std::cout << "\n=== Querying Motor Recv IDs ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
        openarm.query_param_all(static_cast<int>(openarm::damiao_motor::RID::MST_ID));
        // Allow time (10ms) for the motors to respond for slow operations like querying
        // parameter from register
        openarm.recv_all(2000);

        // Access motors through components
        for (const auto& motor : openarm.get_arm().get_motors()) {
            std::cout << "Arm Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }
        for (const auto& motor : openarm.get_gripper().get_motors()) {
            std::cout << "Gripper Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(openarm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }

        // Set zero position for all motors (important!)
        std::cout << "\n=== Setting Zero Position ===" << std::endl;
        openarm.set_zero_all();
        openarm.recv_all(2000);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Set device mode to state and control motor
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // Define target positions for all 7 joints (in radians)
        // std::vector<double> target_positions = {0.5, 0.3, 0.0, 0.2, 0.0, 0.1, 0.0};
        
        // Per-joint control parameters (heavier joints need stronger stiffness)
        // Motor 1-2 (DM8009): Base joints, carry most weight -> high kp
        // Motor 3-4 (DM4340): Mid joints, moderate load -> medium kp
        // Motor 5-7 (DM4310): Wrist joints, light load -> lower kp
        std::vector<double> kp_values = {100.0, 100.0, 60.0, 60.0, 40.0, 40.0, 40.0};
        std::vector<double> kd_values = {3, 3, 2, 2, 1.5, 1.5, 1.5};
        
        // OPTION 2: If still not tracking well, use aggressive uniform gains (uncomment below)
        // std::vector<double> kp_values = {150.0, 150.0, 150.0, 150.0, 100.0, 100.0, 100.0};
        // std::vector<double> kd_values = {5.0, 5.0, 5.0, 5.0, 3.0, 3.0, 3.0};
        
        // std::cout << "Control gains - Motor 1: kp=" << kp_values[0] << ", kd=" << kd_values[0] << std::endl;
        // std::cout << "              - Motor 5: kp=" << kp_values[4] << ", kd=" << kd_values[4] << std::endl;
        
        // Control loop: send commands continuously for 2 seconds at ~200Hz
        // Use a ramp on target position to avoid fast step response.
        std::cout << "Starting closed-loop control for 2 seconds..." << std::endl;
        int control_iterations = 400;  // 400 iterations at 5ms each = 2 seconds
        int ramp_iterations = 200;     // 1 second ramp-up to final target
        
        for (int iter = 0; iter < control_iterations; iter++) {
            double alpha = std::min(1.0, static_cast<double>(iter) / ramp_iterations);

            // Send control commands to all motors with per-joint gains
            openarm.get_arm().mit_control_all({
                openarm::damiao_motor::MITParam{0, kd_values[0], 0, -1, 0},//前
                openarm::damiao_motor::MITParam{0, kd_values[1], 0, 0, 0},//右
                openarm::damiao_motor::MITParam{0, kd_values[2], 0, 0, 0},//外
                openarm::damiao_motor::MITParam{0, kd_values[3], 0, 0, 0},//屈
                openarm::damiao_motor::MITParam{0, kd_values[4], 0, 0, 0},//外
                openarm::damiao_motor::MITParam{0, kd_values[5], 0, 0, 0},//内
                openarm::damiao_motor::MITParam{0, kd_values[6], 0, 0, 0} //前
            });
            
            // Receive feedback
            openarm.recv_all(2000);
            
            // Print progress every 50 iterations (~0.25 seconds)
            // if (iter % 50 == 0) {
            //     std::cout << "\n--- Iteration " << iter << " / " << control_iterations << " ---" << std::endl;
            //     int motor_idx = 0;
            //     for (const auto& motor : openarm.get_arm().get_motors()) {
            //         double current_pos = motor.get_position();
            //         double error = target_positions[motor_idx] - current_pos;
            //         std::cout << "  Motor " << motor_idx + 1 
            //                   << " | Target: " << target_positions[motor_idx]
            //                   << " | Current: " << current_pos
            //                   << " | Error: " << error << " rad" << std::endl;
            //         motor_idx++;
            //     }
            // }
            
            // Sleep for 5ms (200Hz control rate)
            // std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        // std::cout << "\n=== Control Loop Finished ===" << std::endl;

        // Control arm motors with torque control
        // openarm.get_arm().mit_control_all({
        //     openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
        //     openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
        //     openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
        //     openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
        //     openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
        //     openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
        //     openarm::damiao_motor::MITParam{0, 0, 0, 0, 0.1}
        // });
        // openarm.recv_all(500);

        // Final position check
        // std::cout << "\n=== Final Position Report ===" << std::endl;
        openarm.refresh_all();
        // openarm.recv_all(500);
        
        // int motor_idx = 0;
        // for (const auto& motor : openarm.get_arm().get_motors()) {
        //     double current_pos = motor.get_position();
        //     double error = target_positions[motor_idx] - current_pos;
        //     std::cout << "Motor " << motor_idx + 1 
        //               << " | Target: " << target_positions[motor_idx]
        //               << " rad | Final: " << current_pos 
        //               << " rad | Final Error: " << error << " rad ("
        //               << (std::abs(error) * 180.0 / M_PI) << " deg)" << std::endl;
        //     motor_idx++;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // Return to zero position smoothly
        // std::cout << "\n=== Returning To Zero Position ===" << std::endl;
        // int return_iterations = 200;  // 1 second at 200Hz
        // for (int iter = 0; iter < return_iterations; iter++) {
        //     double ratio = static_cast<double>(iter) / return_iterations;
        //     double alpha_back = 1.0 - ratio;

        //     openarm.get_arm().mit_control_all({
        //         openarm::damiao_motor::MITParam{kp_values[0], kd_values[0], alpha_back * target_positions[0], 0, 0},
        //         openarm::damiao_motor::MITParam{kp_values[1], kd_values[1], alpha_back * target_positions[1], 0, 0},
        //         openarm::damiao_motor::MITParam{kp_values[2], kd_values[2], alpha_back * target_positions[2], 0, 0},
        //         openarm::damiao_motor::MITParam{kp_values[3], kd_values[3], alpha_back * target_positions[3], 0, 0},
        //         openarm::damiao_motor::MITParam{kp_values[4], kd_values[4], alpha_back * target_positions[4], 0, 0},
        //         openarm::damiao_motor::MITParam{kp_values[5], kd_values[5], alpha_back * target_positions[5], 0, 0},
        //         openarm::damiao_motor::MITParam{kp_values[6], kd_values[6], alpha_back * target_positions[6], 0, 0}
        //     });

        //     openarm.recv_all(500);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(5));
        // }

        // std::cout << "\n=== Zero Position Report ===" << std::endl;
        // openarm.refresh_all();
        // openarm.recv_all(500);
        // motor_idx = 0;
        // for (const auto& motor : openarm.get_arm().get_motors()) {
        //     double current_pos = motor.get_position();
        //     std::cout << "Motor " << motor_idx + 1
        //               << " | Zero Target: 0.0"
        //               << " rad | Current: " << current_pos
        //               << " rad | Residual: " << current_pos << " rad ("
        //               << (std::abs(current_pos) * 180.0 / M_PI) << " deg)" << std::endl;
        //     motor_idx++;
        // }
        
        // Optional: Disable all motors (uncomment if needed)
        std::cout << "\n=== Disabling Motors ===" << std::endl;
        for(int i = 0; i < 3; i++) {
            openarm.disable_all();
            openarm.recv_all(2000);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
