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
#include <string.h>
#include <unistd.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>
#include <vector>
/*
 * Compute gravity and inertia compensation using Orocos
 * Kinematics and Dynamics Library (KDL).
 */
class Dynamics {
private:
    std::shared_ptr<urdf::ModelInterface> urdf_model_interface; // URDF模型接口

    std::string urdf_path;    // URDF文件路径
    std::string start_link;   // 运动链基座(base) link的名称
    std::string end_link;     // 运动链末端(end effector) link的名称

    // 缓存的KDL数据结构，预分配内存避免控制循环中动态分配，保证实时性
    KDL::JntSpaceInertiaMatrix inertia_matrix;
    KDL::JntArray q;
    KDL::JntArray q_d;
    KDL::JntArray coriolis_forces;
    KDL::JntArray gravity_forces;

    KDL::JntArray biasangle;

    KDL::Tree kdl_tree;                             // 机械臂完整树结构
    KDL::Chain kdl_chain;                           // 从start到end的单一运动链
    std::unique_ptr<KDL::ChainDynParam> solver;     // KDL动力学求解器

public:
    Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
    ~Dynamics();

    /**
     * @brief 初始化动力学模型，主要完成URDF解析和KDL对象配置
     * @return 成功返回 true，失败返回 false
     */
    bool Init();

    /**
     * @brief 获取指定关节位置下的重力补偿力矩
     *        计算使得机械臂克服自身重力从而保持该姿态所需的各个关节的力矩。
     */
    void GetGravity(const double *motor_position, double *gravity);

    /**
     * @brief 计算科里奥利力和离心力
     *        当机械臂具有一定运动速度时，基于运动学的耦合力。
     */
    void GetCoriolis(const double *motor_position, const double *motor_velocity, double *coriolis);

    /**
     * @brief 获取质量(惯性)矩阵的对角线元素
     *        用于阻抗控制中的惯量塑形或获取各关节的当前等效阻力。
     */
    void GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag);

    /**
     * @brief 计算雅可比矩阵(Jacobian Matrix)
     *        将关节空间的速度映射到笛卡尔空间(6xN)。
     */
    void GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian);

    /**
     * @brief 计算速度空间下的零空间投影矩阵 (N = I - J_pinv * J)
     *        对于冗余机械臂，零空间运动可以在不影响末端位姿的情况下改变机器人全身姿态。
     */
    void GetNullSpace(const double *motor_positon, Eigen::MatrixXd &nullspace);

    /**
     * @brief 计算力矩空间下的零空间投影矩阵 (N_T = N.transpose())
     *        常用于零空间力矩注入控制。
     */
    void GetNullSpaceTauSpace(const double *motor_position, Eigen::MatrixXd &nullspace_T);

    /**
     * @brief 正运动学计算：获取当前关节下的末端法相姿态(Rotation, R)与位置(Position, p)
     */
    void GetEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);

    /**
     * @brief 正运动学计算：获取运动链中倒数第二个关节(前置末端)的位姿
     */
    void GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);

    /**
     * @brief 获取完整的质量(惯性)矩阵 H(q) / M(q)
     */
    void GetMassMatrix(const double *motor_position, Eigen::MatrixXd &mass_matrix);
};
