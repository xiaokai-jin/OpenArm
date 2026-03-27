#pragma once

#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace openarm_gravity_compensation {

/**
 * @brief 轻量动力学封装：从 URDF 构建 KDL 链，并计算重力项。
 *
 * 设计目标：
 * - 只做重力补偿所需的最小能力（避免把控制策略耦合到动力学类）
 * - 在节点侧可重复调用，输出每个关节的 tau_g(q)
 *
 * 重要约束：
 * - 输入 joint_positions 的顺序必须与 joint_names 完全一致
 * - start_link / end_link 必须能在 URDF 中形成单一链
 */
class Dynamics {
 public:
  /// @param urdf_xml 完整 URDF XML 文本（通常来自 robot_description 参数）
  /// @param start_link 运动链起点（基座）
  /// @param end_link 运动链终点（末端）
  Dynamics(std::string urdf_xml, std::string start_link, std::string end_link);

  /// 解析 URDF、构建 KDL::Tree/KDL::Chain，并创建动力学求解器。
  bool init();

  /**
   * @brief 计算重力补偿力矩。
   * @param joint_positions 关节位置向量 q（单位弧度）
   * @param gravity_torques 输出重力向量 tau_g(q)
   * @return true 计算成功；false 表示求解器未初始化或输入维度不匹配
   */
  bool get_gravity(const std::vector<double>& joint_positions,
                   std::vector<double>& gravity_torques);

  /// 当前链路自由度（关节数量）
  std::size_t dof() const;

 private:
  // 原始模型文本：避免依赖文件系统路径，便于在 ROS 参数系统中直接传递。
  std::string urdf_xml_;
  // 运动链端点定义（start -> end）
  std::string start_link_;
  std::string end_link_;

  // KDL 模型缓存：Tree 为全树，Chain 为当前控制链。
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  // 重力结果缓存（重复使用内存，减少循环中的频繁分配）。
  KDL::JntArray gravity_forces_;

  // KDL 动力学求解器（智能指针用于自动释放资源）。
  std::unique_ptr<KDL::ChainDynParam> solver_;
};

}  // namespace openarm_gravity_compensation
