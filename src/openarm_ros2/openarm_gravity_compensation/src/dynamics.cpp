#include "openarm_gravity_compensation/dynamics.hpp"

#include <urdf_parser/urdf_parser.h>
#include <cstdio>

namespace openarm_gravity_compensation {

// 构造函数使用 std::move 把入参字符串“搬运”到成员变量，
// 避免不必要的深拷贝（尤其 robot_description 往往很大）。
Dynamics::Dynamics(std::string urdf_xml, std::string start_link, std::string end_link)
    : urdf_xml_(std::move(urdf_xml)),
      start_link_(std::move(start_link)),
      end_link_(std::move(end_link)) {}

bool Dynamics::init() {
  // 1) 解析 URDF XML 字符串 -> urdf::ModelInterface
  auto urdf_model_interface = urdf::parseURDF(urdf_xml_);
  if (!urdf_model_interface) {
    std::fprintf(stderr, "[GravityDynamics] Failed to parse URDF from robot_description\n");
    return false;
  }

  // 2) 从 URDF 模型构建 KDL tree（整机树结构）
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_interface, kdl_tree_)) {
    std::fprintf(stderr, "[GravityDynamics] Failed to build KDL tree from URDF\n");
    return false;
  }

  // 3) 从树中提取 start_link -> end_link 的单一链（重力计算只对链路进行）
  if (!kdl_tree_.getChain(start_link_, end_link_, kdl_chain_)) {
    std::fprintf(stderr, "[GravityDynamics] Failed to extract KDL chain from '%s' to '%s'\n",
                 start_link_.c_str(), end_link_.c_str());
    return false;
  }

  // 4) 预分配结果缓存，避免循环里重复分配内存。
  gravity_forces_.resize(kdl_chain_.getNrOfJoints());
  gravity_forces_.data.setZero();

  // 5) 创建动力学求解器。
  // KDL::Vector(0,0,-9.81) 显式指定重力方向朝 -Z。
  // std::make_unique 是 C++11/14 推荐写法，异常安全且更简洁。
  solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

  std::fprintf(stdout, "[GravityDynamics] Initialized dynamics chain: %zu joints\n",
               static_cast<std::size_t>(kdl_chain_.getNrOfJoints()));
  return true;
}

bool Dynamics::get_gravity(const std::vector<double>& joint_positions,
                           std::vector<double>& gravity_torques) {
  // 保护：必须先 init() 成功后才能调用。
  if (!solver_) {
    return false;
  }

  const auto nj = kdl_chain_.getNrOfJoints();

  // 保护：输入维度要与链路关节数严格一致。
  if (joint_positions.size() != nj) {
    return false;
  }

  // 将 std::vector<double> 转换为 KDL::JntArray。
  // 这里的 for 循环是显式映射，便于后续定位 joint 顺序问题。
  KDL::JntArray q(nj);
  for (std::size_t i = 0; i < nj; ++i) {
    q(i) = joint_positions[i];
  }

  // 核心调用：求解重力项 tau_g(q)
  solver_->JntToGravity(q, gravity_forces_);

  // 拷回标准容器，便于 ROS 消息发布。
  gravity_torques.resize(nj);
  for (std::size_t i = 0; i < nj; ++i) {
    gravity_torques[i] = gravity_forces_(i);
  }
  return true;
}

std::size_t Dynamics::dof() const { return kdl_chain_.getNrOfJoints(); }

}  // namespace openarm_gravity_compensation
