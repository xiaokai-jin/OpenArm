#include "openarm_gravity_compensation/dynamics.hpp"

#include <urdf_parser/urdf_parser.h>
#include <cstdio>

namespace openarm_gravity_compensation {

Dynamics::Dynamics(std::string urdf_xml, std::string start_link, std::string end_link)
    : urdf_xml_(std::move(urdf_xml)),
      start_link_(std::move(start_link)),
      end_link_(std::move(end_link)) {}

bool Dynamics::init() {
  auto urdf_model_interface = urdf::parseURDF(urdf_xml_);
  if (!urdf_model_interface) {
    std::fprintf(stderr, "[GravityDynamics] Failed to parse URDF from robot_description\n");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*urdf_model_interface, kdl_tree_)) {
    std::fprintf(stderr, "[GravityDynamics] Failed to build KDL tree from URDF\n");
    return false;
  }

  if (!kdl_tree_.getChain(start_link_, end_link_, kdl_chain_)) {
    std::fprintf(stderr, "[GravityDynamics] Failed to extract KDL chain from '%s' to '%s'\n",
                 start_link_.c_str(), end_link_.c_str());
    return false;
  }

  gravity_forces_.resize(kdl_chain_.getNrOfJoints());
  gravity_forces_.data.setZero();

  solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

  std::fprintf(stdout, "[GravityDynamics] Initialized dynamics chain: %zu joints\n",
               static_cast<std::size_t>(kdl_chain_.getNrOfJoints()));
  return true;
}

bool Dynamics::get_gravity(const std::vector<double>& joint_positions,
                           std::vector<double>& gravity_torques) {
  if (!solver_) {
    return false;
  }

  const auto nj = kdl_chain_.getNrOfJoints();
  if (joint_positions.size() != nj) {
    return false;
  }

  KDL::JntArray q(nj);
  for (std::size_t i = 0; i < nj; ++i) {
    q(i) = joint_positions[i];
  }

  solver_->JntToGravity(q, gravity_forces_);

  gravity_torques.resize(nj);
  for (std::size_t i = 0; i < nj; ++i) {
    gravity_torques[i] = gravity_forces_(i);
  }
  return true;
}

std::size_t Dynamics::dof() const { return kdl_chain_.getNrOfJoints(); }

}  // namespace openarm_gravity_compensation
