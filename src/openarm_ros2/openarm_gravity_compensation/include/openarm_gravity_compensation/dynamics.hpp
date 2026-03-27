#pragma once

#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace openarm_gravity_compensation {

class Dynamics {
 public:
  Dynamics(std::string urdf_xml, std::string start_link, std::string end_link);

  bool init();
  bool get_gravity(const std::vector<double>& joint_positions,
                   std::vector<double>& gravity_torques);
  std::size_t dof() const;

 private:
  std::string urdf_xml_;
  std::string start_link_;
  std::string end_link_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray gravity_forces_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
};

}  // namespace openarm_gravity_compensation
