#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "openarm_gravity_compensation/dynamics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace openarm_gravity_compensation {

struct ArmConfig {
  std::string name;
  bool enabled{true};
  double alpha{0.9};
  std::string root_link;
  std::string leaf_link;
  std::vector<std::string> joint_names;
  std::string command_topic;

  std::unique_ptr<Dynamics> dynamics;
  std::vector<int> joint_indices;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
  rclcpp::Time last_log_time{0};
};

class GravityCompensationNode : public rclcpp::Node {
 public:
  GravityCompensationNode() : Node("gravity_compensation_node") {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 200.0);
    log_period_sec_ = declare_parameter<double>("log_period_sec", 5.0);
    robot_description_source_node_ =
        declare_parameter<std::string>("robot_description_source_node", "/robot_state_publisher");

    left_arm_.name = "left_arm";
    left_arm_.enabled = declare_parameter<bool>("left_arm.enabled", true);
    left_arm_.alpha = declare_parameter<double>("left_arm.alpha", 0.9);
    left_arm_.root_link = declare_parameter<std::string>("left_arm.root_link", "openarm_body_link0");
    left_arm_.leaf_link = declare_parameter<std::string>("left_arm.leaf_link", "openarm_left_hand");
    left_arm_.joint_names = declare_parameter<std::vector<std::string>>(
        "left_arm.joint_names",
        {"openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3", "openarm_left_joint4",
         "openarm_left_joint5", "openarm_left_joint6", "openarm_left_joint7"});
    left_arm_.command_topic = declare_parameter<std::string>(
        "left_arm.command_topic", "/left_gravity_compensation_controller/commands");

    right_arm_.name = "right_arm";
    right_arm_.enabled = declare_parameter<bool>("right_arm.enabled", true);
    right_arm_.alpha = declare_parameter<double>("right_arm.alpha", 0.9);
    right_arm_.root_link = declare_parameter<std::string>("right_arm.root_link", "openarm_body_link0");
    right_arm_.leaf_link = declare_parameter<std::string>("right_arm.leaf_link", "openarm_right_hand");
    right_arm_.joint_names = declare_parameter<std::vector<std::string>>(
        "right_arm.joint_names",
        {"openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3", "openarm_right_joint4",
         "openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7"});
    right_arm_.command_topic = declare_parameter<std::string>(
        "right_arm.command_topic", "/right_gravity_compensation_controller/commands");

    left_arm_.last_log_time = now();
    right_arm_.last_log_time = now();

    left_arm_.publisher = create_publisher<std_msgs::msg::Float64MultiArray>(left_arm_.command_topic, 10);
    right_arm_.publisher =
        create_publisher<std_msgs::msg::Float64MultiArray>(right_arm_.command_topic, 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 50,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(joint_state_mutex_);
          latest_joint_state_ = msg;
        });

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&GravityCompensationNode::update, this));

    RCLCPP_INFO(get_logger(),
                "Gravity compensation node started (rate=%.1fHz, source=%s)",
                publish_rate_hz_, robot_description_source_node_.c_str());
  }

 private:
  bool init_dynamics_once() {
    if (dynamics_initialized_) {
      return true;
    }

    auto param_client_node = std::make_shared<rclcpp::Node>(
        "gravity_compensation_param_client",
        rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));
    auto param_client =
        std::make_shared<rclcpp::SyncParametersClient>(param_client_node, robot_description_source_node_);

    if (!param_client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting robot_description from node: %s",
                           robot_description_source_node_.c_str());
      return false;
    }

    const auto values = param_client->get_parameters({"robot_description"});
    if (values.empty() || values[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "robot_description not available on node: %s",
                            robot_description_source_node_.c_str());
      return false;
    }

    const std::string robot_description = values[0].as_string();
    if (robot_description.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "robot_description is empty on node: %s",
                            robot_description_source_node_.c_str());
      return false;
    }

    if (left_arm_.enabled) {
      left_arm_.dynamics =
          std::make_unique<Dynamics>(robot_description, left_arm_.root_link, left_arm_.leaf_link);
      if (!left_arm_.dynamics->init()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize left arm dynamics");
        return false;
      }
    }

    if (right_arm_.enabled) {
      right_arm_.dynamics =
          std::make_unique<Dynamics>(robot_description, right_arm_.root_link, right_arm_.leaf_link);
      if (!right_arm_.dynamics->init()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize right arm dynamics");
        return false;
      }
    }

    dynamics_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Dynamics initialized for gravity feedforward.");
    return true;
  }

  bool map_joint_indices(const sensor_msgs::msg::JointState& js, ArmConfig& arm) {
    arm.joint_indices.clear();
    arm.joint_indices.reserve(arm.joint_names.size());

    std::unordered_map<std::string, int> name_to_index;
    name_to_index.reserve(js.name.size());
    for (std::size_t i = 0; i < js.name.size(); ++i) {
      name_to_index[js.name[i]] = static_cast<int>(i);
    }

    for (const auto& joint_name : arm.joint_names) {
      const auto it = name_to_index.find(joint_name);
      if (it == name_to_index.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Joint '%s' not found in /joint_states for %s",
                             joint_name.c_str(), arm.name.c_str());
        arm.joint_indices.clear();
        return false;
      }
      arm.joint_indices.push_back(it->second);
    }

    return true;
  }

  void compute_and_publish(const sensor_msgs::msg::JointState& js, ArmConfig& arm) {
    if (!arm.enabled || !arm.dynamics) {
      return;
    }

    if (arm.joint_indices.size() != arm.joint_names.size()) {
      if (!map_joint_indices(js, arm)) {
        return;
      }
    }

    std::vector<double> joint_positions(arm.joint_indices.size(), 0.0);
    for (std::size_t i = 0; i < arm.joint_indices.size(); ++i) {
      const auto idx = static_cast<std::size_t>(arm.joint_indices[i]);
      if (idx >= js.position.size()) {
        return;
      }
      joint_positions[i] = js.position[idx];
    }

    std::vector<double> gravity_torques;
    if (!arm.dynamics->get_gravity(joint_positions, gravity_torques)) {
      return;
    }

    for (auto& torque : gravity_torques) {
      torque *= arm.alpha;
    }

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = gravity_torques;
    arm.publisher->publish(cmd);

    const auto now = this->now();
    if ((now - arm.last_log_time).seconds() >= log_period_sec_) {
      std::ostringstream oss;
      oss << arm.name << " gravity ff torques:";
      for (std::size_t i = 0; i < gravity_torques.size(); ++i) {
        oss << " J" << (i + 1) << "=" << gravity_torques[i];
      }
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
      arm.last_log_time = now;
    }
  }

  void update() {
    if (!init_dynamics_once()) {
      return;
    }

    sensor_msgs::msg::JointState::SharedPtr js;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      js = latest_joint_state_;
    }

    if (!js) {
      return;
    }

    compute_and_publish(*js, left_arm_);
    compute_and_publish(*js, right_arm_);
  }

  double publish_rate_hz_{200.0};
  double log_period_sec_{5.0};
  std::string robot_description_source_node_;

  bool dynamics_initialized_{false};
  ArmConfig left_arm_;
  ArmConfig right_arm_;

  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace openarm_gravity_compensation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<openarm_gravity_compensation::GravityCompensationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
