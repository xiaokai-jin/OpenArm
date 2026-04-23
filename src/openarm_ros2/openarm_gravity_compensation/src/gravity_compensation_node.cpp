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

// 每条手臂的运行配置与运行时缓存。
// 设计成结构体的好处：左右臂逻辑共用一套函数，避免重复代码。
struct ArmConfig {
  // 仅用于日志展示（left_arm / right_arm）
  std::string name;
  // 是否启用该侧补偿
  bool enabled{true};
  // 重力前馈比例系数：tau_ff = alpha * tau_g
  double alpha{0.9};
  // KDL 链路端点
  std::string root_link;
  std::string leaf_link;
  // 与 /joint_states 对齐的关节名称顺序
  std::vector<std::string> joint_names;
  // effort 前馈控制器的命令话题
  std::string command_topic;

  // 动力学实例（unique_ptr 表示独占所有权）
  std::unique_ptr<Dynamics> dynamics;
  // 运行时缓存：joint 名 -> 索引映射结果
  std::vector<int> joint_indices;
  // 发布器：向 forward_command_controller 发送 Float64MultiArray
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
  // 节流日志时间戳
  rclcpp::Time last_log_time{0};
};

class GravityCompensationNode : public rclcpp::Node {
 public:
  GravityCompensationNode() : Node("gravity_compensation_node") {
    // -----------------------------
    // 1) 读取全局参数
    // -----------------------------
    // declare_parameter<T>(name, default)
    // 语法说明：模板参数 T 明确该参数的期望类型。
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 200.0);
    log_period_sec_ = declare_parameter<double>("log_period_sec", 15.0);
    robot_description_source_node_ =
        declare_parameter<std::string>("robot_description_source_node", "/robot_state_publisher");

    // -----------------------------
    // 2) 左臂参数
    // -----------------------------
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

    // -----------------------------
    // 3) 右臂参数
    // -----------------------------
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

    // 用节点时钟初始化日志时间，确保 time source 一致。
    // （避免 ROS time / system time 混用时出现 subtract 异常）
    left_arm_.last_log_time = now();
    right_arm_.last_log_time = now();

    // -----------------------------
    // 4) 创建发布器
    // -----------------------------
    left_arm_.publisher = create_publisher<std_msgs::msg::Float64MultiArray>(left_arm_.command_topic, 10);
    right_arm_.publisher =
        create_publisher<std_msgs::msg::Float64MultiArray>(right_arm_.command_topic, 10);

    // -----------------------------
    // 5) 订阅 joint_states
    // -----------------------------
    // 这里使用 lambda 回调：[this](...) { ... }
    // - [this] 表示捕获当前对象指针，允许在 lambda 里访问成员变量。
    // - SharedPtr 避免消息对象在回调返回后被提前释放。
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 50,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 多线程安全：订阅回调与定时器回调可能并发访问 latest_joint_state_
          std::lock_guard<std::mutex> lock(joint_state_mutex_);
          latest_joint_state_ = msg;
        });

    // -----------------------------
    // 6) 创建定时器（主循环）
    // -----------------------------
    // publish_rate_hz_ 如果被错误配置为 <=0，用 std::max 做下限保护。
    using namespace std::chrono_literals;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));

    // std::bind(&Class::method, this) 把成员函数绑定成可调用对象给定时器。
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&GravityCompensationNode::update, this));

    RCLCPP_INFO(get_logger(),
                "Gravity compensation node started (rate=%.1fHz, source=%s)",
                publish_rate_hz_, robot_description_source_node_.c_str());
  }

 private:
  // 延迟初始化动力学：
  // 只有当 robot_description 可读后才构建 KDL，避免启动顺序导致的空参数问题。
  bool init_dynamics_once() {
    if (dynamics_initialized_) {
      return true;
    }

    // 单独创建参数客户端节点，避免把当前 node 再次加入 executor 造成冲突。
    auto param_client_node = std::make_shared<rclcpp::Node>(
        "gravity_compensation_param_client",
        rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));
    auto param_client =
        std::make_shared<rclcpp::SyncParametersClient>(param_client_node, robot_description_source_node_);

    // 等待参数服务可用（2 秒），不可用时节流告警并重试。
    if (!param_client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Waiting robot_description from node: %s",
                           robot_description_source_node_.c_str());
      return false;
    }

    // 一次性读取 robot_description（URDF XML 字符串）
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
      // make_unique 创建对象并返回 unique_ptr，语义清晰且异常安全。
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

  // 把 joint name 映射成 joint_states 中的索引，减少每周期字符串查找。
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
        // THROTTLE 版本日志：防止缺关节名时刷屏
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

  // 单臂计算 + 发布。
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
      // arm.joint_indices 存的是 int，这里转 size_t 以匹配 vector 下标类型。
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

    // 按比例缩放重力补偿：tau_ff = alpha * tau_g
    for (auto& torque : gravity_torques) {
      torque *= arm.alpha;
    }

    // 通过 forward_command_controller 的标准输入消息发布。
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = gravity_torques;
    arm.publisher->publish(cmd);

    // 周期性输出当前前馈力矩，便于在线观察补偿是否生效。
    const auto now = this->now();
    // if ((now - arm.last_log_time).seconds() >= log_period_sec_) {
    //   std::ostringstream oss;
    //   oss << arm.name << " gravity ff torques:";
    //   for (std::size_t i = 0; i < gravity_torques.size(); ++i) {
    //     oss << " J" << (i + 1) << "=" << gravity_torques[i];
    //   }
    //   oss << " | joint positions(rad):";
    //   constexpr double kRadToDeg = 57.29577951308232;
    //   for (std::size_t i = 0; i < joint_positions.size(); ++i) {
    //     const double joint_deg = joint_positions[i] * kRadToDeg;
    //     oss << " J" << (i + 1) << "=" << joint_positions[i] << "(" << joint_deg << "deg)";
    //   }
    //   RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    //   arm.last_log_time = now;
    // }
  }

  // 定时器主循环：初始化检查 -> 读取最新状态 -> 左右臂发布
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
      // 启动早期 joint_states 还没到达时直接返回。
      return;
    }

    compute_and_publish(*js, left_arm_);
    compute_and_publish(*js, right_arm_);
  }

  double publish_rate_hz_{200.0};
  double log_period_sec_{15.0};
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
  // 标准 ROS2 程序入口：初始化 -> spin -> 清理
  rclcpp::init(argc, argv);
  auto node = std::make_shared<openarm_gravity_compensation::GravityCompensationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
