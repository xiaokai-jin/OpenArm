#include <memory>
#include <iterator>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h> // 包含MoveIt的移动组接口头文件
#include <moveit/planning_scene_interface/planning_scene_interface.h> // 包含MoveIt的规划场景接口头文件
#include <moveit_msgs/msg/display_robot_state.hpp> // 包含显示机器人状态的消息头文件
#include <moveit_msgs/msg/display_trajectory.hpp> // 包含显示轨迹的消息头文件
#include <moveit_msgs/msg/attached_collision_object.hpp> // 包含附加碰撞对象的消息头文件
#include <moveit_msgs/msg/collision_object.hpp> // 包含碰撞对象的消息头文件
#include <moveit_visual_tools/moveit_visual_tools.h> // 包含MoveIt可视化工具的头文件

static const rclcpp::Logger LOGGER = rclcpp::get_logger("openarm_manipulation");

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto const move_group_node =std::make_shared<rclcpp::Node>(
    "openarm_manipulation",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node); // 将节点添加到执行器
  std::thread([&executor]() { executor.spin(); }).detach(); // 启动一个线程来运行执行器

  auto kinematics_list = move_group_node->list_parameters({"robot_description_kinematics"}, 10);
  if (kinematics_list.names.empty())
  {
    RCLCPP_WARN(
      LOGGER,
      "No kinematics parameters found on this node. Pose IK planning may fail. "
      "Use: ros2 launch manipulation manipulation.launch.py");
  }

  static const std::string PLANNING_GROUP_RIGHT_ARM = "right_arm"; // 定义规划组名称
  static const std::string PLANNING_GROUP_RIGHT_SIDE = "right_side"; // 定义规划组名称        
  static const std::string PLANNING_GROUP_UPPER_BODY = "upper_body"; // 定义规划组名称        

  moveit::planning_interface::MoveGroupInterface move_group_right_arm(move_group_node, PLANNING_GROUP_RIGHT_ARM); // 创建MoveGroupInterface对象
  moveit::planning_interface::MoveGroupInterface move_group_right_side(move_group_node, PLANNING_GROUP_RIGHT_SIDE); // 创建MoveGroupInterface对象
  moveit::planning_interface::MoveGroupInterface move_group_upper_body(move_group_node, PLANNING_GROUP_UPPER_BODY); // 创建MoveGroupInterface对象
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // 创建PlanningSceneInterface
  const moveit::core::JointModelGroup* joint_model_group_right_side =
      move_group_right_side.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_SIDE); // 获取当前状态的关节模型组
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, 
    "world","openarm_marker_visual_tools", move_group_right_side.getRobotModel()); // 创建MoveItVisualTools对象
  
  visual_tools.deleteAllMarkers(); // 删除所有标记
  visual_tools.loadRemoteControl(); 

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z()=1.0;
  visual_tools.publishText(text_pose,"text_demo",rviz_visual_tools::RED,rviz_visual_tools::XLARGE);  //可视化标记文本样例
  
  visual_tools.trigger();
  RCLCPP_INFO(LOGGER, "planning frame: %s", move_group_right_side.getPlanningFrame().c_str());
  // RCLCPP_INFO(LOGGER, "end effector link: %s", move_group_right_side.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "available planning groups:");
  std::copy(move_group_right_side.getJointModelGroupNames().begin(),
   move_group_right_side.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;
  // 启动演示
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
   // 我们可以为此组规划一个运动到末端执行器的目标姿态。
  geometry_msgs::msg::Pose target_pose1;
  // move_group_right_side.setEndEffectorLink("openarm_right_hand");
  move_group_right_side.setPlanningTime(10.0);
  target_pose1 = move_group_right_side.getCurrentPose("openarm_right_hand").pose;
  target_pose1.position.x += 0.05;
  target_pose1.position.z += 0.05;
  move_group_right_side.setPoseTarget(target_pose1); // 设置姿态目标
  // 现在，我们调用规划器来计算计划并进行可视化。
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 
  bool success = (move_group_right_side.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划并检查是否成功
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.publishAxisLabeled(target_pose1, "pose1"); // 可视化目标姿态
  visual_tools.publishText(text_pose,"right_side pose goal",rviz_visual_tools::RED,rviz_visual_tools::XLARGE); // 可视化标记文本样例
  visual_tools.trigger(); // 触发可视化更新
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    /* 在使用真实机器人时取消注释下面的行 */
  move_group_right_side.move(); 
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  // 规划到关节空间目标
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 让我们设置一个关节空间目标并向其移动。这将替换我们上面设置的姿态目标。
  moveit::core::RobotStatePtr current_state = move_group_right_side.getCurrentState(10); // 获取当前状态
  // 接下来获取该组的当前关节值集。
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group_right_side, joint_group_positions); // 复制关节组位置
  // 现在，让我们修改其中一个关节，规划到新的关节空间目标，并可视化计划。
  joint_group_positions[0] = -1.0; // 弧度
  bool within_bounds=move_group_right_side.setJointValueTarget(joint_group_positions); // 设置关节空间目标
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "The joint space goal is not within bounds. Planning will likely fail.");
  }
  success = (move_group_right_side.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划并检查是否成功
  
  rclcpp::shutdown();
  return 0;
}
