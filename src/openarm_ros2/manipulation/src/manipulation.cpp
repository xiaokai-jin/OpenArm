#include <memory>
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

  static const std::string PLANNING_GROUP_RIGHT_SIDE = "right_side"; // 定义规划组名称        
  static const std::string PLANNING_GROUP_UPPER_BODY = "upper_body"; // 定义规划组名称        

  moveit::planning_interface::MoveGroupInterface move_group_right_side(move_group_node, PLANNING_GROUP_RIGHT_SIDE); // 创建MoveGroupInterface对象
  moveit::planning_interface::MoveGroupInterface move_group_upper_body(move_group_node, PLANNING_GROUP_UPPER_BODY); // 创建MoveGroupInterface对象
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // 创建PlanningSceneInterface
  const moveit::core::JointModelGroup* joint_model_group_right_side =
      move_group_right_side.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_SIDE); // 获取当前状态的关节模型组
  const moveit::core::JointModelGroup* joint_model_group_upper_body =
      move_group_upper_body.getCurrentState()->getJointModelGroup(PLANNING_GROUP_UPPER_BODY); // 获取当前状态的关节模型组
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world","openarm_marker_visual_tools", move_group_right_side.getRobotModel()); // 创建MoveItVisualTools对象
  
  visual_tools.deleteAllMarkers(); // 删除所有标记
  visual_tools.loadRemoteControl(); 

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z()=1.0;
  visual_tools.publishText(text_pose,"text_demo",rviz_visual_tools::RED,rviz_visual_tools::XLARGE);  //可视化标记文本样例
  
  visual_tools.trigger();
  RCLCPP_INFO(LOGGER, "planning frame: %s", move_group_right_side.getPlanningFrame().c_str());
  // RCLCPP_INFO(LOGGER, "end effector link: %s", move_group_right_side.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "available planning groups:");
  std::copy(move_group_right_side.getJointModelGroupNames().begin(), move_group_right_side.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;
  // 启动演示
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
   // 我们可以为此组规划一个运动到末端执行器的目标姿态。
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group_right_side.setPoseTarget(target_pose1); // 设置姿态目标
  // 现在，我们调用规划器来计算计划并进行可视化。
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 
  bool success = (move_group_right_side.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); // 规划并检查是否成功
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  visual_tools.publishAxisLabeled(target_pose1, "pose1"); // 可视化目标姿态
  visual_tools.publishText(text_pose,"right_sidepose goal",rviz_visual_tools::RED,rviz_visual_tools::XLARGE); // 可视化标记文本样例
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_right_side); // 可视化轨迹线
  visual_tools.trigger(); // 触发可视化更新
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    /* 在使用真实机器人时取消注释下面的行 */
  /* move_group_right_side.move(); */
  
  // 规划到关节空间目标
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 让我们设置一个关节空间目标并向其移动。这将替换我们上面设置的姿态目标。
  
  rclcpp::shutdown();
  return 0;
}
