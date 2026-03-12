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

  static const std::string PLANNING_GROUP = "upper_body"; // 定义规划组名称        //需要改成在MoveIt中定义的规划组名称
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP); // 创建MoveGroupInterface对象
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // 创建PlanningSceneInterface
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); // 获取当前状态的关节模型组
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world","openarm_marker_visual_tools", move_group.getRobotModel()); // 创建MoveItVisualTools对象
  
  visual_tools.deleteAllMarkers(); // 删除所有标记
  visual_tools.loadRemoteControl(); 

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z()=1.0;
  visual_tools.publishText(text_pose,"text_demo",rviz_visual_tools::RED,rviz_visual_tools::XLARGE);  //可视化标记文本样例
  
  visual_tools.trigger();
  RCLCPP_INFO(LOGGER, "planning frame: %s", move_group.getPlanningFrame().c_str());
  // RCLCPP_INFO(LOGGER, "end effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "available planning groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;
  // 启动演示
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  

  
  
  
  
  
  rclcpp::shutdown();
  return 0;
}
