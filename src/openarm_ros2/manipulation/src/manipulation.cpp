#include <memory>
#include <map>
#include <iterator>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h> // 包含MoveIt的移动组接口头文件
#include <moveit/planning_scene_interface/planning_scene_interface.h> // 包含MoveIt的规划场景接口头文件
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_robot_state.hpp> // 包含显示机器人状态的消息头文件
#include <moveit_msgs/msg/display_trajectory.hpp> // 包含显示轨迹的消息头文件
#include <moveit_msgs/msg/attached_collision_object.hpp> // 包含附加碰撞对象的消息头文件
#include <moveit_msgs/msg/collision_object.hpp> // 包含碰撞对象的消息头文件
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h> // 包含MoveIt可视化工具的头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("openarm_manipulation");

int main(int argc, char ** argv)
{
  // 1) 初始化 ROS2 节点与执行器
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
    // 未加载到 kinematics 配置时，位姿 IK 很可能失败
    RCLCPP_WARN(
      LOGGER,
      "No kinematics parameters found on this node. Pose IK planning may fail. "
      "Use: ros2 launch manipulation manipulation.launch.py");
  }

  static const std::string PLANNING_GROUP_RIGHT_ARM = "right_arm"; // 定义规划组名称
  static const std::string PLANNING_GROUP_RIGHT_SIDE = "right_side"; // 定义规划组名称        
  static const std::string PLANNING_GROUP_UPPER_BODY = "upper_body"; // 定义规划组名称        

  // 2) 创建 MoveGroup：单臂、单侧、双臂整体
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
  // 3) 双臂联合位姿目标（关键思路）
  //    upper_body 是组合组（left_side + right_side），不是单链。
  //    因此不要指望直接用 upper_body 做“单一末端 IK”。
  //    正确方式：
  //      a. 分别对 left_arm / right_arm 求 IK
  //      b. 把结果合并为 upper_body 关节目标
  //      c. 用 upper_body 一次规划与执行，实现双臂协同

  geometry_msgs::msg::Pose target_pose1_right;
  geometry_msgs::msg::Pose target_pose1_left;
  geometry_msgs::msg::Pose target_pose2_right;
  geometry_msgs::msg::Pose target_pose2_left;
  move_group_upper_body.setPlanningTime(10.0);
  move_group_upper_body.setStartStateToCurrentState();

  // 基于当前位姿构造一个小幅目标，右臂向前上，左臂向后上
  // target_pose1_right = move_group_upper_body.getCurrentPose("openarm_right_hand").pose;
  tf2::Quaternion q;
  q.setRPY(0, -M_PI_2, -M_PI_4*3);
  target_pose1_right.orientation = tf2::toMsg(q); // 右臂末端姿态绕 Y 轴旋转 -90 度 ，绕 Z 轴旋转 -45 度
  target_pose1_right.position.x = 0.2;
  target_pose1_right.position.y = 0;
  target_pose1_right.position.z = 0.3;
  
  // target_pose1_left = move_group_upper_body.getCurrentPose("openarm_left_hand").pose;
  tf2::Quaternion q_left;
  q_left.setRPY(0, -M_PI_2, M_PI_4*3); 
  target_pose1_left.orientation = tf2::toMsg(q_left); // 左臂末端姿态绕 Y 轴旋转 -90 度，绕 Z 轴旋转 45 度
  target_pose1_left.position.x = 0.2;
  target_pose1_left.position.y = 0;
  target_pose1_left.position.z = 0.35;
  std::cout << "Target pose right hand: position(" << target_pose1_right.position.x << ", " 
            << target_pose1_right.position.y << ", " << target_pose1_right.position.z 
            << ") orientation(" << target_pose1_right.orientation.w << ")" << std::endl;
  std::cout << "Target pose left hand: position(" << target_pose1_left.position.x << ", " 
            << target_pose1_left.position.y << ", " << target_pose1_left.position.z 
            << ") orientation(" << target_pose1_left.orientation.w << ", " << target_pose1_left.orientation.x << 
            ", " << target_pose1_left.orientation.y << ", " << target_pose1_left.orientation.z << ")" << std::endl;
  
  visual_tools.publishAxisLabeled(target_pose1_right, "pose1_right"); // 可视化目标姿态
  visual_tools.publishAxisLabeled(target_pose1_left, "pose1_left"); // 可视化目标姿态
  visual_tools.publishText(text_pose,"upper body pose goal",rviz_visual_tools::RED,rviz_visual_tools::XLARGE); // 可视化标记文本样例
  visual_tools.trigger(); // 触发可视化更新
  auto current_state = move_group_upper_body.getCurrentState(10);
  if (!current_state)
  {
    // 没有状态就无法求 IK
    RCLCPP_ERROR(LOGGER, "Failed to get current robot state");
    rclcpp::shutdown();
    return 1;
  }

  const auto * left_arm_jmg = move_group_upper_body.getRobotModel()->getJointModelGroup("left_arm");
  const auto * right_arm_jmg = move_group_upper_body.getRobotModel()->getJointModelGroup("right_arm");
  if (!left_arm_jmg || !right_arm_jmg)
  {
    // 关节组不存在（通常是 SRDF 组名不匹配）
    RCLCPP_ERROR(LOGGER, "Failed to get joint model groups for left_arm/right_arm");
    rclcpp::shutdown();
    return 1;
  }

  // 以当前状态为种子分别求两条臂的 IK
  moveit::core::RobotState target_state(*current_state);
  bool left_ok = target_state.setFromIK(left_arm_jmg, target_pose1_left, "openarm_left_hand_tcp", 1.0);
  bool right_ok = target_state.setFromIK(right_arm_jmg, target_pose1_right, "openarm_right_hand_tcp", 1.0);
  if (!left_ok || !right_ok)
  {
    // 任何一边 IK 失败都会导致联合目标不可用
    RCLCPP_ERROR(
      LOGGER,
      "IK failed: left_ok=%d, right_ok=%d. This means the target pose is unreachable or outside the workspace.",
      left_ok, right_ok);
    rclcpp::shutdown();
    return 1;
  }

  std::map<std::string, double> joint_target;
  const std::vector<std::string> upper_body_joint_names = {
    "openarm_left_finger_joint1",
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
    "openarm_right_finger_joint1",
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7"
  };

  // 将 IK 求出的目标状态按 upper_body 关节名导出
  for (const auto & joint_name : upper_body_joint_names)
  {
    joint_target[joint_name] = target_state.getVariablePosition(joint_name);
  }

  if (!move_group_upper_body.setJointValueTarget(joint_target))
  {
    // 目标越界或关节名不匹配会在这里失败
    RCLCPP_ERROR(LOGGER, "Failed to set joint value target for upper_body");
    rclcpp::shutdown();
    return 1;
  }

  // 4) 统一规划并执行（同一条轨迹同时驱动双臂）
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 
  bool success = (move_group_upper_body.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划并检查是否成功
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    /* 在使用真实机器人时取消注释下面的行 */
  if (success)
  {
    // 用已生成的 my_plan 执行，避免 move() 再次触发内部重规划
    move_group_upper_body.execute(my_plan);
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  // // 规划到关节空间目标
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // // 让我们设置一个关节空间目标并向其移动。这将替换我们上面设置的姿态目标。
  // moveit::core::RobotStatePtr current_state_ = move_group_right_side.getCurrentState(10); // 获取当前状态
  // // 接下来获取该组的当前关节值集。
  // std::vector<double> joint_group_positions;
  // current_state_->copyJointGroupPositions(joint_model_group_right_side, joint_group_positions); // 复制关节组位置
  // // 现在，让我们修改其中一个关节，规划到新的关节空间目标，并可视化计划。
  // joint_group_positions[0] = -1.0; // 弧度
  // bool within_bounds=move_group_right_side.setJointValueTarget(joint_group_positions); // 设置关节空间目标
  // if (!within_bounds)
  // {
  //   RCLCPP_WARN(LOGGER, "The joint space goal is not within bounds. Planning will likely fail.");
  // }
  // success = (move_group_right_side.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划并检查是否成功
  
  moveit_msgs::msg::OrientationConstraint ocm_right;
  ocm_right.link_name = "openarm_right_hand_tcp";
  ocm_right.header.frame_id = "world";
  ocm_right.orientation = target_pose1_right.orientation;
  ocm_right.absolute_x_axis_tolerance = 0.1;
  ocm_right.absolute_y_axis_tolerance = 0.1;
  ocm_right.absolute_z_axis_tolerance = 0.1;
  ocm_right.weight = 1.0;
  moveit_msgs::msg::Constraints constraints;
  // moveit_msgs::msg::OrientationConstraint ocm_left;
  // ocm_left.link_name = "openarm_left_hand_tcp";
  // ocm_left.header.frame_id = "world";
  // ocm_left.orientation = target_pose1_left.orientation;
  // ocm_left.absolute_x_axis_tolerance = 0.1;
  // ocm_left.absolute_y_axis_tolerance = 0.1;
  // ocm_left.absolute_z_axis_tolerance = 0.1;
  // ocm_left.weight = 1.0;
  // constraints.orientation_constraints.push_back(ocm_left);
  constraints.orientation_constraints.push_back(ocm_right);
  move_group_upper_body.setPathConstraints(constraints);

  target_pose2_right.orientation = tf2::toMsg(q); // 右臂末端姿态绕 Y 轴旋转 -90 度 ，绕 Z 轴旋转 -45 度
  target_pose2_right.position.x = 0.25;
  target_pose2_right.position.y = 0.05;
  target_pose2_right.position.z = 0.4;
  target_pose2_left.orientation = tf2::toMsg(q_left); // 左臂末端姿态绕 Y 轴旋转 -90 度，绕 Z 轴旋转 45 度
  target_pose2_left.position.x = 0.25;
  target_pose2_left.position.y = 0.05;
  target_pose2_left.position.z = 0.45;

  visual_tools.publishAxisLabeled(target_pose2_right, "pose2_right"); // 可视化目标姿态
  visual_tools.publishAxisLabeled(target_pose2_left, "pose2_left"); // 可视化目标姿态
  visual_tools.publishText(text_pose,"upper body pose2 goal",rviz_visual_tools::RED,rviz_visual_tools::XLARGE); // 可视化标记文本样例
  visual_tools.trigger(); // 触发可视化更新

  current_state = move_group_upper_body.getCurrentState(10);
  if (!current_state)
  {
    // 没有状态就无法求 IK
    RCLCPP_ERROR(LOGGER, "Failed to get current robot state");
    rclcpp::shutdown();
    return 1;
  }
   moveit::core::RobotState target_state2(*current_state);
  left_ok = target_state2.setFromIK(left_arm_jmg, target_pose2_left, "openarm_left_hand_tcp", 1.0);
  right_ok = target_state2.setFromIK(right_arm_jmg, target_pose2_right, "openarm_right_hand_tcp", 1.0);
  if (!left_ok || !right_ok)
  {
    // 任何一边 IK 失败都会导致联合目标不可用
    RCLCPP_ERROR(
      LOGGER,
      "IK failed: left_ok=%d, right_ok=%d. This means the target pose is unreachable or outside the workspace.",
      left_ok, right_ok);
    rclcpp::shutdown();
    return 1;
  }
  for (const auto & joint_name : upper_body_joint_names)
  {
    joint_target[joint_name] = target_state2.getVariablePosition(joint_name);
  }

  if (!move_group_upper_body.setJointValueTarget(joint_target))
  {
    // 目标越界或关节名不匹配会在这里失败
    RCLCPP_ERROR(LOGGER, "Failed to set joint value target for upper_body");
    rclcpp::shutdown();
    return 1;
  }

  // 4) 统一规划并执行（同一条轨迹同时驱动双臂）
   moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  success = (move_group_upper_body.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS); // 规划并检查是否成功
  RCLCPP_INFO(LOGGER, "Visualizing plan 2(pose goal) %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    /* 在使用真实机器人时取消注释下面的行 */
  if (success)
  {
    // 用已生成的 my_plan 执行，避免 move() 再次触发内部重规划
    move_group_upper_body.execute(my_plan2);
  }
  move_group_upper_body.clearPathConstraints(); // 清除路径约束，否则后续规划会一直受限于这个约束
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 5) 双臂带路径约束在 XY 平面画圆
  //    这里使用“离散圆点 + 每点IK + 每段规划执行”的方式实现。
  //    两个末端TCP保持固定姿态，仅在 XY 平面移动（Z 保持不变）。
  // moveit_msgs::msg::OrientationConstraint ocm_left_circle;
  // ocm_left_circle.link_name = "openarm_left_hand_tcp";
  // ocm_left_circle.header.frame_id = "world";
  // ocm_left_circle.orientation = target_pose2_left.orientation;
  // ocm_left_circle.absolute_x_axis_tolerance = 0.12;
  // ocm_left_circle.absolute_y_axis_tolerance = 0.12;
  // ocm_left_circle.absolute_z_axis_tolerance = 0.12;
  // ocm_left_circle.weight = 1.0;

  moveit_msgs::msg::OrientationConstraint ocm_right_circle;
  ocm_right_circle.link_name = "openarm_right_hand_tcp";
  ocm_right_circle.header.frame_id = "world";
  ocm_right_circle.orientation = target_pose2_right.orientation;
  ocm_right_circle.absolute_x_axis_tolerance = 0.12;
  ocm_right_circle.absolute_y_axis_tolerance = 0.12;
  ocm_right_circle.absolute_z_axis_tolerance = 0.12;
  ocm_right_circle.weight = 1.0;

  moveit_msgs::msg::Constraints circle_constraints;
  // circle_constraints.orientation_constraints.push_back(ocm_left_circle);
  circle_constraints.orientation_constraints.push_back(ocm_right_circle);
  move_group_upper_body.setPathConstraints(circle_constraints);

  geometry_msgs::msg::Pose left_circle_start = move_group_upper_body.getCurrentPose("openarm_left_hand_tcp").pose;
  geometry_msgs::msg::Pose right_circle_start = move_group_upper_body.getCurrentPose("openarm_right_hand_tcp").pose;

  const double radius = 0.03;  // 圆半径（约3cm）
  const int circle_steps = 24; // 离散点数，越大越圆滑
  const double two_pi = 2.0 * M_PI;

  const double left_center_x = left_circle_start.position.x - radius;
  const double left_center_y = left_circle_start.position.y;
  const double right_center_x = right_circle_start.position.x - radius;
  const double right_center_y = right_circle_start.position.y;

  visual_tools.publishText(text_pose, "dual-arm xy circle", rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to draw a dual-arm XY circle");

  bool circle_all_success = true;
  for (int step = 1; step <= circle_steps; ++step)
  {
    const double theta = two_pi * static_cast<double>(step) / static_cast<double>(circle_steps);

    geometry_msgs::msg::Pose left_circle_pose = left_circle_start;
    left_circle_pose.position.x = left_center_x + radius * std::cos(theta);
    left_circle_pose.position.y = left_center_y + radius * std::sin(theta);

    geometry_msgs::msg::Pose right_circle_pose = right_circle_start;
    right_circle_pose.position.x = right_center_x + radius * std::cos(theta);
    right_circle_pose.position.y = right_center_y + radius * std::sin(theta);

    auto circle_state = move_group_upper_body.getCurrentState(2.0);
    if (!circle_state)
    {
      RCLCPP_ERROR(LOGGER, "Failed to get current state during circle motion");
      circle_all_success = false;
      break;
    }

    moveit::core::RobotState circle_target_state(*circle_state);
    bool left_circle_ok = circle_target_state.setFromIK(left_arm_jmg, left_circle_pose, "openarm_left_hand_tcp", 1.0);
    bool right_circle_ok = circle_target_state.setFromIK(right_arm_jmg, right_circle_pose, "openarm_right_hand_tcp", 1.0);
    if (!left_circle_ok || !right_circle_ok)
    {
      RCLCPP_WARN(LOGGER, "Circle IK failed at step %d: left=%d right=%d", step, left_circle_ok, right_circle_ok);
      circle_all_success = false;
      break;
    }

    for (const auto & joint_name : upper_body_joint_names)
    {
      joint_target[joint_name] = circle_target_state.getVariablePosition(joint_name);
    }

    if (!move_group_upper_body.setJointValueTarget(joint_target))
    {
      RCLCPP_WARN(LOGGER, "Failed to set circle joint target at step %d", step);
      circle_all_success = false;
      break;
    }

    moveit::planning_interface::MoveGroupInterface::Plan circle_plan;
    bool circle_plan_ok =
      (move_group_upper_body.plan(circle_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!circle_plan_ok)
    {
      RCLCPP_WARN(LOGGER, "Circle planning failed at step %d", step);
      circle_all_success = false;
      break;
    }

    auto exec_ret = move_group_upper_body.execute(circle_plan);
    if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(LOGGER, "Circle execution failed at step %d", step);
      circle_all_success = false;
      break;
    }
  }

  move_group_upper_body.clearPathConstraints();
  RCLCPP_INFO(LOGGER, "Dual-arm XY circle %s", circle_all_success ? "COMPLETED" : "STOPPED");

  rclcpp::shutdown();
  return 0;
}
