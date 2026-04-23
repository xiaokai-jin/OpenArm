#include <memory>
#include <map>
#include <iterator>
#include <thread>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("openarm_manipulation");
static const std::string ERROR_CSV_PATH = "/home/xiaokai/OpenArm/repeatability_error.csv";

struct PoseErrorMetrics
{
  double dx{0.0};
  double dy{0.0};
  double dz{0.0};
  double position_error{0.0};
  double roll_error_deg{0.0};
  double pitch_error_deg{0.0};
  double yaw_error_deg{0.0};
  double orientation_error_deg{0.0};
};

static PoseErrorMetrics calculatePoseErrorMetrics(
  const geometry_msgs::msg::Pose & target_pose,
  const geometry_msgs::msg::Pose & actual_pose)
{
  PoseErrorMetrics metrics;
  metrics.dx = actual_pose.position.x - target_pose.position.x;
  metrics.dy = actual_pose.position.y - target_pose.position.y;
  metrics.dz = actual_pose.position.z - target_pose.position.z;
  metrics.position_error = std::sqrt(
    metrics.dx * metrics.dx + metrics.dy * metrics.dy + metrics.dz * metrics.dz);

  tf2::Quaternion q_target;
  tf2::Quaternion q_actual;
  tf2::fromMsg(target_pose.orientation, q_target);
  tf2::fromMsg(actual_pose.orientation, q_actual);
  q_target.normalize();
  q_actual.normalize();

  const tf2::Quaternion q_delta = q_target.inverse() * q_actual;
  const double sin_half_angle = std::sqrt(
    q_delta.x() * q_delta.x() + q_delta.y() * q_delta.y() + q_delta.z() * q_delta.z());
  const double orientation_error_rad = 2.0 * std::atan2(sin_half_angle, std::fabs(q_delta.w()));

  double roll_error = 0.0;
  double pitch_error = 0.0;
  double yaw_error = 0.0;
  tf2::Matrix3x3(q_delta).getRPY(roll_error, pitch_error, yaw_error);

  metrics.roll_error_deg = roll_error * 180.0 / M_PI;
  metrics.pitch_error_deg = pitch_error * 180.0 / M_PI;
  metrics.yaw_error_deg = yaw_error * 180.0 / M_PI;
  metrics.orientation_error_deg = orientation_error_rad * 180.0 / M_PI;

  return metrics;
}

static void appendPoseErrorToCsv(
  const std::string & name,
  const geometry_msgs::msg::Pose & target_pose,
  const geometry_msgs::msg::Pose & actual_pose,
  const PoseErrorMetrics & metrics)
{
  std::ifstream csv_check(ERROR_CSV_PATH);
  const bool need_header = !csv_check.good() || csv_check.peek() == std::ifstream::traits_type::eof();
  csv_check.close();

  std::ofstream csv_file(ERROR_CSV_PATH, std::ios::out | std::ios::app);
  if (!csv_file.is_open())
  {
    RCLCPP_ERROR(LOGGER, "Failed to open CSV file for writing: %s", ERROR_CSV_PATH.c_str());
    return;
  }

  if (need_header)
  {
    csv_file << "timestamp,name,"
             << "target_x,target_y,target_z,target_qx,target_qy,target_qz,target_qw,"
             << "actual_x,actual_y,actual_z,actual_qx,actual_qy,actual_qz,actual_qw,"
             << "dx,dy,dz,position_error_m,roll_error_deg,pitch_error_deg,yaw_error_deg,orientation_error_deg\n";
  }

  const auto now = std::chrono::system_clock::now();
  const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm{};
  localtime_r(&now_c, &now_tm);

  csv_file << std::put_time(&now_tm, "%F %T") << ','
           << name << ','
           << target_pose.position.x << ','
           << target_pose.position.y << ','
           << target_pose.position.z << ','
           << target_pose.orientation.x << ','
           << target_pose.orientation.y << ','
           << target_pose.orientation.z << ','
           << target_pose.orientation.w << ','
           << actual_pose.position.x << ','
           << actual_pose.position.y << ','
           << actual_pose.position.z << ','
           << actual_pose.orientation.x << ','
           << actual_pose.orientation.y << ','
           << actual_pose.orientation.z << ','
           << actual_pose.orientation.w << ','
           << metrics.dx << ','
           << metrics.dy << ','
           << metrics.dz << ','
           << metrics.position_error << ','
           << metrics.roll_error_deg << ','
           << metrics.pitch_error_deg << ','
           << metrics.yaw_error_deg << ','
           << metrics.orientation_error_deg << '\n';
}

static void reportPoseError(
  const std::string & name,
  const geometry_msgs::msg::Pose & target_pose,
  const geometry_msgs::msg::Pose & actual_pose)
{
  const PoseErrorMetrics metrics = calculatePoseErrorMetrics(target_pose, actual_pose);

  RCLCPP_INFO(
    LOGGER,
    "[%s] position error: dx=%.6f m, dy=%.6f m, dz=%.6f m, norm=%.6f m | "
    "orientation error: roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg, angle=%.3f deg",
    name.c_str(),
    metrics.dx, metrics.dy, metrics.dz, metrics.position_error,
    metrics.roll_error_deg,
    metrics.pitch_error_deg,
    metrics.yaw_error_deg,
    metrics.orientation_error_deg);

  appendPoseErrorToCsv(name, target_pose, actual_pose, metrics);
}

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
    // auto actual_pose_right_1 = move_group_upper_body.getCurrentPose("openarm_right_hand_tcp").pose;
    // auto actual_pose_left_1 = move_group_upper_body.getCurrentPose("openarm_left_hand_tcp").pose;
    // reportPoseError("pose1_right", target_pose1_right, actual_pose_right_1);
    // reportPoseError("pose1_left", target_pose1_left, actual_pose_left_1);
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
    // auto actual_pose_right_2 = move_group_upper_body.getCurrentPose("openarm_right_hand_tcp").pose;
    // auto actual_pose_left_2 = move_group_upper_body.getCurrentPose("openarm_left_hand_tcp").pose;
    // reportPoseError("pose2_right", target_pose2_right, actual_pose_right_2);
    // reportPoseError("pose2_left", target_pose2_left, actual_pose_left_2);
  }
  move_group_upper_body.clearPathConstraints(); // 清除路径约束，否则后续规划会一直受限于这个约束
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  

  rclcpp::shutdown();
  return 0;
}
