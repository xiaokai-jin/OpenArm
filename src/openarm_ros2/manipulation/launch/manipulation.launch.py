from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()

    manipulation_node = Node(
        package="manipulation",
        executable="manipulation",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([manipulation_node])
