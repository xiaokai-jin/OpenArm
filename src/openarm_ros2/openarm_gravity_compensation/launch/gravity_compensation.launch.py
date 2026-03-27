from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("openarm_gravity_compensation"), "config", "gravity_compensation.yaml"]
        ),
        description="Path to gravity compensation parameter file.",
    )

    node = Node(
        package="openarm_gravity_compensation",
        executable="gravity_compensation_node",
        name="gravity_compensation_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        node,
    ])
