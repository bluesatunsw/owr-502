import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.descriptions import ParameterValue
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # FIXME: Should actually feed these arguments into xacro!
    decl_args = []
    decl_args.append(
        DeclareLaunchArgument(
            "task_excavation_construction",
            default_value="false",
        )
    )
    decl_args.append(
        DeclareLaunchArgument(
            "task_mapping_autonomous",
            default_value="false",
        )
    )
    decl_args.append(
        DeclareLaunchArgument(
            "task_post_landing",
            default_value="false",
        )
    )
    decl_args.append(
        DeclareLaunchArgument(
            "task_space_resources",
            default_value="false",
        )
    )

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory("rover"))
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    robot_description_config = Command(
        [
            "xacro ",
            xacro_file,
        ]
    )

    # Create a robot_state_publisher node
    params = {
        "robot_description": ParameterValue(robot_description_config, value_type=str),
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    node_robot_joint_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Launch!
    return LaunchDescription(
        [
            node_robot_joint_publisher,
            node_robot_state_publisher,
        ]
    )
