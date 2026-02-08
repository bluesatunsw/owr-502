
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("rover"), "description", "robot.urdf.xacro"]
                    ),
                ]
            ),
            value_type=str,
        )
    }

    # Start the ROS2Control Controller Manager (we just config'd frequency etc..)
    cm_config = PathJoinSubstitution(
        [
            FindPackageShare("rover"),
            "config",
            "core",
            "shared_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[cm_config],
        output="both",
    )

    # Publish Robot State
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # Publish Joint States of all ROS2Control hardware?
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Drivebase ROS2Control Controller
    swerve_controller_config = PathJoinSubstitution(
        [
            FindPackageShare("rover"),
            "config",
            "core",
            "swerve_drive_controller.yaml",
        ]
    )

    swerve_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "swerve_drive_base_controller",
            "--param-file",
            swerve_controller_config,
        ],
    )

    usb_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("usb_cam"), "launch", "camera.launch.py"]
                )
            ]
        )
    )

    return LaunchDescription(
        [
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            control_node,
            swerve_controller_spawner,
            # usb_cam,
        ]
    )
