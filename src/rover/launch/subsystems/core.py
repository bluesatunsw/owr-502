import launch
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Start the ROS2Control Controller Manager (we just config'd frequency etc..)





    # Publish Joint States of all ROS2Control hardware?




    # Publish Robot State
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )





    # Get URDF via xacro




    # Drivebase ROS2Control Controller
    swerve_controller = PathJoinSubstitution([
        FindPackageShare("rover"),
        "config",
        "core",
        "swerve_drive_controller.yaml",
    ])
    
    swerve_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "swerve_drive_base_controller",
            "--param-file",
            robot_controllers,
        ],
    )                    



    return LaunchDescription([
        swerve_controller_spawner
        node_robot_state_publisher,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [FindPackageShare("usb_cam"), "launch", "camera.launch.py"]
                    )
                ]
            )
        ),
    ])
