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
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rover"), "description", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover"),
            "config",
            "joint_state_broadcaster.yaml",
        ]
    )

    jsp_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            robot_controllers,
        ],
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "pointcloud.enable": "true"
        }.items()
    )


    usb_cam_node = Node(
        package='usb_cam', executable='usb_cam_node_exe',
        namespace="webcam",
        parameters=[{
            "video_device" : Command("realpath /dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0"),
            "frame_id" : "webcam_frame",
            "camera_name" : "webcam"
        }],
    )

    return LaunchDescription(
        [
            realsense_launch,
            usb_cam_node,
            jsp_spawner,
            node_robot_state_publisher,
        ]
    )
