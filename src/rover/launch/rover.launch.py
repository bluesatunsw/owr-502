import launch

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def get_robot_description():
    # Robot Description URDF
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("rover"), "description", "robot.urdf.xacro"]
                    ),
                    " is_simulation:=true",
                ]
            ),
            value_type=str,
        )
    }


def generate_launch_description():
    decl_args = []
    decl_args.append(
        launch.actions.DeclareLaunchArgument(
            "is_simulation",
            default_value="false",
            choices=['true', 'false'],
            description="Start Gazebosim and set up subsystems for simulation usage, combine with other arguments",
        )
    )

    decl_args.append(
        launch.actions.DeclareLaunchArgument(
            "task_excavation_construction",
            default_value="false",
            choices=['true', 'false'],
        )
    )

    decl_args.append(
        launch.actions.DeclareLaunchArgument(
            "task_mapping_autonomous",
            default_value="false",
            choices=['true', 'false'],
        )
    )

    decl_args.append(
        launch.actions.DeclareLaunchArgument(
            "task_post_landing",
            default_value="false",
            choices=['true', 'false'],
        )
    )

    decl_args.append(
        launch.actions.DeclareLaunchArgument(
            "task_space_resources",
            default_value="false",
            choices=['true', 'false'],
        )
    )

    # Core systems (robot state publisher, drivebase, drivebase cameras, etc...)
    core_subsystem = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("rover"), "launch", "subsystems", "core.py"]
                )
            ]
        )
    )

    return LaunchDescription(decl_args + [core_subsystem])
