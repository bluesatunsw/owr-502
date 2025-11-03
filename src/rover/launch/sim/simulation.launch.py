# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover"),
            "config",
            "drivebase",
            "swerve_drive_controller.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "rover",
            "-allow_renaming",
            "true",
            "-z",
            "7.6",  # Spawn ABOVE the surface
        ],
    )

    swerve_controller_spawn_trigger =  RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[swerve_drive_base_controller_spawner],
                )
            )

    swerve_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "swerve_drive_base_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    bridge_params = PathJoinSubstitution(
        [
            FindPackageShare("rover"),
            "config",
            "gazebo",
            "bridge.yaml",
        ]
    )

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_params}],
        output="screen",
    )

    return LaunchDescription(
        [
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "gz_args": [" -r -v 4 dem_moon.sdf"],
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            node_robot_state_publisher,
            start_gazebo_ros_bridge_cmd,
            gz_spawn_entity,
            swerve_controller_spawn_trigger,
        ]
    )
