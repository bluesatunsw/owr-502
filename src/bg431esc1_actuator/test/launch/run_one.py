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
                [FindPackageShare("bg431esc1_actuator"), "description", "urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("bg431esc1_actuator"),
            "config",
            "test_actuator.yaml",
        ]
    )

    # Tell the controller_manager to spawn the forward command controller in
    forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "test_actuator",
            "--param-file",
            robot_controllers,
        ],
    )

    # Spawn in the controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[],
        output="both",
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_robot_joint_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            control_node,
            node_robot_state_publisher,
            node_robot_joint_publisher,
            forward_controller_spawner,
        ]
    )
