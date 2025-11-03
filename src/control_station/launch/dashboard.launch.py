from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    foxglove_bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("foxglove_bridge"),
                        "launch",
                        "foxglove_bridge_launch.xml",
                    ]
                )
            ]
        )
    )

    lichtblick = ExecuteProcess(cmd=["lichtblick"])

    return LaunchDescription([foxglove_bridge, lichtblick])
