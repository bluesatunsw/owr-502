from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
            
    return LaunchDescription([
        launch_ros.actions.Node(
                package="image_transport",
                executable="republish",
                name="republish",
                output="screen",
                arguments=[{"in/compressed:=/image_raw/compressed"}, {"out:=/image_rawer"}],
                parameters=[{'in_transport':"compressed"},{'out_transport':"raw"}],
            ), 
    ])

