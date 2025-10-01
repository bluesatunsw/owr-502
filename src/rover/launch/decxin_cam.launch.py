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

    camera = DeclareLaunchArgument(
            'camera',
            default_value='/dev/video0',
            description="Changes the camera used"
    )
    
    camera_param = LaunchConfiguration('camera')

    return LaunchDescription([
        camera,
        launch_ros.actions.Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="decxin_cam",
                parameters=[{'video_device':camera_param},
                            {'pixel_format':"mjpeg2rgb"},
                            {'frame_id':"base_link"}, #TO CHANGE
                            {'camera_info_url':'package://rover/config/DECXIN_camera/ost.yaml'},
                            ],
            ), 
    ])

