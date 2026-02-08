from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    camera = DeclareLaunchArgument(
        "camera", default_value="/dev/video0", description="Changes the camera used"
    )

    camera_param = LaunchConfiguration("camera")

    return LaunchDescription(
        [
            camera,
            launch_ros.actions.Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="decxin_cam",
                parameters=[
                    {"video_device": camera_param},
                    {"pixel_format": "mjpeg2rgb"},
                    {"frame_id": "base_link"},  # FIXME This currently attaches every camera to the same spot on the rover 
                    {
                        "camera_info_url": "package://rover/config/DECXIN_camera/ost.yaml"
                    },
                ],
            ),
        ]
    )
