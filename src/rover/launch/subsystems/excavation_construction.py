

from launch_ros.actions import Node


def generate_launch_description():
    # Launch excavation/construction specific cameras (just the arm?)

    # ROS2 Controller spawner for arm controller?
    # Something like below...
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
    )
