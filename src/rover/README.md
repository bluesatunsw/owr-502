## Structure of description folder

`<xacro:include>`'s / xacro files in general are processed with a guaranteed evaluation order (top to bottom).

Thus all 'helper' files used across multiple subfiles have been included directly in the largest scope, `robot.urdf.xacro`

- robot.urdf.xacro
    - utils/inertial_macros.xacro
    - utils/materials.xacro
    - chassis/chassis.xacro
    - drivebase/drivebase.xacro
        - drivebase/db_helpers.xacro
        - ros2_control.xacro

## Important launch files

- `rsp.launch.py`
  - This will use the URDF to show a model of the rover and the full TF tree
  - Use rviz2 to view this, 
    - set `Fixed Frame` to `base_link`
    - Add a RobotModel, set `Description Topic` to `/robot_description`
    - (Optional) add a TF

### You may also want to launch:

- `ros2 run key_teleop key_teleop --ros-args -p twist_stamped_enabled:=True -r /key_vel:=/swerve_drive_base_controller/cmd_vel`
    - Runs a small TUI application that listens to arrow keys, and publishes a twist based on this (used for driving around the robot)
    - You will need to install `ros-jazzy-key-teleop` (with apt) first