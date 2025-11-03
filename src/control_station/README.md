# Control Station Launch Files

## Important launch files (TODO)
- `xxxx.py` launch lichtblick + foxglove bridge + key_teleop

### You may also want to launch:

- `ros2 run key_teleop key_teleop --ros-args -p twist_stamped_enabled:=True -r /key_vel:=/swerve_drive_base_controller/cmd_vel`
    - Runs a small TUI application that listens to arrow keys, and publishes a twist based on this (used for driving around the robot)
    - You will need to install `ros-jazzy-key-teleop` (with apt) first
