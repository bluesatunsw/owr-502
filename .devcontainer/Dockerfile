FROM ros:jazzy
ARG USERNAME=dcuser
ARG USER_UID=1010
ARG USER_GID=1010

# Create the user
RUN groupadd --gid $USER_GID $USERNAME
RUN useradd --uid $USER_UID --gid $USER_GID -m $USERNAME
RUN apt-get update
RUN apt-get install -y sudo
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get install -y python3-pip
RUN apt-get update && apt-get upgrade -y

# ********************************************************
# *                 ROS PAGKAGES                         *
# ********************************************************

# Install ros desktop
RUN apt-get install -y ros-jazzy-desktop

# Install additional ros packages
RUN apt-get install -y ros-jazzy-xacro \ 
    ros-jazzy-joint-state-publisher \
    ros-jazzy-librealsense2 ros-jazzy-realsense2-camera

# Gazebo
RUN apt-get install -y ros-jazzy-ros-gz 

# Ros2Control
RUN apt-get install -y ros-jazzy-gz-ros2-control ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller

# ********************************************************
# *                END ROS PAGKAGES                         *
# ********************************************************


# Set up user env, password is an empty string (encrypted)
RUN chsh -s $(which bash) $USERNAME
RUN echo $USERNAME:U6aMy0wojraho | chpasswd -e
ENV SHELL /bin/bash

# Set up ssh
RUN apt-get install -y openssh-server
RUN mkdir /var/run/sshd
# X11Forwarding yes is already set for ros
RUN echo 'PasswordAuthentication yes\nPermitEmptyPasswords yes\nPermitRootLogin yes' >> /etc/ssh/sshd_config
EXPOSE 22

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# Additional ROS setup
ENV ROS_DOMAIN_ID 42

# Set the default user.
USER $USERNAME
CMD ["sudo", "/usr/sbin/sshd", "-D", "-e"]
