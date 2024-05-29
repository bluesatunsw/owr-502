FROM ros:iron
ARG USERNAME=dcuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

# Install ros desktop
RUN apt-get install -y ros-iron-desktop

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

# Set the default user.
USER $USERNAME
CMD ["sudo", "/usr/sbin/sshd", "-D", "-e"]