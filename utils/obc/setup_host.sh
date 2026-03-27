#!/bin/bash

###############################################################################
####                       Rover OBC preparation script                    ####
###############################################################################

# Lets us access cameras
sudo gpasswd -a owr video

# Allow anyone to access cameras, by default only root has camera perms
CAN_RULES_FILE="/etc/udev/rules.d/can-conv.rules"
echo 'KERNEL=="can[0-9]*", SUBSYSTEM=="net", SUBSYSTEMS=="usb", ATTRS{serial}=="003F00544543500120353148", NAME="can-all"' > "${CAN_RULES_FILE}"
echo 'KERNEL=="can[0-9]*", SUBSYSTEM=="net", SUBSYSTEMS=="usb", ATTRS{serial}=="004000544543500120353148", NAME="can-vesc"' >> "${CAN_RULES_FILE}"

KINECT_RULES_FILE="/etc/udev/rules.d/kinect.rules"
# ATTR{product}=="Kinect2"
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666"' > "${KINECT_RULES_FILE}"
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666"' >> "${KINECT_RULES_FILE}"
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"' >> "${KINECT_RULES_FILE}"

CAMERA_RULES_FILE="/etc/udev/rules.d/decxin-cam.rules"
{ \
    echo 'KERNEL=="video[02468]", SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{busnum}=="2", ATTRS{devnum}=="3", SYMLINK+="video-polef"'; \
    echo 'KERNEL=="video[02468]", SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{busnum}=="2", ATTRS{devnum}=="4", SYMLINK+="video-polel"'; \
    echo 'KERNEL=="video[02468]", SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{busnum}=="2", ATTRS{devnum}=="5", SYMLINK+="video-poler"'; \
    echo 'KERNEL=="video[02468]", SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{busnum}=="2", ATTRS{devnum}=="6", SYMLINK+="video-front"'; \
    echo 'KERNEL=="video[02468]", SUBSYSTEM=="video4linux", SUBSYSTEMS=="usb", ATTRS{busnum}=="4", ATTRS{devnum}=="2", SYMLINK+="video-paver"'; \
} > "${CAMERA_RULES_FILE}"

udevadm control --reload-rules
udevadm trigger
