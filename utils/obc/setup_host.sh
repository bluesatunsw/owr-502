#!/bin/bash

###############################################################################
####                       Rover OBC preparation script                    ####
###############################################################################

# Lets us access cameras
sudo gpasswd -a owr video

# Allow anyone to access cameras, by default only root has camera perms
RULES_FILE="/etc/udev/rules.d/can-conv.rules"
echo 'KERNEL=="can[0-9]*", SUBSYSTEM=="net", SUBSYSTEMS=="usb", ATTRS{serial}=="003F00544543500120353148", NAME="can-all"' > "${RULES_FILE}"
echo 'KERNEL=="can[0-9]*", SUBSYSTEM=="net", SUBSYSTEMS=="usb", ATTRS{serial}=="004000544543500120353148", NAME="can-vesc"' >> "${RULES_FILE}"
udevadm control --reload-rules
udevadm trigger
