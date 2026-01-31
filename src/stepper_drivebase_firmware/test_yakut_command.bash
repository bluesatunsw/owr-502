#!/bin/bash

POSITION=${1:-0.0}
TARGET=${2:-3000}

echo $POSITION
echo $TARGET

yakut -i "CAN(can.media.socketcan.SocketCANMedia('can0', 64),100)" \
      pub --count 1 ${TARGET}:reg.udral.physics.dynamics.rotation.Planar\
      "{kinematics: {angular_position: $POSITION, angular_velocity: 0.0, angular_acceleration: 0.0 }}"
