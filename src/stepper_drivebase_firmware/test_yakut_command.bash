#!/bin/bash

yakut -i "CAN(can.media.socketcan.SocketCANMedia('can0', 64),100)" pub 3000:reg.udral.physics.dynamics.rotation.Planar '{kinematics: {angular_position: 3.14, angular_velocity: 0.0, angular_acceleration: 0.0 }}'
