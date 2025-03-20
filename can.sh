#!/bin/bash

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate 1000000 dbitrate 1000000 fd on restart-ms 500 berr-reporting on loopback on
sudo ip link set can0 up
