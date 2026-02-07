#!/bin/bash

sudo modprobe can
sudo modprobe can_raw

sudo ip link set can-all down 
sudo ip link set can-all type can bitrate 1000000 dbitrate 8000000 fd on loopback off
sudo ip link set can-all up

sudo ip link set can-vesc down 
sudo ip link set can-vesc type can bitrate 1000000 fd off loopback off
sudo ip link set can-vesc up
