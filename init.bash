#!/bin/bash

# Run this to automatically set up your term environment
source /ros_entrypoint.sh
source /opt/ros/jazzy/setup.bash
for i in $(seq 1 10);
do
    $SHELL
done
