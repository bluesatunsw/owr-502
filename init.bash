#!/bin/bash

# Run this to automatically set up your term environment
source /ros_entrypoint.sh
for i in $(seq 1 10);
do
    $SHELL
done
