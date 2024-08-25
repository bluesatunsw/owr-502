#!/bin/bash

# Run this to automatically set up your term environment
source /ros_entrypoint.sh
source /workspaces/owr-502/install/local_setup.bash
for i in $(seq 1 10);
do
    $SHELL
done