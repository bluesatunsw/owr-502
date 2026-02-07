#!/bin/bash

# Run without args to set up your terminal environment
# Run `./init.bash owr_update` to run rosdep
function owr_run_subshells {
    if [ $SHLVL -le 10 ]; then
        /bin/bash -c "$0 owr_run_subshells"
        /bin/bash; echo "WARN: Exited subshell $SHLVL"
    else 
        echo "Initialized!"
    fi
}

function owr_update {
    rosdep update && rosdep install --from-paths src --ignore-src -y
}

if [[ $# -gt 0 ]]; then "$@"; exit; fi      # Run func named by first arg
source /ros_entrypoint.sh
owr_run_subshells
