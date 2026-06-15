#!/bin/bash

# Exit on error
set -e

# clearing
clear

# Source ROS2
source /opt/ros/humble/setup.bash

# clean project and build
make clean
make build_env

# Source workspace
source install/setup.bash

# Launch simulation
make run
