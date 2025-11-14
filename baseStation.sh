#!/bin/bash

source install/local_setup.bash

cd launch
ros2 launch swerve_launch_base.py
cd ..

#ros2 launch arm_ik demo.launch.py
