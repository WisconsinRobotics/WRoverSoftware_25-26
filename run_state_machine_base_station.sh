#!/bin/bash

LIDAR_PATH="$1"

ros2 run master_finder master_finder &
ros2 run wr_path_planning path_planner --ros-args -r lidar_path:="$LIDAR_PATH" &
ros2 run state_machine state_machine_controller
