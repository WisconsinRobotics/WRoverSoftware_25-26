#!/bin/bash

ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -r __ns:=/rover1 -p device_serial_string:="'1'" &
ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -r __ns:=/rover2 -p device_serial_string:="'2'" &
ros2 run wr_imu_compass compass &
ros2 run gps_tools single_heading &
ros2 run master_finder master_finder &
ros2 run wr_path_planning path_planner --ros-args -r lidar_path:="~/motel.laz" &
