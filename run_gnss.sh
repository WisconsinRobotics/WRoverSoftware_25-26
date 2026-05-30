#!/bin/bash
source install/setup.bash
ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -r __ns:=/rover1 -p DEVICE_SERIAL_STRING:="'3'" &
ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -r __ns:=/rover2 -p DEVICE_SERIAL_STRING:="'4'" &
ros2 run wr_imu_compass compass &
ros2 run gps_tools heading &
