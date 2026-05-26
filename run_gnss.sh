#!/bin/bash

ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -r __ns:=/rover1 -p device_serial_string:="'1'" &
ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -r __ns:=/rover2 -p device_serial_string:="'2'"
