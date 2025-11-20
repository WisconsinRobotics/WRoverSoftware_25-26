SHELL := /bin/bash


run_battery_telemetry: compile 
	ls ./install/local_setup.sh
	source ./install/local_setup.sh && ros2 run telemetry battery_data_sender --ros-args -p esp_com_port:="/dev/ttyUSB0"

compile:
	rosdep install --from-path src --ignore-src -r -y
	colcon build

help:
	@echo "commands"
	@echo "build - builds project"
	@echo "run_battery_telemetry - runs the telemetry nodes"
