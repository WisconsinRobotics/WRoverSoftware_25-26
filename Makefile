inst_dep:
	sudo apt-get update
	rosdep install --from-path src
build_env:
	colcon build
	echo "DONT FORGET TO SOURCE!"
run:
	ros2 launch simulation simulation.launch.py

run_and_build: install_dependencies build run
