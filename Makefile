inst_dep:
	sudo apt-get update
	rosdep install --from-path src
build_env:
	colcon build
	echo "DONT FORGET TO SOURCE!"
run:
	ros2 launch simulation simulation.launch.py

clean:
	rm -rf build/ install/ log/

run_and_build: install_dependencies build run

fast_build: clean build_env run
