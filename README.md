# WRoverSoftware_25-26
Official Wisconsin Robotics software repository for the 2026 University Rover Challenge, containing autonomy, control, and base station code.


# How To Set Up Simulation
starting from the rot of the WiscRobo directory (where this readme is).
NOTE: for development purposes, it is recommended to use VScode seeing as there are custom
build commands and compile tasks in VSCode

### Prereqs
```
# you need to install rosdep, and colcon
# These are used to build the simulation
sudo apt install python3-colcon-common-extensions
sudo apt-get install python3-rosdep2
sudo apt install ros-humble-joint-state-publisher-gui
```

### Raw Commands
```
# Install package dependencies
source /opt/ros/humble/setup.sh
sudo apt-get update
rosdep install --from-path src

# Build projcet
colcon build

# Run Sim
source install/setup.sh
ros2 launch simulation simulation.launch.py
```

### Make File Shortcuts

