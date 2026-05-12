# WRoverSoftware_25-26
Official Wisconsin Robotics software repository for the 2026 University Rover Challenge, containing autonomy, control, and base station code.

## Notes
Documentation about the software being used, resources for development, things that need to be done, can all be found in [/docs/](./docs/)


## How To Set Up Simulation
starting from the rot of the WiscRobo directory (where this readme is).
NOTE: for development purposes, it is recommended to use VScode seeing as there are custom
build commands and compile tasks in VSCode

### Prereqs
These are required to be installed on the system in order for the simulation to work.
```
sudo apt install python3-colcon-common-extensions
sudo apt-get install python3-rosdep2
sudo apt install ros-humble-joint-state-publisher-gui
```

### Raw Build and Run Commands
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

### Shortcuts
``` bash
# For fast builds use fast_build script at the top level
# This will do the entire build, source, and run routine for you
# make sure to chmod the script
./fast_build

# clean repo
make clean

# install ros dependencies
make inst_dep

# build project 
make build_env

# run after source
make run
```

