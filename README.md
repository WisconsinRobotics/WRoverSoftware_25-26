# WRoverSoftware_25-26
Official Wisconsin Robotics software repository for the 2026 University Rover Challenge, containing autonomy, control, and base station code.


# How To Set Up Simulation
starting from the rot of the WiscRobo directory (where this readme is)

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
```
# Install package dependencies
source /opt/ros/humble/setup.sh
make inst_dep

# Build Project
make build

# Run Projects
source install/setup.sh
ros2 launch simulation simulation.launch.py
```
