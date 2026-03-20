# WRoverSoftware_25-26
Official Wisconsin Robotics software repository for the 2026 University Rover Challenge, containing autonomy, control, and base station code.


# How To Set Up Simulation

## 1. Install Dependencies
start from the root of the wisconsin robotics directory

Makefile shortcut:
```
make inst_dep
```
Raw Commands:
```
sudo apt-get update
rosdep install --from-path src
```

## 2. build project
Makefile shortcut:
```
make build
```
Raw Commands:
```
colcon build
```

## 3. Run Simulation
Makefile shortcut:
```
source install/setup.sh
make run
```
Raw Commands:
```
source install/setup.sh
ros2 launch simulation simulation.launch.py
``` 
