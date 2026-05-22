# Overview

This package implements state machine for autonomous rover navigation during URC competition. 
It guides navigation algorithms to reach targets and search for objects on the path. It allows for operator to directly
control state machine through terminal line commands. 

# Installation

**1. Build the ROS 2 package**
From the `wr_workspace` folder, run:
```bash
colcon build
source install/setup.bash
```

**2. Run state machine**

To launch rover state machine, run:
```bash
ros2 run state_machine state_machine
```
If you want to specify custom points config file, run
```
ros2 run state_machine state_machine --ros-args -p points_file_path:=<points_file_path>
```
To send commands to the state machine from the terminal, run
```
ros2 run state_machine state_machine_controller
```

