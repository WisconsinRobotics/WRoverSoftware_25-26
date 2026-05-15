# Overview
This package finds the shortest navigable path between two GPS points on terrain for the URC competition. It is implemented as a ROS 2 server node — client nodes send requests specifying start and goal coordinates, and the server responds with a planned path. The planner uses LiDAR point cloud data to model the terrain and avoid obstacles.

# Installation

**1. Install Python dependencies**

From `wr_workspace/src/wr_path_planning`, run:
```bash
pip3 install -r requirements.txt
```

**2. Build the ROS 2 package**

From the `wr_workspace` folder, run:
```bash
colcon build
source install/setup.bash
```

**3. Launch the server node**

From the `launch` folder, run:
```bash
ros2 launch path_planner_launch.py lidar_path:=<path/to/lidar/file> test_mode:=<true|false>
```
- `lidar_path` — path to the LiDAR point cloud file (required)
- `test_mode` — set to `true` to enable extra debugging output (default: `false`)

# Usage
To use this server in ros2 nodes, check `wr_workspace/src/wr_path_planning/wr_path_planning/example_client_node.py` to see how to use this server

