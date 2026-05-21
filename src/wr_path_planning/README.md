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
ros2 launch path_planner_launch.py lidar_path:=<path/to/lidar/file> epsg:=<epsg> test_mode:=<true|false>
```
- `lidar_path` — path to the LiDAR point cloud file (required)
- `epsg`- user defined epsg of the lidar file (default: `0`)
- `test_mode` — set to `true` to enable extra debugging output and path visualization (default: `false`)

# Usage
To use this server in ros2 nodes, refer to `wr_workspace/src/wr_path_planning/wr_path_planning/example_client_node.py` for an example.
If you have laz files from USGS website to merge, you can use script at `utilities` folder by running the following command
```bash
python3 laz_merge.py <laz1> <laz2> ... <lazN> <output>
```

# Contributors

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/wizkio">
        <img src="https://github.com/wizkio.png" width="100px;" /><br />
        <b>Sungkar Bolat</b>
      </a>
      <br/>ROS2 Interface Implementation
    </td>
    <td align="center">
      <a href="https://github.com/Arshjeet13">
        <img src="https://github.com/Arshjeet13.png" width="100px;" /><br />
        <b>Arshjeet</b>
      </a>
      <br/>Path Planning Algorithm Implementation
    </td>
    <td align="center">
      <a href="https://github.com/shreyas22112006">
        <img src="https://github.com/shreyas22112006.png" width="100px;" /><br />
        <b>Shreyas Rao Karbar</b>
      </a>
      <br/>Path Planning Algorithm Implementation
    </td>
  </tr>
</table>