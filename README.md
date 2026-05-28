


### Installation

Install the `ublox` package with:
```bash
sudo apt update
sudo apt install ros-[distro]-ublox-dgnss
```

### Running the Node

After setting up RTK on the base with ublox, use the following command to make each base start receiving RTK and publishing the corrected topics:
```bash
ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -r __ns:=/rover1 -p device_serial_string:="'1'"
```
* Replace `/rover1` with your desired output topic namespace.
* Replace "'1'" with your device's specific id

### Output Topics

For this particular example, the output topic will be `/rover1/ubx_nav_pvt`. 

This topic is of type `UBXNavPVT`, which can be imported from the installed `ublox-gps` package with:
```python
from ublox_ubx_msgs.msg import UBXNavPVT
```

### Running base station launch scripts

To configure rtk on the base station:
```bash
./run_rtk_base_station.sh POLE_LAT POLE_LON POLE_ALT DISTANCE BEARING
```
Example:
```bash
./run_rtk_base_station.sh 38.221704 -110.421456 1309.99 0 0
```

To run the path planning, spiral planning, and base station controller:
```bash
./run_state_machine_base_station.sh LIDAR_PATH
```
Example:
```bash
./run_state_machine_base_station.sh ~/motel.laz
```
