


### Installation

Install the `ublox` package with:
```bash
sudo apt update
sudo apt install ros-[distro]-ublox-gps
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
