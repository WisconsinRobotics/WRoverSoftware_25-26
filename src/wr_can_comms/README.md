Currently, publishing a message to "can_msg" topic will print an encoded CAN-VESC message, although uncommenting some code will enable the node to also send the message to can0. 

For example:

1. Set up ROS2
```bash
colcon build
source install/setup.bash
```

2. Start subscriber
```bash
ros2 run wr_can_comms can_comms
```

3. In another terminal, publish an example
```bash
ros2 topic pub can_msg std_msgs/String "data: 23 CAN_PACKET_SET_CURRENT 51 int"
```

This should print:
```
publishing #1: std_msgs.msg.String(data='23 CAN_PACKET_SET_CURRENT 51 int')

[INFO] [1736897861.362478158] [can_subscriber]: I heard: "23 CAN_PACKET_SET_CURRENT 51 int"
Timestamp:        0.000000    ID: 00000117    X Rx                DL:  4    00 00 c7 38
```
