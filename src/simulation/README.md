# simulation docs

# TODO List
1) Remake the swivel links to include wheels and wheel joints [IN PROGRESS]
2) Fix the wheel Inertial values. It only has the inertials for a singular wheel! Needs to account for the both wheels and the rod
2) add ros2 script that controls the direction of the wheels and the speed
3) implement arm...

# Install
Gazebo Robot Fortress:
[https://gazebosim.org/docs/fortress/install_ubuntu/](Gazebo Fortress Install)
ros_gz_bridge:
```sudo apt install ros-humble-ros-gz-bridge```

# Running simulation
First source ros2 humble:
```source /opt/ros/humble/setup.sh```

## Gazebo
Run Gazebo via the command:
```ign gazebo world.sdf```
running these commands with start the Gazebo simulation in a world with the rover

## ros_gz_bridge
this is the link that gets ROS2 packets to communicated with gazebo packages. For each ROS2 node you have that wants to communicate with gazebo, you need to run this command:

[https://docs.ros.org/en/humble/p/ros_gz_bridge/](rosr_gz_bridge docs)
[https://index.ros.org/p/ros_gz_bridge/](ros index info)


```ros2 run ros_gz_bridge parameter_bridge /{INSERT_TOPIC}@{ROS2_MESSAGE_TYPE}@{GAZEBO_MESSAGE_TYPE}```
example:
```ros2 run ros_gz_bridge parameter_bridge /joint_pos@std_msgs/msg/Float64@ignition.msgs.Double```

# Simulation Framework
## Rover model
as of right now, making the model includes going into the team's Onshape and manually exporting models.

The process goes like this:
1) Choose a section of the rover to export (this should be a singular link between any joints)
    - Export this as a low poly model for visuals
2) for each link, define a loose hitbox using the gazebo builting primitive geometries. Position and scale these primitives base off of the position and length values shown in OnShape.
    - These hitbox's dont need to be super precise
3) Manually enter in the mass and inertial values from onshape into the SDF file
4) Define all links, joints, and scripts in a singular SDF part isolated to that link or group of links
