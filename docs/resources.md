
# Resources and Notes
These all serve as notes and general resources aimed at helping other developers. If you find anything useful please put it here

## Gazebo Plugins
Gazebo plugins are scarce and not very common, here are some that are found regarding plugin development.

Gazebo Plugin Development
1) [Gazebo Plugin Tutorial Website by Hrithik Verma](https://sites.google.com/view/gazebo-plugin-tutorials/3-write-a-world-plugin?authuser=0)
2) [Gazebo Plugin Tutorial Youtube Series by Hrithik Verma](https://www.youtube.com/watch?v=UNZMce9z5Fc&list=PLOQhCaBjYnseDMTpd-b52spLSq7hg0ar1&index=1)

Gazebo Documentations
1) [Defualt Gazeo Plugins Github](https://github.com/gazebosim/gz-sim/tree/main/src/systems)
2) [Default Gazebo Plugins Docs](https://gazebosim.org/api/gazebo/6/namespaceignition_1_1gazebo_1_1systems.html)
3) [Gazebo API Docs](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/index.html)
4) [Ignition Math Docs](https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/namespaceignition_1_1math.html)

Gazebo ROS intigration
1) [Example Gazebo/ROS2 project](https://github.com/gazebosim/ros_gz_project_template/tree/main)



## Inertials
This is a note about inertials. Intertials are what tell the simulation how the links interact with the world.
A great wiki artical about this is [Moment of Inertia Wiki Page](https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor), specifically the inertia tensors since that is what URDF uses.

### Approximation Methods
There are two main methods for estimating inertial values: getting it from CAD and estimating it with general shapes.
#### CAD
in Onshape, you can take the moment of inertia from the shapes themselves

### URDF Reference frames
Take the exampple:
``` xml
<joint name="rd_to_bw" type="continuous">
    <parent link="right_chasis"/>
    <child link="back_swivel"/>
    <origin xyz="${bw_pos_x} ${bw_pos_y} ${bw_pos_z}" rpy="0 0 0"/>
    <axis xyz = "0 0 1"/>
</joint>  
```
In the case of URDF, the intertials of the child will be relative to the origin axis, the origin axis being based off the parent's frame of reference
