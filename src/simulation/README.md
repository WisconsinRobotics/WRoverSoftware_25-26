# simulation docs

# Install
Gazebo Robot Fortress:
[https://gazebosim.org/docs/fortress/install_ubuntu/](Gazebo Fortress Install)

# Running simulation
First source ros2 humble:
```source /opt/ros/humble/setup.sh```
Run Gazebo via the command:
```ign gazebo world.sdf```
running these commands with start the Gazebo simulation in a world with the rover


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
