# Note
This is a note about inertials. The inertial values found in this simulation are quite honestly terrible.

They need a lot of work to be refactored into something that is actually usable
[Moment of Inertia Wiki](https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor)
we would essentially need to refactor the inertials of each of the components of the rover to be more realistic

a great approximation of this is using the shapes that are the hitboxes. Someone will need to do this!

## URDF Reference frames
Take the exampple
``` xml
<joint name="rd_to_bw" type="continuous">
            <parent link="right_chasis"/>
            <child link="back_swivel"/>
            <origin xyz="${bw_pos_x} ${bw_pos_y} ${bw_pos_z}" rpy="0 0 0"/>
            <axis xyz = "0 0 1"/>
        </joint>  
```
In the case of URDF, the intertials of the child will be relative to the inertials of the parent
