run:
	LIBGL_ALWAYS_SOFTWARE=1 ign gazebo world.sdf &
	ros2 run ros_gz_bridge parameter_bridge /joint_pos@std_msgs/msg/Float64@ignition.msgs.Double &
#	ros2 topic pub /joint_vel std_msgs/msg/Float64 "data: 1.0"

run_ros:
	#ros2 topic echo /keyboard/keypress
