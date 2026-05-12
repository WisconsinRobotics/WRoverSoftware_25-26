from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction,SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import xacro
import os
from pathlib import Path


def generate_launch_description():
   
    # Gazebo Resource folders look one directory above the package folder
    pkg = get_package_share_directory("simulation")
    
    # Process xacro and write urdf to install dir
    xacro_file = os.path.join(pkg, "urdf", "rover.urdf.xacro")
    doc = xacro.process_file(xacro_file).toxml()
    
    
    return LaunchDescription([
        # Start Gazebo with the world
        ExecuteProcess(
            cmd=['ros2', 'launch', "ros_gz_sim", "gz_sim.launch.py", f'gz_args:=-r {os.path.join(pkg, "worlds", "test.world")}'],
            additional_env={
                'GZ_SIM_RESOURCE_PATH': str(Path(pkg).parent),
            },
            output='screen'
        ),
        
        # Spawn Robot and start ROS Bridge to transfer ros messages to gazebo messages after a delay
        TimerAction(
            period=3.0,
            actions=[
                # Start the robot state publisher to publish the robot's TF frames
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[{'robot_description': doc}],
                    output='screen'
                ),
                Node(
                    package='joint_state_publisher_gui',
                    executable='joint_state_publisher_gui',
                    name='joint_state_publisher_gui',
                ),
                # Start RViz to visualize the robot
                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', os.path.join(pkg, "config", "rviz_config.rviz")]
                ),
                # Spawn the robot in Gazebo
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-string', doc,
                        '-name', 'rover',
                        '-x', '0', '-y', '0', '-z', '0.25'
                    ],
                    output='screen'
                ),
                # Start the ROS-Gazebo bridge
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    parameters = [{
                        "config_file" : os.path.join(pkg, "config", "ROS_gazebo_bridge_config.yaml")
                    }],
                    output='screen'
                )
            ]
        )
    ])
