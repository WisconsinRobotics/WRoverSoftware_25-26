from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction,SetEnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

pkg = get_package_share_directory("simulation")

def generate_launch_description():
    return LaunchDescription([
        # add environment path for resourcs
        SetEnvironmentVariable(
            name = "GZ_SIM_RESOURCE_PATH",
            value = os.path.join(pkg, "models")
        ),

        # Start Gazebo with the world
        ExecuteProcess(
            cmd=['ros2', 'launch', "ros_gz_sim", "gz_sim.launch.py", f'gz_args:=-r {os.path.join(pkg, "models", "world.sdf")}'],
            output='screen'
        ),
        # Start ROS Bridge to transfer ros messages to gazebo messages after a delay
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        # ROS TOPIC | ROS MSG TYPE | IGN MSG TYPE
                        # These need to match in order for things to start correctly
                        '/cmd_wheel_velocity@std_msgs/msg/Float64@ignition.msgs.Double'
                    ],
                    output='screen'
                )
            ]
        )
        # Start ROS nodes
    ])
