from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with the world
        ExecuteProcess(
            cmd=['ign', 'gazebo', 'src/simulation/models/world.sdf'],
            output='screen'
        ),
        # Start ROS Bridge to transfer ros messages to gazebo messages after a delay
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_ign_bridge',
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