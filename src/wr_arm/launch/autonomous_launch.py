from launch import LaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='wr_arm',
            executable='send_to_can_position',
            name='send_to_can_position'
        ),
        Node(
            package='wr_arm',
            executable='autonomous_control',
            name='autonomous_control'
        ),
        

        
        #Control with xbox
        # Node(
        #    package='wr_controller',
        #    executable='xbox_controller',
        #    name='xbox_controller'
        # )

    ])
