from launch import LaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='wr_arm',
            executable='send_to_can',
            name='send_to_can'
        ),
        Node(
            package='wr_arm',
            executable='autonomous_control',
            name='autonomous_control'
        ),
            #Control with xbox
        Node(
           package='wr_controller',
           executable='xbox_controller_bluetooth',
           name='xbox_controller_bluetooth'
        ),
        # Node(
        #     package='wr_can_comms',
        #     executable='can_comms',
        #     name='can_comms'
        # ),

        # Node(
        #     package='wr_can_comms',
        #     executable='receive_can',
        #     name='receive_can'
        # ),


    ])
    
