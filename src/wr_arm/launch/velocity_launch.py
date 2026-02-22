from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='wr_arm',
        #     executable='velocity_control',
        #     name='velocity_control'
        # ),
        
        # Node(
        #     package='wr_arm',
        #     executable='send_to_can',
        #     name='send_to_can'
        # ),
        #Node(
        #    package='arm_test_python',
        #    executable='rail_subscriber',
        #    name='rail_subscriber'
        #),
        Node(
            package='wr_can_comms',
            executable='can_comms',
            name='can_comms'
        ),
    #     #Control with keyboard
    #     # Node(
    #     #    package='wr_controller',
    #    #     executable='keyboard_controller',
    #     #    name='keyboard_controller'
    #     #)
        
    #     #Control with xbox
    #     Node(
    #        package='wr_controller',
    #        executable='xbox_controller',
    #        name='xbox_controller'
    #     )

    ])
