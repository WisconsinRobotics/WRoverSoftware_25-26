from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='wr_science',
        #     executable='get_data',
        #     name='get_data'
        # ),

        Node(
            package='wr_science',
            executable='science_control',
            name='science_control'
        ),
        Node(
            package='wr_science',
            executable='send_to_can',
            name='send_to_can'
        ),
        Node(
            package='wr_controller',
            executable='xbox_controller',
            name='xbox_controller'
        ),
         #Node(
         #    package='wr_xbox_controller',
         #    executable='arm_xbox_ik',
         #    name='arm_xbox_ik'
         #),
         #Node(
         #    package='wr_xbox_controller',
         #    executable='rail_gripper_controller',
         #    name='rail_gripper_controller'
         #)#,
       # Node(
       #     package='wr_depth_camera',
       #     executable='display',
       #     name='display'
       # )
    ])
