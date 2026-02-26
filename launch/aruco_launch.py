from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_part',
            executable='camera_info',
            name='camera_info'
        ),
        Node(
            package='aruco_part',
            executable='finding_tag',
            name='finding_tag'
        ),
        Node(
            package='aruco_part',
            executable='driving_logic',
            name='driving_logic'
        ),
        # Node(
        #     package='wr_xbox_controller',
        #     executable='arm_xbox_ik',
        #     name='arm_xbox_ik'
        # ),
        # Node(
        #     package='wr_xbox_controller',
        #     executable='rail_gripper_controller',
        #     name='rail_gripper_controller'
        # )#,
       # Node(
       #     package='wr_depth_camera',
       #     executable='display',
       #     name='display'
       # )
    ])
