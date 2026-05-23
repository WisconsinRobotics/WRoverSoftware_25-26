from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='wr_science_code',
        #     executable='get_data',
  
        # ),
        Node(
            package='wr_science_code',
            executable='science_control',
  
        ),
        Node(
            package='wr_science_code',
            executable='send_to_can',
  
        ),
        Node(
            package='wr_controller',
            executable='xbox_controller',
  
        ),
    ])