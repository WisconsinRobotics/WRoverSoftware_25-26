import subprocess
import sys

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the LiDAR data file in the config file
    lidar_path = LaunchConfiguration("lidar_path")
    epsg = LaunchConfiguration("epsg")
    test_mode = LaunchConfiguration("test_mode")
    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_path"
        ),
        DeclareLaunchArgument(
            "epsg",
            default_value="0",
            description="EPSG code, 0 means unspecified"
        ),
        DeclareLaunchArgument(
            "test_mode",
            default_value = "false"
        ),
        Node(
            package='wr_path_planning',
            executable='path_planner',
            name='path_planner_node',
            parameters=[
                {
                    "lidar_file": lidar_path,
                    "epsg": epsg,
                    "test_mode": test_mode
                }
            ]
        )
    ])