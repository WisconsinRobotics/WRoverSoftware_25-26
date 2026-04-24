from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction,SetEnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

pkg = get_package_share_directory("simulation")
models = os.path.join(pkg, "models")

def generate_launch_description():
    return LaunchDescription([
        # add environment path for resourcs
        SetEnvironmentVariable(
            name = "GZ_SIM_RESOURCE_PATH",
            value = models + ":" + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),

        SetEnvironmentVariable(
            name = "IGN_GAZEBO_RESOURCE_PATH",
            value = models + ":" + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ),
    
        # Start Gazebo with the world
        ExecuteProcess(
            cmd=['ros2', 'launch', "ros_gz_sim", "gz_sim.launch.py", f'gz_args:=-r {os.path.join(pkg, "worlds", "test.world")}'],
            additional_env={
                'GZ_SIM_RESOURCE_PATH': models,
                'IGN_GAZEBO_RESOURCE_PATH': models,
            },
            output='screen'
        ),
        # Start ROS Bridge to transfer ros messages to gazebo messages after a delay
        TimerAction(
            period=3.0,
            actions=[
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
        # Start ROS nodes
    ])
