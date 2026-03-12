from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    

    package_name = 'wr_imu' 

    # Locate the config file
    config_file = os.path.join(
        get_package_share_directory(package_name), 
        'config', 
        'ekf.yaml'
    )

    return LaunchDescription([
        
        # Connects 'base_link' (robot center) to 'imu_link' (sensor)
        # Arguments: x y z yaw pitch roll parent child
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'] #need to change to real thing
        ),

        # 2. GPS Transform (CRITICAL FOR NAVSAT TRANSFORM)
        # Adjust '0 0 0' to the actual offset of your GPS antenna
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link',
             'gps_link'] 
        ),

        # 2. Local EKF (Odom Frame)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[config_file],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),

        # 3. Global EKF (Map Frame)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[config_file],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),

        # 4. Navsat Transform (GPS -> X/Y)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('imu/data', 'imu/data'),
                ('gps/fix', '/fix'), 
                ('odometry/gps', 'odometry/gps'),
                # Navsat needs to know the robot's current heading to align the GPS
                # So it listens to the Global EKF output
                ('odometry/filtered', 'odometry/global') 
            ]
        )
    ])