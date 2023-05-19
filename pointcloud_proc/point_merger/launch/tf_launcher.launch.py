from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_scanner_static_tf',
            # arguments = ['0.54', '1.40', '0', '-0.7767', '0.0', '3.1416', 'base_link', 'safety_lidar_front_link']
            arguments = ['1.40', '-0.54', '0', '-0.7767', '0.0', '3.1416', 'base_link', 'safety_lidar_front_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_scanner_static_tf',
            # arguments = ['-0.54', '0.8', '0', '1.7453', '0.0', '3.1416', 'base_link', 'safety_lidar_back_link']
            arguments = ['0.8', '0.54', '0', '1.8326', '0.0', '3.1416', 'base_link', 'safety_lidar_back_link']
        ),
    ])