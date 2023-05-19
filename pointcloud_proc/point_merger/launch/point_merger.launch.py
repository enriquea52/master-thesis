from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    extra_angle_offset = 0.0986

    front_angle = -0.7767 - extra_angle_offset

    back_angle = 1.8326 - extra_angle_offset

    package_kiss_icp = 'kiss_icp'

    return LaunchDescription([

        # Launching Individual Nodes

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='front_scanner_static_tf',
        #     # arguments = ['1.40', '-0.54', '0', '-0.7767', '0.0', '3.1416', 'base_link', 'safety_lidar_front_link'] # agv1
        #     arguments = ['1.40', '-0.54', '0', str(front_angle), '0.0', '3.1416', 'base_link', 'safety_lidar_front_link'] # agv1

        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='back_scanner_static_tf',
        #     # arguments = ['0.8', '0.54', '0', '1.8326', '0.0', '3.1416', 'base_link', 'safety_lidar_back_link'] # agv1
        #     arguments = ['0.6', '0.42', '0', str(back_angle), '0.0', '3.1416', 'base_link', 'safety_lidar_back_link'] # agv1

        # ),

        Node(
            package='point_merger',
            executable='point_merger_node',
            name='pm_node',
            parameters = [{'use_sim_time': True}],
            emulate_tty=True,
            output='screen',
        ),

        # Launching KISS ICP

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory(package_kiss_icp),'launch/odometry.launch.py')
                ),
                launch_arguments={
                    'topic': 'joint_points',
                }.items()
            ),

    ])