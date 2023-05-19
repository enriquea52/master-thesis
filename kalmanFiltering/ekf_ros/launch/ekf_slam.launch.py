import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    package_name = 'ekf_ros'
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params_ekf.yaml'
        )
        
    node = Node(
        package = package_name,
        executable = 'ekf_slam_node',
        parameters = [config],
        emulate_tty=True,
        output='screen',
    )
    ld.add_action(node)
    return ld