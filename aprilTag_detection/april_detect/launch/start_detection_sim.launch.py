import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'april_detect'
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'sim_params.yaml'
        )
    
    node = Node(
        package = package_name,
        executable = 'april_detect',
        parameters = [config],
        output='screen',
    )
    ld.add_action(node)
    return ld