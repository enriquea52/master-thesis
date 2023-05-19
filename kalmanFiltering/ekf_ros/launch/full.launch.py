import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory



def generate_launch_description():

    package_name = 'ekf_ros'
    package_apriltag = 'april_detect'
    package_pointclouds = 'point_merger'
    package_kiss_icp = 'kiss_icp'

    sim_time = True


    ld = LaunchDescription()

    config_ekf = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params_ekf.yaml'
        )

    config_lkf = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params_lkf.yaml'
        )
    
    # Launch EKF SLAM ode
    ekf_node = Node(
        package = package_name,
        executable = 'ekf_slam_node',
        parameters = [config_ekf, {'use_sim_time': sim_time}],            
        emulate_tty=True,
        output='screen',
    )

    # Launch LKF Sensor Fusion Node
    lkf_node = Node(
        package = package_name,
        executable = 'fusion_node',
        parameters = [config_lkf, {'use_sim_time': sim_time}],
        emulate_tty=True,
        output='screen',
    )

    # Launch the Point Merger Node
    pm_node = Node(
        package='point_merger',
        executable='point_merger_node',
        name='pm_node',
        parameters = [{'use_sim_time': sim_time}],
        emulate_tty=True,
        output='screen',
    )

    # # AprilTag Detection Node
    aprilDetection_Launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_apriltag),'launch/start_detection_real.launch.py')
        )
    )

    # KISS - ICP Odometry Node
    icpOdometry_Launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory(package_kiss_icp),'launch/odometry.launch.py')
                ),
                launch_arguments={
                    'topic': 'joint_points',
                }.items()
            )

    # Individual Nodes
    ld.add_action(ekf_node)
    ld.add_action(lkf_node)
    ld.add_action(pm_node)

    # launch Files
    ld.add_action(aprilDetection_Launch)
    ld.add_action(icpOdometry_Launch)

    return ld