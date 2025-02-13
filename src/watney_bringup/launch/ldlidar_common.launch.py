import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr_slam.yaml'
    )

    # SLAM Toolbox configuration for LDLidar
    slam_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'slam_toolbox.yaml'
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            lc_mgr_config_path
        ]
    )

    # SLAM Toolbox node in async mode
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path
        ],
        remappings=[
            ('/scan', '/ldlidar_node/scan')
        ]
    )

    # Fake odom publisher
    # fake_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'ldlidar_base']
    # )

    ld = LaunchDescription()
    ld.add_action(lc_mgr_node)
    ld.add_action(slam_toolbox_node)
    # ld.add_action(fake_odom)

    return ld