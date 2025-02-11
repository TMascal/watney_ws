import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # RVIZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'config',
        'ldlidar_slam.rviz'
    )

    # RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz2_config]
    )

    # Include common launch file
    ldlidar_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('watney_bringup'),
            '/launch/ldlidar_common.launch.py'
        ])
    )

    ld = LaunchDescription()
    ld.add_action(ldlidar_common_launch)
    ld.add_action(rviz2_node)

    return ld