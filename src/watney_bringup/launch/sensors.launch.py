#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    
    # Launch high_to_low_node
    high2low_node = Node(
            package='high_to_low',
            executable='high_to_low_node',
            name='high_to_low_node',
            output='screen',
            # parameters=[{'feedback_frequency': LaunchConfiguration('frequency')}]
        )
    
    # Launch LDLidar and supporting nodes
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('watney_bringup'),
                'params', 'lifecycle_mgr_slam.yaml'
            )
        ]
    )

    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('watney_bringup'),
                'launch',
                'ldlidar_bringup.launch.py'
            )
        ),
        launch_arguments={'node_name': 'ldlidar_node'}.items()
    )


    ld.add_action(lc_mgr_node)

    ld.add_action(ldlidar_launch)

    ld.add_action(high2low_node)

    return ld