#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    watney_bringup_share_dir = get_package_share_directory('watney_bringup')
    
    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(os.path.join(get_package_share_directory('watney_bringup'), 'urdf', 'gary.urdf.xml')).read()}]
    )

    # Launch imu_filter_madgwick node
    imu_filter_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[os.path.join(watney_bringup_share_dir, 'params', 'madgwick.yaml')]
        )

    # Launch robot_localization node
    robot_localizaton_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(watney_bringup_share_dir, 'params', 'ekf.yaml'),
                # {'frequency': LaunchConfiguration('frequency')}
            ],
            remappings=[('/odometry/filtered','/odom')]
        )

    laser_filter = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[os.path.join(get_package_share_directory('watney_bringup'), 'params', 'box.yaml')],
            remappings=[('/scan','/ldlidar_node/scan'),('/scan_filtered','/scan')],
        )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='sync_slam_toolbox_node',
        output='screen',
        parameters=[os.path.join(watney_bringup_share_dir, 'params', 'slam_toolbox.yaml')]
    )

    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'params_file': os.path.join(watney_bringup_share_dir, 'params', 'nav2_gary_params.yaml')}.items()
        )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(watney_bringup_share_dir, 'rviz', 'nav2_view.rviz')]
    )

    ld.add_action(rsp_node)
    ld.add_action(imu_filter_node)
    ld.add_action(robot_localizaton_node)
    ld.add_action(laser_filter)
    ld.add_action(slam_node)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_node)

    return ld