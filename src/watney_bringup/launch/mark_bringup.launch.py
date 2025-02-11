from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(FindPackageShare('watney_bringup').find('watney_bringup') + '/launch/ldlidar_control_station.launch.py'),
            # launch_arguments={
            #     'ldlidar_link': 'ldlidar_base'
            # }.items()
        ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(FindPackageShare('nav2_bringup').find('nav2_bringup') + '/launch/navigation_launch.py'),
                launch_arguments={
                    'use_sim_time': 'false'
                }.items()
            ),
    ])