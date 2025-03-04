import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    high_to_low_share_dir = get_package_share_directory('high_to_low')
    watney_bringup_share_dir = get_package_share_directory('watney_bringup')

    # Define the path to the ekf.yaml file
    ekf_param_file = os.path.join(watney_bringup_share_dir, 'params', 'ekf.yaml')

    return LaunchDescription([
        # Launch high_to_low_node
        Node(
            package='high_to_low',
            executable='high_to_low_node',
            name='high_to_low_node',
            output='screen'
        ),
        
        # Launch imu_filter_madgwick node
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[{'use_mag': False}]
        ),
        
        # Launch robot_localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param_file]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()