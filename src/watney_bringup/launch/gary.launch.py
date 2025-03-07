import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='25.0',
        description='Flat sensor rate'
    )

    # Get the package share directory
    watney_bringup_share_dir = get_package_share_directory('watney_bringup')

    # Define the path to the ekf.yaml file
    ekf_param_file = os.path.join(watney_bringup_share_dir, 'params', 'ekf.yaml')

    # Define the path to the ldlidar_slam launch file
    ldlidar_slam_launch_file = os.path.join(watney_bringup_share_dir, 'launch', 'ldlidar_slam.launch.py')

    # Launch high_to_low_node
    high2low_node = Node(
            package='high_to_low',
            executable='high_to_low_node',
            name='high_to_low_node',
            output='screen',
            parameters=[{'feedback_frequency': LaunchConfiguration('frequency')}]
        )
        
    # Launch imu_filter_madgwick node
    imu_filter_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[{'use_mag': False}]
        )
        
    # Launch robot_localization node
    robot_localizaton_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param_file, {'frequency': LaunchConfiguration('frequency')}]
        )

    # Include ldlidar_slam launch file
    ldlidar_slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_slam_launch_file)
        )

    # Throttle LiDAR topic
    lidar_throttle_node = Node(
            package='topic_tools',
            executable='throttle',
            name='lidar_throttle',
            arguments=['messages', '/ldlidar_node/scan', LaunchConfiguration('frequency')],  # Adjust '5.0' to your desired Hz
            output='screen'
        )

    ld = LaunchDescription()

    ld.add_entity(frequency_arg)
    ld.add_action(high2low_node)
    ld.add_action(imu_filter_node)
    ld.add_action(robot_localizaton_node)
    ld.add_action(ldlidar_slam_launch)
    ld.add_action(lidar_throttle_node)

    return ld

if __name__ == '__main__':
    generate_launch_description()