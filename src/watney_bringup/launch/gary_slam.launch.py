import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    madgwick_param_file = os.path.join(watney_bringup_share_dir, 'params', 'madgwick.yaml')

    # Define the path to the ldlidar_slam launch file
    ldlidar_slam_launch_file = os.path.join(watney_bringup_share_dir, 'launch', 'ldlidar_slam.launch.py')

    # RVIZ2 settings
    rviz2_config = os.path.join(get_package_share_directory('watney_bringup'), 'rviz', 'ldlidar_slam.rviz')

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
            parameters=[madgwick_param_file]
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

    # RVIZ2node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]]
    )

    ld = LaunchDescription()

    ld.add_entity(frequency_arg)
    ld.add_action(high2low_node)
    ld.add_action(TimerAction(period=4.0, actions=[imu_filter_node]))  # Wait 5 seconds before launching imu_filter_node
    ld.add_action(TimerAction(period=5.0, actions=[robot_localizaton_node]))  # Wait 10 seconds before launching robot_localizaton_node
    ld.add_action(TimerAction(period=10.0, actions=[ldlidar_slam_launch]))  # Wait 15 seconds before including ldlidar_slam_launch
    ld.add_action(TimerAction(period=12.0, actions=[lidar_throttle_node]))  # Wait 20 seconds before launching lidar_throttle_node
    ld.add_action(TimerAction(period=20.0, actions=[rviz2_node]))  # Wait 20 seconds before launching rviz2_node

    return ld

if __name__ == '__main__':
    generate_launch_description()