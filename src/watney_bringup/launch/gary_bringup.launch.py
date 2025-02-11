from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='high_to_low',
            executable='serial_json',
            name='h2l_node',
            output='screen',
            parameters=[{'port': '/dev/serial0'},{'baudrate': 115200}]
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            remappings=[('/imu/data_raw','/h2l_node/imu/raw'),('/imu/mag','/h2l_node/imu/mag')]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[FindPackageShare('watney_bringup').find('watney_bringup') + '/config/ekf.yaml']
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(FindPackageShare('ldlidar_node').find('ldlidar_node') + '/launch/ldlidar_slam.launch.py'),
            # launch_arguments={'arg_name': 'arg_value'}.items()  # Pass arguments to the included launch file
        ),
        
    ])
