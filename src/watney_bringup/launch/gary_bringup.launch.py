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
            parameters=[{'port': '/dev/serial0','baudrate': 115200}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(FindPackageShare('ldlidar_node').find('ldlidar_node') + '/launch/ldlidar_with_mgr.launch.py'),
            # launch_arguments={'arg_name': 'arg_value'}.items()  # Pass arguments to the included launch file
        ),
    ])