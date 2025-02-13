import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    node_name = LaunchConfiguration('node_name')

    # Lidar node configuration file
    lidar_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'ldlidar.yaml'
    )

    # Launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='ldlidar_node',
        description='Name of the node'
    )

    # LDLidar lifecycle node
    ldlidar_node = Node(
        package='ldlidar_node',
        executable='ldlidar_node',
        name=node_name,
        namespace='',
        output='screen',
        parameters=[
            lidar_config_path
        ]
    )

    urdf = os.path.join(
        get_package_share_directory('ugv_description'),
        'urdf',
        "ugv_rover.urdf"
    )

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ldlidar_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )

    # Include common launch file
    ldlidar_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('watney_bringup'),
            '/launch/ldlidar_common.launch.py'
        ])
    )

    ld = LaunchDescription()
    ld.add_action(declare_node_name_cmd)
    ld.add_action(rsp_node)
    ld.add_action(ldlidar_node)
    ld.add_action(ldlidar_common_launch)

    return ld