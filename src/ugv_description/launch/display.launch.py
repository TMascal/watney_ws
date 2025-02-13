import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_model_path = os.path.join(
        get_package_share_directory('ugv_description'),
        'urdf', 
        'ugv_rover.urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='ugv',
        parameters=[{'robot_description': open(urdf_model_path).read()}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='ugv',
        output='screen',
        parameters=[{'robot_description': open(urdf_model_path).read()}]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('ugv_description'), 'rviz', 'view_description.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node
    ])

if __name__ == '__main__':
    generate_launch_description()