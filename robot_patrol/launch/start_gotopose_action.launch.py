import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    proj_dir = get_package_share_directory('robot_patrol')
    rviz_config_dir = os.path.join(proj_dir, 'config', 'Patrol.rviz')

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='robot_patrol_action_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        ),
    ])