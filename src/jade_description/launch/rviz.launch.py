from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'jade_description'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'jade.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rover.rviz')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
        
    robot_description = {
        'robot_description': robot_description_content
    }
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # Loads the specified rviz configuration
        ),
    ])