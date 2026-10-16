import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bcr_bot = get_package_share_directory('bcr_bot')

    # Declare simulation arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='aruco.sdf',
        description='Name of the world SDF file in bcr_bot/worlds'
    )

    # 1. Include Gazebo simulation launch from bcr_bot
    bcr_bot_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bcr_bot, 'launch', 'gz.launch.py')
        ),
        launch_arguments={
            'camera_enabled': 'True',
            'stereo_camera_enabled': 'True',
            'two_d_lidar_enabled': 'True',
            'position_x': '0.0',
            'position_y': '0.0',
            'orientation_yaw': '0.0',
            'odometry_source': 'world',
            'world_file': LaunchConfiguration('world_file'),
        }.items()
    )

    # Common parameters for all custom Python nodes running in simulation
    sim_time_param = {'use_sim_time': True}

    # 2. ArUco Detector Node
    aruco_detector_node = Node(
        package='rover_aruco',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[sim_time_param]
    )

    # 3. Goal Generator Node
    goal_generator_node = Node(
        package='rover_aruco',
        executable='goal_generator',
        name='goal_generator',
        output='screen',
        parameters=[sim_time_param]
    )

    # 4. Marker Tracker Node
    marker_tracker_node = Node(
        package='rover_aruco',
        executable='marker_tracker',
        name='marker_tracker',
        output='screen',
        parameters=[sim_time_param]
    )

    # 5. Obstacle Avoidance / Movement Node (obs_avoid)
    obs_avoid_node = Node(
        package='rover_aruco',
        executable='obs_avoid',
        name='obs_avoid',
        output='screen',
        parameters=[
            sim_time_param,
            {
                'cmd_vel_topic': '/bcr_bot/cmd_vel',
                'odom_topic': '/bcr_bot/odom',
                'scan_topic': '/bcr_bot/scan'
            }
        ]
    )

    return LaunchDescription([
        world_file_arg,
        bcr_bot_gz_launch,
        aruco_detector_node,
        goal_generator_node,
        marker_tracker_node,
        obs_avoid_node
    ])