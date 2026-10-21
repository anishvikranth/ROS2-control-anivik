import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# The terrain ArUco mission (see rover_description/launch/terrain_aruco.launch.py) driven by
# jade -- the rover with the arm -- instead of rover_sim: real mass and centre of gravity, and
# the ZED where it really sits, on the arm stand. The arm is held in the SRDF's "stow" pose.
# Robot model: urdf/jade_terrain.xacro (jade.xacro + sim settings and nav sensors).

STOW = {  # rover_arm_moveit_config/config/rover_arm.srdf, group_state "stow"
    'base_link_joint': 0.0,
    'shoulder_link_joint': -1.5,
    'elbow_link_joint': 2.3,
    'wrist_link_joint': -0.8,
    'palm_link_joint': 0.0,
}


def generate_launch_description():
    pkg_jade = get_package_share_directory('jade_description')
    pkg_rover = get_package_share_directory('rover_description')
    pkg_arm = get_package_share_directory('rover_arm_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rviz_config_file = os.path.join(pkg_jade, 'config', 'rover.rviz')
    world_path = os.path.join(pkg_rover, 'worlds', 'terrain_world.sdf')
    markers_config = os.path.join(pkg_rover, 'config', 'terrain_markers.yaml')
    models_path = os.path.join(pkg_rover, 'models')

    robot_desc = xacro.process_file(os.path.join(pkg_jade, 'urdf', 'jade_terrain.xacro')).toxml()

    # Meshes (package:// URIs of all three packages), terrain + marker models, worlds
    resource_paths = [os.path.dirname(pkg_jade), os.path.dirname(pkg_rover), os.path.dirname(pkg_arm),
                      models_path, os.path.join(pkg_rover, 'worlds')]
    for i in range(18):
        resource_paths += [f'{models_path}/marker_{i}/materials/textures', f'{models_path}/marker{i}/materials/textures']
    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.pathsep.join(resource_paths))

    sim_time_param = {'use_sim_time': True}

    # 1. Gazebo with the terrain world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 2. Robot State and TFs
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. ZED optical frame (the RGB camera is stamped in it)
    zed_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_optical_frame_publisher',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.57079632679', '--pitch', '0', '--yaw', '-1.57079632679',
            '--frame-id', 'zed_camera_link',
            '--child-frame-id', 'zed_camera_optical_frame'
        ],
        parameters=[sim_time_param]
    )

    # 4. Spawn jade at the start pad
    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '0.5'],
        output='screen'
    )

    # 5. ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/zed/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/zed/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/zed/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # 6. Arm controllers, then move the arm to "stow" and leave it there
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    arm_controller = Node(
        package='controller_manager', executable='spawner', arguments=['arm_controller'])
    stow_arm = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/arm_controller/joint_trajectory',
             'trajectory_msgs/msg/JointTrajectory',
             '{joint_names: [' + ', '.join(STOW) + '], points: [{positions: ['
             + ', '.join(str(v) for v in STOW.values()) + '], time_from_start: {sec: 3}}]}'],
        output='screen'
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_rover, on_exit=[joint_state_broadcaster, arm_controller]))
    stow_after_controller = RegisterEventHandler(
        OnProcessExit(target_action=arm_controller, on_exit=[stow_arm]))

    # 7. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[sim_time_param]
    )

    # 8. Navigation: ArUco detection, marker tracking, goals, driving (as in terrain_aruco.launch.py)
    aruco_detector_node = Node(
        package='rover_aruco',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[sim_time_param, {
            'image_topic': '/zed/image_raw',
            'camera_info_topic': '/zed/camera_info',
            'output_topic': '/aruco/markers',
            'marker_size_meters': 0.175,
            'enable_debug_window': True
        }]
    )
    goal_generator_node = Node(
        package='rover_aruco',
        executable='goal_generator',
        name='goal_generator',
        output='screen',
        parameters=[sim_time_param, markers_config]
    )
    marker_tracker_node = Node(
        package='rover_aruco',
        executable='marker_tracker',
        name='marker_tracker',
        output='screen',
        parameters=[sim_time_param]
    )
    obs_avoid_node = Node(
        package='rover_aruco',
        executable='obs_avoid',
        name='obs_avoid',
        output='screen',
        parameters=[sim_time_param, {
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'max_linear_speed': 0.8,
            'max_angular_speed': 1.0
        }]
    )

    # 9. Terrain mapper: depth cloud -> height map -> /traversability/costmap
    terrain_mapper_node = Node(
        package='rover_traversability',
        executable='terrain_mapper',
        name='terrain_mapper',
        output='screen',
        parameters=[sim_time_param]
    )

    # Navigation waits for the arm: until arm_controller is up the arm hangs limp from its spawn
    # pose (straight out in front), and in testing the forearm propped the rover so it couldn't
    # drive forward. Start once the stow command is sent, plus time for the 3 s move.
    start_navigation = RegisterEventHandler(OnProcessExit(
        target_action=stow_arm,
        on_exit=[TimerAction(period=4.0, actions=[
            aruco_detector_node, goal_generator_node, marker_tracker_node, obs_avoid_node])]))

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        zed_optical_tf,
        spawn_rover,
        ros_gz_bridge,
        start_controllers,
        stow_after_controller,
        start_navigation,
        rviz,
        terrain_mapper_node,
    ])
