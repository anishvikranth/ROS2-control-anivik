import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():  
    pkg_rover_description = get_package_share_directory('rover_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    rviz_config_file = os.path.join(pkg_rover_description, 'config', 'rover.rviz')
    urdf_path = os.path.join(pkg_rover_description, 'urdf', 'rover_sim.urdf')
    world_path = os.path.join(pkg_rover_description, 'worlds', 'aruco_world.sdf')
    models_path = os.path.join(pkg_rover_description, 'models')
    worlds_dir = os.path.join(pkg_rover_description, 'worlds')

    # Read the URDF contents into robot_description
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Points Gazebo to install share directory, models, and worlds so URIs resolve properly
    install_share_dir = os.path.dirname(pkg_rover_description)
    gz_resource_path = f"{install_share_dir}:{models_path}:{worlds_dir}"
    for i in range(18):
        gz_resource_path += f":{models_path}/marker_{i}/materials/textures:{models_path}/marker{i}/materials/textures"

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # 1. Launch Gazebo Harmonic with the custom ArUco world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 2. Publish Robot State and TFs
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 3. Static Transform Publisher for ZED Optical Frame
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
        parameters=[{'use_sim_time': True}]
    )

    # 4. Spawn the robot model into Gazebo Harmonic from /robot_description
    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover',
            '-z', '0.5'
        ],
        output='screen'
    )

    # 5. Expanded ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Simulation & Core Robotics
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # ZED RGB Camera Feed & Calibration Info
            '/zed/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # Direct bridge mapping for Gazebo sensor topics
            '/zed_rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed_rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # ZED Depth Feed
            '/zed/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed_depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            
            # ZED IMU Sensor Data
            '/zed/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            ('/zed_rgb_camera/image', '/zed/image_raw'),
            ('/zed_rgb_camera/camera_info', '/zed/camera_info'),
            ('/zed_depth_camera/image', '/zed/depth/image_raw'),
        ],
        output='screen'
    )

    # 6. RViz2 for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    sim_time_param = {'use_sim_time': True}

    # 7. ArUco Detector Node
    aruco_detector_node = Node(
        package='rover_aruco',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[
            sim_time_param,
            {
                'image_topic': '/zed/image_raw',
                'camera_info_topic': '/zed/camera_info',
                'output_topic': '/aruco/markers',
                'marker_size_meters': 0.175,
                'enable_debug_window': True
            }
        ]
    )

    # 8. Goal Generator Node
    goal_generator_node = Node(
        package='rover_aruco',
        executable='goal_generator',
        name='goal_generator',
        output='screen',
        parameters=[sim_time_param]
    )

    # 9. Marker Tracker Node
    marker_tracker_node = Node(
        package='rover_aruco',
        executable='marker_tracker',
        name='marker_tracker',
        output='screen',
        parameters=[sim_time_param]
    )

    # 10. Obstacle Avoidance / Rover Movement Node
    obs_avoid_node = Node(
        package='rover_aruco',
        executable='obs_avoid',
        name='obs_avoid',
        output='screen',
        parameters=[
            sim_time_param,
            {
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/odom',
                'scan_topic': '/scan'
            }
        ]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        zed_optical_tf,
        spawn_rover,
        ros_gz_bridge,
        rviz,
        aruco_detector_node,
        goal_generator_node,
        marker_tracker_node,
        obs_avoid_node
    ])