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

    # Read the URDF contents into robot_description
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Points Gazebo to the install share directory so meshes resolve properly
    install_share_dir = os.path.dirname(pkg_rover_description)

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[install_share_dir]
    )

    # 1. Launch Gazebo Harmonic with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
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

    # 3. Spawn the robot model into Gazebo Harmonic from /robot_description
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

    # 4. Expanded ROS-Gazebo Bridge (Clock, Control, Odometry, TF, Joint States, ZED Camera & IMU)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Simulation & Core Robotics
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # ZED RGB Camera Feed & Calibration Info
            '/zed/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # ZED Depth Feed
            '/zed/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            
            # ZED IMU Sensor Data
            '/zed/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # 5. RViz2 for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_rover,
        ros_gz_bridge,
        rviz
    ])