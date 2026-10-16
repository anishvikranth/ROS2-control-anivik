from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


import os

def generate_launch_description():
    package_name = 'rover_arm_description'
    pkg_share = get_package_share_directory(package_name)

    declare_arm_controller_cmd = DeclareLaunchArgument(
        'arm_controller',
        default_value='arm_controller',
        description='Name of the ros2_control controller to spawn for the arm joints.'
    )

    gz_resource_paths = [
        os.path.dirname(pkg_share),           # for package://rover_arm_description/...
        os.path.join(pkg_share, 'models'),    # for model://test_cube
    ]
    for env_var in ['GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH']:
        existing = os.environ.get(env_var, '')
        os.environ[env_var] = os.pathsep.join(filter(None, [existing] + gz_resource_paths))

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'rover_arm_sim.xacro'
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro', ' ', xacro_file]),
            value_type=str
        )
    }
    
    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    # 2. Gazebo World Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': f'-r empty.sdf'}.items()
    )

    # 3b. Gazebo Cube Spawner
    node_spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', os.path.join(pkg_share, 'models', 'cube', 'model.sdf'),
            '-name', 'target_cube',
            '-x', '0.3',  # Position 25cm in front of the arm base
            '-y', '-0.3',   # Centered
            '-z', '0.05'   # Drop it slightly above the ground plane
        ]
    )

    # 3. Gazebo Spawner
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
          # '-string', Command([FindExecutable(name='xacro'), ' ', xacro_file]),
            '-topic', 'robot_description',    #Replaced string with topic
            '-name', 'rover_arm',
            '-z', '0.1'
        ]
    )

    # 4. Bridge Node for Clock, Transforms, and Gripper Camera
    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/gripper_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            
            # ZED RGB
            '/zed/zed_node/rgb/color/rect/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/zed_node/rgb/color/rect/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # ZED IMU
            '/zed/zed_node/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            
            # ZED Depth Image & Point Cloud
            '/zed/zed_node/rgb/color/rect/image/depth@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/zed_node/rgb/color/rect/image/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # 5. RViz2
#node_rviz = Node(
#     package='rviz2',
#    executable='rviz2',
# )
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'rviz',
                'rover.rviz'
            ])
        ]
    )

    # ==================== TELEOP & CONTROLLER ADDITIONS ====================

    # 6. Physical Joystick Driver Node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    # 7. Your Custom C++ Teleop Control Node
    sim_control_node = Node(
        package=package_name,
        executable='sim_control_node',
        name='sim_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. ros2_control Spawners
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration('arm_controller')], 
    )

    # Delay loading the controllers until after your robot has actually spawned in Gazebo
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_spawn_entity,
            on_start=[joint_state_broadcaster, arm_controller],
        )
    )
    # =======================================================================

    return LaunchDescription([
        declare_arm_controller_cmd,
        node_robot_state_publisher,
        gazebo_launch,
        node_spawn_entity,
        node_spawn_cube,
        node_gz_bridge,
        node_rviz,
        joy_node,
        sim_control_node,
        delay_controllers
    ])