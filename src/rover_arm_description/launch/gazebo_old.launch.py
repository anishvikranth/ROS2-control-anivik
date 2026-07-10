from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

#Extra imports for checking
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Locating the xacro file using your preferred substitutions
    package_name = 'rover_arm_description'
    
    # 1. Get the path to your package's share directory inside install/
    pkg_share = get_package_share_directory(package_name)

    # 2. Go one directory up so Gazebo can resolve the actual folder name 'rover_arm_description'
    # This points to: ~/sim_ws/install/rover_arm_description/share/
    gazebo_model_path = os.path.dirname(pkg_share)
    
    # 3. Inject this path into Gazebo's resource environment variables
    # We set both variables to support both older Ignition and newer Gazebo configurations
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + gazebo_model_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gazebo_model_path

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] += os.pathsep + gazebo_model_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = gazebo_model_path

    xacro_file = PathJoinSubstitution([
        FindPackageShare('rover_arm_description'), 'urdf', 'rover_arm.xacro'
    ])
    
    # Compiling xacro via the ROS 2 Command pipe
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str
        )
    }

    # 1. Robot State Publisher (Works natively with the Command substitution)
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
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # 3. Gazebo Spawner (We pass the same Command substitution to the arguments list)
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover_arm',
            '-z', '0.1'
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            PathJoinSubstitution([
                FindPackageShare('rover_arm_description'),
                'config',
                'controllers.yaml'
            ]),
        ],
    )

    # 4. Bridge Node for Clock and Transforms
    node_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # 5. RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
    )

  #  joint_state_publisher_gui = Node(
  #      package='joint_state_publisher_gui',
   #     executable='joint_state_publisher_gui',
  #  )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo_launch,
        node_spawn_entity,
        node_gz_bridge,
        node_rviz,
       # joint_state_publisher_gui
    ])