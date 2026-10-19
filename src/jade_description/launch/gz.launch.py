import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_jade_description = get_package_share_directory('jade_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    rviz_config_file = os.path.join(pkg_jade_description, 'config', 'rover.rviz')
    world_path = os.path.join(pkg_jade_description, 'worlds', 'world.sdf')
    declare_arm_controller_cmd = DeclareLaunchArgument(
        'arm_controller',
        default_value='arm_controller',
        description='Name of the ros2_control controller to spawn for the arm joints.'
    )

    urdf_path = os.path.join(pkg_jade_description, 'urdf', 'jade.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Get share parent directories for all packages containing URDF meshes
    pkg_jade_dir = os.path.dirname(pkg_jade_description)
    pkg_rover_dir = os.path.dirname(get_package_share_directory('rover_description'))
    pkg_arm_dir = os.path.dirname(get_package_share_directory('rover_arm_description'))

    # Combine paths separated by colons and retain existing environment variables
    resource_paths = [pkg_jade_dir, pkg_rover_dir, pkg_arm_dir]
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        resource_paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_paths)
    )

    # 2. Gazebo World Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}'
        }.items()
    )

    # 2. Publish Robot State
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

    # 3. Spawn Robot
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

    # 4. ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    # 5. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # 6. Joystick driver nodes -- two physical joysticks, two topics.
    #    device_id 0 -> drive stick, publishes /joy
    #    device_id 1 -> arm stick,   publishes /joy_arm
    joy_node_drive = Node(
        package='joy',
        executable='joy_node',
        name='joy_node_drive',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time': True
        }],
        remappings=[('/joy', '/joy')]
    )

    joy_node_arm = Node(
        package='joy',
        executable='joy_node',
        name='joy_node_arm',
        parameters=[{
            'device_id': 1,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time': True
        }],
        remappings=[('/joy', '/joy_arm')]
    )

    # 7. Arm teleop control node (lives in rover_arm_description, not jade_description)
    sim_control_node = Node(
        package='rover_arm_description',
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

    # Delay loading controllers until spawn_rover has actually FINISHED
    # (process exited after successfully spawning the entity in Gazebo).
    # OnProcessStart fires too early -- the robot and controller_manager
    # aren't ready yet at that instant.
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_rover,
            on_exit=[joint_state_broadcaster, arm_controller],
        )
    )

    return LaunchDescription([
        declare_arm_controller_cmd,
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_rover,
        ros_gz_bridge,
        rviz,
        joy_node_drive,
        joy_node_arm,
        sim_control_node,
        delay_controllers,
    ])