"""The REAL arm software stack running against Gazebo instead of the physical arm.

    Gazebo arm (velocity joints, controller manager in /gz)
        ^ joint velocities          | /gz/joint_states
    virtual_esp.py  (stands in for the ESP32 + arm.py: motor model, encoders, watchdog)
        ^ /arm_target_angles (PWM)  | /enc_angles, /arm_esp_uptime
    arm_real.launch.py  (RoverArmHardware, controllers, teleop, MoveIt) -- unchanged

Usage:
    ros2 launch rover_arm_description arm_real_sim.launch.py
    ros2 launch rover_arm_description arm_real_sim.launch.py open_loop_joints:=all
Failure drills: see the docstring of scripts/virtual_esp.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_arm_description')

    # Mesh / model resolution for Gazebo (same as gazebo.launch.py).
    gz_resource_paths = [os.path.dirname(pkg_share), os.path.join(pkg_share, 'models')]
    for env_var in ['GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH']:
        existing = os.environ.get(env_var, '')
        os.environ[env_var] = os.pathsep.join(filter(None, [existing] + gz_resource_paths))

    args = [
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window'),
        DeclareLaunchArgument('open_loop_joints', default_value='none',
                              description='Passed to arm_real.launch.py'),
        DeclareLaunchArgument('use_joy', default_value='true',
                              description='Start joy_node, publishing the joystick on /joy_arm'),
        DeclareLaunchArgument('spawn_cube', default_value='true', description='Spawn the target cube'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start MoveIt RViz'),
        DeclareLaunchArgument('real_stack', default_value='true',
                              description='Start arm_real.launch.py. false = Gazebo + virtual ESP only '
                              '(e.g. to run arm_bringup.py, which needs nothing else driving PWM)'),
        DeclareLaunchArgument('motor_deadband', default_value='30.0',
                              description='Virtual motors: PWM below which they do not move'),
        DeclareLaunchArgument('pwm_deadband', default_value='30.0',
                              description='RoverArmHardware deadband compensation (match motor_deadband)'),
        DeclareLaunchArgument('ff_gain', default_value='255.0',
                              description='RoverArmHardware feed-forward. The virtual motors reach '
                              '1 rad/s at full PWM, so 255 matches them exactly'),
    ]

    # --- Gazebo side: everything in /gz so it can't clash with the real stack ---
    gz_robot_description = ParameterValue(Command([
        'xacro ', os.path.join(pkg_share, 'urdf', 'rover_arm_sim.xacro'),
        ' gz_command_interface:=velocity gz_namespace:=/gz',
        ' gz_controllers_file:=ros2_controllers_gz_motors.yaml',
    ]), value_type=str)

    gz_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='gz',
        parameters=[{'robot_description': gz_robot_description}],
        # Keep Gazebo's TF out of the real stack's /tf.
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': PythonExpression([
            "'-r empty.sdf' if '", LaunchConfiguration('gui'), "' == 'true' else '-r -s empty.sdf'"])
        }.items(),
    )

    spawn_arm = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/gz/robot_description', '-name', 'rover_arm', '-z', '0.1'],
    )
    spawn_cube = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-file', os.path.join(pkg_share, 'models', 'cube', 'model.sdf'),
                   '-name', 'target_cube', '-x', '0.3', '-y', '-0.3', '-z', '0.05'],
        condition=IfCondition(LaunchConfiguration('spawn_cube')),
    )

    gz_joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/gz/controller_manager'],
    )
    gz_motor_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['motor_velocity_controller', '-c', '/gz/controller_manager'],
    )

    # /clock: Gazebo's controller manager runs on ROS (sim) time and can't switch
    # controllers without it. The real stack uses wall-clock time and ignores it.
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/gripper_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
    )

    virtual_esp = Node(
        package='rover_arm_description', executable='virtual_esp.py', output='screen',
        parameters=[{'pwm_deadband': ParameterValue(LaunchConfiguration('motor_deadband'), value_type=float)}],
    )

    joy = Node(
        package='joy', executable='joy_node',
        parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}],
        remappings=[('/joy', '/joy_arm')],
        condition=IfCondition(LaunchConfiguration('use_joy')),
    )

    # --- Real stack, unchanged except that arm.py is replaced by virtual_esp.py ---
    real_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'arm_real.launch.py')),
        launch_arguments={
            'start_arm_py': 'false',
            'open_loop_joints': LaunchConfiguration('open_loop_joints'),
            'rviz': LaunchConfiguration('rviz'),
            'ff_gain': LaunchConfiguration('ff_gain'),
            'pwm_deadband': LaunchConfiguration('pwm_deadband'),
            # The stand sits on Gazebo's floor, ~0.1 m below the stand origin.
            'collision_floor_z': '-0.1',
        }.items(),
    )
    # Start it only once Gazebo's motors exist, so RoverArmHardware's startup
    # wait for /enc_angles sees real Gazebo encoder data. The spawner exits either
    # way, so check that it succeeded.
    def on_motor_spawner_exit(event, context):
        if event.returncode == 0:
            if LaunchConfiguration('real_stack').perform(context) != 'true':
                return [LogInfo(msg='Gazebo + virtual ESP ready (real_stack:=false).')]
            return [real_stack]
        return [LogInfo(msg='ERROR: Gazebo motor_velocity_controller failed to start -- NOT starting '
                            'the real-arm stack. Restart arm_real_sim.launch.py.')]

    start_real_stack_when_ready = RegisterEventHandler(
        OnProcessExit(target_action=gz_motor_controller, on_exit=on_motor_spawner_exit))

    return LaunchDescription(args + [
        gazebo,
        gz_robot_state_publisher,
        spawn_arm,
        spawn_cube,
        gz_joint_state_broadcaster,
        gz_motor_controller,
        gz_bridge,
        virtual_esp,
        joy,
        start_real_stack_when_ready,
    ])
