import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


ARM_JOINTS = ["base_link_joint", "shoulder_link_joint", "elbow_link_joint", "wrist_link_joint", "palm_link_joint"]
FINGER_JOINTS = ["left_finger_link_joint", "right_finger_link_joint"]


def srdf_poses(srdf_xml):
    """Named poses from the SRDF, as real_arm_teleop_node parameters (the SRDF is the
    single source of truth for poses, shared with MoveIt)."""
    root = ET.fromstring(srdf_xml)
    names, values, gripper = [], [], {}
    for gs in root.findall("group_state"):
        joints = {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
        if gs.get("group") == "arm" and all(j in joints for j in ARM_JOINTS):
            names.append(gs.get("name"))
            values += [joints[j] for j in ARM_JOINTS]
        elif gs.get("group") == "gripper" and all(j in joints for j in FINGER_JOINTS):
            gripper[gs.get("name")] = [joints[j] for j in FINGER_JOINTS]
    return {
        "pose_names": names,
        "pose_values": values,
        "gripper_open": gripper.get("open", []),
        "gripper_closed": gripper.get("closed", []),
    }


def launch_setup(context):
    # Resolved here (not as a LaunchConfiguration) because it has to be baked
    # into the URDF that MoveItConfigsBuilder generates below.
    open_loop_joints = LaunchConfiguration("open_loop_joints").perform(context).strip() or "none"
    ff_gain = LaunchConfiguration("ff_gain").perform(context)
    pwm_deadband = LaunchConfiguration("pwm_deadband").perform(context)
    open_loop = open_loop_joints != "none"

    real_urdf_path = os.path.join(
        get_package_share_directory("rover_arm_description"),
        "urdf",
        "rover_arm_real.xacro",
    )

    # Override the default robot_description source (which would otherwise
    # be rover_arm_moveit_config's own rover_arm.urdf.xacro -> mock hardware)
    # to point straight at the real xacro, which already has its own
    # <ros2_control> block wired to RoverArmHardware. SRDF, kinematics, and
    # planning config still come from rover_arm_moveit_config as normal.
    moveit_config = (
        MoveItConfigsBuilder("rover_arm", package_name="rover_arm_moveit_config")
        .robot_description(file_path=real_urdf_path, mappings={
            "open_loop_joints": open_loop_joints, "ff_gain": ff_gain, "pwm_deadband": pwm_deadband})
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Deliberately NOT using
    # rover_arm_moveit_config's own ros2_controllers.yaml here -- that one
    # is still on position command interfaces for the mock/demo setup.
    real_controllers_yaml = os.path.join(
        get_package_share_directory("rover_arm_description"),
        "config",
        "ros2_controllers_real.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, real_controllers_yaml],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # arm_controller and joystick_only_controller claim the same joints, so
    # exactly one is spawned active. With any open-loop joints we start in
    # joystick-only mode; the teleop node can switch either way at runtime.
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"] + (["--inactive"] if open_loop else []),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"] + (["--inactive"] if open_loop else []),
    )

    joystick_only_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joystick_only_controller", "-c", "/controller_manager"] + ([] if open_loop else ["--inactive"]),
    )

    # Joystick teleop -> /arm_controller/joint_trajectory (manual mode),
    # /joystick_only_controller/commands (joystick-only mode),
    # /arm_state + /input_space + /arm_control_mode (mode publishing). Replaces
    # arm_control_node_v2.py entirely -- see real_arm_teleop_node.cpp for
    # what it does and doesn't port from v2 (no position-space IK yet).
    real_arm_teleop_node = Node(
        package="rover_arm_description",
        executable="real_arm_teleop_node",
        output="screen",
        parameters=[
            {
                "time_from_start_s": LaunchConfiguration("time_from_start_s"),
                "cooldown_s": LaunchConfiguration("cooldown_s"),
                "start_joystick_only": open_loop,
                **srdf_poses(moveit_config.robot_description_semantic["robot_description_semantic"]),
            }
        ],
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        # NOTE: no use_sim_time here, unlike the sim move_group.launch.py --
        # real hardware runs on wall clock.
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
    )

    # SRDF virtual joint world -> stand_link. The real URDF starts at stand_link,
    # so without this RViz's fixed frame "world" doesn't exist.
    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "stand_link"],
    )

    # RViz with the SAME (real) robot model as move_group, on wall-clock time.
    # (moveit_rviz.launch.py would load the sim URDF.)
    moveit_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Optional collision floor for MoveIt (see planning_scene_setup.py).
    floor_z = LaunchConfiguration("collision_floor_z").perform(context).strip()
    scene_setup = [] if floor_z in ("", "none") else [Node(
        package="rover_arm_description",
        executable="planning_scene_setup.py",
        output="screen",
        parameters=[{"floor_z": float(floor_z)}],
    )]

    return scene_setup + [
        world_tf,
        moveit_rviz,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        joystick_only_controller_spawner,
        real_arm_teleop_node,
        move_group,
    ]


def generate_launch_description():

    avatar_repo_arg = DeclareLaunchArgument(
        "avatar_repo_path",
        #Override at launch time with: (Lite only for now)
        #   ros2 launch rover_arm_description arm_real.launch.py avatar_repo_path:=/some/other/path
        default_value=os.path.expanduser("~/AVATAR"),
        description="Path to the cloned AVATAR repo (contains orin_side/arm.py)",
    )
    avatar_repo_path = LaunchConfiguration("avatar_repo_path")

    open_loop_joints_arg = DeclareLaunchArgument(
        "open_loop_joints",
        default_value="none",
        description=(
            "Motors to drive WITHOUT encoder feedback: 'none', 'all', or comma-separated "
            "joint names (e.g. 'wrist_link_joint,palm_link_joint'). Anything other than "
            "'none' starts in joystick-only mode."
        ),
    )

    ff_gain_arg = DeclareLaunchArgument(
        "ff_gain", default_value="100.0",
        description="PWM per rad/s of motor velocity (open loop, and closed-loop feed-forward)")
    pwm_deadband_arg = DeclareLaunchArgument(
        "pwm_deadband", default_value="0.0",
        description="PWM below which the motors don't move (0 = no compensation)")

    collision_floor_arg = DeclareLaunchArgument(
        "collision_floor_z", default_value="none",
        description="Add a floor at this height (m, world frame) to MoveIt's planning scene; "
        "'none' = no floor. arm_real_sim sets Gazebo's floor height.")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Start MoveIt RViz (false on a headless Orin)")

    start_arm_py_arg = DeclareLaunchArgument(
        "start_arm_py",
        default_value="true",
        description="Start the AVATAR serial bridge (arm.py). arm_real_sim.launch.py sets "
        "false and uses virtual_esp.py instead.",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="ESP32 serial port used by arm.py (find it with: ls /dev/ttyACM*)",
    )

    time_from_start_arg = DeclareLaunchArgument(
        "time_from_start_s",
        default_value="0.15",
        description="JointTrajectoryPoint time_from_start for real_arm_teleop_node",
    )
    cooldown_arg = DeclareLaunchArgument(
        "cooldown_s",
        default_value="0.3",
        description="Debounce cooldown (s) for real_arm_teleop_node's mode/autonomous toggle buttons",
    )

    # AVATAR serial bridge -- orin_side/arm.py is a bare script (no
    # package.xml/setup.py in that repo), run directly with python3 rather
    # than via ros2 run. It imports the pip-installed AVATAR python package
    # from the same repo's python/ dir -- make sure that's already
    # `pip install`-ed on the Orin, or this process will fail on import.
    arm_com_node = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([avatar_repo_path, "orin_side", "arm.py"]),
            "--ros-args", "-p", ["port:=", LaunchConfiguration("serial_port")],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_arm_py")),
    )


    return LaunchDescription(
        [
            avatar_repo_arg,
            open_loop_joints_arg,
            serial_port_arg,
            start_arm_py_arg,
            rviz_arg,
            collision_floor_arg,
            ff_gain_arg,
            pwm_deadband_arg,
            time_from_start_arg,
            cooldown_arg,
            arm_com_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
