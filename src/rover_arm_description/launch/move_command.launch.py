from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "rover_arm",
            package_name="rover_arm_moveit_config",
        )
        .to_moveit_configs()
    )

    # true for Gazebo (arm.launch.py), false for the MoveIt demo or the real arm.
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    moveit_goal_node = Node(
        package="rover_arm_description",
        executable="moveit_goal_node",
        name="moveit_goal_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        moveit_goal_node,
    ])
