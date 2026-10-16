from launch import LaunchDescription

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "rover_arm",
            package_name="rover_arm_moveit_config",
        )
        .to_moveit_configs()
    )

    rviz_config = (
        moveit_config.package_path / "config/moveit.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", str(rviz_config)],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
            "use_sim_time": True,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            },
        ],
    )

    return LaunchDescription(
        [
            
            rviz,
        ]
    )