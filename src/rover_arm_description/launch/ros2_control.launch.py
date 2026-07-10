from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare 
from launch_ros.parameter_descriptions import ParameterValue


#Command is for converting xacro to URDF file type
#PackageShare is to figure out where the xacro file is located
#PathJoinSubstitution is to join the path to the xacro file



def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('rover_arm_description'), 'urdf', 'rover_arm.xacro'
    ])  #Tells the launch file where to find the xacro file for the robot description
    robot_description = {
    "robot_description": ParameterValue(Command([FindExecutable(name="xacro")," ",xacro_file]),value_type=str,)}
    #Creating the robot state publisher node, which will publish the robot description to the ROS2 parameter server
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    #Creating the ros2 control node, which will load the robot description and start the controllers
    node_ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,                              #Used by the resource manager
            PathJoinSubstitution([
                FindPackageShare("rover_arm_description"),  #Used by the controller manager
                "config",
                "controllers.yaml", 
            ]),
        ],
    )

    #Creating spawner node which will activate the controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    #Second spawner node which will activate the joint trajectory controllers
    joint_trajectory_controller_spawner = Node(
    package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    # The normal rviz node with the current rviz model
    rviz_config = PathJoinSubstitution([
        FindPackageShare("rover_arm_description"),
        "rviz",
        "rover.rviz",
    ])

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    

    return LaunchDescription([node_robot_state_publisher,node_ros2_control,joint_state_broadcaster_spawner,joint_trajectory_controller_spawner,node_rviz])

