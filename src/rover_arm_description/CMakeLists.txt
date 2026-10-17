cmake_minimum_required(VERSION 3.8)
project(rover_arm_description)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


# ==================== Simulation Control Node ====================

add_executable(sim_control_node
  src/sim_control_node.cpp
)

ament_target_dependencies(
  sim_control_node
  rclcpp
  sensor_msgs
  trajectory_msgs
)


# ==================== Real Arm Teleop Node ====================

add_executable(real_arm_teleop_node
  src/real_arm_teleop_node.cpp
)

ament_target_dependencies(
  real_arm_teleop_node
  rclcpp
  sensor_msgs
  trajectory_msgs
  std_msgs
  controller_manager_msgs
)


# ==================== MoveIt Goal Node ====================

add_executable(moveit_goal_node
  src/moveit_goal_node.cpp
)

ament_target_dependencies(
  moveit_goal_node
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)


# ==================== Install Executables ====================

install(
  TARGETS
    sim_control_node
    real_arm_teleop_node
    moveit_goal_node
  DESTINATION lib/${PROJECT_NAME}
)


# ==================== Install Scripts ====================

install(
  PROGRAMS
    scripts/virtual_esp.py
    scripts/arm_bringup.py
    scripts/planning_scene_setup.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==================== Install Package Resources ====================

install(
  DIRECTORY
    urdf
    stl
    launch
    config
    models
  DESTINATION share/${PROJECT_NAME}
)


ament_package()