#!/usr/bin/env python3
"""Adds static obstacles to MoveIt's planning scene once move_group is up.

Currently: a floor plane at z = floor_z (in the "world" frame, i.e. the arm stand).
Without it MoveIt happily plans through the floor -- in Gazebo the arm then hits the
ground and arm_controller aborts the move. Started by arm_real.launch.py when
collision_floor_z is set (arm_real_sim.launch.py sets it to Gazebo's floor height).
"""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def main():
    rclpy.init()
    node = Node("planning_scene_setup")
    floor_z = node.declare_parameter("floor_z", 0.0).value
    size = node.declare_parameter("floor_size", 4.0).value
    thickness = 0.02

    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info("Waiting for move_group's /apply_planning_scene...")

    floor = CollisionObject()
    floor.header.frame_id = "world"
    floor.id = "floor"
    floor.operation = CollisionObject.ADD
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size, size, thickness])
    pose = Pose()
    pose.position.z = floor_z - thickness / 2.0  # top surface at floor_z
    pose.orientation.w = 1.0
    floor.primitives.append(box)
    floor.primitive_poses.append(pose)

    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects.append(floor)
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is not None and future.result().success:
        node.get_logger().info(f"Collision floor added to MoveIt's planning scene at z = {floor_z:.3f} m.")
    else:
        node.get_logger().error("Failed to add the collision floor to the planning scene.")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
