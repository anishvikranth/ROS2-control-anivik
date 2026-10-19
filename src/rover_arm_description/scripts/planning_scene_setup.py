#!/usr/bin/env python3
"""Adds static obstacles to MoveIt's planning scene once move_group is up.

- a floor plane at z = floor_z (world frame = the arm stand), if floor_z is set
- the rover body (chassis box + wheels) from rover_body_file, if set

Without them MoveIt happily plans through the floor or the rover. Started by
arm_real.launch.py (collision_floor_z:=..., rover_body:=true).
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def make_object(name, shape, dims, position, orientation=(0.0, 0.0, 0.0, 1.0)):
    obj = CollisionObject()
    obj.header.frame_id = "world"
    obj.id = name
    obj.operation = CollisionObject.ADD
    kind = {"box": SolidPrimitive.BOX, "cylinder": SolidPrimitive.CYLINDER}[shape]
    obj.primitives.append(SolidPrimitive(type=kind, dimensions=[float(d) for d in dims]))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in orientation)
    obj.primitive_poses.append(pose)
    return obj


def main():
    rclpy.init()
    node = Node("planning_scene_setup")
    floor_z = node.declare_parameter("floor_z", float("nan")).value
    size = node.declare_parameter("floor_size", 4.0).value
    rover_body_file = node.declare_parameter("rover_body_file", "").value

    scene = PlanningScene(is_diff=True)
    added = []
    if not math.isnan(floor_z):
        thickness = 0.02
        scene.world.collision_objects.append(
            make_object("floor", "box", [size, size, thickness], [0.0, 0.0, floor_z - thickness / 2.0]))
        added.append(f"floor at z = {floor_z:.3f} m")
    if rover_body_file:
        with open(rover_body_file) as f:
            obstacles = yaml.safe_load(f)["obstacles"]
        for o in obstacles:
            scene.world.collision_objects.append(
                make_object("rover_" + o["name"], o["shape"], o["dimensions"], o["position"], o["orientation"]))
        added.append(f"rover body ({len(obstacles)} parts)")
    if not added:
        node.get_logger().info("Nothing to add to the planning scene.")
        rclpy.try_shutdown()
        return

    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info("Waiting for move_group's /apply_planning_scene...")
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is not None and future.result().success:
        node.get_logger().info("Added to MoveIt's planning scene: " + ", ".join(added) + ".")
    else:
        node.get_logger().error("Failed to add obstacles to the planning scene.")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
