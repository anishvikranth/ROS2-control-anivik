#!/usr/bin/env python3
"""
goal_generator.py
=================
Generates target goal poses for navigation based on visual marker detections
or falls back to blind SDF ground truth locations.

CORRECTION ADDED:
1. When a visual goal is lost (msg.id == -1), `self.current_target_id` is now 
   reset back to -1. This allows `blind_navigation_check()` to immediately trigger 
   and publish the SDF ground truth goal when visual tracking is lost mid-turn.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401

from rover_aruco_interfaces.msg import Marker


class GoalGenerator(Node):
    def __init__(self):
        super().__init__('goal_generator')

        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'chassis_footprint')
        self.declare_parameter('target_topic', '/current_target')

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.target_id_pub = self.create_publisher(Int32, '/current_target_id', 10)

        # Fallback marker positions as a flat [id, x, y, yaw, ...] list. The default is
        # aruco_world.sdf; terrain_aruco.launch.py passes config/terrain_markers.yaml.
        self.declare_parameter('marker_poses', [0.0, 5.0, 0.0, 0.0,
                                                1.0, 9.0, -1.0, -0.95,
                                                2.0, 15.0, 2.0, 1.12])
        flat = list(self.get_parameter('marker_poses').get_parameter_value().double_array_value)
        self.sdf_marker_poses = {
            int(flat[k]): (flat[k + 1], flat[k + 2], 0.25, 0.0, 0.0, flat[k + 3])
            for k in range(0, len(flat) - 3, 4)
        }

        # Set of permanently completed marker IDs
        self.visited_markers = set()

        self.current_target_id = -1
        self.using_visual_goal = False

        self.create_subscription(Marker, target_topic, self.target_cb, 10)
        self.create_subscription(Int32, '/marker_reached', self.marker_reached_cb, 10)

        # Timer checks for blind navigation every 1 second
        self.create_timer(1.0, self.blind_navigation_check)

        self.get_logger().info("Goal Generator initialized with auto-fallback to SDF goal.")

    def marker_reached_cb(self, msg: Int32) -> None:
        """Fired by movement logic when a marker arrival wait finishes."""
        reached_id = int(msg.data)
        
        # Permanently mark this ID as completed
        self.visited_markers.add(reached_id)
        self.get_logger().info(f"[GoalGenerator] Marker id={reached_id} PERMANENTLY COMPLETED. Visited list: {self.visited_markers}")

        # Reset active state to trigger search/blind nav for the next target
        if self.current_target_id == reached_id:
            self.current_target_id = -1
            self.using_visual_goal = False

    def target_cb(self, msg: Marker) -> None:
        """Visual detection callback from marker_tracker."""
        # --- STRICT FILTERING ---
        if msg.id in self.visited_markers:
            return

        # Keep driving to active target first if we have a solid visual target lock
        if self.current_target_id != -1 and self.current_target_id != msg.id and self.using_visual_goal:
            return

        # =================================================================
        # ISSUE PREVIOUSLY:
        # When msg.id == -1 arrived, `self.current_target_id` was left stuck at `2`.
        # `blind_navigation_check()` saw target_id was already 2 and did NOT send 
        # the SDF goal pose!
        #
        # CORRECTION:
        # Reset `self.current_target_id = -1` when visual contact is lost so that
        # `blind_navigation_check()` immediately picks up the fallback SDF goal.
        # =================================================================
        if msg.id == -1:
            if self.using_visual_goal:
                self.get_logger().warn(
                    f"Visual lost for active target id={self.current_target_id}. Resetting target state to enable blind SDF fallback.")
                self.using_visual_goal = False
                self.current_target_id = -1
            return

        # --- CASE B: Camera detects an UNVISITED marker ---
        try:
            # Query the LATEST available transform to avoid future extrapolation errors
            tf_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id if msg.header.frame_id else 'zed_camera_optical_frame',
                Time(),
                timeout=Duration(seconds=0.2)
            )

            marker_pose_stamped = PoseStamped()
            marker_pose_stamped.header = msg.header
            marker_pose_stamped.pose = msg.pose

            # Transform marker position into map/odom frame
            goal_in_map = tf2_geometry_msgs.do_transform_pose_stamped(marker_pose_stamped, tf_stamped)

            # Compute heading vector (Rover -> Target Marker)
            target_x = goal_in_map.pose.position.x
            target_y = goal_in_map.pose.position.y

            try:
                rover_tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.2)
                )
                rover_x = rover_tf.transform.translation.x
                rover_y = rover_tf.transform.translation.y

                target_yaw = math.atan2(target_y - rover_y, target_x - rover_x)

                goal_in_map.pose.orientation.w = math.cos(target_yaw * 0.5)
                goal_in_map.pose.orientation.x = 0.0
                goal_in_map.pose.orientation.y = 0.0
                goal_in_map.pose.orientation.z = math.sin(target_yaw * 0.5)

            except TransformException as tf_err:
                self.get_logger().warn(f"Could not compute relative heading to marker: {tf_err}")

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform visual marker id={msg.id}: {ex}')
            return

        goal_in_map.header.stamp = self.get_clock().now().to_msg()
        goal_in_map.header.frame_id = self.map_frame

        if not self.using_visual_goal or self.current_target_id != msg.id:
            self.get_logger().info(f"Visual contact established with UNVISITED marker id={msg.id}! Driving to ZED goal.")

        self._publish_goal(goal_in_map, msg.id)
        self.using_visual_goal = True

    def blind_navigation_check(self) -> None:
        """Drives towards the lowest remaining unvisited SDF pose if no visual marker is active."""
        if self.using_visual_goal:
            return

        if len(self.visited_markers) >= len(self.sdf_marker_poses):
            return

        next_unvisited_id = None
        for mid in sorted(self.sdf_marker_poses.keys()):
            if mid not in self.visited_markers:
                next_unvisited_id = mid
                break

        if next_unvisited_id is None:
            return

        # Now publishes smoothly because current_target_id is reset to -1 on vision loss
        if self.current_target_id != next_unvisited_id:
            self.get_logger().info(
                f"No visual target active. Driving blindly to remaining marker id={next_unvisited_id} SDF pose...")
            self.publish_sdf_goal(next_unvisited_id)

    def publish_sdf_goal(self, marker_id: int) -> None:
        if marker_id not in self.sdf_marker_poses:
            return

        x, y, z, r, p, yaw = self.sdf_marker_poses[marker_id]

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.map_frame

        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal_msg.pose.orientation.w = cy
        goal_msg.pose.orientation.z = sy

        self._publish_goal(goal_msg, marker_id)

    def _publish_goal(self, goal_pose: PoseStamped, marker_id: int) -> None:
        # ID before goal: movement.py tags each goal with the last ID it received, so
        # publishing the goal first made it report the previous marker as reached.
        id_msg = Int32()
        id_msg.data = marker_id
        self.target_id_pub.publish(id_msg)

        self.goal_pub.publish(goal_pose)

        self.current_target_id = marker_id


def main(args=None):
    rclpy.init(args=args)
    node = GoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()