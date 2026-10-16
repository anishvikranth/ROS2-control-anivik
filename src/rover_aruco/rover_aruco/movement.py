#!/usr/bin/env python3
"""
movement.py (GoalFollowerAvoider)
=================================
Drives the robot to the pose published on /goal_pose using proportional control
and obstacle avoidance.

UPDATED & FIXED:
1. Dynamic Pose Updates: Removed the check that blocked continuous pose updates
   during the APPROACH phase. The node now accepts refined visual goal estimates
   as the rover gets closer to the marker.
2. Tighter Arrival Tolerance: Reduced position tolerance slightly to force closer
   approach before entering final alignment.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32


class GoalFollowerAvoider(Node):
    def __init__(self):
        super().__init__('goal_follower_avoider')

        # ---- Parameters ----
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # ---- Gains & Limits ----
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('kp_align', 2.0)
        self.declare_parameter('max_linear_speed', 2.4)
        self.declare_parameter('max_angular_speed', 6.0)

        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.kp_align = self.get_parameter('kp_align').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        # ---- Tolerances ----
        # CORRECTION: Decreased default position tolerance from 0.50m to 0.35m to ensure
        # closer approach to the marker before starting final yaw alignment.
        self.declare_parameter('position_tolerance', 0.35)   # meters
        self.declare_parameter('yaw_tolerance', 0.10)        # radians (~5.7 deg)
        self.declare_parameter('obstacle_stop_dist', 0.65)    # meters
        self.declare_parameter('front_fov_rad', 0.8)         # radians (~45 deg half-angle)

        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').get_parameter_value().double_value
        self.obstacle_stop_dist = self.get_parameter('obstacle_stop_dist').get_parameter_value().double_value
        self.front_fov_rad = self.get_parameter('front_fov_rad').get_parameter_value().double_value

        # ---- Stop Duration ----
        self.declare_parameter('stop_duration_sec', 3.0)
        self.stop_duration_sec = self.get_parameter('stop_duration_sec').get_parameter_value().double_value
        self._wait_start_time = None

        # ---- Robot State ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

        # ---- Goal State ----
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.current_target_id = -1
        self._active_goal_id = -1

        # Phases: 'APPROACH', 'ALIGN', 'WAIT'
        self.phase = 'APPROACH'

        # ---- Obstacle State ----
        self.min_front_dist = float('inf')
        self.obstacle_angle = 0.0

        # ---- Publishers & Subscribers ----
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.reached_pub = self.create_publisher(Int32, '/marker_reached', 10)

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)
        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.create_subscription(Int32, '/current_target_id', self.target_id_cb, 10)

        self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.get_logger().info('GoalFollowerAvoider ready with dynamic pose update support.')

    def odom_cb(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def target_id_cb(self, msg: Int32) -> None:
        """Tracks target ID updates from generator/tracker."""
        self.current_target_id = int(msg.data)

    def goal_cb(self, msg: PoseStamped) -> None:
        """Receives goal pose updates from goal_generator."""
        target_id = self.current_target_id

        # =================================================================
        # ISSUE IN PREVIOUS VERSION:
        # `if self.goal_x is not None and target_id == self._active_goal_id:`
        # locked the initial goal pose permanently. All subsequent refined pose
        # updates coming from vision during approach were ignored, leading to
        # parking errors based on noisy, far-away estimates.
        #
        # CORRECTION:
        # Only reject updates if we are actively executing the final 'ALIGN' or 'WAIT' phase.
        # During 'APPROACH', continuous position/orientation updates are accepted to refine pathing.
        # =================================================================
        if self.phase in ['ALIGN', 'WAIT'] and target_id == self._active_goal_id:
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = self._quat_to_yaw(msg.pose.orientation)
        
        # If target ID changes, reset phase back to APPROACH
        if self._active_goal_id != target_id:
            self.phase = 'APPROACH'
            self._active_goal_id = target_id

        self.get_logger().info(
            f'Goal Updated for Target ID={target_id}: '
            f'x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={math.degrees(self.goal_yaw):.1f}°',
            throttle_duration_sec=2.0)

    def scan_cb(self, msg: LaserScan) -> None:
        min_dist = float('inf')
        min_angle = 0.0

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if -self.front_fov_rad < angle < self.front_fov_rad:
                if r < min_dist:
                    min_dist = r
                    min_angle = angle

        self.min_front_dist = min_dist
        self.obstacle_angle = min_angle

    def control_loop(self) -> None:
        if not self.have_odom:
            return

        if self.phase == 'WAIT':
            self._run_wait()
            return

        if self.goal_x is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist_to_goal = math.hypot(dx, dy)

        # Transition to ALIGN when within position tolerance
        if self.phase == 'APPROACH' and dist_to_goal < self.position_tolerance:
            self.phase = 'ALIGN'
            self.get_logger().info('Position reached. Aligning orientation...')

        if self.phase == 'APPROACH':
            self._run_approach(dx, dy, dist_to_goal)
        elif self.phase == 'ALIGN':
            self._run_align()

    def _run_approach(self, dx: float, dy: float, dist_to_goal: float) -> None:
        cmd = Twist()

        # Reactive Obstacle Avoidance
        if self.min_front_dist < self.obstacle_stop_dist:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.8 if self.obstacle_angle > 0 else 0.8
            self.cmd_pub.publish(cmd)
            return

        # Proportional Steering
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self.yaw)

        speed_factor = max(0.0, math.cos(yaw_error))
        cmd.linear.x = min(self.max_linear_speed, self.kp_linear * dist_to_goal) * speed_factor
        cmd.angular.z = self._clamp(
            self.kp_angular * yaw_error, -self.max_angular_speed, self.max_angular_speed)

        self.cmd_pub.publish(cmd)

    def _run_align(self) -> None:
        yaw_error = self._normalize_angle(self.goal_yaw - self.yaw)

        if abs(yaw_error) < self.yaw_tolerance:
            self.cmd_pub.publish(Twist())  # Stop moving
            self.phase = 'WAIT'
            self._wait_start_time = self.get_clock().now()
            self.get_logger().info(f'Goal reached! Holding position for {self.stop_duration_sec:.1f}s...')
            return

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self._clamp(
            self.kp_align * yaw_error, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

    def _run_wait(self) -> None:
        self.cmd_pub.publish(Twist())  # Keep stopped

        elapsed_sec = (self.get_clock().now() - self._wait_start_time).nanoseconds / 1e9
        if elapsed_sec < self.stop_duration_sec:
            return

        # Goal sequence finished
        if self._active_goal_id != -1:
            reached_msg = Int32()
            reached_msg.data = self._active_goal_id
            self.reached_pub.publish(reached_msg)
            self.get_logger().info(f'Finished waiting. Reported target ID={self._active_goal_id} reached.')

        # Reset states for next goal
        self.goal_x = self.goal_y = self.goal_yaw = None
        self._active_goal_id = -1
        self.phase = 'APPROACH'

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


def main(args=None):
    rclpy.init(args=args)
    node = GoalFollowerAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()