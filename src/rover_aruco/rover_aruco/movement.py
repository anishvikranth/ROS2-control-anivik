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
3. Turn In Place: Above turn_in_place_threshold of heading error the rover now
   turns on the spot instead of arcing. On skid-steer, a forward arc with a big
   turn command stops one side's wheels and the other side cannot drag the rover
   round it - in sim the rover froze 1 m from marker 2 with one side spinning at
   100% slip. Counter-rotating both sides does turn it.
4. Stall Recovery: If motion is commanded but the rover neither moves nor turns
   for stall_timeout_sec, it backs up for backup_duration_sec and retries. After
   max_recoveries back-ups with no progress in between it stops in ABORTED, since
   backing up cannot fix e.g. a flipped bogie; a new target ID clears it.
5. Turn-In-Place Dead Band: turning in place starts above turn_in_place_threshold
   and only ends below turn_in_place_exit, so the mode doesn't flip every tick
   when the heading error hovers near the threshold (it did on a hillside).
6. No Pivoting While Driving: the turn rate is capped so the inner wheels keep at
   least min_inner_wheel_ratio of the forward speed (track_width sets the geometry).
   v=0.29, w=0.9 nearly stops the inner side - the same pivot that stalls skid-steer
   - and jade stalled on it repeatedly. Bigger corrections wait for turn-in-place.
7. A back-up only counts as progress for the recovery count once the rover is
   recovery_reset_dist from where it first stalled; otherwise backing up reset the
   count every time and the rover looped forever instead of reaching ABORTED.
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

        # ---- Turn In Place ----
        self.declare_parameter('turn_in_place_threshold', 0.5)  # radians (~29 deg), start turning in place
        self.declare_parameter('turn_in_place_exit', 0.3)       # radians (~17 deg), resume driving
        self.declare_parameter('min_turn_speed', 0.3)           # rad/s, below this skid-steer won't break friction
        self.turn_in_place_threshold = self.get_parameter('turn_in_place_threshold').get_parameter_value().double_value
        self.turn_in_place_exit = self.get_parameter('turn_in_place_exit').get_parameter_value().double_value
        self.min_turn_speed = self.get_parameter('min_turn_speed').get_parameter_value().double_value
        self._turning_in_place = False

        # ---- Skid-Steer Turn Limit While Driving ----
        self.declare_parameter('track_width', 0.63)            # m, as DiffDrive's wheel_separation
        self.declare_parameter('min_inner_wheel_ratio', 0.3)   # inner wheels keep >= this x forward speed
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.min_inner_wheel_ratio = self.get_parameter('min_inner_wheel_ratio').get_parameter_value().double_value

        # ---- Stall Recovery ----
        self.declare_parameter('stall_timeout_sec', 3.0)
        self.declare_parameter('stall_min_dist', 0.05)      # meters moved that count as progress
        self.declare_parameter('stall_min_yaw', 0.05)       # radians turned that count as progress
        self.declare_parameter('backup_duration_sec', 1.5)
        self.declare_parameter('backup_speed', 0.3)         # m/s
        self.declare_parameter('max_recoveries', 3)         # back-ups without progress before ABORTED
        self.declare_parameter('recovery_reset_dist', 0.5)  # m from the first stall that counts as progress
        self.stall_timeout_sec = self.get_parameter('stall_timeout_sec').get_parameter_value().double_value
        self.stall_min_dist = self.get_parameter('stall_min_dist').get_parameter_value().double_value
        self.stall_min_yaw = self.get_parameter('stall_min_yaw').get_parameter_value().double_value
        self.backup_duration_sec = self.get_parameter('backup_duration_sec').get_parameter_value().double_value
        self.backup_speed = self.get_parameter('backup_speed').get_parameter_value().double_value
        self.max_recoveries = self.get_parameter('max_recoveries').get_parameter_value().integer_value
        self.recovery_reset_dist = self.get_parameter('recovery_reset_dist').get_parameter_value().double_value
        self._recovery_count = 0        # back-ups since progress was last seen
        self._first_stall_xy = None     # where the current run of back-ups started
        self._stall_ref = None          # (time, x, y, yaw) when progress was last seen
        self._recover_start_time = None
        self._phase_before_recover = 'APPROACH'

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

        # Phases: 'APPROACH', 'ALIGN', 'WAIT', 'RECOVER', 'ABORTED'
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
            self._recovery_count = 0
            self._turning_in_place = False
            self._stall_ref = None
            self._first_stall_xy = None

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

        if self.phase == 'RECOVER':
            self._run_recover()
            return

        if self.phase == 'ABORTED':
            self.cmd_pub.publish(Twist())
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
            self._publish_and_check_stall(cmd)
            return

        # Proportional Steering
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self.yaw)

        if abs(yaw_error) > self.turn_in_place_threshold:
            self._turning_in_place = True
        elif abs(yaw_error) < self.turn_in_place_exit:
            self._turning_in_place = False

        if self._turning_in_place:
            # Large heading error: turn on the spot (both sides counter-rotate)
            cmd.linear.x = 0.0
            cmd.angular.z = self._turn_in_place_speed(self.kp_angular * yaw_error)
        else:
            speed_factor = math.cos(yaw_error)
            cmd.linear.x = min(self.max_linear_speed, self.kp_linear * dist_to_goal) * speed_factor
            # Keep the inner wheels turning forward: inner = v - |w| * track / 2 >= ratio * v
            w_max = min(self.max_angular_speed,
                        2.0 * cmd.linear.x * (1.0 - self.min_inner_wheel_ratio) / self.track_width)
            cmd.angular.z = self._clamp(self.kp_angular * yaw_error, -w_max, w_max)

        self._publish_and_check_stall(cmd)

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
        cmd.angular.z = self._turn_in_place_speed(self.kp_align * yaw_error)
        self._publish_and_check_stall(cmd)

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

    def _turn_in_place_speed(self, raw: float) -> float:
        """Clamps an in-place turn rate to [min_turn_speed, max_angular_speed] in magnitude."""
        mag = self._clamp(abs(raw), self.min_turn_speed, self.max_angular_speed)
        return math.copysign(mag, raw)

    def _publish_and_check_stall(self, cmd: Twist) -> None:
        """Publishes cmd and starts RECOVER if motion is commanded but the rover has
        neither moved stall_min_dist nor turned stall_min_yaw for stall_timeout_sec."""
        self.cmd_pub.publish(cmd)
        now = self.get_clock().now()
        commanding = abs(cmd.linear.x) > 0.05 or abs(cmd.angular.z) > 0.1
        if not commanding or self._stall_ref is None:
            self._stall_ref = (now, self.x, self.y, self.yaw)
            return

        t0, x0, y0, yaw0 = self._stall_ref
        moved = math.hypot(self.x - x0, self.y - y0) > self.stall_min_dist
        turned = abs(self._normalize_angle(self.yaw - yaw0)) > self.stall_min_yaw
        if moved or turned:
            self._stall_ref = (now, self.x, self.y, self.yaw)
            if self._first_stall_xy is not None and math.hypot(
                    self.x - self._first_stall_xy[0],
                    self.y - self._first_stall_xy[1]) > self.recovery_reset_dist:
                self._recovery_count = 0
                self._first_stall_xy = None
            return

        if (now - t0).nanoseconds / 1e9 > self.stall_timeout_sec:
            if self._first_stall_xy is None:
                self._first_stall_xy = (self.x, self.y)
            if self._recovery_count >= self.max_recoveries:
                self.cmd_pub.publish(Twist())
                self.phase = 'ABORTED'
                self.get_logger().error(
                    f'Still stalled after {self._recovery_count} back-ups at '
                    f'x={self.x:.2f}, y={self.y:.2f}. ABORTED - stopping until a new target ID '
                    f'arrives. Check the rover in sim (flipped bogie, high-centred on a rock?).')
                return
            self._recovery_count += 1
            self.get_logger().warn(
                f'Stalled in {self.phase} for {self.stall_timeout_sec:.1f}s '
                f'(cmd v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}). '
                f'Backing up {self.backup_duration_sec:.1f}s '
                f'(attempt {self._recovery_count}/{self.max_recoveries}).')
            self._phase_before_recover = self.phase
            self.phase = 'RECOVER'
            self._recover_start_time = now

    def _run_recover(self) -> None:
        elapsed_sec = (self.get_clock().now() - self._recover_start_time).nanoseconds / 1e9
        if elapsed_sec < self.backup_duration_sec:
            cmd = Twist()
            cmd.linear.x = -self.backup_speed
            self.cmd_pub.publish(cmd)
            return

        self.cmd_pub.publish(Twist())
        self._stall_ref = None
        self.phase = self._phase_before_recover
        self.get_logger().info(f'Recovery done. Resuming {self.phase}.')

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