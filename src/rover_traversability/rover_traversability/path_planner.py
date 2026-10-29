#!/usr/bin/env python3
"""
path_planner.py
===============
Plans a path from the rover to the current goal over terrain_mapper's cost map (A*,
grid_planner.py) and republishes it once a second as the map fills in.

  /traversability/costmap  nav_msgs/OccupancyGrid  (terrain_mapper)
  /odom                    nav_msgs/Odometry       rover pose, same frame as the map
  /goal_pose               geometry_msgs/PoseStamped  (goal_generator)
  /marker_reached          std_msgs/Int32          clears the goal until the next one
  ->  /planned_path        nav_msgs/Path           start .. goal, a pose every path_spacing m
  ->  /speed_limit         std_msgs/Float64        m/s for the next slow_lookahead m of the path

Speed limit: the worst cost-map cell along the path over the next slow_lookahead metres ahead
of the rover sets it, so the rover slows *before* the trench, a steep hill or rocks. Cost up
to slow_cost_start: speed_max; from there to no-go: down linearly to speed_min; unseen ground:
speed_unknown. Published at speed_rate while a path exists; movement.py caps its speed by it.

movement.py follows /planned_path while the goal is far away. An empty Path means "no
plan" (no goal, or no path found); movement.py then falls back to steering straight.

Like terrain_mapper, run it WITHOUT use_sim_time: nothing here needs the node's clock (the
path is stamped with the cost map's time), and Gazebo's ~1 kHz /clock would only load it.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float64, Int32

from rover_traversability.grid_planner import GridPlanner


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.declare_parameter('robot_radius', 0.45)       # m; no-go cells inflated by this
        self.declare_parameter('inflation_radius', 1.0)    # m; extra cost out to here
        self.declare_parameter('inflation_cost', 40.0)
        self.declare_parameter('unknown_cost', 40.0)       # unseen cells: passable but discouraged
        self.declare_parameter('cost_weight', 1.0)         # step cost = dist * (1 + cost_weight * cost/100)
        self.declare_parameter('goal_tolerance', 1.0)      # m; a no-go goal moves to a free cell this close
        self.declare_parameter('plan_rate', 1.0)           # Hz
        self.declare_parameter('path_spacing', 0.3)        # m between published poses
        # Rover centre in the odom child frame (chassis_footprint), m. That frame sits ~0.31 m
        # left of the chassis centre (rover_raw/rover_sim chassis_link_joint offset); planning
        # from it would keep a point near the left wheels clear, not the rover.
        self.declare_parameter('base_offset', [-0.05, -0.31])
        # Speed limit from the cost ahead on the path
        self.declare_parameter('speed_max', 0.8)            # m/s on easy ground (movement.py's cap)
        self.declare_parameter('speed_min', 0.25)           # m/s at the no-go limit
        self.declare_parameter('speed_unknown', 0.5)        # m/s when the path crosses unseen ground
        self.declare_parameter('slow_cost_start', 30.0)     # cost below this doesn't slow the rover
        self.declare_parameter('slow_lookahead', 2.5)       # m of path ahead that sets the limit
        self.declare_parameter('speed_rate', 5.0)           # Hz

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.planner = GridPlanner(gp('robot_radius'), gp('inflation_radius'), gp('inflation_cost'),
                                   gp('unknown_cost'), gp('cost_weight'), gp('goal_tolerance'))
        self.path_spacing = gp('path_spacing')
        self.base_offset = list(gp('base_offset'))
        self.speed_max = gp('speed_max')
        self.speed_min = gp('speed_min')
        self.speed_unknown = gp('speed_unknown')
        self.slow_cost_start = gp('slow_cost_start')
        self.slow_lookahead = gp('slow_lookahead')
        self.path = []              # full-resolution path from the last plan (cell centres)
        self.last_limit = None

        self.costmap = None
        self.pose = None
        self.goal = None
        self.last_reason = None

        self.create_subscription(OccupancyGrid, '/traversability/costmap', self.costmap_cb, 1)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(Int32, '/marker_reached', self.reached_cb, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 1)
        self.speed_pub = self.create_publisher(Float64, '/speed_limit', 1)
        self.create_timer(1.0 / gp('plan_rate'), self.plan)
        self.create_timer(1.0 / gp('speed_rate'), self.publish_speed_limit)

        self.get_logger().info(
            f'PathPlanner: A* on /traversability/costmap, robot radius {gp("robot_radius")} m, '
            f'cost weight {gp("cost_weight")}.')

    def costmap_cb(self, msg):
        self.costmap = msg

    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        ox, oy = self.base_offset
        self.pose = (p.x + math.cos(yaw) * ox - math.sin(yaw) * oy,
                     p.y + math.sin(yaw) * ox + math.cos(yaw) * oy)

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def reached_cb(self, msg):
        self.goal = None
        self.path = []
        self.publish([], self.costmap.header if self.costmap else None)

    def plan(self):
        if self.costmap is None or self.pose is None or self.goal is None:
            return
        m = self.costmap
        cost = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        t0 = time.monotonic()
        path, info = self.planner.plan(cost, m.info.origin.position.x, m.info.origin.position.y,
                                       m.info.resolution, self.pose, self.goal)
        dt = (time.monotonic() - t0) * 1000
        if info['reason'] != self.last_reason:      # log changes, not every plan
            if path:
                self.get_logger().info(
                    f'Path to ({self.goal[0]:.1f}, {self.goal[1]:.1f}): '
                    f'{self.length(path):.1f} m, {dt:.0f} ms')
            else:
                self.get_logger().warn(f'No path to ({self.goal[0]:.1f}, {self.goal[1]:.1f}): {info["reason"]}')
            self.last_reason = info['reason']
        self.path = path
        self.publish(path, m.header)

    def speed_for(self, costs):
        """Speed limit (m/s) for the cost-map values met along the path ahead."""
        known = [c for c in costs if c >= 0]
        limit = self.speed_max
        if known:
            worst = max(known)
            frac = (worst - self.slow_cost_start) / (100.0 - self.slow_cost_start)
            limit = self.speed_max - min(max(frac, 0.0), 1.0) * (self.speed_max - self.speed_min)
        if len(known) < len(costs):
            limit = min(limit, self.speed_unknown)
        return limit

    def publish_speed_limit(self):
        if not self.path or self.pose is None or self.costmap is None:
            return
        m = self.costmap
        res, w, h = m.info.resolution, m.info.width, m.info.height
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        # From the path point nearest the rover, collect costs over the next slow_lookahead m
        k = min(range(len(self.path)),
                key=lambda i: (self.path[i][0] - self.pose[0]) ** 2 + (self.path[i][1] - self.pose[1]) ** 2)
        costs, travelled = [], 0.0
        while k < len(self.path) and travelled <= self.slow_lookahead:
            x, y = self.path[k]
            i, j = int((y - oy) / res), int((x - ox) / res)
            if 0 <= i < h and 0 <= j < w:
                costs.append(m.data[i * w + j])
            if k + 1 < len(self.path):
                travelled += math.hypot(self.path[k + 1][0] - x, self.path[k + 1][1] - y)
            k += 1
        if not costs:
            return
        limit = self.speed_for(costs)
        self.speed_pub.publish(Float64(data=limit))
        if self.last_limit is None or abs(limit - self.last_limit) > 0.1:
            self.get_logger().info(f'Speed limit {limit:.2f} m/s (worst cost ahead {max(costs)})')
            self.last_limit = limit

    @staticmethod
    def length(path):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))

    def publish(self, path, header):
        msg = Path()
        if header is not None:
            msg.header = header
        # Thin to one pose every path_spacing m (always keeping both ends), heading along the path
        pts, acc = [], 0.0
        for k, p in enumerate(path):
            if k == 0 or k == len(path) - 1:
                pts.append(p)
                continue
            acc += math.hypot(p[0] - path[k - 1][0], p[1] - path[k - 1][1])
            if acc >= self.path_spacing:
                pts.append(p)
                acc = 0.0
        for k, (x, y) in enumerate(pts):
            nx, ny = pts[min(k + 1, len(pts) - 1)]
            px, py = pts[max(k - 1, 0)]
            yaw = math.atan2(ny - py, nx - px)
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(ps)
        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
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
