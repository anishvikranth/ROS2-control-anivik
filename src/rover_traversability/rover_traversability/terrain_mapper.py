#!/usr/bin/env python3
"""
terrain_mapper.py
=================
Builds a rover-centred height map from the ZED depth point cloud and publishes a
traversability cost grid.

  /zed/depth/points (PointCloud2) --TF--> odom frame --> TerrainGrid.fuse()
  every 1/publish_rate s:  TerrainGrid.layers() -> cost
    /traversability/costmap  nav_msgs/OccupancyGrid   -1 unknown, 1..99 cost, 100 no-go
    /traversability/cells    sensor_msgs/PointCloud2  one point per known cell at its
                                                      height, fields slope/step/rough/cost
                                                      (RViz: colour by any of them)

The cloud must be transformable into map_frame at its stamp. In the terrain sim
odom -> chassis_footprint is the full 3D pose (terrain_aruco.launch.py), so the
rover's pitch and roll are taken out before heights are binned.
"""

import os
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException

from rover_traversability.terrain_grid import TerrainGrid


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TerrainMapper(Node):
    def __init__(self):
        super().__init__('terrain_mapper')

        # ---- Parameters ----
        self.declare_parameter('cloud_topic', '/zed/depth/points')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'chassis_footprint')
        self.declare_parameter('resolution', 0.15)          # m per cell
        self.declare_parameter('map_size', 20.0)            # m, square, centred on the rover
        self.declare_parameter('min_range', 0.3)            # m from the camera
        self.declare_parameter('max_range', 8.0)            # m; stereo depth error grows with range^2
        self.declare_parameter('max_points', 60000)         # random subsample per cloud
        self.declare_parameter('max_fusion_weight', 10.0)   # frames of history per cell
        self.declare_parameter('window_cells', 2)           # plane-fit window = (2w+1)^2 cells
        self.declare_parameter('min_window_points', 6)
        self.declare_parameter('max_slope_deg', 20.0)       # no-go above (traction limit with margin)
        self.declare_parameter('max_step_m', 0.12)          # no-go above (wheel radius)
        self.declare_parameter('publish_rate', 2.0)         # Hz
        self.declare_parameter('max_tf_skew', 0.1)          # s between cloud stamp and the TF used
        self.declare_parameter('dump_path', '')             # if set, save layers to this .npz each publish
        # Self-filter: points inside this box (base_frame, m) are the rover itself -- wheels and
        # the stowed arm are in the ZED's view on jade -- and are dropped before mapping.
        # Generous in z: dropping the ground under the rover costs nothing.
        self.declare_parameter('self_filter', True)
        self.declare_parameter('self_box_min', [-0.80, -0.55, -1.0])
        self.declare_parameter('self_box_max', [0.75, 0.55, 1.5])

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.map_frame = gp('map_frame')
        self.base_frame = gp('base_frame')
        self.min_range = gp('min_range')
        self.max_range = gp('max_range')
        self.max_points = gp('max_points')
        self.window_cells = gp('window_cells')
        self.min_window_points = gp('min_window_points')
        self.max_slope_deg = gp('max_slope_deg')
        self.max_step_m = gp('max_step_m')
        self.dump_path = gp('dump_path')
        self.max_tf_skew = gp('max_tf_skew')
        self.self_filter = gp('self_filter')
        self.self_box_min = np.array(gp('self_box_min'), dtype=float)
        self.self_box_max = np.array(gp('self_box_max'), dtype=float)

        self.grid = TerrainGrid(gp('resolution'), gp('map_size'), gp('max_fusion_weight'))
        self.rng = np.random.default_rng(0)
        self.frames_fused = 0
        self.tf_failures = 0
        self.self_points = 0

        # TF callbacks must keep running while cloud_cb waits on a lookup. tf2 puts its
        # subscriptions in their own reentrant group; the mapping work gets a separate group and
        # main() spins everything on one MultiThreadedExecutor. (Not spin_thread=True: that
        # spins this same node in a second executor, and on jade the TF feed stalled after ~10 s.)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        work = MutuallyExclusiveCallbackGroup()

        self.create_subscription(PointCloud2, gp('cloud_topic'), self.cloud_cb, 2, callback_group=work)
        self.cost_pub = self.create_publisher(OccupancyGrid, '/traversability/costmap', 1)
        self.cells_pub = self.create_publisher(PointCloud2, '/traversability/cells', 1)
        self.create_timer(1.0 / gp('publish_rate'), self.publish_layers, callback_group=work)

        self.get_logger().info(
            f'TerrainMapper: {self.grid.n}x{self.grid.n} cells of {self.grid.res} m '
            f'({self.grid.size_m:.1f} m square) in {self.map_frame}, '
            f'no-go above {self.max_slope_deg} deg slope or {self.max_step_m} m step.')

    # ------------------------------------------------------------------
    def lookup(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(target, source, stamp, timeout=Duration(seconds=0.05))
        except TransformException:
            pass
        # Clouds are often stamped a few ms after the newest odom TF (30 Hz). Accept the
        # latest transform if it is within max_tf_skew of the requested time.
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            skew = abs((Time.from_msg(tf.header.stamp) - stamp).nanoseconds) / 1e9
            if stamp.nanoseconds == 0 or skew <= self.max_tf_skew:
                return tf
            raise TransformException(f'latest transform is {skew:.3f} s from the cloud')
        except TransformException as ex:
            self.tf_failures += 1
            self.get_logger().warn(f'TF {source} -> {target}: {ex}', throttle_duration_sec=5.0)
            return None

    def cloud_cb(self, msg: PointCloud2) -> None:
        stamp = Time.from_msg(msg.header.stamp)
        tf = self.lookup(self.map_frame, msg.header.frame_id, stamp)
        base = self.lookup(self.map_frame, self.base_frame, stamp)
        if tf is None or base is None:
            return

        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)].astype(np.float64)
        rng = np.linalg.norm(pts, axis=1)
        pts = pts[(rng > self.min_range) & (rng < self.max_range)]
        if len(pts) > self.max_points:
            pts = pts[self.rng.choice(len(pts), self.max_points, replace=False)]
        if len(pts) == 0:
            return

        R = quat_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        pts = pts @ R.T + np.array([t.x, t.y, t.z])

        bt = base.transform.translation
        base_xyz = np.array([bt.x, bt.y, bt.z])
        if self.self_filter:
            in_base = (pts - base_xyz) @ quat_to_matrix(base.transform.rotation)  # R^T (p - t)
            inside = np.all((in_base > self.self_box_min) & (in_base < self.self_box_max), axis=1)
            pts = pts[~inside]
            self.self_points += int(inside.sum())

        self.grid.recenter(bt.x, bt.y)
        self.grid.fuse(pts)
        self.frames_fused += 1

    def publish_layers(self) -> None:
        if self.frames_fused == 0:
            return
        t0 = time.monotonic()
        slope, step, rough, valid = self.grid.layers(self.window_cells, self.min_window_points)
        cost = TerrainGrid.cost(slope, step, valid, self.max_slope_deg, self.max_step_m)
        stamp = self.get_clock().now().to_msg()

        grid_msg = OccupancyGrid()
        grid_msg.header = Header(stamp=stamp, frame_id=self.map_frame)
        grid_msg.info.map_load_time = stamp
        grid_msg.info.resolution = float(self.grid.res)
        grid_msg.info.width = grid_msg.info.height = self.grid.n
        grid_msg.info.origin.position.x = float(self.grid.origin_x)
        grid_msg.info.origin.position.y = float(self.grid.origin_y)
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = cost.ravel().tolist()
        self.cost_pub.publish(grid_msg)

        X, Y = self.grid.cell_centres()
        cells = np.zeros(int(valid.sum()), dtype=[(f, np.float32) for f in
                                                  ('x', 'y', 'z', 'slope', 'step', 'rough', 'cost')])
        for name, arr in (('x', X), ('y', Y), ('z', self.grid.height), ('slope', slope),
                          ('step', step), ('rough', rough), ('cost', cost)):
            cells[name] = arr[valid]
        fields = [PointField(name=f, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                  for k, f in enumerate(cells.dtype.names)]
        self.cells_pub.publish(point_cloud2.create_cloud(grid_msg.header, fields, cells))

        if self.dump_path:
            # Write then rename, so a reader never sees a half-written file
            tmp = self.dump_path + '.tmp.npz'
            np.savez(tmp, height=self.grid.height, slope=slope, step=step, rough=rough,
                     cost=cost, valid=valid, origin=(self.grid.origin_x, self.grid.origin_y),
                     res=self.grid.res)
            os.replace(tmp, self.dump_path)

        dt = time.monotonic() - t0
        self.get_logger().info(
            f'{self.frames_fused} clouds fused ({self.tf_failures} dropped, no TF), '
            f'{self.self_points} self points dropped, {int(valid.sum())} cells known, '
            f'{int((cost == 100).sum())} no-go; layers {dt * 1000:.0f} ms',
            throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainMapper()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
