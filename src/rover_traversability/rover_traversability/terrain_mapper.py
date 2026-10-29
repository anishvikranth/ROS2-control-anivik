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
                                                      height, fields slope/step/rough/cost/sigma
                                                      (RViz: colour by any of them)

Each point's height variance comes from its range, sigma = max(noise_min, noise_k * r^2):
stereo depth error grows with range squared (the default k gives ~1% at 3 m and ~5% at
15 m, roughly the ZED's spec). Cells fuse readings by 1/variance, so close readings
dominate far ones; cells still too uncertain (sigma > max_cell_sigma) stay unknown.

Run it WITHOUT use_sim_time, also in Gazebo: with sim time every node subscribes to
/clock, which Gazebo publishes at ~1 kHz, and in this Python node that flood alone made a
map update take 1-3 s instead of ~0.2 s and starved the clouds. Nothing here needs the
node's clock: TF lookups use each cloud's own stamp and outputs carry the newest cloud's
stamp. (On the real rover there is no /clock either way.)

The cloud must be transformable into map_frame at its stamp. In the terrain sim
odom -> chassis_footprint is the full 3D pose (terrain_aruco.launch.py), so the
rover's pitch and roll are taken out before heights are binned.
"""

import os
import threading
import time

# One BLAS thread: numpy's multi-threaded BLAS fights the executor's threads and the
# other processes (an update once took 1-2 s instead of milliseconds). Set before import.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

import numpy as np  # noqa: E402
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
        self.declare_parameter('noise_k', 0.0033)           # point sigma = max(noise_min, noise_k * range^2)
        self.declare_parameter('noise_min', 0.01)           # m
        self.declare_parameter('process_noise', 0.01)       # m added per update, so cells can still change
        self.declare_parameter('max_cell_sigma', 0.2)       # m; more uncertain cells are left unknown
        self.declare_parameter('window_cells', 2)           # plane-fit window = (2w+1)^2 cells
        self.declare_parameter('min_window_points', 6)
        self.declare_parameter('max_slope_deg', 20.0)       # no-go above (traction limit with margin)
        self.declare_parameter('max_step_m', 0.12)          # no-go above (wheel radius)
        self.declare_parameter('publish_rate', 2.0)         # Hz
        self.declare_parameter('max_tf_skew', 0.1)          # s between cloud stamp and the TF used
        self.declare_parameter('dump_path', '')             # if set, save layers to this .npz each publish
        # Self-filter: points inside this box (base_frame, m) are the rover itself and are
        # dropped before mapping. chassis_footprint sits ~0.31 m left of the chassis centre
        # (rover_raw/rover_sim chassis_link_joint offset), so the box is offset to match: wheels
        # at y +0.02 and -0.64 (+-0.09 tyre). It reaches 1.05 m forward for jade's stowed
        # gripper (fingers at x ~0.83, z ~0.45): at 0.75 m the ZED mapped them as a tall rock
        # just ahead of the rover, leaving a no-go trail and blocking the path. Generous in z:
        # dropping the ground under the rover costs nothing.
        self.declare_parameter('self_filter', True)
        self.declare_parameter('self_box_min', [-0.80, -0.85, -1.0])
        self.declare_parameter('self_box_max', [1.05, 0.25, 1.5])

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

        self.noise_k = gp('noise_k')
        self.noise_min = gp('noise_min')
        self.max_cell_sigma = gp('max_cell_sigma')
        self.grid = TerrainGrid(gp('resolution'), gp('map_size'), gp('process_noise'))
        self.rng = np.random.default_rng(0)
        self.frames_fused = 0
        self.last_cloud_stamp = None   # outputs are stamped with the newest cloud they include
        self.tf_failures = 0
        self.self_points = 0

        # Three things run concurrently on one MultiThreadedExecutor (see main()): TF (tf2 puts
        # its subscriptions in their own reentrant group), fusing clouds, and computing layers,
        # each in its own group. Clouds and layers used to share one group: when an update ran
        # longer than the timer period the timer was always due, and clouds were starved after
        # the first few. A lock guards the grid; layers are computed on a copy.
        # (Not spin_thread=True: that spins this node in a second executor, and on jade the TF
        # feed stalled after ~10 s.)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.grid_lock = threading.Lock()

        self.create_subscription(PointCloud2, gp('cloud_topic'), self.cloud_cb, 2,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.cost_pub = self.create_publisher(OccupancyGrid, '/traversability/costmap', 1)
        self.cells_pub = self.create_publisher(PointCloud2, '/traversability/cells', 1)
        self.create_timer(1.0 / gp('publish_rate'), self.publish_layers,
                          callback_group=MutuallyExclusiveCallbackGroup())

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
        rng = np.linalg.norm(pts, axis=1)               # range from the camera
        keep = (rng > self.min_range) & (rng < self.max_range)
        pts, rng = pts[keep], rng[keep]
        if len(pts) > self.max_points:
            pick = self.rng.choice(len(pts), self.max_points, replace=False)
            pts, rng = pts[pick], rng[pick]
        if len(pts) == 0:
            return
        variances = np.maximum(self.noise_min, self.noise_k * rng * rng) ** 2

        R = quat_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        pts = pts @ R.T + np.array([t.x, t.y, t.z])

        bt = base.transform.translation
        base_xyz = np.array([bt.x, bt.y, bt.z])
        if self.self_filter:
            in_base = (pts - base_xyz) @ quat_to_matrix(base.transform.rotation)  # R^T (p - t)
            inside = np.all((in_base > self.self_box_min) & (in_base < self.self_box_max), axis=1)
            pts, variances = pts[~inside], variances[~inside]
            self.self_points += int(inside.sum())

        with self.grid_lock:
            self.grid.recenter(bt.x, bt.y)
            self.grid.fuse(pts, variances)
        self.frames_fused += 1
        self.last_cloud_stamp = msg.header.stamp

    def publish_layers(self) -> None:
        if self.frames_fused == 0:
            return
        t0 = time.monotonic()
        with self.grid_lock:
            grid = self.grid.copy()
        slope, step, rough, valid, sigma = grid.layers(
            self.window_cells, self.min_window_points, self.max_cell_sigma)
        cost = TerrainGrid.cost(slope, step, valid, self.max_slope_deg, self.max_step_m)
        # The newest fused cloud's time, not this node's clock: then the map lines up with TF
        # whether or not the node uses sim time (and it shouldn't, see the module docstring).
        stamp = self.last_cloud_stamp

        grid_msg = OccupancyGrid()
        grid_msg.header = Header(stamp=stamp, frame_id=self.map_frame)
        grid_msg.info.map_load_time = stamp
        grid_msg.info.resolution = float(grid.res)
        grid_msg.info.width = grid_msg.info.height = grid.n
        grid_msg.info.origin.position.x = float(grid.origin_x)
        grid_msg.info.origin.position.y = float(grid.origin_y)
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = cost.ravel().tolist()
        self.cost_pub.publish(grid_msg)

        X, Y = grid.cell_centres()
        cells = np.zeros(int(valid.sum()), dtype=[(f, np.float32) for f in
                                                  ('x', 'y', 'z', 'slope', 'step', 'rough', 'cost', 'sigma')])
        for name, arr in (('x', X), ('y', Y), ('z', grid.height), ('slope', slope),
                          ('step', step), ('rough', rough), ('cost', cost), ('sigma', sigma)):
            cells[name] = arr[valid]
        fields = [PointField(name=f, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                  for k, f in enumerate(cells.dtype.names)]
        self.cells_pub.publish(point_cloud2.create_cloud(grid_msg.header, fields, cells))

        if self.dump_path:
            # Write then rename, so a reader never sees a half-written file
            tmp = self.dump_path + '.tmp.npz'
            np.savez(tmp, height=grid.height, slope=slope, step=step, rough=rough, sigma=sigma,
                     cost=cost, valid=valid, origin=(grid.origin_x, grid.origin_y),
                     res=grid.res)
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
    executor = MultiThreadedExecutor(num_threads=4)
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
