#!/usr/bin/env python3
"""
perception/aruco_detector.py
=============================
ROS 2 node: detects ArUco markers (DICT_4X4_50) in the ZED left rectified
image stream, estimates each marker's 6-DOF pose via cv2.solvePnP(),
publishes a MarkerArray, and shows a debug visualization window.

Scope: perception ONLY. No navigation, no cross-frame tracking, no
"visited marker" memory - those live in navigation/marker_tracker.py.

Design note: we do not synchronize Image/CameraInfo with message_filters.
The ZED intrinsics are effectively static for a fixed resolution, so we
simply cache the latest CameraInfo via CameraModel and use whatever is
current whenever an image arrives.
"""

import time
from typing import List, Optional, Tuple
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


# COMMON ERROR:
# If CameraInfo never arrives, check the ZED wrapper topic name AND that this
# node's subscriber QoS is compatible with the publisher's QoS - a RELIABLE
# subscriber can fail to match a BEST_EFFORT publisher under DDS QoS rules.

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError  # Converts sensor_msgs/Image <-> numpy BGR array.
from rover_aruco.common.camera_model import CameraModel
from rover_aruco_interfaces.msg import Marker, MarkerArray




class ArucoDetector(Node):
    """Detects ArUco markers, estimates pose via solvePnP, publishes
    MarkerArray, and renders a debug window. Perception only - no
    navigation, tracking, or marker memory."""

    ARUCO_DICTIONARY_ID: int = cv2.aruco.DICT_4X4_50
    DEFAULT_MARKER_SIZE_METERS: float = 0.175

    def __init__(self) -> None:
        super().__init__('aruco_detector')
        import cv2
        import sys
        print("=" * 60)
        print("Python executable :", sys.executable)
        print("cv2 location      :", cv2.__file__)
        print("OpenCV version    :", cv2.__version__)
        print("Has ArucoDetector :", hasattr(cv2.aruco, "ArucoDetector"))
        print("=" * 60)
# ROS parameters: using this declare_parameter we can edit from the launch file or the terminal itself
        self.declare_parameter('marker_size_meters', self.DEFAULT_MARKER_SIZE_METERS)
        self.declare_parameter('enable_debug_window', True)
        self.declare_parameter('image_topic', '/zed/image_raw')       

        self.declare_parameter('camera_info_topic', '/zed/camera_info')
        self.declare_parameter('output_topic', '/aruco/markers')
#We are making topic as a parameter cuz it is easy to modify if we have more than 1 camera
        self.declare_parameter('use_rectified_image',True)

        # self.marker_size: physical side length (metres) of the printed marker        
        self.marker_size: float = float(
            self.get_parameter('marker_size_meters').get_parameter_value().double_value
        )
        half = self.marker_size / 2.0
        self.object_points = np.array(
            [
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0]
            ],
            dtype=np.float64
        )
        self.enable_debug_window: bool = bool(
            self.get_parameter('enable_debug_window').get_parameter_value().bool_value
        )
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

#The next node for checking if the incoming node is rectified or raw (Application in estimate marker_pose function and draw_axes)
        self.use_rectified_image = bool(self.get_parameter('use_rectified_image').get_parameter_value().bool_value)

        self.camera_model: CameraModel = CameraModel()
        self.bridge: CvBridge = CvBridge()



        # Image: high rate, latest-frame-only, drops tolerated.
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
        )
        # CameraInfo: latched so a late subscriber still gets the last
        # known calibration immediately on connect.
        camera_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
        )
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, image_qos)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, camera_info_qos)

        # Output: reliable, since a planner/tracker downstream should not
        # silently miss a detection result.
        publisher_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, output_topic, publisher_qos)

        # Debug/HUD state. FPS bookkeeping is human-facing only - never
        # published, never used for ROS timing decisions.
        self._last_frame_time: Optional[float] = None
        self._fps: float = 0.0
        self._debug_window_name: str = 'ArUco Detector - Debug View'
        self._debug_window_created: bool = False
        self._shutdown_requested: bool = False

        self.aruco_dictionary, self.detector_params, self.detector = self.build_detector()

        self.get_logger().info(
            f'ArucoDetector started. image="{image_topic}" '
            f'camera_info="{camera_info_topic}" output="{output_topic}" '
            f'marker_size={self.marker_size:.4f} m debug_window={self.enable_debug_window}'
        )

    
    def build_detector(self) -> Tuple[cv2.aruco.Dictionary, cv2.aruco.DetectorParameters, cv2.aruco.ArucoDetector]:
        #Build the ArUco dictionary + params + detector (once, in __init__)
    
        dictionary = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICTIONARY_ID)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary,params,)

        
        return dictionary, params,detector


    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_model.update(msg)


    def image_callback(self, msg: Image) -> None:
        """Convert the incoming Image to a BGR numpy frame and hand it to
        process_frame(). Skips the frame entirely if calibration isn't
        ready yet, since solvePnP with a zero/garbage K would silently
        produce meaningless poses rather than erroring."""
        if not self.camera_model.is_ready():
            self.get_logger().warn(
                'CameraModel not ready (no CameraInfo yet) - skipping frame.',
                throttle_duration_sec=2.0)
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return
        self.process_frame(frame, msg.header)

    def rotation_to_euler(self, rotation_matrix: np.ndarray) -> Tuple[float, float, float]:
        """Convert a 3x3 rotation matrix to (roll, pitch, yaw) radians,
        using the standard Z-Y-X (yaw-pitch-roll) intrinsic convention.
        Used only for the HUD - the published quaternion is built directly
        from the matrix in rotation_matrix_to_quaternion(), avoiding a lossy
        matrix->Euler->quaternion round trip in the actual message data.

        sy = sqrt(r00^2 + r10^2) is the "cosine of pitch" magnitude; near
        zero it signals pitch ~ +/-90 deg (gimbal lock), where roll and yaw
        become coupled and we fall back to a single-angle formula.
        """
        r = rotation_matrix
        sy = float(np.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2))
        singular = sy < 1e-6
        if not singular:
            roll = float(np.arctan2(r[2, 1], r[2, 2]))
            pitch = float(np.arctan2(-r[2, 0], sy))
            yaw = float(np.arctan2(r[1, 0], r[0, 0]))
        else:
            roll = float(np.arctan2(-r[1, 2], r[1, 1]))
            pitch = float(np.arctan2(-r[2, 0], sy))
            yaw = 0.0
        return roll, pitch, yaw


    def rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> Quaternion:
        """Convert a 3x3 rotation matrix directly to geometry_msgs/Quaternion
        using the trace ("Shepperd's") method, chosen for numerical
        stability across all rotation magnitudes. This avoids routing
        through Euler angles for the actual published pose."""  
        m = rotation_matrix
        trace = m[0, 0] + m[1, 1] + m[2, 2]
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m[2, 1] - m[1, 2]) * s
            y = (m[0, 2] - m[2, 0]) * s
            z = (m[1, 0] - m[0, 1]) * s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


    def estimate_marker_pose(self, corners: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Estimate one marker's pose via cv2.solvePnP().

        solvePnP finds the rigid transform (rotation + translation) mapping
        a set of known 3D object points into the camera frame such that
        projecting them through the camera intrinsics reproduces the given
        2D image points. For a flat square marker, the 4 object points are
        its own corners (Z=0), centred on the marker's own origin:
            TL=(-h,h,0) TR=(h,h,0) BR=(h,-h,0) BL=(-h,-h,0), h=marker_size/2
        This ordering MUST match ArUco's returned corner order (TL, TR, BR,
        BL) or the pose silently comes out mirrored/rotated.

        cv2.SOLVEPNP_IPPE_SQUARE is used because it's specialised for
        exactly this "planar square target, 4 coplanar points" case
        (Infinitesimal Plane-based Pose Estimation) - faster and more
        robust here than the generic iterative solver.

        Because the input image is "rect_color" (already rectified by the
        ZED wrapper), the physically correct distortion for THIS image is
        zero - we pass zero distortion rather than camera_model.D to avoid
        double-correcting. camera_model.K is assumed to already be the
        rectified/projection intrinsics; if your CameraModel instead keeps K
        as the raw (pre-rectification) intrinsics, use camera_model.P here
        instead.

        Returns (rvec, tvec, rotation_matrix) in metres/radians, or None if
        solvePnP fails.

        COMMON ERROR:
        If solvePnP fails, verify: (1) marker_size is correct and non-zero,
        (2) corners reshapes cleanly to (4, 2), (3) camera_model.K/D are
        valid (only possible once CameraInfo has arrived - already guarded
        in image_callback()).
        """
        object_points = self.object_points
        image_points = corners.reshape(4, 2).astype(np.float64)
        camera_matrix = self.camera_model.K
        if self.use_rectified_image:
            dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        else:
            dist_coeffs = self.camera_model.D.reshape(-1, 1)    

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error as exc:
            self.get_logger().error(f'solvePnP raised an OpenCV error: {exc}')
            return None
        if not success:
            return None

        # Rodrigues: rvec's direction is the rotation axis, its magnitude
        # (radians) is the rotation angle about that axis; this expands it
        # into the equivalent 3x3 rotation matrix needed for Euler/quaternion.
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        return rvec, tvec, rotation_matrix


    def draw_axes(self, frame: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> None:

        axis_length = self.marker_size * 0.75
        if self.use_rectified_image:
            dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        else:
            dist_coeffs = self.camera_model.D.reshape(-1, 1)
        cv2.drawFrameAxes(frame, self.camera_model.K, dist_coeffs, rvec, tvec, axis_length)

    
    def draw_pose_text(self, frame: np.ndarray, marker_id: int, corners: np.ndarray,
                        tvec: np.ndarray, rotation_matrix: np.ndarray, distance: float) -> None:
        """Draw per-marker debug text (ID, distance, XYZ, RPY-in-degrees)
        anchored above the marker's top-left corner so it tracks the marker
        as it moves. distance is passed in (already computed once in
        process_frame()) rather than recomputed here."""
        roll, pitch, yaw = self.rotation_to_euler(rotation_matrix)
        roll_deg, pitch_deg, yaw_deg = np.degrees([roll, pitch, yaw])
        tx, ty, tz = tvec.flatten()

        anchor = corners.reshape(4, 2)[0]
        anchor_x, anchor_y = int(anchor[0]), int(anchor[1]) - 10

        lines = [
            f'ID: {marker_id}',
            f'dist: {distance:.3f} m',
            f'xyz: {tx:.2f}, {ty:.2f}, {tz:.2f} m',
            f'rpy: {roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f} deg',
        ]
        line_height = 16
        for i, line in enumerate(lines):
            y = anchor_y - (len(lines) - 1 - i) * line_height
            cv2.putText(frame, line, (anchor_x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 255), 1, cv2.LINE_AA)

    
    def draw_hud(self, frame: np.ndarray, marker_count: int) -> None:
        """Draw frame-level HUD: FPS, marker count, camera resolution, on a
        semi-opaque background box for legibility over a busy scene."""
        lines = [
            f'FPS: {self._fps:.1f}',
            f'Markers: {marker_count}',
            f'Resolution: {self.camera_model.width}x{self.camera_model.height}',
        ]
        overlay = frame.copy()
        box_h = 20 + 18 * len(lines)
        cv2.rectangle(overlay, (5, 5), (230, box_h), (0, 0, 0), thickness=-1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0.0, frame)
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (12, 25 + i * 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)

    
    def update_fps(self) -> None:
        """Exponential-moving-average FPS for the HUD, from time.time()
        deltas. Human-facing debug metric only - never published, and
        deliberately independent of the ROS clock / use_sim_time."""
        now = time.time()
        if self._last_frame_time is not None:
            dt = now - self._last_frame_time
            if dt > 0.0:
                instantaneous_fps = 1.0 / dt
                alpha = 0.1
                self._fps = alpha * instantaneous_fps + (1.0 - alpha) * self._fps
        self._last_frame_time = now

    # -------------------------------------------------------------------
    def process_frame(self, frame: np.ndarray, header: Header) -> None:
        """Per-frame pipeline: detect -> estimate pose per marker -> build
        and publish MarkerArray -> render debug view. header is the
        ORIGINAL Image message's header, reused verbatim on every Marker so
        downstream TF lookups use the correct frame_id/stamp, not this
        callback's wall-clock time.

        self.detector.detectMarkers(frame) returns (corners, ids, rejected):
        corners is a list of (1,4,2) arrays (one per marker, same order as
        ids); ids is (N,1) or None if nothing was found; rejected (unused
        here) holds marker-like candidates that failed dictionary decoding -
        useful for tuning detector_params if markers are being missed.
        """
        self.update_fps()
        corners, ids, rejected = self.detector.detectMarkers(frame)
        marker_array = MarkerArray()
        marker_array.markers = []
        marker_count = 0 if ids is None else len(ids)

        if ids is not None and marker_count > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(marker_count):
                marker_id = int(ids[i][0])
                marker_corners = corners[i]

                pose_result = self.estimate_marker_pose(marker_corners)
                if pose_result is None:
                    self.get_logger().warn(
                        f'solvePnP failed for marker id={marker_id}; skipping.')
                    continue
                rvec, tvec, rotation_matrix = pose_result
                distance = float(np.linalg.norm(tvec))

                marker_msg = Marker()
                marker_msg.header = header
                marker_msg.id = marker_id
                marker_msg.pose = Pose(
                    position=Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2])),
                    orientation=self.rotation_matrix_to_quaternion(rotation_matrix),
                )
                marker_msg.distance = distance
                marker_array.markers.append(marker_msg)

                if self.enable_debug_window:
                    self.draw_axes(frame, rvec, tvec)
                    self.draw_pose_text(frame, marker_id, marker_corners, tvec,
                                        rotation_matrix, distance)

        self.publish_markers(marker_array)

        if self.enable_debug_window:
            self.draw_hud(frame, marker_count)
            self._show_debug_window(frame)


    def publish_markers(self, marker_array: MarkerArray) -> None:
        """Publish the MarkerArray, even when empty - publishing zero
        markers on a no-detection frame lets consumers tell "nothing seen
        this frame" apart from "this node stopped publishing"."""
        self.marker_pub.publish(marker_array)


    def _show_debug_window(self, frame: np.ndarray) -> None:
        #Press q to exit
        if not self._debug_window_created:
            cv2.namedWindow(self._debug_window_name, cv2.WINDOW_NORMAL)
            self._debug_window_created = True
        cv2.imshow(self._debug_window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("'q' pressed - requesting shutdown.")
            # Flag rather than call rclpy.shutdown() directly here, to avoid
            # tearing down the executor recursively mid-callback; main()'s
            # spin loop checks this flag and exits cleanly instead.
            self._shutdown_requested = True


    def destroy_node(self) -> None:

        if self._debug_window_created:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._shutdown_requested:
                break
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received - shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()