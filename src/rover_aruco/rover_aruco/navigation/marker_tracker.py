#!/usr/bin/env python3
"""
navigation/marker_tracker.py
=============================
ROS 2 node: consumes /aruco/markers (published by perception/aruco_detector.py)
and maintains a persistent, time-aware database of markers - which exist,
which are currently visible, which are stable enough to trust, which have
already been visited, and which one is the current navigation target.

Scope: tracking and marker MANAGEMENT only. No camera, no OpenCV, no
re-derived poses - this node consumes Marker/MarkerArray exactly as
aruco_detector.py publishes them and would be unchanged if the perception
node were swapped for a different detector publishing the same messages.

Why a database instead of just relaying the latest MarkerArray: a single
message only tells you what THIS frame saw. Driving toward a marker needs
state that persists ACROSS messages - that a marker isn't a one-frame false
positive (stability), that a marker gone for 2 frames isn't gone forever
(lost-frame tolerance vs. immediate deletion), and that an already-visited
marker should not be reselected (visited state). That's exactly what this
node owns.

Downstream consumer: navigation/drive_auto_v6.py reads /current_target (and
optionally /tracked_markers) to decide where to drive - all eligibility
bookkeeping happens here, so drive_auto_v6.py stays a pure "steer toward
whatever pose is on /current_target" consumer.
"""

import time
import threading
# threading.Lock guards self._database. This node already has two
# independent timing sources touching it (marker_callback, driven by
# message arrival, and the maintenance timer, driven by wall-clock period).
# rclpy's default single-threaded executor never runs them concurrently, but
# locking now means switching to a MultiThreadedExecutor or adding a service
# later doesn't silently become unsafe.

from dataclasses import dataclass, replace
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import Pose
from rover_aruco_interfaces.msg import Marker, MarkerArray
# Reused EXACTLY as published by aruco_detector.py - no redefinition.

@dataclass
class MarkerInfo:
    """One database row: everything known about a single marker ID,
    independent of any single incoming message. A plain dataclass (not a
    dict/namedtuple) because it's mutated field-by-field on every refresh
    and documents its shape in one place instead of scattering key names as
    string literals across the file."""
    id: int
    pose: Pose
    distance: float
    header: Header

    first_seen_time: float
    last_seen_time: float           #changed by remove_stale_markers() later
    times_seen: int = 1

    visited: bool = False
    visible: bool = True
    lost_count: int = 0
    stable: bool = False


class MarkerTracker(Node):
    """Owns the marker database. Subscribes to /aruco/markers, maintains
    per-marker state across frames, and publishes the currently selected
    navigation target (and optionally a snapshot of tracked markers)."""

    def __init__(self) -> None:
        super().__init__('marker_tracker')

        self.declare_parameter('marker_topic', '/aruco/markers')   #listen
        self.declare_parameter('stale_timeout', 10.0)
        self.declare_parameter('stable_detection_count', 5)
        self.declare_parameter('max_lost_frames', 15)
        self.declare_parameter('publish_target', True)
        self.declare_parameter('publish_database', True)
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('maintenance_period_sec', 0.5)

        self.marker_topic: str = self.get_parameter(
            'marker_topic').get_parameter_value().string_value
        self.stale_timeout: float = float(self.get_parameter(
            'stale_timeout').get_parameter_value().double_value)
        self.stable_detection_count: int = int(self.get_parameter(
            'stable_detection_count').get_parameter_value().integer_value)
        self.max_lost_frames: int = int(self.get_parameter(
            'max_lost_frames').get_parameter_value().integer_value)
        self.publish_target_enabled: bool = bool(self.get_parameter(
            'publish_target').get_parameter_value().bool_value)
        self.publish_database_enabled: bool = bool(self.get_parameter(
            'publish_database').get_parameter_value().bool_value)
        self.publish_visualization_enabled: bool = bool(self.get_parameter(
            'publish_visualization').get_parameter_value().bool_value)
        maintenance_period: float = float(self.get_parameter(
            'maintenance_period_sec').get_parameter_value().double_value)

        # Dict keyed by marker_id -> MarkerInfo: O(1) lookup/insert/update,
        # the only access pattern this node needs. Every public method
        # below either looks up by ID or scans the small, bounded set of
        # known markers - nothing here is O(n^2).
        self._database: Dict[int, MarkerInfo] = {}   #Underscore means only internal implementation other functions will use a copy of the database
        self._lock = threading.Lock()

        # Selected target stored as an ID (not a MarkerInfo copy) so it
        # always resolves against the live entry, never a stale snapshot.
        self._target_id: Optional[int] = None
        self._last_search_mode_log_time: float = 0.0

        # QoS: match aruco_detector.py's publisher (RELIABLE, KEEP_LAST) -
        # left implicit/default here can silently fail to connect under DDS
        # QoS compatibility rules if it doesn't match.
        input_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        output_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.marker_sub = self.create_subscription(
            MarkerArray, self.marker_topic, self.marker_callback, input_qos)

        # Feedback loop: movement.py (via goal_generator.py's forwarded
        # /current_target_id) knows which marker it just finished driving
        # to and stopping at. It reports that back here so mark_visited()
        # - previously never called by anything in this stack - actually
        # gets invoked, letting select_next_target() move on to the next
        # marker instead of re-selecting the same one forever.
        self.create_subscription(Int32, '/marker_reached', self.marker_reached_cb, 10)

        # /current_target reuses the SAME Marker type aruco_detector.py
        # publishes - drive_auto_v6.py needs no new message type. Always
        # created regardless of the publish_target parameter so a 
        # runtime parameter change doesn't require a publisher rebuild.
        self.target_pub = self.create_publisher(Marker, '/current_target', output_qos)
        self.tracked_pub = self.create_publisher(MarkerArray, '/tracked_markers', output_qos)

        # Stale removal must NOT depend on marker_callback firing - it
        # guards against the detector going silent entirely (crash, camera
        # disconnect), which is exactly when the callback stops running. A
        # wall-clock timer is the only way to detect "time passed with
        # nothing arriving at all".
        self._maintenance_timer = self.create_timer(
            maintenance_period, self._maintenance_tick)

        self.get_logger().info(
            f'MarkerTracker started. marker_topic="{self.marker_topic}" '
            f'stale_timeout={self.stale_timeout}s '
            f'stable_detection_count={self.stable_detection_count} '
            f'max_lost_frames={self.max_lost_frames} '
            f'publish_target={self.publish_target_enabled} '
            f'publish_database={self.publish_database_enabled} '
            f'publish_visualization={self.publish_visualization_enabled}')

    # =====================================================================
    # Phase 1: ingest
    # =====================================================================
    def marker_callback(self, msg: MarkerArray) -> None:
        """WHAT: validates each incoming marker, upserts it, then
        reconciles the rest of the database against "seen this frame"
        (incrementing lost_count for anyone absent), re-selects the
        target, and publishes.

        WHY: the only place new information enters the database - every
        other method here reads it or is driven by wall-clock time.

        DESIGN: an empty MarkerArray is still meaningful ("camera saw
        nothing") and still runs the full reconciliation pass - a marker
        that's truly out of frame must accumulate lost_count on those
        frames too, which only happens if this callback's full logic runs
        even when msg.markers is empty.
        """
        seen_ids: List[int] = []

        with self._lock:
            for marker in msg.markers:
                if not self._validate_marker(marker):
                    continue
                if marker.id in seen_ids:
                    # Duplicate ID within one message: shouldn't happen
                    # from the detector, but a corrupted/merged message
                    # could produce it. Keep the LAST occurrence, just warn.
                    self.get_logger().warn(
                        f'Duplicate marker id={marker.id} within one '
                        f'MarkerArray - keeping the later entry.',
                        throttle_duration_sec=5.0)
                else:
                    seen_ids.append(marker.id)
                self.update_database(marker)

            self.update_lost_counters(seen_ids)
            self.select_next_target()

        # Outside the lock: publish() doesn't touch self._database, so
        # holding the lock across it would widen the critical section for
        # no safety benefit.
        if self.publish_database_enabled:
            self._publish_tracked_markers()
        if self.publish_target_enabled:
            self.publish_target()
        if self.publish_visualization_enabled:
            self._log_visualization()

    def marker_reached_cb(self, msg: Int32) -> None:
        """WHAT: callback for /marker_reached - marks the given marker id
        visited via the existing mark_visited() API.
        WHY: this is the missing wiring - mark_visited() was documented as
        "expected to be called by [a downstream consumer] on arrival" but
        nothing in the stack actually called it, so no marker ever left
        the eligible-candidate pool and the same target kept being
        re-selected indefinitely.
        """
        marker_id = int(msg.data)
        if not self.mark_visited(marker_id):
            self.get_logger().warn(
                f'/marker_reached for id={marker_id}, but it is not in the '
                f'database (already stale/removed?).')

    def _validate_marker(self, marker: Marker) -> bool:
        """WHAT: rejects a marker with a negative distance (physically
        meaningless for a camera-to-marker range).

        WHY: aruco_detector.py is trusted, but this node's correctness
        (especially distance comparisons in select_next_target()) must
        not silently corrupt itself if a future detector version, a
        replayed bag, or a hand-built test message sends bad data.

        DESIGN: an empty header.frame_id is NOT a hard rejection - it
        doesn't break any comparison this node performs (no TF lookups
        here), so a throttled warning is more useful than dropping
        otherwise-good pose/distance data.
        """
        if marker.distance < 0.0:
            self.get_logger().warn(
                f'Rejecting marker id={marker.id}: negative distance '
                f'({marker.distance:.3f}).', throttle_duration_sec=5.0)
            return False
        if not marker.header.frame_id:
            self.get_logger().warn(
                f'Marker id={marker.id} has an empty frame_id - accepting '
                f'anyway (no TF lookups here), but check the detector.',
                throttle_duration_sec=10.0)
        return True

    def update_database(self, marker: Marker) -> None:
        """WHAT: inserts a new marker ID, or refreshes an existing entry's
        pose/distance/header/timestamps/counters. The single upsert path -
        every other method that changes this data goes through here.

        WHY: one place defines what "seeing a marker again" means, instead
        of duplicating that logic between separate insert/update branches.

        DESIGN: `visited` is NEVER touched here, insert or update. This is
        the mechanism behind "never lose visited state" - a marker that
        temporarily leaves frame and reappears stays visited; only
        clear_visited() (an explicit, separate operation) can undo it.
        """
        now = time.time()
        existing = self._database.get(marker.id)

        if existing is None:
            self._database[marker.id] = MarkerInfo(
                id=marker.id, pose=marker.pose, distance=marker.distance,
                header=marker.header, first_seen_time=now, last_seen_time=now,
                times_seen=1, visited=False, visible=True, lost_count=0,
                stable=(1 >= self.stable_detection_count))
            self.get_logger().info(f'New marker discovered: id={marker.id}')
            return

        existing.pose = marker.pose
        existing.distance = marker.distance
        existing.header = marker.header
        existing.last_seen_time = now
        existing.times_seen += 1
        existing.visible = True
        existing.lost_count = 0
        self.update_stability(marker.id)

    def update_stability(self, marker_id: int) -> None:
        """WHAT: promotes a marker to stable=True after
        stable_detection_count consecutive refreshes.

        WHY: a single-frame ArUco read can be a false positive (motion
        blur, reflection, partial occlusion). Requiring several
        confirmations before a marker is eligible as a target filters
        these out.

        DESIGN: stability is monotonic - never reset False here. A marker
        that briefly drops and returns keeps its times_seen (incremented,
        not reset, in update_database()), so stability answers "have we
        confirmed this exists", not "is it in an unbroken streak right
        now" - that's `visible`'s job.
        """
        info = self._database.get(marker_id)
        if info is None:
            return
        if not info.stable and info.times_seen >= self.stable_detection_count:
            info.stable = True
            self.get_logger().info(
                f'Marker id={marker_id} promoted to STABLE after '
                f'{info.times_seen} detections.')

    def update_lost_counters(self, seen_ids: List[int]) -> None:
        """WHAT: for every entry NOT in this frame's seen_ids, increments
        lost_count; past max_lost_frames, flips visible False. Never
        deletes - deletion is stale_timeout's job alone (remove_stale_markers()).

        WHY: a marker briefly leaving frame (a turn, brief occlusion) is
        normal and shouldn't mean "gone", but it should stop being a
        target candidate the moment it's not reliably trackable - flipping
        `visible` achieves that without discarding history.

        DESIGN: lost_count resets to 0 the instant the marker reappears
        (in update_database()), so it measures a CONSECUTIVE absence
        streak, not a lifetime miss count - a marker seen on and off all
        session is judged only on its current streak, which is what
        actually matters for "trackable right now".
        """
        for marker_id, info in self._database.items():
            if marker_id in seen_ids:
                continue
            info.lost_count += 1
            if info.visible and info.lost_count > self.max_lost_frames:
                info.visible = False
                self.get_logger().info(
                    f'Marker id={marker_id} marked NOT VISIBLE '
                    f'(lost_count={info.lost_count} > max_lost_frames='
                    f'{self.max_lost_frames}). Still tracked; purged only '
                    f'after {self.stale_timeout}s of silence.')

    def update_visibility(self, marker_id: int) -> Optional[str]:
        """WHAT: derives a human-readable status - 'visible',
        'temporarily_lost', or 'removed' - for one marker.

        WHY: `visible` plus presence-in-database already encode this
        tri-state; centralising the mapping avoids duplicating the
        if/elif logic across every caller (logging, a future debug
        service).

        DESIGN: pure and read-only - never mutates lost_count/visible
        itself (that lives in update_lost_counters()/
        remove_stale_markers()), so calling it repeatedly is always safe.
        """
        info = self._database.get(marker_id)
        if info is None:
            return 'removed'
        return 'visible' if info.visible else 'temporarily_lost'

    # =====================================================================
    # Phase 1: maintenance (time-driven, not message-driven)
    # =====================================================================
    def _maintenance_tick(self) -> None:
        #Answers when a cleanup should happen irrespective of messages
        """WHAT: periodic timer callback that removes stale markers.

        WHY: the only mechanism that notices the detector has stopped
        publishing entirely - marker_callback only runs in response to a
        message, which by definition isn't arriving in that failure case.

        DESIGN: deliberately tiny, delegating straight to
        remove_stale_markers() - keeps "run periodically" separate from
        the removal POLICY, which stays testable/callable on its own.
        """
        with self._lock:
            self.remove_stale_markers()

    def remove_stale_markers(self) -> None:
        #Delete markers from the data base that haven't been seen for too long.
        """WHAT: deletes any marker whose last_seen_time exceeds
        stale_timeout seconds ago. The ONLY method that deletes entries.

        WHY: without eventual deletion the database grows forever across
        a long session (every marker ever glimpsed lingers indefinitely).

        DESIGN: deletion is keyed on wall-clock time since last_seen_time,
        decoupled from visible/lost_count (a FRAME count). This makes
        max_lost_frames and stale_timeout independently tunable: mark a
        marker "not currently visible" quickly, while giving it a much
        longer real-world grace period before forgetting it entirely -
        useful since a marker lost mid-turn is far more likely to
        reappear than one unseen for many seconds.
        """
        #Change made...never deletes the visited markers 
        now = time.time()
        stale_ids = [
            marker_id for marker_id, info in self._database.items()
            if not info.visited and (now - info.last_seen_time) > self.stale_timeout]
        for marker_id in stale_ids:
            del self._database[marker_id]
            self.get_logger().info(
                f'Unvisited marker id={marker_id} purged from database (stale for > {self.stale_timeout}s).')
            if self._target_id == marker_id:
                # Don't pick a replacement here - select_next_target()
                # already runs on every marker_callback and will pick a
                # new target (or correctly report none) next cycle;
                # duplicating that logic here could act on conditions
                # that were true a maintenance-period ago, not "now".
                self._target_id = None

    # =====================================================================
    # Phase 1: read-only query API
    # =====================================================================
    def get_marker(self, marker_id: int) -> Optional[MarkerInfo]:   #Optional to return none if id which is not present in the data base is called
        """WHAT: look up one marker by ID.
        WHY: the most basic query a downstream consumer needs.
        DESIGN: returns a COPY (dataclasses.replace), not the live
        reference - a caller can't mutate internal state by holding the
        returned object; all mutation must go through this class's own
        methods, which is what keeps `visited` reliably one-way.
        """
        with self._lock:
            info = self._database.get(marker_id)   #Using get to avoid exceptions
            return replace(info) if info is not None else None

    def get_all_visible_markers(self) -> List[MarkerInfo]:
        #returnns copies of every marker currently flagged visible.
        with self._lock:
            return [replace(info) for info in self._database.values() if info.visible]
# replace: It creates a new dataclass object with the same field values. (we can modify what fields we want)


    def get_nearest_marker(self) -> Optional[MarkerInfo]:
        #returns a copy of the closest currently-visible marker, or None.
        with self._lock:
            visible = [info for info in self._database.values() if info.visible]
            if not visible:
                return None
            return replace(min(visible, key=lambda info: info.distance))

    def get_database_snapshot(self) -> Dict[int, MarkerInfo]:
        #returns a copy of the ENTIRE database (visible or not).

        with self._lock:
            return {mid: replace(info) for mid, info in self._database.items()}

    def get_unvisited_markers(self) -> List[MarkerInfo]:
        #returns the copies of every marker (any visibility) not yet visited.

        with self._lock:
            return [replace(info) for info in self._database.values() if not info.visited]

    # =====================================================================
    # Phase 2: visited-state management
    # =====================================================================
    def mark_visited(self, marker_id: int) -> bool:
        """WHAT: flags a marker visited. Returns True if it existed.
        WHY: the ONLY place `visited` is ever set True - a single call
        site to reason about "what counts as reaching a marker" (expected
        to be called by drive_auto_v6.py on arrival).
        DESIGN: does nothing (returns False) if the marker is unknown,
        rather than raising - marking a marker visited right as it goes
        stale is a race that should degrade gracefully, not crash a
        navigation-critical node.
        """
        #Updated mark_visited() so that if /marker_reached arrives for an ID that isn't currently 
        #in self._database, it creates a permanent stub record marked visited=True instead of 
        #returning False:
        with self._lock:
            info = self._database.get(marker_id)
            if info is None:
                # CHANGED: Create a permanent stub record if not present in DB
                nowx=time.time()
                self._database[marker_id] = MarkerInfo(
                    id=marker_id, pose=Pose(), distance=0.0, header=Header(),
                    first_seen_time=nowx, last_seen_time=nowx, times_seen=1,
                    visited=True, visible=False, lost_count=999, stable=True
                )
                self.get_logger().info(f'Marker id={marker_id} created as PERMANENTLY VISITED in DB.')
                return False
            info.visited = True
            self.get_logger().info(f'Marker id={marker_id} marked VISITED.')
            if self._target_id == marker_id:
                self.select_next_target()
            return True

    def clear_visited(self) -> None:
        """WHAT: resets visited=False on every marker in the database.
        WHY: supports re-running a full visiting mission (e.g. a second
        lap) without restarting the node and losing stability/timing
        history for markers still in view.
        DESIGN: an explicit, rarely-called operation (an operator or
        mission-restart trigger), never automatic - visited state should
        only change because a "new lap" was deliberately declared.
        """        
    
        with self._lock:
            info 
            for info in self._database.values():
                info.visited = False
            self.select_next_target()
        self.get_logger().info('All markers reset to UNVISITED.')

    # =====================================================================
    # Phase 2: target selection
    # =====================================================================
    def select_next_target(self) -> Optional[int]:
        """WHAT: (re-)selects the target from the database and returns its
        ID (or None). Must be called with self._lock already held (it's
        invoked from marker_callback(), mark_visited(), clear_visited()).

        WHY: the single place "which marker to drive toward" is decided -
        drive_auto_v6.py just reads whatever this last published.

        DESIGN: eligibility is three independent filters - visible,
        stable, not visited - rather than one combined boolean, since each
        answers a different question and keeping them separate makes it
        obvious which condition disqualifies a marker. Among eligible
        candidates, strict nearest-distance selection: simple, predictable
        policy - heading alignment/path cost belong in drive_auto_v6.py,
        not this marker-management layer.
        """ 
        candidates = [
            info for info in self._database.values()
            if info.visible and info.stable and not info.visited]
        if not candidates:
            self._target_id = None
            self._log_search_mode_if_needed()
            return None
        self._target_id = min(candidates, key=lambda info: info.distance).id
        return self._target_id

    def search_mode(self) -> bool:
#Answers the question: Is there a valid target right now? After select target returns an ID, this func 
#returns false saying don't search anymore move to the ID marker
        """WHAT: True when no stable, unvisited, visible marker exists to
        target - the rover has nothing to drive toward.
        WHY: drive_auto_v6.py needs a clean boolean to fall back to a
        search/explore behaviour without re-deriving eligibility rules.
        DESIGN: `self._target_id is None`, not a fresh database scan -
        select_next_target() already recomputes _target_id on every
        relevant change; re-deriving here could drift from what was
        actually published to /current_target.
        """
        with self._lock:
            return self._target_id is None

    def _log_search_mode_if_needed(self) -> None:

        now = time.time()
        if now - self._last_search_mode_log_time > 5.0:
            self.get_logger().info(
                'No eligible target (search_mode=True): no visible, '
                'stable, unvisited marker in the database.')
            self._last_search_mode_log_time = now

    # =====================================================================
    # Publishing
    # =====================================================================
    def publish_target(self) -> None:
        """WHAT: publishes the selected target as a Marker on
        /current_target, using the exact type aruco_detector.py publishes.

        WHY: the node's primary output - the whole point of the database
        is answering "where do I drive next".

        DESIGN: when no eligible target exists, still publishes a Marker
        with id=-1, distance=-1.0 as an explicit "no target" sentinel,
        rather than skipping the publish. A consumer relying on "topic
        stopped updating" can't distinguish that from this node crashing;
        a continuously-published sentinel lets drive_auto_v6.py (or
        search_mode()) tell the two apart.
        """
        with self._lock:
            info = self._database.get(self._target_id) if self._target_id is not None else None
            if info is None:
                sentinel = Marker()
                sentinel.header = Header()
                sentinel.header.stamp = self.get_clock().now().to_msg()
                sentinel.id = -1
                sentinel.distance = -1.0
                self.target_pub.publish(sentinel)
                return

            target_msg = Marker()
            target_msg.header = info.header
            target_msg.id = info.id
            target_msg.pose = info.pose
            target_msg.distance = info.distance
        self.target_pub.publish(target_msg)

    def _publish_tracked_markers(self) -> None:
        """WHAT: publishes every currently-visible marker as a MarkerArray
        on /tracked_markers.

        WHY: gives an operator (RViz, `ros2 topic echo`, a dashboard)
        visibility into live tracker state without a custom service.

        DESIGN: publishes only `visible` markers, not the full database -
        Marker.msg has no field to carry visibility status, so including
        temporarily-lost/stale-adjacent entries would present stale poses
        with no way for a consumer to tell. get_database_snapshot() covers
        the full-history in-process use case instead.
        """
        with self._lock:
            array_msg = MarkerArray()
            array_msg.markers = []
            for info in self._database.values():
                if not info.visible:
                    continue
                m = Marker()
                m.header = info.header
                m.id = info.id
                m.pose = info.pose
                m.distance = info.distance
                array_msg.markers.append(m)
        self.tracked_pub.publish(array_msg)

    def _log_visualization(self) -> None:
        """WHAT: one human-readable info log summarising the database
        (id, distance, stable/visible/visited, lost_count) per callback.

        WHY: this node has no camera image to draw an overlay onto (all
        HUD rendering lives in aruco_detector.py) - a concise textual
        snapshot is the equivalent an operator can watch scroll by.

        DESIGN: one multi-line f-string logged once per callback, not once
        per marker - avoids flooding the log with a line per marker per
        frame; the whole table is one throttleable log record.
        """
        with self._lock:
            if not self._database:
                summary = '  (no markers tracked)'
            else:
                rows = []
                for info in sorted(self._database.values(), key=lambda i: i.id):
                    rows.append(
                        f'  id={info.id:<3} dist={info.distance:6.2f}m '
                        f'seen={info.times_seen:<4} lost={info.lost_count:<3} '
                        f'stable={str(info.stable):<5} visible={str(info.visible):<5} '
                        f'visited={info.visited}'
                        + ('  <== TARGET' if info.id == self._target_id else ''))
                summary = '\n'.join(rows)
            target_id = self._target_id

        self.get_logger().info(
            f'--- Marker database (target={target_id}) ---\n{summary}',
            throttle_duration_sec=1.0)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MarkerTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received - shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()