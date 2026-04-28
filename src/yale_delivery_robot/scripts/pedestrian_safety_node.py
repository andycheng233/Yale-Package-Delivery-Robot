#!/usr/bin/env python3
"""
Yale Delivery Robot - Pedestrian Safety Node
=============================================
Pipeline:
  1. YOLOv8n  — detect persons in RGB image
  2. Depth cam — project 2D bounding-box centre → 3D point (camera frame)
  3. Kalman    — track each person, estimate 3D velocity vector
  4. Threat    — if velocity vector aims at robot AND person is close → FREEZE
  5. Freeze    — publish zero Twist to /cmd_vel_safety (multiplexed with Nav2)

ROS topics consumed:
  /camera/image_raw           sensor_msgs/Image   (RGB)
  /camera/depth/image_raw     sensor_msgs/Image   (float32, metres)
  /camera/camera_info         sensor_msgs/CameraInfo

ROS topics published:
  /pedestrian_detections      std_msgs/String     (JSON status, for UI/logging)
  /cmd_vel_safety             geometry_msgs/TwistStamped  (zero when frozen)
  /pedestrian_markers         visualization_msgs/MarkerArray  (RViz)

Install deps once:
  pip3 install ultralytics --break-system-packages
  ros2 run yale_delivery_robot pedestrian_safety_node
"""

import json
import time
import threading
import numpy as np
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


# ── Config (tune these for your environment) ──────────────────────────────────

YOLO_MODEL       = 'yolov8n.pt'    # nano = fastest; swap for yolov8s.pt for accuracy
YOLO_CONF        = 0.45            # detection confidence threshold
YOLO_DEVICE      = 'cpu'          # 'cpu' or '0' for GPU (your P1000 should handle nano)

# Tracking
MAX_TRACK_AGE    = 1.5   # seconds before dropping a lost track
MIN_DETECTIONS   = 3     # min hits before a track is considered real (reduces false positives)

# Threat thresholds
THREAT_DIST_M    = 2.5   # metres — persons closer than this are evaluated
APPROACH_SPEED   = 0.3   # m/s toward robot — triggers freeze
FREEZE_HOLD_S    = 1.5   # seconds to stay frozen after threat clears (hysteresis)

# Depth sampling
DEPTH_SAMPLE_R   = 5     # pixel radius around BB centre to median-sample depth

# Camera intrinsics (overridden by /camera/camera_info if available)
DEFAULT_FX = 554.0
DEFAULT_FY = 554.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0


# ── Kalman Filter (constant-velocity, 3D) ─────────────────────────────────────
class KalmanTracker3D:
    """
    State: [x, y, z, vx, vy, vz]   (position + velocity in camera frame, metres)
    Measurement: [x, y, z]

    Process noise Q: tuned for pedestrian dynamics (~1.5 m/s max accel)
    Measurement noise R: tuned for depth camera noise (~0.05 m std)
    """

    def __init__(self, x: float, y: float, z: float):
        # State vector
        self.x = np.array([x, y, z, 0.0, 0.0, 0.0], dtype=np.float64)

        # State covariance
        self.P = np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0])

        # Measurement matrix (we only observe x,y,z not velocities)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # Process noise
        q_pos = 0.05
        q_vel = 0.5
        self.Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])

        # Measurement noise (depth camera ~5cm std)
        self.R = np.diag([0.05**2, 0.05**2, 0.10**2])

        self.last_update = time.monotonic()
        self.hit_count   = 0

    def predict(self) -> None:
        """Extrapolate state forward by actual elapsed time."""
        now = time.monotonic()
        dt  = now - self.last_update
        dt  = min(dt, 0.5)   # cap at 0.5s to avoid explosion

        # Transition matrix
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, x: float, y: float, z: float) -> None:
        """Incorporate a new measurement."""
        z_meas = np.array([x, y, z])
        y_innov = z_meas - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y_innov
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.last_update = time.monotonic()
        self.hit_count  += 1

    @property
    def position(self) -> np.ndarray:
        return self.x[:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:]

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.x[3:]))

    @property
    def age(self) -> float:
        return time.monotonic() - self.last_update


# ── Track dataclass ────────────────────────────────────────────────────────────
@dataclass
class PersonTrack:
    track_id:  int
    kalman:    KalmanTracker3D
    bbox:      tuple = (0, 0, 0, 0)    # x1,y1,x2,y2 in image
    is_threat: bool  = False

    @property
    def confirmed(self) -> bool:
        return self.kalman.hit_count >= MIN_DETECTIONS


# ── Main node ─────────────────────────────────────────────────────────────────
class PedestrianSafetyNode(Node):

    def __init__(self):
        super().__init__('pedestrian_safety_node')

        # ── Load YOLO ─────────────────────────────────────────────────────────
        try:
            from ultralytics import YOLO
            self.yolo = YOLO(YOLO_MODEL)
            self.yolo_available = True
            self.get_logger().info(f'YOLOv8 loaded: {YOLO_MODEL} on {YOLO_DEVICE}')
        except ImportError:
            self.get_logger().error(
                'ultralytics not installed! '
                'Run: pip3 install ultralytics --break-system-packages')
            self.yolo_available = False
        except Exception as e:
            self.get_logger().error(f'YOLO load failed: {e}')
            self.yolo_available = False

        # ── Camera intrinsics ─────────────────────────────────────────────────
        self.fx = DEFAULT_FX
        self.fy = DEFAULT_FY
        self.cx = DEFAULT_CX
        self.cy = DEFAULT_CY
        self.camera_info_received = False

        # ── State ─────────────────────────────────────────────────────────────
        self.bridge       = CvBridge()
        self.depth_image  = None
        self.depth_lock   = threading.Lock()
        self.tracks:      Dict[int, PersonTrack] = {}
        self.next_id      = 0
        self.frozen       = False
        self.freeze_until = 0.0   # monotonic time to stay frozen until

        # Processing rate limiting (don't run YOLO on every frame)
        self.last_detect_time = 0.0
        self.detect_interval  = 0.1   # 10 Hz detection

        # ── Subscribers ───────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.create_subscription(
            Image, '/camera/image_raw', self._rgb_cb, best_effort_qos)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_cb, best_effort_qos)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._freeze_pub = self.create_publisher(
            TwistStamped, '/cmd_vel_safety', 10)
        self._status_pub = self.create_publisher(
            String, '/pedestrian_detections', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/pedestrian_markers', 10)

        # ── Timer: publish freeze command + status at 10 Hz ──────────────────
        self.create_timer(0.1, self._safety_timer_cb)

        self.get_logger().info('PedestrianSafetyNode ready')
        self.get_logger().info(
            f'Threat zone: {THREAT_DIST_M}m, '
            f'approach threshold: {APPROACH_SPEED} m/s')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _depth_cb(self, msg: Image):
        """Store latest depth image (float32, metres)."""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.depth_lock:
                self.depth_image = depth.copy()
        except Exception as e:
            self.get_logger().warn(f'Depth conversion error: {e}', throttle_duration_sec=5.0)

    def _rgb_cb(self, msg: Image):
        """Run detection pipeline on RGB frames."""
        if not self.yolo_available:
            return

        now = time.monotonic()
        if (now - self.last_detect_time) < self.detect_interval:
            return
        self.last_detect_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'RGB conversion error: {e}', throttle_duration_sec=5.0)
            return

        with self.depth_lock:
            depth = self.depth_image.copy() if self.depth_image is not None else None

        if depth is None:
            return

        self._run_detection(frame, depth)

    # ── Detection pipeline ────────────────────────────────────────────────────
    def _run_detection(self, frame: np.ndarray, depth: np.ndarray):
        """
        1. Run YOLO to get person bounding boxes
        2. Project each BB centre to 3D using depth
        3. Associate detections with existing tracks (nearest-neighbour)
        4. Update Kalman filters
        5. Assess threats
        6. Publish markers
        """
        h, w = frame.shape[:2]

        # ── YOLO detection ────────────────────────────────────────────────────
        results = self.yolo(
            frame,
            classes=[0],          # class 0 = person
            conf=YOLO_CONF,
            device=YOLO_DEVICE,
            verbose=False
        )

        detections = []   # list of (cx_px, cy_px, x3d, y3d, z3d, bbox)
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                cx_px = (x1 + x2) // 2
                cy_px = (y1 + y2) // 2

                # ── Depth sampling (median in a small window) ─────────────────
                r  = DEPTH_SAMPLE_R
                y_lo = max(0, cy_px - r);  y_hi = min(h, cy_px + r + 1)
                x_lo = max(0, cx_px - r);  x_hi = min(w, cx_px + r + 1)
                patch = depth[y_lo:y_hi, x_lo:x_hi]
                valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 20.0)]

                if valid.size < 4:
                    continue   # no reliable depth reading

                z3d = float(np.median(valid))

                # ── Back-project to 3D (camera frame: x=right, y=down, z=fwd) ─
                x3d = (cx_px - self.cx) * z3d / self.fx
                y3d = (cy_px - self.cy) * z3d / self.fy

                detections.append((cx_px, cy_px, x3d, y3d, z3d,
                                    (x1, y1, x2, y2)))

        # ── Track association (greedy nearest-neighbour by 3D distance) ───────
        matched_ids = set()

        for (cx_px, cy_px, x3d, y3d, z3d, bbox) in detections:
            meas = np.array([x3d, y3d, z3d])
            best_id   = None
            best_dist = 1.5   # metres — max association distance

            for tid, track in self.tracks.items():
                if tid in matched_ids:
                    continue
                track.kalman.predict()
                dist = float(np.linalg.norm(track.kalman.position - meas))
                if dist < best_dist:
                    best_dist = dist
                    best_id   = tid

            if best_id is not None:
                self.tracks[best_id].kalman.update(x3d, y3d, z3d)
                self.tracks[best_id].bbox = bbox
                matched_ids.add(best_id)
            else:
                # New track
                new_id = self.next_id
                self.next_id += 1
                kf = KalmanTracker3D(x3d, y3d, z3d)
                self.tracks[new_id] = PersonTrack(
                    track_id=new_id, kalman=kf, bbox=bbox)
                matched_ids.add(new_id)

        # ── Prune stale tracks ────────────────────────────────────────────────
        self.tracks = {
            tid: t for tid, t in self.tracks.items()
            if t.kalman.age < MAX_TRACK_AGE
        }

        # ── Threat assessment ─────────────────────────────────────────────────
        any_threat = False
        threat_info = []

        for tid, track in self.tracks.items():
            if not track.confirmed:
                track.is_threat = False
                continue

            pos = track.kalman.position   # [x, y, z] in camera frame
            vel = track.kalman.velocity   # [vx, vy, vz]

            # Horizontal distance (xz plane — ignore vertical y)
            dist_xz = float(np.sqrt(pos[0]**2 + pos[2]**2))

            # Approach speed = component of velocity toward camera (negative z = toward robot)
            # vz < 0 means person is moving toward camera
            approach = -vel[2]  # positive = approaching

            is_threat = (dist_xz < THREAT_DIST_M) and (approach > APPROACH_SPEED)
            track.is_threat = is_threat

            if is_threat:
                any_threat = True
                threat_info.append({
                    'id':       tid,
                    'dist_m':   round(dist_xz, 2),
                    'speed_ms': round(approach, 2),
                })
                self.get_logger().warn(
                    f'THREAT: person {tid} at {dist_xz:.2f}m '
                    f'approaching at {approach:.2f} m/s',
                    throttle_duration_sec=1.0)

        # ── Update freeze state ───────────────────────────────────────────────
        now = time.monotonic()
        if any_threat:
            self.freeze_until = now + FREEZE_HOLD_S
            if not self.frozen:
                self.frozen = True
                self.get_logger().warn('FREEZING: pedestrian threat detected')
        elif now > self.freeze_until:
            if self.frozen:
                self.frozen = False
                self.get_logger().info('Resuming: threat cleared')

        # ── Publish status JSON ───────────────────────────────────────────────
        status = {
            'frozen':    self.frozen,
            'tracks':    len(self.tracks),
            'confirmed': sum(1 for t in self.tracks.values() if t.confirmed),
            'threats':   threat_info,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

        # ── Publish RViz markers ──────────────────────────────────────────────
        self._publish_markers()

    # ── Safety timer (10 Hz) ─────────────────────────────────────────────────
    def _safety_timer_cb(self):
        """
        Continuously publish zero velocity when frozen.
        This overrides Nav2's cmd_vel via the topic multiplexer.
        """
        if not self.frozen:
            return

        zero = TwistStamped()
        zero.header.stamp    = self.get_clock().now().to_msg()
        zero.header.frame_id = 'base_footprint'
        # All velocities default to 0.0 — robot stops
        self._freeze_pub.publish(zero)

    # ── RViz markers ──────────────────────────────────────────────────────────
    def _publish_markers(self):
        arr = MarkerArray()

        # Clear old markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for tid, track in self.tracks.items():
            if not track.confirmed:
                continue

            pos = track.kalman.position
            vel = track.kalman.velocity

            # Sphere at person position
            sphere = Marker()
            sphere.header.frame_id = 'camera_link'
            sphere.header.stamp    = self.get_clock().now().to_msg()
            sphere.ns              = 'persons'
            sphere.id              = tid
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position.x = float(pos[0])
            sphere.pose.position.y = float(pos[1])
            sphere.pose.position.z = float(pos[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4

            # Red if threat, yellow if close, green otherwise
            if track.is_threat:
                sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            elif float(np.linalg.norm(pos[[0, 2]])) < THREAT_DIST_M:
                sphere.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
            else:
                sphere.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)

            lifetime = Duration()
            lifetime.sec     = 0
            lifetime.nanosec = 300_000_000   # 0.3s
            sphere.lifetime  = lifetime
            arr.markers.append(sphere)

            # Arrow showing velocity direction
            if np.linalg.norm(vel) > 0.05:
                arrow = Marker()
                arrow.header       = sphere.header
                arrow.ns           = 'velocities'
                arrow.id           = tid + 1000
                arrow.type         = Marker.ARROW
                arrow.action       = Marker.ADD
                arrow.pose.position.x = float(pos[0])
                arrow.pose.position.y = float(pos[1])
                arrow.pose.position.z = float(pos[2])
                arrow.pose.orientation.w = 1.0
                arrow.scale.x = float(np.linalg.norm(vel))   # arrow length = speed
                arrow.scale.y = 0.05
                arrow.scale.z = 0.05
                arrow.color   = sphere.color
                arrow.lifetime = lifetime
                arr.markers.append(arrow)

        self._marker_pub.publish(arr)


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = PedestrianSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
