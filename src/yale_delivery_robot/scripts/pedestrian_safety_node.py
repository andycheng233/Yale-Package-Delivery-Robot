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
  6. Debug     — publish annotated RGB image with boxes to /pedestrian_debug_image

ROS topics consumed:
  /camera/image_raw/compressed   sensor_msgs/CompressedImage
  /camera/depth/image_raw        sensor_msgs/Image
  /camera/camera_info            sensor_msgs/CameraInfo

ROS topics published:
  /pedestrian_detections      std_msgs/String
  /cmd_vel_safety             geometry_msgs/TwistStamped
  /pedestrian_markers         visualization_msgs/MarkerArray
  /pedestrian_debug_image     sensor_msgs/Image
"""

import json
import time
import threading
from dataclasses import dataclass
from typing import Dict

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


# ── Config ────────────────────────────────────────────────────────────────────

YOLO_MODEL = 'yolov8n.pt'
YOLO_CONF = 0.45
YOLO_DEVICE = 'cpu'   # keep cpu unless you install a PyTorch build that supports your GPU

RGB_TOPIC = '/camera/image_raw/compressed'
DEPTH_TOPIC = '/camera/depth/image_raw'
CAM_INFO_TOPIC = '/camera/camera_info'

# Tracking
MAX_TRACK_AGE = 1.5
MIN_DETECTIONS = 3

# Threat thresholds
THREAT_DIST_M = 2.5
APPROACH_SPEED = 0.3
FREEZE_HOLD_S = 1.5

# Depth sampling
DEPTH_SAMPLE_R = 5

# Camera intrinsics fallback
DEFAULT_FX = 554.0
DEFAULT_FY = 554.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0


# ── Kalman Filter ─────────────────────────────────────────────────────────────

class KalmanTracker3D:
    """
    State: [x, y, z, vx, vy, vz]
    Measurement: [x, y, z]
    """

    def __init__(self, x: float, y: float, z: float):
        self.x = np.array([x, y, z, 0.0, 0.0, 0.0], dtype=np.float64)

        self.P = np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0])

        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        q_pos = 0.05
        q_vel = 0.5
        self.Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])

        self.R = np.diag([0.05**2, 0.05**2, 0.10**2])

        self.last_update = time.monotonic()
        self.hit_count = 0

    def predict(self) -> None:
        now = time.monotonic()
        dt = min(now - self.last_update, 0.5)

        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, x: float, y: float, z: float) -> None:
        z_meas = np.array([x, y, z], dtype=np.float64)
        y_innov = z_meas - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y_innov
        self.P = (np.eye(6) - K @ self.H) @ self.P

        self.last_update = time.monotonic()
        self.hit_count += 1

    @property
    def position(self) -> np.ndarray:
        return self.x[:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:]

    @property
    def age(self) -> float:
        return time.monotonic() - self.last_update


# ── Track wrapper ─────────────────────────────────────────────────────────────

@dataclass
class PersonTrack:
    track_id: int
    kalman: KalmanTracker3D
    bbox: tuple = (0, 0, 0, 0)
    is_threat: bool = False

    @property
    def confirmed(self) -> bool:
        return self.kalman.hit_count >= MIN_DETECTIONS


# ── Main node ─────────────────────────────────────────────────────────────────

class PedestrianSafetyNode(Node):
    def __init__(self):
        super().__init__('pedestrian_safety_node')

        # YOLO
        try:
            from ultralytics import YOLO
            self.yolo = YOLO(YOLO_MODEL)
            self.yolo_available = True
            self.get_logger().info(f'YOLO loaded: {YOLO_MODEL} on {YOLO_DEVICE}')
        except ImportError:
            self.get_logger().error(
                'ultralytics not installed. Run: pip3 install ultralytics --break-system-packages'
            )
            self.yolo_available = False
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO: {e}')
            self.yolo_available = False

        # Camera intrinsics
        self.fx = DEFAULT_FX
        self.fy = DEFAULT_FY
        self.cx = DEFAULT_CX
        self.cy = DEFAULT_CY
        self.camera_info_received = False

        # State
        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_lock = threading.Lock()

        self.tracks: Dict[int, PersonTrack] = {}
        self.next_id = 0

        self.frozen = False
        self.freeze_until = 0.0

        self.last_detect_time = 0.0
        self.detect_interval = 0.1  # ~10 Hz max

        # QoS
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.create_subscription(CompressedImage, RGB_TOPIC, self._rgb_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(CameraInfo, CAM_INFO_TOPIC, self._camera_info_cb, 10)

        # Publishers
        self._freeze_pub = self.create_publisher(TwistStamped, '/cmd_vel_safety', 10)
        self._status_pub = self.create_publisher(String, '/pedestrian_detections', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/pedestrian_markers', 10)
        self._debug_image_pub = self.create_publisher(Image, '/pedestrian_debug_image', 10)

        # Timer
        self.create_timer(0.1, self._safety_timer_cb)

        self.get_logger().info('PedestrianSafetyNode ready')
        self.get_logger().info(
            f'Threat zone: {THREAT_DIST_M} m, approach threshold: {APPROACH_SPEED} m/s'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera intrinsics loaded: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}'
            )

    def _depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.depth_lock:
                self.depth_image = depth.copy()
        except Exception as e:
            self.get_logger().warn(f'Depth conversion error: {e}', throttle_duration_sec=5.0)

    def _rgb_cb(self, msg: CompressedImage):
        if not self.yolo_available:
            return

        now = time.monotonic()
        if (now - self.last_detect_time) < self.detect_interval:
            return
        self.last_detect_time = now

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn('Failed to decode compressed RGB image')
                return
        except Exception as e:
            self.get_logger().warn(f'RGB decode error: {e}', throttle_duration_sec=5.0)
            return

        with self.depth_lock:
            depth = self.depth_image.copy() if self.depth_image is not None else None

        if depth is None:
            return

        self._run_detection(frame, depth)

    # ── Main detection pipeline ───────────────────────────────────────────

    def _run_detection(self, frame: np.ndarray, depth: np.ndarray):
        orig_h, orig_w = frame.shape[:2]
        debug_frame = frame.copy()

        results = self.yolo(
            frame,
            classes=[0],   # person
            conf=YOLO_CONF,
            device=YOLO_DEVICE,
            verbose=False
        )

        detections = []   # (cx_px, cy_px, x3d, y3d, z3d, bbox)

        for result in results:
            for box in result.boxes:
                sx1, sy1, sx2, sy2 = [int(v) for v in box.xyxy[0].tolist()]

                x1 = int(sx1)
                y1 = int(sy1)
                x2 = int(sx2)
                y2 = int(sy2)

                x1 = max(0, min(orig_w - 1, x1))
                y1 = max(0, min(orig_h - 1, y1))
                x2 = max(0, min(orig_w - 1, x2))
                y2 = max(0, min(orig_h - 1, y2))

                cx_px = (x1 + x2) // 2
                cy_px = (y1 + y2) // 2

                # draw raw detection
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(debug_frame, (cx_px, cy_px), 4, (0, 0, 255), -1)
                cv2.putText(
                    debug_frame,
                    'person',
                    (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                # depth patch around center
                r = DEPTH_SAMPLE_R
                y_lo = max(0, cy_px - r)
                y_hi = min(depth.shape[0], cy_px + r + 1)
                x_lo = max(0, cx_px - r)
                x_hi = min(depth.shape[1], cx_px + r + 1)

                patch = depth[y_lo:y_hi, x_lo:x_hi]
                valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 20.0)]

                if valid.size < 4:
                    continue

                z3d = float(np.median(valid))
                x3d = (cx_px - self.cx) * z3d / self.fx
                y3d = (cy_px - self.cy) * z3d / self.fy

                detections.append((cx_px, cy_px, x3d, y3d, z3d, (x1, y1, x2, y2)))

        matched_ids = set()

        for (_, _, x3d, y3d, z3d, bbox) in detections:
            meas = np.array([x3d, y3d, z3d], dtype=np.float64)
            best_id = None
            best_dist = 1.5

            for tid, track in self.tracks.items():
                if tid in matched_ids:
                    continue
                track.kalman.predict()
                dist = float(np.linalg.norm(track.kalman.position - meas))
                if dist < best_dist:
                    best_dist = dist
                    best_id = tid

            if best_id is not None:
                self.tracks[best_id].kalman.update(x3d, y3d, z3d)
                self.tracks[best_id].bbox = bbox
                matched_ids.add(best_id)
            else:
                new_id = self.next_id
                self.next_id += 1
                kf = KalmanTracker3D(x3d, y3d, z3d)
                self.tracks[new_id] = PersonTrack(
                    track_id=new_id,
                    kalman=kf,
                    bbox=bbox
                )
                matched_ids.add(new_id)

        # remove stale tracks
        self.tracks = {
            tid: track for tid, track in self.tracks.items()
            if track.kalman.age < MAX_TRACK_AGE
        }

        # threat assessment
        any_threat = False
        threat_info = []

        for tid, track in self.tracks.items():
            if not track.confirmed:
                track.is_threat = False
                continue

            pos = track.kalman.position
            vel = track.kalman.velocity

            dist_xz = float(np.sqrt(pos[0]**2 + pos[2]**2))
            approach = -vel[2]

            is_threat = (dist_xz < THREAT_DIST_M) and (approach > APPROACH_SPEED)
            track.is_threat = is_threat

            x1, y1, x2, y2 = track.bbox
            color = (0, 0, 255) if is_threat else (0, 255, 255)

            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                debug_frame,
                f'ID {tid}  d={dist_xz:.2f}m  vz={approach:.2f}',
                (x1, min(orig_h - 10, y2 + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2
            )

            if is_threat:
                any_threat = True
                threat_info.append({
                    'id': tid,
                    'dist_m': round(dist_xz, 2),
                    'speed_ms': round(approach, 2),
                })
                self.get_logger().warn(
                    f'THREAT: person {tid} at {dist_xz:.2f} m approaching at {approach:.2f} m/s',
                    throttle_duration_sec=1.0
                )

        # freeze logic
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

        # overlay freeze state
        status_text = 'FROZEN' if self.frozen else 'CLEAR'
        status_color = (0, 0, 255) if self.frozen else (0, 255, 0)
        cv2.putText(
            debug_frame,
            status_text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            status_color,
            3
        )

        # publish status JSON
        status = {
            'frozen': self.frozen,
            'tracks': len(self.tracks),
            'confirmed': sum(1 for t in self.tracks.values() if t.confirmed),
            'threats': threat_info,
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self._status_pub.publish(status_msg)

        # publish markers
        self._publish_markers()

        # publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            self._debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'Debug image publish error: {e}', throttle_duration_sec=5.0)

    # ── Safety timer ──────────────────────────────────────────────────────

    def _safety_timer_cb(self):
        if not self.frozen:
            return

        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        zero.header.frame_id = 'base_footprint'
        self._freeze_pub.publish(zero)

    # ── RViz markers ──────────────────────────────────────────────────────

    def _publish_markers(self):
        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for tid, track in self.tracks.items():
            if not track.confirmed:
                continue

            pos = track.kalman.position
            vel = track.kalman.velocity

            sphere = Marker()
            sphere.header.frame_id = 'camera_link'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'persons'
            sphere.id = tid
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(pos[0])
            sphere.pose.position.y = float(pos[1])
            sphere.pose.position.z = float(pos[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.4
            sphere.scale.y = 0.4
            sphere.scale.z = 0.4

            if track.is_threat:
                sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            elif float(np.linalg.norm(pos[[0, 2]])) < THREAT_DIST_M:
                sphere.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
            else:
                sphere.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)

            lifetime = Duration()
            lifetime.sec = 0
            lifetime.nanosec = 300_000_000
            sphere.lifetime = lifetime
            arr.markers.append(sphere)

            if np.linalg.norm(vel) > 0.05:
                arrow = Marker()
                arrow.header = sphere.header
                arrow.ns = 'velocities'
                arrow.id = tid + 1000
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = float(pos[0])
                arrow.pose.position.y = float(pos[1])
                arrow.pose.position.z = float(pos[2])
                arrow.pose.orientation.w = 1.0
                arrow.scale.x = float(np.linalg.norm(vel))
                arrow.scale.y = 0.05
                arrow.scale.z = 0.05
                arrow.color = sphere.color
                arrow.lifetime = lifetime
                arr.markers.append(arrow)

        self._marker_pub.publish(arr)


# ── Main ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
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