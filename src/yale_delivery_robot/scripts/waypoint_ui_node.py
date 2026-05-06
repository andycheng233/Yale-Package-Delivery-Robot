#!/usr/bin/env python3
"""
Yale Delivery Robot - Waypoint UI Node
---------------------------------------
- Single goals  → /goal_pose (PoseStamped)
- Routes        → sequential /goal_pose with status monitoring
- Status        → /behavior_tree_log (most reliable in Nav2 Jazzy)
- Cancel        → /navigate_to_pose/_action/cancel_goal service
- Pose          → /amcl_pose (better than odom, uses localization)
- Camera        → /pedestrian_debug_image (YOLO-annotated, streamed as MJPEG)

Open http://localhost:8765 in browser.
"""

import os, json, yaml, math, threading, time
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import BehaviorTreeLog
from action_msgs.srv import CancelGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HTTP_PORT = 8765


class WaypointUINode(Node):

    def __init__(self):
        super().__init__('waypoint_ui_node')

        # ── Load config ───────────────────────────────────────────────────────
        try:
            pkg_dir   = get_package_share_directory('yale_delivery_robot')
            yaml_path = os.path.join(pkg_dir, 'config', 'waypoints.yaml')
        except Exception:
            yaml_path = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'waypoints.yaml')

        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)

        self.waypoints = cfg.get('waypoints', {})
        self.routes    = cfg.get('routes', {})

        # ── State ─────────────────────────────────────────────────────────────
        self.nav_status      = 'IDLE'   # IDLE | NAVIGATING | SUCCEEDED | FAILED
        self.current_goal    = None
        self.robot_pose      = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._route_thread   = None
        self._route_cancel   = threading.Event()
        self._route_name     = None
        self._route_progress = []
        self._lock           = threading.Lock()

        # ── Camera state ──────────────────────────────────────────────────────
        self._bridge        = CvBridge()
        self._latest_frame  = None          # raw JPEG bytes
        self._frame_lock    = threading.Lock()
        self._frame_event   = threading.Event()

        # ── Publishers ────────────────────────────────────────────────────────
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        bt_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self._bt_log_cb, bt_qos)

        # Pedestrian debug image — raw BGR8 published by PedestrianSafetyNode
        debug_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(
            Image, '/pedestrian_debug_image', self._debug_image_cb, debug_qos)

        # Cancel service
        self._cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.get_logger().info(
            f'WaypointUINode ready — {len(self.waypoints)} waypoints, '
            f'{len(self.routes)} routes')
        self.get_logger().info(
            f'Camera stream → http://localhost:{HTTP_PORT}/stream')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _amcl_cb(self, msg):
        p = msg.pose.pose
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_pose = {
            'x':   round(p.position.x, 3),
            'y':   round(p.position.y, 3),
            'yaw': round(math.atan2(siny, cosy), 3),
        }

    def _bt_log_cb(self, msg):
        for event in msg.event_log:
            node = event.node_name
            prev = event.previous_status
            curr = event.current_status

            if node == 'NavigateRecovery' and prev == 'IDLE' and curr == 'SUCCESS':
                with self._lock:
                    self.get_logger().info(f'Goal reached: {self.current_goal}')
                    self.nav_status = 'SUCCEEDED'
                    if self._route_progress:
                        for i, (stop, _) in enumerate(self._route_progress):
                            if stop == self.current_goal:
                                self._route_progress[i] = (stop, 'done')
                                break

            elif node == 'NavigateRecovery' and prev == 'IDLE' and curr == 'FAILURE':
                with self._lock:
                    self.get_logger().warn(f'Goal failed: {self.current_goal}')
                    self.nav_status = 'FAILED'
                    if self._route_progress:
                        for i, (stop, _) in enumerate(self._route_progress):
                            if stop == self.current_goal:
                                self._route_progress[i] = (stop, 'failed')
                                break

    def _debug_image_cb(self, msg: Image):
        """
        Convert sensor_msgs/Image (bgr8) → JPEG bytes and store for MJPEG stream.
        PedestrianSafetyNode publishes bgr8; cv_bridge handles the conversion.
        """
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, 80])   # 80 = good quality / low latency
            if ok:
                with self._frame_lock:
                    self._latest_frame = buf.tobytes()
                self._frame_event.set()
        except Exception as e:
            self.get_logger().warn(
                f'Debug image encode error: {e}', throttle_duration_sec=5.0)

    # ── Navigation ────────────────────────────────────────────────────────────

    def _make_pose(self, wp: dict) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id    = 'map'
        p.header.stamp       = self.get_clock().now().to_msg()
        p.pose.position.x    = float(wp['x'])
        p.pose.position.y    = float(wp['y'])
        p.pose.position.z    = float(wp.get('z', 0.0))
        p.pose.orientation.x = float(wp.get('qx', 0.0))
        p.pose.orientation.y = float(wp.get('qy', 0.0))
        p.pose.orientation.z = float(wp.get('qz', 0.0))
        p.pose.orientation.w = float(wp.get('qw', 1.0))
        return p

    def send_goal(self, name: str) -> dict:
        if name not in self.waypoints:
            return {'ok': False, 'error': f'Unknown waypoint: {name}'}
        with self._lock:
            self.nav_status   = 'NAVIGATING'
            self.current_goal = name
        self._goal_pub.publish(self._make_pose(self.waypoints[name]))
        self.get_logger().info(f'Goal → {name}')
        return {'ok': True, 'target': name}

    def cancel_goal(self) -> dict:
        self._route_cancel.set()
        with self._lock:
            self.nav_status      = 'IDLE'
            self.current_goal    = None
            self._route_name     = None
            self._route_progress = []
        if self._cancel_client.wait_for_service(timeout_sec=0.5):
            self._cancel_client.call_async(CancelGoal.Request())
            self.get_logger().info('Cancellation sent')
        else:
            self.get_logger().warn('Cancel service unavailable')
        return {'ok': True}

    def run_route(self, route_name: str) -> dict:
        if route_name not in self.routes:
            return {'ok': False, 'error': f'Unknown route: {route_name}'}
        if self._route_thread and self._route_thread.is_alive():
            return {'ok': False, 'error': 'Route already running — cancel first'}

        stops = self.routes[route_name]
        self._route_cancel.clear()
        with self._lock:
            self._route_name     = route_name
            self._route_progress = [(s, 'pending') for s in stops]

        def _execute():
            self.get_logger().info(
                f'Route "{route_name}" started — {len(stops)} stops')
            for stop in stops:
                if self._route_cancel.is_set():
                    self.get_logger().info('Route cancelled by user')
                    break
                with self._lock:
                    for i, (s, _) in enumerate(self._route_progress):
                        if s == stop:
                            self._route_progress[i] = (s, 'active')
                            break
                self.send_goal(stop)
                deadline = time.time() + 120
                while time.time() < deadline:
                    if self._route_cancel.is_set():
                        break
                    with self._lock:
                        status = self.nav_status
                    if status in ('SUCCEEDED', 'FAILED'):
                        break
                    time.sleep(0.4)
                if self._route_cancel.is_set():
                    break
                with self._lock:
                    if self.nav_status == 'FAILED':
                        self.get_logger().warn(
                            f'Stop "{stop}" failed — aborting route')
                        break
                with self._lock:
                    self.nav_status = 'NAVIGATING'
                time.sleep(1.5)
            with self._lock:
                self.nav_status   = 'IDLE'
                self.current_goal = None
                self._route_name  = None
            self.get_logger().info(f'Route "{route_name}" finished')

        self._route_thread = threading.Thread(target=_execute, daemon=True)
        self._route_thread.start()
        return {'ok': True, 'route': route_name, 'stops': stops}

    # ── State for UI ──────────────────────────────────────────────────────────

    def get_state(self) -> dict:
        with self._lock:
            return {
                'waypoints':      list(self.waypoints.keys()),
                'routes':         self.routes,
                'nav_status':     self.nav_status,
                'current_goal':   self.current_goal,
                'robot_pose':     self.robot_pose,
                'route_name':     self._route_name,
                'route_progress': self._route_progress,
                'route_active':   bool(
                    self._route_thread and self._route_thread.is_alive()),
                'camera_active':  self._latest_frame is not None,
            }


# ── Threaded HTTP Server ───────────────────────────────────────────────────────
# ThreadingMixIn ensures the MJPEG stream connection never blocks /state polling

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


# ── HTTP Handler ───────────────────────────────────────────────────────────────

_node: WaypointUINode = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass

    def _send(self, code, data, ctype='application/json'):
        body = (json.dumps(data) if ctype == 'application/json' else data
                ).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', len(body))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,POST')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        if self.path == '/state':
            self._send(200, _node.get_state())
        elif self.path == '/stream':
            self._stream_mjpeg()
        elif self.path in ('/', '/index.html'):
            self._send(200, UI_HTML, 'text/html; charset=utf-8')
        else:
            self._send(404, {'error': 'not found'})

    def _stream_mjpeg(self):
        """
        Serve a continuous MJPEG stream.
        The browser treats a multipart/x-mixed-replace response as a live <img>.
        Blocks until client disconnects — runs safely in its own thread
        thanks to ThreadedHTTPServer.
        """
        self.send_response(200)
        self.send_header(
            'Content-Type', 'multipart/x-mixed-replace; boundary=--FRAME')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        try:
            while True:
                # Block until a fresh frame arrives (or 2 s timeout)
                got = _node._frame_event.wait(timeout=2.0)
                _node._frame_event.clear()

                if not got:
                    # Send a tiny keep-alive comment so the connection stays open
                    self.wfile.write(b'--FRAME\r\n\r\n')
                    self.wfile.flush()
                    continue

                with _node._frame_lock:
                    frame = _node._latest_frame

                if frame is None:
                    continue

                self.wfile.write(
                    b'--FRAME\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(frame)).encode() + b'\r\n\r\n'
                    + frame + b'\r\n'
                )
                self.wfile.flush()

        except (BrokenPipeError, ConnectionResetError):
            pass   # Client navigated away or closed the tab

    def do_POST(self):
        n    = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(n)) if n else {}
        routes = {
            '/goto':   lambda: _node.send_goal(body.get('waypoint', '')),
            '/cancel': lambda: _node.cancel_goal(),
            '/route':  lambda: _node.run_route(body.get('route', '')),
        }
        fn = routes.get(self.path)
        if fn:
            self._send(200, fn())
        else:
            self._send(404, {'error': 'not found'})


# ── UI HTML ────────────────────────────────────────────────────────────────────
UI_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Yale Delivery Robot</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=DM+Mono:wght@400;500&family=Syne:wght@700;800&display=swap');
:root{
  --bg:#0d0f14;--panel:#13161d;--border:#1f2433;
  --accent:#00e5a0;--blue:#0099ff;--warn:#ff6b35;
  --text:#e8eaf0;--muted:#5a6070;--fail:#ff4466;
}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'DM Mono',monospace;background:var(--bg);color:var(--text);
     min-height:100vh;display:flex;flex-direction:column}

header{padding:16px 28px;background:var(--panel);border-bottom:1px solid var(--border);
       display:flex;align-items:center;justify-content:space-between;gap:12px}
header h1{font-family:'Syne',sans-serif;font-size:1.15rem;font-weight:800;
          letter-spacing:.06em;color:var(--accent)}
header h1 span{color:var(--text)}
#badge{font-size:.65rem;padding:5px 14px;border-radius:999px;font-weight:500;
       letter-spacing:.1em;text-transform:uppercase;transition:all .3s;white-space:nowrap}
.b-IDLE{background:#1f2433;color:var(--muted);border:1px solid var(--border)}
.b-NAVIGATING{background:#0099ff18;color:var(--blue);border:1px solid var(--blue);animation:pulse 1.4s infinite}
.b-SUCCEEDED{background:#00e5a018;color:var(--accent);border:1px solid var(--accent)}
.b-FAILED{background:#ff446618;color:var(--fail);border:1px solid var(--fail)}
.b-OFFLINE{background:#ff6b3518;color:var(--warn);border:1px solid var(--warn)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.45}}

main{flex:1;display:grid;grid-template-columns:1fr 300px}
#left{padding:24px 28px;border-right:1px solid var(--border);display:flex;flex-direction:column;gap:28px}

.sec-title{font-size:.62rem;letter-spacing:.15em;text-transform:uppercase;
           color:var(--muted);margin-bottom:12px;padding-bottom:7px;
           border-bottom:1px solid var(--border)}

#wp-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:10px}
.wp{background:var(--panel);border:1px solid var(--border);border-radius:10px;
    padding:15px 16px;cursor:pointer;text-align:left;transition:all .15s;
    color:var(--text);font-family:'DM Mono',monospace;position:relative;overflow:hidden}
.wp:hover{border-color:var(--accent);transform:translateY(-2px)}
.wp:active{transform:translateY(0)}
.wp.active{border-color:var(--blue);background:#0099ff0c;animation:pulse 1.4s infinite}
.wp.done{border-color:var(--accent);background:#00e5a00c}
.wp.failed{border-color:var(--fail);background:#ff44660c}
.wp.pending{opacity:.5}
.wp-name{font-family:'Syne',sans-serif;font-size:.85rem;font-weight:700;
         text-transform:capitalize;margin-bottom:3px}
.wp-hint{font-size:.58rem;color:var(--muted)}

#route-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(190px,1fr));gap:10px}
.rt{background:var(--panel);border:1px solid var(--border);border-radius:10px;
    padding:14px 16px;cursor:pointer;text-align:left;
    color:var(--text);font-family:'DM Mono',monospace;transition:all .15s}
.rt:hover{border-color:var(--blue);transform:translateY(-2px)}
.rt.running{border-color:var(--blue);background:#0099ff0c;animation:pulse 1.4s infinite}
.rt-name{font-family:'Syne',sans-serif;font-size:.82rem;font-weight:700;
         text-transform:capitalize;margin-bottom:5px}
.rt-stops{font-size:.57rem;color:var(--muted);line-height:1.7}

#sidebar{padding:20px 18px;display:flex;flex-direction:column;gap:20px;overflow-y:auto}
.sbi h3{font-size:.6rem;letter-spacing:.15em;text-transform:uppercase;
        color:var(--muted);margin-bottom:9px;padding-bottom:5px;
        border-bottom:1px solid var(--border)}

/* ── Camera feed ── */
#cam-wrap{
  position:relative;background:#0a0c10;border:1px solid var(--border);
  border-radius:8px;overflow:hidden;aspect-ratio:16/9;
  display:flex;align-items:center;justify-content:center}
#cam-img{width:100%;height:100%;object-fit:cover;display:block}
#cam-off{
  display:none;flex-direction:column;align-items:center;
  gap:6px;color:var(--muted);font-size:.65rem;position:absolute}
/* Threat overlay banner — shown by JS when status JSON carries threats */
#threat-banner{
  position:absolute;bottom:0;left:0;right:0;
  background:#ff446688;backdrop-filter:blur(4px);
  color:#fff;font-size:.65rem;letter-spacing:.1em;
  text-align:center;padding:5px 0;text-transform:uppercase;
  display:none}
/* Camera signal dot */
#cam-dot{
  position:absolute;top:7px;right:8px;
  width:7px;height:7px;border-radius:50%;
  background:var(--muted);transition:background .4s}
#cam-dot.live{background:var(--accent);box-shadow:0 0 6px var(--accent);
              animation:pulse 1.4s infinite}

/* pose */
.pose-row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.pi{background:var(--panel);border:1px solid var(--border);border-radius:7px;
    padding:9px;text-align:center}
.pl{font-size:.55rem;color:var(--muted);margin-bottom:2px}
.pv{font-size:.85rem;color:var(--accent)}

#goal-box{background:var(--panel);border:1px solid var(--border);border-radius:8px;
          padding:11px 13px;font-size:.72rem;color:var(--muted);
          min-height:40px;display:flex;align-items:center;transition:all .2s}
#goal-box.on{color:var(--blue);border-color:var(--blue)}

#prog-box{background:var(--panel);border:1px solid var(--border);border-radius:8px;
          padding:11px 13px;font-size:.65rem;min-height:40px;color:var(--muted)}
.prog-stop{display:flex;align-items:center;gap:8px;padding:3px 0}
.prog-dot{width:7px;height:7px;border-radius:50%;flex-shrink:0;background:var(--border)}
.prog-dot.active{background:var(--blue);box-shadow:0 0 6px var(--blue)}
.prog-dot.done{background:var(--accent)}
.prog-dot.failed{background:var(--fail)}
.prog-label{text-transform:capitalize}

#cancel-btn{width:100%;padding:13px;background:transparent;
            border:1px solid var(--fail);border-radius:8px;
            color:var(--fail);font-family:'DM Mono',monospace;
            font-size:.75rem;cursor:pointer;letter-spacing:.08em;
            text-transform:uppercase;transition:all .15s}
#cancel-btn:hover{background:#ff446612}
#cancel-btn:disabled{opacity:.3;cursor:not-allowed}

#log{font-size:.62rem;color:var(--muted);max-height:130px;
     overflow-y:auto;display:flex;flex-direction:column;gap:3px}
.le{display:flex;gap:8px}
.lt{color:#252a3a;flex-shrink:0}
.lok{color:var(--accent)}
.lerr{color:var(--fail)}
.lwarn{color:var(--warn)}
</style>
</head>
<body>
<header>
  <h1>YALE <span>DELIVERY ROBOT</span></h1>
  <div id="badge" class="b-IDLE">IDLE</div>
</header>

<main>
<div id="left">
  <div>
    <div class="sec-title">Delivery Destinations</div>
    <div id="wp-grid"><span style="color:var(--muted);font-size:.8rem">Loading...</span></div>
  </div>
  <div>
    <div class="sec-title">Routes</div>
    <div id="route-grid"><span style="color:var(--muted);font-size:.8rem">Loading...</span></div>
  </div>
</div>

<div id="sidebar">

  <!-- ── Pedestrian Safety Camera ── -->
  <div class="sbi">
    <h3>Pedestrian Safety Feed</h3>
    <div id="cam-wrap">
      <img id="cam-img"
           src="http://localhost:8765/stream"
           onerror="camError()"
      />
      <div id="cam-off">
        <span style="font-size:1.4rem">📷</span>
        No camera signal
      </div>
      <div id="threat-banner">⚠ THREAT DETECTED — FROZEN</div>
      <div id="cam-dot"></div>
    </div>
  </div>

  <!-- ── Robot Pose ── -->
  <div class="sbi">
    <h3>Robot Pose (AMCL)</h3>
    <div class="pose-row">
      <div class="pi"><div class="pl">X (m)</div><div class="pv" id="px">—</div></div>
      <div class="pi"><div class="pl">Y (m)</div><div class="pv" id="py">—</div></div>
      <div class="pi"><div class="pl">YAW</div><div class="pv" id="pw">—</div></div>
    </div>
  </div>

  <!-- ── Current Goal ── -->
  <div class="sbi">
    <h3>Current Goal</h3>
    <div id="goal-box">No active goal</div>
  </div>

  <!-- ── Route Progress ── -->
  <div class="sbi">
    <h3>Route Progress</h3>
    <div id="prog-box" style="color:var(--muted)">No route running</div>
  </div>

  <!-- ── Control ── -->
  <div class="sbi">
    <h3>Control</h3>
    <button id="cancel-btn" disabled onclick="doCancel()">⬛ Cancel</button>
  </div>

  <!-- ── Log ── -->
  <div class="sbi">
    <h3>Log</h3>
    <div id="log"></div>
  </div>

</div>
</main>

<script>
const API = 'http://localhost:8765';
let prev = {};
let built = false;
let camErrored = false;

function camError() {
  camErrored = true;
  document.getElementById('cam-img').style.display = 'none';
  document.getElementById('cam-off').style.display = 'flex';
  document.getElementById('cam-dot').classList.remove('live');
}

async function poll() {
  try {
    const s = await fetch(`${API}/state`).then(r => r.json());
    render(s);
    prev = s;
  } catch {
    setBadge('OFFLINE');
  }
  setTimeout(poll, 800);
}

function render(s) {
  setBadge(s.nav_status, s.current_goal);

  // Camera dot + reconnect if previously errored
  const dot = document.getElementById('cam-dot');
  if (s.camera_active) {
    dot.classList.add('live');
    if (camErrored) {
      // Retry stream — node has frames now
      const img = document.getElementById('cam-img');
      img.style.display = 'block';
      document.getElementById('cam-off').style.display = 'none';
      img.src = API + '/stream?' + Date.now();
      camErrored = false;
    }
  } else {
    dot.classList.remove('live');
  }

  // Pose
  document.getElementById('px').textContent = s.robot_pose?.x ?? '—';
  document.getElementById('py').textContent = s.robot_pose?.y ?? '—';
  document.getElementById('pw').textContent = s.robot_pose?.yaw ?? '—';

  // Goal box
  const gb = document.getElementById('goal-box');
  if (s.current_goal) {
    gb.textContent = '→ ' + fmt(s.current_goal);
    gb.classList.add('on');
  } else {
    gb.textContent = 'No active goal';
    gb.classList.remove('on');
  }

  // Cancel button
  document.getElementById('cancel-btn').disabled =
    s.nav_status === 'IDLE' && !s.route_active;

  // Build grids once
  if (!built && s.waypoints?.length) {
    buildWP(s.waypoints);
    buildRoutes(s.routes || {});
    built = true;
  }

  // Waypoint button states
  document.querySelectorAll('.wp').forEach(btn => {
    btn.classList.remove('active', 'done', 'failed', 'pending');
    if (s.route_active) {
      const prog = (s.route_progress || []).find(([n]) => n === btn.dataset.wp);
      if (prog) btn.classList.add(
        prog[1] === 'done' ? 'done' :
        prog[1] === 'failed' ? 'failed' :
        prog[1] === 'pending' ? 'pending' : '');
    }
    if (btn.dataset.wp === s.current_goal && s.nav_status === 'NAVIGATING')
      btn.classList.add('active');
    else if (btn.dataset.wp === s.current_goal && s.nav_status === 'SUCCEEDED')
      btn.classList.add('done');
  });

  // Route button states
  document.querySelectorAll('.rt').forEach(btn => {
    btn.classList.toggle('running',
      s.route_active && btn.dataset.route === s.route_name);
  });

  // Route progress panel
  const pb = document.getElementById('prog-box');
  if (s.route_active && s.route_progress?.length) {
    pb.innerHTML = s.route_progress.map(([name, status]) => `
      <div class="prog-stop">
        <div class="prog-dot ${status}"></div>
        <span class="prog-label">${fmt(name)}</span>
        <span style="margin-left:auto;font-size:.55rem;color:var(--muted)">${status}</span>
      </div>`).join('');
  } else {
    pb.textContent = 'No route running';
    pb.style.color = 'var(--muted)';
  }

  // Log status changes
  if (prev.nav_status !== s.nav_status) {
    if (s.nav_status === 'SUCCEEDED')
      log(`Reached: ${prev.current_goal || s.current_goal}`, 'ok');
    else if (s.nav_status === 'FAILED')
      log(`Failed at: ${prev.current_goal || s.current_goal}`, 'err');
  }
}

function buildWP(wps) {
  const g = document.getElementById('wp-grid');
  g.innerHTML = '';
  wps.forEach(wp => {
    const b = document.createElement('button');
    b.className  = 'wp';
    b.dataset.wp = wp;
    b.innerHTML  = `<div class="wp-name">${fmt(wp)}</div>
                    <div class="wp-hint">click to navigate</div>`;
    b.onclick = () => gotoWP(wp);
    g.appendChild(b);
  });
}

function buildRoutes(routes) {
  const g = document.getElementById('route-grid');
  g.innerHTML = '';
  Object.entries(routes).forEach(([name, stops]) => {
    const b = document.createElement('button');
    b.className     = 'rt';
    b.dataset.route = name;
    b.innerHTML = `<div class="rt-name">${fmt(name)}</div>
                   <div class="rt-stops">${stops.join(' → ')}</div>`;
    b.onclick = () => runRoute(name);
    g.appendChild(b);
  });
}

async function gotoWP(wp) {
  log(`Navigating → ${fmt(wp)}`);
  const r = await post('/goto', {waypoint: wp});
  if (!r.ok) log(r.error, 'err');
}

async function runRoute(name) {
  log(`Starting route: ${fmt(name)}`, 'ok');
  const r = await post('/route', {route: name});
  if (!r.ok) log(r.error, 'err');
}

async function doCancel() {
  log('Cancelling...', 'warn');
  await post('/cancel', {});
}

async function post(path, data) {
  try {
    return await fetch(API + path, {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(data)
    }).then(r => r.json());
  } catch(e) {
    return {ok: false, error: 'Connection failed'};
  }
}

function setBadge(status, goal) {
  const b = document.getElementById('badge');
  b.className = 'b-' + status;
  b.textContent = status === 'NAVIGATING' && goal
    ? 'NAVIGATING → ' + fmt(goal) : status;
}

function fmt(s) {
  return s.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
}

function log(msg, type='') {
  const el  = document.getElementById('log');
  const now = new Date().toTimeString().slice(0, 8);
  const div = document.createElement('div');
  div.className = 'le';
  div.innerHTML = `<span class="lt">${now}</span>
                   <span class="l${type||'muted'}">${msg}</span>`;
  el.prepend(div);
  while (el.children.length > 30) el.removeChild(el.lastChild);
}

poll();
</script>
</body>
</html>"""


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    global _node
    rclpy.init()
    _node = WaypointUINode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    ros_thread.start()

    server = ThreadedHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    _node.get_logger().info(f'UI → http://localhost:{HTTP_PORT}')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        _node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()