#!/usr/bin/env python3
"""
Yale Delivery Robot - Waypoint UI Node
---------------------------------------
- Single goals  → /goal_pose (PoseStamped)
- Routes        → /waypoints (nav2_msgs/msg/WaypointFollowerResult via action)
                  Falls back to sequential /goal_pose with status monitoring
- Status        → /behavior_tree_log (most reliable in Nav2 Jazzy)
- Cancel        → /navigate_to_pose/_action/cancel_goal service
- Pose          → /amcl_pose (better than odom, uses localization)

Open http://localhost:8765 in browser.
"""

import os, json, yaml, math, threading, time
from http.server import HTTPServer, BaseHTTPRequestHandler
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import BehaviorTreeLog
from action_msgs.srv import CancelGoal

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
        self._route_progress = []   # list of (stop, status) for UI
        self._lock           = threading.Lock()

        # ── Publishers ────────────────────────────────────────────────────────
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        # AMCL pose (localization estimate — much better than odom)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        # Behavior tree log — most reliable status source in Nav2 Jazzy
        bt_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self._bt_log_cb, bt_qos)

        # Cancel service
        self._cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.get_logger().info(
            f'WaypointUINode ready — {len(self.waypoints)} waypoints, '
            f'{len(self.routes)} routes')

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
        """
        Observed Nav2 Jazzy success pattern:
          NavigateWithReplanning: RUNNING -> SUCCESS
          NavigateRecovery:       IDLE    -> SUCCESS  <- use this (fires last)

        Failure: NavigateRecovery: IDLE -> FAILURE
        Key: NavigateRecovery goes RUNNING->IDLE->SUCCESS not RUNNING->SUCCESS
        """
        for event in msg.event_log:
            node = event.node_name
            prev = event.previous_status
            curr = event.current_status

            # Success: NavigateRecovery IDLE -> SUCCESS
            if node == 'NavigateRecovery' and prev == 'IDLE' and curr == 'SUCCESS':
                with self._lock:
                    self.get_logger().info(f'Goal reached: {self.current_goal}')
                    self.nav_status = 'SUCCEEDED'
                    if self._route_progress:
                        for i, (stop, _) in enumerate(self._route_progress):
                            if stop == self.current_goal:
                                self._route_progress[i] = (stop, 'done')
                                break

            # Failure: NavigateRecovery IDLE -> FAILURE
            elif node == 'NavigateRecovery' and prev == 'IDLE' and curr == 'FAILURE':
                with self._lock:
                    self.get_logger().warn(f'Goal failed: {self.current_goal}')
                    self.nav_status = 'FAILED'
                    if self._route_progress:
                        for i, (stop, _) in enumerate(self._route_progress):
                            if stop == self.current_goal:
                                self._route_progress[i] = (stop, 'failed')
                                break

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
        """Send single waypoint goal."""
        if name not in self.waypoints:
            return {'ok': False, 'error': f'Unknown waypoint: {name}'}

        with self._lock:
            self.nav_status   = 'NAVIGATING'
            self.current_goal = name

        self._goal_pub.publish(self._make_pose(self.waypoints[name]))
        self.get_logger().info(f'Goal → {name}')
        return {'ok': True, 'target': name}

    def cancel_goal(self) -> dict:
        """Cancel active goal and any running route."""
        # Signal route thread to stop
        self._route_cancel.set()

        with self._lock:
            self.nav_status   = 'IDLE'
            self.current_goal = None
            self._route_name  = None
            self._route_progress = []

        # Try the cancel service
        if self._cancel_client.wait_for_service(timeout_sec=0.5):
            self._cancel_client.call_async(CancelGoal.Request())
            self.get_logger().info('Cancellation sent')
        else:
            self.get_logger().warn('Cancel service unavailable')

        return {'ok': True}

    def run_route(self, route_name: str) -> dict:
        """Run a full route — sequential goals, waits for each to finish."""
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

                # Mark as active
                with self._lock:
                    for i, (s, _) in enumerate(self._route_progress):
                        if s == stop:
                            self._route_progress[i] = (s, 'active')
                            break

                self.send_goal(stop)

                # Wait up to 120s for this stop to complete or fail
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

                # Reset for next stop
                with self._lock:
                    self.nav_status = 'NAVIGATING'

                time.sleep(1.5)  # brief pause at each stop

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
            }


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
        elif self.path in ('/', '/index.html'):
            self._send(200, UI_HTML, 'text/html; charset=utf-8')
        else:
            self._send(404, {'error': 'not found'})

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

/* header */
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

/* layout */
main{flex:1;display:grid;grid-template-columns:1fr 290px}
#left{padding:24px 28px;border-right:1px solid var(--border);display:flex;flex-direction:column;gap:28px}

/* section */
.sec-title{font-size:.62rem;letter-spacing:.15em;text-transform:uppercase;
           color:var(--muted);margin-bottom:12px;padding-bottom:7px;
           border-bottom:1px solid var(--border)}

/* waypoint grid */
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

/* route grid */
#route-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(190px,1fr));gap:10px}
.rt{background:var(--panel);border:1px solid var(--border);border-radius:10px;
    padding:14px 16px;cursor:pointer;text-align:left;
    color:var(--text);font-family:'DM Mono',monospace;transition:all .15s}
.rt:hover{border-color:var(--blue);transform:translateY(-2px)}
.rt.running{border-color:var(--blue);background:#0099ff0c;animation:pulse 1.4s infinite}
.rt-name{font-family:'Syne',sans-serif;font-size:.82rem;font-weight:700;
         text-transform:capitalize;margin-bottom:5px}
.rt-stops{font-size:.57rem;color:var(--muted);line-height:1.7}

/* sidebar */
#sidebar{padding:20px 18px;display:flex;flex-direction:column;gap:20px}
.sbi h3{font-size:.6rem;letter-spacing:.15em;text-transform:uppercase;
        color:var(--muted);margin-bottom:9px;padding-bottom:5px;
        border-bottom:1px solid var(--border)}

/* pose */
.pose-row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.pi{background:var(--panel);border:1px solid var(--border);border-radius:7px;
    padding:9px;text-align:center}
.pl{font-size:.55rem;color:var(--muted);margin-bottom:2px}
.pv{font-size:.85rem;color:var(--accent)}

/* goal */
#goal-box{background:var(--panel);border:1px solid var(--border);border-radius:8px;
          padding:11px 13px;font-size:.72rem;color:var(--muted);
          min-height:40px;display:flex;align-items:center;transition:all .2s}
#goal-box.on{color:var(--blue);border-color:var(--blue)}

/* route progress */
#prog-box{background:var(--panel);border:1px solid var(--border);border-radius:8px;
          padding:11px 13px;font-size:.65rem;min-height:40px;color:var(--muted)}
.prog-stop{display:flex;align-items:center;gap:8px;padding:3px 0}
.prog-dot{width:7px;height:7px;border-radius:50%;flex-shrink:0;background:var(--border)}
.prog-dot.active{background:var(--blue);box-shadow:0 0 6px var(--blue)}
.prog-dot.done{background:var(--accent)}
.prog-dot.failed{background:var(--fail)}
.prog-label{text-transform:capitalize}

/* cancel */
#cancel-btn{width:100%;padding:13px;background:transparent;
            border:1px solid var(--fail);border-radius:8px;
            color:var(--fail);font-family:'DM Mono',monospace;
            font-size:.75rem;cursor:pointer;letter-spacing:.08em;
            text-transform:uppercase;transition:all .15s}
#cancel-btn:hover{background:#ff446612}
#cancel-btn:disabled{opacity:.3;cursor:not-allowed}

/* log */
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
  <div class="sbi">
    <h3>Robot Pose (AMCL)</h3>
    <div class="pose-row">
      <div class="pi"><div class="pl">X (m)</div><div class="pv" id="px">—</div></div>
      <div class="pi"><div class="pl">Y (m)</div><div class="pv" id="py">—</div></div>
      <div class="pi"><div class="pl">YAW</div><div class="pv" id="pw">—</div></div>
    </div>
  </div>

  <div class="sbi">
    <h3>Current Goal</h3>
    <div id="goal-box">No active goal</div>
  </div>

  <div class="sbi">
    <h3>Route Progress</h3>
    <div id="prog-box" style="color:var(--muted)">No route running</div>
  </div>

  <div class="sbi">
    <h3>Control</h3>
    <button id="cancel-btn" disabled onclick="doCancel()">⬛ Cancel</button>
  </div>

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

async function poll() {
  try {
    const s = await fetch(`${API}/state`).then(r=>r.json());
    render(s);
    prev = s;
  } catch { setBadge('OFFLINE'); }
  setTimeout(poll, 800);
}

function render(s) {
  setBadge(s.nav_status, s.current_goal);

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

  // Update waypoint button states
  document.querySelectorAll('.wp').forEach(btn => {
    btn.classList.remove('active','done','failed','pending');
    if (s.route_active && s.current_goal !== btn.dataset.wp) {
      // Check route progress
      const prog = (s.route_progress || []).find(([n])=>n===btn.dataset.wp);
      if (prog) btn.classList.add(prog[1] === 'done' ? 'done' :
                                   prog[1] === 'failed' ? 'failed' :
                                   prog[1] === 'pending' ? 'pending' : '');
    }
    if (btn.dataset.wp === s.current_goal && s.nav_status === 'NAVIGATING')
      btn.classList.add('active');
    else if (btn.dataset.wp === s.current_goal && s.nav_status === 'SUCCEEDED')
      btn.classList.add('done');
  });

  // Update route button states
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
    b.className       = 'rt';
    b.dataset.route   = name;
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
      headers: {'Content-Type':'application/json'},
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
  return s.replace(/_/g,' ').replace(/\b\w/g, c=>c.toUpperCase());
}

function log(msg, type='') {
  const el   = document.getElementById('log');
  const now  = new Date().toTimeString().slice(0,8);
  const div  = document.createElement('div');
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
_node: WaypointUINode = None

def main():
    global _node
    rclpy.init()
    _node = WaypointUINode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    ros_thread.start()

    server = HTTPServer(('0.0.0.0', HTTP_PORT), Handler)
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