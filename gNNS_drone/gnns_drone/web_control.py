"""
gNNS Drone — Web Control Panel
================================
Beautiful web UI for interactive drone control.
Replaces glitchy terminal input with a real control page.

Run:
  Terminal 1: sim_vehicle.py -v ArduCopter --no-mavproxy
  Terminal 2: cd gNNS_drone && python3 -m gnns_drone.web_control --sitl

Opens browser at http://localhost:5000
"""

import sys
import os
import time
import math
import json
import threading
import logging
from pathlib import Path
from http.server import HTTPServer, SimpleHTTPRequestHandler
import urllib.parse

# Add project root
PROJECT_ROOT = str(Path(__file__).parent.parent)
sys.path.insert(0, PROJECT_ROOT)

from .mavlink_bridge import MAVLinkBridge
from .rtabmap_odom import RTABMapOdom, OdomData
from .flight_controller import FlightController, FlightConfig

logger = logging.getLogger("gnns.web")


class DroneController:
    """Manages the drone connection and flight operations."""

    def __init__(self, sitl_mode=True):
        self.sitl_mode = sitl_mode
        self.bridge = MAVLinkBridge()
        if sitl_mode:
            self.bridge.config["connection"]["port"] = "tcp:127.0.0.1:5760"
            self.bridge.config["connection"]["sitl_mode"] = True

        fc_path = str(Path(__file__).parent.parent / "config" / "flight_config.yaml")
        self.fc = FlightController(FlightConfig.from_yaml(fc_path))
        self.config = self.fc.config

        if sitl_mode:
            self.odom = RTABMapOdom(mode="sitl", config={"bridge": self.bridge})
        else:
            self.odom = RTABMapOdom(mode="ros2")

        self.connected = False
        self.flying = False
        self.flight_log = []
        self.status_msg = "Not connected"
        self._fly_thread = None

    def connect(self):
        """Connect to drone."""
        self.status_msg = "Connecting..."
        if not self.bridge.connect():
            self.status_msg = "Connection failed!"
            return False
        if not self.bridge.wait_for_heartbeat():
            self.status_msg = "No heartbeat!"
            return False

        if self.sitl_mode:
            self.bridge.request_all_streams(10)
            time.sleep(2)
            self.bridge.wait_ekf_ready(timeout=60)

        self.odom.start()
        time.sleep(2)
        self.connected = True
        self.status_msg = "Connected & ready"
        return True

    def takeoff(self):
        """Arm + takeoff."""
        self.status_msg = "Taking off..."
        self.bridge.set_mode("GUIDED")
        time.sleep(1)
        retries = 10 if self.sitl_mode else 3
        if not self.bridge.arm(retries=retries):
            self.status_msg = "ARM FAILED!"
            return False
        time.sleep(2)
        self.bridge.takeoff(self.config.takeoff_altitude)
        time.sleep(3)
        self.status_msg = "Hovering"
        return True

    def fly_to(self, target_n, target_e):
        """Fly to NED position (non-blocking)."""
        if self._fly_thread and self._fly_thread.is_alive():
            self.status_msg = "Already flying!"
            return

        def _fly():
            self.flying = True
            self.fc.reset()
            data = self.odom.get()
            dist = math.sqrt((target_n - data.x)**2 + (target_e - data.y)**2)
            self.status_msg = f"Flying to ({target_n:.1f}, {target_e:.1f}) — {dist:.1f}m"

            start = time.time()
            dt = 0.05  # 20Hz control loop for smooth flight
            last_log = 0

            for step in range(1200):  # 60s max at 20Hz
                if not self.flying:
                    break
                loop_start = time.time()
                data = self.odom.get()
                dist = math.sqrt((target_n - data.x)**2 + (target_e - data.y)**2)

                if dist < self.config.arrival_radius:
                    self.bridge.send_velocity_ned(0, 0, 0)
                    elapsed = time.time() - start
                    self.status_msg = f"✓ Arrived! ({elapsed:.1f}s, error={dist:.2f}m)"
                    self.flight_log.append({
                        "target": [target_n, target_e],
                        "error": round(dist, 2),
                        "time": round(elapsed, 1),
                        "ok": True,
                    })
                    self.flying = False
                    return

                # PID returns (vx, vy, vz, yaw_heading)
                result = self.fc.compute_waypoint_velocity(
                    target_n, target_e, self.config.cruise_altitude,
                    data.x, data.y, data.altitude, dt=dt
                )
                vx, vy, vz = result[0], result[1], result[2]
                yaw = result[3] if len(result) > 3 else 0.0
                
                # Send with LOCKED YAW (nose points toward target)
                self.bridge.send_velocity_ned_yaw(vx, vy, vz, yaw)

                # Log progress
                now = time.time()
                if now - last_log > 5.0:
                    self.status_msg = f"Flying... {dist:.1f}m to go | {data.speed_horizontal:.1f}m/s"
                    last_log = now

                sleep_t = dt - (time.time() - loop_start)
                if sleep_t > 0:
                    time.sleep(sleep_t)

            self.bridge.send_velocity_ned(0, 0, 0)
            self.status_msg = "✗ Timeout"
            self.flight_log.append({
                "target": [target_n, target_e],
                "error": round(dist, 2),
                "time": 60,
                "ok": False,
            })
            self.flying = False

        self._fly_thread = threading.Thread(target=_fly, daemon=True)
        self._fly_thread.start()

    def land(self):
        """Land at current position."""
        self.flying = False
        self.bridge.send_velocity_ned(0, 0, 0)
        time.sleep(0.5)
        self.bridge.land()
        self.status_msg = "Landing..."
        self.bridge.wait_landed(timeout=30)
        self.status_msg = "Landed"

    def go_home(self):
        """Fly home then land."""
        self.fly_to(0, 0)

    def hover(self):
        """Stop and hover."""
        self.flying = False
        self.bridge.send_velocity_ned(0, 0, 0)
        self.status_msg = "Hovering"

    def get_state(self):
        """Get full drone state as dict (for web API)."""
        data = self.odom.get()
        return {
            "connected": self.connected,
            "flying": self.flying,
            "status": self.status_msg,
            "n": round(data.x, 2),
            "e": round(data.y, 2),
            "alt": round(data.altitude, 2),
            "speed": round(data.speed_horizontal, 2),
            "home_dist": round(data.distance_from_home, 2),
            "vx": round(data.vx, 2),
            "vy": round(data.vy, 2),
            "log": self.flight_log[-10:],  # Last 10 entries
        }


# Global controller
drone = None

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>gNNS Drone Control</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg: #0a0e1a;
    --card: #111827;
    --card-border: #1f2937;
    --accent: #3b82f6;
    --accent-hover: #2563eb;
    --green: #10b981;
    --red: #ef4444;
    --yellow: #f59e0b;
    --text: #f3f4f6;
    --text-dim: #9ca3af;
    --orange: #f97316;
  }
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: 'Inter', sans-serif;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* Header */
  .header {
    background: linear-gradient(135deg, #1e3a5f 0%, #0f172a 100%);
    padding: 16px 24px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    border-bottom: 1px solid var(--card-border);
  }
  .header h1 { font-size: 20px; font-weight: 700; }
  .header h1 span { color: var(--accent); }
  .status-badge {
    padding: 6px 14px;
    border-radius: 20px;
    font-size: 13px;
    font-weight: 600;
    background: rgba(16,185,129,0.15);
    color: var(--green);
    border: 1px solid rgba(16,185,129,0.3);
  }
  .status-badge.error { background: rgba(239,68,68,0.15); color: var(--red); border-color: rgba(239,68,68,0.3); }
  .status-badge.warn { background: rgba(245,158,11,0.15); color: var(--yellow); border-color: rgba(245,158,11,0.3); }

  /* Grid */
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 16px;
    padding: 16px 24px;
    max-width: 1200px;
    margin: 0 auto;
  }
  @media (max-width: 768px) { .grid { grid-template-columns: 1fr; } }

  /* Cards */
  .card {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: 12px;
    padding: 20px;
  }
  .card h2 {
    font-size: 14px;
    font-weight: 600;
    color: var(--text-dim);
    text-transform: uppercase;
    letter-spacing: 1px;
    margin-bottom: 16px;
  }

  /* Position display */
  .pos-grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 12px;
  }
  .pos-item {
    text-align: center;
    padding: 12px;
    background: rgba(59,130,246,0.08);
    border-radius: 8px;
    border: 1px solid rgba(59,130,246,0.15);
  }
  .pos-item .label { font-size: 11px; color: var(--text-dim); margin-bottom: 4px; }
  .pos-item .value { font-size: 22px; font-weight: 700; font-variant-numeric: tabular-nums; }
  .pos-item .unit { font-size: 12px; color: var(--text-dim); margin-left: 2px; }

  /* Fly To card */
  .fly-form {
    display: flex;
    gap: 10px;
    align-items: flex-end;
    flex-wrap: wrap;
  }
  .fly-form .field { flex: 1; min-width: 100px; }
  .fly-form label { display: block; font-size: 12px; color: var(--text-dim); margin-bottom: 6px; }
  .fly-form input {
    width: 100%;
    padding: 10px 14px;
    background: var(--bg);
    border: 1px solid var(--card-border);
    border-radius: 8px;
    color: var(--text);
    font-size: 16px;
    font-weight: 600;
    font-family: 'Inter', sans-serif;
    transition: border-color 0.2s;
  }
  .fly-form input:focus { outline: none; border-color: var(--accent); }

  /* Buttons */
  .btn {
    padding: 10px 20px;
    border: none;
    border-radius: 8px;
    font-size: 14px;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.2s;
    font-family: 'Inter', sans-serif;
  }
  .btn:active { transform: scale(0.97); }
  .btn-fly { background: var(--accent); color: white; }
  .btn-fly:hover { background: var(--accent-hover); }
  .btn-land { background: var(--red); color: white; }
  .btn-land:hover { background: #dc2626; }
  .btn-takeoff { background: var(--green); color: white; }
  .btn-takeoff:hover { background: #059669; }
  .btn-home { background: var(--orange); color: white; }
  .btn-home:hover { background: #ea580c; }
  .btn-hover { background: var(--card-border); color: var(--text); }
  .btn-hover:hover { background: #374151; }

  .btn-row { display: flex; gap: 10px; flex-wrap: wrap; }

  /* Map */
  .map-container {
    position: relative;
    width: 100%;
    height: 300px;
    background: rgba(0,0,0,0.3);
    border-radius: 8px;
    overflow: hidden;
    border: 1px solid var(--card-border);
  }
  .map-canvas {
    width: 100%;
    height: 100%;
  }

  /* Log */
  .log-table { width: 100%; border-collapse: collapse; font-size: 13px; }
  .log-table th { text-align: left; color: var(--text-dim); font-weight: 500; padding: 8px; border-bottom: 1px solid var(--card-border); }
  .log-table td { padding: 8px; border-bottom: 1px solid rgba(31,41,55,0.5); font-variant-numeric: tabular-nums; }
  .log-table .ok { color: var(--green); }
  .log-table .fail { color: var(--red); }

  /* Full width sections */
  .full-width { grid-column: 1 / -1; }
</style>
</head>
<body>

<div class="header">
  <h1><span>gNNS</span> Drone Control</h1>
  <div class="status-badge" id="statusBadge">Connecting...</div>
</div>

<div class="grid">
  <!-- Position -->
  <div class="card">
    <h2>Position</h2>
    <div class="pos-grid">
      <div class="pos-item">
        <div class="label">NORTH</div>
        <div class="value"><span id="posN">0.00</span><span class="unit">m</span></div>
      </div>
      <div class="pos-item">
        <div class="label">EAST</div>
        <div class="value"><span id="posE">0.00</span><span class="unit">m</span></div>
      </div>
      <div class="pos-item">
        <div class="label">ALTITUDE</div>
        <div class="value"><span id="posAlt">0.0</span><span class="unit">m</span></div>
      </div>
      <div class="pos-item">
        <div class="label">SPEED</div>
        <div class="value"><span id="posSpd">0.00</span><span class="unit">m/s</span></div>
      </div>
      <div class="pos-item">
        <div class="label">FROM HOME</div>
        <div class="value"><span id="posDist">0.0</span><span class="unit">m</span></div>
      </div>
      <div class="pos-item">
        <div class="label">STATUS</div>
        <div class="value" style="font-size:14px;" id="posStatus">—</div>
      </div>
    </div>
  </div>

  <!-- Fly To -->
  <div class="card">
    <h2>Fly To Coordinates</h2>
    <div class="fly-form" id="flyForm">
      <div class="field">
        <label>North (m)</label>
        <input type="number" id="inputN" step="0.5" value="10" placeholder="N">
      </div>
      <div class="field">
        <label>East (m)</label>
        <input type="number" id="inputE" step="0.5" value="5" placeholder="E">
      </div>
      <button class="btn btn-fly" onclick="flyTo()">Fly →</button>
    </div>
    <div style="margin-top:16px;">
      <div class="btn-row">
        <button class="btn btn-takeoff" onclick="cmd('takeoff')">Takeoff</button>
        <button class="btn btn-land" onclick="cmd('land')">Land</button>
        <button class="btn btn-home" onclick="cmd('home')">Home</button>
        <button class="btn btn-hover" onclick="cmd('hover')">Hover</button>
      </div>
    </div>
    <div style="margin-top:12px; font-size:12px; color:var(--text-dim);">
      Quick: 
      <a href="#" onclick="flyPreset(10,0)" style="color:var(--accent)">N10</a> · 
      <a href="#" onclick="flyPreset(0,10)" style="color:var(--accent)">E10</a> · 
      <a href="#" onclick="flyPreset(10,10)" style="color:var(--accent)">NE10</a> · 
      <a href="#" onclick="flyPreset(-10,0)" style="color:var(--accent)">S10</a> · 
      <a href="#" onclick="flyPreset(5,5)" style="color:var(--accent)">NE5</a>
    </div>
  </div>

  <!-- Map -->
  <div class="card full-width">
    <h2>Flight Map</h2>
    <div class="map-container">
      <canvas id="mapCanvas" class="map-canvas"></canvas>
    </div>
  </div>

  <!-- Flight Log -->
  <div class="card full-width">
    <h2>Flight Log</h2>
    <table class="log-table">
      <thead><tr><th>#</th><th>Target</th><th>Error</th><th>Time</th><th>Status</th></tr></thead>
      <tbody id="logBody"></tbody>
    </table>
    <div id="logEmpty" style="color:var(--text-dim); padding:12px; font-size:13px;">No flights yet. Enter coordinates and click Fly.</div>
  </div>
</div>

<script>
const API = '';
let trail = [];
let targets = [];

function flyTo() {
  const n = parseFloat(document.getElementById('inputN').value);
  const e = parseFloat(document.getElementById('inputE').value);
  if (isNaN(n) || isNaN(e)) return;
  targets.push({n, e});
  fetch(`${API}/api/fly?n=${n}&e=${e}`);
}

function flyPreset(n, e) {
  document.getElementById('inputN').value = n;
  document.getElementById('inputE').value = e;
  flyTo();
}

function cmd(action) {
  fetch(`${API}/api/${action}`);
}

function updateState(s) {
  document.getElementById('posN').textContent = s.n.toFixed(2);
  document.getElementById('posE').textContent = s.e.toFixed(2);
  document.getElementById('posAlt').textContent = s.alt.toFixed(1);
  document.getElementById('posSpd').textContent = s.speed.toFixed(2);
  document.getElementById('posDist').textContent = s.home_dist.toFixed(1);
  document.getElementById('posStatus').textContent = s.status;

  const badge = document.getElementById('statusBadge');
  badge.textContent = s.status;
  badge.className = 'status-badge';
  if (s.status.includes('✗') || s.status.includes('FAIL')) badge.className += ' error';
  else if (s.flying) badge.className += ' warn';

  trail.push({n: s.n, e: s.e});
  if (trail.length > 500) trail.shift();

  // Log
  const tbody = document.getElementById('logBody');
  const empty = document.getElementById('logEmpty');
  if (s.log && s.log.length > 0) {
    empty.style.display = 'none';
    tbody.innerHTML = '';
    s.log.forEach((l, i) => {
      const cls = l.ok ? 'ok' : 'fail';
      tbody.innerHTML += `<tr>
        <td>${i+1}</td>
        <td>N=${l.target[0].toFixed(1)}, E=${l.target[1].toFixed(1)}</td>
        <td>${l.error.toFixed(2)}m</td>
        <td>${l.time.toFixed(1)}s</td>
        <td class="${cls}">${l.ok ? '✓' : '✗'}</td>
      </tr>`;
    });
  }

  drawMap(s);
}

function drawMap(s) {
  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');
  canvas.width = canvas.offsetWidth * window.devicePixelRatio;
  canvas.height = canvas.offsetHeight * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
  const W = canvas.offsetWidth;
  const H = canvas.offsetHeight;

  ctx.clearRect(0, 0, W, H);

  // Auto-scale
  let allPts = [...trail, {n:0, e:0}, {n: s.n, e: s.e}];
  targets.forEach(t => allPts.push(t));
  let minN = Math.min(...allPts.map(p=>p.n)) - 5;
  let maxN = Math.max(...allPts.map(p=>p.n)) + 5;
  let minE = Math.min(...allPts.map(p=>p.e)) - 5;
  let maxE = Math.max(...allPts.map(p=>p.e)) + 5;
  let rangeN = maxN - minN || 20;
  let rangeE = maxE - minE || 20;
  let range = Math.max(rangeN, rangeE);
  let scale = Math.min(W, H) * 0.85 / range;
  let cx = W / 2, cy = H / 2;
  let midN = (minN + maxN) / 2, midE = (minE + maxE) / 2;

  function toXY(n, e) {
    return [cx + (e - midE) * scale, cy - (n - midN) * scale];
  }

  // Grid
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  ctx.lineWidth = 1;
  let gridStep = range > 40 ? 10 : range > 15 ? 5 : 2;
  for (let g = Math.floor(minN / gridStep) * gridStep; g <= maxN; g += gridStep) {
    let [x1, y1] = toXY(g, minE);
    let [x2, y2] = toXY(g, maxE);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  }
  for (let g = Math.floor(minE / gridStep) * gridStep; g <= maxE; g += gridStep) {
    let [x1, y1] = toXY(minN, g);
    let [x2, y2] = toXY(maxN, g);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  }

  // Home marker
  let [hx, hy] = toXY(0, 0);
  ctx.beginPath(); ctx.arc(hx, hy, 6, 0, Math.PI * 2);
  ctx.fillStyle = '#10b981'; ctx.fill();
  ctx.font = '11px Inter'; ctx.fillStyle = '#9ca3af';
  ctx.fillText('HOME', hx + 10, hy + 4);

  // Target markers
  targets.forEach((t, i) => {
    let [tx, ty] = toXY(t.n, t.e);
    ctx.beginPath(); ctx.arc(tx, ty, 5, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(249,115,22,0.6)'; ctx.fill();
    ctx.fillStyle = '#f97316';
    ctx.fillText(`WP${i+1}`, tx + 8, ty + 4);
  });

  // Trail
  if (trail.length > 1) {
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(59,130,246,0.4)';
    ctx.lineWidth = 2;
    let [sx, sy] = toXY(trail[0].n, trail[0].e);
    ctx.moveTo(sx, sy);
    for (let i = 1; i < trail.length; i++) {
      let [px, py] = toXY(trail[i].n, trail[i].e);
      ctx.lineTo(px, py);
    }
    ctx.stroke();
  }

  // Drone
  let [dx, dy] = toXY(s.n, s.e);
  ctx.beginPath(); ctx.arc(dx, dy, 8, 0, Math.PI * 2);
  ctx.fillStyle = '#3b82f6'; ctx.fill();
  ctx.beginPath(); ctx.arc(dx, dy, 12, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(59,130,246,0.4)'; ctx.lineWidth = 2; ctx.stroke();

  // Speed vector
  if (s.speed > 0.1) {
    let [vx, vy] = [s.vy * scale * 2, -s.vx * scale * 2];
    ctx.beginPath(); ctx.moveTo(dx, dy); ctx.lineTo(dx + vx, dy + vy);
    ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 2; ctx.stroke();
  }

  // Labels
  ctx.fillStyle = '#6b7280'; ctx.font = '10px Inter';
  ctx.fillText('N ↑', W - 30, 16);
  ctx.fillText('E →', W - 30, 28);
}

// Poll state
setInterval(async () => {
  try {
    const res = await fetch(`${API}/api/state`);
    const data = await res.json();
    updateState(data);
  } catch(e) {}
}, 500);

// Enter key to fly
document.getElementById('inputE').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') flyTo();
});
document.getElementById('inputN').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') document.getElementById('inputE').focus();
});
</script>
</body>
</html>"""


class WebHandler(SimpleHTTPRequestHandler):
    """HTTP handler for the drone control API."""

    def log_message(self, fmt, *args):
        pass  # Suppress default HTTP logging

    def do_GET(self):
        global drone
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        params = urllib.parse.parse_qs(parsed.query)

        if path == '/' or path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())

        elif path == '/api/state':
            state = drone.get_state() if drone else {"connected": False, "status": "No drone"}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(state).encode())

        elif path == '/api/fly':
            n = float(params.get('n', [0])[0])
            e = float(params.get('e', [0])[0])
            drone.fly_to(n, e)
            self._json_ok(f"Flying to ({n}, {e})")

        elif path == '/api/takeoff':
            threading.Thread(target=drone.takeoff, daemon=True).start()
            self._json_ok("Taking off")

        elif path == '/api/land':
            threading.Thread(target=drone.land, daemon=True).start()
            self._json_ok("Landing")

        elif path == '/api/home':
            drone.go_home()
            self._json_ok("Going home")

        elif path == '/api/hover':
            drone.hover()
            self._json_ok("Hovering")

        elif path == '/api/connect':
            threading.Thread(target=drone.connect, daemon=True).start()
            self._json_ok("Connecting")

        else:
            self.send_response(404)
            self.end_headers()

    def _json_ok(self, msg):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps({"ok": True, "msg": msg}).encode())


def main():
    global drone
    import argparse
    parser = argparse.ArgumentParser(description="gNNS Drone Web Control")
    parser.add_argument("--sitl", action="store_true", help="SITL mode")
    parser.add_argument("--port", type=int, default=5000, help="Web server port")
    parser.add_argument("--no-auto-connect", action="store_true",
                        help="Don't auto-connect on startup")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S"
    )

    drone = DroneController(sitl_mode=args.sitl)

    # Auto-connect
    if not args.no_auto_connect:
        print("Connecting to drone...")
        if drone.connect():
            print("Connected!")
        else:
            print("Connection failed! Start SITL first.")
            return

    # Start web server
    server = HTTPServer(('0.0.0.0', args.port), WebHandler)
    print(f"\n  gNNS Drone Control Panel")
    print(f"  ========================")
    print(f"  Open: http://localhost:{args.port}")
    print(f"  Press Ctrl+C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        if drone.connected:
            drone.land()
        server.shutdown()


if __name__ == '__main__':
    main()
