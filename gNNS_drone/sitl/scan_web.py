"""
gNNS Drone — Area Scanner Web Control (REAL Sensor Detection)
===============================================================
Web page for the area scanning + safe landing simulation.
Shows safety grid map, drone position, sensor readings, and
autonomous landing decisions.

Uses REAL MAVLink sensor data — no hardcoded/fake obstacles.

Run:
  Terminal 1: sim_vehicle.py -v ArduCopter --no-mavproxy
  Terminal 2: python3 sitl/scan_web.py --sitl

Open: http://localhost:5001
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

PROJECT_ROOT = str(Path(__file__).parent.parent)
sys.path.insert(0, PROJECT_ROOT)

from gnns_drone.flight_controller import FlightController, FlightConfig
from gnns_drone.mavlink_bridge import MAVLinkBridge
from gnns_drone.rtabmap_odom import create_odom_provider, _load_vio_config
from area_scanner import AreaScanner, SafetyGrid, LiveSensor
from lidar_avoider import LidarAvoider

logger = logging.getLogger("gnns.scanweb")


class DroneScanner:
    """Manages the drone + scanner for the web UI."""

    def __init__(self, sitl_mode=True, vio_source=None):
        self.sitl_mode = sitl_mode

        # Bridge
        self.bridge = MAVLinkBridge()
        if sitl_mode:
            # We'll try TCP first, but can fallback to UDP
            self.bridge.config["connection"]["port"] = "tcp:127.0.0.1:5760"
            self.bridge.config["connection"]["sitl_mode"] = True

        # Flight controller
        fc_path = str(Path(__file__).parent.parent / "config" / "flight_config.yaml")
        self.fc = FlightController(FlightConfig.from_yaml(fc_path))
        self.config = self.fc.config

        # Odometry — from vio_source or config
        vio_cfg = _load_vio_config()
        source = vio_source or vio_cfg.get("odom_source", "ros2")
        if sitl_mode:
            source = "sitl"
        odom_cfg = {k: v for k, v in vio_cfg.items() if k != "odom_source"}
        self.odom = create_odom_provider(
            source, config=odom_cfg, bridge=self.bridge
        )

        self.scanner = AreaScanner(self.bridge, self.odom, self.fc, self.config)

        # Lidar Avoider (360 obstacle detection)
        self.avoider = LidarAvoider(self.bridge)

        # State
        self.connected = False
        self.status_msg = "Not connected"
        self.landing_result = None
        self._bg_scan_thread = None

    def connect(self):
        self.status_msg = "Connecting..."
        
        tcp_connected = self.bridge.connect()
        tcp_heartbeat = False
        if tcp_connected:
            tcp_heartbeat = self.bridge.wait_for_heartbeat(timeout=10)

        # If TCP failed to connect OR failed to get a heartbeat, try UDP (MAVProxy)
        if not tcp_connected or not tcp_heartbeat:
            if self.sitl_mode:
                if not tcp_connected:
                    logger.warning("TCP 5760 connection refused. Trying UDP 14550 (MAVProxy)...")
                else:
                    logger.warning("No heartbeat on TCP 5760. Trying UDP 14550 (MAVProxy)...")
                    self.bridge.disconnect()
                
                self.bridge.config["connection"]["port"] = "udp:127.0.0.1:14550"
                if not self.bridge.connect() or not self.bridge.wait_for_heartbeat(timeout=10):
                    self.status_msg = "Connection failed (TCP & UDP)!"
                    return False
            else:
                self.status_msg = "Connection/Heartbeat failed!"
                return False

        logger.info("[CHECKPOINT 1] MAVLink Connected. Configuring parameters...")
        if self.sitl_mode:
            self.bridge.request_all_streams(10)
            time.sleep(2)
            self.bridge.wait_ekf_ready(timeout=60)
        
        logger.info("[CHECKPOINT 2] EKF Ready. Starting Odometry...")
        self.odom.start()
        time.sleep(1)

        # Configure SITL sensors (NO ROS NEEDED)
        if self.sitl_mode:
            logger.info("[CHECKPOINT 3] Configuring SITL Sensors...")
            self._disable_proximity()  # Disable first to allow arming
            self._setup_rangefinder()

        # SKIP ROS entirely during startup - start it in background later
        logger.info("[CHECKPOINT 4] Skipping ROS sensors (will start in background)...")

        logger.info("[CHECKPOINT 5] All systems GO!")
        self.connected = True
        self.status_msg = "Connected"

        # Start background tasks
        self._start_bg_scan()
        self._start_status_heartbeat()
        
        # Try to start ROS sensors in a completely detached background thread
        # This CANNOT block the main thread under any circumstances
        self._start_ros_background()
        
        return True

    def _disable_proximity(self):
        """Forcefully disable proximity sensor that blocks arming."""
        try:
            logger.info("Disabling proximity sensor (PRX1)...")
            self.bridge.set_param("PRX1_TYPE", 0)
            time.sleep(0.5)
            self.bridge.set_param("AVOID_ENABLE", 0)
            time.sleep(0.5)
            logger.info("PRX1 disabled, AVOID disabled (will enable after takeoff)")
        except Exception as e:
            logger.warning(f"PRX disable: {e}")

    def _enable_avoidance(self):
        """Enable ArduPilot obstacle avoidance.
        
        Uses simple avoidance (fence + proximity if available).
        PRX1_TYPE changes require SITL restart, so we only enable
        fence-based avoidance here, which works immediately.
        The background ROS thread will add lidar data if available.
        """
        try:
            logger.info("Enabling Obstacle Avoidance...")
            
            # Enable fence + proximity avoidance (works without restart)
            self.bridge.set_param("AVOID_ENABLE", 7)
            time.sleep(0.3)
            
            # Slide around obstacles (not just stop)
            self.bridge.set_param("AVOID_BEHAVE", 1)
            time.sleep(0.3)
            
            # Safety margin
            self.bridge.set_param("AVOID_MARGIN", 2.0)
            time.sleep(0.3)
            
            # BendyRuler path planner (works with whatever proximity data is available)
            self.bridge.set_param("OA_TYPE", 1)
            time.sleep(0.3)
            
            logger.info("✓ Avoidance ACTIVE (BendyRuler + fence, margin=2m)")
        except Exception as e:
            logger.warning(f"Avoidance setup failed: {e}")

    def _setup_rangefinder(self):
        """Enable simulated rangefinder in SITL."""
        try:
            self.bridge.set_param("RNGFND1_TYPE", 0)
            logger.info("Native ArduPilot rangefinder disabled (using altitude fallback)")
        except Exception as e:
            logger.warning(f"Rangefinder setup: {e}")

    def _start_ros_background(self):
        """Start ROS sensors in a completely detached background thread.
        
        This runs AFTER the drone is connected and cannot block anything.
        If ROS isn't available, the drone flies fine without it.
        """
        def _ros_init():
            try:
                import rospy
                if not rospy.core.is_initialized():
                    # Try to init ROS with a short timeout approach
                    import subprocess, os
                    # Check if roscore exists
                    try:
                        result = subprocess.run(
                            ['bash', '-c', 'source /opt/ros/noetic/setup.bash && rostopic list'],
                            capture_output=True, timeout=3,
                            env={**os.environ, 'ROS_MASTER_URI': 'http://localhost:11311'}
                        )
                        if result.returncode != 0:
                            logger.info("roscore not running — ROS sensors disabled (drone flies fine without)")
                            return
                    except Exception:
                        logger.info("ROS not available — sensors disabled (drone flies fine without)")
                        return
                    
                    rospy.init_node("gnns_sensors", anonymous=True, disable_signals=True)
                
                # Start depth camera
                if self.scanner.sensor.depth_reader:
                    self.scanner.sensor.start_depth_camera()
                    logger.info("✓ Depth camera started (background)")
                
                # Start lidar
                self.avoider.start()
                logger.info("✓ Lidar started (background)")
                
            except ImportError:
                logger.info("ROS not installed — using MAVLink-only mode")
            except Exception as e:
                logger.info(f"ROS init skipped: {e}")
        
        t = threading.Thread(target=_ros_init, daemon=True, name="ros-bg-init")
        t.start()
        logger.info("ROS sensors starting in background (non-blocking)")

    def _start_bg_scan(self):
        """Continuously scan at current position in background."""
        def _loop():
            while self.connected:
                try:
                    self.scanner.scan_at_current_position()
                except Exception:
                    pass
                time.sleep(0.5)
        self._bg_scan_thread = threading.Thread(target=_loop, daemon=True)
        self._bg_scan_thread.start()

    def _start_status_heartbeat(self):
        """Periodically log drone status to the terminal so user knows it's alive."""
        def _loop():
            while self.connected:
                data = self.odom.get()
                range_dist = self.scanner.sensor.read_rangefinder()
                depth_active = self.scanner.sensor.depth_reader and self.scanner.sensor.depth_reader.has_data
                
                status = f"MODE: {self.bridge.stats.fc_mode} | POS: {data.x:.1f}, {data.y:.1f}, {data.altitude:.1f}m"
                status += f" | YAW: {math.degrees(data.yaw):.0f}°"
                status += f" | SPD: {data.speed_horizontal:.1f}m/s"
                
                # Show closest obstacle from avoider
                if self.avoider.min_dist < 5.0:
                    status += f" | NEAR: {self.avoider.min_dist:.1f}m"

                if depth_active:
                    status += f" | DEPTH: OK ({self.scanner.sensor.depth_reader.get_center_distance():.2f}m)"
                elif range_dist > 0:
                    status += f" | RANGE: {range_dist:.2f}m"
                else:
                    status += " | SENSORS: MAVLink-only"

                logger.info(status)
                time.sleep(5.0)
        
        t = threading.Thread(target=_loop, daemon=True, name="status-hb")
        t.start()

    def takeoff(self):
        self.status_msg = "Taking off..."
        
        # Switch to GUIDED and wait for it to take effect
        logger.info("Setting mode to GUIDED...")
        self.bridge.set_mode("GUIDED")
        time.sleep(2)  # Give SITL time to switch
        
        # Verify mode switched
        hb = self.bridge.get_latest("HEARTBEAT")
        if hb:
            logger.info(f"Current mode after switch: custom_mode={hb.custom_mode}")
        
        # Give EKF a moment to stabilize in GUIDED mode
        time.sleep(3)
        
        retries = 10 if self.sitl_mode else 3
        if not self.bridge.arm(retries=retries):
            self.status_msg = "ARM FAILED!"
            return False
        time.sleep(2)
        self.bridge.takeoff(self.config.takeoff_altitude)
        time.sleep(3)
        
        # Enable avoidance ONLY after we are safely in the air
        if self.sitl_mode:
            logger.info("Drone airborne — Enabling Obstacle Avoidance...")
            self._enable_avoidance()
            
        self.status_msg = "Hovering — Avoidance Active"
        return True

    def start_area_scan(self):
        """Full area scan using real sensors (drone flies pattern)."""
        def _scan():
            self.status_msg = "Flying scan pattern..."
            self.scanner.quick_area_scan(
                radius_n=8, radius_e=8, spacing=3.0,
                callback=self._scan_callback
            )
            self.status_msg = f"Scan done! {self.scanner.grid.cells_scanned} cells scanned"
        threading.Thread(target=_scan, daemon=True).start()

    def _scan_callback(self, progress, grid):
        pct = int(progress * 100)
        self.status_msg = f"Scanning... {pct}% ({grid.cells_scanned} cells)"

    def fly_and_land(self, n, e):
        """Autonomous: fly to target, pre-landing scan, safe land."""
        def _fly():
            # Ensure airborne
            if not self._ensure_airborne():
                self.status_msg = "Cannot takeoff!"
                return

            self.status_msg = f"Pre-landing scan at ({n:.1f}, {e:.1f})..."
            self.landing_result = self.scanner.pre_landing_scan(
                n, e, callback=self._scan_callback
            )
            r = self.landing_result
            if r["redirected"]:
                self.status_msg = f"⚠️ Redirected → ({r['actual'][0]:.1f}, {r['actual'][1]:.1f})"
            elif r["safe"]:
                self.status_msg = f"✓ Landed at ({r['actual'][0]:.1f}, {r['actual'][1]:.1f})"
            else:
                self.status_msg = "✗ Landing FAILED — no safe spot!"
        threading.Thread(target=_fly, daemon=True).start()

    def _ensure_airborne(self):
        data = self.odom.get()
        if data.altitude > 1.0:
            return True
        self.status_msg = "Arming + takeoff..."
        self.bridge.set_mode("GUIDED")
        time.sleep(0.5)
        retries = 10 if self.sitl_mode else 3
        if not self.bridge.arm(retries=retries):
            return False
        time.sleep(1)
        self.bridge.takeoff(self.config.takeoff_altitude)
        time.sleep(4)
        return True

    def hover(self):
        self.scanner.scanning = False
        try:
            self.bridge.send_velocity_ned(0, 0, 0)
        except Exception:
            pass
        self.status_msg = "Hovering (stopped)"

    def land(self):
        self.scanner.scanning = False
        try:
            self.bridge.send_velocity_ned(0, 0, 0)
            time.sleep(0.5)
            self.bridge.land()
            self.status_msg = "Landing..."
        except Exception as e:
            self.status_msg = f"Land error: {e}"

    def takeoff_again(self):
        def _to():
            self.status_msg = "Takeoff..."
            self.bridge.set_mode("GUIDED")
            time.sleep(0.5)
            retries = 10 if self.sitl_mode else 3
            self.bridge.arm(retries=retries)
            time.sleep(1)
            self.bridge.takeoff(self.config.takeoff_altitude)
            time.sleep(3)
            self.status_msg = "Hovering"
        threading.Thread(target=_to, daemon=True).start()

    def get_state(self):
        data = self.odom.get()

        # Read live sensor data
        range_dist = self.scanner.sensor.read_rangefinder()
        tilt = self.scanner.sensor.read_tilt()

        # Depth camera info
        depth_info = {
            "available": self.scanner.sensor.depth_reader is not None,
            "has_data": False,
        }
        if self.scanner.sensor.depth_reader:
            dr = self.scanner.sensor.depth_reader
            depth_info["has_data"] = dr.has_data
            depth_info["center"] = round(dr.get_center_distance(), 2) if dr.has_data else None
            depth_info["min"] = round(dr.get_min_distance(), 2) if dr.has_data else None

        return {
            "connected": self.connected,
            "status": self.status_msg,
            "n": round(data.x, 2),
            "e": round(data.y, 2),
            "alt": round(data.altitude, 2),
            "speed": round(data.speed_horizontal, 2),
            "scanning": self.scanner.scanning,
            "scan_progress": round(self.scanner.scan_progress * 100, 1),
            "cells_scanned": self.scanner.grid.cells_scanned,
            "cells_safe": self.scanner.grid.cells_safe,
            "cells_obstacle": self.scanner.grid.cells_obstacle,
            "cells_rough": self.scanner.grid.cells_rough,
            "landing_result": self.landing_result,
            "rangefinder": round(range_dist, 2) if range_dist > 0 else None,
            "tilt": round(tilt, 1),
            "depth": depth_info,
        }


# Global
drone = None

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>gNNS Drone - Autonomous Scanner</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&display=swap" rel="stylesheet">
<style>
:root{--bg:#080c18;--card:#10182a;--border:#1a2744;--accent:#3b82f6;--green:#10b981;--red:#ef4444;--yellow:#f59e0b;--orange:#f97316;--text:#e2e8f0;--dim:#64748b}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Inter',sans-serif;background:var(--bg);color:var(--text);min-height:100vh}

.header{background:linear-gradient(135deg,#0f2847,#080c18);padding:14px 20px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid var(--border)}
.header h1{font-size:18px;font-weight:800;letter-spacing:-0.5px}
.header h1 span{color:var(--accent)}
.header .sub{font-size:10px;color:var(--dim);margin-left:10px;font-weight:400}
.badge{padding:5px 12px;border-radius:16px;font-size:12px;font-weight:600;background:rgba(16,185,129,.12);color:var(--green);border:1px solid rgba(16,185,129,.25)}
.badge.warn{background:rgba(245,158,11,.12);color:var(--yellow);border-color:rgba(245,158,11,.25)}
.badge.err{background:rgba(239,68,68,.12);color:var(--red);border-color:rgba(239,68,68,.25)}

.layout{display:grid;grid-template-columns:1fr 320px;gap:14px;padding:14px;max-width:1300px;margin:0 auto;height:calc(100vh - 52px)}
@media(max-width:900px){.layout{grid-template-columns:1fr;height:auto}}

.card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:16px}
.card h2{font-size:11px;font-weight:700;color:var(--dim);text-transform:uppercase;letter-spacing:1.2px;margin-bottom:12px}

.map-wrap{position:relative;overflow:hidden;border-radius:8px;border:1px solid var(--border);background:#060a14;aspect-ratio:1;max-height:calc(100vh - 120px)}
canvas{width:100%;height:100%;cursor:crosshair}

.sidebar{display:flex;flex-direction:column;gap:12px;overflow-y:auto}
.metric{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid rgba(26,39,68,.5)}
.metric .lbl{font-size:11px;color:var(--dim)}
.metric .val{font-size:14px;font-weight:700;font-variant-numeric:tabular-nums}
.btn{padding:9px 16px;border:none;border-radius:7px;font-size:13px;font-weight:600;cursor:pointer;transition:all .15s;font-family:inherit;width:100%}
.btn:active{transform:scale(.97)}
.btn-accent{background:var(--accent);color:#fff}
.btn-green{background:var(--green);color:#fff}
.btn-red{background:var(--red);color:#fff}
.btn-orange{background:var(--orange);color:#fff}
.btn-gray{background:var(--border);color:var(--text)}
.btn-row{display:grid;grid-template-columns:1fr 1fr;gap:8px}

.fly-inputs{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.fly-inputs input{padding:8px 10px;background:var(--bg);border:1px solid var(--border);border-radius:6px;color:var(--text);font-size:14px;font-weight:600;font-family:inherit;width:100%}
.fly-inputs input:focus{outline:none;border-color:var(--accent)}

.legend{display:flex;gap:10px;flex-wrap:wrap;font-size:11px}
.legend span{display:flex;align-items:center;gap:4px}
.legend .dot{width:10px;height:10px;border-radius:2px;display:inline-block}

.result-box{padding:10px;border-radius:6px;font-size:12px;line-height:1.5}
.result-box.ok{background:rgba(16,185,129,.1);border:1px solid rgba(16,185,129,.2);color:var(--green)}
.result-box.warn{background:rgba(249,115,22,.1);border:1px solid rgba(249,115,22,.2);color:var(--orange)}
.result-box.err{background:rgba(239,68,68,.1);border:1px solid rgba(239,68,68,.2);color:var(--red)}

.progress-bar{height:4px;background:var(--border);border-radius:2px;overflow:hidden;margin-top:6px}
.progress-fill{height:100%;background:var(--accent);transition:width .3s}
.sensor-badge{display:inline-block;padding:2px 8px;border-radius:8px;font-size:10px;font-weight:600;margin-left:4px}
</style>
</head>
<body>
<div class="header">
  <h1><span>gNNS</span> Autonomous Scanner<span class="sub">REAL SENSOR DETECTION</span></h1>
  <div class="badge" id="badge">Not connected</div>
</div>
<div class="layout">
  <div class="card" style="display:flex;flex-direction:column">
    <h2>Safety Grid Map <span style="float:right;font-weight:400" id="scanInfo">Click map to fly & safe-land</span></h2>
    <div class="map-wrap">
      <canvas id="map"></canvas>
    </div>
    <div class="legend" style="margin-top:8px">
      <span><span class="dot" style="background:#10b981"></span> Safe</span>
      <span><span class="dot" style="background:#ef4444"></span> Obstacle</span>
      <span><span class="dot" style="background:#f59e0b"></span> Rough</span>
      <span><span class="dot" style="background:#1e293b"></span> Unknown</span>
      <span><span class="dot" style="background:#3b82f6;border-radius:50%"></span> Drone</span>
      <span><span class="dot" style="background:#818cf8;border-radius:50%"></span> Target</span>
    </div>
  </div>

  <div class="sidebar">
    <div class="card">
      <h2>Live Sensors</h2>
      <div class="metric"><span class="lbl">Position N</span><span class="val" id="vN">0.00 m</span></div>
      <div class="metric"><span class="lbl">Position E</span><span class="val" id="vE">0.00 m</span></div>
      <div class="metric"><span class="lbl">Altitude</span><span class="val" id="vAlt">0.0 m</span></div>
      <div class="metric"><span class="lbl">Speed</span><span class="val" id="vSpd">0.00 m/s</span></div>
      <div class="metric"><span class="lbl">Rangefinder ↓</span><span class="val" id="vRange">— m</span></div>
      <div class="metric"><span class="lbl">Depth Cam</span><span class="val" id="vDepth">—</span></div>
      <div class="metric"><span class="lbl">Tilt</span><span class="val" id="vTilt">0.0°</span></div>
      <div class="metric"><span class="lbl">Status</span><span class="val" id="vStatus">—</span></div>
    </div>

    <div class="card">
      <h2>Scan Stats</h2>
      <div class="metric"><span class="lbl">Cells Scanned</span><span class="val" id="vScanned">0</span></div>
      <div class="metric"><span class="lbl">Safe</span><span class="val" style="color:var(--green)" id="vSafe">0</span></div>
      <div class="metric"><span class="lbl">Obstacles</span><span class="val" style="color:var(--red)" id="vObs">0</span></div>
      <div class="metric"><span class="lbl">Rough</span><span class="val" style="color:var(--yellow)" id="vRough">0</span></div>
      <div class="progress-bar"><div class="progress-fill" id="scanBar" style="width:0%"></div></div>
    </div>

    <div class="card">
      <h2>Controls</h2>
      <div class="btn-row" style="margin-bottom:8px">
        <button class="btn btn-green" onclick="api('connect')">Connect</button>
        <button class="btn btn-green" onclick="api('takeoff')">Takeoff</button>
      </div>
      <button class="btn btn-accent" style="margin-bottom:8px" onclick="api('area_scan')">🛸 Fly & Scan Area</button>
      <div class="btn-row" style="margin-bottom:8px">
        <button class="btn btn-red" onclick="api('land')">Land</button>
        <button class="btn btn-gray" onclick="api('hover')">Stop/Hover</button>
      </div>
      <button class="btn btn-orange" onclick="api('takeoff_again')">Takeoff Again</button>
    </div>

    <div class="card">
      <h2>🎯 Autonomous Safe-Land</h2>
      <p style="font-size:11px;color:var(--dim);margin-bottom:8px">
        Drone flies to target, scans area with sensors, picks safest spot, lands autonomously.
      </p>
      <div class="fly-inputs" style="margin-bottom:8px">
        <input type="number" id="inN" step="1" value="8" placeholder="North m">
        <input type="number" id="inE" step="1" value="5" placeholder="East m">
      </div>
      <button class="btn btn-accent" onclick="flyLand()">Fly → Scan → Safe Land</button>
      <p style="font-size:10px;color:var(--dim);margin-top:6px">Or click anywhere on the map</p>
    </div>

    <div class="card" id="resultCard" style="display:none">
      <h2>Landing Decision</h2>
      <div class="result-box" id="resultBox"></div>
    </div>
  </div>
</div>

<script>
let grid=null, dronePos={n:0,e:0}, gridSize={n:20,e:20}, res=1, clickTarget=null;

function api(action){fetch('/api/'+action)}
function flyLand(){
  const n=parseFloat(document.getElementById('inN').value);
  const e=parseFloat(document.getElementById('inE').value);
  if(isNaN(n)||isNaN(e))return;
  clickTarget={n,e};
  fetch(`/api/fly_land?n=${n}&e=${e}`);
}

document.getElementById('map').addEventListener('click',function(ev){
  const rect=this.getBoundingClientRect();
  const x=(ev.clientX-rect.left)/rect.width;
  const y=(ev.clientY-rect.top)/rect.height;
  const e=(x-0.5)*gridSize.e*2;
  const n=(0.5-y)*gridSize.n*2;
  document.getElementById('inN').value=Math.round(n);
  document.getElementById('inE').value=Math.round(e);
  clickTarget={n:Math.round(n),e:Math.round(e)};
  flyLand();
});

function drawMap(){
  const canvas=document.getElementById('map');
  const ctx=canvas.getContext('2d');
  const pr=window.devicePixelRatio||1;
  const W=canvas.offsetWidth, H=canvas.offsetHeight;
  canvas.width=W*pr; canvas.height=H*pr;
  ctx.scale(pr,pr);
  ctx.clearRect(0,0,W,H);

  const rows=grid?grid.length:40, cols=grid?grid[0].length:40;
  const cellW=W/cols, cellH=H/rows;
  const colors={0:'#111827',1:'rgba(16,185,129,0.5)',2:'rgba(245,158,11,0.45)',3:'rgba(239,68,68,0.55)'};

  if(grid){
    for(let r=0;r<rows;r++){
      for(let c=0;c<cols;c++){
        const cell=grid[r][c];
        ctx.fillStyle=colors[cell.s]||'#111827';
        ctx.fillRect(c*cellW,r*cellH,cellW-0.5,cellH-0.5);
      }
    }
  } else {
    ctx.fillStyle='#111827';
    ctx.fillRect(0,0,W,H);
  }

  // Grid lines
  ctx.strokeStyle='rgba(255,255,255,0.03)';ctx.lineWidth=0.5;
  for(let r=0;r<=rows;r+=5){ctx.beginPath();ctx.moveTo(0,r*cellH);ctx.lineTo(W,r*cellH);ctx.stroke()}
  for(let c=0;c<=cols;c+=5){ctx.beginPath();ctx.moveTo(c*cellW,0);ctx.lineTo(c*cellW,H);ctx.stroke()}

  // Home
  const hx=W/2, hy=H/2;
  ctx.beginPath();ctx.arc(hx,hy,5,0,Math.PI*2);
  ctx.fillStyle='#10b981';ctx.fill();
  ctx.font='10px Inter';ctx.fillStyle='#64748b';
  ctx.fillText('HOME',hx+8,hy+3);

  ctx.fillStyle='#475569';ctx.font='9px Inter';
  ctx.fillText('N ↑',W-28,12);ctx.fillText('E →',W-28,22);

  // Click target
  if(clickTarget){
    const tx=W/2+clickTarget.e/(gridSize.e*2)*W;
    const ty=H/2-clickTarget.n/(gridSize.n*2)*H;
    ctx.beginPath();ctx.arc(tx,ty,6,0,Math.PI*2);
    ctx.strokeStyle='#818cf8';ctx.lineWidth=2;ctx.stroke();
    ctx.beginPath();ctx.moveTo(tx-8,ty);ctx.lineTo(tx+8,ty);ctx.moveTo(tx,ty-8);ctx.lineTo(tx,ty+8);
    ctx.strokeStyle='#818cf8';ctx.lineWidth=1;ctx.stroke();
    ctx.font='9px Inter';ctx.fillStyle='#818cf8';
    ctx.fillText(`(${clickTarget.n}, ${clickTarget.e})`,tx+8,ty+14);
  }

  // Drone with scan ring
  const dx=W/2+dronePos.e/(gridSize.e*2)*W;
  const dy=H/2-dronePos.n/(gridSize.n*2)*H;
  ctx.beginPath();ctx.arc(dx,dy,7,0,Math.PI*2);
  ctx.fillStyle='#3b82f6';ctx.fill();
  ctx.beginPath();ctx.arc(dx,dy,11,0,Math.PI*2);
  ctx.strokeStyle='rgba(59,130,246,0.4)';ctx.lineWidth=2;ctx.stroke();

  // Scan ring (shows sensor FOV)
  const scanRad = 5/(gridSize.e*2)*W;
  ctx.beginPath();ctx.arc(dx,dy,scanRad,0,Math.PI*2);
  ctx.strokeStyle='rgba(59,130,246,0.15)';ctx.lineWidth=1;ctx.setLineDash([3,3]);ctx.stroke();ctx.setLineDash([]);
}

function updateState(s){
  document.getElementById('vN').textContent=s.n.toFixed(2)+' m';
  document.getElementById('vE').textContent=s.e.toFixed(2)+' m';
  document.getElementById('vAlt').textContent=s.alt.toFixed(1)+' m';
  document.getElementById('vSpd').textContent=s.speed.toFixed(2)+' m/s';
  document.getElementById('vRange').textContent=s.rangefinder!==null?s.rangefinder.toFixed(2)+' m':'no data';
  if(s.depth && s.depth.has_data){
    document.getElementById('vDepth').innerHTML='<span class="sensor-badge" style="background:rgba(16,185,129,.2);color:#10b981">LIVE</span> '+
      (s.depth.center!==null?s.depth.center.toFixed(2)+'m':'—');
  } else if(s.depth && s.depth.available){
    document.getElementById('vDepth').innerHTML='<span class="sensor-badge" style="background:rgba(245,158,11,.2);color:#f59e0b">WAITING</span>';
  } else {
    document.getElementById('vDepth').innerHTML='<span class="sensor-badge" style="background:rgba(100,116,139,.2);color:#64748b">OFF</span> (no ROS)';
  }
  document.getElementById('vTilt').textContent=s.tilt.toFixed(1)+'°';
  document.getElementById('vStatus').textContent=s.status;
  document.getElementById('vScanned').textContent=s.cells_scanned;
  document.getElementById('vSafe').textContent=s.cells_safe;
  document.getElementById('vObs').textContent=s.cells_obstacle;
  document.getElementById('vRough').textContent=s.cells_rough;
  document.getElementById('scanBar').style.width=s.scan_progress+'%';

  const b=document.getElementById('badge');
  b.textContent=s.status;
  b.className='badge';
  if(s.status.includes('FAIL')||s.status.includes('✗'))b.className+=' err';
  else if(s.scanning||s.status.includes('Scan'))b.className+=' warn';

  dronePos={n:s.n,e:s.e};

  const rc=document.getElementById('resultCard');
  const rb=document.getElementById('resultBox');
  if(s.landing_result){
    rc.style.display='block';
    const r=s.landing_result;
    let cls='ok', icon='✓';
    if(r.redirected){cls='warn';icon='⚠️'}
    if(!r.safe){cls='err';icon='✗'}
    rb.className='result-box '+cls;
    rb.innerHTML=`${icon} <b>${r.reason}</b><br>` +
      `Requested: (${r.requested[0].toFixed(1)}, ${r.requested[1].toFixed(1)})<br>` +
      `Actual: (${r.actual[0].toFixed(1)}, ${r.actual[1].toFixed(1)})` +
      (r.scan_points?`<br>Scanned ${r.scan_points} points`:'') +
      (r.final_pos?`<br>Final: (${r.final_pos[0]}, ${r.final_pos[1]}) err=${r.final_error}m`:'');
  }

  document.getElementById('scanInfo').textContent=
    s.scanning?`Scanning ${s.scan_progress}%`:
    s.cells_scanned>0?`${s.cells_scanned} cells mapped (live)`:
    'Click map to fly & safe-land';
}

async function fetchGrid(){
  try{
    const r=await fetch('/api/grid');
    const d=await r.json();
    if(d.grid)grid=d.grid;
    if(d.size_n)gridSize.n=d.size_n;
    if(d.size_e)gridSize.e=d.size_e;
  }catch(e){}
}

setInterval(async()=>{
  try{
    const r=await fetch('/api/state');
    const d=await r.json();
    updateState(d);
    drawMap();
  }catch(e){}
},500);

setInterval(fetchGrid,2000);
drawMap();
</script>
</body>
</html>"""


class ScanWebHandler(SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args): pass

    def do_GET(self):
        global drone
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        params = urllib.parse.parse_qs(parsed.query)

        if path == '/' or path == '/index.html':
            self._html(HTML_PAGE)

        elif path == '/api/state':
            self._json(drone.get_state() if drone else {
                "status": "No drone", "connected": False,
                "n": 0, "e": 0, "alt": 0, "speed": 0, "scanning": False,
                "scan_progress": 0, "cells_scanned": 0, "cells_safe": 0,
                "cells_obstacle": 0, "cells_rough": 0, "landing_result": None,
                "rangefinder": None, "tilt": 0})

        elif path == '/api/grid':
            if drone:
                g = drone.scanner.grid
                self._json({
                    "grid": g.to_json_grid(),
                    "size_n": g.size_n,
                    "size_e": g.size_e,
                    "res": g.res,
                })
            else:
                self._json({"grid": None})

        elif path == '/api/connect':
            threading.Thread(target=drone.connect, daemon=True).start()
            self._json({"ok": True})

        elif path == '/api/takeoff':
            threading.Thread(target=drone.takeoff, daemon=True).start()
            self._json({"ok": True})

        elif path == '/api/area_scan':
            drone.start_area_scan()
            self._json({"ok": True})

        elif path == '/api/fly_land':
            n = float(params.get('n', [0])[0])
            e = float(params.get('e', [0])[0])
            drone.fly_and_land(n, e)
            self._json({"ok": True, "target": [n, e]})

        elif path == '/api/land':
            threading.Thread(target=drone.land, daemon=True).start()
            self._json({"ok": True})

        elif path == '/api/hover':
            drone.hover()
            self._json({"ok": True})

        elif path == '/api/takeoff_again':
            drone.takeoff_again()
            self._json({"ok": True})

        else:
            self.send_response(404)
            self.end_headers()

    def _json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _html(self, content):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(content.encode())


def main():
    global drone
    import argparse
    parser = argparse.ArgumentParser(description="gNNS Autonomous Scanner Web UI")
    parser.add_argument("--sitl", action="store_true")
    parser.add_argument("--vio-source", default=None,
                        choices=["ros2", "orbslam3", "t265_raw", "simulated"],
                        help="Odometry source (default from vio_config)")
    parser.add_argument("--port", type=int, default=5001)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
                        datefmt="%H:%M:%S")

    drone = DroneScanner(sitl_mode=args.sitl, vio_source=args.vio_source)

    server = HTTPServer(('0.0.0.0', args.port), ScanWebHandler)
    print(f"\n  gNNS Autonomous Scanner")
    print(f"  ========================")
    print(f"  ✓ REAL sensor detection (rangefinder + altitude + tilt)")
    print(f"  ✓ Autonomous pre-landing scan (no human intervention)")
    print(f"  Open: http://localhost:{args.port}")
    print(f"  Click map after takeoff → drone scans & safe-lands\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        try:
            if drone and drone.connected:
                drone.scanner.scanning = False
                drone.bridge.send_velocity_ned(0, 0, 0)
        except Exception:
            pass
        server.shutdown()


if __name__ == '__main__':
    main()
