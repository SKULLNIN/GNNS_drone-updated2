"""
gNNS Drone — VOXL RTABMap Client
===================================
Interface to the RTABMap server running on the ModalAI VOXL board.

Two connection modes:
  "ros2_topic"  (default)
    Subscribe to /rtabmap/odom (nav_msgs/Odometry) and
    /rtabmap/info (rtabmap_msgs/Info) via ROS2 rclpy.
    Requires ROS2 and DDS bridging between Jetson and VOXL.

  "tcp_socket"
    Connect to a lightweight JSON server on the VOXL (port 4567).
    Send: {"img_b64": <base64 jpeg>, "imu": [[ax,ay,az,wx,wy,wz,t], ...]}
    Receive: {"pose": [px,py,pz,qw,qx,qy,qz], "cov": [6×6 flat],
              "closure_id": int, "map_nodes": int, "quality": 0-100}
    This mode lets VIO run even without a full ROS2 bridge.

Both modes emit the same interface:
  get_pose() → Optional[tuple[np.ndarray(3), np.ndarray(3), np.ndarray(6,6)]]
                         pos_ned             rpy_rad             covariance

  is_connected  property
  on_loop_closure(callback)   → callback(closure_id: int)
  start() / stop()

ENU → NED conversion follows the same convention as orbslam3_odom.py.
"""

import time
import math
import json
import base64
import logging
import socket
import threading
from typing import Optional, Tuple, List, Callable

import numpy as np

logger = logging.getLogger("gnns.voxl")

# ------------------------------------------------------------------ #
# ENU ↔ NED helpers (mirrors orbslam3_odom.py)
# ------------------------------------------------------------------ #

def _enu_to_ned(x: float, y: float, z: float
                ) -> Tuple[float, float, float]:
    """ROS ENU → NED: north=y, east=x, down=-z."""
    return y, x, -z


def _enu_vel_to_ned(vx: float, vy: float, vz: float
                    ) -> Tuple[float, float, float]:
    return vy, vx, -vz


def _quat_to_euler(qw: float, qx: float, qy: float, qz: float
                   ) -> Tuple[float, float, float]:
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


# ------------------------------------------------------------------ #
# Main client class
# ------------------------------------------------------------------ #

class VoxlRTABMapClient:
    """
    Client for RTABMap pose data originating from a VOXL board.

    The client runs a background thread that continuously updates its
    internal pose cache.  Consumers call get_pose() at any time.

    Loop-closure callbacks are dispatched from the background thread;
    they must be thread-safe.
    """

    def __init__(self, mode: str = "ros2_topic",
                 config: Optional[dict] = None):
        cfg = config or {}
        self.mode = mode

        # ROS2 settings
        self._odom_topic   = cfg.get("odom_topic",  "/rtabmap/odom")
        self._info_topic   = cfg.get("info_topic",  "/rtabmap/info")

        # TCP settings
        self._host         = cfg.get("voxl_host",  "10.0.0.2")
        self._port         = cfg.get("voxl_port",  4567)
        self._timeout_s    = cfg.get("timeout_s",  2.0)
        self._reconnect_s  = cfg.get("reconnect_s", 3.0)

        # Shared state
        self._pos_ned:    Optional[np.ndarray] = None      # (3,)
        self._rpy:        Optional[np.ndarray] = None      # (3,)
        self._cov6:       Optional[np.ndarray] = None      # (6,6)
        self._closure_id: int  = -1
        self._map_nodes:  int  = 0
        self._quality:    int  = 0
        self._connected:  bool = False
        self._last_update: float = 0.0

        self._lock        = threading.Lock()
        self._running     = False
        self._thread: Optional[threading.Thread] = None

        # Callbacks
        self._on_loop_closure: List[Callable[[int], None]] = []

        # Image / IMU queue for TCP mode
        self._pending_img:  Optional[bytes] = None
        self._pending_imu:  List[list]      = []
        self._queue_lock    = threading.Lock()

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self):
        """Start the background connection thread."""
        self._running = True
        if self.mode == "ros2_topic":
            self._thread = threading.Thread(
                target=self._run_ros2, daemon=True, name="voxl-ros2"
            )
        elif self.mode == "tcp_socket":
            self._thread = threading.Thread(
                target=self._run_tcp, daemon=True, name="voxl-tcp"
            )
        else:
            raise ValueError(f"Unknown VoxlRTABMapClient mode: {self.mode!r}")
        self._thread.start()
        logger.info(f"VoxlRTABMapClient started (mode={self.mode})")

    def stop(self):
        """Stop the background thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info("VoxlRTABMapClient stopped")

    # ------------------------------------------------------------------ #
    # Public interface
    # ------------------------------------------------------------------ #

    def on_loop_closure(self, callback: Callable[[int], None]):
        """
        Register a callback invoked when RTABMap detects a loop closure.

        callback(closure_id: int)  — closure_id > 0
        """
        self._on_loop_closure.append(callback)

    def get_pose(
        self,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """
        Return the most recently received pose.

        Returns:
            (pos_ned, rpy_rad, cov_6x6) or None if no pose yet / stale.

            pos_ned: np.ndarray (3,)  [north, east, down]
            rpy_rad: np.ndarray (3,)  [roll, pitch, yaw]
            cov_6x6: np.ndarray (6,6) position+attitude covariance
        """
        with self._lock:
            if self._pos_ned is None:
                return None
            if time.time() - self._last_update > 2.0:
                return None   # stale
            return (self._pos_ned.copy(),
                    self._rpy.copy(),
                    self._cov6.copy())

    def get_loop_closure_id(self) -> int:
        with self._lock:
            return self._closure_id

    def get_map_nodes(self) -> int:
        with self._lock:
            return self._map_nodes

    def get_quality(self) -> int:
        with self._lock:
            return self._quality

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------ #
    # TCP mode: send frame + IMU, receive pose
    # ------------------------------------------------------------------ #

    def send_frame(self, jpeg_bytes: bytes,
                   imu_samples: List[Tuple]) -> None:
        """
        Queue a JPEG-compressed frame and IMU samples for the TCP server.

        Args:
            jpeg_bytes:  JPEG-encoded image bytes.
            imu_samples: list of (ax,ay,az,wx,wy,wz,t) tuples.
        """
        with self._queue_lock:
            self._pending_img = jpeg_bytes
            self._pending_imu = [list(s) for s in imu_samples]

    def _run_tcp(self):
        """TCP mode: connect to VOXL JSON server and exchange frames."""
        sock: Optional[socket.socket] = None

        while self._running:
            # ---- Connect ----------------------------------------
            try:
                sock = socket.create_connection(
                    (self._host, self._port),
                    timeout=self._timeout_s,
                )
                sock.settimeout(self._timeout_s)
                self._connected = True
                logger.info(f"VOXL TCP connected to {self._host}:{self._port}")
            except (ConnectionRefusedError, OSError) as e:
                self._connected = False
                logger.warning(f"VOXL TCP connect failed: {e}; "
                               f"retry in {self._reconnect_s}s")
                time.sleep(self._reconnect_s)
                continue

            # ---- Exchange loop ----------------------------------
            try:
                while self._running:
                    with self._queue_lock:
                        img  = self._pending_img
                        imu  = list(self._pending_imu)
                        self._pending_img = None
                        self._pending_imu = []

                    if img is None:
                        time.sleep(0.01)
                        continue

                    payload = {
                        "img_b64": base64.b64encode(img).decode("ascii"),
                        "imu":     imu,
                        "ts":      time.time(),
                    }
                    data = (json.dumps(payload) + "\n").encode("utf-8")

                    sock.sendall(data)

                    resp_bytes = b""
                    while b"\n" not in resp_bytes:
                        chunk = sock.recv(4096)
                        if not chunk:
                            raise ConnectionResetError("VOXL TCP server closed")
                        resp_bytes += chunk

                    resp = json.loads(resp_bytes.split(b"\n")[0])
                    self._parse_tcp_response(resp)

            except (ConnectionResetError, socket.timeout, OSError) as e:
                self._connected = False
                logger.warning(f"VOXL TCP error: {e}; reconnecting")
            finally:
                try:
                    sock.close()
                except Exception:
                    pass
                sock = None
                self._connected = False
                if self._running:
                    time.sleep(self._reconnect_s)

    def _parse_tcp_response(self, resp: dict):
        """Parse JSON response from VOXL TCP server."""
        try:
            pose_raw   = resp.get("pose")     # [px,py,pz,qw,qx,qy,qz] ENU
            cov_raw    = resp.get("cov")      # 36 floats (6×6 flat) or None
            closure_id = int(resp.get("closure_id", -1))
            map_nodes  = int(resp.get("map_nodes",  0))
            quality    = int(resp.get("quality",    0))

            if pose_raw is None or len(pose_raw) < 7:
                return

            px, py, pz = pose_raw[0:3]
            qw, qx, qy, qz = pose_raw[3:7]

            # Convert ENU → NED
            north, east, down = _enu_to_ned(px, py, pz)
            roll, pitch, yaw  = _quat_to_euler(qw, qx, qy, qz)

            if cov_raw and len(cov_raw) == 36:
                cov6 = np.array(cov_raw, dtype=float).reshape(6, 6)
            else:
                cov6 = np.eye(6) * 0.01

            self._store(north, east, down, roll, pitch, yaw,
                        cov6, closure_id, map_nodes, quality)

        except Exception as e:
            logger.error(f"VoxlRTABMapClient TCP parse error: {e}")

    # ------------------------------------------------------------------ #
    # ROS2 mode
    # ------------------------------------------------------------------ #

    def _run_ros2(self):
        """ROS2 mode: subscribe to /rtabmap/odom and /rtabmap/info."""
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from nav_msgs.msg import Odometry
        except ImportError:
            logger.error(
                "rclpy not found — VoxlRTABMapClient cannot use ros2_topic mode. "
                "Install ROS2 or switch to 'tcp_socket' mode."
            )
            self._connected = False
            return

        if not rclpy.ok():
            rclpy.init()

        node = rclpy.create_node("gnns_voxl_subscriber")
        self._connected = True

        # Try to import rtabmap_msgs Info (optional)
        try:
            from rtabmap_msgs.msg import Info as RTABInfo
            _has_rtabinfo = True
        except ImportError:
            _has_rtabinfo = False
            logger.info("rtabmap_msgs.msg.Info not found; "
                        "loop-closure data unavailable via ROS2")

        def odom_callback(msg: Odometry):
            try:
                pos = msg.pose.pose.position
                q   = msg.pose.pose.orientation

                # ROS uses ENU → convert to NED
                north, east, down = _enu_to_ned(pos.x, pos.y, pos.z)
                roll, pitch, yaw  = _quat_to_euler(q.w, q.x, q.y, q.z)

                cov_flat = list(msg.pose.covariance)
                if len(cov_flat) == 36:
                    cov6 = np.array(cov_flat, dtype=float).reshape(6, 6)
                else:
                    cov6 = np.eye(6) * 0.05

                self._store(north, east, down, roll, pitch, yaw, cov6)

            except Exception as e:
                logger.error(f"VoxlRTABMapClient odom_callback: {e}")

        node.create_subscription(
            Odometry,
            self._odom_topic,
            odom_callback,
            qos_profile_sensor_data,
        )

        if _has_rtabinfo:
            from rtabmap_msgs.msg import Info as RTABInfo

            def info_callback(msg: RTABInfo):
                try:
                    closure_id = int(msg.loop_closure_id) if hasattr(msg, "loop_closure_id") else -1
                    map_nodes  = int(msg.ref_words)        if hasattr(msg, "ref_words")       else 0
                    with self._lock:
                        if closure_id > 0 and closure_id != self._closure_id:
                            self._closure_id = closure_id
                            self._map_nodes  = map_nodes
                            for cb in self._on_loop_closure:
                                try:
                                    cb(closure_id)
                                except Exception as exc:
                                    logger.error(f"loop_closure callback: {exc}")
                        elif map_nodes:
                            self._map_nodes = map_nodes
                except Exception as e:
                    logger.error(f"VoxlRTABMapClient info_callback: {e}")

            node.create_subscription(
                RTABInfo,
                self._info_topic,
                info_callback,
                qos_profile_sensor_data,
            )

        logger.info(f"VOXL ROS2 subscribed: {self._odom_topic}, {self._info_topic}")

        try:
            while self._running:
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception as e:
            logger.error(f"VoxlRTABMapClient ROS2 spin error: {e}")
        finally:
            self._connected = False
            try:
                node.destroy_node()
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    # Internal state store
    # ------------------------------------------------------------------ #

    def _store(self, north: float, east: float, down: float,
               roll: float, pitch: float, yaw: float,
               cov6: np.ndarray,
               closure_id: int = -1,
               map_nodes:  int = 0,
               quality:    int = 0):
        """Update internal pose cache and fire loop-closure callbacks."""
        with self._lock:
            self._pos_ned     = np.array([north, east, down])
            self._rpy         = np.array([roll,  pitch, yaw])
            self._cov6        = cov6
            self._last_update = time.time()
            self._quality     = quality
            if map_nodes:
                self._map_nodes = map_nodes

            if closure_id > 0 and closure_id != self._closure_id:
                self._closure_id = closure_id
                for cb in self._on_loop_closure:
                    try:
                        cb(closure_id)
                    except Exception as e:
                        logger.error(f"loop_closure callback error: {e}")

    # ------------------------------------------------------------------ #
    # Debug
    # ------------------------------------------------------------------ #

    def print_status(self):
        pose = self.get_pose()
        if pose is None:
            print("VoxlRTABMapClient: no pose (not connected or stale)")
            return
        pos, rpy, cov = pose
        print(f"\n--- VOXL RTABMap Client ---")
        print(f"  Mode:    {self.mode}  Connected: {self.is_connected}")
        print(f"  Pos NED: N={pos[0]:.3f}  E={pos[1]:.3f}  D={pos[2]:.3f}")
        print(f"  RPY:     R={math.degrees(rpy[0]):.1f}°  "
              f"P={math.degrees(rpy[1]):.1f}°  Y={math.degrees(rpy[2]):.1f}°")
        print(f"  Closure: {self._closure_id}  Nodes: {self._map_nodes}  "
              f"Quality: {self._quality}")
        print(f"  Age:     {time.time()-self._last_update:.2f}s")
