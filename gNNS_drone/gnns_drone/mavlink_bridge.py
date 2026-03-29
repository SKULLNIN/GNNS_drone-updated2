"""
gNNS Drone — MAVLink Bridge
============================
Core communication layer between Jetson Nano and CubeOrange FC.

Handles:
  - Connection management with auto-reconnect
  - VISION_POSITION_ESTIMATE sending (VIO → FC)
  - Flight commands (arm, disarm, takeoff, land, goto)
  - Message request rate configuration
  - Link quality monitoring & debugging
  - Custom gNNS message sending
"""

import time
import math
import threading
import logging
import yaml
from pathlib import Path
from enum import IntEnum
from typing import Optional, Any
from pymavlink import mavutil

logger = logging.getLogger("gnns.mavlink")

# Sentinel for NaN guard
def _is_finite(*values) -> bool:
    """Check that all values are finite (not NaN/Inf)."""
    return all(math.isfinite(v) for v in values)


class FCMode(IntEnum):
    """ArduPilot Copter flight modes."""
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    GUIDED_NOGPS = 20


class LinkStats:
    """MAVLink link quality statistics."""

    def __init__(self):
        self.messages_sent = 0
        self.messages_received = 0
        self.msg_drops = 0
        self.last_heartbeat_time = 0.0
        self.latency_ms = -1.0
        self.connected = False
        self.fc_armed = False
        self.fc_mode = ""
        self.fc_mode_id = -1

    def __repr__(self):
        return (f"LinkStats(tx={self.messages_sent}, rx={self.messages_received}, "
                f"drops={self.msg_drops}, latency={self.latency_ms:.1f}ms, "
                f"armed={self.fc_armed}, mode={self.fc_mode})")


class MAVLinkBridge:
    """
    MAVLink communication bridge to CubeOrange flight controller.
    
    Usage:
        bridge = MAVLinkBridge()             # Uses default config
        bridge.connect()                     # Connect to FC
        bridge.wait_for_heartbeat()          # Wait for FC heartbeat
        bridge.configure_message_rates()     # Set up data streams
        
        # Send VIO position at 30 Hz
        bridge.send_vision_position(x, y, z, roll, pitch, yaw)
        
        # Navigate
        bridge.set_mode("GUIDED")
        bridge.arm()
        bridge.takeoff(2.0)
        bridge.goto_position_ned(10.0, 5.0, -2.0)
        bridge.land()
    """

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize MAVLink bridge.

        Args:
            config_path: Path to mavlink_config.yaml. If None, uses defaults.
        """
        # Load configuration
        self.config = self._load_config(config_path)

        self.conn: Any = None
        self.stats = LinkStats()
        self._running = False
        self._recv_thread = None
        self._heartbeat_thread = None
        
        # Message callbacks — register handlers for specific messages
        self._callbacks = {}
        
        # Latest received messages (cached)
        self._latest_msgs = {}
        self._msg_lock = threading.Lock()
        
        # ACK tracking (thread-safe, fixes race condition with recv thread)
        self._pending_ack_cmd = -1
        self._pending_ack_result = -1
        self._ack_event = threading.Event()
        
        # Param tracking (thread-safe, fixes race condition with recv thread)
        self._pending_param_name = ""
        self._pending_param_value = None
        self._param_event = threading.Event()
        
        # Logging already set in main() or parent script
        pass

    def _load_config(self, config_path: Optional[str] = None) -> dict:
        """Load MAVLink configuration from YAML file."""
        if config_path is None:
            # Try default location
            default = Path(__file__).parent.parent / "config" / "mavlink_config.yaml"
            if default.exists():
                config_path = str(default)
            else:
                logger.warning("No config file found, using defaults")
                return self._default_config()

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            logger.info(f"Loaded config from {config_path}")
            return config if config is not None else self._default_config()

    def _default_config(self) -> dict:
        return {
            "connection": {
                "port": "tcp:127.0.0.1:5762",
                "baudrate": 921600,
                "source_system": 1,
                "source_component": 191,
                "heartbeat_timeout_s": 5.0,
                "sitl_mode": False,
            },
            "send_rates": {
                "vision_position_estimate": 30,
            },
            "request_rates": {
                "LOCAL_POSITION_NED": 30,
                "ATTITUDE": 30,
                "EKF_STATUS_REPORT": 2,
                "VFR_HUD": 1,
            },
            "debug": {"log_level": "DEBUG"},
        }

    def _conn(self) -> Any:
        """Return the MAVLink connection; raise if not connected (for type checker)."""
        if self.conn is None:
            raise RuntimeError("MAVLink not connected")
        return self.conn

    # ==============================================================
    # CONNECTION MANAGEMENT
    # ==============================================================

    def connect(self) -> bool:
        """Establish MAVLink connection to flight controller."""
        conn_cfg = self.config["connection"]
        port = conn_cfg["port"]
        baud = conn_cfg.get("baudrate", 921600)

        logger.info(f"Connecting to FC: {port} @ {baud} baud...")

        try:
            self.conn = mavutil.mavlink_connection(
                port,
                baud=baud,
                source_system=conn_cfg.get("source_system", 1),
                source_component=conn_cfg.get("source_component", 191),
            )
            logger.info("MAVLink connection object created")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    def wait_for_heartbeat(self, timeout: float = 30.0) -> bool:
        """Wait for heartbeat from FC. Returns True if received."""
        logger.info(f"Waiting for FC heartbeat (timeout={timeout}s)...")
        hb = self._conn().wait_heartbeat(timeout=timeout)
        if hb:
            self.stats.connected = True
            self.stats.last_heartbeat_time = time.time()
            logger.info(
                f"Heartbeat received! System={self._conn().target_system}, "
                f"Component={self._conn().target_component}, "
                f"Type={hb.type}, Autopilot={hb.autopilot}"
            )
            # Start receive thread + heartbeat thread
            self._start_recv_thread()
            self._start_heartbeat_thread()
            return True
        else:
            logger.error("No heartbeat received — check wiring & baud rate!")
            return False

    def disconnect(self):
        """Close the MAVLink connection."""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=2.0)
        if self.conn:
            self._conn().close()
        logger.info("Disconnected from FC")

    def _start_heartbeat_thread(self):
        """Start periodic companion computer heartbeat (1 Hz)."""
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="mavlink-hb"
        )
        self._heartbeat_thread.start()
        logger.debug("Heartbeat thread started (1 Hz)")

    def _heartbeat_loop(self):
        """Send companion heartbeat at 1 Hz — required by ArduPilot."""
        while self._running:
            try:
                self.send_heartbeat()
            except Exception as e:
                logger.error(f"Heartbeat send error: {e}")
            time.sleep(1.0)

    # ==============================================================
    # MESSAGE RECEIVE LOOP
    # ==============================================================

    def _start_recv_thread(self):
        """Start background thread for receiving MAVLink messages."""
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True, name="mavlink-recv"
        )
        self._recv_thread.start()
        logger.debug("Receive thread started")

    def _recv_loop(self):
        """Background loop that receives and dispatches MAVLink messages."""
        debug_cfg = self.config.get("debug", {})
        log_all = debug_cfg.get("log_all_incoming", False)
        log_unknown = debug_cfg.get("log_unknown_messages", True)
        log_acks = debug_cfg.get("log_command_acks", True)
        log_mode = debug_cfg.get("log_mode_changes", True)

        last_mode = -1

        while self._running:
            try:
                msg = self._conn().recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue

                msg_type = msg.get_type()
                self.stats.messages_received += 1

                if log_all:
                    logger.debug(f"RX: {msg_type}: {msg.to_dict()}")

                # Cache latest message
                with self._msg_lock:
                    self._latest_msgs[msg_type] = msg

                # Handle heartbeat
                if msg_type == "HEARTBEAT":
                    self.stats.last_heartbeat_time = time.time()
                    self.stats.fc_armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    new_mode = msg.custom_mode
                    if new_mode != last_mode:
                        mode_name = self._mode_id_to_name(new_mode)
                        self.stats.fc_mode = mode_name
                        self.stats.fc_mode_id = new_mode
                        if log_mode and last_mode >= 0:
                            logger.info(f"Mode changed: {self._mode_id_to_name(last_mode)} -> {mode_name}")
                        last_mode = new_mode

                # Handle command ACKs — notify waiting _wait_ack()
                elif msg_type == "COMMAND_ACK":
                    if log_acks:
                        result_names = {
                            0: "ACCEPTED", 1: "TEMPORARILY_REJECTED",
                            2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED",
                            5: "IN_PROGRESS",
                        }
                        result = result_names.get(msg.result, f"UNKNOWN({msg.result})")
                        logger.info(f"CMD_ACK: command={msg.command}, result={result}")
                    # Wake up _wait_ack if waiting for this command
                    if msg.command == self._pending_ack_cmd:
                        self._pending_ack_result = msg.result
                        self._ack_event.set()

                # Handle PARAM_VALUE — notify waiting get_param/set_param
                elif msg_type == "PARAM_VALUE":
                    param_id = msg.param_id.strip('\x00')
                    if param_id == self._pending_param_name:
                        self._pending_param_value = msg.param_value
                        self._param_event.set()

                # Handle STATUSTEXT from FC
                elif msg_type == "STATUSTEXT":
                    severity_names = {
                        0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
                        4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"
                    }
                    severity = getattr(msg, 'severity', 6)  # Default to INFO
                    text = msg.text
                    if severity <= 3: # Error or higher
                        logger.error(f"FC [ERROR]: {text}")
                    elif severity <= 4: # Warning
                        logger.warning(f"FC [WARN]: {text}")
                    else:
                        logger.info(f"FC [INFO]: {text}")

                # Dispatch to registered callbacks
                if msg_type in self._callbacks:
                    for cb in self._callbacks[msg_type]:
                        try:
                            cb(msg)
                        except Exception as e:
                            logger.error(f"Callback error for {msg_type}: {e}")

            except Exception as e:
                logger.error(f"Recv error: {e}")
                time.sleep(0.1)

    def register_callback(self, msg_type: str, callback):
        """Register a callback for a specific message type."""
        if msg_type not in self._callbacks:
            self._callbacks[msg_type] = []
        self._callbacks[msg_type].append(callback)

    def get_latest(self, msg_type: str):
        """Get the most recently received message of a given type."""
        with self._msg_lock:
            return self._latest_msgs.get(msg_type)

    # ==============================================================
    # MESSAGE RATE CONFIGURATION
    # ==============================================================

    def configure_message_rates(self):
        """
        Tell the FC how often to send us each message type.
        Based on request_rates in config.
        """
        rates = self.config.get("request_rates", {})
        logger.info("Configuring FC message rates...")

        msg_id_map = {
            "HEARTBEAT": mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
            "SYS_STATUS": mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            "LOCAL_POSITION_NED": mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            "ATTITUDE": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            "EKF_STATUS_REPORT": 193,  # ArduPilot-specific
            "VIBRATION": mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION,
            "BATTERY_STATUS": mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
            "VFR_HUD": mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
            "HOME_POSITION": mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
        }

        for msg_name, rate_hz in rates.items():
            if msg_name in msg_id_map:
                msg_id = msg_id_map[msg_name]
                interval_us = int(1e6 / rate_hz) if rate_hz > 0 else 0

                self._conn().mav.command_long_send(
                    self._conn().target_system,
                    self._conn().target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    interval_us,
                    0, 0, 0, 0, 0
                )
                status = "enabled" if rate_hz > 0 else "disabled"
                logger.debug(f"  {msg_name}: {rate_hz} Hz ({status})")
                time.sleep(0.05)  # Small delay between commands

        logger.info("Message rates configured")

    # ==============================================================
    # VIO POSITION SENDING
    # ==============================================================

    def send_vision_position(self, x: float, y: float, z: float,
                              roll: float = 0, pitch: float = 0, yaw: float = 0,
                              covariance: Optional[list] = None):
        """
        Send VISION_POSITION_ESTIMATE to FC.
        This is the core message that replaces GPS!
        
        Args:
            x: North position in meters (NED frame)
            y: East position in meters
            z: Down position in meters (negative = up!)
            roll, pitch, yaw: Orientation in radians
            covariance: Optional 21-element upper-triangle covariance matrix
        """
        # NaN guard — sending NaN to EKF will crash it
        if not _is_finite(x, y, z, roll, pitch, yaw):
            logger.warning("NaN/Inf in vision position! Skipping send.")
            return

        usec = int(time.time() * 1e6)

        if covariance and len(covariance) == 21:
            self._conn().mav.vision_position_estimate_send(
                usec, x, y, z, roll, pitch, yaw, covariance
            )
        else:
            self._conn().mav.vision_position_estimate_send(
                usec, x, y, z, roll, pitch, yaw
            )
        self.stats.messages_sent += 1

    def send_vision_speed(self, vx: float, vy: float, vz: float):
        """Send VISION_SPEED_ESTIMATE to FC (helps EKF converge faster)."""
        if not _is_finite(vx, vy, vz):
            logger.warning("NaN/Inf in vision speed! Skipping send.")
            return
        usec = int(time.time() * 1e6)
        self._conn().mav.vision_speed_estimate_send(usec, vx, vy, vz)
        self.stats.messages_sent += 1

    # ==============================================================
    # FLIGHT COMMANDS
    # ==============================================================

    def set_mode(self, mode_name: str, timeout: float = 3.0) -> bool:
        """Set flight mode by name. Returns True if accepted."""
        if isinstance(mode_name, str):
            try:
                mode_id = FCMode[mode_name.upper()].value
            except KeyError:
                mode_mapping = self._conn().mode_mapping()
                mode_id = mode_mapping.get(mode_name)
                if mode_id is None:
                    logger.error(f"Unknown mode: {mode_name}")
                    return False
        else:
            mode_id = int(mode_name)

        logger.info(f"Setting mode: {mode_name} (id={mode_id})")
        self._conn().mav.command_long_send(
            self._conn().target_system, self._conn().target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0
        )
        self.stats.messages_sent += 1
        return self._wait_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout)

    def arm(self, timeout: float = 5.0, retries: int = 1) -> bool:
        """
        Arm motors. Returns True if accepted.
        
        Args:
            timeout: Timeout per attempt
            retries: Number of retry attempts (use >1 for SITL where EKF
                     may not be ready yet)
        """
        for attempt in range(retries):
            logger.info(f"Arming (attempt {attempt+1}/{retries})...")
            self._conn().mav.command_long_send(
                self._conn().target_system, self._conn().target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0
            )
            self.stats.messages_sent += 1
            
            # Method 1: Check command ACK
            result = self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
            if result:
                logger.info("Armed successfully! (ACK received)")
                return True
            
            # Method 2: Check heartbeat armed flag (some SITL versions don't send ACK)
            armed = False
            check_start = time.time()
            while time.time() - check_start < 3.0:
                hb = self.get_latest("HEARTBEAT") 
                if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    armed = True
                    break
                time.sleep(0.2)
            
            if armed:
                logger.info("Armed successfully! (heartbeat flag confirmed)")
                return True
            
            if attempt < retries - 1:
                logger.warning(f"Arm failed (attempt {attempt+1}), waiting 5s for EKF...")
                time.sleep(5)
        
        logger.error("Arm FAILED — check pre-arm messages above")
        return False

    def disarm(self, force: bool = False, timeout: float = 3.0) -> bool:
        """Disarm motors."""
        logger.info(f"Disarming (force={force})...")
        self._conn().mav.command_long_send(
            self._conn().target_system, self._conn().target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196 if force else 0, 0, 0, 0, 0, 0
        )
        self.stats.messages_sent += 1
        return self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)

    def takeoff(self, altitude: float = 2.0, timeout: float = 15.0) -> bool:
        """
        Take off to altitude (meters above ground).
        Must be in GUIDED mode and armed first.
        """
        logger.info(f"Takeoff to {altitude}m...")
        self._conn().mav.command_long_send(
            self._conn().target_system, self._conn().target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude
        )
        self.stats.messages_sent += 1

        # Wait until altitude reached
        # In NED: z is NEGATIVE when above ground
        start = time.time()
        while time.time() - start < timeout:
            pos = self.get_latest("LOCAL_POSITION_NED")
            if pos and pos.z <= -(altitude * 0.85):
                logger.info(f"Reached altitude: {-pos.z:.1f}m")
                return True
            time.sleep(0.2)

        # Log current altitude even on timeout
        pos = self.get_latest("LOCAL_POSITION_NED")
        alt_str = f"{-pos.z:.1f}m" if pos else "unknown"
        logger.warning(f"Takeoff timeout! Current altitude: {alt_str}")
        return False

    def land(self):
        """Switch to LAND mode."""
        logger.info("Landing...")
        return self.set_mode("LAND")

    def goto_position_ned(self, north: float, east: float, down: float,
                           yaw: float = float('nan')):
        """
        Command drone to fly to a local NED position.
        
        Args:
            north: X position (meters, North)
            east: Y position (meters, East)
            down: Z position (meters, Down — NEGATIVE = UP)
            yaw: Target yaw in radians. NaN = don't change yaw.
        """
        if not _is_finite(north, east, down):
            logger.warning("NaN/Inf in goto_position_ned! Skipping.")
            return

        type_mask = 0b0000_1111_1111_1000

        if not math.isnan(yaw):
            type_mask = 0b0000_1011_1111_1000  # Also use yaw

        self._conn().mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self._conn().target_system, self._conn().target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north, east, down,
            0, 0, 0,   # velocity
            0, 0, 0,   # acceleration
            yaw, 0      # yaw, yaw_rate
        )
        self.stats.messages_sent += 1

    def goto_position_ned_wait(self, north: float, east: float, down: float,
                                tolerance: float = 1.0, timeout: float = 120.0,
                                resend_interval: float = 1.0) -> bool:
        """
        Go to NED position and wait until arrived.
        Resends command periodically in case it's dropped.
        
        Returns True if arrived, False if timeout.
        """
        logger.info(f"GoTo NED({north:.1f}, {east:.1f}, {down:.1f}) "
                     f"tolerance={tolerance}m timeout={timeout}s")
        start = time.time()
        last_send = 0

        while time.time() - start < timeout:
            # Resend command periodically
            if time.time() - last_send > resend_interval:
                self.goto_position_ned(north, east, down)
                last_send = time.time()

            # Check position
            pos = self.get_latest("LOCAL_POSITION_NED")
            if pos:
                dist = math.sqrt((pos.x - north)**2 + (pos.y - east)**2)
                if dist < tolerance:
                    logger.info(f"Arrived! Distance: {dist:.2f}m")
                    return True

                if int(time.time()) % 5 == 0:  # Log every 5 seconds
                    logger.debug(f"  En route — dist={dist:.1f}m, "
                                  f"pos=({pos.x:.1f}, {pos.y:.1f})")

            time.sleep(0.2)

        logger.warning(f"GoTo timeout after {timeout}s!")
        return False

    def wait_landed(self, timeout: float = 30.0) -> bool:
        """Wait until drone has landed and disarmed."""
        start = time.time()
        while time.time() - start < timeout:
            if not self.stats.fc_armed:
                logger.info("Landed and disarmed!")
                return True
            time.sleep(0.5)
        logger.warning("Landing timeout!")
        return False

    # ==============================================================
    # OBSTACLE DISTANCE (LiDAR → FC)
    # ==============================================================

    def send_obstacle_distances(self, distances_cm: list,
                                 increment_deg: int = 5,
                                 angle_offset: float = 0):
        """
        Send OBSTACLE_DISTANCE to FC from LiDAR data.
        
        Args:
            distances_cm: List of 72 distance values in centimeters
                          (one per angular sector, 5° apart for 360°)
            increment_deg: Angular increment between sectors
            angle_offset: Starting angle offset in degrees
        """
        self._conn().mav.obstacle_distance_send(
            int(time.time() * 1e6),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            distances_cm,
            increment_deg,
            15,     # min distance cm
            1200,   # max distance cm (12m)
            0,      # increment_f
            angle_offset,
            mavutil.mavlink.MAV_FRAME_BODY_FRD
        )
        self.stats.messages_sent += 1

    # ==============================================================
    # FC PARAMETER ACCESS
    # ==============================================================

    def get_param(self, param_name: str, timeout: float = 3.0):
        """Read a parameter from the FC (thread-safe, uses recv thread)."""
        self._param_event.clear()
        self._pending_param_name = param_name
        self._pending_param_value = None

        self._conn().mav.param_request_read_send(
            self._conn().target_system, self._conn().target_component,
            param_name.encode('utf-8'), -1
        )
        # Wait for recv thread to deliver the PARAM_VALUE
        if self._param_event.wait(timeout=timeout):
            value = self._pending_param_value
            logger.debug(f"Param {param_name} = {value}")
            self._pending_param_name = ""
            return value

        logger.warning(f"Failed to read param: {param_name}")
        self._pending_param_name = ""
        return None

    def set_param(self, param_name: str, value: float, timeout: float = 3.0) -> bool:
        """Set a parameter on the FC (thread-safe, uses recv thread)."""
        logger.info(f"Setting param {param_name} = {value}")
        self._param_event.clear()
        self._pending_param_name = param_name
        self._pending_param_value = None

        self._conn().mav.param_set_send(
            self._conn().target_system, self._conn().target_component,
            param_name.encode('utf-8'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        # Wait for recv thread to deliver confirmation
        if self._param_event.wait(timeout=timeout):
            received = self._pending_param_value
            self._pending_param_name = ""
            if received is not None and abs(received - value) < 0.001:
                logger.info(f"Param {param_name} set to {received}")
                return True
        logger.error(f"Failed to set param {param_name}")
        self._pending_param_name = ""
        return False

    # ==============================================================
    # HELPERS
    # ==============================================================

    def _wait_ack(self, command_id: int, timeout: float = 3.0) -> bool:
        """
        Wait for COMMAND_ACK for a specific command.
        Uses event-based approach: the recv thread sets the event when it
        receives the matching ACK. No recv_match race condition.
        """
        self._ack_event.clear()
        self._pending_ack_cmd = command_id
        self._pending_ack_result = -1

        if self._ack_event.wait(timeout=timeout):
            self._pending_ack_cmd = -1
            return self._pending_ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        
        self._pending_ack_cmd = -1
        logger.warning(f"ACK timeout for command {command_id}")
        return False

    def _mode_id_to_name(self, mode_id: int) -> str:
        """Convert ArduPilot mode ID to name."""
        for mode in FCMode:
            if mode.value == mode_id:
                return mode.name
        return f"UNKNOWN({mode_id})"

    @property
    def is_connected(self) -> bool:
        timeout = self.config["connection"].get("heartbeat_timeout_s", 5.0)
        return (time.time() - self.stats.last_heartbeat_time) < timeout

    @property
    def is_armed(self) -> bool:
        return self.stats.fc_armed

    def send_heartbeat(self):
        """Send companion computer heartbeat to FC."""
        self._conn().mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        self.stats.messages_sent += 1

    def send_velocity_ned(self, vx: float, vy: float, vz: float,
                           yaw_rate: float = 0):
        """
        Send velocity command in NED frame (yaw rate mode).
        
        Args:
            vx: North velocity (m/s)
            vy: East velocity (m/s)
            vz: Down velocity (m/s, positive = descend)
            yaw_rate: Yaw rotation rate (rad/s)
        """
        if not _is_finite(vx, vy, vz, yaw_rate):
            logger.warning("NaN/Inf in velocity command! Sending zero.")
            vx = vy = vz = yaw_rate = 0

        # type_mask: bits set = fields IGNORED
        # 0b0000_0111_1100_0111 = use velocity + yaw_rate only
        type_mask = 0b0000_0111_1100_0111

        self._conn().mav.set_position_target_local_ned_send(
            0,
            self._conn().target_system,
            self._conn().target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,           # position (ignored)
            vx, vy, vz,        # velocity
            0, 0, 0,           # acceleration (ignored)
            0, yaw_rate         # yaw, yaw_rate
        )
        self.stats.messages_sent += 1

    def send_velocity_ned_yaw(self, vx: float, vy: float, vz: float,
                               yaw: float = 0):
        """
        Send velocity command with LOCKED YAW HEADING.
        
        This tells ArduPilot: "fly at this velocity AND point nose at this angle."
        Much more stable than yaw_rate mode because ArduPilot's inner loop
        actively holds the heading.
        
        Args:
            vx: North velocity (m/s)
            vy: East velocity (m/s)
            vz: Down velocity (m/s, positive = descend)
            yaw: Desired yaw heading (radians, 0=North, pi/2=East)
        """
        if not _is_finite(vx, vy, vz, yaw):
            logger.warning("NaN/Inf in velocity+yaw command! Sending zero.")
            vx = vy = vz = yaw = 0

        # type_mask: use velocity + yaw (NOT yaw_rate)
        # Bit 10 = ignore yaw_rate, Bit 11 = DON'T ignore yaw
        # 0b0000_0101_1100_0111 = velocity + yaw heading
        type_mask = 0b0000_0101_1100_0111

        self._conn().mav.set_position_target_local_ned_send(
            0,
            self._conn().target_system,
            self._conn().target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,           # position (ignored)
            vx, vy, vz,        # velocity
            0, 0, 0,           # acceleration (ignored)
            yaw, 0              # yaw (heading), yaw_rate (ignored)
        )
        self.stats.messages_sent += 1

    def request_all_streams(self, rate_hz: int = 10):
        """
        Request all data streams at given rate.
        Needed for SITL (real FC uses configure_message_rates instead).
        """
        logger.info(f"Requesting all streams at {rate_hz} Hz")
        self._conn().mav.request_data_stream_send(
            self._conn().target_system, self._conn().target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1
        )
        self.stats.messages_sent += 1

    def wait_ekf_ready(self, timeout: float = 60.0) -> bool:
        """
        Wait for EKF to initialize.
        In SITL: takes 15-30s. On real hardware: usually instant after GPS fix.
        """
        logger.info(f"Waiting for EKF to initialize (timeout={timeout}s)...")
        start = time.time()

        while time.time() - start < timeout:
            pos = self.get_latest("LOCAL_POSITION_NED")
            hb = self.get_latest("HEARTBEAT")

            # EKF is ready when we have position data and system is STANDBY+
            if pos is not None and hb is not None:
                if hasattr(hb, 'system_status') and hb.system_status >= 3:
                    elapsed = time.time() - start
                    if elapsed > 5:  # Give at least 5s
                        logger.info(f"EKF ready ({elapsed:.0f}s)")
                        return True
            time.sleep(1.0)

        logger.warning(f"EKF timeout after {timeout}s — continuing anyway")
        return True

    @property
    def is_sitl(self) -> bool:
        """Check if running in SITL mode."""
        return self.config.get("connection", {}).get("sitl_mode", False)
