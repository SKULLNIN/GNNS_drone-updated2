"""
gNNS Drone — Flight Controller (PID + Smooth Velocity Profiles)
=================================================================
Per-axis PID controller with acceleration-limited velocity output.

Inspired by KumarRobotics autonomy_core SO3 controller.
Their gains: pos={4.0, 4.0, 4.0}, vel={3.5, 3.5, 3.5}
We adapt these for velocity-command mode (ArduPilot does final control).

Key features:
  - Per-axis PID with anti-windup
  - Acceleration limiting (smooth velocity changes)
  - Velocity ramp profiles (no sudden jumps)
  - Low-pass filtered derivative (clean D-term)

Data flow:
  position_error → PID → raw_velocity → accel_limiter → smooth_velocity → FC
"""

import time
import math
import logging
import yaml
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger("gnns.pid")


@dataclass
class PIDGains:
    """PID gains for one control axis or group."""
    kp: float = 0.5
    ki: float = 0.0
    kd: float = 0.1
    max_integral: float = 1.0


@dataclass
class FlightConfig:
    """
    All flight parameters loaded from flight_config.yaml.
    Single source of truth for all tuning.
    """
    # Position gains
    horizontal_gains: PIDGains = field(default_factory=lambda: PIDGains(0.6, 0.02, 0.15, 1.0))
    vertical_gains: PIDGains = field(default_factory=lambda: PIDGains(0.8, 0.03, 0.2, 0.5))

    # Velocity limits
    cruise_max_speed: float = 1.5
    approach_max_speed: float = 1.0
    landing_horiz_speed: float = 0.5
    landing_final_speed: float = 0.2
    ascent_max_speed: float = 1.0
    descent_max_speed: float = 1.0
    descent_landing_speed: float = 0.3
    descent_final_speed: float = 0.15

    # Acceleration limits
    horiz_accel_limit: float = 2.0
    vert_accel_limit: float = 1.5
    landing_accel_limit: float = 0.5

    # Waypoint
    cruise_altitude: float = 2.5
    arrival_radius: float = 2.5
    slowdown_radius: float = 5.0

    # Takeoff
    takeoff_altitude: float = 2.5
    takeoff_climb_rate: float = 0.8
    takeoff_stabilize_time: float = 2.0
    takeoff_ramp_time: float = 1.5

    # Landing
    approach_altitude: float = 3.0
    align_descent_rate: float = 0.3
    final_descent_rate: float = 0.15
    align_center_tol: float = 0.5
    final_center_tol: float = 0.3
    touchdown_altitude: float = 0.15
    search_timeout: float = 30.0
    max_landing_time: float = 60.0
    max_tilt_deg: float = 15.0
    target_lost_align_timeout: float = 10.0
    target_lost_final_timeout: float = 5.0

    # Landing PID
    align_gains: PIDGains = field(default_factory=lambda: PIDGains(0.8, 0.0, 0.2, 0.3))
    final_gains: PIDGains = field(default_factory=lambda: PIDGains(0.5, 0.0, 0.3, 0.2))

    # Safety
    geofence_radius: float = 100.0
    min_battery_pct: float = 20.0
    max_altitude: float = 10.0
    min_odom_confidence: int = 30

    @staticmethod
    def from_yaml(path: Optional[str] = None) -> 'FlightConfig':
        """Load config from YAML file."""
        if path is None:
            path = str(Path(__file__).parent.parent / "config" / "flight_config.yaml")

        cfg = FlightConfig()

        if not Path(path).exists():
            logger.warning(f"Flight config not found: {path}, using defaults")
            return cfg

        with open(path) as f:
            data = yaml.safe_load(f) or {}

        # Position gains
        pg = data.get("position_gains", {})
        h = pg.get("horizontal", {})
        v = pg.get("vertical", {})
        cfg.horizontal_gains = PIDGains(h.get("kp", 0.6), h.get("ki", 0.02),
                                         h.get("kd", 0.15), h.get("max_integral", 1.0))
        cfg.vertical_gains = PIDGains(v.get("kp", 0.8), v.get("ki", 0.03),
                                       v.get("kd", 0.2), v.get("max_integral", 0.5))

        # Velocity limits
        vl = data.get("velocity_limits", {})
        cfg.cruise_max_speed = vl.get("cruise_max", 1.5)
        cfg.approach_max_speed = vl.get("approach_max", 1.0)
        cfg.landing_horiz_speed = vl.get("landing_horizontal", 0.5)
        cfg.landing_final_speed = vl.get("landing_final", 0.2)
        cfg.ascent_max_speed = vl.get("ascent_max", 1.0)
        cfg.descent_max_speed = vl.get("descent_max", 1.0)
        cfg.descent_landing_speed = vl.get("descent_landing", 0.3)
        cfg.descent_final_speed = vl.get("descent_final", 0.15)

        # Acceleration limits
        al = data.get("acceleration_limits", {})
        cfg.horiz_accel_limit = al.get("horizontal", 2.0)
        cfg.vert_accel_limit = al.get("vertical", 1.5)
        cfg.landing_accel_limit = al.get("landing", 0.5)

        # Waypoint
        wp = data.get("waypoint", {})
        cfg.cruise_altitude = wp.get("cruise_altitude", 2.5)
        cfg.arrival_radius = wp.get("arrival_radius", 2.5)
        cfg.slowdown_radius = wp.get("slowdown_radius", 5.0)

        # Takeoff
        to = data.get("takeoff", {})
        cfg.takeoff_altitude = to.get("target_altitude", 2.5)
        cfg.takeoff_climb_rate = to.get("vertical_speed", 0.8)
        cfg.takeoff_stabilize_time = to.get("stabilize_time", 2.0)
        cfg.takeoff_ramp_time = to.get("ramp_up_time", 1.5)

        # Landing
        ld = data.get("landing", {})
        cfg.approach_altitude = ld.get("approach_altitude", 3.0)
        cfg.align_descent_rate = ld.get("align_descent_rate", 0.3)
        cfg.final_descent_rate = ld.get("final_descent_rate", 0.15)
        cfg.align_center_tol = ld.get("align_center_tolerance", 0.5)
        cfg.final_center_tol = ld.get("final_center_tolerance", 0.3)
        cfg.touchdown_altitude = ld.get("touchdown_altitude", 0.15)
        cfg.search_timeout = ld.get("search_timeout", 30.0)
        cfg.max_landing_time = ld.get("max_landing_time", 60.0)
        cfg.max_tilt_deg = ld.get("max_tilt_deg", 15.0)
        cfg.target_lost_align_timeout = ld.get("target_lost_align_timeout", 10.0)
        cfg.target_lost_final_timeout = ld.get("target_lost_final_timeout", 5.0)

        ag = ld.get("align_gains", {})
        cfg.align_gains = PIDGains(ag.get("kp", 0.8), ag.get("ki", 0.0),
                                    ag.get("kd", 0.2))
        fg = ld.get("final_gains", {})
        cfg.final_gains = PIDGains(fg.get("kp", 0.5), fg.get("ki", 0.0),
                                    fg.get("kd", 0.3))

        logger.info(f"Flight config loaded from {path}")
        return cfg


class PIDController:
    """
    PID controller for one axis.
    
    Features:
      - Anti-windup (integral clamp)
      - Filtered derivative (low-pass on D-term, not raw delta)
      - Output clamping
      
    autonomy_core uses SO3 geometry-based control (thrust+attitude).
    We use PID → velocity commands, so gains scale is different.
    Their Kp=4.0 maps force directly; ours maps meters → m/s.
    """

    def __init__(self, gains: PIDGains):
        self.kp = gains.kp
        self.ki = gains.ki
        self.kd = gains.kd
        self.max_integral = gains.max_integral

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0
        self._initialized = False
        self._d_filter_alpha = 0.3   # Heavy D-term filtering for smooth derivative
        self._filtered_d = 0.0

    def reset(self):
        """Reset controller state (call on mode change)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0
        self._initialized = False

    def update(self, error: float, dt: Optional[float] = None) -> float:
        """
        Compute PID output.
        
        Args:
            error: Current error (target - actual)
            dt: Time step. If None, computed from wall clock.
            
        Returns:
            Control output (velocity command)
        """
        now = time.time()

        if not self._initialized:
            self._prev_error = error
            self._prev_time = now
            self._initialized = True
            return self.kp * error  # First call: pure P

        if dt is None:
            dt = now - self._prev_time
        if dt <= 0 or dt > 1.0:
            dt = 0.02  # Fallback: assume 50Hz

        # P term
        p = self.kp * error

        # I term with anti-windup
        self._integral += error * dt
        self._integral = max(-self.max_integral, min(self.max_integral, self._integral))
        i = self.ki * self._integral

        # D term with low-pass filter (prevents VIO noise spikes)
        raw_d = (error - self._prev_error) / dt
        if not hasattr(self, '_filtered_d'):
            self._filtered_d = 0.0
        self._filtered_d = (self._d_filter_alpha * raw_d +
                           (1 - self._d_filter_alpha) * self._filtered_d)
        d = self.kd * self._filtered_d

        self._prev_error = error
        self._prev_time = now

        return p + i + d


class VelocitySmoother:
    """
    Acceleration-limited velocity smoother.
    
    Prevents jerky velocity changes by limiting how fast
    velocity can change per time step.
    
    Inspired by autonomy_core's stopping_policy:
      acc_xy_des: 6.0 (theirs is aggressive, ours is gentler)
      jerk_xy_des: 3.0
    
    Input:  raw velocity command (from PID)
    Output: smoothed velocity (acceleration-limited)
    """

    def __init__(self, max_accel: float = 2.0):
        self.max_accel = max_accel
        self._current_vel = 0.0
        self._prev_time = 0.0

    def update(self, target_vel: float, dt: Optional[float] = None) -> float:
        """
        Smooth velocity toward target, limited by max acceleration.
        
        Args:
            target_vel: Desired velocity
            dt: Time step (seconds)
            
        Returns:
            Smoothed velocity
        """
        now = time.time()
        if dt is None:
            dt = now - self._prev_time if self._prev_time > 0 else 0.1
        # Floor dt at 10ms — prevents near-zero dt when called
        # immediately after PID in the same compute cycle
        if dt < 0.01:
            dt = 0.1  # Assume 10Hz control loop
        if dt > 1.0:
            dt = 0.1
        self._prev_time = now

        # Maximum velocity change this step
        max_delta = self.max_accel * dt

        # Limit the change
        delta = target_vel - self._current_vel
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)

        self._current_vel += delta
        return self._current_vel

    def reset(self):
        self._current_vel = 0.0
        self._prev_time = 0.0

    @property
    def current(self) -> float:
        return self._current_vel


class PositionFilter:
    """
    Low-pass filter for VIO position data.
    
    VIO (RTAB-Map / T265) position updates are noisy — small jumps
    of ±2-10cm per frame. Without filtering, the PID D-term sees
    these as velocity spikes and creates oscillation/wobble.
    
    This exponential moving average smooths position before PID:
      filtered = alpha * new_measurement + (1 - alpha) * prev_filtered
    
    Lower alpha = smoother but laggier (0.2 = very smooth)
    Higher alpha = faster but noisier (0.8 = barely filtered)
    
    For a drone on Jetson Nano with RTAB-Map at 15-30Hz:
      alpha=0.4 is a good balance
    """

    def __init__(self, alpha: float = 0.3):
        # Alpha 0.3 = smooth position (filters VIO noise before PID)
        # This is applied to RAW POSITION, not error!
        self.alpha = alpha
        self._filtered = {}

    def update(self, name: str, raw_value: float) -> float:
        if name not in self._filtered:
            self._filtered[name] = raw_value
            return raw_value
        self._filtered[name] = (self.alpha * raw_value +
                                (1 - self.alpha) * self._filtered[name])
        return self._filtered[name]

    def reset(self):
        self._filtered = {}


class FlightController:
    """
    High-level flight controller using PID + velocity smoothing.
    
    Replaces the single-gain P-controller in navigator.py
    with proper per-axis PID and acceleration limiting.
    
    Usage:
        fc = FlightController()  # Loads from flight_config.yaml
        
        # In control loop (10-20 Hz):
        vx, vy, vz = fc.compute_waypoint_velocity(
            target_n=50.0, target_e=30.0, target_alt=2.5,
            current_n=48.0, current_e=29.0, current_alt=2.3,
            current_vx=0.5, current_vy=0.3, current_vz=0.0
        )
        bridge.send_velocity_ned(vx, vy, vz)
    """

    def __init__(self, config: Optional[FlightConfig] = None):
        self.config = config or FlightConfig.from_yaml()

        # PID controllers — one per axis
        self._pid_north = PIDController(self.config.horizontal_gains)
        self._pid_east = PIDController(self.config.horizontal_gains)
        self._pid_alt = PIDController(self.config.vertical_gains)

        # Velocity smoothers — one per axis
        self._smooth_north = VelocitySmoother(self.config.horiz_accel_limit)
        self._smooth_east = VelocitySmoother(self.config.horiz_accel_limit)
        self._smooth_alt = VelocitySmoother(self.config.vert_accel_limit)

        # Landing PID (separate gains for landing precision)
        self._pid_land_fwd = PIDController(self.config.align_gains)
        self._pid_land_right = PIDController(self.config.align_gains)
        self._smooth_land_fwd = VelocitySmoother(self.config.landing_accel_limit)
        self._smooth_land_right = VelocitySmoother(self.config.landing_accel_limit)

        logger.info("FlightController initialized with PID gains: "
                     f"H(kp={self.config.horizontal_gains.kp}, "
                     f"ki={self.config.horizontal_gains.ki}, "
                     f"kd={self.config.horizontal_gains.kd}) "
                     f"V(kp={self.config.vertical_gains.kp})")

        # Position filter — smooths VIO noise before PID
        # IMPORTANT: Filter RAW POSITION, not error signal!
        self._pos_filter = PositionFilter(alpha=0.3)

    def reset(self):
        """Reset all controllers (call before new flight phase)."""
        self._pid_north.reset()
        self._pid_east.reset()
        self._pid_alt.reset()
        self._smooth_north.reset()
        self._smooth_east.reset()
        self._smooth_alt.reset()
        self._pid_land_fwd.reset()
        self._pid_land_right.reset()
        self._smooth_land_fwd.reset()
        self._smooth_land_right.reset()
        self._pos_filter.reset()

    def compute_waypoint_velocity(self, target_n: float, target_e: float,
                                   target_alt: float,
                                   current_n: float, current_e: float,
                                   current_alt: float,
                                   dt: Optional[float] = None) -> tuple:
        """
        Compute velocity command to fly toward a waypoint.
        
        Uses PID + acceleration limiting for smooth flight.
        Automatically slows down near the waypoint.
        Returns yaw heading to point nose toward target.
        
        Args:
            target_n/e: Target NED position (meters)
            target_alt: Target altitude (positive up, meters)
            current_n/e: Current NED position
            current_alt: Current altitude (positive up)
            dt: Time step (seconds). If None, auto-computed from wall clock.
            
        Returns:
            (vx, vy, vz, yaw) in NED frame (m/s) + yaw in radians
        """
        # Filter RAW POSITIONS first (not errors!)
        # This prevents VIO noise from entering the PID at all
        filt_n = self._pos_filter.update('cur_n', current_n)
        filt_e = self._pos_filter.update('cur_e', current_e)
        filt_alt = self._pos_filter.update('cur_alt', current_alt)

        # Position errors (from smoothed position)
        err_n = target_n - filt_n
        err_e = target_e - filt_e
        err_alt = target_alt - filt_alt

        # Guard NaN
        if any(math.isnan(v) for v in (err_n, err_e, err_alt)):
            return (0.0, 0.0, 0.0, getattr(self, '_last_yaw', 0.0))

        distance = math.sqrt(err_n ** 2 + err_e ** 2)

        # Compute desired yaw: point nose toward target
        if distance > 1.0:
            # Far enough: point toward waypoint
            desired_yaw = math.atan2(err_e, err_n)
            self._last_yaw = desired_yaw
        else:
            # Close to target: hold last heading (no spinning)
            desired_yaw = getattr(self, '_last_yaw', 0.0)

        # Dynamic speed limit: slow down near waypoint
        if distance > self.config.slowdown_radius:
            speed_limit = self.config.cruise_max_speed
        elif distance > self.config.arrival_radius:
            frac = (distance - self.config.arrival_radius) / \
                   (self.config.slowdown_radius - self.config.arrival_radius)
            speed_limit = 0.3 + frac * (self.config.cruise_max_speed - 0.3)
        else:
            speed_limit = 0.3

        # PID per axis
        raw_vn = self._pid_north.update(err_n, dt)
        raw_ve = self._pid_east.update(err_e, dt)

        # Clamp horizontal speed
        raw_speed = math.sqrt(raw_vn ** 2 + raw_ve ** 2)
        if raw_speed > speed_limit and raw_speed > 0:
            scale = speed_limit / raw_speed
            raw_vn *= scale
            raw_ve *= scale

        # Smooth with acceleration limiter (pass same dt)
        vn = self._smooth_north.update(raw_vn, dt)
        ve = self._smooth_east.update(raw_ve, dt)

        # Vertical: PID on altitude error
        raw_vz_up = self._pid_alt.update(err_alt, dt)
        raw_vz_up = max(-self.config.descent_max_speed,
                        min(self.config.ascent_max_speed, raw_vz_up))
        vz_up = self._smooth_alt.update(raw_vz_up, dt)

        # Convert to NED: vz NED = negative of vz_up
        vz = -vz_up

        return (vn, ve, vz, desired_yaw)

    def compute_landing_velocity(self, offset_fwd: float, offset_right: float,
                                  phase: str = "align",
                                  should_descend: bool = True) -> tuple:
        """
        Compute velocity command for precision landing centering.
        
        Uses separate PID gains for landing precision.
        
        Args:
            offset_fwd: Forward offset to target (m, positive = target ahead)
            offset_right: Right offset to target (m, positive = target right)
            phase: "align" or "final" (different gains/limits)
            should_descend: If True, include descent velocity
            
        Returns:
            (vx, vy, vz) in NED frame (m/s)
        """
        # Select gains based on phase
        if phase == "final":
            gains = self.config.final_gains
            speed_limit = self.config.landing_final_speed
            descent_rate = self.config.final_descent_rate
            accel_limit = self.config.landing_accel_limit * 0.5
        else:
            gains = self.config.align_gains
            speed_limit = self.config.landing_horiz_speed
            descent_rate = self.config.align_descent_rate
            accel_limit = self.config.landing_accel_limit

        # Update PIDs if gains changed
        self._pid_land_fwd.kp = gains.kp
        self._pid_land_fwd.kd = gains.kd
        self._pid_land_right.kp = gains.kp
        self._pid_land_right.kd = gains.kd
        self._smooth_land_fwd.max_accel = accel_limit
        self._smooth_land_right.max_accel = accel_limit

        # PID: negative because we want to move OPPOSITE to offset
        raw_vx = self._pid_land_fwd.update(-offset_fwd)
        raw_vy = self._pid_land_right.update(-offset_right)

        # Clamp
        speed = math.sqrt(raw_vx ** 2 + raw_vy ** 2)
        if speed > speed_limit and speed > 0:
            scale = speed_limit / speed
            raw_vx *= scale
            raw_vy *= scale

        # Smooth
        vx = self._smooth_land_fwd.update(raw_vx)
        vy = self._smooth_land_right.update(raw_vy)

        # Descent
        vz = descent_rate if should_descend else 0.0

        return (vx, vy, vz)

    def compute_altitude_hold(self, target_alt: float, current_alt: float,
                               dt: Optional[float] = None) -> float:
        """Compute vz to hold a target altitude. Returns NED vz."""
        err = target_alt - current_alt
        if math.isnan(err):
            return 0.0
        raw = self._pid_alt.update(err, dt)
        raw = max(-self.config.descent_max_speed,
                  min(self.config.ascent_max_speed, raw))
        smoothed = self._smooth_alt.update(raw, dt)
        return -smoothed  # NED: negative = up
