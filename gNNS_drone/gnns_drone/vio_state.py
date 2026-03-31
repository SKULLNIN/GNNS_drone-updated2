"""
gNNS Drone — VIO State
========================
Rich output dataclass produced by the VIO pipeline every camera frame.

This is the single, authoritative data structure that flows from:
  VIOAlgorithm.process_frame()
    → SafetyMonitor._check_vio_state()
    → TakeoffController
    → MAVLinkBridge.send_vision_position() / send_optical_flow_rad()

Fields are grouped by subsystem to make downstream consumers self-documenting.
"""

import time
import math
from dataclasses import dataclass, field
from typing import List
from .vio_tracker import VIOStatus


@dataclass
class VIOState:
    """
    Complete VIO pipeline output for one camera frame.

    All positions/velocities are in NED frame (North-East-Down, metres / m/s).
    All angles are in radians.

    Consumers should check `state` and `confidence` before trusting pose data:
      if state == VIOStatus.LOST or confidence < 40:
          # do not forward to FC
    """

    # ------------------------------------------------------------------ #
    # Timing
    # ------------------------------------------------------------------ #
    timestamp: float = 0.0
    """Unix wall-clock seconds at end of processing this frame."""

    # ------------------------------------------------------------------ #
    # IMU integration diagnostics
    # ------------------------------------------------------------------ #
    imu_dt: float = 0.0
    """Time in seconds covered by the IMU pre-integration for this frame.
    Nominal: ~0.033 s at 30 Hz.  Spikes > 0.5 s indicate IMU dropout."""

    imu_sample_count: int = 0
    """Number of IMU samples integrated (nominal: 6–7 at 200 Hz / 30 Hz)."""

    accel_bias: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    """Estimated accelerometer bias [bax, bay, baz] m/s².  Updated by EKF."""

    gyro_bias: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    """Estimated gyroscope bias [bgx, bgy, bgz] rad/s.  Updated by EKF."""

    # ------------------------------------------------------------------ #
    # Position — NED frame, metres from takeoff origin
    # ------------------------------------------------------------------ #
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0

    # ------------------------------------------------------------------ #
    # Velocity — NED frame, m/s
    # ------------------------------------------------------------------ #
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0

    # ------------------------------------------------------------------ #
    # Acceleration — body frame, m/s², bias-corrected
    # ------------------------------------------------------------------ #
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0

    # ------------------------------------------------------------------ #
    # Attitude
    # ------------------------------------------------------------------ #
    roll: float = 0.0
    """Roll angle (rad).  Positive = right wing down."""

    pitch: float = 0.0
    """Pitch angle (rad).  Positive = nose up."""

    yaw: float = 0.0
    """Yaw angle (rad), NED convention.  0 = North, π/2 = East."""

    # ------------------------------------------------------------------ #
    # Body angular rates from gyro, bias-corrected (rad/s)
    # ------------------------------------------------------------------ #
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

    # ------------------------------------------------------------------ #
    # Feature tracking quality
    # ------------------------------------------------------------------ #
    num_features_detected: int = 0
    """Features found by detector in this frame (before tracking)."""

    num_features_tracked: int = 0
    """Features successfully tracked from previous frame (after LK pass)."""

    num_inliers: int = 0
    """Geometrically consistent tracks after RANSAC fundamental-matrix check."""

    tracking_ratio: float = 0.0
    """num_inliers / max(num_features_tracked, 1).  Range 0–1.
    < 0.5 indicates large motion / poor texture / occlusion."""

    mean_flow_magnitude: float = 0.0
    """Mean optical-flow displacement across all tracked features (pixels/frame).
    High values during hover indicate drift or vibration."""

    frames_since_detect: int = 0
    """Frames elapsed since the last feature re-detection.
    Re-detection is triggered when num_features_tracked < min_features."""

    # ------------------------------------------------------------------ #
    # Depth / range sensors
    # ------------------------------------------------------------------ #
    depth_range_m: float = -1.0
    """Nearest valid depth reading from D455 depth map (metres).
    -1 if depth is unavailable or all pixels are invalid."""

    lidar_alt_m: float = -1.0
    """Altitude from nadir-facing LiDAR bin (metres AGL).
    -1 if LiDAR scan has not been received in the last 500 ms."""

    # ------------------------------------------------------------------ #
    # VIO health / state
    # ------------------------------------------------------------------ #
    state: VIOStatus = VIOStatus.UNINITIALIZED
    """High-level state of the VIO pipeline (mirrors MAVLink GNNS_VIO_STATUS)."""

    confidence: float = 0.0
    """Composite confidence score 0–100.

    Formula:
      0.30 × clamp(num_inliers/200, 0, 1) × 100   # feature count
    + 0.30 × tracking_ratio × 100                   # geometric inlier ratio
    + 0.20 × (1 − clamp(pos_cov_norm, 0, 1)) × 100 # EKF uncertainty
    + 0.10 × (1 if imu_dt < 0.04 else 0) × 100     # IMU rate health
    + 0.10 × (1 if lidar_alt_m > 0 else 0) × 100   # LiDAR available

    State from confidence:
      ≥ 70  → TRACKING
      40–70 → DEGRADED
      < 40 for 2 s → LOST → RELOCALIZING
    """

    position_covariance: List[float] = field(
        default_factory=lambda: [1.0, 1.0, 1.0]
    )
    """Diagonal of EKF position covariance [σ²_north, σ²_east, σ²_down] m²."""

    attitude_covariance: List[float] = field(
        default_factory=lambda: [0.1, 0.1, 0.1]
    )
    """Diagonal of EKF attitude covariance [σ²_roll, σ²_pitch, σ²_yaw] rad²."""

    # ------------------------------------------------------------------ #
    # Optical flow (PX4Flow-style raw output for MAVLink OPTICAL_FLOW_RAD)
    # ------------------------------------------------------------------ #
    flow_vx_ms: float = 0.0
    """Body-frame X velocity estimate from optical flow (m/s, forward +)."""

    flow_vy_ms: float = 0.0
    """Body-frame Y velocity estimate from optical flow (m/s, right +)."""

    flow_quality: int = 0
    """Optical flow quality 0–255 (255 = best).  Computed from tracking_ratio
    and mean_flow_magnitude stability."""

    # ------------------------------------------------------------------ #
    # RTABMap / loop-closure
    # ------------------------------------------------------------------ #
    loop_closure_id: int = -1
    """ID of the matched prior node if a loop closure was detected this frame.
    -1 means no closure."""

    map_nodes: int = 0
    """Total nodes in the RTAB-Map working graph."""

    # ------------------------------------------------------------------ #
    # Processing diagnostics
    # ------------------------------------------------------------------ #
    processing_time_ms: float = 0.0
    """Wall-clock time taken to produce this VIOState (ms).
    Nominal < 15 ms on Jetson Orin; > 30 ms may cause frame drops."""

    pipeline_stage: str = "output"
    """Stage at which this state was produced.
    One of: 'warmup' | 'detect' | 'track' | 'ekf' | 'output'."""

    # ------------------------------------------------------------------ #
    # Convenience properties
    # ------------------------------------------------------------------ #

    @property
    def altitude(self) -> float:
        """Altitude above takeoff point (metres, positive up)."""
        return -self.down

    @property
    def speed_horizontal(self) -> float:
        """Horizontal ground speed (m/s)."""
        return math.sqrt(self.vn ** 2 + self.ve ** 2)

    @property
    def speed_3d(self) -> float:
        """3-D speed magnitude (m/s)."""
        return math.sqrt(self.vn ** 2 + self.ve ** 2 + self.vd ** 2)

    @property
    def age_sec(self) -> float:
        """Seconds since this state was stamped."""
        return time.time() - self.timestamp if self.timestamp > 0 else float("inf")

    @property
    def is_stale(self) -> bool:
        """True if the state has not been refreshed for more than 2 s."""
        return self.age_sec > 2.0

    @property
    def is_tracking(self) -> bool:
        """True when confidence is high enough to trust for FC injection."""
        return self.state in (VIOStatus.TRACKING, VIOStatus.DEGRADED) and self.confidence >= 30.0

    @property
    def pos_cov_scalar(self) -> float:
        """Scalar position uncertainty: mean diagonal of position covariance."""
        return sum(self.position_covariance) / 3.0

    def to_dict(self) -> dict:
        """Serialise to a plain dict (for logging / JSON / MAVLink debug)."""
        return {
            "timestamp": self.timestamp,
            "imu_dt": self.imu_dt,
            "imu_sample_count": self.imu_sample_count,
            "accel_bias": self.accel_bias,
            "gyro_bias": self.gyro_bias,
            "north": self.north,
            "east": self.east,
            "down": self.down,
            "vn": self.vn,
            "ve": self.ve,
            "vd": self.vd,
            "ax": self.ax,
            "ay": self.ay,
            "az": self.az,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "roll_rate": self.roll_rate,
            "pitch_rate": self.pitch_rate,
            "yaw_rate": self.yaw_rate,
            "num_features_detected": self.num_features_detected,
            "num_features_tracked": self.num_features_tracked,
            "num_inliers": self.num_inliers,
            "tracking_ratio": self.tracking_ratio,
            "mean_flow_magnitude": self.mean_flow_magnitude,
            "frames_since_detect": self.frames_since_detect,
            "depth_range_m": self.depth_range_m,
            "lidar_alt_m": self.lidar_alt_m,
            "state": self.state.name,
            "confidence": self.confidence,
            "position_covariance": self.position_covariance,
            "attitude_covariance": self.attitude_covariance,
            "flow_vx_ms": self.flow_vx_ms,
            "flow_vy_ms": self.flow_vy_ms,
            "flow_quality": self.flow_quality,
            "loop_closure_id": self.loop_closure_id,
            "map_nodes": self.map_nodes,
            "processing_time_ms": self.processing_time_ms,
            "pipeline_stage": self.pipeline_stage,
        }

    def pretty_print(self):
        """Print a human-readable status block."""
        print(f"\n{'─'*54}")
        print(f"  VIOState  [{self.state.name}]  conf={self.confidence:.1f}%  "
              f"age={self.age_sec*1000:.0f}ms")
        print(f"{'─'*54}")
        print(f"  IMU      dt={self.imu_dt*1000:.1f}ms  "
              f"samples={self.imu_sample_count}  "
              f"abias=[{self.accel_bias[0]:.3f},{self.accel_bias[1]:.3f},{self.accel_bias[2]:.3f}]  "
              f"gbias=[{self.gyro_bias[0]:.4f},{self.gyro_bias[1]:.4f},{self.gyro_bias[2]:.4f}]")
        print(f"  Pos(NED) N={self.north:+.3f}  E={self.east:+.3f}  "
              f"D={self.down:+.3f}  alt={self.altitude:.2f}m")
        print(f"  Vel(NED) vN={self.vn:+.2f}  vE={self.ve:+.2f}  "
              f"vD={self.vd:+.2f}  spd={self.speed_3d:.2f}m/s")
        print(f"  Att      R={math.degrees(self.roll):+.1f}°  "
              f"P={math.degrees(self.pitch):+.1f}°  "
              f"Y={math.degrees(self.yaw):+.1f}°")
        print(f"  Rates    ṙ={math.degrees(self.roll_rate):+.1f}°/s  "
              f"ṗ={math.degrees(self.pitch_rate):+.1f}°/s  "
              f"ẏ={math.degrees(self.yaw_rate):+.1f}°/s")
        print(f"  Features det={self.num_features_detected}  "
              f"trk={self.num_features_tracked}  "
              f"inliers={self.num_inliers}  "
              f"ratio={self.tracking_ratio:.2f}  "
              f"flow={self.mean_flow_magnitude:.1f}px")
        print(f"  Depth    range={self.depth_range_m:.2f}m  "
              f"lidar={self.lidar_alt_m:.2f}m")
        print(f"  Flow     vx={self.flow_vx_ms:.2f}m/s  "
              f"vy={self.flow_vy_ms:.2f}m/s  "
              f"qual={self.flow_quality}")
        print(f"  RTABMap  closure={self.loop_closure_id}  "
              f"nodes={self.map_nodes}")
        print(f"  Timing   {self.processing_time_ms:.1f}ms  "
              f"stage={self.pipeline_stage}")
        print(f"  PosCov   {self.position_covariance}")
        print(f"{'─'*54}")
