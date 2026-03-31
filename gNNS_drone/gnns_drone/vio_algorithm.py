"""
gNNS Drone — VIO Algorithm Core
=================================
Custom Visual-Inertial Odometry pipeline for GPS-denied navigation.

Pipeline per camera frame (30 Hz):
  1. FeatureDetector     — FAST9 / Shi-Tomasi / Harris / Variance
  2. LKOpticalFlowTracker — Pyramid Lucas-Kanade + RANSAC outlier rejection
  3. IMUPreintegrator    — Midpoint RK2 integration of 200 Hz IMU samples
  4. DepthProjector      — Unproject tracked features to 3-D using D455 depth
  5. VIOEKF              — 16-state EKF fusing visual, flow, LiDAR and RTABMap
  6. VIOAlgorithm        — Orchestrator; computes confidence; returns VIOState

All coordinate frames are NED (North-East-Down).
"""

import time
import math
import logging
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .vio_state import VIOState
from .vio_tracker import VIOStatus

logger = logging.getLogger("gnns.vio_algo")


# ======================================================================
# HELPER: rotation utilities
# ======================================================================

def _quat_to_euler(qw: float, qx: float, qy: float, qz: float
                   ) -> Tuple[float, float, float]:
    """Convert unit quaternion (w,x,y,z) to ZYX Euler angles (roll, pitch, yaw)."""
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


def _euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler → 3×3 rotation matrix (body → world NED)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ],
    ])


def _rot_to_quat(R: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → unit quaternion [w, x, y, z]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two quaternions [w,x,y,z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _skew(v: np.ndarray) -> np.ndarray:
    """3-vector → 3×3 skew-symmetric matrix."""
    return np.array([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0],
    ])


# ======================================================================
# 1. FEATURE DETECTOR
# ======================================================================

class FeatureDetector:
    """
    Detect sparse 2-D feature points in a grayscale frame.

    Supported methods:
      "fast9"      — FAST-9-16 corner detector (fastest, best for real-time)
      "shi_tomasi" — Shi-Tomasi / Good Features to Track (highest quality)
      "harris"     — Harris corner detector
      "variance"   — Block variance map → local maxima (texture-poor scenes)

    All methods return an (N, 1, 2) float32 array compatible with
    cv2.calcOpticalFlowPyrLK.
    """

    METHODS = ("fast9", "shi_tomasi", "harris", "variance")

    def __init__(self, method: str = "fast9", max_corners: int = 300,
                 config: Optional[dict] = None):
        cfg = config or {}
        self.method = method.lower()
        if self.method not in self.METHODS:
            raise ValueError(f"method must be one of {self.METHODS}")

        self.max_corners = max_corners

        # FAST9 params
        self._fast_threshold  = cfg.get("fast_threshold",  20)
        self._fast_nms        = cfg.get("fast_nms",        True)

        # Shi-Tomasi params
        self._st_quality      = cfg.get("st_quality",      0.01)
        self._st_min_dist     = cfg.get("st_min_dist",     10)

        # Harris params
        self._harris_k        = cfg.get("harris_k",        0.04)
        self._harris_block    = cfg.get("harris_block",    3)
        self._harris_ksize    = cfg.get("harris_ksize",    3)
        self._harris_thresh   = cfg.get("harris_thresh",   0.01)

        # Variance params
        self._var_block       = cfg.get("var_block",       16)
        self._var_min_dist    = cfg.get("var_min_dist",    12)

        # Grid-based suppression: divide frame into cells, take best per cell
        self._grid_cols       = cfg.get("grid_cols",       8)
        self._grid_rows       = cfg.get("grid_rows",       6)

        # Lazy-import cv2 so unit tests can import without OpenCV
        self._cv2 = None

    def _cv(self):
        if self._cv2 is None:
            import cv2
            self._cv2 = cv2
        return self._cv2

    def detect(self, gray: np.ndarray) -> np.ndarray:
        """
        Detect feature points.

        Args:
            gray: uint8 grayscale image (H × W).

        Returns:
            np.ndarray of shape (N, 1, 2) float32.
            May be empty (shape (0, 1, 2)) if no features found.
        """
        cv2 = self._cv()
        h, w = gray.shape[:2]

        if self.method == "fast9":
            pts = self._detect_fast9(cv2, gray)
        elif self.method == "shi_tomasi":
            pts = self._detect_shi_tomasi(cv2, gray)
        elif self.method == "harris":
            pts = self._detect_harris(cv2, gray)
        else:  # variance
            pts = self._detect_variance(cv2, gray)

        if pts is None or len(pts) == 0:
            return np.empty((0, 1, 2), dtype=np.float32)

        # Apply grid-based non-maximum suppression to spread points evenly
        pts = self._grid_nms(pts, h, w)

        # Cap to max_corners
        if len(pts) > self.max_corners:
            pts = pts[:self.max_corners]

        return pts.reshape(-1, 1, 2).astype(np.float32)

    # ------------------------------------------------------------------
    def _detect_fast9(self, cv2, gray: np.ndarray) -> Optional[np.ndarray]:
        detector = cv2.FastFeatureDetector_create(
            threshold=self._fast_threshold,
            nonmaxSuppression=self._fast_nms,
            type=cv2.FastFeatureDetector_TYPE_9_16,
        )
        kps = detector.detect(gray, None)
        if not kps:
            return None
        # Sort by response (strongest first)
        kps = sorted(kps, key=lambda k: k.response, reverse=True)
        pts = np.array([[kp.pt] for kp in kps], dtype=np.float32)
        return pts

    def _detect_shi_tomasi(self, cv2, gray: np.ndarray) -> Optional[np.ndarray]:
        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self._st_quality,
            minDistance=self._st_min_dist,
            blockSize=3,
        )
        return pts  # already (N,1,2) float32 or None

    def _detect_harris(self, cv2, gray: np.ndarray) -> Optional[np.ndarray]:
        gray_f = gray.astype(np.float32)
        dst = cv2.cornerHarris(gray_f, self._harris_block,
                               self._harris_ksize, self._harris_k)
        dst = cv2.dilate(dst, None)
        threshold = self._harris_thresh * dst.max()
        mask = dst > threshold
        ys, xs = np.where(mask)
        if len(xs) == 0:
            return None
        # Sort by response
        responses = dst[ys, xs]
        order = np.argsort(-responses)
        xs, ys = xs[order], ys[order]
        pts = np.stack([xs, ys], axis=1).astype(np.float32)
        return pts.reshape(-1, 1, 2)

    def _detect_variance(self, cv2, gray: np.ndarray) -> Optional[np.ndarray]:
        """Block-variance map: compute local variance, find maxima."""
        b = self._var_block
        h, w = gray.shape
        g = gray.astype(np.float32)

        # Compute variance with box filter
        mean   = cv2.boxFilter(g, cv2.CV_32F, (b, b))
        mean_sq = cv2.boxFilter(g * g, cv2.CV_32F, (b, b))
        var = mean_sq - mean * mean
        var = np.maximum(var, 0.0)

        # Non-maximum suppression with step = min_dist
        d = self._var_min_dist
        candidates = []
        local_max = (var == cv2.dilate(var, np.ones((d, d), np.uint8)))
        ys, xs = np.where(local_max & (var > 10.0))
        if len(xs) == 0:
            return None
        responses = var[ys, xs]
        order = np.argsort(-responses)
        xs, ys = xs[order], ys[order]
        pts = np.stack([xs, ys], axis=1).astype(np.float32)
        return pts.reshape(-1, 1, 2)

    # ------------------------------------------------------------------
    def _grid_nms(self, pts: np.ndarray, h: int, w: int) -> np.ndarray:
        """Keep at most one point per grid cell (uniform distribution)."""
        xy = pts.reshape(-1, 2)
        cell_w = w / self._grid_cols
        cell_h = h / self._grid_rows
        seen = {}
        kept = []
        for pt in xy:
            cx = int(pt[0] / cell_w)
            cy = int(pt[1] / cell_h)
            key = (min(cx, self._grid_cols - 1),
                   min(cy, self._grid_rows - 1))
            if key not in seen:
                seen[key] = True
                kept.append(pt)
        if not kept:
            return np.empty((0, 2), dtype=np.float32)
        return np.array(kept, dtype=np.float32)


# ======================================================================
# 2. LUCAS-KANADE OPTICAL FLOW TRACKER
# ======================================================================

class LKOpticalFlowTracker:
    """
    Pyramid Lucas-Kanade optical flow with:
      • Forward-backward consistency check (sub-pixel error threshold)
      • RANSAC fundamental-matrix geometric outlier rejection
      • Auto re-detection when tracked feature count < min_features

    Returns inlier pairs suitable for EKF visual update.
    """

    def __init__(self, detector: FeatureDetector,
                 min_features: int = 80,
                 config: Optional[dict] = None):
        cfg = config or {}
        self.detector = detector
        self.min_features = min_features

        # LK parameters
        win = cfg.get("win_size", 21)
        self._lk_params = dict(
            winSize=(win, win),
            maxLevel=cfg.get("max_level", 3),
            criteria=(
                3,  # COUNT | EPS
                cfg.get("max_iter", 30),
                cfg.get("epsilon", 0.01),
            ),
            flags=0,
            minEigThreshold=cfg.get("min_eig_thresh", 1e-4),
        )

        # Forward-backward max error (pixels)
        self._fb_max_err = cfg.get("fb_max_err", 1.5)

        # RANSAC threshold (pixels for fundamental matrix)
        self._ransac_thresh = cfg.get("ransac_thresh", 3.0)
        self._ransac_conf   = cfg.get("ransac_conf",   0.99)
        self._min_pts_ransac = cfg.get("min_pts_ransac", 8)

        self._cv2 = None

        # State between frames
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts:  Optional[np.ndarray] = None   # (N,1,2) float32
        self._frames_since_detect: int = 0

    def _cv(self):
        if self._cv2 is None:
            import cv2
            self._cv2 = cv2
        return self._cv2

    def reset(self):
        """Reset tracker state (called on VIO re-initialization)."""
        self._prev_gray = None
        self._prev_pts  = None
        self._frames_since_detect = 0

    def track(self, curr_gray: np.ndarray
              ) -> Tuple[np.ndarray, np.ndarray, int, int, float]:
        """
        Track features from previous frame to current frame.

        Args:
            curr_gray: uint8 grayscale current frame.

        Returns:
            (prev_inliers, curr_inliers, num_tracked, num_inliers, mean_flow)
              prev_inliers, curr_inliers: (K, 1, 2) float32 matched pairs
              num_tracked:  features surviving LK (before RANSAC)
              num_inliers:  features surviving RANSAC
              mean_flow:    mean displacement magnitude in pixels
        """
        cv2 = self._cv()

        # First frame or after reset: detect and store, return empty
        if self._prev_gray is None or self._prev_pts is None or len(self._prev_pts) == 0:
            self._prev_gray = curr_gray.copy()
            self._prev_pts  = self.detector.detect(curr_gray)
            self._frames_since_detect = 0
            empty = np.empty((0, 1, 2), dtype=np.float32)
            return empty, empty, 0, 0, 0.0

        # ---- Forward LK pass ----------------------------------------
        curr_pts, st_fwd, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, curr_gray,
            self._prev_pts, None, **self._lk_params
        )

        # ---- Backward consistency check -----------------------------
        prev_back, st_bwd, _ = cv2.calcOpticalFlowPyrLK(
            curr_gray, self._prev_gray,
            curr_pts, None, **self._lk_params
        )

        fb_err = np.linalg.norm(
            self._prev_pts.reshape(-1, 2) - prev_back.reshape(-1, 2),
            axis=1
        )
        fb_mask = (st_fwd.ravel() == 1) & (st_bwd.ravel() == 1) & \
                  (fb_err < self._fb_max_err)

        prev_good = self._prev_pts.reshape(-1, 2)[fb_mask]
        curr_good = curr_pts.reshape(-1, 2)[fb_mask]
        num_tracked = int(fb_mask.sum())

        if num_tracked < 4:
            # Not enough to do anything useful; trigger re-detect
            inlier_mask = np.ones(num_tracked, dtype=bool)
        else:
            inlier_mask = np.ones(num_tracked, dtype=bool)

        # ---- RANSAC fundamental-matrix outlier rejection ------------
        if num_tracked >= self._min_pts_ransac:
            _, ransac_mask = cv2.findFundamentalMat(
                prev_good, curr_good,
                cv2.FM_RANSAC,
                self._ransac_thresh,
                self._ransac_conf,
            )
            if ransac_mask is not None:
                inlier_mask = ransac_mask.ravel().astype(bool)

        prev_inliers = prev_good[inlier_mask].reshape(-1, 1, 2)
        curr_inliers = curr_good[inlier_mask].reshape(-1, 1, 2)
        num_inliers  = int(inlier_mask.sum())

        # ---- Mean optical flow magnitude ---------------------------
        if num_inliers > 0:
            flow_vecs  = curr_inliers.reshape(-1, 2) - prev_inliers.reshape(-1, 2)
            mean_flow  = float(np.mean(np.linalg.norm(flow_vecs, axis=1)))
        else:
            mean_flow = 0.0

        # ---- Update state for next frame ---------------------------
        self._frames_since_detect += 1

        if num_inliers < self.min_features:
            # Re-detect from current frame
            new_pts = self.detector.detect(curr_gray)
            self._prev_pts = new_pts
            self._frames_since_detect = 0
        else:
            self._prev_pts = curr_inliers.copy()

        self._prev_gray = curr_gray.copy()

        return prev_inliers, curr_inliers, num_tracked, num_inliers, mean_flow

    @property
    def frames_since_detect(self) -> int:
        return self._frames_since_detect


# ======================================================================
# 3. IMU PRE-INTEGRATOR
# ======================================================================

@dataclass
class _IMUSample:
    ax: float; ay: float; az: float
    wx: float; wy: float; wz: float
    t: float


@dataclass
class PreintegrationResult:
    """
    Result of integrating IMU samples between two camera frames.

    delta_p: 3-vector position change in body frame at start (metres)
    delta_v: 3-vector velocity change (m/s)
    delta_R: 3×3 rotation matrix (attitude change, body_start → body_end)
    dt:      total integration time (seconds)
    sample_count: number of IMU samples used
    covariance: 9×9 covariance of [delta_p, delta_v, delta_R_vec]
    last_gyro:  bias-corrected gyro reading at end of interval (rad/s)
    last_accel: bias-corrected accel reading at end of interval (m/s²)
    """
    delta_p:      np.ndarray = field(default_factory=lambda: np.zeros(3))
    delta_v:      np.ndarray = field(default_factory=lambda: np.zeros(3))
    delta_R:      np.ndarray = field(default_factory=lambda: np.eye(3))
    dt:           float = 0.0
    sample_count: int   = 0
    covariance:   np.ndarray = field(default_factory=lambda: np.eye(9) * 1e-4)
    last_gyro:    np.ndarray = field(default_factory=lambda: np.zeros(3))
    last_accel:   np.ndarray = field(default_factory=lambda: np.zeros(3))


class IMUPreintegrator:
    """
    Pre-integrates high-rate IMU samples between camera frames.

    Method: midpoint (RK-2) integration on SO(3) rotation, trapezoid on vel/pos.
    Propagates a 9×9 state covariance for the pre-integration quantities.

    Bias fields are updated by VIOEKF.update_bias() and subtracted automatically.
    """

    # Noise spectral densities (tuned for RealSense D455 IMU)
    ACCEL_NOISE_DENSITY = 2.0e-3   # m/s²/√Hz  (D455 datasheet ~1.4 e-3)
    GYRO_NOISE_DENSITY  = 1.7e-4   # rad/s/√Hz
    ACCEL_BIAS_WALK     = 3.0e-5   # m/s³/√Hz
    GYRO_BIAS_WALK      = 2.2e-6   # rad/s²/√Hz

    def __init__(self, config: Optional[dict] = None):
        cfg = config or {}
        self._buffer: deque = deque()
        self.accel_bias = np.zeros(3)   # updated from EKF
        self.gyro_bias  = np.zeros(3)   # updated from EKF

        # Noise parameters (may be overridden from config)
        self._an = cfg.get("accel_noise", self.ACCEL_NOISE_DENSITY)
        self._gn = cfg.get("gyro_noise",  self.GYRO_NOISE_DENSITY)
        self._aw = cfg.get("accel_walk",  self.ACCEL_BIAS_WALK)
        self._gw = cfg.get("gyro_walk",   self.GYRO_BIAS_WALK)

    def push(self, ax: float, ay: float, az: float,
             wx: float, wy: float, wz: float, t: float):
        """Add one IMU sample (body frame, SI units, Unix seconds)."""
        self._buffer.append(_IMUSample(ax, ay, az, wx, wy, wz, t))

    def integrate(self) -> PreintegrationResult:
        """
        Consume all buffered samples and return pre-integration result.
        Buffer is cleared after call.

        If buffer is empty, returns a zero-motion result with large covariance.
        """
        samples = list(self._buffer)
        self._buffer.clear()

        if len(samples) < 2:
            result = PreintegrationResult()
            result.covariance = np.eye(9) * 1.0   # large uncertainty
            return result

        gravity_ned = np.array([0.0, 0.0, 9.81])  # NED frame: g is positive-down

        # Integration state
        delta_p = np.zeros(3)
        delta_v = np.zeros(3)
        delta_R = np.eye(3)

        # Covariance propagation matrices
        P = np.zeros((9, 9))   # [dp, dv, dphi]
        # Discrete noise covariance per step (scaled by dt²)
        an2 = self._an ** 2
        gn2 = self._gn ** 2
        aw2 = self._aw ** 2
        gw2 = self._gw ** 2

        t_start = samples[0].t
        t_end   = samples[-1].t
        dt_total = t_end - t_start

        prev_s = samples[0]
        last_gyro  = np.zeros(3)
        last_accel = np.zeros(3)

        for s in samples[1:]:
            dt = s.t - prev_s.t
            if dt <= 0 or dt > 0.5:
                prev_s = s
                continue

            # Bias-corrected measurements (midpoint of two samples)
            a0 = np.array([prev_s.ax, prev_s.ay, prev_s.az]) - self.accel_bias
            a1 = np.array([s.ax,      s.ay,      s.az])      - self.accel_bias
            w0 = np.array([prev_s.wx, prev_s.wy, prev_s.wz]) - self.gyro_bias
            w1 = np.array([s.wx,      s.wy,      s.wz])      - self.gyro_bias

            a_mid = 0.5 * (a0 + a1)   # midpoint accel
            w_mid = 0.5 * (w0 + w1)   # midpoint gyro

            # RK-2 rotation integration
            theta = w_mid * dt
            angle = np.linalg.norm(theta)
            if angle > 1e-8:
                axis = theta / angle
                K    = _skew(axis)
                dR   = np.eye(3) + math.sin(angle) * K + \
                       (1.0 - math.cos(angle)) * (K @ K)
            else:
                dR = np.eye(3) + _skew(theta)

            # Rotate accel to world frame before integration
            a_world = delta_R @ a_mid

            # Trapezoidal position/velocity integration
            delta_p += delta_v * dt + 0.5 * a_world * dt * dt
            delta_v += a_world * dt
            delta_R  = delta_R @ dR

            # ---- Covariance propagation (linearised) ----------------
            # State: [delta_p(3), delta_v(3), delta_phi(3)]
            F = np.eye(9)
            F[0:3, 3:6] = np.eye(3) * dt
            F[0:3, 6:9] = -delta_R @ _skew(a_mid) * dt * dt * 0.5
            F[3:6, 6:9] = -delta_R @ _skew(a_mid) * dt

            Q = np.zeros((9, 9))
            Q[3:6, 3:6] = an2 * dt * np.eye(3)
            Q[6:9, 6:9] = gn2 * dt * np.eye(3)
            # Bias walk (extends to 9×9 but we keep 9-state approximation)
            Q[0:3, 0:3] += aw2 * dt * np.eye(3) * (dt ** 2)

            P = F @ P @ F.T + Q

            last_gyro  = w1
            last_accel = a1
            prev_s = s

        result = PreintegrationResult(
            delta_p=delta_p,
            delta_v=delta_v,
            delta_R=delta_R,
            dt=dt_total,
            sample_count=len(samples),
            covariance=P,
            last_gyro=last_gyro,
            last_accel=last_accel,
        )
        return result

    def reset(self):
        """Clear buffer (called after re-initialisation)."""
        self._buffer.clear()

    def update_bias(self, accel_bias: np.ndarray, gyro_bias: np.ndarray):
        """Called by VIOEKF to feed back corrected bias estimates."""
        self.accel_bias = accel_bias.copy()
        self.gyro_bias  = gyro_bias.copy()


# ======================================================================
# 4. DEPTH PROJECTOR
# ======================================================================

class DepthProjector:
    """
    Unproject 2-D tracked feature points to 3-D using the D455 depth map.

    Uses the depth intrinsics from the RealSense SDK when available,
    or falls back to a configurable pinhole model.
    """

    def __init__(self, config: Optional[dict] = None):
        cfg = config or {}
        # Default D455 depth intrinsics (640×480 mode)
        self.fx = cfg.get("fx", 384.0)
        self.fy = cfg.get("fy", 384.0)
        self.cx = cfg.get("cx", 320.0)
        self.cy = cfg.get("cy", 240.0)
        self.depth_scale = cfg.get("depth_scale", 0.001)   # RealSense default 1mm
        self.min_depth   = cfg.get("min_depth",   0.3)     # metres
        self.max_depth   = cfg.get("max_depth",   8.0)     # metres

    def update_intrinsics(self, fx, fy, cx, cy, depth_scale):
        """Update camera intrinsics (called from VIOAlgorithm on stream start)."""
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.depth_scale = depth_scale

    def get_nearest_valid(self, depth_frame: np.ndarray) -> float:
        """
        Return the nearest valid depth value in the frame (metres).
        Used for depth_range_m field of VIOState.
        Returns -1.0 if no valid depth.
        """
        if depth_frame is None:
            return -1.0
        valid = depth_frame[(depth_frame > 0)]
        if len(valid) == 0:
            return -1.0
        return float(valid.min()) * self.depth_scale

    def unproject(self, pts2d: np.ndarray,
                  depth_frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Unproject 2-D points to 3-D camera-frame coordinates.

        Args:
            pts2d:       (N, 1, 2) float32 pixel coordinates.
            depth_frame: (H, W) uint16 depth image (raw values).

        Returns:
            pts3d:  (M, 3) float32 — valid 3-D points (camera frame: x-right, y-down, z-fwd)
            valid:  (N,)   bool mask — which input points had valid depth
        """
        if depth_frame is None or len(pts2d) == 0:
            return np.empty((0, 3), dtype=np.float32), \
                   np.zeros(len(pts2d), dtype=bool)

        xy    = pts2d.reshape(-1, 2)
        n     = len(xy)
        valid = np.zeros(n, dtype=bool)
        pts3d = np.zeros((n, 3), dtype=np.float32)
        h, w  = depth_frame.shape[:2]

        for i, (u, v) in enumerate(xy):
            ui, vi = int(round(u)), int(round(v))
            if not (0 <= ui < w and 0 <= vi < h):
                continue
            d_raw = float(depth_frame[vi, ui])
            if d_raw == 0:
                continue
            d = d_raw * self.depth_scale
            if not (self.min_depth <= d <= self.max_depth):
                continue
            x = (u - self.cx) * d / self.fx
            y = (v - self.cy) * d / self.fy
            z = d
            pts3d[i] = [x, y, z]
            valid[i] = True

        return pts3d[valid], valid


# ======================================================================
# 5. VIO EKF (16-state)
# ======================================================================

class VIOEKF:
    """
    16-state Extended Kalman Filter for Visual-Inertial Odometry.

    State vector (NED frame):
      x[0:3]   = position      p  (metres)
      x[3:6]   = velocity      v  (m/s)
      x[6:10]  = attitude quat [w, x, y, z]  (unit quaternion body→NED)
      x[10:13] = accel bias    ba (m/s²)
      x[13:16] = gyro  bias    bg (rad/s)

    Update sources:
      propagate()           — IMU pre-integration prediction step
      update_visual()       — 3-D point reprojection residuals
      update_flow_velocity()— optical-flow velocity pseudo-measurement
      update_lidar_alt()    — LiDAR range → NED-down update
      update_rtabmap()      — full 6-DOF pose from VOXL/RTABMap
    """

    N = 16   # state dimension

    # Process noise standard deviations (tunable)
    _SIG_POS   = 0.02    # m/step
    _SIG_VEL   = 0.05    # m/s/step
    _SIG_ATT   = 0.005   # rad/step
    _SIG_ABIAS = 1e-5
    _SIG_GBIAS = 1e-6

    def __init__(self, config: Optional[dict] = None):
        cfg = config or {}
        n = self.N

        # State — initialised at origin, identity attitude
        self.x = np.zeros(n)
        self.x[6] = 1.0   # qw = 1 (identity quaternion)

        # Covariance
        self.P = np.eye(n) * cfg.get("init_cov", 0.1)
        self.P[6:10, 6:10] = np.eye(4) * 1e-4   # quaternion tightly initialised
        self.P[10:13, 10:13] = np.eye(3) * 1e-3  # accel bias
        self.P[13:16, 13:16] = np.eye(3) * 1e-4  # gyro bias

        # Process noise covariance Q (diagonal)
        self.Q = np.zeros((n, n))
        self.Q[0:3, 0:3]   = np.eye(3) * cfg.get("q_pos",   self._SIG_POS**2)
        self.Q[3:6, 3:6]   = np.eye(3) * cfg.get("q_vel",   self._SIG_VEL**2)
        self.Q[6:10, 6:10] = np.eye(4) * cfg.get("q_att",   self._SIG_ATT**2)
        self.Q[10:13, 10:13] = np.eye(3)*cfg.get("q_abias", self._SIG_ABIAS**2)
        self.Q[13:16, 13:16] = np.eye(3)*cfg.get("q_gbias", self._SIG_GBIAS**2)

        # Gravity NED (m/s²)
        self._g = np.array([0.0, 0.0, 9.81])

        # Measurement noise
        self._R_vis   = cfg.get("r_visual",     0.05)  # m per 3-D point
        self._R_flow  = cfg.get("r_flow",       0.10)  # m/s
        self._R_lidar = cfg.get("r_lidar",      0.05)  # m altitude
        self._R_rtab  = cfg.get("r_rtabmap",    0.02)  # m pose

        self._lock = threading.Lock()

    # ------------------------------------------------------------------ #
    # Public accessors
    # ------------------------------------------------------------------ #

    @property
    def position(self) -> np.ndarray:
        return self.x[0:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:6].copy()

    @property
    def quaternion(self) -> np.ndarray:
        return self.x[6:10].copy()

    @property
    def accel_bias(self) -> np.ndarray:
        return self.x[10:13].copy()

    @property
    def gyro_bias(self) -> np.ndarray:
        return self.x[13:16].copy()

    @property
    def euler(self) -> Tuple[float, float, float]:
        q = self.x[6:10]
        return _quat_to_euler(q[0], q[1], q[2], q[3])

    @property
    def pos_covariance(self) -> np.ndarray:
        return np.diag(self.P[0:3, 0:3]).copy()

    @property
    def att_covariance(self) -> np.ndarray:
        return np.diag(self.P[6:9, 6:9]).copy()[:3]

    # ------------------------------------------------------------------ #
    # Prediction step (IMU pre-integration)
    # ------------------------------------------------------------------ #

    def propagate(self, result: PreintegrationResult):
        """
        EKF prediction step using IMU pre-integration result.

        Updates state x and covariance P.
        """
        if result.dt <= 0 or result.sample_count < 2:
            return

        with self._lock:
            p  = self.x[0:3]
            v  = self.x[3:6]
            q  = self.x[6:10]
            ba = self.x[10:13]
            bg = self.x[13:16]

            # Current rotation matrix body→NED
            roll, pitch, yaw = _quat_to_euler(q[0], q[1], q[2], q[3])
            R = _euler_to_rot(roll, pitch, yaw)

            # Propagate position and velocity
            dp_world = R @ result.delta_p
            dv_world = R @ result.delta_v

            new_p = p + v * result.dt + dp_world
            new_v = v + dv_world - self._g * result.dt

            # Propagate attitude: q_new = q ⊗ q(delta_R)
            dR_q = _rot_to_quat(result.delta_R)
            new_q = _quat_mult(q, dR_q)
            new_q = new_q / np.linalg.norm(new_q)

            # Biases unchanged (modelled as random walk in Q)
            self.x[0:3]   = new_p
            self.x[3:6]   = new_v
            self.x[6:10]  = new_q
            # x[10:16] unchanged

            # ---- Covariance prediction (simplified linearisation) ----
            dt = result.dt
            F  = np.eye(self.N)

            # dp/dv
            F[0:3, 3:6]  = np.eye(3) * dt
            # dp/d_phi (via rotation)
            F[0:3, 6:9]  = -R @ _skew(result.delta_p)
            # dv/d_phi
            F[3:6, 6:9]  = -R @ _skew(result.delta_v)
            # dv/d_ba
            F[3:6, 10:13] = -R * dt
            # d_phi/d_bg
            F[6:9, 13:16] = -np.eye(3) * dt

            Q_scaled = self.Q.copy()
            Q_scaled[0:3,  0:3]   *= dt
            Q_scaled[3:6,  3:6]   *= dt
            Q_scaled[6:10, 6:10]  *= dt
            Q_scaled[10:13,10:13] *= dt
            Q_scaled[13:16,13:16] *= dt

            self.P = F @ self.P @ F.T + Q_scaled

    # ------------------------------------------------------------------ #
    # Update: visual 3-D points (position + attitude correction)
    # ------------------------------------------------------------------ #

    def update_visual(self, pts3d_world: np.ndarray):
        """
        Update EKF with 3-D feature positions expressed in world (NED) frame.

        Each point contributes an independent position observation
        h(x) = p  (the drone centre, approximated).
        We use the centroid of all inlier 3-D points as a position pseudo-obs.

        Args:
            pts3d_world: (M, 3) float32 world-frame 3-D positions.
        """
        if len(pts3d_world) < 4:
            return

        with self._lock:
            # Use centroid as a proxy position measurement
            obs = np.mean(pts3d_world, axis=0)
            # Measurement model: h(x) = p + R @ mean_cam_offset ≈ p
            H = np.zeros((3, self.N))
            H[0:3, 0:3] = np.eye(3)

            R_meas = np.eye(3) * (self._R_vis ** 2 / len(pts3d_world))
            self._update(obs, H, R_meas)

    # ------------------------------------------------------------------ #
    # Update: optical flow velocity
    # ------------------------------------------------------------------ #

    def update_flow_velocity(self, flow_vx: float, flow_vy: float,
                              quality: int):
        """
        Update EKF with body-frame velocity derived from optical flow.

        Flow velocity is converted to NED using current yaw estimate.

        Args:
            flow_vx: body-X velocity (m/s, forward +)
            flow_vy: body-Y velocity (m/s, right +)
            quality: 0–255 flow quality
        """
        if quality < 50:
            return

        with self._lock:
            roll, pitch, yaw = _quat_to_euler(*self.x[6:10])
            cy, sy = math.cos(yaw), math.sin(yaw)

            # Body XY → NED horizontal
            vn = cy * flow_vx - sy * flow_vy
            ve = sy * flow_vx + cy * flow_vy
            obs = np.array([vn, ve, self.x[5]])   # keep current vd

            H = np.zeros((3, self.N))
            H[0:3, 3:6] = np.eye(3)

            scale = max(1.0, (255 - quality) / 255.0)
            R_meas = np.eye(3) * (self._R_flow * scale) ** 2
            self._update(obs, H, R_meas)

    # ------------------------------------------------------------------ #
    # Update: LiDAR altitude
    # ------------------------------------------------------------------ #

    def update_lidar_alt(self, range_m: float):
        """
        Update EKF with nadir LiDAR range measurement.

        In NED frame, altitude = -p_z, so the measurement h(x) = -x[2].

        Args:
            range_m: LiDAR range above ground (metres, > 0).
        """
        if range_m <= 0 or range_m > 20.0:
            return

        with self._lock:
            # h(x) = -p_z  (NED down is negative altitude)
            obs = np.array([-range_m])
            H   = np.zeros((1, self.N))
            H[0, 2] = -1.0
            R_meas = np.array([[self._R_lidar ** 2]])
            self._update(obs, H, R_meas)

    # ------------------------------------------------------------------ #
    # Update: RTABMap / VOXL full pose
    # ------------------------------------------------------------------ #

    def update_rtabmap(self, pos_ned: np.ndarray, rpy: np.ndarray,
                       cov6: np.ndarray):
        """
        Update EKF with 6-DOF pose from RTABMap / VOXL.

        Args:
            pos_ned: [north, east, down] position (metres)
            rpy:     [roll, pitch, yaw] (radians)
            cov6:    6×6 covariance matrix (pos, att)
        """
        with self._lock:
            obs = np.concatenate([pos_ned, rpy])
            roll, pitch, yaw = _quat_to_euler(*self.x[6:10])
            pred = np.concatenate([self.x[0:3], [roll, pitch, yaw]])

            H    = np.zeros((6, self.N))
            H[0:3, 0:3]  = np.eye(3)
            H[3:6, 6:9]  = np.eye(3)

            R_meas = cov6 if cov6.shape == (6, 6) else np.eye(6) * self._R_rtab ** 2
            self._update(obs, H, R_meas)

    # ------------------------------------------------------------------ #
    # EKF update kernel (standard KF update equations)
    # ------------------------------------------------------------------ #

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        """
        Standard EKF update (called with self._lock already held).

        z: measurement
        H: measurement Jacobian (m × N)
        R: measurement noise covariance (m × m)
        """
        m = len(z)
        z_pred = H @ self.x[:H.shape[1]] if H.shape[1] <= self.N else H @ self.x

        # Pad H to full state width if needed
        if H.shape[1] < self.N:
            H_full = np.zeros((m, self.N))
            H_full[:, :H.shape[1]] = H
            H = H_full

        y = z - (H @ self.x)   # innovation
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        dx = K @ y
        self.x += dx

        # Re-normalise quaternion after state update
        q_norm = np.linalg.norm(self.x[6:10])
        if q_norm > 1e-8:
            self.x[6:10] /= q_norm

        # Joseph form for numerical stability
        I_KH = np.eye(self.N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    # ------------------------------------------------------------------ #
    # Bias feedback
    # ------------------------------------------------------------------ #

    def update_bias(self, accel_bias: np.ndarray, gyro_bias: np.ndarray):
        """Allow external code to initialise bias (e.g. from calibration)."""
        with self._lock:
            self.x[10:13] = accel_bias
            self.x[13:16] = gyro_bias

    def reset(self):
        """Re-initialise state and covariance (called on RELOCALIZING)."""
        with self._lock:
            self.x[:] = 0.0
            self.x[6] = 1.0
            self.P = np.eye(self.N) * 0.5
            self.P[6:10, 6:10] = np.eye(4) * 1e-4


# ======================================================================
# 6. VIO ALGORITHM ORCHESTRATOR
# ======================================================================

class VIOAlgorithm:
    """
    Complete VIO pipeline orchestrator.

    Ties together: FeatureDetector → LKOpticalFlowTracker →
    IMUPreintegrator → DepthProjector → VIOEKF → VIOState output.

    Usage (called from VIOTracker._run_d435i):

        algo = VIOAlgorithm(detector_method="fast9")
        algo.start()

        # In RealSense frame loop:
        imu_deque  # deque of (ax,ay,az,wx,wy,wz,t)
        state = algo.process_frame(color_frame, depth_frame, list(imu_deque))
        imu_deque.clear()

        # Optionally feed LiDAR and RTABMap
        algo.feed_lidar(range_m)
        algo.feed_rtabmap(pos_ned, rpy, cov6)

        algo.stop()
    """

    WARM_UP_FRAMES = 30   # discard first N frames (auto-exposure settle)

    def __init__(self, detector_method: str = "fast9",
                 config: Optional[dict] = None):
        cfg = config or {}

        self._detector  = FeatureDetector(
            method=detector_method,
            max_corners=cfg.get("max_corners", 300),
            config=cfg.get("detector", {}),
        )
        self._tracker   = LKOpticalFlowTracker(
            detector=self._detector,
            min_features=cfg.get("min_features", 80),
            config=cfg.get("tracker", {}),
        )
        self._imu       = IMUPreintegrator(config=cfg.get("imu", {}))
        self._depth     = DepthProjector(config=cfg.get("depth", {}))
        self._ekf       = VIOEKF(config=cfg.get("ekf", {}))

        self._frame_count   = 0
        self._last_state    = VIOState()
        self._state_lock    = threading.Lock()
        self._running       = False

        # External feed state
        self._lidar_range: float = -1.0
        self._rtabmap_pose: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]] = None
        self._rtabmap_map_nodes: int = 0
        self._rtabmap_closure_id: int = -1

        # Low-confidence tracking timer for state machine
        self._low_conf_start: float = 0.0
        self._vio_status: VIOStatus = VIOStatus.UNINITIALIZED

        # Camera intrinsics (updated on stream start)
        self._fx = cfg.get("fx", 384.0)
        self._fy = cfg.get("fy", 384.0)
        self._cx = cfg.get("cx", 320.0)
        self._cy = cfg.get("cy", 240.0)
        self._depth_scale = cfg.get("depth_scale", 0.001)

        logger.info(f"VIOAlgorithm initialised (detector={detector_method})")

    def start(self):
        self._running = True
        self._vio_status = VIOStatus.INITIALIZING
        logger.info("VIOAlgorithm started")

    def stop(self):
        self._running = False
        logger.info("VIOAlgorithm stopped")

    def update_camera_intrinsics(self, fx, fy, cx, cy, depth_scale=0.001):
        """Call once when RealSense streams are configured."""
        self._fx = fx; self._fy = fy
        self._cx = cx; self._cy = cy
        self._depth_scale = depth_scale
        self._depth.update_intrinsics(fx, fy, cx, cy, depth_scale)
        logger.info(f"Camera intrinsics updated: fx={fx:.1f} fy={fy:.1f} "
                    f"cx={cx:.1f} cy={cy:.1f} scale={depth_scale}")

    # ------------------------------------------------------------------ #
    # External sensor feeds
    # ------------------------------------------------------------------ #

    def feed_imu(self, ax: float, ay: float, az: float,
                 wx: float, wy: float, wz: float, t: float):
        """Push one IMU sample (called from the IMU callback at 200 Hz)."""
        self._imu.push(ax, ay, az, wx, wy, wz, t)

    def feed_lidar(self, range_m: float):
        """Update nadir LiDAR range (called from LidarFusion)."""
        self._lidar_range = range_m

    def feed_rtabmap(self, pos_ned: np.ndarray, rpy: np.ndarray,
                     cov6: np.ndarray, loop_closure_id: int = -1,
                     map_nodes: int = 0):
        """Update RTABMap pose (called from VoxlRTABMapClient)."""
        self._rtabmap_pose = (pos_ned, rpy, cov6)
        self._rtabmap_closure_id = loop_closure_id
        self._rtabmap_map_nodes  = map_nodes

    # ------------------------------------------------------------------ #
    # Main processing entry point
    # ------------------------------------------------------------------ #

    def process_frame(self, color_frame: np.ndarray,
                      depth_frame: Optional[np.ndarray],
                      imu_samples: Optional[List[Tuple]] = None,
                      ) -> VIOState:
        """
        Run one full VIO pipeline iteration.

        Args:
            color_frame:  uint8 BGR or grayscale (H×W or H×W×3).
            depth_frame:  uint16 depth image aligned to colour (H×W), or None.
            imu_samples:  list of (ax,ay,az,wx,wy,wz,t) tuples from this frame
                          interval.  If None, uses the buffer from feed_imu().

        Returns:
            VIOState with all fields populated.
        """
        t_start = time.time()
        self._frame_count += 1

        # ---- Grayscale conversion --------------------------------
        gray = self._to_gray(color_frame)

        # ---- Push IMU samples if provided as argument -----------
        if imu_samples:
            for s in imu_samples:
                self._imu.push(*s)

        # ---- Warm-up phase: skip first N frames -----------------
        if self._frame_count <= self.WARM_UP_FRAMES:
            # Still run tracking to initialise prev_gray/prev_pts
            self._tracker.track(gray)
            imu_result = self._imu.integrate()
            state = VIOState(
                timestamp=t_start,
                state=VIOStatus.INITIALIZING,
                confidence=0.0,
                pipeline_stage="warmup",
                imu_dt=imu_result.dt,
                imu_sample_count=imu_result.sample_count,
                processing_time_ms=(time.time() - t_start) * 1000,
            )
            with self._state_lock:
                self._last_state = state
            return state

        # ---- Feature tracking -----------------------------------
        prev_inliers, curr_inliers, num_tracked, num_inliers, mean_flow = \
            self._tracker.track(gray)

        num_detected = self._detector.detect(gray).shape[0] \
            if num_inliers < self._tracker.min_features else num_tracked

        # ---- IMU pre-integration --------------------------------
        imu_result = self._imu.integrate()

        # Feed back bias from EKF to IMU integrator
        self._imu.update_bias(self._ekf.accel_bias, self._ekf.gyro_bias)

        # ---- EKF propagation ------------------------------------
        self._ekf.propagate(imu_result)

        # ---- Depth projection -----------------------------------
        depth_range_m = self._depth.get_nearest_valid(depth_frame)
        if depth_frame is not None and num_inliers >= 4:
            pts3d, valid = self._depth.unproject(curr_inliers, depth_frame)
            if len(pts3d) >= 4:
                # Convert camera frame (x-right,y-down,z-fwd) → NED world
                # Simplified: rotate by current attitude
                roll, pitch, yaw = self._ekf.euler
                R_b2w = _euler_to_rot(roll, pitch, yaw)
                # Camera → body (approximate for forward-facing camera)
                R_cam2body = np.array([
                    [0, 0, 1],
                    [-1, 0, 0],
                    [0, -1, 0],
                ], dtype=float)
                pts3d_world = (R_b2w @ R_cam2body @ pts3d.T).T + self._ekf.position
                self._ekf.update_visual(pts3d_world)

        # ---- LiDAR altitude update ------------------------------
        if self._lidar_range > 0:
            self._ekf.update_lidar_alt(self._lidar_range)

        # ---- RTABMap pose update --------------------------------
        rtab_closure = -1
        rtab_nodes   = 0
        if self._rtabmap_pose is not None:
            pos_ned, rpy, cov6 = self._rtabmap_pose
            self._ekf.update_rtabmap(pos_ned, rpy, cov6)
            rtab_closure          = self._rtabmap_closure_id
            rtab_nodes            = self._rtabmap_map_nodes
            self._rtabmap_pose    = None   # consume once

        # ---- Optical flow velocity estimate (body frame) --------
        flow_vx_ms = flow_vy_ms = 0.0
        flow_quality = 0
        if num_inliers >= 8 and imu_result.dt > 0:
            # Estimate body-frame velocity from optical flow
            flow_q  = self._compute_flow_quality(num_inliers, mean_flow)
            if flow_q > 50:
                fv = self._flow_to_velocity(curr_inliers, prev_inliers,
                                            imu_result.dt)
                if fv is not None:
                    flow_vx_ms, flow_vy_ms = fv
                    flow_quality = flow_q
                    self._ekf.update_flow_velocity(flow_vx_ms, flow_vy_ms,
                                                   flow_quality)

        # ---- Confidence score -----------------------------------
        pos_cov     = self._ekf.pos_covariance
        pos_cov_norm = float(np.mean(pos_cov))
        confidence  = self._compute_confidence(
            num_inliers, num_tracked, imu_result.dt, pos_cov_norm
        )

        # ---- State machine --------------------------------------
        vio_status = self._update_status(confidence)

        # ---- Collect full VIOState ------------------------------
        roll, pitch, yaw = self._ekf.euler
        pos = self._ekf.position
        vel = self._ekf.velocity
        ba  = self._ekf.accel_bias
        bg  = self._ekf.gyro_bias
        att_cov = self._ekf.att_covariance

        state = VIOState(
            timestamp     = t_start,
            # IMU
            imu_dt          = imu_result.dt,
            imu_sample_count= imu_result.sample_count,
            accel_bias      = ba.tolist(),
            gyro_bias       = bg.tolist(),
            # Position NED
            north           = float(pos[0]),
            east            = float(pos[1]),
            down            = float(pos[2]),
            # Velocity NED
            vn              = float(vel[0]),
            ve              = float(vel[1]),
            vd              = float(vel[2]),
            # Acceleration (body, bias-corrected)
            ax              = float(imu_result.last_accel[0]) if len(imu_result.last_accel) else 0.0,
            ay              = float(imu_result.last_accel[1]) if len(imu_result.last_accel) else 0.0,
            az              = float(imu_result.last_accel[2]) if len(imu_result.last_accel) else 0.0,
            # Attitude
            roll            = float(roll),
            pitch           = float(pitch),
            yaw             = float(yaw),
            # Body rates
            roll_rate       = float(imu_result.last_gyro[0]) if len(imu_result.last_gyro) else 0.0,
            pitch_rate      = float(imu_result.last_gyro[1]) if len(imu_result.last_gyro) else 0.0,
            yaw_rate        = float(imu_result.last_gyro[2]) if len(imu_result.last_gyro) else 0.0,
            # Features
            num_features_detected = num_detected,
            num_features_tracked  = num_tracked,
            num_inliers           = num_inliers,
            tracking_ratio        = num_inliers / max(num_tracked, 1),
            mean_flow_magnitude   = mean_flow,
            frames_since_detect   = self._tracker.frames_since_detect,
            # Range
            depth_range_m   = depth_range_m,
            lidar_alt_m     = self._lidar_range,
            # Health
            state           = vio_status,
            confidence      = confidence,
            position_covariance = pos_cov.tolist(),
            attitude_covariance = att_cov.tolist(),
            # Optical flow
            flow_vx_ms      = flow_vx_ms,
            flow_vy_ms      = flow_vy_ms,
            flow_quality    = flow_quality,
            # RTABMap
            loop_closure_id = rtab_closure,
            map_nodes       = rtab_nodes,
            # Timing
            processing_time_ms = (time.time() - t_start) * 1000,
            pipeline_stage     = "output",
        )

        with self._state_lock:
            self._last_state = state

        return state

    def get_last_state(self) -> VIOState:
        """Thread-safe access to the most recent VIOState."""
        with self._state_lock:
            return self._last_state

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _to_gray(frame: np.ndarray) -> np.ndarray:
        try:
            import cv2
            if frame.ndim == 3:
                return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except ImportError:
            if frame.ndim == 3:
                return (0.299 * frame[:, :, 2] +
                        0.587 * frame[:, :, 1] +
                        0.114 * frame[:, :, 0]).astype(np.uint8)
        return frame

    @staticmethod
    def _compute_confidence(num_inliers: int, num_tracked: int,
                             imu_dt: float, pos_cov_norm: float) -> float:
        """
        Composite confidence score 0–100.

        Weights:
          30% — inlier count (saturates at 200)
          30% — geometric tracking ratio
          20% — EKF position covariance (inverse)
          10% — IMU timing health (dt < 40 ms)
          10% — reserved / LiDAR (handled externally via lidar_alt_m)
        """
        c_features = min(num_inliers / 200.0, 1.0) * 100.0
        c_ratio    = (num_inliers / max(num_tracked, 1)) * 100.0
        c_cov      = (1.0 - min(pos_cov_norm, 1.0)) * 100.0
        c_imu      = 100.0 if imu_dt < 0.04 else 0.0

        return (0.30 * c_features +
                0.30 * c_ratio +
                0.20 * c_cov +
                0.10 * c_imu)

    def _update_status(self, confidence: float) -> VIOStatus:
        """Hysteresis state machine: TRACKING / DEGRADED / LOST / RELOCALIZING."""
        if self._frame_count <= self.WARM_UP_FRAMES:
            return VIOStatus.INITIALIZING

        if confidence >= 70.0:
            self._low_conf_start = 0.0
            if self._vio_status == VIOStatus.RELOCALIZING:
                logger.info("VIO relocalized — back to TRACKING")
            self._vio_status = VIOStatus.TRACKING

        elif confidence >= 40.0:
            self._low_conf_start = 0.0
            self._vio_status = VIOStatus.DEGRADED

        else:
            if self._low_conf_start == 0.0:
                self._low_conf_start = time.time()
                logger.warning(f"VIO low confidence: {confidence:.1f}%")
            elif time.time() - self._low_conf_start > 2.0:
                if self._vio_status != VIOStatus.LOST:
                    logger.error("VIO LOST — triggering relocalization")
                self._vio_status = VIOStatus.LOST
                # Attempt re-initialization after 2 more seconds
                if time.time() - self._low_conf_start > 4.0:
                    self._vio_status = VIOStatus.RELOCALIZING
                    self._tracker.reset()
                    self._ekf.reset()
                    self._low_conf_start = time.time()
                    logger.info("VIO reset for relocalization")

        return self._vio_status

    @staticmethod
    def _compute_flow_quality(num_inliers: int, mean_flow: float) -> int:
        """Convert tracking metrics to 0–255 flow quality byte."""
        q_features = min(num_inliers / 200.0, 1.0)
        # Penalise very large flows (likely frame drop / fast rotation)
        q_flow = 1.0 - min(mean_flow / 50.0, 1.0)
        quality = int((0.6 * q_features + 0.4 * q_flow) * 255)
        return max(0, min(255, quality))

    def _flow_to_velocity(self, curr_pts: np.ndarray,
                          prev_pts: np.ndarray,
                          dt: float) -> Optional[Tuple[float, float]]:
        """
        Estimate body-frame (x-fwd, y-right) velocity from optical flow.

        Uses the median flow displacement scaled by altitude estimate.
        """
        if dt <= 0 or len(curr_pts) < 4 or len(prev_pts) < 4:
            return None

        flow = (curr_pts.reshape(-1, 2) - prev_pts.reshape(-1, 2))
        median_dx = float(np.median(flow[:, 0]))   # pixels, +x = right in image
        median_dy = float(np.median(flow[:, 1]))   # pixels, +y = down in image

        # Use current altitude for scale; fall back to LiDAR
        alt = max(-self._ekf.position[2], self._lidar_range
                  if self._lidar_range > 0 else 1.0)
        if alt < 0.2:
            return None

        # pixel disparity → rad/s → m/s via altitude
        vx_body = -(median_dy / self._fy) * alt / dt  # fwd = -image-y
        vy_body =  (median_dx / self._fx) * alt / dt  # right = +image-x
        return vx_body, vy_body
