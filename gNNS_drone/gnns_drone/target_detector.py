"""
gNNS Drone — Landing Target Detector
======================================
Detects landing targets using the depth camera (D435i).

Supports multiple detection methods:
  1. ArUco Marker detection (recommended — precise, works from 5m+)
  2. Color-based detection (colored landing pad)
  3. Shape-based detection (circle/H-pad detection)

The detector:
  - Finds the target in the camera image
  - Computes its position relative to the drone (x,y offset + distance)
  - Feeds offsets to the precision landing controller
  - Works with depth data for accurate 3D positioning
"""

import time
import math
import threading
import logging
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple
from enum import IntEnum

logger = logging.getLogger("gnns.target")


class TargetType(IntEnum):
    ARUCO = 0
    COLOR_PAD = 1
    CIRCLE = 2
    H_PAD = 3


@dataclass
class TargetDetection:
    """A detected landing target."""
    detected: bool = False
    timestamp: float = 0.0

    # Target position relative to drone (meters, body frame)
    offset_forward: float = 0.0   # Forward from drone (positive = in front)
    offset_right: float = 0.0     # Right from drone (positive = right)
    offset_down: float = 0.0      # Below drone (positive = below, this is altitude AGL)

    # Target in image coordinates
    pixel_x: int = 0              # Pixel X in image
    pixel_y: int = 0              # Pixel Y in image

    # Confidence and size
    confidence: float = 0.0       # 0-1 detection confidence
    target_size_pixels: float = 0 # Size in pixels (larger = closer)
    distance_m: float = 0.0       # Distance to target in meters

    # For ArUco: which marker ID
    marker_id: int = -1

    @property
    def horizontal_offset(self) -> float:
        """Horizontal distance from directly above target."""
        return math.sqrt(self.offset_forward**2 + self.offset_right**2)

    @property
    def is_centered(self) -> bool:
        """True if drone is roughly centered over target (within 30cm)."""
        return self.horizontal_offset < 0.3

    @property
    def is_close(self) -> bool:
        """True if drone is close enough for final descent (within 1m horiz)."""
        return self.horizontal_offset < 1.0


class TargetDetector:
    """
    Detects landing targets using depth camera.
    
    Usage:
        detector = TargetDetector(method="aruco")
        detector.start()
        
        detection = detector.get_detection()
        if detection.detected:
            print(f"Target found! Offset: fwd={detection.offset_forward:.2f}m, "
                  f"right={detection.offset_right:.2f}m, "
                  f"down={detection.offset_down:.2f}m")
    """

    def __init__(self, method: str = "aruco", target_marker_id: int = 42,
                 color_lower=(0, 100, 100), color_upper=(10, 255, 255)):
        """
        Args:
            method: "aruco", "color", or "circle"
            target_marker_id: ArUco marker ID to look for
            color_lower/upper: HSV range for color-based detection
        """
        self.method = method
        self.target_marker_id = target_marker_id
        self.color_lower = np.array(color_lower)
        self.color_upper = np.array(color_upper)

        self._detection = TargetDetection()
        self._det_lock = threading.Lock()
        self._running = False
        self._thread = None

        # Camera intrinsics (updated when camera starts)
        self._fx = 0  # Focal length X
        self._fy = 0  # Focal length Y
        self._cx = 0  # Principal point X
        self._cy = 0  # Principal point Y

    def get_detection(self) -> TargetDetection:
        """Get latest target detection (thread-safe)."""
        with self._det_lock:
            return TargetDetection(
                detected=self._detection.detected,
                timestamp=self._detection.timestamp,
                offset_forward=self._detection.offset_forward,
                offset_right=self._detection.offset_right,
                offset_down=self._detection.offset_down,
                pixel_x=self._detection.pixel_x,
                pixel_y=self._detection.pixel_y,
                confidence=self._detection.confidence,
                target_size_pixels=self._detection.target_size_pixels,
                distance_m=self._detection.distance_m,
                marker_id=self._detection.marker_id,
            )

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._detection_loop,
                                         daemon=True, name="target-det")
        self._thread.start()
        logger.info(f"Target detector started (method={self.method})")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def _detection_loop(self):
        """Main detection loop using D435i depth camera."""
        try:
            import pyrealsense2 as rs
            import cv2
        except ImportError:
            logger.error("Need pyrealsense2 and opencv-python!")
            return

        # Setup D435i
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        align = rs.align(rs.stream.color)

        try:
            profile = pipe.start(cfg)

            # Get camera intrinsics
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self._fx = intr.fx
            self._fy = intr.fy
            self._cx = intr.ppx
            self._cy = intr.ppy
            logger.info(f"D435i started: fx={self._fx:.1f} fy={self._fy:.1f}")

        except Exception as e:
            logger.error(f"D435i init failed: {e}")
            self._run_simulated()
            return

        # ArUco setup
        if self.method == "aruco":
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
            aruco_params = cv2.aruco.DetectorParameters()
            aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        while self._running:
            try:
                frames = pipe.wait_for_frames(timeout_ms=500)
                aligned = align.process(frames)

                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color_img = np.asanyarray(color_frame.get_data())
                depth_img = np.asanyarray(depth_frame.get_data())

                detection = TargetDetection(timestamp=time.time())

                if self.method == "aruco":
                    detection = self._detect_aruco(
                        color_img, depth_frame, aruco_detector
                    )
                elif self.method == "color":
                    detection = self._detect_color(color_img, depth_frame)
                elif self.method == "circle":
                    detection = self._detect_circle(color_img, depth_frame)

                with self._det_lock:
                    self._detection = detection

            except Exception as e:
                logger.error(f"Detection error: {e}")
                time.sleep(0.01)

        pipe.stop()

    # ==============================================================
    # ARUCO MARKER DETECTION
    # ==============================================================

    def _detect_aruco(self, color_img, depth_frame, detector) -> TargetDetection:
        """
        Detect ArUco markers in the image.
        
        ArUco markers are small checkerboard-like patterns that are
        very reliable to detect. Print one and place it at each landing spot.
        
        Use DICT_4X4_100 markers — they work from far away.
        Print at ~20cm size for detection from 5m altitude.
        """
        import cv2
        
        det = TargetDetection(timestamp=time.time())
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            det.detected = False
            return det

        # Find our target marker
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == self.target_marker_id or self.target_marker_id == -1:
                # Get center of marker
                corner_points = corners[i][0]
                cx = int(np.mean(corner_points[:, 0]))
                cy = int(np.mean(corner_points[:, 1]))

                # Get depth at center
                depth_m = depth_frame.get_distance(cx, cy)
                if depth_m <= 0 or depth_m > 10:
                    # Try average of corners
                    depths = []
                    for pt in corner_points:
                        d = depth_frame.get_distance(int(pt[0]), int(pt[1]))
                        if 0 < d < 10:
                            depths.append(d)
                    depth_m = np.mean(depths) if depths else 0

                if depth_m <= 0:
                    continue

                # Convert pixel + depth to 3D position relative to camera
                # Camera points DOWN on the drone, so:
                #   Camera X (right) → Drone right
                #   Camera Y (down in image) → Drone forward
                #   Camera Z (depth) → Drone down (distance below)
                offset_right = (cx - self._cx) * depth_m / self._fx
                offset_forward = (cy - self._cy) * depth_m / self._fy
                offset_down = depth_m

                # Marker size in pixels for distance estimation
                size = np.max(np.linalg.norm(
                    corner_points[0] - corner_points[2]
                ))

                det.detected = True
                det.offset_forward = offset_forward
                det.offset_right = offset_right
                det.offset_down = offset_down
                det.pixel_x = cx
                det.pixel_y = cy
                det.confidence = 0.95
                det.target_size_pixels = size
                det.distance_m = depth_m
                det.marker_id = int(marker_id)

                logger.debug(
                    f"ArUco #{marker_id}: fwd={offset_forward:.2f}m "
                    f"right={offset_right:.2f}m down={offset_down:.2f}m"
                )
                break

        return det

    # ==============================================================
    # COLOR-BASED DETECTION (colored pad)
    # ==============================================================

    def _detect_color(self, color_img, depth_frame) -> TargetDetection:
        """Detect a colored landing pad (e.g., orange/red pad on the ground)."""
        import cv2

        det = TargetDetection(timestamp=time.time())
        hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        # Morphology to clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            det.detected = False
            return det

        # Find largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 500:  # Too small — noise
            det.detected = False
            return det

        M = cv2.moments(largest)
        if M["m00"] == 0:
            det.detected = False
            return det

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        depth_m = depth_frame.get_distance(cx, cy)
        if depth_m <= 0 or depth_m > 10:
            det.detected = False
            return det

        det.detected = True
        det.offset_right = (cx - self._cx) * depth_m / self._fx
        det.offset_forward = (cy - self._cy) * depth_m / self._fy
        det.offset_down = depth_m
        det.pixel_x = cx
        det.pixel_y = cy
        det.confidence = min(1.0, area / 5000)
        det.target_size_pixels = math.sqrt(area)
        det.distance_m = depth_m

        return det

    # ==============================================================
    # CIRCLE DETECTION (for circular landing pads / helipads)
    # ==============================================================

    def _detect_circle(self, color_img, depth_frame) -> TargetDetection:
        """Detect circular landing target using Hough circles."""
        import cv2

        det = TargetDetection(timestamp=time.time())
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=100, param2=50, minRadius=20, maxRadius=200
        )

        if circles is None:
            det.detected = False
            return det

        # Pick the most centered / largest circle
        best = None
        best_score = -1
        img_cx, img_cy = color_img.shape[1]//2, color_img.shape[0]//2

        for c in circles[0]:
            x, y, r = int(c[0]), int(c[1]), int(c[2])
            # Score: bigger and more centered = better
            dist_center = math.sqrt((x - img_cx)**2 + (y - img_cy)**2)
            score = r - dist_center * 0.5
            if score > best_score:
                best_score = score
                best = (x, y, r)

        if best is None:
            det.detected = False
            return det

        cx, cy, radius = best
        depth_m = depth_frame.get_distance(cx, cy)
        if depth_m <= 0 or depth_m > 10:
            det.detected = False
            return det

        det.detected = True
        det.offset_right = (cx - self._cx) * depth_m / self._fx
        det.offset_forward = (cy - self._cy) * depth_m / self._fy
        det.offset_down = depth_m
        det.pixel_x = cx
        det.pixel_y = cy
        det.confidence = min(1.0, radius / 100)
        det.target_size_pixels = radius * 2
        det.distance_m = depth_m

        return det

    def _run_simulated(self):
        """Simulated detection for testing."""
        logger.info("Running simulated target detection")
        while self._running:
            with self._det_lock:
                self._detection = TargetDetection(
                    detected=True, timestamp=time.time(),
                    offset_forward=0.1, offset_right=-0.05, offset_down=2.0,
                    confidence=0.9, distance_m=2.0
                )
            time.sleep(1.0 / 15)
