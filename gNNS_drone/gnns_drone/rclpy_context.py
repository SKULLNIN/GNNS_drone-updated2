"""
Ref-counted rclpy init/shutdown.

Multiple modules (RTAB-Map odom, ORB-SLAM3 odom, LiDAR fusion) may share one
process; only the last release() should call rclpy.shutdown().
"""

from __future__ import annotations

import threading

_lock = threading.Lock()
_refcount = 0


def rclpy_acquire() -> None:
    """Increment holders; call rclpy.init() if this is the first acquire."""
    global _refcount
    import rclpy

    with _lock:
        if not rclpy.ok():
            rclpy.init()
        _refcount += 1


def rclpy_release() -> None:
    """Decrement holders; shutdown when the last reference is dropped."""
    global _refcount
    import rclpy

    with _lock:
        if _refcount <= 0:
            return
        _refcount -= 1
        if _refcount == 0 and rclpy.ok():
            rclpy.shutdown()
