"""
gNNS Drone — Coordinate Utilities
==================================
GPS to Local NED coordinate conversion functions.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

EARTH_RADIUS_M = 6371000.0


@dataclass
class GPSCoord:
    lat: float
    lon: float
    alt: float = 0.0

    def __repr__(self):
        return f"GPS({self.lat:.7f}, {self.lon:.7f}, alt={self.alt:.1f}m)"


@dataclass
class NEDCoord:
    north: float
    east: float
    down: float

    @property
    def altitude(self) -> float:
        return -self.down

    @property
    def horizontal_distance(self) -> float:
        return math.sqrt(self.north ** 2 + self.east ** 2)

    def distance_to(self, other: 'NEDCoord') -> float:
        return math.sqrt((self.north - other.north)**2 + (self.east - other.east)**2)

    def bearing_to(self, other: 'NEDCoord') -> float:
        dn = other.north - self.north
        de = other.east - self.east
        return math.degrees(math.atan2(de, dn)) % 360

    def __repr__(self):
        return f"NED(N={self.north:.2f}m, E={self.east:.2f}m, D={self.down:.2f}m)"


def gps_to_ned(origin: GPSCoord, target: GPSCoord) -> NEDCoord:
    """Convert GPS coordinate to local NED offset from origin (equirectangular)."""
    d_lat = math.radians(target.lat - origin.lat)
    d_lon = math.radians(target.lon - origin.lon)
    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(math.radians(origin.lat))
    down = -(target.alt - origin.alt)
    return NEDCoord(north=north, east=east, down=down)


def ned_to_gps(origin: GPSCoord, ned: NEDCoord) -> GPSCoord:
    """Convert local NED position back to GPS (for logging/display)."""
    d_lat = ned.north / EARTH_RADIUS_M
    d_lon = ned.east / (EARTH_RADIUS_M * math.cos(math.radians(origin.lat)))
    return GPSCoord(lat=origin.lat + math.degrees(d_lat),
                    lon=origin.lon + math.degrees(d_lon),
                    alt=origin.alt - ned.down)


def haversine_distance(p1: GPSCoord, p2: GPSCoord) -> float:
    """Great-circle distance between two GPS points (meters)."""
    lat1, lon1 = math.radians(p1.lat), math.radians(p1.lon)
    lat2, lon2 = math.radians(p2.lat), math.radians(p2.lon)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_between(p1: GPSCoord, p2: GPSCoord) -> float:
    """Initial bearing from p1 to p2 in degrees (0=North, 90=East)."""
    lat1, lat2 = math.radians(p1.lat), math.radians(p2.lat)
    dlon = math.radians(p2.lon - p1.lon)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360


class WaypointManager:
    """Manages GPS waypoints and their NED equivalents."""

    def __init__(self):
        self.home: GPSCoord | None = None
        self.waypoints_gps: List[GPSCoord] = []
        self.waypoints_ned: List[NEDCoord] = []

    def set_home(self, home: GPSCoord):
        self.home = home
        self.waypoints_ned = [gps_to_ned(self.home, wp) for wp in self.waypoints_gps]

    def add_waypoint(self, gps: GPSCoord) -> NEDCoord:
        if self.home is None:
            raise ValueError("Must call set_home() before adding waypoints!")
        self.waypoints_gps.append(gps)
        ned = gps_to_ned(self.home, gps)
        self.waypoints_ned.append(ned)
        return ned

    def get_waypoint_ned(self, index: int) -> NEDCoord:
        return self.waypoints_ned[index]

    def get_waypoint_gps(self, index: int) -> GPSCoord:
        return self.waypoints_gps[index]

    def get_all(self) -> List[Tuple[GPSCoord, NEDCoord]]:
        return list(zip(self.waypoints_gps, self.waypoints_ned))

    @property
    def count(self) -> int:
        return len(self.waypoints_gps)

    def total_mission_distance(self) -> float:
        if not self.waypoints_ned:
            return 0.0
        total = 0.0
        home_ned = NEDCoord(0, 0, 0)
        total += home_ned.distance_to(self.waypoints_ned[0])
        for i in range(len(self.waypoints_ned) - 1):
            total += self.waypoints_ned[i].distance_to(self.waypoints_ned[i + 1])
        total += self.waypoints_ned[-1].distance_to(home_ned)
        return total

    def print_mission_summary(self):
        if self.home is None:
            print("No home position set!")
            return
        print(f"\n{'='*60}")
        print("  MISSION SUMMARY")
        print(f"{'='*60}")
        print(f"  Home: {self.home}")
        print(f"  Waypoints: {self.count}")
        print(f"  Total distance: {self.total_mission_distance():.1f}m")
        print(f"{'-'*60}")
        home_ned = NEDCoord(0, 0, 0)
        prev = home_ned
        for i, (gps, ned) in enumerate(self.get_all()):
            dist = prev.distance_to(ned)
            bearing = prev.bearing_to(ned)
            print(f"  WP{i+1}: {gps}")
            print(f"        -> NED: {ned}")
            print(f"        -> Dist: {dist:.1f}m, Bearing: {bearing:.1f} deg")
            prev = ned
        dist = prev.distance_to(home_ned)
        print(f"  RTH:  Distance: {dist:.1f}m")
        print(f"{'='*60}")


if __name__ == "__main__":
    print("Coordinate Utilities Self-Test")
    origin = GPSCoord(28.6139, 77.2090)
    target = GPSCoord(28.6150, 77.2100)
    ned = gps_to_ned(origin, target)
    print(f"Origin: {origin}")
    print(f"Target: {target}")
    print(f"NED:    {ned}")
    print(f"Dist:   {ned.horizontal_distance:.2f}m")
    back = ned_to_gps(origin, ned)
    print(f"Roundtrip error: {haversine_distance(target, back):.6f}m")

    wm = WaypointManager()
    wm.set_home(GPSCoord(28.6139, 77.2090))
    for lat, lon in [(28.615, 77.210), (28.616, 77.208),
                     (28.6145, 77.207), (28.613, 77.2095), (28.6155, 77.2105)]:
        wm.add_waypoint(GPSCoord(lat, lon))
    wm.print_mission_summary()
    print("\nAll tests passed!")
