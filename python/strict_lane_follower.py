"""
strict_lane_follower.oy - Canny + HoughLinesP lane detection with:
  - Wider ROI trapezoid (better on curves)
    - LEFT + RIGHT boundary detection (center accurately)
    - Dropout tolerance (no instant stop on brief loss)

"""

import argparse
import math
import time
import socket
import struct
from typing import Optional, Tuple, Dict, List
from dataclasses import dataclass, field

import cv2
import numpy as np

from pal.products.qcar import QCar
from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar, QCarLidar, QCarRealSense
import threading
from collections import deque
from typing import Deque
import queue

# ===================================================================
#  LED Direct Control
# ===================================================================

class _QLabsLEDDirect:
    """Send LED-strip colour commands directly to the QLabs TCP server."""
    _QCAR2_CLASS_ID     = 161
    _FCN_LED_UNIFORM     = 30
    _BASE_CONTAINER_SIZE = 13

    def __init__(self, host: str = "localhost", port: int = 18000,
                 actor: int = 0, timeout: float = 3.0):
        self._actor = int(actor)
        self._sock: socket.socket | None = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            s.connect((host, port))
            s.settimeout(0.5)
            self._sock = s
        except Exception:
            self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def set_color(self, r: float, g: float, b: float) -> bool:
        if self._sock is None:
            return False
        payload = struct.pack(">fff", float(r), float(g), float(b))
        csz = self._BASE_CONTAINER_SIZE + len(payload)
        pkt = (
            struct.pack("<i", 1 + csz)
            + struct.pack(">BiiiB", 123, csz,
                          self._QCAR2_CLASS_ID, self._actor,
                          self._FCN_LED_UNIFORM)
            + payload
        )
        try:
            self._sock.sendall(pkt)
            try:
                self._sock.recv(512)
            except Exception:
                pass
            return True
        except Exception:
            return False

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

# ===================================================================
#  QLabs World-Transform Client (custom TCP)
# 
#  Packet wire format:
#    TX: <i(1+containerSize) >B(123) >i(containerSize) >i(classID)
#        >i(actorNumber) >B(actorFunction) [payload…]
#    RX: <I(packetSize) >B(123) then containers:
#        >I(containerSize) >I(classID) >I(actorNumber) B(actorFunction)
#        [payload…]
# ===================================================================

class _QLabsWorldTransform:
    """Lightweight raw-socket client that queries QLabs for world transform.
    """

    _QCAR2_CLASS_ID              = 161
    _FCN_REQUEST_WORLD_TRANSFORM  = 3   # QLabsActor.FCN_REQUEST_WORLD_TRANSFORM
    _FCN_RESPONSE_WORLD_TRANSFORM = 4   # QLabsActor.FCN_RESPONSE_WORLD_TRANSFORM
    _BASE_CONTAINER_SIZE          = 13  # 4+4+4+1

    def __init__(self, host: str = "localhost", port: int = 18000,
                 actor: int = 0, timeout: float = 5.0):
        self._actor = int(actor)
        self._host  = host
        self._port  = port
        self._timeout = timeout
        self._sock: socket.socket | None = None
        self._connect()

    # -- connection management --

    def _connect(self):
        """Open (or re-open) a TCP connection to QLabs."""
        self._close_sock()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self._timeout)
            s.connect((self._host, self._port))
            s.settimeout(self._timeout)
            self._sock = s
        except Exception:
            self._sock = None

    def _close_sock(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    # -- low-level send / recv helpers --

    def _send_container(self, class_id: int, actor: int,
                        func: int, payload: bytes = b"") -> bool:
        """Build and send one container packet."""
        # (existing implementation continues)


# ---------- LIDAR helpers and simple mapper ----------
def polar_to_xy(angles: np.ndarray, distances: np.ndarray):
    """Convert repository lidar angle convention -> car-frame XY.
    angles: radians array as provided by QCarLidar/Lidar
    returns Nx2 float32 array (x forward, y left)
    """
    if angles is None or distances is None:
        return np.empty((0, 2), dtype=np.float32)
    a = (-angles + np.pi).astype(np.float32)
    a = (a + np.pi) % (2.0 * np.pi) - np.pi
    d = distances.astype(np.float32, copy=False)
    # include measurements up to and including the sensor's max range (8.0m)
    valid = np.isfinite(d) & (d > 0.01) & (d <= 8.0)
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float32)
    a = a[valid]
    d = d[valid]
    x = d * np.cos(a)
    y = d * np.sin(a)
    pts = np.column_stack([x, y]).astype(np.float32)
    return pts


class LidarMapper:
    """World-frame occupancy mapper and birdseye renderer.

    - Builds a persistent occupancy grid in world coordinates.
    - Centers the map at the first observed vehicle pose (configurable).
    - `accumulate(points_xy, pose, t)` expects `points_xy` in car-frame.
      If `pose` is provided (x,y,yaw), points are transformed to world-frame
      and integrated into the occupancy grid. The grid stores hit counts.
    - `render()` returns an RGB image with occupancy visualised and a car pose marker.
    """
    def __init__(self, map_size_m=(20.0, 20.0), px_per_m: float = 40.0, origin_world=None, map_rotation_deg: int = 0):
        # map_size_m: (width_m, height_m)
        self.map_size_m = (float(map_size_m[0]), float(map_size_m[1]))
        self.px_per_m = float(px_per_m)
        self.map_w_px = max(16, int(self.map_size_m[0] * self.px_per_m))
        self.map_h_px = max(16, int(self.map_size_m[1] * self.px_per_m))
        self._occupancy = np.zeros((self.map_h_px, self.map_w_px), dtype=np.float32)
        self._lock = threading.Lock()
        # origin_world is the (min_x, min_y) world coord corresponding to map pixel (0,0)
        # if not provided, will be set on first pose to center the map near the vehicle
        self.origin_world = None if origin_world is None else (float(origin_world[0]), float(origin_world[1]))
        self._first_pose_received = False
        self.last_pose = None
        self._last_raw = None
        # Rotation to apply to final birdseye image (CCW). Use to align world-north to image-up.
        if map_rotation_deg not in (0, 90, 180, 270):
            raise ValueError("map_rotation_deg must be one of (0,90,180,270)")
        self.map_rotation_deg = int(map_rotation_deg)
        self._rot_k = (self.map_rotation_deg // 90) % 4

    def _ensure_origin(self, pose_x: float, pose_y: float):
        if self.origin_world is None:
            # center map around current pose
            ox = float(pose_x) - 0.5 * self.map_size_m[0]
            oy = float(pose_y) - 0.5 * self.map_size_m[1]
            self.origin_world = (ox, oy)

    def accumulate(self, points_xy: np.ndarray, pose: Tuple[float, float, float] | None, tstamp: float):
        """Accumulate a scan in the map. points_xy is in car-frame.
        pose: (x,y,yaw) world pose of vehicle. If None, skip accumulation unless last pose exists.
        """
        if points_xy is None or points_xy.size == 0:
            return
        # always keep most recent raw (car-frame) scan for fallback rendering
        try:
            self._last_raw = points_xy.copy()
        except Exception:
            self._last_raw = points_xy

        if pose is None and self.last_pose is None:
            # no world pose yet; keep raw scan for display but cannot integrate into world occupancy
            return

        with self._lock:
            if pose is not None:
                self.last_pose = pose
            # ensure we have an origin
            if self.last_pose is not None:
                self._ensure_origin(self.last_pose[0], self.last_pose[1])
            if self.origin_world is None:
                return
            # transform points into world-frame using last_pose if needed
            if pose is not None:
                th = float(pose[2])
                R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]], dtype=np.float32)
                pts_w = (R @ points_xy.T).T + np.array([pose[0], pose[1]], dtype=np.float32)
            else:
                # use last_pose to place car-frame points into world
                th = float(self.last_pose[2])
                R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]], dtype=np.float32)
                pts_w = (R @ points_xy.T).T + np.array([self.last_pose[0], self.last_pose[1]], dtype=np.float32)

            # convert world points to pixel indices
            ox, oy = self.origin_world
            rel_x = pts_w[:, 0] - ox
            rel_y = pts_w[:, 1] - oy
            px = np.floor(rel_x * self.px_per_m).astype(np.int32)
            py = np.floor(rel_y * self.px_per_m).astype(np.int32)
            # image y is top-down; our rel_y grows upward -> convert to image coords
            py_img = self.map_h_px - 1 - py

            # integrate into occupancy grid
            valid_mask = (px >= 0) & (px < self.map_w_px) & (py_img >= 0) & (py_img < self.map_h_px)
            if not np.any(valid_mask):
                return
            pxv = px[valid_mask]
            pyv = py_img[valid_mask]
            for xi, yi in zip(pxv, pyv):
                # increment occupancy; cap to avoid overflow
                self._occupancy[yi, xi] = min(255.0, self._occupancy[yi, xi] + 1.0)

    def store_raw_scan(self, angles: np.ndarray, distances: np.ndarray):
        """Store the most recent raw scan (car-frame) for fallback rendering.
        Accepts raw angle/dist arrays as provided by the LIDAR object.
        """
        try:
            pts = polar_to_xy(angles, distances)
        except Exception:
            pts = np.empty((0, 2), dtype=np.float32)
        with self._lock:
            try:
                self._last_raw = pts.copy()
            except Exception:
                self._last_raw = pts

    def render(self, show_car: bool = True, heading_override: float | None = None):
        with self._lock:
            occ = self._occupancy.copy()
            origin = self.origin_world
            pose = self.last_pose
            last_raw = None if self._last_raw is None else self._last_raw.copy()

        # If we have a world-origin and a pose, render the occupancy map
        if origin is not None and pose is not None:
            if occ.max() > 0:
                norm = np.clip((occ / occ.max()) * 255.0, 0, 255).astype(np.uint8)
            else:
                norm = np.zeros_like(occ, dtype=np.uint8)
            img = cv2.applyColorMap(norm, cv2.COLORMAP_HOT)
            if show_car:
                ox, oy = origin
                cx = int((pose[0] - ox) * self.px_per_m)
                cy = int((pose[1] - oy) * self.px_per_m)
                cy_img = self.map_h_px - 1 - cy
                if 0 <= cx < self.map_w_px and 0 <= cy_img < self.map_h_px:
                    th = float(pose[2])
                    # compute forward vector using same rotation convention as accumulation
                    c = math.cos(th); s = math.sin(th)
                    Rfwd_x = c * 1.0 + -s * 0.0
                    Rfwd_y = s * 1.0 +  c * 0.0
                    hx = int(round(cx + Rfwd_x * self.px_per_m * 0.6))
                    hy = int(round(cy_img - Rfwd_y * self.px_per_m * 0.6))
                    cv2.arrowedLine(img, (cx, cy_img), (hx, hy), (0, 255, 0), 2, tipLength=0.3)
                    cv2.circle(img, (cx, cy_img), max(2, int(self.px_per_m * 0.08)), (0, 200, 0), -1)
            # rotate final occupancy image to align north->up if requested
            if self._rot_k != 0:
                img = np.rot90(img, k=self._rot_k)
            return img

        # Fallback: render the most recent raw scan centered in the birdseye
        be_h, be_w = self.map_h_px, self.map_w_px
        img2 = np.zeros((be_h, be_w, 3), dtype=np.uint8)
        if last_raw is None or last_raw.size == 0:
            # if no raw scan, still render arrow if heading_override provided
            if heading_override is None:
                return img2
        cx = be_w // 2
        cy = be_h // 2
        for px_m, py_m in last_raw:
            px = int(round(cx + float(px_m) * self.px_per_m))
            py = int(round(cy - float(py_m) * self.px_per_m))
            if 0 <= px < be_w and 0 <= py < be_h:
                y0 = max(0, py - 1); y1 = min(be_h, py + 2)
                x0 = max(0, px - 1); x1 = min(be_w, px + 2)
                img2[y0:y1, x0:x1] = (255, 255, 255)
        # draw arrow using heading_override when world-pose is not available
        hy_top = cy - int(self.px_per_m * 0.6)
        if show_car:
            if heading_override is None:
                # point upwards by default
                cv2.arrowedLine(img2, (cx, cy), (cx, max(0, hy_top)), (0, 255, 0), 2, tipLength=0.3)
            else:
                th = float(heading_override)
                c = math.cos(th); s = math.sin(th)
                # forward vector in world coords (1,0) rotated by th -> image coords
                fx = c; fy = s
                hx = int(round(cx + fx * self.px_per_m * 0.6))
                hy = int(round(cy - fy * self.px_per_m * 0.6))
                cv2.arrowedLine(img2, (cx, cy), (hx, hy), (0, 255, 0), 2, tipLength=0.3)
            cv2.circle(img2, (cx, cy), max(2, int(self.px_per_m * 0.08)), (0, 200, 0), -1)

        # rotate final image to align north->up if requested
        if self._rot_k != 0:
            img2 = np.rot90(img2, k=self._rot_k)
        return img2

    def _recv_all(self, n: int) -> bytes | None:
        """Receive exactly *n* bytes from the socket."""
        buf = bytearray()
        while len(buf) < n:
            try:
                chunk = self._sock.recv(n - len(buf))
            except Exception:
                return None
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _wait_for_container(self, class_id: int, actor: int,
                            func: int) -> bytes | None:
        """Block until we receive a container matching (classID, actor, func).

        Returns the payload bytes, or None on failure / timeout.
        """
        if self._sock is None:
            return None
        deadline = time.time() + self._timeout
        recv_buf = bytearray()
        while time.time() < deadline:
            # Read from socket
            try:
                self._sock.settimeout(max(0.05, deadline - time.time()))
                chunk = self._sock.recv(4096)
                if chunk:
                    recv_buf.extend(chunk)
            except socket.timeout:
                pass
            except Exception:
                return None

            # Try to parse one or more packets out of recv_buf
            while len(recv_buf) >= 5:
                # Packet length (little-endian uint32)
                pkt_len_raw = struct.unpack("<I", recv_buf[0:4])[0]
                total_pkt = 4 + pkt_len_raw
                if len(recv_buf) < total_pkt:
                    break  # need more bytes

                # Validate magic byte
                if recv_buf[4] != 123:
                    recv_buf = recv_buf[1:]  # skip bad byte
                    continue

                # Walk containers inside this packet
                idx = 5  # skip 4-byte length + 1-byte magic
                found_payload = None
                while idx + self._BASE_CONTAINER_SIZE <= total_pkt:
                    c_size = struct.unpack(">I", recv_buf[idx:idx+4])[0]
                    c_cls  = struct.unpack(">I", recv_buf[idx+4:idx+8])[0]
                    c_act  = struct.unpack(">I", recv_buf[idx+8:idx+12])[0]
                    c_func = recv_buf[idx+12]
                    payload_start = idx + self._BASE_CONTAINER_SIZE
                    payload_end   = idx + c_size
                    c_payload = bytes(recv_buf[payload_start:payload_end])

                    if c_cls == class_id and c_act == actor and c_func == func:
                        found_payload = c_payload

                    idx += c_size
                    if idx > total_pkt:
                        break

                recv_buf = recv_buf[total_pkt:]

                if found_payload is not None:
                    return found_payload

        return None

    # -- public API --

    def get_world_transform(self):
        """Query QLabs for the actor's world transform.

        Returns (ok, [x,y,z], [roll,pitch,yaw], [sx,sy,sz]).
        """
        location = [0.0, 0.0, 0.0]
        rotation = [0.0, 0.0, 0.0]
        scale    = [0.0, 0.0, 0.0]

        # Flush any stale data sitting in the socket buffer
        if self._sock is not None:
            self._sock.setblocking(False)
            try:
                while self._sock.recv(4096):
                    pass
            except Exception:
                pass
            self._sock.setblocking(True)
            self._sock.settimeout(self._timeout)

        if not self._send_container(self._QCAR2_CLASS_ID, self._actor,
                                    self._FCN_REQUEST_WORLD_TRANSFORM):
            return False, location, rotation, scale

        payload = self._wait_for_container(
            self._QCAR2_CLASS_ID, self._actor,
            self._FCN_RESPONSE_WORLD_TRANSFORM)

        if payload is not None and len(payload) == 36:
            vals = struct.unpack(">fffffffff", payload[0:36])
            location = [vals[0], vals[1], vals[2]]
            rotation = [vals[3], vals[4], vals[5]]
            scale    = [vals[6], vals[7], vals[8]]
            return True, location, rotation, scale

        return False, location, rotation, scale

    def close(self):
        self._close_sock()


# ===================================================================
#  Utilities
# ===================================================================

def now() -> float:
    return time.time()

def clamp(x: float, lo: float, hi: float) -> float:
    return float(max(lo, min(hi, x)))

def wrap_pi(a: float) -> float:
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


# ===================================================================
#  Waypoints
# ===================================================================

def load_waypoints_txt(path: str) -> Dict[str, np.ndarray]:
    txt = open(path, "r", encoding="utf-8").read()
    out: Dict[str, np.ndarray] = {}
    current_name = None
    current_lines: list = []
    for raw in txt.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.endswith(":") and line.lower().startswith("path_"):
            if current_name and current_lines:
                out[current_name] = _parse_xy_lines(current_lines)
            current_name = line[:-1].strip().lower()
            current_lines = []
            continue
        if current_name is None:
            continue
        if line.startswith("[") or line.startswith("]"):
            continue
        current_lines.append(line)

    if current_name and current_lines:
        out[current_name] = _parse_xy_lines(current_lines)
    if not out:
        raise RuntimeError(f"Couldn't parse any paths from {path}")
    return out


def _parse_xy_lines(lines) -> np.ndarray:
    pts = []
    for line in lines:
        if '#' in line:
            line = line[:line.index('#')]
        line = line.replace(";", "").replace(",", " ")
        parts = [p for p in line.split() if p]
        if len(parts) >= 2:
            pts.append((float(parts[0]), float(parts[1])))
    return np.array(pts, dtype=np.float64)


def interpolate_waypoints(pts: np.ndarray, spacing: float = 1.0) -> np.ndarray:
    """Resample a polyline at uniform arc-length spacing."""
    if pts.ndim != 2 or pts.shape[0] < 2:
        return pts
    diffs = np.diff(pts, axis=0)
    segs = np.hypot(diffs[:, 0], diffs[:, 1])
    cum = np.concatenate(([0.0], np.cumsum(segs)))
    total = cum[-1]
    if total < spacing:
        return pts
    n_pts = max(2, int(total / spacing) + 1)
    s_new = np.linspace(0.0, total, n_pts)
    x_new = np.interp(s_new, cum, pts[:, 0])
    y_new = np.interp(s_new, cum, pts[:, 1])
    return np.column_stack([x_new, y_new])


# ===================================================================
#  Map Data  (from Setup_Real_Scenario_fullscale_x10.py)
#  All coordinates are in pre-fs 1:1 scale (before the ×10 scaling).
#  facing_deg is the rotation from the Setup file – the direction the
#  sign's face points toward approaching traffic.
# ===================================================================

@dataclass
class MapSign:
    sign_type: str          # "stop", "yield", "roundabout"
    x: float
    y: float
    facing_deg: float
    trigger_radius: float = 0.45
    cooldown_s: float = 15.0

@dataclass
class MapTrafficLight:
    x: float
    y: float
    facing_deg: float
    pair_id: int            # TLs with same pair_id share colour state

# Stop signs -------------------------------------------------------
MAP_STOP_SIGNS: List[MapSign] = [
    # Parking-lot area
    MapSign("stop", -1.500,  3.600, -35.0,  trigger_radius=0.40, cooldown_s=15.0),
    MapSign("stop", -1.500,  2.200,  35.0,  trigger_radius=0.40, cooldown_s=15.0),
    # x+ side of map
    MapSign("stop",  2.410,  0.206, -90.0,  trigger_radius=0.40, cooldown_s=15.0),
    MapSign("stop",  1.766,  1.697,  90.0,  trigger_radius=0.40, cooldown_s=15.0),
]

# Yield signs ------------------------------------------------------
MAP_YIELD_SIGNS: List[MapSign] = [
    # One-way exit
    MapSign("yield",  0.000, -1.300, -180.0, trigger_radius=0.40, cooldown_s=12.0),
    # Roundabout yields
    MapSign("yield",  2.400,  3.200,  -90.0, trigger_radius=0.40, cooldown_s=12.0),
    MapSign("yield",  1.100,  2.800, -145.0, trigger_radius=0.40, cooldown_s=12.0),
    MapSign("yield",  0.490,  3.800,  135.0, trigger_radius=0.40, cooldown_s=12.0),
]

# Roundabout signs -------------------------------------------------
MAP_ROUNDABOUT_SIGNS: List[MapSign] = [
    MapSign("roundabout", 2.392, 2.522,  -90.0, trigger_radius=0.55, cooldown_s=15.0),
    MapSign("roundabout", 0.698, 2.483, -145.0, trigger_radius=0.55, cooldown_s=15.0),
    MapSign("roundabout", 0.007, 3.973,  135.0, trigger_radius=0.55, cooldown_s=15.0),
]

# All signs in a single flat list for proximity scan
ALL_MAP_SIGNS: List[MapSign] = MAP_STOP_SIGNS + MAP_YIELD_SIGNS + MAP_ROUNDABOUT_SIGNS

# Traffic lights  (intersection 1) ---------------------------------
#   pair 0 : TL1 + TL3  (initially RED;  GREEN when flag==2)
#   pair 1 : TL2 + TL4  (initially GREEN; RED when flag==2)
MAP_TRAFFIC_LIGHTS: List[MapTrafficLight] = [
    MapTrafficLight( 0.600,  1.550,    0.0, pair_id=0),  # TL1
    MapTrafficLight(-0.600,  1.280,   90.0, pair_id=1),  # TL2
    MapTrafficLight(-0.370,  0.300,  180.0, pair_id=0),  # TL3
    MapTrafficLight( 0.750,  0.480,  -90.0, pair_id=1),  # TL4
]

TL_DETECTION_RADIUS = 0.80   # 1:1 scale – activate YOLO TL colour detection


def check_sign_proximity(
    car_x: float, car_y: float,
    sign_x: float, sign_y: float, sign_facing_rad: float,
    radius: float,
) -> Tuple[bool, float]:
    """Return (triggered, distance) iff car is within *radius* of the sign
    AND on the front side (within ±108° of the sign's facing direction)."""
    dx = car_x - sign_x
    dy = car_y - sign_y
    dist = math.hypot(dx, dy)
    if dist > radius:
        return False, dist
    angle_to_car = math.atan2(dy, dx)
    angle_diff = wrap_pi(angle_to_car - sign_facing_rad)
    if abs(angle_diff) > math.radians(108):
        return False, dist
    return True, dist  


def now() -> float:
    return time.time()


def clamp(x: float, lo: float, hi: float) -> float:
    return float(max(lo, min(hi, x)))


def speed_to_throttle(speed_mps: float) -> float:
    """Very simple speed->throttle map for QCar."""
    speed_mps = float(speed_mps)
    if abs(speed_mps) <= 1e-3:
        return 0.0
    s = abs(speed_mps)
    thr = clamp(0.05 + 0.10 * s, 0.06, 0.35)
    return -thr if speed_mps < 0.0 else thr

def draw_steer_overlay(img: np.ndarray,
                       steer: float,
                       steer_max: float = 0.42,
                       title: str = "STEERING") -> np.ndarray:
    """
    Visual steering correction UI:
      - Bar fills LEFT or RIGHT
      - Large L / R indicator
      - Arrow showing direction
      - Intensity grows with steering magnitude
    """

    if img is None or img.size == 0:
        return img

    H, W = img.shape[:2]
    out = img.copy()

    # Clamp steer
    s = float(np.clip(steer, -steer_max, steer_max))
    magnitude = abs(s) / steer_max

    # Meter geometry
    meter_w = int(0.45 * W)
    meter_h = 22
    x0 = int(0.5 * W - meter_w / 2)
    y0 = H - 280

    center_x = x0 + meter_w // 2

    # Background
    cv2.rectangle(out,
                  (x0 - 10, y0 - 40),
                  (x0 + meter_w + 10, y0 + meter_h + 12),
                  (0, 0, 0), -1)

    # Bar outline
    cv2.rectangle(out,
                  (x0, y0),
                  (x0 + meter_w, y0 + meter_h),
                  (255, 255, 255), 2)

    # Center tick
    cv2.line(out,
             (center_x, y0 - 5),
             (center_x, y0 + meter_h + 5),
             (255, 255, 0), 2)

    # Determine direction
    if s > 0:
        # LEFT correction
        fill_x = int(center_x - magnitude * (meter_w // 2))
        cv2.rectangle(out,
                      (fill_x, y0),
                      (center_x, y0 + meter_h),
                      (0, 0, 255), -1)
        direction_label = "L"
        arrow_start = (center_x, y0 - 18)
        arrow_end = (center_x - 60, y0 - 18)

    elif s < 0:
        # RIGHT correction
        fill_x = int(center_x + magnitude * (meter_w // 2))
        cv2.rectangle(out,
                      (center_x, y0),
                      (fill_x, y0 + meter_h),
                      (0, 0, 255), -1)
        direction_label = "R"
        arrow_start = (center_x, y0 - 18)
        arrow_end = (center_x + 60, y0 - 18)

    else:
        direction_label = "C"
        arrow_start = arrow_end = None

    # Draw arrow if turning
    if arrow_start is not None:
        cv2.arrowedLine(out,
                        arrow_start,
                        arrow_end,
                        (0, 255, 255),
                        3,
                        tipLength=0.3)

    # Title
    cv2.putText(out,
                f"{title}: {s:+.2f}",
                (x0, y0 - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2)

    # Big L / R indicator
    cv2.putText(out,
                direction_label,
                (int(W * 0.5 - 20), y0 - 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.6,
                (0, 255, 255),
                3)

    return out

# =============================================================================== 
#
# Lane Controller - uses cv2 canny + houghlinesP to detect lane lines
#                   and compute steering correction to stay centered.
#                   Stronger correction when both left+right boundaries detected
#
# ===============================================================================
class RightLaneFollower:
    """
    Canny + HoughLinesP lane detection with:
      - Wider ROI trapezoid (better on curves)
      - LEFT + RIGHT boundary detection (center accurately)
      - Dropout tolerance (no instant stop on brief loss)
    """

    def __init__(self):
        # --- ROI: wider so you can see the lane through turns ---
        self.roi_top_y_ratio = 0.35        # lower means farther
        self.roi_bottom_y_ratio = 1.00
        self.roi_top_width_ratio = 0.55   # was ~0.18 (much wider at top)
        self.roi_bottom_width_ratio = 1.02 # was ~0.95 (almost full width)

        # --- Edge/Hough params (QLabs lines are clean; keep moderate thresholds) ---
        self.canny_low = 55
        self.canny_high = 155
        self.hough_rho = 2
        self.hough_theta = np.pi / 180
        self.hough_thresh = 28
        self.hough_min_line_len = 22
        self.hough_max_line_gap = 40

        # --- Lane geometry assumption (px) ---
        self.lane_width_px = 360.0  # will self-adapt when both sides detected

        # --- Control gains ---
        self.k_lat = 1.00   # stronger for tighter centering
        self.k_head = 0.25
        self.kd = 0.04
        self.max_steer = 0.60

        self.prev_err = 0.0
        self.steer_smooth = 0.0

        # --- Dropout tolerance ---
        self.last_good_time = 0.0
        self.last_good_steer = 0.0
        self.loss_grace_s = 0.35     # keep driving/steer for short losses
        self.slow_after_s = 0.55     # start slowing after this
        self.stop_after_s = 1.20     # only stop if lane missing this long

        # Persisted boundary/state for short dropout recovery & right-biasing
        self.last_left = None           # last reliable left x (px)
        self.last_right = None          # last reliable right x (px)
        self.last_bound_time = 0.0      # timestamp of last reliable bound
        self.max_bound_age_s = 2.0      # reuse persisted bounds within this age
        self.bias_right_px = 0.0        # absolute px bias (overrides fraction if >0)
        self.target_offset_frac = 0.20  # fraction of lane width to shift right-of-center
        self.last_center = None

    def _roi_mask(self, H: int, W: int) -> np.ndarray:
        y_top = int(self.roi_top_y_ratio * H)
        y_bot = int(self.roi_bottom_y_ratio * H)

        top_w = int(self.roi_top_width_ratio * W)
        bot_w = int(self.roi_bottom_width_ratio * W)

        x_mid = W // 2
        pts = np.array([
            [x_mid - bot_w // 2, y_bot],
            [x_mid - top_w // 2, y_top],
            [x_mid + top_w // 2, y_top],
            [x_mid + bot_w // 2, y_bot],
        ], dtype=np.int32)

        mask = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        return mask

    @staticmethod
    def _fit_line_from_segments(segments):
        """
        Fit y = m x + b using least squares from many Hough segments.
        segments: list of (x1,y1,x2,y2)
        returns (m,b, weight) or (None,None,0)
        """
        if not segments:
            return None, None, 0.0

        xs = []
        ys = []
        wts = []
        for (x1, y1, x2, y2) in segments:
            # sample endpoints; weight by segment length
            L = math.hypot(x2 - x1, y2 - y1)
            if L < 8:
                continue
            xs.extend([x1, x2])
            ys.extend([y1, y2])
            wts.extend([L, L])

        if len(xs) < 4:
            return None, None, 0.0

        x = np.array(xs, dtype=np.float32)
        y = np.array(ys, dtype=np.float32)
        w = np.array(wts, dtype=np.float32)

        # weighted least squares for y = m x + b
        W = np.diag(w)
        A = np.column_stack([x, np.ones_like(x)])
        try:
            sol = np.linalg.lstsq(W @ A, W @ y, rcond=None)[0]
            m, b = float(sol[0]), float(sol[1])
            return m, b, float(np.sum(w))
        except Exception:
            return None, None, 0.0

    @staticmethod
    def _x_at_y(m: float, b: float, y: float) -> float:
        if m is None or b is None or abs(m) < 1e-6:
            return float("nan")
        return (y - b) / m

    def step(self, bgr: np.ndarray, dt: float, t_now: float, debug: bool = True):
        if bgr is None or bgr.size == 0:
            return 0.0, False, 0.0, None

        if not np.isfinite(dt) or dt <= 0:
            dt = 1.0 / 30.0

        H, W = bgr.shape[:2]
        mid_x = 0.5 * W

        # Contrast-limited adaptive histogram equalization to boost faint lines
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_clahe = clahe.apply(gray)

        # Color mask to focus on white/yellow lane markings (broader/robust ranges)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # White: low saturation, high value
        white_mask = cv2.inRange(hsv, np.array((0, 0, 200)), np.array((180, 75, 255)))
        # Yellow: broader hue/sat/val range to handle lighting variations
        yellow_mask = cv2.inRange(hsv, np.array((10, 60, 100)), np.array((40, 255, 255)))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)
        # small open to remove speckle, but keep thin lines
        km = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, km, iterations=1)

        # Adaptive Canny thresholds based on local median (prefer inside color mask)
        blur = cv2.GaussianBlur(gray_clahe, (5, 5), 1.2)
        try:
            if np.count_nonzero(color_mask):
                med = float(np.median(blur[color_mask > 0]))
            else:
                med = float(np.median(blur))
        except Exception:
            med = float(np.median(blur))
        sigma = 0.50
        low = int(max(8, (1.0 - sigma) * med))
        high = int(min(255, (1.0 + sigma) * med))
        if low >= high:
            low = max(8, int(0.5 * high))

        edges = cv2.Canny(blur, low, high)

        # Connect faint segments
        edges = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations=1)

        # If color mask is strong enough, restrict edges to it; otherwise don't mask
        color_count = int(np.count_nonzero(color_mask))
        if color_count > 500:
            # dilate color mask so thin edges aren't lost by strict overlap
            km2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            color_mask_d = cv2.dilate(color_mask, km2, iterations=1)
            edges = cv2.bitwise_and(edges, edges, mask=color_mask_d)

        # close small gaps after potential masking
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)), iterations=1)

        # ROI mask (same as before) and crop to ROI
        roi = self._roi_mask(H, W)
        edges_roi = cv2.bitwise_and(edges, edges, mask=roi)

        # debug views
        if debug:
            try:
                cv2.imshow("gray_clahe", gray_clahe)
                cv2.imshow("color_mask", color_mask)
                cv2.imshow("canny_edges", edges)
            except Exception:
                pass

        lines = cv2.HoughLinesP(
            edges_roi,
            rho=self.hough_rho,
            theta=self.hough_theta,
            threshold=self.hough_thresh,
            minLineLength=self.hough_min_line_len,
            maxLineGap=self.hough_max_line_gap,
        )

        left_segs = []
        right_segs = []

        if lines is not None:
            for (x1, y1, x2, y2) in lines[:, 0]:
                dx = (x2 - x1)
                dy = (y2 - y1)
                if abs(dx) < 3:
                    continue
                m = dy / dx

                # Reject near-horizontal and absurdly steep noise
                if abs(m) < 0.30 or abs(m) > 6.0:
                    continue

                # Classify by slope sign and position:
                # - Right boundary: positive slope, mostly on right half
                # - Left boundary: negative slope, mostly on left half
                if m > 0 and max(x1, x2) > int(0.52 * W):
                    right_segs.append((x1, y1, x2, y2))
                elif m < 0 and min(x1, x2) < int(0.48 * W):
                    left_segs.append((x1, y1, x2, y2))

        # Fit robust single line for each side
        mL, bL, wL = self._fit_line_from_segments(left_segs)
        mR, bR, wR = self._fit_line_from_segments(right_segs)

        y_bottom = float(H - 1)
        xL_bottom = self._x_at_y(mL, bL, y_bottom) if mL is not None else float("nan")
        xR_bottom = self._x_at_y(mR, bR, y_bottom) if mR is not None else float("nan")

        have_L = np.isfinite(xL_bottom)
        have_R = np.isfinite(xR_bottom)

        # sanity checks
        if have_L and (xL_bottom < 0 or xL_bottom > W):
            have_L = False
        if have_R and (xR_bottom < 0 or xR_bottom > W):
            have_R = False

        # --- Choose lane center ---
        # Prefer measured bounds; fall back to recently persisted bounds if available
        recent_ok = (t_now - self.last_bound_time) < self.max_bound_age_s
        xL_use = xL_bottom if have_L else (self.last_left if (self.last_left is not None and recent_ok) else float("nan"))
        xR_use = xR_bottom if have_R else (self.last_right if (self.last_right is not None and recent_ok) else float("nan"))
        have_xL = np.isfinite(xL_use)
        have_xR = np.isfinite(xR_use)

        if have_xL and have_xR and (xR_use > xL_use + 40):
            # Both sides available -> true center with optional right bias
            w_est = float(xR_use - xL_use)
            if 180.0 < w_est < 900.0:
                self.lane_width_px = 0.95 * self.lane_width_px + 0.05 * w_est

            offset_px = self.bias_right_px if abs(self.bias_right_px) > 1e-6 else (self.target_offset_frac * w_est)
            x_center = 0.5 * (xL_use + xR_use) + offset_px

            mode = "LR"
            conf = clamp((wL + wR) / 900.0, 0.0, 1.0)

            angL = math.atan(mL) if mL is not None else 0.0
            angR = math.atan(mR) if mR is not None else 0.0
            ang = 0.5 * (angL + angR)
        elif have_xR:
            # Right-only fallback; bias toward right side
            bias_term = self.bias_right_px if (self.bias_right_px > 0.0) else (self.target_offset_frac * self.lane_width_px)
            x_center = xR_use - 0.5 * self.lane_width_px + bias_term
            mode = "R"
            conf = clamp(wR / 650.0, 0.0, 1.0)
            ang = math.atan(mR) if mR is not None else math.radians(70.0)
        elif have_xL:
            # Left-only fallback
            bias_term = self.bias_right_px if (self.bias_right_px > 0.0) else (self.target_offset_frac * self.lane_width_px)
            x_center = xL_use + 0.5 * self.lane_width_px + bias_term
            mode = "L"
            conf = clamp(wL / 650.0, 0.0, 1.0)
            ang = math.atan(mL) if mL is not None else math.radians(110.0)
        else:
            # No lane this frame -> handle dropout gracefully
            dbg = None
            if debug:
                dbg = cv2.cvtColor(edges_roi, cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg, "LANE: LOST", (10, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return float(self.last_good_steer), False, 0.0, dbg

        x_center = float(np.clip(x_center, 0.0, W - 1.0))

        # --- Control (center on lane) ---
        err = (mid_x - x_center) / max(1.0, mid_x)
        derr = (err - self.prev_err) / max(1e-3, dt)
        self.prev_err = err

        # heading term: steer to keep lane orientation stable
        # target angle: about 70deg for right boundary / ~90 for vertical.
        nominal = math.radians(85.0)
        head_err = clamp((nominal - ang), -0.9, 0.9)

        steer = self.k_lat * err + self.k_head * head_err + self.kd * derr
        steer = clamp(steer, -self.max_steer, self.max_steer)

        # smooth
        self.steer_smooth = 0.78 * self.steer_smooth + 0.22 * steer
        steer = self.steer_smooth

        # mark good
        lane_ok = conf > 0.18
        if lane_ok:
            self.last_good_time = t_now
            self.last_good_steer = steer
            # persist measured bounds when frame is confident
            if have_L:
                self.last_left = float(xL_bottom)
            if have_R:
                self.last_right = float(xR_bottom)
            self.last_bound_time = t_now

        # remember last center for diagnostics
        try:
            self.last_center = float(x_center)
        except Exception:
            pass

        dbg = None
        if debug:
            dbg = cv2.cvtColor(edges_roi, cv2.COLOR_GRAY2BGR)
            # show ROI edge
            roi_edges = cv2.Canny(roi, 50, 150)
            dbg[roi_edges > 0] = (255, 255, 255)

            # draw fitted lines
            def draw_line(m, b, color):
                if m is None or b is None:
                    return
                y1 = int(0.55 * H)
                y2 = H - 1
                x1 = int(self._x_at_y(m, b, y1))
                x2 = int(self._x_at_y(m, b, y2))
                if 0 <= x1 < W and 0 <= x2 < W:
                    cv2.line(dbg, (x1, y1), (x2, y2), color, 3)

            draw_line(mL, bL, (255, 0, 0))   # left = blue
            draw_line(mR, bR, (0, 255, 0))   # right = green

            yb = H - 6
            cv2.circle(dbg, (int(x_center), yb), 7, (0, 0, 255), -1)

            # draw persisted bound markers if present
            if self.last_left is not None:
                try:
                    cv2.circle(dbg, (int(self.last_left), yb), 5, (128, 128, 255), -1)
                except Exception:
                    pass
            if self.last_right is not None:
                try:
                    cv2.circle(dbg, (int(self.last_right), yb), 5, (128, 255, 128), -1)
                except Exception:
                    pass

            cv2.putText(dbg, f"mode={mode} conf={conf:.2f} err={err:+.2f} head={math.degrees(head_err):+.1f} steer={steer:+.2f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
            cv2.putText(dbg, f"lane_width_px={self.lane_width_px:.0f}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        return float(steer), bool(lane_ok), float(conf), dbg


def run_lane_only(actor: int, rate_hz: float, speed_mps: float, debug: bool,
                  sign_model_path: Optional[str] = None,
                  sign_labels_path: str = "sign_labels.txt",
                  sign_conf: float = 0.35,
                  sign_device: Optional[str] = None,
                  sign_stride: int = 3):
    car = QCar(readMode=1, frequency=int(rate_hz))
    cam = Camera2D(
        cameraId="3@tcpip://localhost:18964",
        frameWidth=820,
        frameHeight=410,
        frameRate=rate_hz,
    )

    follower = RightLaneFollower()

    # --- LIDAR init & mapper ---
    use_lidar = True
    lidar = None
    lidar_thread = None
    lidar_stop_evt = threading.Event()
    lidar_lock = threading.Lock()
    latest_scan = {"angles": None, "distances": None, "t": 0.0}
    # persistent world-frame occupancy map: 20m x 20m at 60 px/m
    mapper = LidarMapper(map_size_m=(20.0, 20.0), px_per_m=60.0, origin_world=(2.0, 0.0))
    qlabs = None
    try:
        qlabs = _QLabsWorldTransform(actor=actor)
    except Exception:
        qlabs = None
    # --- Sign detection / response state ---
    sign_model_obj = None
    sign_names: List[str] = []
    sign_frame_q: "queue.Queue" = queue.Queue(maxsize=1)
    sign_worker_stop = threading.Event()
    sign_state = {
        'stop_sign_active': False,
        'stop_sign_stopped_t': 0.0,
        'yield_slow_until': 0.0,
        'tl_red_until': 0.0,
        'tl_yellow_until': 0.0,
        'tl_green_until': 0.0,
        'sign_last_trigger': {},
    }
    sign_lock = threading.Lock()
    # tuning constants
    STOP_SIGN_DWELL_S = 3.0
    YIELD_SPEED = 1.0
    # attempt to load sign model if provided
    if sign_model_path is not None:
        try:
            from ultralytics import YOLO
            try:
                sign_model_obj = YOLO(sign_model_path)
            except Exception:
                sign_model_obj = None
        except Exception:
            sign_model_obj = None
        # load label names if available
        try:
            with open(sign_labels_path, 'r', encoding='utf-8') as fh:
                sign_names = [l.strip() for l in fh.read().splitlines() if l.strip()]
        except Exception:
            sign_names = []

    def _sign_worker():
        """Background worker: consumes RGB frames (numpy) from `sign_frame_q` and
        runs YOLO inference to set traffic-light state timestamps in `sign_state`.
        """
        while not sign_worker_stop.is_set():
            try:
                rgb = sign_frame_q.get(timeout=0.1)
            except Exception:
                continue
            if sign_model_obj is None:
                continue
            try:
                results = sign_model_obj.predict(source=rgb, conf=sign_conf, device=sign_device, verbose=False)
                for r in results:
                    boxes = getattr(r, 'boxes', None)
                    if boxes is None:
                        continue
                    for b in boxes:
                        try:
                            cls_idx = int(b.cls[0].item()) if hasattr(b, 'cls') else int(getattr(b, 'cls'))
                        except Exception:
                            continue
                        name = sign_names[cls_idx] if 0 <= cls_idx < len(sign_names) else None
                        if name is None:
                            continue
                        with sign_lock:
                            texp = now() + 2.0
                            if 'red' in name:
                                sign_state['tl_red_until'] = max(sign_state['tl_red_until'], texp)
                            elif 'yellow' in name:
                                sign_state['tl_yellow_until'] = max(sign_state['tl_yellow_until'], texp)
                            elif 'green' in name:
                                sign_state['tl_green_until'] = max(sign_state['tl_green_until'], texp)
            except Exception:
                pass

    sign_worker_thread = None
    if sign_model_obj is not None:
        sign_worker_thread = threading.Thread(target=_sign_worker, daemon=True)
        sign_worker_thread.start()
    # track previous world pose to compute movement vector
    last_world_pos = None  # (x,y)
    last_world_t = 0.0
    movement_heading = None

    if use_lidar:
        try:
            lidar = QCarLidar(numMeasurements=1000, rangingDistanceMode=2, interpolationMode=0, enableFiltering=True, angularResolution=math.radians(1.0))
        except Exception as e:
            lidar = None
            if debug:
                print(f"[WARN] LIDAR init failed: {e}")

    def _lidar_reader():
        while not lidar_stop_evt.is_set():
            if lidar is None:
                time.sleep(0.1)
                continue
            try:
                _ = lidar.read()
                a = getattr(lidar, 'angles', None)
                d = getattr(lidar, 'distances', None)
                with lidar_lock:
                    latest_scan['angles'] = None if a is None else a.copy()
                    latest_scan['distances'] = None if d is None else d.copy()
                    latest_scan['t'] = now()
            except Exception as e:
                if debug:
                    print(f"[WARN] LIDAR read error: {e}")
            time.sleep(0.02)

    if use_lidar:
        lidar_thread = threading.Thread(target=_lidar_reader, daemon=True)
        lidar_thread.start()
    # --- end LIDAR init ---

    dt_nom = 1.0 / float(rate_hz)
    t_prev = now()

    steer_smooth = 0.0
    # estimated yaw from onboard gyroscope (rad)
    est_yaw = 0.0

    try:
        while True:
            t = now()
            dt = t - t_prev
            t_prev = t
            if not np.isfinite(dt) or dt <= 0:
                dt = dt_nom

            # Read sensors
            try:
                car.read()
            except Exception:
                pass
            # integrate yaw from IMU if available for UI heading fallback
            try:
                yaw_rate = float(car.gyroscope[2]) if hasattr(car, "gyroscope") else 0.0
                est_yaw += yaw_rate * dt
                est_yaw = wrap_pi(est_yaw)
            except Exception:
                pass

            bgr = None
            try:
                cam.read()
                bgr = cam.imageData
            except Exception:
                bgr = None

            if bgr is None or not getattr(bgr, "size", 0):
                # no camera: stop
                car.read_write_std(throttle=0.0, steering=0.0)
                time.sleep(dt_nom)
                continue

            # Consume latest lidar scan (non-blocking)
            with lidar_lock:
                _angles = latest_scan.get('angles', None)
                _dist = latest_scan.get('distances', None)
                _ts = latest_scan.get('t', 0.0)

            if _angles is not None and _dist is not None:
                # store raw scan for fallback rendering
                try:
                    mapper.store_raw_scan(_angles, _dist)
                except Exception:
                    pass
                pts = polar_to_xy(_angles, _dist)
                try:
                    pose = None
                    if qlabs is not None and qlabs.connected:
                        try:
                            ok, location, rotation, scale = qlabs.get_world_transform()
                            if ok:
                                x = float(location[0]); y = float(location[1]); yaw = float(rotation[2])
                                pose = (x, y, yaw)
                                # compute movement heading from last_world_pos -> current
                                t_pose = now()
                                if last_world_pos is not None:
                                    dtp = t_pose - last_world_t
                                    if dtp > 1e-3:
                                        dx = x - last_world_pos[0]
                                        dy = y - last_world_pos[1]
                                        dist2 = dx * dx + dy * dy
                                        if dist2 > 1e-6:
                                            movement_heading = math.atan2(dy, dx)
                                last_world_pos = (x, y)
                                last_world_t = t_pose
                        except Exception:
                            pose = None
                    mapper.accumulate(pts, pose, _ts)
                except Exception:
                    pass

            t_now = now()
            steer_cmd, lane_ok, conf, dbg = follower.step(bgr, dt, t_now=t_now, debug=debug)

            time_since_good = t_now - follower.last_good_time

            if lane_ok or time_since_good <= follower.loss_grace_s:
                # keep driving through brief dropouts
                v_cmd = speed_mps
            elif time_since_good <= follower.slow_after_s:
                # start easing off
                v_cmd = 0.8 * speed_mps
            elif time_since_good <= follower.stop_after_s:
                # creep, trying to reacquire
                v_cmd = 0.35
            else:
                # only stop if truly lost for a while
                v_cmd = 0.0

            # --- Sign handling (map proximity + optional YOLO TL detection) ---
            try:
                # push frame to sign worker at reduced rate
                if sign_model_obj is not None:
                    sign_frame_q.put_nowait(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            except Exception:
                try:
                    # replace queued frame if full
                    _ = sign_frame_q.get_nowait()
                    sign_frame_q.put_nowait(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                except Exception:
                    pass

            # map-based proximity triggers (use world pose if available)
            try:
                car_x = car_y = car_yaw = None
                if 'pose' in locals() and pose is not None:
                    car_x, car_y, car_yaw = float(pose[0]), float(pose[1]), float(pose[2])
                elif last_world_pos is not None:
                    car_x, car_y = last_world_pos[0], last_world_pos[1]
                    car_yaw = movement_heading if movement_heading is not None else 0.0

                if car_x is not None:
                    for i, ms in enumerate(ALL_MAP_SIGNS):
                        trig, dist = check_sign_proximity(car_x, car_y, ms.x, ms.y, math.radians(ms.facing_deg), ms.trigger_radius)
                        if not trig:
                            continue
                        last_t = sign_state['sign_last_trigger'].get(i, 0.0)
                        if now() - last_t < ms.cooldown_s:
                            continue
                        sign_state['sign_last_trigger'][i] = now()
                        if ms.sign_type == 'stop':
                            sign_state['stop_sign_active'] = True
                            sign_state['stop_sign_stopped_t'] = 0.0
                        elif ms.sign_type == 'yield':
                            sign_state['yield_slow_until'] = max(sign_state['yield_slow_until'], now() + 3.0)
                        elif ms.sign_type == 'roundabout':
                            # roundabout handling could set steer bias; ignore for now
                            pass
            except Exception:
                pass

            # apply sign state to v_cmd (under lock)
            try:
                with sign_lock:
                    tcur = now()
                    # traffic light red overrides
                    if tcur < sign_state.get('tl_red_until', 0.0):
                        v_cmd = 0.0
                    # stop sign active: ensure dwell
                    if sign_state.get('stop_sign_active', False):
                        if sign_state.get('stop_sign_stopped_t', 0.0) <= 0.0:
                            sign_state['stop_sign_stopped_t'] = tcur
                            v_cmd = 0.0
                        else:
                            if (tcur - sign_state['stop_sign_stopped_t']) >= STOP_SIGN_DWELL_S:
                                sign_state['stop_sign_active'] = False
                                sign_state['stop_sign_stopped_t'] = 0.0
                    # yield slows
                    if tcur < sign_state.get('yield_slow_until', 0.0):
                        v_cmd = min(v_cmd, YIELD_SPEED)
            except Exception:
                pass

            throttle = speed_to_throttle(v_cmd)

            # Extra smoothing at actuation
            steer_smooth = 0.80 * steer_smooth + 0.20 * steer_cmd
            steer_smooth = clamp(steer_smooth, -0.60, 0.60)

            car.read_write_std(throttle=throttle, steering=steer_smooth)

            if debug and dbg is not None:
                dbg_ui = draw_steer_overlay(dbg, steer_smooth, steer_max=0.42, title="STEER")
                cv2.imshow("right_lane_edges_hough", dbg_ui)
                cv2.imshow("camera", bgr)
                try:
                    # Map steering to arrow orientation: base west (left) then rotate by steering
                    # Left turns should rotate arrow CCW, right turns CW.
                    try:
                        use_steering_for_arrow = True
                        if use_steering_for_arrow:
                            base_angle = math.pi + math.radians(-3.0)  # west + 3 degrees CW
                            max_steer = getattr(follower, 'max_steer', 0.60)
                            steer_scale = math.radians(60.0) / max_steer
                            ui_heading = base_angle - float(steer_smooth) * steer_scale
                        else:
                            ui_heading = None
                            if movement_heading is not None:
                                ui_heading = movement_heading
                            elif 'pose' in locals() and pose is not None:
                                ui_heading = float(pose[2])
                            if ui_heading is None:
                                ui_heading = est_yaw
                    except Exception:
                        ui_heading = est_yaw
                    bird = mapper.render(show_car=True, heading_override=ui_heading)
                    cv2.imshow('lidar_birdeye', bird)
                except Exception:
                    pass
                cv2.waitKey(1)

            # pacing
            sleep_dt = dt_nom - (now() - t)
            if sleep_dt > 0:
                time.sleep(sleep_dt)

    finally:
        try:
            car.terminate()
        except Exception:
            pass
        # stop lidar thread and terminate lidar
        try:
            lidar_stop_evt.set()
            if lidar_thread is not None:
                lidar_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            sign_worker_stop.set()
            if sign_worker_thread is not None:
                sign_worker_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if lidar is not None:
                lidar.terminate()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--actor", type=int, default=0)
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=1.6)  # start slower while tuning
    ap.add_argument("--no-debug", action="store_true")
    ap.add_argument("--sign-model", type=str, default=None, help="path to YOLO sign model (optional)")
    ap.add_argument("--sign-labels", type=str, default="sign_labels.txt", help="path to sign labels file")
    ap.add_argument("--sign-conf", type=float, default=0.35, help="YOLO confidence for sign detection")
    ap.add_argument("--sign-device", type=str, default=None, help="device for YOLO inference (cpu/cuda)")
    ap.add_argument("--sign-stride", type=int, default=3, help="inference stride (frames)")
    
    args = ap.parse_args()

    run_lane_only(
        actor=args.actor,
        rate_hz=args.rate,
        speed_mps=args.speed,
        debug=(not args.no_debug),
        sign_model_path=args.sign_model,
        sign_labels_path=args.sign_labels,
        sign_conf=args.sign_conf,
        sign_device=args.sign_device,
        sign_stride=args.sign_stride,
    )


if __name__ == "__main__":
    main()