"""qcar2_detailed_scenario_runner.py  –  Custom TCP + Map-Known Signs

Localization : Direct TCP to QLabs for get_world_transform() every frame.
               Returns simulation x10 coordinates; scaled ×0.10 to
               match the 1:1 real-world frame used by the path controller.

Path following: Pure Pursuit on interpolated waypoint segments.
Lane detection: secondary centering correction.

Signs / TLs  : positions hard-coded from Setup_Real_Scenario_fullscale_x10.py.
               Stop, yield, roundabout signs  → proximity triggers.
               Traffic lights                  → YOLO colour when close.

Safety       : depth + LiDAR frontal obstacle detection + sidewalk guard.
"""

import argparse
import json
import math
import socket
import struct
import time
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import cv2
import numpy as np

from pal.products.qcar import QCar, QCarLidar, QCarRealSense
from pal.utilities.vision import Camera2D
from hal.utilities.control import PurePursuitController

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
        if self._sock is None:
            return False
        container_size = self._BASE_CONTAINER_SIZE + len(payload)
        pkt = (
            struct.pack("<i", 1 + container_size)         # packet length
            + struct.pack(">BiiiB", 123, container_size,  # magic + header
                          class_id, actor, func)
            + payload
        )
        try:
            self._sock.sendall(pkt)
            return True
        except Exception:
            return False

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
    MapSign("stop", -1.500,  3.600, -35.0,  trigger_radius=0.54, cooldown_s=15.0),
    MapSign("stop", -1.500,  2.200,  35.0,  trigger_radius=0.54, cooldown_s=15.0),
    # x+ side of map
    MapSign("stop",  2.410,  0.206, -90.0,  trigger_radius=0.54, cooldown_s=15.0),
    MapSign("stop",  1.766,  1.697,  90.0,  trigger_radius=0.54, cooldown_s=15.0),
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
# Increased trigger radius for earlier detection to prevent wrong-way entry
MAP_ROUNDABOUT_SIGNS: List[MapSign] = [
    MapSign("roundabout", 2.392, 2.522,  -90.0, trigger_radius=1.2, cooldown_s=15.0),
    MapSign("roundabout", 0.698, 2.483, -145.0, trigger_radius=1.2, cooldown_s=15.0),
    MapSign("roundabout", 0.007, 3.973,  135.0, trigger_radius=1.2, cooldown_s=15.0),
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

TL_DETECTION_RADIUS = 0.75   # 1:1 scale – activate YOLO TL colour detection

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
    if abs(angle_diff) > math.radians(90):
        return False, dist
    return True, dist

# Sign detection gating (relative bearing of sign from car heading).
SIGN_REL_BEARING_MIN_DEG = -90.0  # full front hemisphere
SIGN_REL_BEARING_MAX_DEG =  90.0

# Max misalignment between car heading and expected approach direction for a sign.
# A car approaching a sign HEAD-ON has alignment ~0 deg. A car passing a sign sideways
# (e.g. northbound car next to an east-facing sign) has alignment ~90 deg.
# Setting this to 45 deg blocks sideways false-positives while catching head-on approaches.
SIGN_HEADING_ALIGN_MAX_DEG = 45.0

def sign_in_front_and_right(
    car_x: float, car_y: float, car_heading_rad: float,
    sign_x: float, sign_y: float,
    sign_facing_rad: float = None,
) -> bool:
    """Return True iff the sign lies within +/-90 deg of the car's heading AND the car is
    approaching the sign from roughly the correct direction.

    The heading alignment check is the primary discriminator: it computes whether the
    car's heading is close to the expected approach direction for this sign
    (sign_facing + 180 deg). This blocks signs the car is passing sideways
    (e.g. sign #3 at 1.766,1.697 facing east when the car heads north -- 180 deg mismatch)
    while still catching signs the car approaches head-on.
    """
    dx = sign_x - car_x
    dy = sign_y - car_y
    bearing_to_sign = math.atan2(dy, dx)
    rel_bearing = wrap_pi(bearing_to_sign - car_heading_rad)

    lo = math.radians(float(SIGN_REL_BEARING_MIN_DEG))
    hi = math.radians(float(SIGN_REL_BEARING_MAX_DEG))
    if lo <= hi:
        in_bearing = bool(lo <= rel_bearing <= hi)
    else:
        in_bearing = bool((rel_bearing >= lo) or (rel_bearing <= hi))

    if not in_bearing:
        return False

    # Heading alignment: car heading vs expected approach direction
    if sign_facing_rad is not None:
        approach_dir = sign_facing_rad + math.pi  # direction car should travel TOWARD sign
        heading_align = abs(wrap_pi(car_heading_rad - approach_dir))
        if heading_align > math.radians(SIGN_HEADING_ALIGN_MAX_DEG):
            return False

    return True

# ===================================================================

#  Lane Controller  (yellow / white boundary fitting)
# ===================================================================

class LaneController:
    """Fit yellow/white boundaries and command lane-center steering.

    Sign convention: steer_raw positive = LEFT (internal).
    """

    def __init__(self):
        self.kernel = np.ones((5, 5), np.uint8)
        self.prev_err = 0.0
        self.lane_width_px = 360.0
        # Slight right bias = lane center (not road center); too high = hug right edge
        self.bias_right_px = 20.0
        self.k_lat = 0.65
        self.k_head = 0.25
        self.kd = 0.02
        self.max_steer = 0.34
        self.steer_smooth = 0.0
        self.last_dbg = None

    # --- helpers ---

    @staticmethod
    def _strip_x(mask: np.ndarray, y1: int, y2: int, *, min_pix: int = 120) -> float | None:
        y1 = int(np.clip(y1, 0, mask.shape[0] - 1))
        y2 = int(np.clip(y2, 0, mask.shape[0]))
        if y2 <= y1:
            return None
        win = mask[y1:y2, :]
        xs = np.where(win > 0)[1]
        if xs.size < min_pix:
            return None
        return float(np.percentile(xs.astype(np.float32, copy=False), 50))

    @staticmethod
    def _strip_x_range(mask: np.ndarray, y1: int, y2: int,
                       x1: int, x2: int, *, min_pix: int = 120) -> float | None:
        y1 = int(np.clip(y1, 0, mask.shape[0] - 1))
        y2 = int(np.clip(y2, 0, mask.shape[0]))
        x1 = int(np.clip(x1, 0, mask.shape[1] - 1))
        x2 = int(np.clip(x2, 0, mask.shape[1]))
        if (y2 <= y1) or (x2 <= x1):
            return None
        win = mask[y1:y2, x1:x2]
        xs = np.where(win > 0)[1]
        if xs.size < min_pix:
            return None
        xs = xs.astype(np.float32, copy=False) + float(x1)
        return float(np.percentile(xs, 50))

    @classmethod
    def _boundary_pose(cls, mask: np.ndarray, roi_h: int
                       ) -> Tuple[float | None, Tuple[float, float] | None]:
        x_bot = cls._strip_x(mask, roi_h - 26, roi_h - 2)
        x_mid = cls._strip_x(mask, roi_h - 92, roi_h - 64)
        if x_bot is None:
            return None, None
        if x_mid is None:
            return x_bot, None
        vx = float(x_mid - x_bot)
        vy = float((roi_h - 78) - (roi_h - 14))
        if vy < 0.0:
            vx, vy = -vx, -vy
        if abs(vy) < 1e-3:
            return x_bot, None
        return x_bot, (vx, vy)

    @classmethod
    def _boundary_pose_range(cls, mask: np.ndarray, roi_h: int,
                             x1: int, x2: int, *, min_pix: int = 120
                             ) -> Tuple[float | None, Tuple[float, float] | None]:
        x_bot = cls._strip_x_range(mask, roi_h - 26, roi_h - 2, x1, x2, min_pix=min_pix)
        x_mid = cls._strip_x_range(mask, roi_h - 92, roi_h - 64, x1, x2, min_pix=min_pix)
        if x_bot is None:
            return None, None
        if x_mid is None:
            return x_bot, None
        vx = float(x_mid - x_bot)
        vy = float((roi_h - 78) - (roi_h - 14))
        if vy < 0.0:
            vx, vy = -vx, -vy
        if abs(vy) < 1e-3:
            return x_bot, None
        return x_bot, (vx, vy)

    # --- main step ---

    def step(self, bgr: np.ndarray, dt: float, debug: bool = False):
        if bgr is None or bgr.size == 0:
            return 0.0, False, 0.0, 0.0, None, None, None, None

        if not np.isfinite(dt) or dt <= 0.0:
            dt = 1.0 / 60.0

        H, W = bgr.shape[:2]
        y0 = int(0.62 * H)
        roi = bgr[y0:H, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        y_mask = cv2.inRange(hsv, (10, 70, 90), (45, 255, 255))
        h, s, v = cv2.split(hsv)
        w_mask = ((s < 70) & (v > 180)).astype(np.uint8) * 255

        y_mask = cv2.morphologyEx(y_mask, cv2.MORPH_OPEN, self.kernel)
        y_mask = cv2.morphologyEx(y_mask, cv2.MORPH_CLOSE, self.kernel)
        w_mask = cv2.morphologyEx(w_mask, cv2.MORPH_OPEN, self.kernel)
        w_mask = cv2.morphologyEx(w_mask, cv2.MORPH_CLOSE, self.kernel)

        # Edge detection: Canny on grayscale, restricted to lane-colored regions
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 1.0)
        canny = cv2.Canny(blur, 50, 150)
        lane_region = cv2.dilate(cv2.bitwise_or(y_mask, w_mask), np.ones((5, 5), np.uint8))
        edge_lane = cv2.bitwise_and(canny, lane_region)
        if np.count_nonzero(edge_lane) < 200:
            edge_lane = canny

        roi_h, roi_w = roi.shape[:2]
        y_eval = float(roi_h - 6)
        mid_x = int(0.5 * roi_w)
        edge_xL, edge_vL = self._boundary_pose_range(edge_lane, roi_h, 0, mid_x, min_pix=45)
        edge_xR, edge_vR = self._boundary_pose_range(edge_lane, roi_h, mid_x, roi_w, min_pix=45)

        # CRITICAL: Detect all THREE lines explicitly for better accuracy
        # Line 1: Left boundary (yellow line on left side)
        y_xL, y_vL = self._boundary_pose_range(y_mask, roi_h, 0, mid_x)
        # Line 2: Right boundary (white line on right side)  
        w_xR, w_vR = self._boundary_pose_range(w_mask, roi_h, mid_x, roi_w)
        # Line 3: Center line (yellow line on right side OR white line on left side)
        y_xR, y_vR = self._boundary_pose_range(y_mask, roi_h, mid_x, roi_w)
        w_xL, w_vL = self._boundary_pose_range(w_mask, roi_h, 0, mid_x)

        # Use left boundary (yellow left) and right boundary (white right) as primary
        y_x, y_v = (y_xL, y_vL) if (y_xL is not None) else (y_xR, y_vR)
        w_x, w_v = (w_xR, w_vR) if (w_xR is not None) else (w_xL, w_vL)
        
        # Determine center line from available detections
        center_line_x = None
        if y_xR is not None and w_xL is not None:
            # Both center line candidates detected - use average for accuracy
            center_line_x = 0.5 * (y_xR + w_xL)
        elif y_xR is not None:
            center_line_x = y_xR
        elif w_xL is not None:
            center_line_x = w_xL

        if (y_x is None) or (w_x is None):
            yg_x, yg_v = self._boundary_pose(y_mask, roi_h)
            wg_x, wg_v = self._boundary_pose(w_mask, roi_h)
            if y_x is None:
                y_x, y_v = yg_x, yg_v
            if w_x is None:
                w_x, w_v = wg_x, wg_v

        xl = xr = None
        x_center = None
        lane_mode = ""

        if (y_x is not None) and (w_x is not None):
            yxf = float(y_x)
            wxf = float(w_x)
            w_est = float(abs(wxf - yxf))
            
            # CRITICAL: Use tighter width constraints and slower adaptation
            if 170.0 < w_est < 900.0:
                # Slower adaptation (0.95 vs 0.92) for more stable, tighter lane width
                self.lane_width_px = 0.95 * self.lane_width_px + 0.05 * w_est  # Was 0.92/0.08
            
            # CRITICAL: Use all three lines when available for maximum accuracy
            if center_line_x is not None:
                # Three lines detected - use center line to refine boundaries
                center_xf = float(center_line_x)
                if yxf <= center_xf <= wxf:
                    # Center line is between left and right - use it to tighten boundaries
                    lane_mode = "3L"  # Three lines detected
                    xl = yxf
                    xr = wxf
                    # Use center line to refine center position - tighter calculation
                    x_center = center_xf + float(self.bias_right_px)
                elif yxf <= wxf:
                    # Normal case - center line might be outside, use standard calculation
                    lane_mode = "R"
                    xl = yxf
                    xr = wxf
                    x_center = 0.5 * (xl + xr) + float(self.bias_right_px)
                else:
                    # Overlapping case
                    lane_mode = "L->R"
                    xl = yxf
                    xr = xl + float(self.lane_width_px)
                    x_center = float(xl + 0.5 * self.lane_width_px + self.bias_right_px)
                    w_v = None
            elif yxf <= wxf:
                # Two lines detected - standard case
                lane_mode = "R"
                xl = yxf
                xr = wxf
                x_center = 0.5 * (xl + xr) + float(self.bias_right_px)
            else:
                lane_mode = "L->R"
                xl = yxf
                xr = xl + float(self.lane_width_px)
                x_center = float(xl + 0.5 * self.lane_width_px + self.bias_right_px)
                w_v = None
        elif y_x is not None:
            xl = float(y_x)
            xr = float(xl + self.lane_width_px)
            x_center = float(xl + 0.5 * self.lane_width_px + self.bias_right_px)
        elif w_x is not None:
            xr = float(w_x)
            xl = float(xr - self.lane_width_px)
            x_center = float(xr - 0.5 * self.lane_width_px + self.bias_right_px)

        # Fuse color boundaries with edge detection for robustness (shadows, worn paint)
        if edge_xL is not None:
            xl = (0.55 * xl + 0.45 * edge_xL) if xl is not None else float(edge_xL)
        if edge_xR is not None:
            xr = (0.55 * xr + 0.45 * edge_xR) if xr is not None else float(edge_xR)
        if xl is not None and xr is not None and np.isfinite(xl) and np.isfinite(xr):
            x_center = float(0.5 * (xl + xr) + self.bias_right_px)

        if x_center is None or not np.isfinite(x_center):
            if debug:
                self.last_dbg = roi
            return 0.0, False, 0.0, 0.0, (roi if debug else None), None, None, y0

        x_center = float(np.clip(x_center, 0.0, roi_w - 1.0))
        
        # CRITICAL: Use tighter boundaries - reduce lane width by 5% for safety margin
        # This prevents the detected lane from extending onto sidewalks
        tight_lane_width = 0.95 * self.lane_width_px  # 5% tighter for safety
        
        xl_use = float(np.clip(xl if xl is not None else (x_center - 0.5 * tight_lane_width), 0.0, roi_w - 1.0))
        xr_use = float(np.clip(xr if xr is not None else (x_center + 0.5 * tight_lane_width), 0.0, roi_w - 1.0))
        if xl_use > xr_use:
            xl_use, xr_use = xr_use, xl_use

        mid = 0.5 * roi_w
        err = float((mid - x_center) / max(1.0, mid))
        derr = float((err - self.prev_err) / max(1e-3, dt))
        self.prev_err = err

        v_list = []
        if y_v is not None:
            v_list.append(y_v)
        if w_v is not None:
            v_list.append(w_v)
        if v_list:
            vx = float(sum(v[0] for v in v_list) / len(v_list))
            vy = float(sum(v[1] for v in v_list) / len(v_list))
            ang = math.atan2(vy, vx)
            head_err = wrap_pi((math.pi / 2.0) - ang)
        else:
            head_err = 0.0

        steer = (self.k_lat * err) + (self.k_head * head_err) + (self.kd * derr)
        steer = clamp(steer, -self.max_steer, self.max_steer)
        # Low-pass to reduce oscillation from vision noise
        self.steer_smooth = 0.72 * self.steer_smooth + 0.28 * steer
        steer = self.steer_smooth

        y_nf = y_mask[max(0, roi_h - 120):roi_h, :]
        w_nf = w_mask[max(0, roi_h - 120):roi_h, :]
        lane_pix = float((y_nf > 0).mean() + (w_nf > 0).mean())
        conf = clamp(lane_pix / 0.07, 0.0, 1.0)
        valid = bool(conf > 0.25)

        dbg = None
        if debug:
            dbg = roi.copy()
            edge_overlay = cv2.cvtColor(edge_lane, cv2.COLOR_GRAY2BGR)
            edge_overlay[edge_lane > 0] = (0, 255, 0)
            cv2.addWeighted(edge_overlay, 0.35, dbg, 1.0, 0, dbg)
            cv2.circle(dbg, (int(x_center), int(y_eval)), 7, (0, 0, 255), -1)
            # Draw left boundary (yellow line)
            if xl is not None:
                cv2.line(dbg, (int(xl), 0), (int(xl), roi_h - 1), (0, 255, 255), 2)
            # Draw right boundary (white line)
            if xr is not None:
                cv2.line(dbg, (int(xr), 0), (int(xr), roi_h - 1), (255, 255, 255), 2)
            # Draw center line if detected (green dashed line)
            if center_line_x is not None:
                center_x_int = int(center_line_x)
                for y in range(0, roi_h, 20):  # Dashed line
                    cv2.line(dbg, (center_x_int, y), (center_x_int, min(y + 10, roi_h - 1)), (0, 255, 0), 2)
            if lane_mode:
                mode_text = f"mode={lane_mode}"
                if center_line_x is not None:
                    mode_text += " (3L)"  # Indicate three lines detected
                cv2.putText(dbg, mode_text, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(dbg, f"conf={conf:.2f} err={err:+.2f} head={head_err:+.2f}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(dbg, f"conf={conf:.2f} err={err:+.2f} head={head_err:+.2f}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.last_dbg = dbg

        return float(steer), bool(valid), float(conf), float(err), dbg, float(xl_use), float(xr_use), int(y0)

# ===================================================================
#  Sidewalk / Curb Guard
# ===================================================================

class SidewalkGuard:
    def __init__(self):
        self.kernel = np.ones((5, 5), np.uint8)

    def step(self, bgr: np.ndarray, lane_y0: int | None,
             xl: float | None, xr: float | None
             ) -> Tuple[float, float, float, np.ndarray | None]:
        if bgr is None or bgr.size == 0:
            return 0.0, 0.0, 0.0, None

        H, W = bgr.shape[:2]
        y0 = int(lane_y0) if (lane_y0 is not None) else int(0.72 * H)
        roi = bgr[y0:H, :]
        rH, rW = roi.shape[:2]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        s = hsv[:, :, 1].astype(np.float32)
        v = hsv[:, :, 2].astype(np.float32)
        L = lab[:, :, 0].astype(np.float32)

        py1, py2 = int(0.55 * rH), int(0.95 * rH)
        px1, px2 = int(0.35 * rW), int(0.65 * rW)
        patch_L = L[py1:py2, px1:px2]
        patch_s = s[py1:py2, px1:px2]
        patch_v = v[py1:py2, px1:px2]
        if patch_L.size:
            L_med = float(np.percentile(patch_L, 35))
            s_med = float(np.percentile(patch_s, 55))
            v_med = float(np.percentile(patch_v, 35))
        else:
            L_med, s_med, v_med = 120.0, 40.0, 120.0

        # NOTE: sw_center/sw_near are MEANS of a binary mask in [0,1]. In bright QLabs lighting,
        # the older thresholds were overly permissive and frequently classified asphalt as "sidewalk".
        # Tighten the LAB/HSV thresholds so only truly over-bright, low-saturation regions trigger.
        sw_rel = (
            (L > (L_med + 75.0))
            & (s < min(35.0, s_med + 8.0))
            & (v > (v_med + 55.0))
        )
        sw_abs = (L > 240.0) & (s < 35.0) & (v > 230.0)
        sw = (sw_rel | sw_abs).astype(np.uint8) * 255
        sw = cv2.morphologyEx(sw, cv2.MORPH_OPEN, self.kernel, iterations=1)
        sw = cv2.morphologyEx(sw, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        sw_bin = (sw > 0).astype(np.float32)

        if xl is None or xr is None or not np.isfinite(xl) or not np.isfinite(xr):
            sw_center, sw_bias, sw_near = 0.0, 0.0, 0.0
        else:
            xl_i = int(np.clip(xl, 0, rW - 1))
            xr_i = int(np.clip(xr, 0, rW - 1))
            if xl_i > xr_i:
                xl_i, xr_i = xr_i, xl_i
            # CRITICAL: Use TIGHTER boundaries - minimal padding to prevent sidewalk encroachment
            # Reduced padding from 3% to 1% for tighter lane definition
            pad = int(0.01 * rW)  # Much tighter - was 0.03
            xL = int(np.clip(xl_i + pad, 0, rW - 1))
            xR = int(np.clip(xr_i - pad, 0, rW - 1))
            if xR <= xL + 10:
                sw_center, sw_bias, sw_near = 0.0, 0.0, 0.0
            else:
                band0 = int(0.45 * rH)
                near = sw_bin[band0:, :]
                out_left = float(near[:, :xL].mean()) if xL > 5 else 0.0
                out_right = float(near[:, xR:].mean()) if xR < (rW - 5) else 0.0
                denom = out_left + out_right + 1e-6
                sw_bias = clamp((out_right - out_left) / denom, -1.0, 1.0)

                lane_w = max(1, xR - xL)
                # CRITICAL: Much tighter core padding - reduced from 18% to 5% for accuracy
                # This ensures we only check the actual lane center, not extended boundaries
                core_pad = int(max(2, 0.05 * lane_w))  # Much tighter - was 0.18
                cxL = xL + core_pad
                cxR = xR - core_pad
                if cxR <= cxL + 8:
                    cxL, cxR = xL, xR
                in_lane = near[:, cxL:cxR]
                sw_center = float(in_lane.mean())
                near0 = int(0.75 * rH)
                sw_near = float(sw_bin[near0:, cxL:cxR].mean())

        dbg = cv2.cvtColor(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
        dbg[sw > 0] = (0, 0, 255)
        if xl is not None and xr is not None and np.isfinite(xl) and np.isfinite(xr):
            xl_i = int(np.clip(xl, 0, rW - 1))
            xr_i = int(np.clip(xr, 0, rW - 1))
            cv2.line(dbg, (xl_i, 0), (xl_i, rH - 1), (0, 255, 255), 2)
            cv2.line(dbg, (xr_i, 0), (xr_i, rH - 1), (0, 255, 255), 2)
        cv2.putText(dbg, f"swC={sw_center:.2f} swN={sw_near:.2f} swB={sw_bias:+.2f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return float(sw_center), float(sw_bias), float(sw_near), dbg

# ===================================================================
#  Depth / LiDAR Safety
# ===================================================================

def _depth_to_m(depth_px: np.ndarray) -> np.ndarray:
    d = depth_px.astype(np.float32, copy=False)
    sample = d[np.isfinite(d) & (d > 0)]
    if sample.size:
        med = float(np.percentile(sample, 50))
        if med > 20.0:
            d = d * 0.001
    return d

def depth_front(depth_px: np.ndarray) -> Tuple[float, float, float, bool]:
    if depth_px is None or depth_px.size == 0:
        return float("inf"), float("inf"), 0.0, False
    d = _depth_to_m(depth_px)
    H, W = d.shape[:2]
    y1 = int(0.46 * H)
    y2 = int(0.90 * H)
    rois = [
        (int(0.15 * W), int(0.42 * W)),
        (int(0.42 * W), int(0.58 * W)),
        (int(0.58 * W), int(0.85 * W)),
    ]
    mins = []
    total_valid = 0
    for x1, x2 in rois:
        win = d[y1:y2, x1:x2]
        win = win[np.isfinite(win) & (win > 0.32)]
        total_valid += int(win.size)
        mins.append(float(np.percentile(win, 8)) if win.size else float("inf"))
    left, center, right = mins
    min_front = float(min(left, center, right))
    valid = bool(np.isfinite(min_front) and (min_front > 0.32) and (min_front < 30.0)
                 and (total_valid > 1200))
    invL = 0.0 if not np.isfinite(left) else 1.0 / max(left, 0.08)
    invR = 0.0 if not np.isfinite(right) else 1.0 / max(right, 0.08)
    bias = clamp(0.35 * (invR - invL), -0.35, 0.35)
    return float(min_front), float(center), float(bias), valid

def lidar_front(angles: np.ndarray, dist: np.ndarray
                ) -> Tuple[float, float, float, float, bool]:
    if angles is None or dist is None:
        return float("inf"), float("inf"), 0.0, 0.0, False
    a = (-angles + np.pi)
    a = (a + np.pi) % (2.0 * np.pi) - np.pi
    d = dist.astype(np.float32, copy=False)
    ok = np.isfinite(d)
    f = ok & (np.abs(a) < math.radians(70.0))
    if not np.any(f):
        return float("inf"), float("inf"), 0.0, 0.0, False
    a = a[f]; d = d[f]
    x = d * np.cos(a); y = d * np.sin(a)
    f2 = (x > 0.18) & (d < 8.0)
    if not np.any(f2):
        return float("inf"), float("inf"), 0.0, 0.0, False
    a = a[f2]; y = y[f2]; d = d[f2]
    front = d[np.abs(a) < math.radians(35.0)]
    min_front = float(np.percentile(front, 5)) if front.size else float(np.min(d))
    valid = bool(np.isfinite(min_front) and min_front > 0.05)
    center = d[np.abs(a) < math.radians(12.0)]
    center_front = float(np.percentile(center, 10)) if center.size else float("inf")
    left = d[y > 0]; right = d[y < 0]
    dL = float(np.percentile(left, 10)) if left.size else float("inf")
    dR = float(np.percentile(right, 10)) if right.size else float("inf")
    invL = 0.0 if not np.isfinite(dL) else 1.0 / max(dL, 0.08)
    invR = 0.0 if not np.isfinite(dR) else 1.0 / max(dR, 0.08)
    bias = clamp(0.25 * (invR - invL), -0.30, 0.30)
    conf = 0.0
    if np.isfinite(dL) and np.isfinite(dR):
        conf = clamp(1.0 - abs(dL - dR) / max(dL + dR, 1e-3), 0.0, 1.0)
    return float(min_front), float(center_front), float(bias), float(conf), valid

# ===================================================================
#  Actuation helpers
# ===================================================================

def speed_to_throttle(speed_mps: float) -> float:
    """Map a signed speed command (m/s) to QCar throttle.

    Positive -> forward, negative -> reverse.
    """
    speed_mps = float(speed_mps)
    if abs(speed_mps) <= 1e-3:
        return 0.0
    s = abs(speed_mps)
    thr = clamp(0.05 + 0.10 * s, 0.06, 0.35)
    return -thr if speed_mps < 0.0 else thr

def render_hud(bgr: np.ndarray, title: str, lines: list[str]) -> np.ndarray:
    img = bgr.copy()
    cv2.putText(img, title, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 255), 3)
    y = 75
    for ln in lines:
        cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        y += 22
    return img

# ===================================================================
#  Waypoint map (Pure Pursuit debug) – same frame as QLabs world transform
# ===================================================================

def _world_to_pixel(
    x: float, y: float,
    x_min: float, x_max: float, y_min: float, y_max: float,
    width: int, height: int,
) -> Tuple[int, int]:
    """Map world (x,y) to image pixel (px, py). World +Y is up in image."""
    if x_max <= x_min or y_max <= y_min:
        return width // 2, height // 2
    px = int((x - x_min) / (x_max - x_min) * (width - 1))
    py = int((y_max - y) / (y_max - y_min) * (height - 1))
    return (max(0, min(width - 1, px)), max(0, min(height - 1, py)))

def render_waypoint_map(
    route_segments: list,
    active_wp: np.ndarray,
    pose_x: float,
    pose_y: float,
    pose_th: float,
    pure_pursuit,
    i_near: int,
    step_name: str,
    width: int = 520,
    height: int = 520,
    margin_m: float = 0.8,
) -> np.ndarray:
    """Top-down map in QLabs world frame: waypoints as checkpoints, car pose, lookahead.
    Use for debugging Pure Pursuit (waypoints and pose share the same 1:1 scale).
    """
    # Bounds from all segments
    all_x, all_y = [], []
    for _name, wp in route_segments:
        all_x.extend(wp[:, 0].tolist())
        all_y.extend(wp[:, 1].tolist())
    all_x.append(pose_x)
    all_y.append(pose_y)
    if pure_pursuit is not None and hasattr(pure_pursuit, "p_ref"):
        all_x.append(pure_pursuit.p_ref[0])
        all_y.append(pure_pursuit.p_ref[1])
    x_min = float(min(all_x)) - margin_m
    x_max = float(max(all_x)) + margin_m
    y_min = float(min(all_y)) - margin_m
    y_max = float(max(all_y)) + margin_m
    if x_max - x_min < 0.5:
        x_max = x_min + 0.5
    if y_max - y_min < 0.5:
        y_max = y_min + 0.5

    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (40, 40, 40)

    def pt(x: float, y: float) -> Tuple[int, int]:
        return _world_to_pixel(x, y, x_min, x_max, y_min, y_max, width, height)

    # Draw inactive segments (gray)
    for name, wp in route_segments:
        if name == step_name or wp.shape[0] < 2:
            continue
        pts = [pt(wp[i, 0], wp[i, 1]) for i in range(wp.shape[0])]
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], (80, 80, 80), 1)

    # Active segment: path line (cyan) and checkpoint dots (green)
    if active_wp.shape[0] >= 2:
        pts = [pt(active_wp[i, 0], active_wp[i, 1]) for i in range(active_wp.shape[0])]
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], (255, 255, 128), 2)
        # Checkpoints: every waypoint as a small circle; every 5th slightly larger + index
        CHECKPOINT_STEP = max(1, active_wp.shape[0] // 30)
        for i in range(active_wp.shape[0]):
            px, py = pt(active_wp[i, 0], active_wp[i, 1])
            if i % CHECKPOINT_STEP == 0:
                cv2.circle(img, (px, py), 4, (0, 255, 100), -1)
                cv2.putText(img, str(i), (px + 3, py - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 255, 200), 1)
            else:
                cv2.circle(img, (px, py), 2, (0, 200, 150), -1)

    # Lookahead point (Pure Pursuit target)
    if pure_pursuit is not None and hasattr(pure_pursuit, "p_ref"):
        lx, ly = pure_pursuit.p_ref[0], pure_pursuit.p_ref[1]
        lpx, lpy = pt(lx, ly)
        cv2.circle(img, (lpx, lpy), 8, (0, 0, 255), 2)
        cv2.putText(img, "L", (lpx + 10, lpy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Car position and heading (triangle)
    cpx, cpy = pt(pose_x, pose_y)
    L = 12
    dx = L * math.cos(pose_th)
    dy = -L * math.sin(pose_th)  # image Y down
    nose = (int(cpx + dx), int(cpy + dy))
    cv2.arrowedLine(img, (cpx, cpy), nose, (0, 255, 255), 2, tipLength=0.3)
    cv2.circle(img, (cpx, cpy), 6, (0, 255, 255), -1)

    # Title and checkpoint text on image
    n_wp = active_wp.shape[0]
    cv2.putText(img, f"Pure Pursuit map | {step_name}", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(img, f"checkpoint {i_near}/{n_wp}", (8, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
    return img

# ===================================================================
#  Main Runner
# ===================================================================

def run_scenario(
    waypoints_file: str,
    actor_number: int = 0,
    sample_rate_hz: float = 30.0,
    max_speed_mps: float = 2.6,
    debug_print: bool = True,
    use_lidar: bool = True,
    use_realsense: bool = True,
    recover_enable: bool = True,
    sign_model: str = "",
    sign_labels: str = "",
    sign_conf: float = 0.40,
    sign_device: str = "cuda",
):
    STEER_OUTPUT_SIGN = 1.0
    WAYPOINT_SCALE_M  = 0.10   # waypoints.txt is in x10 scale
    WORLD_SCALE       = 0.10   # QLabs world coords are x10 simulation scale

    # ------------------------------------------------------------------
    # Load & prepare waypoints
    # ------------------------------------------------------------------
    paths = load_waypoints_txt(waypoints_file)
    for r in ("path_to_pickup", "path_to_dropoff", "path_to_hub"):
        if r not in paths:
            raise RuntimeError(f"Missing required path '{r}' in {waypoints_file}")

    P_pick = interpolate_waypoints(paths["path_to_pickup"],  spacing=0.5)
    P_drop = interpolate_waypoints(paths["path_to_dropoff"], spacing=0.5)
    P_hub  = interpolate_waypoints(paths["path_to_hub"],     spacing=0.5)

    route_segments: list[tuple[str, np.ndarray]] = [
        ("TO_PICKUP",  np.array(P_pick, dtype=np.float64) * WAYPOINT_SCALE_M),
        ("TO_DROPOFF", np.array(P_drop, dtype=np.float64) * WAYPOINT_SCALE_M),
        ("TO_HUB",     np.array(P_hub,  dtype=np.float64) * WAYPOINT_SCALE_M),
    ]

    if debug_print:
        for name, wp in route_segments:
            print(f"[WP] {name}: {wp.shape[0]} pts")

    # ------------------------------------------------------------------
    # Perception modules
    # ------------------------------------------------------------------
    lane = LaneController()
    swg  = SidewalkGuard()

    # ------------------------------------------------------------------
    # Devices
    # ------------------------------------------------------------------
    car = QCar(readMode=1, frequency=int(sample_rate_hz))
    cam = Camera2D(cameraId="3@tcpip://localhost:18964",
                   frameWidth=820, frameHeight=410,
                   frameRate=sample_rate_hz)

    # QLabs connection for world-transform based localisation (custom TCP)
    _car_qlabs = _QLabsWorldTransform(host="localhost", port=18000,
                                       actor=int(actor_number), timeout=5.0)
    if not _car_qlabs.connected:
        raise RuntimeError("Could not connect to QLabs TCP on localhost:18000 - is the simulation running?")
    _wt_ok, _wt_loc, _wt_rot, _ = _car_qlabs.get_world_transform()
    if debug_print:
        print(f"[LOC] QLabs world-transform (custom TCP): ok={_wt_ok}  loc={_wt_loc}  rot_deg={[round(math.degrees(r),1) for r in _wt_rot]}")
    if not _wt_ok:
        raise RuntimeError("Could not read world transform from QLabs - is the simulation running?")

    rs    = QCarRealSense(mode="RGB, Depth") if use_realsense else None
    lidar = QCarLidar(numMeasurements=1000, rangingDistanceMode=2,
                      interpolationMode=0) if use_lidar else None

    _led_ctrl = _QLabsLEDDirect(host="localhost", port=18000, actor=actor_number)
    if debug_print:
        print(f"[LED] QLabs direct: connected={_led_ctrl.connected}")
    _prev_led_rgb = (-1.0, -1.0, -1.0)

    lidar_warn_t = 0.0

    # ------------------------------------------------------------------
    # Sign model (YOLO26 / Ultralytics) – traffic-light colour detection
    # Pass --sign-model path to a yolo26*.pt or trained best.pt
    # ------------------------------------------------------------------
    sign_model_obj = None
    sign_names: list[str] = []
    sign_stride = 6           # run YOLO every N frames (~5 Hz at 30 fps)
    sign_frame  = 0
    if sign_model and sign_labels:
        sign_names = [ln.strip() for ln in
                      open(sign_labels, "r", encoding="utf-8").read().splitlines()
                      if ln.strip()]
        from ultralytics import YOLO  # type: ignore
        # Explicitly set task to avoid autodetect warning from ultralytics
        sign_model_obj = YOLO(sign_model, task='detect')
        if debug_print:
            print(f"[SIGN] YOLO loaded: {sign_model}  labels={sign_names}")

    # ------------------------------------------------------------------
    # Sign-action state
    # ------------------------------------------------------------------
    STOP_SIGN_DWELL_S   = 2.0
    YIELD_SLOW_S        = 2.0
    YIELD_SPEED         = 1.4
    TL_HOLD_S           = 4.0
    TL_YELLOW_SPEED     = 1.4
    MIN_CRUISE_SPEED    = 2.1   # Minimum forward speed when not in a hard-stop situation

    stop_sign_active    = False
    stop_sign_stopped_t = 0.0
    yield_slow_until    = 0.0
    tl_red_until        = 0.0
    tl_yellow_until     = 0.0
    tl_green_until      = 0.0

    # Per-sign cooldown tracker  {sign_index: last_trigger_time}
    sign_last_trigger: Dict[int, float] = {}

    # Roundabout state
    rb_active_until = 0.0
    rb_entry_zone_until = 0.0  # When approaching roundabout (before entry)
    rb_in_roundabout = False   # True when actively traversing roundabout
    # QLabs ACC roundabout is effectively clockwise / keep-right.
    # CRITICAL: Internal sign convention: positive steer = LEFT, so keep-right is NEGATIVE.
    # KEY RULE: KEEP RIGHT AT ALL TIMES!
    RB_KEEP_RIGHT_STEER = -0.60   # Maximum right bias – override lane/path at fork (lines can’t pull left)
    RB_HOLD_S = 12.0              # hold bias longer through the roundabout (was 10.0)
    RB_ENTRY_ZONE_M = 8.0         # Start stay-right well before the fork
    RB_APPROACH_ZONE_M = 14.0     # Apply stay-right very early so fork lines don’t take over
    RB_EXIT_ZONE_M = 1.5          # Distance threshold for roundabout exit zone
    RB_WRONG_WAY_THRESHOLD = 0.80   # Wrong-way detection (~46°) – avoid false trigger on approach
    RB_WRONG_WAY_NEAR_M = 2.0      # Only trigger force-right when this close to RB
    RB_WRONG_WAY_FORCE_RIGHT_S = 2.5  # When wrong-way detected: force keep-right this long (steer, don’t just resync)
    RB_WRONG_WAY_RESYNC_COOLDOWN_S = 1.8  # Resync at most once per this interval to avoid loop
    RB_WRONG_WAY_SEVERE_RAD = 2.0   # When |heading_err| > this (~115 deg), do not force right; use pure pursuit only
    RB_WRONG_WAY_SEVERE_RESYNC_S = 0.4   # When severe: resync to waypoint ahead this often
    RB_WRONG_WAY_SEVERE_LOOKAHEAD = 14   # Resync to i_near + this so pure pursuit targets correct direction
    RB_WRONG_WAY_RIGHT_BIAS = -0.35     # Wrong-way: add to target_steer so we follow path but bias right (not fixed right)
    RB_WRONG_WAY_CTE_DRIFT_M = 0.18     # When wrong-way correcting, use this as drift threshold to switch to pure pursuit sooner
    RB_WRONG_WAY_CTE_RESYNC_M = 0.25    # When CTE above this, use smaller lookahead when resyncing so we rejoin path first
    RB_WRONG_WAY_LOOKAHEAD_SMALL = 5    # Smaller lookahead when CTE elevated (rejoin before aiming far ahead)
    RB_LANE_DISABLE_DIST = 2.5    # Disable lane following within this distance (was 1.8)
    CTE_LARGE_M = 0.55             # When CTE exceeds this, treat as "off-path" rather than "on sidewalk" (lowered from 0.8 based on logs)
    CTE_STUCK_TIMEOUT_S = 2.0      # If stopped with large CTE for this long, force recovery

    # ------------------------------------------------------------------
    # Route / segment state
    # ------------------------------------------------------------------
    segment_idx = 0
    segment_hold_until = 0.0
    SEGMENT_HOLD_S     = 2.0
    route_done         = False

    # ------------------------------------------------------------------
    # Final parking (after the last route segment)
    # ------------------------------------------------------------------
    # If PARK_GOAL_XYTH is None, we "park in the space ahead" by driving forward
    # PARK_FORWARD_M from the moment the parking state starts.
    # Otherwise set PARK_GOAL_XYTH = (x, y, yaw_rad) in 1:1 world coords.
    PARK_ENABLE = True
    PARK_GOAL_XYTH: tuple[float, float, float] | None = None
    PARK_FORWARD_M = 1.20
    PARK_LATERAL_M = 0.00
    PARK_SPEED_MPS = 0.35
    PARK_LOOKAHEAD_M = 0.35
    PARK_STOP_RADIUS_M = 0.18
    PARK_STOP_YAW_DEG = 25.0
    PARK_TIMEOUT_S = 10.0

    parking_active = False
    park_goal_x = 0.0
    park_goal_y = 0.0
    park_goal_yaw = 0.0
    park_start_t = 0.0
    park_pursuit: PurePursuitController | None = None
    park_wp: np.ndarray | None = None

    active_name, active_wp = route_segments[segment_idx]
    seg_max_idx = 0
    SEG_GOAL_RADIUS_M     = 0.60
    SEG_GOAL_RADIUS_TAIL_M = 1.2   # When at last waypoints, "close enough" to complete segment
    SEG_TAIL_WP_COUNT  = 8

    pure_pursuit = PurePursuitController(waypoints=active_wp.T, lookahead=0.6, cyclic=False)
    pure_pursuit.maxSteeringAngle = 0.42

    # Track the last waypoint index we explicitly set and provide a safe setter
    # to prevent resyncs that jump backwards or wildly during the TO_HUB segment.
    last_set_wp_index = 0
    def set_waypoint_index_safe(idx):
        nonlocal last_set_wp_index
        try:
            idx = int(idx)
            # On TO_HUB: only allow forward-progressing resyncs (no stepping back),
            # and cap overly-large forward jumps to a reasonable fraction of the path.
            if active_name == "TO_HUB":
                # Use the current PurePursuit index as a base as well as the last
                # explicitly-set index. This avoids clipping/resync logic using a
                # stale `last_set_wp_index` value 
                # which could allow a lookahead jump far ahead.
                try:
                    current_wpi = int(getattr(pure_pursuit, "wpi", 0))
                except Exception:
                    current_wpi = 0
                base_idx = max(last_set_wp_index, current_wpi)
                if idx < base_idx:
                    return
                max_jump = max(8, int(0.15 * max(1, active_wp.shape[0])))
                if idx > base_idx + max_jump:
                    idx = base_idx + max_jump
            pure_pursuit.set_waypoint_index(idx)
            last_set_wp_index = idx
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Pure Pursuit + Vision: blend waypoints with lane for steering
    # ------------------------------------------------------------------
    FOLLOW_WAYPOINTS_ONLY = False  # Use lane + path so we center in lane, not road center
    W_PURE_PURSUIT = 0.68   # Weight for waypoint steering
    W_LANE         = 0.32   # Weight for lane (vision) – stronger so we track lane center
    LANE_CONF_MIN  = 0.35   # Use lane in blend only when conf >= this
    LANE_CONF_DEPART_MIN = 0.48   # Lane-depart logic only when conf above this
    DRIFT_CTE_PURE_PURSUIT_M = 0.28   # When CTE exceeds this, use pure pursuit only (ignore lane)
    TURN_LANE_BLEND_MAX   = 0.01
    LANE_ONLY_CONF_MIN    = 0.72
    LANE_ONLY_MAX_PLAN_STEER = 0.12
    TURN_ACTIVE_STEER_ON    = 0.14
    TURN_ACTIVE_HOLD_S      = 0.70

    # ------------------------------------------------------------------
    # Guardrail / safety thresholds
    # ------------------------------------------------------------------
    STOP_FRONT_M       = 0.75
    SLOW_FRONT_M       = 1.60
    CENTER_STOP_M      = 0.55
    CENTER_SLOW_M      = 1.00
    SIDE_HARD_STOP_M   = 0.30
    SIDE_BYPASS_CLEAR_M = 1.50
    HIGH_STEER_SPEED_CAP = 2.0    # Cap when steering is really high
    MOD_STEER_SPEED_CAP  = 2.0    # Moderate steer
    OBSTACLE_STOP_CONFIRM_S = 0.22

    # Reverse recovery (unstick when wedged against curb/wall)
    # NOTE: This is intentionally conservative and gated; disable via CLI with --no-recover if needed.
    RECOVER_ENABLE          = bool(recover_enable)
    RECOVER_STUCK_CONFIRM_S = 0.80
    RECOVER_REVERSE_S       = 1.10
    RECOVER_REVERSE_SPEED   = 0.75
    RECOVER_FORWARD_S       = 0.55
    RECOVER_FORWARD_SPEED   = 0.45
    RECOVER_COOLDOWN_S      = 2.50

    LANE_LOST_SLOW_SPEED    = 0.32
    LANE_LOST_STOP_S        = 1.20
    LANE_DEPART_ERR_SLOW    = 0.45
    LANE_DEPART_ERR_STOP    = 0.70
    LANE_DEPART_STEER_MAX   = 0.38
    LANE_DEPART_ERR_LANE_ONLY = 0.90
    LANE_DEPART_RECOVER_SPEED = 0.38
    LANE_DEPART_RECOVER_CLEAR_M = 1.20
    LANE_DEPART_STOP_GRACE_S = 0.75
    WRONG_SIDE_LANE_ERR = 0.36   # |lane_err| above this and car left of lane = wrong side (lane_err < 0)
    WRONG_SIDE_FORCE_RIGHT_S = 2.5  # How long to keep forcing right after wrong-side detected
    WRONG_SIDE_STEER_RIGHT = -0.43  # Steer hard right to get back (negative = right)
    WRONG_SIDE_SPEED = 0.52        # Speed while correcting (fast enough to rejoin, not crawl)

    # EXTREME sidewalk guard – never broken by any other logic.
    # SidewalkGuard returns MEANS of a binary mask (0..1). Tune thresholds accordingly.
    # Empirically tuned from debug logs: values around 0.02–0.18 can occur on normal asphalt/edges.
    # Only treat sidewalk as "confirmed dangerous" at substantially higher mask fractions.
    SW_SLOW = 0.06        # Soft caution: small detected fraction
    SW_STOP = 0.22        # Strong: only when a large fraction is detected
    SW_NEAR_STOP = 0.18   # Near-sidewalk: slightly more sensitive than SW_STOP
    SW_STRONG_CONFIRM_S = 0.15  # Confirm quickly, but avoid spikes
    SW_PROBE_CLEAR_M = 1.10
    SW_PROBE_SPEED   = 0.12
    SW_EXTREME_THRESHOLD = 0.18  # Final guard: very strong signal forces steer away + slow (unbreakable)

    # ------------------------------------------------------------------
    # Runtime variables
    # ------------------------------------------------------------------
    steer_out = 0.0
    last_steer_smooth = 0.0
    speed_cmd = 0.0
    dt_nom    = 1.0 / float(sample_rate_hz)
    t_prev    = now()

    # GPS pose (1:1 scale) – initialised from first waypoint as fallback
    # until first GPS read succeeds.
    pose_x  = float(route_segments[0][1][0, 0])
    pose_y  = float(route_segments[0][1][0, 1])
    pose_th = math.radians(-44.7)          # spawn heading

    sidewalk_stop_streak_s    = 0.0
    sidewalk_strong_streak_s  = 0.0
    lane_lost_streak_s        = 0.0
    lane_depart_stop_streak_s = 0.0
    obstacle_stop_streak_s    = 0.0
    recover_stuck_streak_s    = 0.0
    cte_stuck_streak_s        = 0.0
    recover_reverse_until     = 0.0
    recover_forward_until     = 0.0
    recover_cooldown_until    = 0.0
    recover_reason            = ""
    recover_trigger_count     = 0   # number of recoveries this stuck episode (longer reverse if >= 2)
    failover_creep_until      = 0.0  # after repeated recoveries: allow brief cautious creep to escape curb wedge
    sw_latch_until  = 0.0
    turn_latch_until = 0.0
    turning_active   = False
    wrong_way_print_until = 0.0
    wrong_way_force_right_until = 0.0   # When wrong-way near RB: force keep-right until this time
    last_wrong_way_resync_t = -999.0    # Last time we resynced for wrong-way (cooldown)
    severe_wrong_way_active = False     # True when heading is severely wrong (~115 deg+); relax sidewalk full-stop to allow creep
    i_near = 0   # Nearest waypoint index (for HUD / waypoint map; updated in segment block)

    # IO timeout / backoff
    last_bgr     = None
    cam_fail     = 0
    car_fail     = 0
    rs_fail      = 0
    next_cam_try = 0.0
    next_car_try = 0.0
    next_rs_try  = 0.0

    def _is_timeout_exc(e: BaseException) -> bool:
        s = str(e).lower()
        return ("timed out" in s or "timeout" in s or "would block" in s
                or "connection" in s)

    START_MAGENTA_S = 1.2
    start_magenta_until = now() + START_MAGENTA_S

    if debug_print:
        print(f"[INFO] QLabs runner starting  segment={active_name}")

    # ==================================================================
    #  MAIN LOOP
    # ==================================================================
    try:
        _dbg_loop_count = 0
        while True:
            t = now()
            dt = t - t_prev
            t_prev = t
            if not np.isfinite(dt) or dt <= 0.0:
                dt = dt_nom
            

            # ----------------------------------------------------------
            # 1. Sensor reads
            # ----------------------------------------------------------

            # Car (tachometer / gyro)
            if t >= next_car_try:
                try:
                    car.read()
                    car_fail = 0
                except Exception as e:
                    if _is_timeout_exc(e):
                        car_fail += 1
                        next_car_try = t + min(1.0, 0.05 * car_fail)
                    else:
                        raise

            # Camera
            bgr = None
            if t >= next_cam_try:
                try:
                    cam.read()
                    bgr = cam.imageData
                    if bgr is not None and getattr(bgr, "size", 0):
                        last_bgr = bgr
                    cam_fail = 0
                except Exception as e:
                    if _is_timeout_exc(e):
                        cam_fail += 1
                        next_cam_try = t + min(1.5, 0.08 * cam_fail)
                        if cam_fail in (25, 60):
                            try:
                                if hasattr(cam, "terminate"):
                                    cam.terminate()
                            except Exception:
                                pass
                            try:
                                cam = Camera2D(
                                    cameraId="3@tcpip://localhost:18964",
                                    frameWidth=820, frameHeight=410,
                                    frameRate=sample_rate_hz,
                                )
                            except Exception:
                                pass
                    else:
                        raise
            if bgr is None or not getattr(bgr, "size", 0):
                bgr = last_bgr

            # RealSense depth
            depth_px = None
            if rs is not None and t >= next_rs_try:
                try:
                    rs.read_RGB()
                    rs.read_depth(dataMode="PX")
                    rs_fail = 0
                except Exception as e:
                    if _is_timeout_exc(e):
                        rs_fail += 1
                        next_rs_try = t + min(1.5, 0.08 * rs_fail)
                    else:
                        raise
                depth_px = getattr(rs, "imageBufferDepthPX", None)

            # LiDAR
            lidar_angles = None
            lidar_dist   = None
            if lidar is not None:
                try:
                    lidar.read()
                    lidar_angles = getattr(lidar, "angles", None)
                    lidar_dist   = getattr(lidar, "distances", None)
                except Exception as e:
                    if debug_print and (t - lidar_warn_t) > 2.0:
                        lidar_warn_t = t
                        print(f"[WARN] LiDAR read failed: {e}")

            # ----------------------------------------------------------
            # 2. World-Transform Pose  (primary localisation)
            # ----------------------------------------------------------
            wt_ok, wt_loc, wt_rot, _ = _car_qlabs.get_world_transform()
            if wt_ok:
                pose_x  = float(wt_loc[0]) * WORLD_SCALE
                pose_y  = float(wt_loc[1]) * WORLD_SCALE
                pose_th = float(wt_rot[2])                   # yaw in radians

            # Tachometer speed (for Stanley denominator / HUD)
            v_est = float(getattr(car, "motorTach", 0.0))
            if not np.isfinite(v_est):
                v_est = 0.0
            dr_speed = max(0.0, v_est)

            # ----------------------------------------------------------
            # 3. Segment management + proactive roundabout wrong-way prevention
            # ----------------------------------------------------------
            if not route_done:
                severe_wrong_way_active = False
                # Nearest waypoint index (monotonic)
                d2 = (active_wp[:, 0] - pose_x) ** 2 + (active_wp[:, 1] - pose_y) ** 2
                i_near = int(np.argmin(d2))
                seg_max_idx = max(seg_max_idx, i_near)
                
                # CRITICAL: Proactive wrong-way check near roundabouts.
                # When heading left of path: force keep-right STEER (don’t just resync every frame).
                # Resync at most once per cooldown to avoid resync loop.
                for rb_sign in MAP_ROUNDABOUT_SIGNS:
                    dist_to_rb = math.hypot(pose_x - rb_sign.x, pose_y - rb_sign.y)
                    if 0.8 < dist_to_rb < 10.0:
                        look_ahead_idx = min(i_near + 8, active_wp.shape[0] - 1)
                        if look_ahead_idx > i_near:
                            wp_ahead = active_wp[look_ahead_idx]
                            dx_ahead = wp_ahead[0] - pose_x
                            dy_ahead = wp_ahead[1] - pose_y
                            path_heading = math.atan2(dy_ahead, dx_ahead)
                            heading_to_path = wrap_pi(path_heading - pose_th)
                            wrong_way_left = heading_to_path < -RB_WRONG_WAY_THRESHOLD
                            wrong_way_any = abs(heading_to_path) > RB_WRONG_WAY_THRESHOLD
                            # Only trigger when close enough to RB (avoid false trigger on approach at 2.5m)
                            near_rb = dist_to_rb < RB_WRONG_WAY_NEAR_M
                            if (wrong_way_left or wrong_way_any) and near_rb:
                                cte_proxy = math.hypot(pose_x - active_wp[i_near, 0], pose_y - active_wp[i_near, 1])
                                lookahead = RB_WRONG_WAY_LOOKAHEAD_SMALL if cte_proxy > RB_WRONG_WAY_CTE_RESYNC_M else RB_WRONG_WAY_SEVERE_LOOKAHEAD
                                severe_wrong_way = abs(heading_to_path) > RB_WRONG_WAY_SEVERE_RAD
                                if severe_wrong_way:
                                    severe_wrong_way_active = True
                                    # Resync to waypoint AHEAD so pure pursuit targets correct direction (escape)
                                    resync_cooldown = RB_WRONG_WAY_SEVERE_RESYNC_S
                                    if t >= last_wrong_way_resync_t + resync_cooldown:
                                        ahead = min(i_near + lookahead, active_wp.shape[0] - 1)
                                        if ahead > i_near:
                                            set_waypoint_index_safe(ahead)
                                        else:
                                            set_waypoint_index_safe(i_near)
                                        last_wrong_way_resync_t = t
                                    wrong_way_force_right_until = max(wrong_way_force_right_until, t + 4.0)
                                    if debug_print and t > wrong_way_print_until:
                                        wrong_way_print_until = t + 5.0
                                        print(f"[NAV] Wrong-way near RB (severe) resync ahead heading_err={math.degrees(heading_to_path):.1f}° dist_to_rb={dist_to_rb:.2f}")
                                else:
                                    wrong_way_force_right_until = max(wrong_way_force_right_until, t + RB_WRONG_WAY_FORCE_RIGHT_S)
                                    # When >90° wrong, resync to waypoint ahead often (0.5s); else normal cooldown
                                    resync_cd = 0.5 if abs(heading_to_path) > 1.57 else RB_WRONG_WAY_RESYNC_COOLDOWN_S
                                    if t >= last_wrong_way_resync_t + resync_cd:
                                        if abs(heading_to_path) > 1.57:
                                            ahead = min(i_near + lookahead, active_wp.shape[0] - 1)
                                            if ahead > i_near:
                                                set_waypoint_index_safe(ahead)
                                            else:
                                                set_waypoint_index_safe(i_near)
                                        else:
                                            set_waypoint_index_safe(i_near)
                                        last_wrong_way_resync_t = t
                                    if debug_print and t > wrong_way_print_until:
                                        wrong_way_print_until = t + 2.0
                                        print(f"[NAV] Wrong-way near RB (heading LEFT)! Force right {RB_WRONG_WAY_FORCE_RIGHT_S}s "
                                              f"heading_err={math.degrees(heading_to_path):.1f}° dist_to_rb={dist_to_rb:.2f}")
                                break

                goal_x = float(active_wp[-1, 0])
                goal_y = float(active_wp[-1, 1])
                dist_to_goal = float(math.hypot(pose_x - goal_x, pose_y - goal_y))

                n_wp = active_wp.shape[0]
                # Allow a larger "close enough" radius when returning to the hub so we
                # don't drive all the way into the wall. This keeps the final pose a bit
                # further out while still counting the segment as complete.
                if active_name == "TO_HUB":
                    local_goal_radius = 1.0
                    # Require BOTH proximity AND 75% waypoint progress so the segment
                    # cannot complete instantly when dropoff and hub are physically close.
                    seg_complete = bool(
                        (dist_to_goal <= local_goal_radius)
                        and (seg_max_idx >= max(0, int(0.75 * n_wp)))
                    )
                else:
                    local_goal_radius = SEG_GOAL_RADIUS_TAIL_M if seg_max_idx >= max(0, n_wp - 2) else SEG_GOAL_RADIUS_M
                    seg_complete = bool(
                        (dist_to_goal <= local_goal_radius)
                        and (
                            (seg_max_idx >= max(0, n_wp - SEG_TAIL_WP_COUNT))
                            or (seg_max_idx >= max(0, int(0.75 * n_wp)))
                        )
                    )

                if (t >= segment_hold_until) and (seg_complete or bool(pure_pursuit.pathComplete)):
                    segment_idx += 1
                    recover_trigger_count = 0  # reset so next segment starts fresh
                    if segment_idx >= len(route_segments):
                        # Route finished → park in the space ahead (final manoeuvre)
                        route_done = True
                        if PARK_ENABLE and not parking_active:
                            parking_active = True
                            park_start_t = t

                            # Determine parking goal pose (1:1 world coords)
                            if PARK_GOAL_XYTH is None:
                                fwd = np.array([math.cos(pose_th), math.sin(pose_th)], dtype=np.float64)
                                right = np.array([math.cos(pose_th - math.pi / 2.0),
                                                  math.sin(pose_th - math.pi / 2.0)], dtype=np.float64)
                                goal_xy = (
                                    np.array([pose_x, pose_y], dtype=np.float64)
                                    + float(PARK_FORWARD_M) * fwd
                                    + float(PARK_LATERAL_M) * right
                                )
                                park_goal_x = float(goal_xy[0])
                                park_goal_y = float(goal_xy[1])
                                park_goal_yaw = float(pose_th)
                            else:
                                park_goal_x = float(PARK_GOAL_XYTH[0])
                                park_goal_y = float(PARK_GOAL_XYTH[1])
                                park_goal_yaw = float(PARK_GOAL_XYTH[2])

                            # Short 3-point path into the spot (Pure Pursuit)
                            p0 = np.array([pose_x, pose_y], dtype=np.float64)
                            p2 = np.array([park_goal_x, park_goal_y], dtype=np.float64)
                            p1 = p0 + 0.5 * (p2 - p0)
                            park_wp = np.vstack([p0, p1, p2])
                            park_pursuit = PurePursuitController(
                                waypoints=park_wp.T, lookahead=float(PARK_LOOKAHEAD_M), cyclic=False
                            )
                            park_pursuit.maxSteeringAngle = 0.38

                            # Re-point the debug map / CTE computations at the parking path
                            active_name = "PARK"
                            active_wp = park_wp
                            pure_pursuit = park_pursuit

                            if debug_print:
                                print(
                                    f"[PARK] start -> goal=({park_goal_x:+.2f},{park_goal_y:+.2f}) "
                                    f"yaw={math.degrees(park_goal_yaw):+.1f}deg"
                                )
                    else:
                        active_name, active_wp = route_segments[segment_idx]
                        seg_max_idx = 0
                        # Special handling for return-to-hub: densify waypoints and
                        # use a shorter lookahead + softer steering for higher accuracy.
                        if active_name == "TO_HUB":
                            try:
                                active_wp = interpolate_waypoints(active_wp, spacing=0.50)
                            except Exception:
                                pass
                            pure_pursuit = PurePursuitController(waypoints=active_wp.T, lookahead=0.40, cyclic=False)
                            pure_pursuit.maxSteeringAngle = 0.38
                        else:
                            pure_pursuit.updatePath(active_wp.T, cyclic=False)
                            pure_pursuit.maxSteeringAngle = 0.42
                        segment_hold_until = t + SEGMENT_HOLD_S
                        if debug_print:
                            print(f"[NAV] Segment switch -> {active_name}")

            # ----------------------------------------------------------
            # 4. Stanley steering
            # ----------------------------------------------------------
            stop_reason = ""
            target_steer = 0.0
            if route_done and not parking_active:
                step_name = "DONE"
                speed_cmd = 0.0

            elif route_done and parking_active:
                step_name = "PARK"
                # Parking controller: short Pure Pursuit path into the bay + stop on pose tolerance
                v_ctrl = max(0.20, dr_speed)
                if park_pursuit is not None:
                    target_steer = float(park_pursuit.update(
                        np.array([pose_x, pose_y], dtype=np.float64), pose_th, v_ctrl))
                else:
                    target_steer = 0.0
                target_steer = clamp(target_steer, -0.38, 0.38)
                speed_cmd = float(min(max_speed_mps, float(PARK_SPEED_MPS)))
                stop_reason = "park"

                # Completion / timeout
                dist_park = float(math.hypot(pose_x - park_goal_x, pose_y - park_goal_y))
                yaw_err = abs(wrap_pi(park_goal_yaw - pose_th))
                if (dist_park <= float(PARK_STOP_RADIUS_M)) and (yaw_err <= math.radians(float(PARK_STOP_YAW_DEG))):
                    speed_cmd = 0.0
                    parking_active = False
                    stop_reason = "park_done"
                    if debug_print:
                        print(f"[PARK] done  dist={dist_park:.2f} yaw_err={math.degrees(yaw_err):.1f}deg")
                elif (t - park_start_t) > float(PARK_TIMEOUT_S):
                    speed_cmd = 0.0
                    parking_active = False
                    stop_reason = "park_timeout"
                    if debug_print:
                        print("[PARK] timeout – stopping")

            else:
                step_name = active_name
                v_ctrl = max(0.20, dr_speed)
                target_steer = float(pure_pursuit.update(
                    np.array([pose_x, pose_y], dtype=np.float64), pose_th, v_ctrl))
                target_steer = clamp(target_steer, -0.42, 0.42)

                # Wrong-way detection: only resync if heading is truly backwards (>90°)
                # Trust Stanley path following - waypoints are correct!
                heading_err = math.atan2(
                    math.sin(pure_pursuit.th_ref - pose_th), math.cos(pure_pursuit.th_ref - pose_th))
                
                # Only resync if heading is significantly wrong (>90° = truly backwards).
                # Skip at segment end: no waypoints ahead, so resync to i_near is useless and causes a loop.
                wrong_way_threshold = 1.57  # 90° - only resync if truly backwards
                n_wp = active_wp.shape[0]
                if abs(heading_err) > wrong_way_threshold and i_near < n_wp - 2:
                    set_waypoint_index_safe(i_near)
                    if debug_print and t > wrong_way_print_until:
                        wrong_way_print_until = t + 2.0
                        wpx = float(active_wp[i_near, 0]) if i_near < len(active_wp) else float('nan')
                        wpy = float(active_wp[i_near, 1]) if i_near < len(active_wp) else float('nan')
                        msg = (f"wrong-way resync wpi->{i_near} heading_err_deg={math.degrees(heading_err):.0f} "
                               f"pose=({pose_x:.2f},{pose_y:.2f}) wpt=({wpx:.2f},{wpy:.2f})")
                        print(f"[NAV] {msg}")
                        try:
                            _dbg2("NAV", "wrong_way_resync", msg,
                                  {"i_near": int(i_near), "heading_err_deg": math.degrees(heading_err), "pose": [pose_x, pose_y], "wpt": [wpx, wpy]})
                        except Exception:
                            pass

                goal_x = float(active_wp[-1, 0])
                goal_y = float(active_wp[-1, 1])
                dist_to_goal = float(math.hypot(pose_x - goal_x, pose_y - goal_y))

                speed_cmd = float(max_speed_mps)
                if dist_to_goal < 1.60:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                if dist_to_goal < 0.90:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                # Extra slow approach when returning to hub – avoid hitting wall
                if active_name == "TO_HUB" and dist_to_goal < 4.0:
                    speed_cmd = min(speed_cmd, 0.45)

                # Only slow for sharp curves – keep at least MIN_CRUISE_SPEED
                abs_plan_steer = abs(target_steer)
                if abs_plan_steer > 0.35:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                elif abs_plan_steer > 0.28:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                # else: no cap – keep full speed for straights and gentle curves

                # Cross-track error – only slow when really off path
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(pure_pursuit.p_ref)))
                MAX_CTE_M = 0.75
                if cte > MAX_CTE_M:
                    speed_cmd = min(speed_cmd, 0.62)
                    if debug_print and t > wrong_way_print_until:
                        wrong_way_print_until = t + 2.0
                        msg = (f"Large cross-track error: {cte:.2f}m - reducing speed "
                               f"pose=({pose_x:.2f},{pose_y:.2f}) nearest_wpi={i_near}")
                        print(f"[NAV] {msg}")
                        try:
                            _dbg2("NAV", "large_cte", msg,
                                  {"cte_m": float(cte), "pose": [pose_x, pose_y], "nearest_wpi": int(i_near)})
                        except Exception:
                            pass
                elif cte > 0.40:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                elif cte > 0.30:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)

                if abs_plan_steer >= TURN_ACTIVE_STEER_ON:
                    turn_latch_until = max(turn_latch_until, t + TURN_ACTIVE_HOLD_S)
                turning_active = bool(t < turn_latch_until)

                if t < segment_hold_until:
                    speed_cmd = 0.0
                    stop_reason = "hold"

                if t < start_magenta_until:
                    speed_cmd = 0.0
                    stop_reason = "hub_wait"

            # ----------------------------------------------------------
            # 5. Map-based sign proximity checks + Roundabout zone detection
            # ----------------------------------------------------------
            # Detect roundabout zones: approach (far), entry (approaching), active (in), exit (leaving)
            # CRITICAL: Start applying keep-right bias EARLY to prevent wrong-way entry
            rb_approach_zone = False  # NEW: Far approach zone
            rb_entry_zone = False
            rb_in_zone = False
            rb_exit_zone = False
            min_dist_to_rb = float("inf")
            closest_rb_sign = None
            
            if not route_done:
                for rb_sign in MAP_ROUNDABOUT_SIGNS:
                    dist_to_rb = math.hypot(pose_x - rb_sign.x, pose_y - rb_sign.y)
                    if dist_to_rb < min_dist_to_rb:
                        min_dist_to_rb = dist_to_rb
                        closest_rb_sign = rb_sign
                
                if closest_rb_sign is not None:
                    # Approach zone: far from roundabout (3.5-4.5m away) - START KEEP-RIGHT EARLY
                    if RB_ENTRY_ZONE_M < min_dist_to_rb <= RB_APPROACH_ZONE_M:
                        rb_approach_zone = True
                        rb_entry_zone_until = max(rb_entry_zone_until, t + 4.0)  # Extend entry zone
                    # Entry zone: approaching roundabout (2.0-3.5m away)
                    elif RB_EXIT_ZONE_M < min_dist_to_rb <= RB_ENTRY_ZONE_M:
                        rb_entry_zone = True
                        rb_entry_zone_until = max(rb_entry_zone_until, t + 4.0)  # Longer hold
                    # Active zone: in roundabout (< 1.5m from center)
                    elif min_dist_to_rb <= RB_EXIT_ZONE_M:
                        rb_in_zone = True
                        rb_in_roundabout = True
                        rb_active_until = max(rb_active_until, t + RB_HOLD_S)
                    # Exit zone: leaving roundabout (1.5-2.0m, but was recently active)
                    elif min_dist_to_rb <= RB_ENTRY_ZONE_M and t < rb_active_until:
                        rb_exit_zone = True
            
            for i, sign in enumerate(ALL_MAP_SIGNS):
                facing_rad = math.radians(sign.facing_deg)
                triggered, dist = check_sign_proximity(
                    pose_x, pose_y, sign.x, sign.y, facing_rad, sign.trigger_radius)
                if not triggered:
                    continue
                # Only use signs in front and right of QCar to avoid false positives
                if not sign_in_front_and_right(pose_x, pose_y, pose_th, sign.x, sign.y, facing_rad):
                    continue
                last_t = sign_last_trigger.get(i, -999.0)
                if (t - last_t) < sign.cooldown_s:
                    continue
                sign_last_trigger[i] = t

                if sign.sign_type == "stop":
                    stop_sign_active = True
                    stop_sign_stopped_t = 0.0
                    if debug_print:
                        print(f"[MAP] STOP sign #{i} at ({sign.x:.2f},{sign.y:.2f})  dist={dist:.2f}")

                elif sign.sign_type == "yield":
                    yield_slow_until = max(yield_slow_until, t + YIELD_SLOW_S)
                    if debug_print:
                        print(f"[MAP] YIELD sign #{i} at ({sign.x:.2f},{sign.y:.2f})  dist={dist:.2f}")

                elif sign.sign_type == "roundabout":
                    rb_active_until = max(rb_active_until, t + RB_HOLD_S)
                    rb_in_roundabout = True
                    if debug_print:
                        print(f"[MAP] ROUNDABOUT sign #{i} at ({sign.x:.2f},{sign.y:.2f})  dist={dist:.2f}")
                    # Immediately check for wrong-way when roundabout sign detected
                    # Force resync if heading is wrong-way
                    if not route_done:
                        heading_err_rb = math.atan2(
                            math.sin(pure_pursuit.th_ref - pose_th), math.cos(pure_pursuit.th_ref - pose_th))
                        if abs(heading_err_rb) > RB_WRONG_WAY_THRESHOLD:
                            set_waypoint_index_safe(i_near)
                            if debug_print:
                                print(f"[MAP] ROUNDABOUT wrong-way detected! Resyncing wpi->{i_near} heading_err={math.degrees(heading_err_rb):.1f}°")

            # ----------------------------------------------------------
            # 6. Traffic-light proximity → YOLO colour detection
            # ----------------------------------------------------------
            near_tl = False
            for tl in MAP_TRAFFIC_LIGHTS:
                facing_rad = math.radians(tl.facing_deg)
                trig, _ = check_sign_proximity(
                    pose_x, pose_y, tl.x, tl.y, facing_rad, TL_DETECTION_RADIUS)
                if trig and sign_in_front_and_right(pose_x, pose_y, pose_th, tl.x, tl.y):
                    near_tl = True
                    break

            if near_tl and sign_model_obj is not None and bgr is not None and getattr(bgr, "size", 0):
                sign_frame += 1
                if (sign_frame % max(1, sign_stride)) == 0:
                    try:
                        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                        results = sign_model_obj.predict(
                            source=rgb, verbose=False,
                            conf=float(sign_conf), device=str(sign_device))
                        r0 = results[0]
                        if getattr(r0, "boxes", None) is not None and len(r0.boxes):
                            clss  = r0.boxes.cls.cpu().numpy().astype(int)
                            confs = r0.boxes.conf.cpu().numpy().astype(float)
                            for cid, sc in zip(clss, confs):
                                sname = sign_names[cid] if 0 <= cid < len(sign_names) else ""
                                if sc < float(sign_conf):
                                    continue
                                if sname == "traffic_light_red":
                                    tl_red_until = max(tl_red_until, t + TL_HOLD_S)
                                    tl_green_until = 0.0
                                    if debug_print:
                                        print(f"[TL] RED  conf={sc:.2f}")
                                elif sname == "traffic_light_yellow":
                                    tl_yellow_until = max(tl_yellow_until, t + TL_HOLD_S)
                                    if debug_print:
                                        print(f"[TL] YELLOW  conf={sc:.2f}")
                                elif sname == "traffic_light_green":
                                    tl_red_until = 0.0
                                    tl_yellow_until = 0.0
                                    tl_green_until = max(tl_green_until, t + TL_HOLD_S)
                                    if debug_print:
                                        print(f"[TL] GREEN  conf={sc:.2f}")
                    except Exception:
                        pass

            # ----------------------------------------------------------
            # 7. Apply sign / TL actions to speed
            # ----------------------------------------------------------

            # Stop sign: full stop for STOP_SIGN_DWELL_S seconds
            if stop_sign_active:
                speed_cmd = 0.0
                stop_reason = "stop_sign"
                if dr_speed < 0.05:
                    stop_sign_stopped_t += dt
                if stop_sign_stopped_t >= STOP_SIGN_DWELL_S:
                    stop_sign_active = False
                    stop_sign_stopped_t = 0.0
                    if debug_print:
                        print("[SIGN] Stop sign dwell complete – resuming")

            # Yield: slow down
            if t < yield_slow_until:
                speed_cmd = min(speed_cmd, YIELD_SPEED)
                if not stop_reason:
                    stop_reason = "yield"

            # Traffic light red → stop
            if t < tl_red_until and t >= tl_green_until:
                speed_cmd = 0.0
                stop_reason = "tl_red"

            # Traffic light yellow → slow
            if t < tl_yellow_until and t >= tl_green_until:
                speed_cmd = min(speed_cmd, TL_YELLOW_SPEED)
                if not stop_reason:
                    stop_reason = "tl_yellow"

            # ----------------------------------------------------------
            # 8. Lane detection (secondary correction)
            # ----------------------------------------------------------
            lane_steer, lane_ok, lane_conf, lane_err, lane_dbg, xl, xr, lane_y0 = \
                lane.step(bgr, dt, debug=True)

            # ----------------------------------------------------------
            # 8.5. SIDEWALK GUARD - ABSOLUTE PRIORITY - CHECK BEFORE EVERYTHING ELSE
            # ----------------------------------------------------------
            # CRITICAL: Sidewalk safety must be checked BEFORE roundabout logic
            # This prevents roundabout bias from pushing car onto sidewalk
            
            # Depth / LiDAR (needed for sidewalk guard)
            depth_min, depth_center, depth_bias, depth_valid = \
                depth_front(depth_px) if depth_px is not None else (99.0, float("inf"), 0.0, False)
            lidar_min, lidar_center, lidar_bias, _, lidar_valid = \
                lidar_front(lidar_angles, lidar_dist) \
                if (lidar_angles is not None and lidar_dist is not None) \
                else (99.0, float("inf"), 0.0, 0.0, False)

            min_front    = min(depth_min, lidar_min)
            center_front = min(depth_center, lidar_center)
            front_valid  = bool(depth_valid or lidar_valid)

            # Sidewalk guard - CRITICAL: Must prevent ANY curb/sidewalk contact
            swC, swB, swN, sw_dbg = swg.step(bgr, lane_y0, xl, xr)

            # Smart false-positive filter - only filter if VERY confident we're safe
            # This prevents false stops while maintaining safety
            # Do NOT suppress sidewalk signals – extreme guard: any curb hint must trigger
            sidewalk_strong = bool(swC > SW_STOP or swN > SW_NEAR_STOP)
            sidewalk_soft   = bool(swC > SW_SLOW or swN > 0.5 * SW_NEAR_STOP)
            if abs(swB) > 0.25 and (swC > 0.005 or swN > 0.005):
                sidewalk_soft = True
            
            # Track sidewalk state for later use
            sidewalk_detected = bool(sidewalk_strong or sidewalk_soft)
            
            if sidewalk_strong:
                sidewalk_strong_streak_s += dt
            else:
                sidewalk_strong_streak_s = 0.0
            sidewalk_confirmed = bool(sidewalk_strong and sidewalk_strong_streak_s >= SW_STRONG_CONFIRM_S)

            if sidewalk_confirmed:
                sw_latch_until = max(sw_latch_until, t + 1.5)
            if t < sw_latch_until:
                sidewalk_soft = True
                sidewalk_detected = True

            # Special-case: suppress sidewalk detection during close return-to-hub approach.
            # False positives near the hub's curbs often trigger repeated cte_recover/sidewalk
            # handling and cause oscillatory reverse/recover loops. Ignore sidewalk signals
            # when on the TO_HUB segment and reasonably close to the goal to prioritize
            # pure-pursuit completion.
            try:
                # Fully suppress sidewalk detection and responses for the entire
                # return-to-hub segment. This avoids repeated false-positive curb
                # detections that currently cause oscillatory recoveries and
                # heading flips when finishing the route.
                if active_name == "TO_HUB":
                    sidewalk_strong = False
                    sidewalk_soft = False
                    sidewalk_detected = False
                    sidewalk_confirmed = False
                    swC = 0.0
                    swN = 0.0
                    swB = 0.0
                    sw_latch_until = t  # clear any latching so it won't re-enable
            except Exception:
                pass

            # ----------------------------------------------------------
            # 9. Steering mixer: Stanley + Lane + Roundabout bias
            # ----------------------------------------------------------
            # CRITICAL: In roundabouts, disable lane following - use Stanley path only
            # Include wrong-way force-right period so we actually steer right instead of resync loop
            in_roundabout_zone = bool(
                rb_in_roundabout or rb_entry_zone or rb_exit_zone or rb_approach_zone
                or t < rb_active_until or t < wrong_way_force_right_until
            )
            
            # Drift: when CTE is high, use pure pursuit only (ignore lane) so we rejoin the path
            wrong_way_correcting_steer = (t < wrong_way_force_right_until)
            drift_cte_m = RB_WRONG_WAY_CTE_DRIFT_M if wrong_way_correcting_steer else DRIFT_CTE_PURE_PURSUIT_M
            drifted = (not route_done) and (cte >= drift_cte_m)
            # When drifted, do NOT use roundabout/wrong-way steering – follow pure pursuit to rejoin path
            # When FOLLOW_WAYPOINTS_ONLY: always follow green line – no roundabout/lane steer override
            # On TO_HUB we follow waypoints back to hub – do not apply roundabout keep-right or sidewalk_rb
            in_roundabout_zone_steer = (
                (not FOLLOW_WAYPOINTS_ONLY) and in_roundabout_zone and not drifted
                and active_name != "TO_HUB"
            )

            # Get distance to closest roundabout for bias calculation
            min_dist_to_rb_steer = float("inf")
            if in_roundabout_zone_steer:
                for rb_sign in MAP_ROUNDABOUT_SIGNS:
                    dist_to_rb = math.hypot(pose_x - rb_sign.x, pose_y - rb_sign.y)
                    if dist_to_rb < min_dist_to_rb_steer:
                        min_dist_to_rb_steer = dist_to_rb
            
            if in_roundabout_zone_steer:
                # ROUNDABOUT MODE: wrong-way -> follow path + right bias to avoid large CTE; else fixed keep-right.
                if wrong_way_correcting_steer:
                    steer_raw = clamp(target_steer + RB_WRONG_WAY_RIGHT_BIAS, -0.60, 0.25)
                else:
                    steer_raw = RB_KEEP_RIGHT_STEER
                
                # CRITICAL: Only apply roundabout bias if NO sidewalk is detected
                # Sidewalk safety takes absolute priority over roundabout navigation
                if not sidewalk_detected:
                    # Pure keep-right – no blend with path/lane so fork lines can’t mess anything up
                    if not wrong_way_correcting_steer:
                        steer_raw = RB_KEEP_RIGHT_STEER
                    
                    # Reduce speed in roundabouts only when really needed (allow faster through)
                    if not stop_reason:
                        if rb_in_zone:
                            speed_cmd = min(speed_cmd, 0.95)  # In roundabout – faster
                        elif rb_entry_zone:
                            speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                        elif rb_approach_zone:
                            speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                else:
                    # Sidewalk detected in roundabout: avoid curb but NEVER follow lines left
                    if swC > 0.05 or swN > 0.04:  # Very high detection
                        speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                    else:
                        speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                    if not stop_reason:
                        stop_reason = "sidewalk_rb"
                    # Heavy keep-right blend – lines/sidewalk must not override
                    if swB > 0.15:  # Sidewalk on right – tiny ease left, still strong right
                        steer_raw = clamp(0.15 * 0.25 + 0.85 * RB_KEEP_RIGHT_STEER, -0.60, 0.08)
                    elif swB < -0.15:  # Sidewalk on left – full keep-right
                        steer_raw = RB_KEEP_RIGHT_STEER
                    else:
                        steer_raw = clamp(0.2 * swB + 0.8 * RB_KEEP_RIGHT_STEER, -0.60, 0.25)
                
                rb_bias_applied = True
            else:
                # NORMAL MODE: Pure Pursuit (green line) or blend with lane
                if FOLLOW_WAYPOINTS_ONLY:
                    steer_raw = clamp(target_steer, -0.42, 0.42)
                elif drifted:
                    steer_raw = clamp(target_steer, -0.40, 0.40)
                elif lane_ok and lane_conf >= LANE_CONF_MIN:
                    lane_w = W_LANE
                    pp_w = W_PURE_PURSUIT
                    if turning_active or abs(target_steer) >= TURN_ACTIVE_STEER_ON:
                        lane_w = TURN_LANE_BLEND_MAX
                        pp_w = 1.0 - lane_w
                    steer_raw = clamp(
                        pp_w * target_steer + lane_w * lane_steer,
                        -0.45, 0.45)
                    if (abs(lane_err) > LANE_DEPART_ERR_LANE_ONLY
                            and lane_conf >= LANE_ONLY_CONF_MIN
                            and abs(target_steer) <= LANE_ONLY_MAX_PLAN_STEER
                            and not turning_active):
                        steer_raw = clamp(lane_steer, -0.34, 0.34)
                else:
                    steer_raw = clamp(target_steer, -0.40, 0.40)
                rb_bias_applied = False

            # Update turning latch from final steer
            if max(abs(steer_raw), abs(target_steer)) >= TURN_ACTIVE_STEER_ON:
                turn_latch_until = max(turn_latch_until, t + TURN_ACTIVE_HOLD_S)
            # Roundabouts are always considered "turning active" to reduce lane blend
            turning_active = bool(t < turn_latch_until or in_roundabout_zone)

            # ----------------------------------------------------------
            # 10. Safety overrides (sidewalk guard already checked above)
            # ----------------------------------------------------------

            obs_bias_terms = []
            if depth_valid:
                obs_bias_terms.append(float(depth_bias))
            if lidar_valid:
                obs_bias_terms.append(float(lidar_bias))
            obs_bias = float(sum(obs_bias_terms) / len(obs_bias_terms)) if obs_bias_terms else 0.0

            # ============================================================
            # SIDEWALK GUARD – EXTREME: never broken by roundabout, lane, or any other logic
            # During wrong-way correction (wrong_way_force_right_until): never steer LEFT.
            # ============================================================
            in_recovery = bool(t < recover_reverse_until or t < recover_forward_until)
            wrong_way_correcting = bool(t < wrong_way_force_right_until)
            in_failover_creep = bool(t < failover_creep_until)
            
            
            if sidewalk_confirmed and not in_recovery:
                # If we've entered the failover creep latch (after exhausting rock recoveries),
                # keep commanding a cautious creep (unless an obstacle/other hard stop already forced 0).
                if in_failover_creep:
                    if speed_cmd <= 1e-6:
                        speed_cmd = SW_PROBE_SPEED
                    stop_reason = stop_reason or "sidewalk_cte_failover"
                    steer_raw = clamp(0.6 * steer_raw + 0.4 * target_steer, -0.45, 0.45)
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(pure_pursuit.p_ref)))
                if swC > SW_STOP or swN > SW_NEAR_STOP:
                    if cte > 0.25:
                        # When CTE is large, treat as "off-path" rather than "on sidewalk" - allow recovery
                        if cte > CTE_LARGE_M:
                            # Large CTE: allow creep to rejoin path instead of full stop
                            # Force minimum speed (don't let segment hold or other stops override recovery)
                                speed_cmd = max(min(speed_cmd, 0.15), 0.15)
                                sidewalk_stop_streak_s = 0.0
                                # Don't force a CTE-based recovery while returning to hub - prefer
                                # completing the pure-pursuit path instead of toggling recoveries.
                                if active_name != "TO_HUB":
                                    stop_reason = "cte_recover"  # Override any previous stop reason
                                # Near the hub (final segment), sidewalk contrast around the right curb can
                                # incorrectly push us left when we actually need to turn right to finish.
                                # In that specific case, trust the path (pure pursuit) and ignore swB bias.
                                if active_name == "TO_HUB":
                                    steer_raw = clamp(target_steer, -0.45, 0.45)
                                else:
                                    steer_raw = clamp(target_steer + 0.30 * swB, -0.45, 0.45)
                        # When wrong-way correcting near RB, allow creep so we can escape (sidewalk can be curb in roundabout)
                        elif wrong_way_correcting:
                            speed_cmd = min(speed_cmd, 0.22)
                            sidewalk_stop_streak_s = 0.0
                            if not stop_reason:
                                stop_reason = "sidewalk_caution"
                            steer_raw = max(steer_raw, -0.40)
                        # When severely wrong-way + drifted, allow creep so pure pursuit can steer us away
                        elif severe_wrong_way_active and drifted:
                            speed_cmd = min(speed_cmd, 0.12)
                            sidewalk_stop_streak_s = 0.0
                            if not stop_reason:
                                stop_reason = "sidewalk_caution"
                            steer_raw = clamp(target_steer + 0.35 * swB, -0.45, 0.45)
                        else:
                            # If forward space exists and lane fit is strong, avoid hard-stopping on curb-color hints.
                            # This prevents deadlock where sidewalk detection remains "confirmed" but rock recovery can't escape.
                            _front_clear_for_creep = bool((center_front > 0.90) and (min_front > 0.55))
                            _lane_strong = bool(lane_ok and (lane_conf is None or lane_conf >= 0.90))
                            if _front_clear_for_creep and _lane_strong:
                                speed_cmd = min(max(speed_cmd, SW_PROBE_SPEED), SW_PROBE_SPEED)
                                sidewalk_stop_streak_s = 0.0
                                stop_reason = "sidewalk_caution"
                                steer_raw = clamp(0.6 * target_steer + 0.4 * (target_steer + 0.35 * swB), -0.45, 0.45)
                            else:
                                speed_cmd = 0.0
                                sidewalk_stop_streak_s += dt
                                if not stop_reason:
                                    stop_reason = "sidewalk_stop"

                            # Failover: if we've already tried rock recovery many times and are still in a long sidewalk_stop,
                            # allow a cautious creep using path steering even when center_front is below the normal probe threshold.
                            if (recover_trigger_count >= 8
                                    and sidewalk_stop_streak_s > 10.0
                                    and center_front > 0.65
                                    and lane_ok
                                    and (lane_conf is None or lane_conf >= 0.9)):
                                speed_cmd = SW_PROBE_SPEED
                                stop_reason = "sidewalk_cte_failover"
                                steer_raw = clamp(0.6 * steer_raw + 0.4 * target_steer, -0.45, 0.45)
                                sidewalk_stop_streak_s = 0.0
                                failover_creep_until = max(failover_creep_until, t + 2.0)
                                _dbg2("H2", "qcar2_detailed_scenario_runner.py:sidewalk_failover_creep",
                                      "sidewalk_failover_creep",
                                      {
                                          "cte": float(cte),
                                          "speed_cmd": float(speed_cmd),
                                          "swC": float(swC),
                                          "swN": float(swN),
                                          "swB": float(swB),
                                          "center_front": float(center_front),
                                          "min_front": float(min_front),
                                          "recover_trigger_count": int(recover_trigger_count),
                                      })
                        if not severe_wrong_way_active:
                            if not wrong_way_correcting:
                                if swB > 0.10:
                                    steer_raw = 0.45
                                elif swB < -0.10:
                                    steer_raw = -0.45
                                else:
                                    steer_raw = clamp(0.6 * swB, -0.45, 0.45)
                            elif swB < -0.10:
                                steer_raw = -0.45
                            # else wrong_way_correcting and swB >= 0: keep keep-right
                        if center_front > 1.20 and sidewalk_stop_streak_s > 0.25:
                            speed_cmd = SW_PROBE_SPEED
                            if not stop_reason:
                                stop_reason = "sidewalk_recover"
                            if not wrong_way_correcting:
                                # When CTE is large, prioritize path-following to rejoin path
                                if cte > CTE_LARGE_M:
                                    steer_raw = clamp(0.7 * target_steer + 0.3 * steer_raw, -0.45, 0.45)
                                else:
                                    steer_raw = clamp(0.5 * steer_raw + 0.5 * target_steer, -0.45, 0.45)
                    else:
                        speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                        sidewalk_stop_streak_s = 0.0
                        if not stop_reason:
                            stop_reason = "sidewalk_caution"
                        if wrong_way_correcting and swB > 0:
                            steer_raw = max(steer_raw, -0.35)
                        else:
                            steer_raw = clamp(target_steer + 0.40 * swB, -0.45, 0.45)
                else:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                    sidewalk_stop_streak_s = 0.0
                    if not stop_reason:
                        stop_reason = "sidewalk_caution"
                    if wrong_way_correcting and swB > 0:
                        steer_raw = max(steer_raw, -0.35)
                    else:
                        steer_raw = clamp(target_steer + 0.35 * swB, -0.45, 0.45)
                    
            elif sidewalk_strong and not in_recovery:
                speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                sidewalk_stop_streak_s = 0.0
                if not stop_reason:
                    stop_reason = "sidewalk_strong"
                if wrong_way_correcting and swB > 0.15:
                    steer_raw = max(steer_raw, -0.35)
                elif swB > 0.15:
                    steer_raw = 0.42
                elif swB < -0.15:
                    steer_raw = -0.42
                else:
                    steer_raw = clamp(target_steer + 0.40 * swB, -0.45, 0.45)
                    
            elif sidewalk_soft and not in_recovery:
                speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                sidewalk_stop_streak_s = 0.0
                # Avoid turning a soft sidewalk detection into a hard stop when
                # we're inside/near a roundabout zone; prefer roundabout logic.
                if not stop_reason and not in_roundabout_zone:
                    stop_reason = "sidewalk_soft"
                
                # Special case: On TO_HUB segment near goal, trust path following over sidewalk detection
                # This prevents false-positive curb detection from blocking completion
                # Increased threshold to 8.0m to catch cases where car gets stuck further from hub
                if active_name == "TO_HUB" and dist_to_goal <= 8.0:
                    # Trust pure pursuit path - ignore sidewalk bias when close to hub
                    steer_raw = clamp(target_steer, -0.45, 0.45)
                elif wrong_way_correcting and swB > 0:
                    steer_raw = max(steer_raw, -0.35)
                else:
                    steer_raw = clamp(target_steer + 0.35 * swB, -0.45, 0.45)
            else:
                sidewalk_stop_streak_s = 0.0

            # Timeout-based recovery for stuck state with large CTE
            # NOTE: Disabled while on TO_HUB to avoid triggering recoveries during final approach.
            if not route_done:
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(pure_pursuit.p_ref)))
                if active_name == "TO_HUB":
                    cte_stuck_streak_s = 0.0
                else:
                    if cte > CTE_LARGE_M and speed_cmd < 0.1:
                        cte_stuck_streak_s += dt
                        if cte_stuck_streak_s > CTE_STUCK_TIMEOUT_S:
                            # Force recovery: creep forward to rejoin path
                            # Override any stop reason and ensure minimum speed
                            speed_cmd = max(min(speed_cmd, 0.20), 0.20)
                            stop_reason = "cte_stuck_recover"  # Override any previous stop reason
                            steer_raw = clamp(target_steer, -0.40, 0.40)
                    else:
                        cte_stuck_streak_s = 0.0

            # Lane-lost guardrail - TRUST PATH FOLLOWING, don't stop for lane loss
            # Waypoints are correct - if lane detection fails, trust Stanley
            if lane_ok:
                lane_lost_streak_s = 0.0
            else:
                lane_lost_streak_s += dt

            # Lane lost - TRUST PATH FOLLOWING, don't stop unnecessarily
            # Only slow down if lane lost AND we're significantly off path AND front blocked
            # Otherwise trust Stanley path following - keep moving!
            if not turning_active and not lane_ok and not in_roundabout_zone:
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(pure_pursuit.p_ref)))
                # Only worry if VERY significantly off path AND front blocked
                if cte > 0.50 and front_valid and min_front < SLOW_FRONT_M:  # Increased threshold
                    speed_cmd = min(speed_cmd, LANE_LOST_SLOW_SPEED)
                    if not stop_reason:
                        stop_reason = "lane_lost"
                    if not FOLLOW_WAYPOINTS_ONLY:
                        steer_raw = clamp(target_steer + 0.20 * swB + 0.30 * obs_bias, -0.35, 0.35)
                    # NEVER hard stop for lane loss - just slow down
                    # Trust path following will get us back on track
                else:
                    # Trust path following - lane detection is just a helper
                    lane_lost_streak_s = 0.0

            # Lane departure guardrail - only when vision is good; else trust waypoints
            # When FOLLOW_WAYPOINTS_ONLY or drifted, skip so we follow path
            if (not FOLLOW_WAYPOINTS_ONLY and not drifted and not turning_active and lane_ok
                    and lane_conf >= LANE_CONF_DEPART_MIN and not in_roundabout_zone):
                abs_lane_err = abs(float(lane_err))
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(pure_pursuit.p_ref)))
                
                # Only worry about lane departure if we're also off path
                # If following path correctly, lane error is just visual
                if abs_lane_err >= LANE_DEPART_ERR_STOP and cte > 0.30:
                    lane_depart_stop_streak_s += dt
                    # Blend lane correction with path following
                    steer_raw = clamp(0.4 * lane_steer + 0.6 * target_steer, -LANE_DEPART_STEER_MAX, LANE_DEPART_STEER_MAX)
                    if front_valid and center_front > LANE_DEPART_RECOVER_CLEAR_M:
                        speed_cmd = min(speed_cmd, LANE_DEPART_RECOVER_SPEED)
                        if not stop_reason:
                            stop_reason = "lane_depart_recover"
                    elif lane_depart_stop_streak_s <= LANE_DEPART_STOP_GRACE_S:
                        speed_cmd = 0.0
                        stop_reason = "lane_depart_stop"
                    else:
                        speed_cmd = min(speed_cmd, 0.36)
                        if not stop_reason:
                            stop_reason = "lane_depart_creep"
                elif abs_lane_err >= LANE_DEPART_ERR_SLOW and cte > 0.20:
                    lane_depart_stop_streak_s = 0.0
                    speed_cmd = min(speed_cmd, 0.70)
                    if not stop_reason:
                        stop_reason = "lane_depart_slow"
                    # Blend lane with path
                    steer_raw = clamp(0.3 * lane_steer + 0.7 * target_steer, -0.40, 0.40)
                else:
                    lane_depart_stop_streak_s = 0.0
            else:
                lane_depart_stop_streak_s = 0.0

            # Side-obstacle bypass
            side_obstacle_only = bool(
                front_valid and lane_ok and lane_conf > 0.70
                and center_front > SIDE_BYPASS_CLEAR_M and min_front < 0.55)
            if side_obstacle_only:
                speed_cmd = min(speed_cmd, 0.55)
                steer_raw = clamp(steer_raw + 0.22 * obs_bias, -0.35, 0.35)
                if not stop_reason:
                    stop_reason = "side_bypass"

            # Hard obstacle stop
            hard_stop_raw = False
            if front_valid:
                both = bool(depth_valid and lidar_valid)
                if both:
                    hard_stop_raw = bool(
                        center_front < CENTER_STOP_M
                        or (min_front < SIDE_HARD_STOP_M and center_front < CENTER_SLOW_M))
                elif lidar_valid:
                    hard_stop_raw = bool(
                        lidar_center < CENTER_STOP_M
                        or (lidar_min < SIDE_HARD_STOP_M and lidar_center < CENTER_SLOW_M))
                elif depth_valid:
                    hard_stop_raw = bool(
                        depth_center < (CENTER_STOP_M - 0.15)
                        or (depth_min < (SIDE_HARD_STOP_M - 0.10)
                            and depth_center < (CENTER_SLOW_M - 0.15)))

            if hard_stop_raw:
                obstacle_stop_streak_s += dt
            else:
                obstacle_stop_streak_s = 0.0

            # Obstacle detection - only stop if CONFIRMED obstacle AND very close
            if front_valid and hard_stop_raw and obstacle_stop_streak_s >= OBSTACLE_STOP_CONFIRM_S:
                # Only hard stop if obstacle is VERY close in our forward path.
                # LiDAR `min_front` can be dominated by near-side curb while `center_front` remains safe.

                # Special-case: final approach to the hub. When we're on the TO_HUB segment and a solid
                # object is directly in front of us at very close range, interpret this as "arrived at hub"
                # rather than an endless obstacle stop, so the route can complete cleanly.
                # Increased threshold from 0.24m to 0.40m to stop earlier and avoid crashes.
                # Also check min_front to catch cases where center_front might be slightly higher but min_front is very close.
                _hub_arrival_blocked = bool(
                    active_name == "TO_HUB"
                    and (center_front < 0.40 or min_front < 0.30)
                    and front_valid
                    and lane_ok
                    and dist_to_goal < 1.5
                )

                _side_only_very_close = bool((min_front < 0.25) and (center_front >= CENTER_STOP_M))
                # When we're already in a pure CTE-based recovery with good lane signal (typical near the hub),
                # treat moderately-close curb returns as "side-only" and keep creeping instead of full obstacle stop.
                _cte_recover_creep = bool(
                    stop_reason in ("cte_recover", "cte_stuck_recover")
                    and lane_ok
                    and center_front > 0.30
                    and min_front > 0.20
                )

                if _hub_arrival_blocked:
                    route_done = True
                    speed_cmd = 0.0
                    stop_reason = "hub_wait"
                elif _side_only_very_close:
                    # Slow and steer away, but avoid deadlocking into obstacle-stop/recovery loops.
                    speed_cmd = min(speed_cmd, 0.12)
                    steer_raw = clamp(steer_raw + 0.22 * float(obs_bias), -0.45, 0.45)
                    if not stop_reason:
                        stop_reason = "obstacle_side"
                elif _cte_recover_creep:
                    # Keep creeping forward on the planned path while respecting a low speed cap.
                    # This allows the car to finish approaching the hub without getting stuck
                    # in oscillating obstacle/recover loops against a curb.
                    speed_cmd = min(speed_cmd, 0.15)
                    if not stop_reason or stop_reason in ("cte_recover", "cte_stuck_recover"):
                        stop_reason = "obstacle_cte_creep"
                elif center_front < 0.30 or (min_front < 0.25 and active_name != "TO_HUB"):  # Very close - real danger (but skip if TO_HUB - handled by _hub_arrival_blocked)
                    speed_cmd = 0.0
                    stop_reason = "obstacle"
                else:
                    # Slow down but keep moving
                    speed_cmd = min(speed_cmd, 0.15)
                    if not stop_reason:
                        stop_reason = "obstacle_slow"
            elif front_valid and (center_front < 0.65 or min_front < 0.45):
                # Only reduce speed when obstacle is reasonably close
                speed_cmd *= 0.80

            # High-steer speed cap – only slow on sharp steering
            abs_steer = abs(float(steer_raw))
            if abs_steer > 0.35:
                speed_cmd = min(speed_cmd, HIGH_STEER_SPEED_CAP)
            elif abs_steer > 0.25:
                speed_cmd = min(speed_cmd, MOD_STEER_SPEED_CAP)

            # Reverse recovery when wedged against curb / wall
            # Use world-transform heading vs path tangent to steer toward path (reorient, not just rock).
            if RECOVER_ENABLE:
                if t < recover_reverse_until:
                    # Back up: steer to align pose_th toward path (pure_pursuit.th_ref) so we reorient.
                    speed_cmd = -float(RECOVER_REVERSE_SPEED)
                    if recover_reason:
                        stop_reason = recover_reason
                    align_err = math.atan2(
                        math.sin(pure_pursuit.th_ref - pose_th), math.cos(pure_pursuit.th_ref - pose_th))
                    align_steer = clamp(float(align_err) * 1.0, -0.45, 0.45)
                    steer_raw = clamp(
                        align_steer + 0.35 * float(obs_bias) + 0.30 * float(swB),
                        -0.45, 0.45,
                    )
                    # Wrong-way in roundabout: bias steer right so we actually rotate (correct circulation)
                    if "wrongway_rb" in (recover_reason or ""):
                        steer_raw = max(steer_raw, 0.38)

                elif t < recover_forward_until:
                    # Forward: keep aligning to path so we drive out the right way.
                    speed_cmd = float(RECOVER_FORWARD_SPEED)
                    if recover_reason:
                        stop_reason = recover_reason
                    align_err = math.atan2(
                        math.sin(pure_pursuit.th_ref - pose_th), math.cos(pure_pursuit.th_ref - pose_th))
                    align_steer = clamp(float(align_err) * 0.9, -0.45, 0.45)
                    steer_raw = clamp(
                        align_steer + 0.25 * float(obs_bias) + 0.20 * float(swB),
                        -0.45, 0.45,
                    )

                else:
                    # Include recover-related reasons so we keep trying after sidewalk/lane_depart creep.
                    # Also include roundabout/curb stop reasons so reverse recovery can trigger when stuck at roundabout.
                    stop_reasons_never_recover = ("stop_sign", "tl_red", "hold", "hub_wait")
                    # Default values so they are always defined for downstream logic
                    blocked_reason = False
                    not_moving = bool(dr_speed < 0.05)
                    trying_to_move = bool(speed_cmd > 0.12)
                    front_blocked = bool(front_valid and (center_front < 0.70 or min_front < 0.45))  # More lenient
                    no_progress = False
                    stuck_now = False

                    if stop_reason in stop_reasons_never_recover:
                        recover_stuck_streak_s = 0.0
                    else:
                        blocked_reason = stop_reason in (
                            "obstacle", "obstacle_slow", "sidewalk_stop", "lane_lost_stop", "lane_depart_stop",
                            "sidewalk_recover", "lane_depart_recover",
                            "sidewalk_rb", "sidewalk_caution", "sidewalk_strong", "sidewalk_soft", "lane_lost",
                            "wrong_side",
                            "cte_recover", "cte_stuck_recover",
                        )

                        # Detect "no progress" even if we're commanding forward (common failure mode: wedged but still commanding).
                        # Intentionally avoid using blocked_reason here to prevent circular logic (stop_reason -> blocked_reason -> no_progress).
                        # For CTE-only recovery reasons, require FRONT blockage for "no progress" (avoid over-triggering rock on pure CTE + sidewalk hints).
                        sidewalk_can_count_for_progress = bool(
                            sidewalk_confirmed
                            and stop_reason not in ("cte_recover", "cte_stuck_recover")
                        )
                        no_progress = bool(
                            trying_to_move
                            and not_moving
                            and (front_blocked or sidewalk_can_count_for_progress)
                        )

                        # More lenient stuck detection - only if REALLY stuck / not moving.
                        stuck_now = bool(
                            not_moving
                            and (
                                ((speed_cmd <= 0.05) and blocked_reason)
                                or no_progress
                            )
                        )

                    # Check for wrong-way in roundabout - only when actually INSIDE (rb_in_zone), not at entrance.
                    # At entrance, path curves sharply and heading error can exceed 1.2 rad without being wrong-way.
                    in_roundabout = t < rb_active_until
                    actually_inside_rb = rb_in_zone  # Only true when < RB_EXIT_ZONE_M from center
                    heading_err_recover = math.atan2(
                        math.sin(pure_pursuit.th_ref - pose_th), math.cos(pure_pursuit.th_ref - pose_th))
                    # Use 90° (1.57 rad) so only truly backwards triggers; require stuck or blocked so we don't reverse on heading alone at entrance
                    wrong_way_in_rb = bool(actually_inside_rb and abs(heading_err_recover) > 1.57)
                    wrong_way_can_trigger = wrong_way_in_rb and (stuck_now or front_blocked)
                    # In roundabout: allow recovery when stuck (low cmd + not moving) even if front not blocked (e.g. wedged on curb)
                    in_rb_stuck = bool(actually_inside_rb and stuck_now)
                    can_trigger_rb_stuck = bool(actually_inside_rb and (stuck_now or no_progress) and (blocked_reason or front_blocked or bool(sidewalk_confirmed)))

                    # When outside the roundabout and NOT front‑blocked, never treat pure CTE-based stop reasons
                    # as "stuck" – this prevents repeated rock recoveries such as recover_cte_recover with clear front.
                    if (
                        stop_reason in ("cte_recover", "cte_stuck_recover")
                        and not front_blocked
                        and not actually_inside_rb
                        and not wrong_way_can_trigger
                        and not can_trigger_rb_stuck
                    ):
                        no_progress = False
                        stuck_now = False

                    # Hard cap on how many recovery cycles we will attempt in a single contiguous stuck episode.
                    # This prevents effectively infinite rock/reverse loops when CTE-based recovery cannot make progress.
                    MAX_RECOVER_TRIGGERS = 8
                    
                    if t < recover_cooldown_until:
                        recover_stuck_streak_s = 0.0
                    else:
                        # Allow recovery even when not in RB and front isn't blocked (e.g., wedged on curb with sidewalk_confirmed/cte_recover).
                        should_attempt_recover = bool(
                            (not route_done) and (active_name != "TO_HUB")
                            and recover_trigger_count < MAX_RECOVER_TRIGGERS
                            and (
                                (stuck_now and (front_blocked or actually_inside_rb or blocked_reason or bool(sidewalk_confirmed)))
                                or wrong_way_can_trigger
                                or can_trigger_rb_stuck
                            )
                        )

                    if t < recover_cooldown_until:
                        pass
                    elif should_attempt_recover:
                        if wrong_way_can_trigger:
                            # Wrong-way inside roundabout and stuck/blocked: more aggressive recovery
                            recover_stuck_streak_s += dt * 1.5
                        else:
                            recover_stuck_streak_s += dt
                        
                        # Shorter confirm when wrong-way or roundabout-only stuck so we recover sooner
                        # Also shorten confirm when we detect "no_progress" (wedged/curb contact often oscillates speed and would never reach 0.8s).
                        no_progress_mult = 0.65 if no_progress else 1.0
                        confirm_time = RECOVER_STUCK_CONFIRM_S * (0.6 if (wrong_way_can_trigger or can_trigger_rb_stuck) else 1.0) * no_progress_mult
                        if recover_stuck_streak_s >= confirm_time:
                            recover_stuck_streak_s = 0.0
                            recover_trigger_count += 1
                            # Longer reverse after 2+ recoveries or if wrong-way in roundabout (more rotation to escape)
                            wrong_way_mult = 1.9 if wrong_way_in_rb else 1.0
                            rev_s = float(RECOVER_REVERSE_S) * (1.65 if recover_trigger_count >= 2 else 1.0) * wrong_way_mult
                            fwd_s = float(RECOVER_FORWARD_S) * (1.2 if recover_trigger_count >= 2 else 1.0)
                            recover_reverse_until = t + rev_s
                            recover_forward_until = recover_reverse_until + fwd_s
                            recover_cooldown_until = t + float(RECOVER_COOLDOWN_S)
                            base_reason = stop_reason or ("no_progress" if no_progress else "stuck")
                            recover_reason = f"recover_{base_reason}" + ("_wrongway_rb" if wrong_way_in_rb else "")
                            # So th_ref points correct circulation: resync pure_pursuit to a waypoint AHEAD on path
                            if wrong_way_in_rb and pure_pursuit is not None and active_wp.size > 0:
                                n_wp = active_wp.shape[0]
                                ahead = min(i_near + 14, n_wp - 1)
                                if ahead > i_near:
                                    set_waypoint_index_safe(ahead)
                                # Keep "force right" after recovery so we don't immediately drift wrong again
                                wrong_way_force_right_until = max(wrong_way_force_right_until, t + rev_s + fwd_s + 3.0)

                            # Suppress verbose per-attempt recover prints to avoid
                            # flooding the console near the end of a scenario.
                            pass
                    else:
                        # Don't hard-reset: decay so brief speed spikes don't prevent recovery from ever triggering.
                        recover_stuck_streak_s = max(0.0, float(recover_stuck_streak_s) - float(dt) * 0.6)
                    # Reset recovery count once we've been moving again for a while
                    if t > recover_cooldown_until + 4.0 and dr_speed > 0.08:
                        recover_trigger_count = 0

            # Enforce minimum cruise speed when moving forward (unless hard stop / cautious creep)
            _no_min_speed = (
                "stop_sign", "tl_red", "hold", "hub_wait", "obstacle", "obstacle_slow",
                "lane_depart_stop", "lane_depart_creep", "lane_depart_recover",
                "sidewalk_stop", "sidewalk_recover", "lane_lost", "park",
            )
            if speed_cmd > 1e-3 and stop_reason not in _no_min_speed and not (stop_reason or "").startswith("recover_"):
                speed_cmd = max(speed_cmd, MIN_CRUISE_SPEED)

            # ----------------------------------------------------------
            # WRONG SIDE OF ROAD – if lane says we're left of lane center, force steer right
            # (e.g. after roundabout or curve we must not stay in oncoming lane)
            # ----------------------------------------------------------
            # Wrong side: car left of lane center (lane_err < 0); steer right to rejoin
            if not in_recovery and lane_ok and lane_conf >= 0.35 and float(lane_err) < -WRONG_SIDE_LANE_ERR:
                wrong_way_force_right_until = max(wrong_way_force_right_until, t + WRONG_SIDE_FORCE_RIGHT_S)
                steer_raw = min(steer_raw, float(WRONG_SIDE_STEER_RIGHT))
                # Use correction speed so we rejoin (don't override full stop for obstacles)
                if speed_cmd > 0.08:
                    speed_cmd = max(min(speed_cmd, float(WRONG_SIDE_SPEED)), 0.42)
                if not stop_reason:
                    stop_reason = "wrong_side"

            # ----------------------------------------------------------
            # EXTREME SIDEWALK GUARD (unbreakable) – runs last, nothing can override
            # EXCEPT during wrong-way correction: never steer LEFT (would worsen wrong-way at roundabout).
            # ----------------------------------------------------------
            wrong_way_correcting = bool(t < wrong_way_force_right_until)
            if not in_recovery and (swC > SW_EXTREME_THRESHOLD or swN > SW_EXTREME_THRESHOLD):
                speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                if wrong_way_correcting:
                    if swB < -0.02:
                        steer_raw = -0.42   # Sidewalk on left → steer right (ok)
                    # else: keep current steer_raw (keep-right), do NOT steer left
                else:
                    if swB > 0.02:
                        steer_raw = 0.42   # Sidewalk on right → steer left
                    elif swB < -0.02:
                        steer_raw = -0.42  # Sidewalk on left → steer right
                    else:
                        steer_raw = 0.38 if swC >= swN else -0.38

            # ----------------------------------------------------------
            # 11. Actuate
            # ----------------------------------------------------------
            # #endregion agent log
            throttle  = speed_to_throttle(speed_cmd)
            # Low-pass on steering to damp oscillation (control + vision)
            steer_smoothed = 0.80 * last_steer_smooth + 0.20 * steer_raw
            steer_smoothed = clamp(steer_smoothed, -0.45, 0.45)
            last_steer_smooth = steer_smoothed
            # Final sidewalk enforcement (extra safety): if sidewalk is strongly
            # detected, force steering AWAY from curb and cap speed.
            try:
                if not in_recovery and (swC > SW_EXTREME_THRESHOLD or swN > SW_EXTREME_THRESHOLD):
                    if swB > 0.02:
                        steer_smoothed = max(steer_smoothed, 0.20)
                    elif swB < -0.02:
                        steer_smoothed = min(steer_smoothed, -0.20)
                    throttle = min(throttle, SW_PROBE_SPEED)
            except Exception:
                pass

            steer_out = float(STEER_OUTPUT_SIGN * steer_smoothed)

            # LED colour (segment-aware)
            if t < start_magenta_until or step_name in ("HUB_WAIT", "DONE"):
                led_arr = np.array([0, 0, 0, 0, 1, 0, 1, 1], dtype=np.float64)
                _led_rgb = (1.0, 0.0, 1.0)
            elif step_name == "TO_DROPOFF":
                led_arr = np.array([0, 0, 0, 0, 0, 1, 1, 1], dtype=np.float64)
                _led_rgb = (0.0, 0.0, 1.0)
            elif step_name in ("TO_HUB", "PARK"):
                led_arr = np.array([1, 1, 1, 1, 0, 0, 1, 1], dtype=np.float64)
                _led_rgb = (1.0, 0.5, 0.0)
            else:
                led_arr = np.array([0, 0, 0, 0, 0, 0, 1, 1], dtype=np.float64)
                _led_rgb = (0.0, 1.0, 0.0)

            car.read_write_std(throttle=throttle, steering=steer_out, LEDs=led_arr)
            if _led_ctrl.connected and _led_rgb != _prev_led_rgb:
                if _led_ctrl.set_color(*_led_rgb):
                    _prev_led_rgb = _led_rgb

            # If the route is finished and we're parked/stopped (and not in a recovery),
            # exit the main loop so the CLI can present the post-run prompt.
            if route_done and not parking_active and (dr_speed < 0.08) and not in_recovery:
                break

            # ----------------------------------------------------------
            # 12. HUD + waypoint map (Pure Pursuit debug)
            # ----------------------------------------------------------
            n_wp_hud = active_wp.shape[0]
            checkpoint_str = f"checkpoint wp={i_near}/{n_wp_hud}"
            lookahead_str = ""
            if pure_pursuit is not None and hasattr(pure_pursuit, "p_ref"):
                lx, ly = pure_pursuit.p_ref[0], pure_pursuit.p_ref[1]
                lookahead_str = f"  lookahead=({lx:+.2f},{ly:+.2f})"
            if bgr is not None and bgr.size:
                action = ("REV" if speed_cmd < -1e-3
                          else ("STOP" if speed_cmd <= 1e-3 else "GO"))
                lines = [
                    f"seg={step_name} idx={segment_idx+1}/{len(route_segments)}",
                    f"pos=({pose_x:+.3f},{pose_y:+.3f}) th={math.degrees(pose_th):+.1f}deg  wt_ok={int(wt_ok)}",
                    f"{checkpoint_str}{lookahead_str}",
                    f"lane_ok={int(lane_ok)} conf={lane_conf:.2f} lane_steer={lane_steer:+.2f} err={lane_err:+.2f}",
                    f"pure_pursuit={target_steer:+.2f}  steer_raw={steer_raw:+.2f}  sent={steer_out:+.2f}",
                    f"speed={speed_cmd:.2f}  v_tach={dr_speed:.2f}  thr={throttle:.2f}",
                    f"stop={'ON' if stop_sign_active else 'off'}({stop_sign_stopped_t:.1f}s)  yield={max(0.0,yield_slow_until-t):.1f}s",
                    f"tlR={max(0.0,tl_red_until-t):.1f}s tlY={max(0.0,tl_yellow_until-t):.1f}s tlG={max(0.0,tl_green_until-t):.1f}s  near_tl={int(near_tl)}",
                    f"rb={max(0.0,rb_active_until-t):.1f}s {'IN' if rb_in_zone else ('ENTRY' if rb_entry_zone else ('EXIT' if rb_exit_zone else ''))}",
                    f"swC={swC:.2f} swN={swN:.2f} swB={swB:+.2f}",
                    f"minF={min_front:.2f} cF={center_front:.2f}",
                    f"reason={stop_reason}",
                ]
                hud = render_hud(bgr, f"action: {action}", lines)
                cv2.imshow("sign_debug", hud)
            # Waypoint map (QLabs world frame): path + car + lookahead for Pure Pursuit debug
            wp_map = render_waypoint_map(
                route_segments, active_wp, pose_x, pose_y, pose_th,
                pure_pursuit, i_near, step_name,
            )
            cv2.imshow("waypoint_map", wp_map)
            if lane_dbg is not None:
                cv2.imshow("lane_fit_debug", lane_dbg)
            if sw_dbg is not None:
                cv2.imshow("lane_sw_debug", sw_dbg)

            cv2.waitKey(1)

            # Pacing
            sleep_dt = dt_nom - (now() - t)
            if sleep_dt > 0:
                time.sleep(sleep_dt)

    finally:
        try:
            _led_ctrl.close()
        except Exception:
            pass
        try:
            if rs is not None:
                rs.terminate()
        except Exception:
            pass
        try:
            if lidar is not None:
                lidar.terminate()
        except Exception:
            pass
        try:
            _car_qlabs.close()
        except Exception:
            pass
        try:
            car.terminate()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

# ===================================================================
#  CLI
# ===================================================================

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--waypoints", default="waypoints.txt")
    ap.add_argument("--actor", type=int, default=0)
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=2.6)
    ap.add_argument("--no-print", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--no-realsense", action="store_true")
    ap.add_argument("--no-recover", action="store_true",
                    help="Disable reverse/rock recovery when stuck.")
    ap.add_argument("--sign-model", required=True)
    ap.add_argument("--sign-labels", required=True)
    ap.add_argument("--sign-conf", type=float, default=0.40)
    ap.add_argument("--sign-device", default="cuda")
    args = ap.parse_args()

    # Loop the scenario runner and offer a simple post-run prompt.
    while True:
        run_scenario(
            waypoints_file=args.waypoints,
            actor_number=args.actor,
            sample_rate_hz=args.rate,
            max_speed_mps=args.speed,
            debug_print=(not args.no_print),
            use_lidar=(not args.no_lidar),
            use_realsense=(not args.no_realsense),
            recover_enable=(not args.no_recover),
            sign_model=args.sign_model,
            sign_labels=args.sign_labels,
            sign_conf=args.sign_conf,
            sign_device=args.sign_device,
        )

        # Scenario finished
        try:
            print("[NAV] DONE")
            ans = input("Press '2' to run again, or Ctrl+C to quit: ").strip()
        except KeyboardInterrupt:
            print()
            break

        if ans == "2":
            continue
        else:
            break

if __name__ == "__main__":
    main()
