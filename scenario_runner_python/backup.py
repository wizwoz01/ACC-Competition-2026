"""qcar2_detailed_scenario_runner.py  –  Custom TCP + Map-Known Signs

                Beach Autonomous Systems - CSULB CECS

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
import threading
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


def estimate_local_curvature(pts: np.ndarray, idx: int, step: int = 3) -> float:
    """Estimate local path curvature (rad/m) around waypoint index idx."""
    if pts is None or pts.ndim != 2 or pts.shape[0] < (2 * step + 1):
        return 0.0
    n = int(pts.shape[0])
    i0 = max(0, int(idx) - step)
    i2 = min(n - 1, int(idx) + step)
    i1 = (i0 + i2) // 2
    p0 = pts[i0]
    p1 = pts[i1]
    p2 = pts[i2]
    v1 = p1 - p0
    v2 = p2 - p1
    l1 = float(np.linalg.norm(v1))
    l2 = float(np.linalg.norm(v2))
    if l1 < 1e-4 or l2 < 1e-4:
        return 0.0
    h1 = math.atan2(float(v1[1]), float(v1[0]))
    h2 = math.atan2(float(v2[1]), float(v2[0]))
    dtheta = abs(wrap_pi(h2 - h1))
    ds = max(1e-3, l1 + l2)
    return float(dtheta / ds)

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
#  Map Data  
#  All coordinates are in pre-fs 1:1 scale.
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

TL_DETECTION_RADIUS = 0.8   # 1:1 scale – activate YOLO TL colour detection

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
    """Canny + HoughLinesP lane detection (RightLaneFollower algorithm).

    Wider ROI trapezoid for curves; dropout tolerance; persistent bounds.
    Return signature: (steer, ok, conf, err, dbg, xl, xr, y0)
    """

    def __init__(self):
        # --- ROI trapezoid (wider = sees further into curves) ---
        self.roi_top_y_ratio = 0.35
        self.roi_bottom_y_ratio = 1.00
        self.roi_top_width_ratio = 0.55
        self.roi_bottom_width_ratio = 1.02

        # --- Hough params ---
        self.hough_rho = 2
        self.hough_theta = np.pi / 180
        self.hough_thresh = 28
        self.hough_min_line_len = 22
        self.hough_max_line_gap = 40

        self.lane_width_px = 360.0

        # --- Control gains ---
        self.k_lat = 1.00
        self.k_head = 0.25
        self.kd = 0.04
        self.max_steer = 0.60

        self.prev_err = 0.0
        self.steer_smooth = 0.0

        # --- Dropout tolerance ---
        self.last_good_time = 0.0
        self.last_good_steer = 0.0

        # --- Persisted boundaries ---
        self.last_left = None
        self.last_right = None
        self.last_bound_time = 0.0
        self.max_bound_age_s = 2.0
        self.bias_right_px = 0.0
        self.target_offset_frac = 0.20
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
        if not segments:
            return None, None, 0.0
        xs, ys, wts = [], [], []
        for (x1, y1, x2, y2) in segments:
            L = math.hypot(x2 - x1, y2 - y1)
            if L < 8:
                continue
            xs.extend([x1, x2]); ys.extend([y1, y2]); wts.extend([L, L])
        if len(xs) < 4:
            return None, None, 0.0
        x = np.array(xs, dtype=np.float32)
        y = np.array(ys, dtype=np.float32)
        w = np.array(wts, dtype=np.float32)
        W_mat = np.diag(w)
        A = np.column_stack([x, np.ones_like(x)])
        try:
            sol = np.linalg.lstsq(W_mat @ A, W_mat @ y, rcond=None)[0]
            return float(sol[0]), float(sol[1]), float(np.sum(w))
        except Exception:
            return None, None, 0.0

    @staticmethod
    def _x_at_y(m: float, b: float, y: float) -> float:
        if m is None or b is None or abs(m) < 1e-6:
            return float("nan")
        return (y - b) / m

    def step(self, bgr: np.ndarray, dt: float, debug: bool = False):
        """Canny+HoughLinesP lane detection. Returns (steer, ok, conf, err, dbg, xl, xr, y0)."""
        t_now = time.time()
        if bgr is None or bgr.size == 0:
            return 0.0, False, 0.0, 0.0, None, None, None, 0

        if not np.isfinite(dt) or dt <= 0:
            dt = 1.0 / 30.0

        H, W = bgr.shape[:2]
        mid_x = 0.5 * W
        y0_roi = int(self.roi_top_y_ratio * H)

        # CLAHE + adaptive Canny inside color mask
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_clahe = clahe.apply(gray)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Wider ranges: QLabs lane markings can be slightly dim (V~160) or warm-white
        white_mask = cv2.inRange(hsv, np.array((0, 0, 150)), np.array((180, 85, 255)))
        yellow_mask = cv2.inRange(hsv, np.array((8, 45, 80)), np.array((45, 255, 255)))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)
        km = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, km, iterations=1)

        blur = cv2.GaussianBlur(gray_clahe, (5, 5), 1.2)
        try:
            med = float(np.median(blur[color_mask > 0])) if np.count_nonzero(color_mask) else float(np.median(blur))
        except Exception:
            med = float(np.median(blur))
        sigma = 0.50
        low = int(max(8, (1.0 - sigma) * med))
        high = int(min(255, (1.0 + sigma) * med))
        if low >= high:
            low = max(8, int(0.5 * high))

        edges = cv2.Canny(blur, low, high)
        edges = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations=1)

        # Lower guard: only mask edges to color regions when the mask is meaningful
        if int(np.count_nonzero(color_mask)) > 200:
            km2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            color_mask_d = cv2.dilate(color_mask, km2, iterations=2)  # wider dilation
            edges = cv2.bitwise_and(edges, edges, mask=color_mask_d)

        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)), iterations=1)

        roi_mask = self._roi_mask(H, W)
        edges_roi = cv2.bitwise_and(edges, edges, mask=roi_mask)

        lines = cv2.HoughLinesP(
            edges_roi,
            rho=self.hough_rho,
            theta=self.hough_theta,
            threshold=self.hough_thresh,
            minLineLength=self.hough_min_line_len,
            maxLineGap=self.hough_max_line_gap,
        )

        left_segs, right_segs = [], []
        if lines is not None:
            for (x1, y1, x2, y2) in lines[:, 0]:
                dx = x2 - x1
                dy = y2 - y1
                if abs(dx) < 3:
                    continue
                m = dy / dx
                if abs(m) < 0.30 or abs(m) > 6.0:
                    continue
                if m > 0 and max(x1, x2) > int(0.52 * W):
                    right_segs.append((x1, y1, x2, y2))
                elif m < 0 and min(x1, x2) < int(0.48 * W):
                    left_segs.append((x1, y1, x2, y2))

        mL, bL, wL = self._fit_line_from_segments(left_segs)
        mR, bR, wR = self._fit_line_from_segments(right_segs)

        y_bottom = float(H - 1)
        xL_bottom = self._x_at_y(mL, bL, y_bottom) if mL is not None else float("nan")
        xR_bottom = self._x_at_y(mR, bR, y_bottom) if mR is not None else float("nan")

        have_L = bool(np.isfinite(xL_bottom) and 0 <= xL_bottom <= W)
        have_R = bool(np.isfinite(xR_bottom) and 0 <= xR_bottom <= W)

        recent_ok = (t_now - self.last_bound_time) < self.max_bound_age_s
        xL_use = xL_bottom if have_L else (self.last_left if (self.last_left is not None and recent_ok) else float("nan"))
        xR_use = xR_bottom if have_R else (self.last_right if (self.last_right is not None and recent_ok) else float("nan"))
        have_xL = bool(np.isfinite(xL_use))
        have_xR = bool(np.isfinite(xR_use))

        if have_xL and have_xR and (xR_use > xL_use + 40):
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
            bias_term = self.bias_right_px if self.bias_right_px > 0.0 else (self.target_offset_frac * self.lane_width_px)
            x_center = xR_use - 0.5 * self.lane_width_px + bias_term
            mode = "R"
            conf = clamp(wR / 650.0, 0.0, 1.0)
            ang = math.atan(mR) if mR is not None else math.radians(70.0)
        elif have_xL:
            bias_term = self.bias_right_px if self.bias_right_px > 0.0 else (self.target_offset_frac * self.lane_width_px)
            x_center = xL_use + 0.5 * self.lane_width_px + bias_term
            mode = "L"
            conf = clamp(wL / 650.0, 0.0, 1.0)
            ang = math.atan(mL) if mL is not None else math.radians(110.0)
        else:
            dbg = None
            if debug:
                dbg = cv2.cvtColor(edges_roi, cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg, "LANE: LOST", (10, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return (float(self.last_good_steer), False, 0.0, float(self.prev_err),
                    dbg, self.last_left, self.last_right, y0_roi)

        x_center = float(np.clip(x_center, 0.0, W - 1.0))
        err = float((mid_x - x_center) / max(1.0, mid_x))
        derr = float((err - self.prev_err) / max(1e-3, dt))
        self.prev_err = err

        nominal = math.radians(85.0)
        head_err = clamp((nominal - ang), -0.9, 0.9)

        steer = self.k_lat * err + self.k_head * head_err + self.kd * derr
        steer = clamp(steer, -self.max_steer, self.max_steer)
        # Stronger smoothing for perfectly smooth lane steering
        self.steer_smooth = 0.92 * self.steer_smooth + 0.08 * steer
        steer = self.steer_smooth

        # Higher confidence threshold - only use lane when we're very sure
        lane_ok = bool(conf > 0.45)
        if lane_ok:
            self.last_good_time = t_now
            self.last_good_steer = steer
            if have_L:
                self.last_left = float(xL_bottom)
            if have_R:
                self.last_right = float(xR_bottom)
            self.last_bound_time = t_now
        try:
            self.last_center = float(x_center)
        except Exception:
            pass

        xl_ret = float(xL_use) if have_xL else (self.last_left if self.last_left is not None else None)
        xr_ret = float(xR_use) if have_xR else (self.last_right if self.last_right is not None else None)

        dbg = None
        if debug:
            dbg = cv2.cvtColor(edges_roi, cv2.COLOR_GRAY2BGR)

            def _draw_line(m_v, b_v, color):
                if m_v is None or b_v is None:
                    return
                y1b = int(0.35 * H)
                y2b = H - 1
                x1b = int(self._x_at_y(m_v, b_v, y1b))
                x2b = int(self._x_at_y(m_v, b_v, y2b))
                if 0 <= x1b < W and 0 <= x2b < W:
                    cv2.line(dbg, (x1b, y1b), (x2b, y2b), color, 3)

            _draw_line(mL, bL, (255, 80, 0))   # left = orange
            _draw_line(mR, bR, (0, 255, 80))   # right = green
            cv2.circle(dbg, (int(x_center), H - 6), 7, (0, 0, 255), -1)
            cv2.putText(dbg, f"conf={conf:.2f} err={err:+.2f} head={math.degrees(head_err):+.1f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.putText(dbg, f"mode={mode} steer={steer:+.2f}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        return float(steer), bool(lane_ok), float(conf), float(err), dbg, xl_ret, xr_ret, y0_roi

# ===================================================================
#  Sidewalk / Curb Guard - AGGRESSIVE WHITE DETECTION
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

        # CRITICAL: Look at bottom HALF of image to detect white sidewalk/curb
        # Use lower 50% of image for detection (where car + road ahead is)
        y_start = int(0.50 * H)
        roi = bgr[y_start:H, :]
        rH, rW = roi.shape[:2]

        # Convert to HSV for white detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # CURB DETECTION: Look for VERY bright white (sidewalk/curb only, not road markings)
        # Sidewalk in QLabs is PURE WHITE - much brighter than road
        # Use HIGH thresholds to ignore road markings and only detect actual curbs
        white_mask = cv2.inRange(hsv, np.array([0, 0, 220]), np.array([180, 30, 255]))

        # Also detect very bright areas (curb only) - HIGH threshold
        v = hsv[:, :, 2]
        bright_mask = (v > 230).astype(np.uint8) * 255

        # Combine masks - only very bright white
        sw = cv2.bitwise_or(white_mask, bright_mask)
        sw = cv2.morphologyEx(sw, cv2.MORPH_OPEN, self.kernel, iterations=1)
        sw = cv2.morphologyEx(sw, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        sw_bin = (sw > 0).astype(np.float32)

        # Calculate sidewalk metrics based on entire bottom ROI
        # Left and right side detection
        left_region = sw_bin[:, :int(0.4 * rW)]  # Left 40%
        right_region = sw_bin[:, int(0.6 * rW):]  # Right 40%
        center_region = sw_bin[:, int(0.3 * rW):int(0.7 * rW)]  # Center 40%

        sw_left = float(left_region.mean())
        sw_right = float(right_region.mean())
        sw_center = float(center_region.mean())

        # Calculate bias: positive = sidewalk on right, negative = sidewalk on left
        total = sw_left + sw_right + 1e-6
        sw_bias = clamp((sw_right - sw_left) / total, -1.0, 1.0)

        # Near detection - bottom portion where wheels are
        wheel_region = sw_bin[int(0.7 * rH):, :]
        sw_near = float(wheel_region.mean())

        # If we have lane lines, refine the detection
        if xl is not None and xr is not None and np.isfinite(xl) and np.isfinite(xr):
            xl_i = int(np.clip(xl, 0, rW - 1))
            xr_i = int(np.clip(xr, 0, rW - 1))
            if xl_i > xr_i:
                xl_i, xr_i = xr_i, xl_i

            # Check outside lane boundaries
            pad = int(0.05 * rW)
            xL = max(0, xl_i - pad)
            xR = min(rW - 1, xr_i + pad)

            if xR > xL:
                outside_left = sw_bin[:, :xL].mean() if xL > 5 else 0.0
                outside_right = sw_bin[:, xR:].mean() if xR < rW - 5 else 0.0
                total_out = outside_left + outside_right + 1e-6
                sw_bias = clamp((outside_right - outside_left) / total_out, -1.0, 1.0)

        # Create debug visualization
        dbg = cv2.cvtColor(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
        dbg[sw > 0] = (0, 0, 255)  # Red overlay for detected sidewalk

        # Draw lane boundaries if available
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
    a_raw = (-angles + np.pi)
    a = np.arctan2(np.sin(a_raw), np.cos(a_raw))
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
    # More aggressive mapping to reach higher speeds quicker while keeping
    # a capped maximum throttle for safety. Allow full throttle when needed.
    thr = clamp(0.05 + 0.22 * s, 0.06, 1.00)
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
#  LiDAR occupancy mapper
# ===================================================================

def _polar_to_xy_car(angles: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """Convert QCarLidar angles/distances to car-frame XY (x=forward, y=left)."""
    if angles is None or distances is None:
        return np.empty((0, 2), dtype=np.float32)
    a_raw = (-angles + np.pi).astype(np.float32)
    a = np.arctan2(np.sin(a_raw), np.cos(a_raw)).astype(np.float32)
    d = distances.astype(np.float32, copy=False)
    valid = np.isfinite(d) & (d > 0.01) & (d <= 8.0)
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float32)
    a = a[valid]; d = d[valid]
    return np.column_stack([d * np.cos(a), d * np.sin(a)]).astype(np.float32)


class LidarMapper:
    """World-frame occupancy grid with birdseye renderer."""

    def __init__(self, map_size_m=(12.0, 12.0), px_per_m: float = 50.0):
        self.map_size_m = (float(map_size_m[0]), float(map_size_m[1]))
        self.px_per_m   = float(px_per_m)
        self.map_w_px   = max(16, int(self.map_size_m[0] * self.px_per_m))
        self.map_h_px   = max(16, int(self.map_size_m[1] * self.px_per_m))
        self._occupancy = np.zeros((self.map_h_px, self.map_w_px), dtype=np.float32)
        self._lock      = threading.Lock()
        self.origin_world = None
        self.last_pose    = None
        self._last_raw    = None

    def _ensure_origin(self, px: float, py: float):
        if self.origin_world is None:
            self.origin_world = (px - 0.5 * self.map_size_m[0],
                                 py - 0.5 * self.map_size_m[1])

    def accumulate(self, pts_xy: np.ndarray, pose):
        if pts_xy is None or pts_xy.size == 0:
            return
        self._last_raw = pts_xy.copy()
        if pose is None and self.last_pose is None:
            return
        with self._lock:
            if pose is not None:
                self.last_pose = pose
            lp = self.last_pose
            if lp is None:
                return
            self._ensure_origin(lp[0], lp[1])
            th = float(lp[2])
            c, s = math.cos(th), math.sin(th)
            R = np.array([[c, -s], [s, c]], dtype=np.float32)
            pts_w = (R @ pts_xy.T).T + np.array([lp[0], lp[1]], dtype=np.float32)
            ox, oy = self.origin_world
            px_i = np.floor((pts_w[:, 0] - ox) * self.px_per_m).astype(np.int32)
            py_i = self.map_h_px - 1 - np.floor(
                (pts_w[:, 1] - oy) * self.px_per_m).astype(np.int32)
            ok = ((px_i >= 0) & (px_i < self.map_w_px) &
                  (py_i >= 0) & (py_i < self.map_h_px))
            for xi, yi in zip(px_i[ok], py_i[ok]):
                self._occupancy[yi, xi] = min(255.0, self._occupancy[yi, xi] + 1.0)

    def render(self) -> np.ndarray:
        with self._lock:
            occ    = self._occupancy.copy()
            origin = self.origin_world
            pose   = self.last_pose
            raw    = None if self._last_raw is None else self._last_raw.copy()

        if origin is not None and pose is not None and occ.max() > 0:
            norm = np.clip((occ / occ.max()) * 255.0, 0, 255).astype(np.uint8)
            img  = cv2.applyColorMap(norm, cv2.COLORMAP_HOT)
            ox, oy = origin
            cx = int((pose[0] - ox) * self.px_per_m)
            cy = self.map_h_px - 1 - int((pose[1] - oy) * self.px_per_m)
            if 0 <= cx < self.map_w_px and 0 <= cy < self.map_h_px:
                th = float(pose[2])
                fwd_x = int(round(cx + math.cos(th) * self.px_per_m * 0.4))
                fwd_y = int(round(cy - math.sin(th) * self.px_per_m * 0.4))
                cv2.arrowedLine(img, (cx, cy), (fwd_x, fwd_y),
                                (0, 255, 0), 2, tipLength=0.35)
                cv2.circle(img, (cx, cy),
                           max(3, int(self.px_per_m * 0.08)), (0, 220, 0), -1)
            return img

        # Fallback: car-frame scan on black canvas
        img2 = np.zeros((self.map_h_px, self.map_w_px, 3), dtype=np.uint8)
        cx, cy = self.map_w_px // 2, self.map_h_px // 2
        if raw is not None:
            for px_m, py_m in raw:
                pxi = int(round(cx + float(px_m) * self.px_per_m))
                pyi = int(round(cy - float(py_m) * self.px_per_m))
                if 0 <= pxi < self.map_w_px and 0 <= pyi < self.map_h_px:
                    img2[max(0, pyi-1):pyi+2, max(0, pxi-1):pxi+2] = (200, 200, 200)
        cv2.circle(img2, (cx, cy),
                   max(3, int(self.px_per_m * 0.08)), (0, 220, 0), -1)
        return img2


# ===================================================================
#  Main Runner
# ===================================================================

def run_scenario(
    waypoints_file: str,
    actor_number: int = 0,
    sample_rate_hz: float = 30.0,
    # Increased default max speed to help complete route faster while
    # preserving existing safety caps elsewhere in the controller.
    max_speed_mps: float = 0.6,
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

    # ------------------------------------------------------------------
    # Load & prepare waypoints
    # ------------------------------------------------------------------
    paths = load_waypoints_txt(waypoints_file)
    for r in ("path_to_pickup", "path_to_dropoff", "path_to_hub"):
        if r not in paths:
            raise RuntimeError(f"Missing required path '{r}' in {waypoints_file}")

    raw_max_abs = max(
        float(np.max(np.abs(np.asarray(paths[r], dtype=np.float64))))
        for r in ("path_to_pickup", "path_to_dropoff", "path_to_hub")
    )
    waypoint_scale_m = 0.10 if raw_max_abs > 8.0 else 1.0
    world_scale = waypoint_scale_m
    compact_track = raw_max_abs <= 8.0

    def pure_pursuit_lookahead_m(segment_name: str) -> float:
        # TIGHT LOOKAHEAD for precise tracking - less slop, more accuracy
        if segment_name == "PARK":
            return 0.18 if compact_track else 0.28
        if segment_name == "TO_HUB":
            # Very tight lookahead for precise hub return
            return 0.18 if compact_track else 0.35
        # Default: tight lookahead for all segments
        return 0.25 if compact_track else 0.50

    P_pick = interpolate_waypoints(paths["path_to_pickup"],  spacing=0.5)
    P_drop = interpolate_waypoints(paths["path_to_dropoff"], spacing=0.5)
    # Use denser hub waypoints so Pure Pursuit can hold tighter arcs.
    P_hub  = interpolate_waypoints(paths["path_to_hub"],     spacing=0.25)

    route_segments: list[tuple[str, np.ndarray]] = [
        ("TO_PICKUP",  np.array(P_pick, dtype=np.float64) * waypoint_scale_m),
        ("TO_DROPOFF", np.array(P_drop, dtype=np.float64) * waypoint_scale_m),
        ("TO_HUB",     np.array(P_hub,  dtype=np.float64) * waypoint_scale_m),
    ]

    if debug_print:
        scale_mode = "x10->meters" if waypoint_scale_m < 1.0 else "meters"
        print(f"[WP] waypoint scale mode: {scale_mode} (factor={waypoint_scale_m:.2f}, raw_max_abs={raw_max_abs:.3f})")
        print(f"[LOC] world scale factor: {world_scale:.2f}")
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
    
    # Initialize headlights to OFF immediately
    car.read_write_std(throttle=0.0, steering=0.0, LEDs=np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64))

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

    # LiDAR occupancy map (world-frame, 12 m × 12 m at 50 px/m = 600×600 px)
    _lidar_mapper = LidarMapper(map_size_m=(12.0, 12.0), px_per_m=50.0)

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
    # Reduced dwell at stop signs to speed up runs while still enforcing a
    # brief full stop. Adjust if more stopping time is required for safety.
    STOP_SIGN_DWELL_S   = 0.8
    YIELD_SLOW_S        = 2.0
    YIELD_SPEED         = 0.14
    TL_HOLD_S           = 4.0
    TL_YELLOW_SPEED     = 0.14
    # Cruise speed cap used for curves and goal approach.
    # On the compact 1/10-scale track keep this well below max_speed_mps.
    # MIN_CRUISE_SPEED    = 0.10 if compact_track else 0.52
    MIN_CRUISE_SPEED    = 0.08 
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
    route_done_reason  = ""
    HUB_MIN_ACTIVE_S = 5.0
    HUB_MIN_PROGRESS_FRAC = 0.10
    HUB_MIN_PROGRESS_WP = 8
    hub_gate_print_until = 0.0

    # ------------------------------------------------------------------
    # Final parking (after the last route segment)
    # ------------------------------------------------------------------
    # If PARK_GOAL_XYTH is None, we "park in the space ahead" by driving forward
    # PARK_FORWARD_M from the moment the parking state starts.
    # Otherwise set PARK_GOAL_XYTH = (x, y, yaw_rad) in 1:1 world coords.
    PARK_ENABLE = True
    # Set PARK_GOAL_XYTH to None to park in place (no forward movement)
    # Or set to (x, y, yaw) for specific parking goal
    PARK_GOAL_XYTH: tuple[float, float, float] | None = None
    PARK_FORWARD_M = 0.30  # Just pull forward slightly - car is already at destination
    PARK_LATERAL_M = 0.00
    PARK_SPEED_MPS = 0.03   # Very slow for gentle stop
    PARK_LOOKAHEAD_M = 0.20  # Very tight for precision
    PARK_STOP_RADIUS_M = 0.30  # Stop as soon as reasonably close
    PARK_STOP_YAW_DEG = 45.0  # Don't care about final angle
    PARK_TIMEOUT_S = 5.0    # Short timeout - should stop quickly

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
    segment_started_t = now()

    pure_pursuit = PurePursuitController(
        waypoints=active_wp.T,
        lookahead=pure_pursuit_lookahead_m(active_name),
        cyclic=False,
    )
    pure_pursuit.maxSteeringAngle = 0.50  # More steering for tighter tracking
    last_path_cte_m = 0.0
    if debug_print:
        print(f"[NAV] Pure Pursuit lookahead {active_name}={pure_pursuit.lookahead:.2f}m")

    # Track the last waypoint index we explicitly set and provide a safe setter
    # to prevent resyncs that jump backwards or wildly during the TO_HUB segment.
    last_set_wp_index = 0
    def set_waypoint_index_safe(idx):
        nonlocal last_set_wp_index
        # Hardened setter: clamp index to valid range, avoid backwards resyncs
        # during TO_HUB and limit overly-large forward jumps that send the
        # controller far ahead of the vehicle (causes driving off-map).
        try:
            idx = int(idx)
        except Exception:
            return

        # Ensure within path bounds
        n_wp = max(1, int(active_wp.shape[0]))
        if idx < 0:
            idx = 0
        if idx >= n_wp:
            idx = n_wp - 1

        # On TO_HUB be conservative: don't step backwards and limit forward jumps
        if active_name == "TO_HUB":
            try:
                current_wpi = int(getattr(pure_pursuit, "wpi", 0))
            except Exception:
                current_wpi = 0
            # Use the largest-seen progress as the base to avoid stale jumps
            base_idx = max(last_set_wp_index, current_wpi, int(seg_max_idx))
            if idx < base_idx:
                if debug_print:
                    print(f"[NAV] Ignoring backward resync {idx} < base {base_idx}")
                return
            # Tighten the allowed forward jump on TO_HUB to avoid huge lookaheads
            max_jump = max(6, int(0.10 * n_wp))
            allowed_max = min(base_idx + max_jump, n_wp - 1)
            if idx > allowed_max:
                if debug_print:
                    print(f"[NAV] Clipping resync {idx} -> {allowed_max} (base={base_idx} max_jump={max_jump} n_wp={n_wp})")
                idx = allowed_max

        # Apply to controller; only update last_set_wp_index on success
        try:
            pure_pursuit.set_waypoint_index(idx)
            last_set_wp_index = idx
        except Exception as e:
            if debug_print:
                print(f"[NAV] Failed to set waypoint index {idx}: {e}")

    # ------------------------------------------------------------------
    # Pure Pursuit ONLY: disable lane detection entirely for clean path following
    # ------------------------------------------------------------------
    FOLLOW_WAYPOINTS_ONLY = True  # Disable lane - use only waypoints
    TO_HUB_FORCE_WAYPOINTS_ONLY = True  # Return path is hand-mapped: suppress lane pull on TO_HUB.
    W_PURE_PURSUIT = 0.95   # Weight for waypoint steering - trust pure pursuit heavily
    W_LANE         = 0.05   # Weight for lane centering correction - minimal

    # Simple lookahead smoothing - balanced response
    LOOKAHEAD_SMOOTH_ALPHA = 0.85  # Smoothing factor
    LOOKAHEAD_SMOOTH_BETA = 0.15   # Response factor
    LANE_CONF_MIN  = 0.55   # Use lane in blend only when conf >= this (increased for stability)
    LANE_CONF_DEPART_MIN = 0.65   # Lane-depart logic only when conf above this (higher for stability)
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
    HIGH_STEER_SPEED_CAP = 0.2    # Cap when steering is really high
    MOD_STEER_SPEED_CAP  = 0.2    # Moderate steer
    OBSTACLE_STOP_CONFIRM_S = 0.22

    # Reverse recovery (unstick when wedged against curb/wall)
    # DISABLED: Recovery mode causes steering interference - disabled for clean driving
    RECOVER_ENABLE          = False
    RECOVER_STUCK_CONFIRM_S = 0.80
    RECOVER_REVERSE_S       = 1.10
    RECOVER_REVERSE_SPEED   = 0.075
    RECOVER_FORWARD_S       = 0.55
    RECOVER_FORWARD_SPEED   = 0.045
    RECOVER_COOLDOWN_S      = 2.50

    LANE_LOST_SLOW_SPEED    = 0.032
    LANE_LOST_STOP_S        = 1.20
    LANE_DEPART_ERR_SLOW    = 0.45
    LANE_DEPART_ERR_STOP    = 0.70
    LANE_DEPART_STEER_MAX   = 0.38
    LANE_DEPART_ERR_LANE_ONLY = 0.90
    LANE_DEPART_RECOVER_SPEED = 0.038
    LANE_DEPART_RECOVER_CLEAR_M = 1.20
    LANE_DEPART_STOP_GRACE_S = 0.75
    WRONG_SIDE_LANE_ERR = 0.36   # |lane_err| above this and car left of lane = wrong side (lane_err < 0)
    WRONG_SIDE_FORCE_RIGHT_S = 2.5  # How long to keep forcing right after wrong-side detected
    WRONG_SIDE_STEER_RIGHT = -0.43  # Steer hard right to get back (negative = right)
    WRONG_SIDE_SPEED = 0.160        # Speed while correcting (fast enough to rejoin, not crawl)

    # Sidewalk guard – only trigger when actually on/near the curb
    # Use higher thresholds to avoid false positives from road markings
    SW_SLOW = 0.15        # Only trigger for significant white (was 0.005)
    SW_STOP = 0.25        # Strong reaction needs more evidence (was 0.02)
    SW_NEAR_STOP = 0.20   # Wheels must be near actual curb (was 0.01)
    SW_STRONG_CONFIRM_S = 0.10  # Brief confirmation needed (was 0.02)
    SW_PROBE_CLEAR_M = 1.10
    SW_PROBE_SPEED   = 0.15  # Faster escape speed
    SW_EXTREME_THRESHOLD = 0.30  # Higher threshold for final enforcement (was 0.03)

    # ------------------------------------------------------------------
    # Runtime variables
    # ------------------------------------------------------------------
    steer_out = 0.0
    last_steer_smooth = 0.0
    speed_cmd = 0.0
    dt_nom    = 1.0 / float(sample_rate_hz)
    t_prev    = now()

    # Simple steering smoothing - single low-pass filter
    last_steer_smooth = 0.0

    # Waypoint update timing
    control_mode_until = 0.0  # Used for waypoint update cooldown

    # GPS pose (1:1 scale) – initialised from first waypoint as fallback
    # until first GPS read succeeds.
    pose_x  = float(route_segments[0][1][0, 0])
    pose_y  = float(route_segments[0][1][0, 1])
    pose_th = math.radians(-44.7)          # spawn heading

    # Waypoint index stability variables - initialized after pose is defined
    WP_UPDATE_MIN_DIST_M = 0.25    # Minimum distance moved before updating waypoint index
    last_wp_update_x = pose_x      # Position of last waypoint update
    last_wp_update_y = pose_y
    WP_UPDATE_COOLDOWN_S = 0.1     # Minimum time between waypoint updates

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
    last_generic_wrong_way_resync_t = -999.0
    severe_wrong_way_active = False     # True when heading is severely wrong (~115 deg+); relax sidewalk full-stop to allow creep
    i_near = 0   # Nearest waypoint index (for HUD / waypoint map; updated in segment block)

    GENERIC_WRONG_WAY_THRESHOLD = math.radians(105.0 if compact_track else 90.0)
    GENERIC_WRONG_WAY_CTE_M = 0.45 if compact_track else 0.60
    GENERIC_WRONG_WAY_RESYNC_COOLDOWN_S = 1.0 if compact_track else 0.5
    GENERIC_WRONG_WAY_START_GRACE_S = 1.0 if compact_track else 0.5
    HIGH_CTE_M = 0.60 if compact_track else 0.75
    MID_CTE_M = 0.40
    LOW_CTE_M = 0.25 if compact_track else 0.30
    HIGH_CTE_SPEED_CAP = min(float(max_speed_mps), 0.12 if compact_track else 0.62)
    MID_CTE_SPEED_CAP = min(float(max_speed_mps), 0.18 if compact_track else MIN_CRUISE_SPEED)
    LOW_CTE_SPEED_CAP = min(float(max_speed_mps), 0.22 if compact_track else MIN_CRUISE_SPEED)
    START_HUB_SIDEWALK_SUPPRESS_S = 4.0 if compact_track else 0.0
    START_HUB_SIDEWALK_RADIUS_M = 0.55 if compact_track else 0.0

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
    # DATA LOGGING FOR PAPER - ADDED FOR ACC 2026 PAPER
    # ==================================================================
    import csv
    import os

    # Create log directory with timestamp
    log_dir = "paper_logs"
    os.makedirs(log_dir, exist_ok=True)

    # Open log files
    cte_log = open(os.path.join(log_dir, "cte_data.csv"), "w", newline="")
    speed_log = open(os.path.join(log_dir, "speed_data.csv"), "w", newline="")
    steer_log = open(os.path.join(log_dir, "steer_data.csv"), "w", newline="")
    segment_log = open(os.path.join(log_dir, "segment_times.csv"), "w", newline="")
    sign_log = open(os.path.join(log_dir, "sign_events.csv"), "w", newline="")

    cte_writer = csv.writer(cte_log)
    speed_writer = csv.writer(speed_log)
    steer_writer = csv.writer(steer_log)
    segment_writer = csv.writer(segment_log)
    sign_writer = csv.writer(sign_log)

    # Write headers
    cte_writer.writerow(["time_s", "cte_m", "segment", "pose_x", "pose_y"])
    speed_writer.writerow(["time_s", "speed_cmd_mps", "dr_speed_mps", "throttle"])
    steer_writer.writerow(["time_s", "target_steer_rad", "lane_steer_rad", "steer_raw_rad", "steer_out_rad"])
    segment_writer.writerow(["segment", "start_time_s", "end_time_s", "duration_s", "max_cte_m", "avg_cte_m", "success"])
    sign_writer.writerow(["time_s", "sign_type", "sign_index", "x", "y", "dist_m"])

    # Track segment data
    segment_start_recorded = {}
    segment_cte_values = {}  # Store CTE values per segment for avg calculation
    sign_event_count = {
        "stop": 0,
        "yield": 0,
        "roundabout": 0,
        "red_light": 0,
        "yellow_light": 0,
        "green_light": 0
    }
    segment_times = {}  # Store durations per segment for mean/std later

    # Flag to track if we've recorded segment completion this loop
    segment_completed_this_frame = False

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
            
            segment_completed_this_frame = False

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
                pose_x  = float(wt_loc[0]) * world_scale
                pose_y  = float(wt_loc[1]) * world_scale
                pose_th = float(wt_rot[2])                   # yaw in radians

            # Feed LiDAR occupancy mapper (uses fresh pose from this frame)
            if lidar_angles is not None and lidar_dist is not None:
                _lidar_mapper.accumulate(
                    _polar_to_xy_car(lidar_angles, lidar_dist),
                    (pose_x, pose_y, pose_th) if wt_ok else None,
                )

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

                # Waypoint index update with stability threshold to prevent micro-jumps
                dist_since_wp_update = math.hypot(pose_x - last_wp_update_x, pose_y - last_wp_update_y)
                wp_update_ready = dist_since_wp_update >= WP_UPDATE_MIN_DIST_M and t >= control_mode_until

                # Keep Pure Pursuit index advancing on TO_HUB so it does not re-target
                # stale points that can pull the car off the return corridor.
                if active_name == "TO_HUB":
                    try:
                        current_wpi = int(getattr(pure_pursuit, "wpi", 0))
                    except Exception:
                        current_wpi = 0
                    # Only update when we've moved enough and enough time has passed
                    if i_near >= current_wpi + 2 and wp_update_ready:
                        set_waypoint_index_safe(i_near)
                        last_wp_update_x = pose_x
                        last_wp_update_y = pose_y
                        control_mode_until = t + WP_UPDATE_COOLDOWN_S
                
                # CRITICAL: Proactive wrong-way check near roundabouts.
                # When heading left of path: force keep-right STEER (don’t just resync every frame).
                # Resync at most once per cooldown to avoid resync loop.
                # Skip on TO_DROPOFF: car is EXITING the roundabout; heading away from RB is correct.
                # Skip on TO_HUB: the return path passes by all 3 roundabouts while circling the
                # track; heading mismatches vs RB approach direction are expected, not wrong-way.
                # The RB wrong-way override was driving the car into the east wall (CTE 4-6m).
                _rb_check = [] if active_name in ("TO_DROPOFF", "TO_HUB") else MAP_ROUNDABOUT_SIGNS
                for rb_sign in _rb_check:
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
                    hub_elapsed_s = max(0.0, float(t - segment_started_t))
                    hub_progress_gate = bool(
                        seg_max_idx >= max(int(HUB_MIN_PROGRESS_WP), int(HUB_MIN_PROGRESS_FRAC * n_wp))
                    )
                    hub_gate_ready = bool((hub_elapsed_s >= HUB_MIN_ACTIVE_S) and hub_progress_gate)
                    # Require BOTH proximity AND 75% waypoint progress so the segment
                    # cannot complete instantly when dropoff and hub are physically close.
                    seg_complete = bool(
                        hub_gate_ready
                        and
                        (dist_to_goal <= local_goal_radius)
                        and (seg_max_idx >= max(0, int(0.75 * n_wp)))
                    )
                else:
                    # TO_DROPOFF must reach the actual goal tightly: the path ends with a hard
                    # right turn and the loose TAIL radius lets the segment complete while the
                    # car is still on the left-wall descent, before that turn is made.
                    if active_name == "TO_DROPOFF":
                        local_goal_radius = SEG_GOAL_RADIUS_M  # always tight (0.60m), never 1.2m
                    else:
                        local_goal_radius = SEG_GOAL_RADIUS_TAIL_M if seg_max_idx >= max(0, n_wp - 2) else SEG_GOAL_RADIUS_M
                    seg_complete = bool(
                        (dist_to_goal <= local_goal_radius)
                        and (
                            (seg_max_idx >= max(0, n_wp - SEG_TAIL_WP_COUNT))
                            or (seg_max_idx >= max(0, int(0.75 * n_wp)))
                        )
                    )

                # Gate pure_pursuit.pathComplete by proximity: the controller can reach
                # pathComplete from a wrong position (e.g. after a large-CTE resync that
                # jumped wpi to near the end). Require the car to be within TAIL radius.
                # For TO_DROPOFF use the same tight 0.60m radius so the car must actually
                # arrive at the goal before the segment advances.
                _pp_done_radius = SEG_GOAL_RADIUS_M if active_name == "TO_DROPOFF" else SEG_GOAL_RADIUS_TAIL_M
                if active_name == "TO_HUB":
                    _pp_done_gate = bool((hub_elapsed_s >= HUB_MIN_ACTIVE_S) and hub_progress_gate)
                else:
                    _pp_done_gate = True
                pp_done = bool(pure_pursuit.pathComplete) and (dist_to_goal <= _pp_done_radius) and _pp_done_gate
                if (
                    debug_print
                    and active_name == "TO_HUB"
                    and (dist_to_goal <= local_goal_radius)
                    and (seg_max_idx >= max(0, int(0.75 * n_wp)))
                    and not hub_gate_ready
                    and t > hub_gate_print_until
                ):
                    hub_gate_print_until = t + 1.5
                    print(
                        f"[NAV] TO_HUB completion gated elapsed={hub_elapsed_s:.1f}s/{HUB_MIN_ACTIVE_S:.1f}s "
                        f"progress={seg_max_idx}/{n_wp} req>={max(int(HUB_MIN_PROGRESS_WP), int(HUB_MIN_PROGRESS_FRAC * n_wp))}"
                    )
                if (t >= segment_hold_until) and (seg_complete or pp_done):
                    segment_idx += 1
                    recover_trigger_count = 0  # reset so next segment starts fresh
                    # Log segment completion (added for ACC 2026 paper)
                    if not segment_completed_this_frame:
                        try:
                            end_t = t
                            start_t = segment_start_recorded.get(active_name, segment_started_t)
                            duration = end_t - start_t
                            max_cte = max(segment_cte_values.get(active_name, [0])) if segment_cte_values.get(active_name) else 0
                            avg_cte = (
                                sum(segment_cte_values.get(active_name, [0])) / len(segment_cte_values.get(active_name, [1]))
                                if segment_cte_values.get(active_name)
                                else 0
                            )
                            segment_writer.writerow([active_name, start_t, end_t, duration, max_cte, avg_cte, True])
                        except Exception:
                            pass
                        segment_times[active_name] = duration if 'duration' in locals() else 0
                        segment_completed_this_frame = True
                        # Reset for next segment
                        segment_cte_values[active_name] = []
                    if segment_idx >= len(route_segments):
                        # Route finished → park in the space ahead (final manoeuvre)
                        route_done_reason = f"segment_complete:{active_name}"
                        route_done = True
                        if debug_print:
                            print(f"[NAV] route_done reason={route_done_reason}")
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

                            # Simple 3-point straight-line parking path
                            p0 = np.array([pose_x, pose_y], dtype=np.float64)
                            p2 = np.array([park_goal_x, park_goal_y], dtype=np.float64)
                            p1 = p0 + 0.5 * (p2 - p0)  # Midpoint
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
                        segment_started_t = t
                        last_path_cte_m = 0.0
                        last_set_wp_index = 0
                        # Special handling for return-to-hub: densify waypoints and
                        # use a shorter lookahead + softer steering for higher accuracy.
                        if active_name == "TO_HUB":
                            try:
                                active_wp = interpolate_waypoints(active_wp, spacing=0.25)
                            except Exception:
                                pass
                            pure_pursuit = PurePursuitController(
                                waypoints=active_wp.T,
                                lookahead=pure_pursuit_lookahead_m(active_name),
                                cyclic=False,
                            )
                            pure_pursuit.maxSteeringAngle = 0.50  # More steering for tighter tracking
                        else:
                            pure_pursuit.updatePath(active_wp.T, cyclic=False)
                            pure_pursuit.lookahead = pure_pursuit_lookahead_m(active_name)
                            pure_pursuit.set_waypoint_index(0)
                            pure_pursuit.maxSteeringAngle = 0.50  # More steering for tighter tracking
                        segment_hold_until = t + SEGMENT_HOLD_S
                        if debug_print:
                            print(f"[NAV] Segment switch -> {active_name} lookahead={pure_pursuit.lookahead:.2f}m")

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
                # Simple parking: just drive forward to goal point
                v_ctrl = max(0.10, dr_speed)
                if park_pursuit is not None:
                    target_steer = float(park_pursuit.update(
                        np.array([pose_x, pose_y], dtype=np.float64), pose_th, v_ctrl))
                else:
                    target_steer = 0.0
                target_steer = clamp(target_steer, -0.38, 0.38)
                speed_cmd = float(min(max_speed_mps, float(PARK_SPEED_MPS)))
                stop_reason = "park"

                # Stop when close enough to goal OR if obstacle detected
                dist_park = float(math.hypot(pose_x - park_goal_x, pose_y - park_goal_y))
                obstacle_near = bool(front_valid and (center_front < 0.40 or min_front < 0.30))

                if dist_park <= float(PARK_STOP_RADIUS_M) or obstacle_near:
                    speed_cmd = 0.0
                    parking_active = False
                    stop_reason = "park_done" if not obstacle_near else "park_obstacle"
                elif (t - park_start_t) > float(PARK_TIMEOUT_S):
                    speed_cmd = 0.0
                    parking_active = False
                    stop_reason = "park_timeout"

            else:
                step_name = active_name
                v_ctrl = max(0.20, dr_speed)
                base_lookahead_m = pure_pursuit_lookahead_m(active_name)
                if active_name == "TO_HUB":
                    # Adaptive lookahead for return-to-hub: moderate adjustments only
                    hub_la_min = 0.30 if compact_track else 0.50  # Reasonable minimum
                    hub_la_max = base_lookahead_m

                    # Simple CTE-based adjustment
                    if last_path_cte_m > HIGH_CTE_M:
                        hub_la_des = hub_la_min
                    elif last_path_cte_m > MID_CTE_M:
                        hub_la_des = max(hub_la_min, 0.85 * base_lookahead_m)
                    else:
                        hub_la_des = base_lookahead_m

                    hub_la_prev = float(getattr(pure_pursuit, "lookahead", base_lookahead_m))
                    # Apply smoothing
                    pure_pursuit.lookahead = clamp(
                        LOOKAHEAD_SMOOTH_ALPHA * hub_la_prev + LOOKAHEAD_SMOOTH_BETA * hub_la_des,
                        hub_la_min,
                        hub_la_max,
                    )
                elif last_path_cte_m > MID_CTE_M:
                    hub_la_prev = float(getattr(pure_pursuit, "lookahead", base_lookahead_m))
                    hub_la_des = max(0.30 if compact_track else 0.50, 0.85 * base_lookahead_m)
                    pure_pursuit.lookahead = LOOKAHEAD_SMOOTH_ALPHA * hub_la_prev + LOOKAHEAD_SMOOTH_BETA * hub_la_des
                else:
                    # Keep lookahead stable - don't change it unnecessarily
                    pass
                target_steer = float(pure_pursuit.update(
                    np.array([pose_x, pose_y], dtype=np.float64), pose_th, v_ctrl))
                target_steer = clamp(target_steer, -0.50, 0.50)

                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(pure_pursuit.p_ref)))
                last_path_cte_m = cte

                # Log CTE data (add after cte is computed)
                if 'cte' in locals() and 'active_name' in locals():
                    try:
                        cte_writer.writerow([t, cte, active_name, pose_x, pose_y])
                    except Exception:
                        pass
                    # Store CTE for segment average
                    if active_name not in segment_cte_values:
                        segment_cte_values[active_name] = []
                    segment_cte_values[active_name].append(cte)

                # Wrong-way detection: only resync if heading is truly backwards (>90°)
                # Trust Stanley path following - waypoints are correct!
                heading_err = math.atan2(
                    math.sin(pure_pursuit.th_ref - pose_th), math.cos(pure_pursuit.th_ref - pose_th))
                
                # Only resync if heading is significantly wrong (>90° = truly backwards).
                # Skip at segment end: no waypoints ahead, so resync to i_near is useless and causes a loop.
                # Skip on TO_DROPOFF: the car just exited the roundabout; any heading error is from the
                # exit angle, not genuine wrong-way travel. Resyncing here drags the car into the top wall.
                n_wp = active_wp.shape[0]
                current_wpi = int(getattr(pure_pursuit, "wpi", 0)) if hasattr(pure_pursuit, "wpi") else 0
                if (
                    active_name != "TO_DROPOFF"
                    and abs(heading_err) > GENERIC_WRONG_WAY_THRESHOLD
                    and cte > GENERIC_WRONG_WAY_CTE_M
                    and i_near < n_wp - 2
                    and t >= segment_started_t + GENERIC_WRONG_WAY_START_GRACE_S
                    and t >= last_generic_wrong_way_resync_t + GENERIC_WRONG_WAY_RESYNC_COOLDOWN_S
                ):
                    resync_idx = max(i_near, current_wpi)
                    if compact_track and cte > HIGH_CTE_M:
                        resync_idx = min(resync_idx + 1, n_wp - 2)
                    set_waypoint_index_safe(resync_idx)
                    last_generic_wrong_way_resync_t = t
                    if debug_print and t > wrong_way_print_until:
                        wrong_way_print_until = t + 2.0
                        wpx = float(active_wp[resync_idx, 0]) if resync_idx < len(active_wp) else float('nan')
                        wpy = float(active_wp[resync_idx, 1]) if resync_idx < len(active_wp) else float('nan')
                        msg = (f"wrong-way resync wpi->{resync_idx} heading_err_deg={math.degrees(heading_err):.0f} "
                               f"cte={cte:.2f} pose=({pose_x:.2f},{pose_y:.2f}) wpt=({wpx:.2f},{wpy:.2f})")
                        print(f"[NAV] {msg}")
                        try:
                            _dbg2("NAV", "wrong_way_resync", msg,
                                  {"i_near": int(i_near), "resync_wpi": int(resync_idx), "heading_err_deg": math.degrees(heading_err), "cte_m": float(cte), "pose": [pose_x, pose_y], "wpt": [wpx, wpy]})
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
                    speed_cmd = min(speed_cmd, 0.38)
                # Cap speed near the right-boundary section of TO_HUB (x > 1.3 m)
                # The path passes through x≈2.07–2.11 which is close to the right wall.
                if active_name == "TO_HUB" and pose_x > 1.3:
                    speed_cmd = min(speed_cmd, 0.18)

                # Only slow for sharp curves – keep at least MIN_CRUISE_SPEED
                abs_plan_steer = abs(target_steer)
                if abs_plan_steer > 0.35:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                elif abs_plan_steer > 0.28:
                    speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                # Extra curve speed cap on TO_HUB to stay glued to tighter turns.
                if active_name == "TO_HUB":
                    if abs_plan_steer > 0.30:
                        speed_cmd = min(speed_cmd, 0.15)
                    elif abs_plan_steer > 0.22:
                        speed_cmd = min(speed_cmd, 0.18)
                # else: no cap – keep full speed for straights and gentle curves

                # Cross-track error – use actual low-speed caps on the compact 1/10 route.
                if cte > HIGH_CTE_M:
                    speed_cmd = min(speed_cmd, HIGH_CTE_SPEED_CAP)
                    if debug_print and t > wrong_way_print_until:
                        wrong_way_print_until = t + 2.0
                        msg = (f"Large cross-track error: {cte:.2f}m - reducing speed "
                               f"pose=({pose_x:.2f},{pose_y:.2f}) nearest_wpi={i_near} lookahead={pure_pursuit.lookahead:.2f}")
                        print(f"[NAV] {msg}")
                        try:
                            _dbg2("NAV", "large_cte", msg,
                                  {"cte_m": float(cte), "pose": [pose_x, pose_y], "nearest_wpi": int(i_near), "lookahead_m": float(pure_pursuit.lookahead)})
                        except Exception:
                            pass
                elif cte > MID_CTE_M:
                    speed_cmd = min(speed_cmd, MID_CTE_SPEED_CAP)
                elif cte > LOW_CTE_M:
                    speed_cmd = min(speed_cmd, LOW_CTE_SPEED_CAP)

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

                # Log sign event (add after sign is triggered)
                try:
                    sign_writer.writerow([t, sign.sign_type, i, sign.x, sign.y, dist])
                except Exception:
                    pass
                sign_event_count[sign.sign_type] = sign_event_count.get(sign.sign_type, 0) + 1

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
                                    # Log traffic light event
                                    try:
                                        sign_event_count["red_light"] = sign_event_count.get("red_light", 0) + 1
                                        sign_writer.writerow([t, "red_light", -1, tl.x if 'tl' in locals() else 0, tl.y if 'tl' in locals() else 0, 0])
                                    except Exception:
                                        pass
                                elif sname == "traffic_light_yellow":
                                    tl_yellow_until = max(tl_yellow_until, t + TL_HOLD_S)
                                    if debug_print:
                                        print(f"[TL] YELLOW  conf={sc:.2f}")
                                    try:
                                        sign_event_count["yellow_light"] = sign_event_count.get("yellow_light", 0) + 1
                                    except Exception:
                                        pass
                                elif sname == "traffic_light_green":
                                    tl_red_until = 0.0
                                    tl_yellow_until = 0.0
                                    tl_green_until = max(tl_green_until, t + TL_HOLD_S)
                                    if debug_print:
                                        print(f"[TL] GREEN  conf={sc:.2f}")
                                    try:
                                        sign_event_count["green_light"] = sign_event_count.get("green_light", 0) + 1
                                    except Exception:
                                        pass
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

            # NEVER suppress sidewalk detection - white curb/sidewalk detection is ALWAYS active
            # The suppression code that was here was causing the car to drive onto white curbs
            # Sidewalk safety is the #1 priority - always detect and avoid white areas

            # ----------------------------------------------------------
            # 9. Steering mixer: Simple Pure Pursuit
            # ----------------------------------------------------------
            # DISABLED: Wrong-way and roundabout logic - interfering with clean driving
            in_roundabout_zone = False
            wrong_way_correcting_steer = False
            drifted = (not route_done) and (cte >= DRIFT_CTE_PURE_PURSUIT_M)
            # When drifted, do NOT use roundabout/wrong-way steering – follow pure pursuit to rejoin path
            # When FOLLOW_WAYPOINTS_ONLY: always follow green line – no roundabout/lane steer override
            # On TO_HUB we follow waypoints back to hub – do not apply roundabout keep-right or sidewalk_rb
            in_roundabout_zone_steer = (
                (not FOLLOW_WAYPOINTS_ONLY) and in_roundabout_zone and not drifted
                and active_name not in ("TO_HUB", "TO_DROPOFF")
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
                            # Allow faster traversal of roundabouts when safe
                            speed_cmd = min(speed_cmd, 2.2)  # In roundabout – allow higher speed
                        elif rb_entry_zone:
                            speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                        elif rb_approach_zone:
                            speed_cmd = min(speed_cmd, MIN_CRUISE_SPEED)
                else:
                    # Sidewalk detected in roundabout: AGGRESSIVE curb avoidance
                    speed_cmd = min(speed_cmd, SW_PROBE_SPEED)  # Slow down more
                    if not stop_reason:
                        stop_reason = "sidewalk_rb"
                    # STEER AWAY FROM CURB - don't just keep-right blindly!
                    if swB > 0.05:  # Sidewalk/curb on right → steer left HARD
                        steer_raw = 0.45  # Override roundabout keep-right
                    elif swB < -0.05:  # Sidewalk/curb on left → steer right
                        steer_raw = RB_KEEP_RIGHT_STEER
                    else:
                        steer_raw = RB_KEEP_RIGHT_STEER  # Default keep-right when no curb detected
                
                rb_bias_applied = True
            else:
                # SIMPLE MODE: Pure Pursuit always, with optional tiny lane correction
                # NO mode switching, NO lane-only mode, NO hysteresis complexity
                # Just follow the path with minimal centering help from lane detection

                if FOLLOW_WAYPOINTS_ONLY or active_name == "TO_DROPOFF" or (TO_HUB_FORCE_WAYPOINTS_ONLY and active_name == "TO_HUB"):
                    # Pure path following only - allow full steering authority
                    steer_raw = clamp(target_steer, -0.50, 0.50)
                elif drifted:
                    # When drifted, still allow good steering range
                    steer_raw = clamp(target_steer, -0.48, 0.48)
                elif lane_ok and lane_conf >= LANE_CONF_MIN and not turning_active:
                    # Simple blend: Pure Pursuit + tiny lane centering correction
                    centering_correction = clamp(-0.12 * lane_err, -0.06, 0.06)
                    steer_raw = clamp(target_steer + centering_correction, -0.50, 0.50)
                else:
                    # Default: pure path following
                    steer_raw = clamp(target_steer, -0.50, 0.50)

                rb_bias_applied = False

            # Simple turning latch for debug/display only - don't use to disable lane
            if max(abs(steer_raw), abs(target_steer)) >= TURN_ACTIVE_STEER_ON:
                turn_latch_until = max(turn_latch_until, t + TURN_ACTIVE_HOLD_S)
            # Always allow lane correction - even during turns
            turning_active = False

            # ----------------------------------------------------------
            # 10. Safety overrides (sidewalk guard already checked above)
            # ----------------------------------------------------------

            obs_bias_terms = []
            if depth_valid:
                obs_bias_terms.append(float(depth_bias))
            if lidar_valid:
                obs_bias_terms.append(float(lidar_bias))
            obs_bias = float(sum(obs_bias_terms) / len(obs_bias_terms)) if obs_bias_terms else 0.0

            # Timeout-based recovery for stuck state with large CTE
            # NOTE: Disabled while on TO_HUB to avoid triggering recoveries during final approach.
            if not route_done:
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(pure_pursuit.p_ref)))
                if active_name == "TO_HUB":
                    cte_stuck_streak_s = 0.0
                else:
                    # Trigger when: large CTE + either speed_cmd is low OR the car is not moving
                    # (the car may keep commanding speed into the wall — dr_speed is the ground truth)
                    if cte > CTE_LARGE_M and (speed_cmd < 0.1 or dr_speed < 0.04):
                        cte_stuck_streak_s += dt
                        if cte_stuck_streak_s > CTE_STUCK_TIMEOUT_S:
                            # Force recovery: reverse away from wall then creep forward to rejoin path
                            cte_stuck_streak_s = 0.0
                            recover_stuck_streak_s = 0.0
                            recover_trigger_count += 1
                            recover_reverse_until = t + float(RECOVER_REVERSE_S) * 1.5
                            recover_forward_until = recover_reverse_until + float(RECOVER_FORWARD_S) * 1.2
                            recover_cooldown_until = t + float(RECOVER_COOLDOWN_S)
                            recover_reason = "cte_stuck_wall"
                            stop_reason = "cte_stuck_recover"
                    else:
                        cte_stuck_streak_s = 0.0

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
                _hub_gate_ready_now = bool(
                    (t - segment_started_t) >= HUB_MIN_ACTIVE_S
                    and seg_max_idx >= max(int(HUB_MIN_PROGRESS_WP), int(HUB_MIN_PROGRESS_FRAC * max(1, int(active_wp.shape[0]))))
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

                if _hub_arrival_blocked and _hub_gate_ready_now:
                    route_done_reason = "hub_arrival_blocked"
                    route_done = True
                    speed_cmd = 0.0
                    stop_reason = "hub_wait"
                    if debug_print:
                        print(
                            f"[NAV] route_done reason={route_done_reason} "
                            f"dist_to_goal={dist_to_goal:.2f} center_front={center_front:.2f} min_front={min_front:.2f}"
                        )
                elif _hub_arrival_blocked and (not _hub_gate_ready_now):
                    speed_cmd = min(speed_cmd, 0.12)
                    if not stop_reason or stop_reason.startswith("obstacle"):
                        stop_reason = "hub_gate_hold"
                    if debug_print and t > hub_gate_print_until:
                        hub_gate_print_until = t + 1.5
                        req_wp = max(int(HUB_MIN_PROGRESS_WP), int(HUB_MIN_PROGRESS_FRAC * max(1, int(active_wp.shape[0]))))
                        print(
                            f"[NAV] hub-arrival gate hold elapsed={max(0.0, t-segment_started_t):.1f}s/{HUB_MIN_ACTIVE_S:.1f}s "
                            f"progress={seg_max_idx} req>={req_wp} dist={dist_to_goal:.2f}"
                        )
                elif _side_only_very_close:
                    # Slow and steer away, but avoid deadlocking into obstacle-stop/recovery loops.
                    speed_cmd = min(speed_cmd, 0.12)
                    steer_raw = clamp(steer_raw + 0.22 * float(obs_bias), -0.50, 0.50)
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

            # Recovery mode is DISABLED - skip all recovery logic
            # Keep the stuck detection variables at default values
            stop_reasons_never_recover = ("stop_sign", "tl_red", "hold", "hub_wait")
            # Recovery mode is DISABLED - keep variables at defaults
            blocked_reason = False
            not_moving = False
            trying_to_move = False
            front_blocked = False
            no_progress = False
            stuck_now = False
            in_recovery = False

            # Enforce minimum cruise speed when moving forward (unless hard stop / cautious creep)
            _no_min_speed = (
                "stop_sign", "tl_red", "hold", "hub_wait", "obstacle", "obstacle_slow",
                "lane_depart_stop", "lane_depart_creep", "lane_depart_recover",
                "sidewalk_stop", "sidewalk_recover", "lane_lost", "park",
            )
            if speed_cmd > 1e-3 and stop_reason not in _no_min_speed and not (stop_reason or "").startswith("recover_"):
                speed_cmd = max(speed_cmd, MIN_CRUISE_SPEED)

            # DISABLED: Wrong-side and extreme sidewalk guards - interfering with Pure Pursuit
            # Let Pure Pursuit handle all steering without interference

            # ----------------------------------------------------------
            # 11. Actuate
            # ----------------------------------------------------------
            # #endregion agent log
            throttle  = speed_to_throttle(speed_cmd)

            # MINIMAL smoothing for tight tracking - allow quick corrections
            steer_smoothed = 0.50 * last_steer_smooth + 0.50 * steer_raw
            steer_smoothed = clamp(steer_smoothed, -0.50, 0.50)
            last_steer_smooth = steer_smoothed

            steer_out = float(STEER_OUTPUT_SIGN * steer_smoothed)

            # LED colour (segment-aware)
            if t < start_magenta_until or step_name in ("HUB_WAIT", "DONE"):
                led_arr = np.array([0, 0, 0, 0, 1, 0, 0, 0], dtype=np.float64)
                _led_rgb = (1.0, 0.0, 1.0)
            elif step_name == "TO_DROPOFF":
                led_arr = np.array([0, 0, 0, 0, 0, 1, 0, 0], dtype=np.float64)
                _led_rgb = (0.0, 0.0, 1.0)
            elif step_name in ("TO_HUB", "PARK"):
                led_arr = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
                _led_rgb = (1.0, 0.5, 0.0)
            else:
                led_arr = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
                _led_rgb = (0.0, 1.0, 0.0)

            # Log speed data
            if 'speed_cmd' in locals() and 'throttle' in locals():
                try:
                    speed_writer.writerow([t, speed_cmd, dr_speed, throttle])
                except Exception:
                    pass

            # Log steering data (add after steer_raw is computed)
            if 'target_steer' in locals() and 'steer_raw' in locals() and 'steer_out' in locals():
                lane_steer_val = lane_steer if 'lane_steer' in locals() else 0.0
                try:
                    steer_writer.writerow([t, target_steer, lane_steer_val, steer_raw, steer_out])
                except Exception:
                    pass

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
            # Make 600x600 to match lidar map size
            wp_map = render_waypoint_map(
                route_segments, active_wp, pose_x, pose_y, pose_th,
                pure_pursuit, i_near, step_name,
                width=600, height=600,
            )
            wp_map_small = cv2.resize(wp_map, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            cv2.imshow("waypoint_map", wp_map_small)
            if lane_dbg is not None:
                cv2.imshow("lane_fit_debug", lane_dbg)
            if sw_dbg is not None:
                cv2.imshow("lane_sw_debug", sw_dbg)

            lidar_map_img = _lidar_mapper.render()
            lidar_map_small = cv2.resize(lidar_map_img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            cv2.imshow("lidar_map", lidar_map_small)

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

        # Close log files
        try:
            cte_log.close()
            speed_log.close()
            steer_log.close()
            segment_log.close()
            sign_log.close()
        except Exception:
            pass
        
        # Print summary to console
        if debug_print:
            try:
                print("\n" + "="*50)
                print("PAPER DATA LOGGING SUMMARY")
                print("="*50)
                print(f"Sign events:")
                for sign_type, count in sign_event_count.items():
                    print(f"  {sign_type}: {count}")
                print(f"Segment times:")
                for seg, dur in segment_times.items():
                    print(f"  {seg}: {dur:.1f}s")
                print(f"Log files saved to: {os.path.abspath(log_dir)}")
                print("="*50 + "\n")
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
    ap.add_argument("--speed", type=float, default=0.6)
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
