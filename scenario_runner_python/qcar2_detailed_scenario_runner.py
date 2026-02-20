"""qcar2_detailed_scenario_runner.py  –  Custom TCP + Map-Known Signs

Localization : Direct TCP to QLabs for get_world_transform() every frame.
               Returns simulation x10 coordinates; scaled ×0.10 to
               match the 1:1 real-world frame used by StanleyController.

Path following: StanleyController on interpolated waypoint segments.
Lane detection: secondary centering correction.

Signs / TLs  : positions hard-coded from Setup_Real_Scenario_fullscale_x10.py.
               Stop, yield, roundabout signs  → proximity triggers.
               Traffic lights                  → YOLO colour when close.

Safety       : depth + LiDAR frontal obstacle detection + sidewalk guard.
"""

import argparse
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
from hal.utilities.control import StanleyController


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
        self.bias_right_px = -28.0
        self.k_lat = 0.65
        self.k_head = 0.25
        self.kd = 0.06
        self.max_steer = 0.34
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

        roi_h, roi_w = roi.shape[:2]
        y_eval = float(roi_h - 6)
        mid_x = int(0.5 * roi_w)

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

        y_nf = y_mask[max(0, roi_h - 120):roi_h, :]
        w_nf = w_mask[max(0, roi_h - 120):roi_h, :]
        lane_pix = float((y_nf > 0).mean() + (w_nf > 0).mean())
        conf = clamp(lane_pix / 0.07, 0.0, 1.0)
        valid = bool(conf > 0.25)

        dbg = None
        if debug:
            dbg = roi.copy()
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

        sw_rel = (L > (L_med + 52.0)) & (s < min(55.0, s_med + 18.0)) & (v > (v_med + 34.0))
        sw_abs = (L > 220.0) & (s < 45.0) & (v > 205.0)
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
    thr = clamp(0.05 + 0.08 * s, 0.06, 0.30)
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
#  Main Runner
# ===================================================================

def run_scenario(
    waypoints_file: str,
    actor_number: int = 0,
    sample_rate_hz: float = 30.0,
    max_speed_mps: float = 2.2,
    debug_print: bool = True,
    use_lidar: bool = True,
    use_realsense: bool = True,
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
    # Sign model (YOLO) – used only for traffic-light colour detection
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
        sign_model_obj = YOLO(sign_model)
        if debug_print:
            print(f"[SIGN] YOLO loaded: {sign_model}  labels={sign_names}")

    # ------------------------------------------------------------------
    # Sign-action state
    # ------------------------------------------------------------------
    STOP_SIGN_DWELL_S   = 2.0
    YIELD_SLOW_S        = 2.0
    YIELD_SPEED         = 0.6
    TL_HOLD_S           = 4.0
    TL_YELLOW_SPEED     = 0.55

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
    RB_KEEP_RIGHT_STEER = -0.42   # MUCH stronger bias to ensure correct entry (was -0.32)
    RB_HOLD_S = 12.0              # hold bias longer through the roundabout (was 10.0)
    RB_ENTRY_ZONE_M = 3.5         # Distance threshold for roundabout entry zone - START EARLIER (was 2.0)
    RB_APPROACH_ZONE_M = 4.5      # NEW: Apply bias even earlier when approaching
    RB_EXIT_ZONE_M = 1.5          # Distance threshold for roundabout exit zone
    RB_WRONG_WAY_THRESHOLD = 0.8  # Stricter wrong-way detection in roundabouts (~46°) (was 1.0)
    RB_LANE_DISABLE_DIST = 2.5    # Disable lane following within this distance (was 1.8)

    # ------------------------------------------------------------------
    # Route / segment state
    # ------------------------------------------------------------------
    segment_idx = 0
    segment_hold_until = 0.0
    SEGMENT_HOLD_S     = 2.0
    route_done         = False

    active_name, active_wp = route_segments[segment_idx]
    seg_max_idx = 0
    SEG_GOAL_RADIUS_M  = 0.60
    SEG_TAIL_WP_COUNT  = 8

    stanley = StanleyController(waypoints=active_wp.T, k=1.30, cyclic=False)
    stanley.maxSteeringAngle = 0.42

    # ------------------------------------------------------------------
    # Steering blend weights
    # ------------------------------------------------------------------
    W_STANLEY = 0.95
    W_LANE    = 0.05
    TURN_LANE_BLEND_MAX     = 0.01
    LANE_ONLY_CONF_MIN      = 0.72
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
    HIGH_STEER_SPEED_CAP = 0.35
    MOD_STEER_SPEED_CAP  = 0.55
    OBSTACLE_STOP_CONFIRM_S = 0.22

    # Reverse recovery (unstick when wedged against curb/wall)
    RECOVER_ENABLE          = True
    RECOVER_STUCK_CONFIRM_S = 0.80
    RECOVER_REVERSE_S       = 1.10
    RECOVER_REVERSE_SPEED   = 0.75
    RECOVER_FORWARD_S       = 0.55
    RECOVER_FORWARD_SPEED   = 0.45
    RECOVER_COOLDOWN_S      = 2.50

    LANE_LOST_SLOW_SPEED    = 0.18
    LANE_LOST_STOP_S        = 1.20
    LANE_DEPART_ERR_SLOW    = 0.45
    LANE_DEPART_ERR_STOP    = 0.70
    LANE_DEPART_STEER_MAX   = 0.38
    LANE_DEPART_ERR_LANE_ONLY = 0.90
    LANE_DEPART_RECOVER_SPEED = 0.20
    LANE_DEPART_RECOVER_CLEAR_M = 1.20
    LANE_DEPART_STOP_GRACE_S = 0.75

    # CRITICAL: Sidewalk thresholds - balanced to prevent sidewalk contact while allowing movement
    # Increased from overly sensitive values to prevent false stops
    SW_SLOW = 0.015       # Low threshold - catch sidewalk early but not too sensitive (was 0.005)
    SW_STOP = 0.025       # Stop threshold - must stop before touching curb (was 0.008)
    SW_NEAR_STOP = 0.020  # Near-sidewalk detection threshold (was 0.006)
    SW_STRONG_CONFIRM_S = 0.15  # Confirmation time - prevent false positives (was 0.03)
    SW_PROBE_CLEAR_M = 1.10
    SW_PROBE_SPEED   = 0.15  # Slow but not stopped when probing (was 0.05)

    # ------------------------------------------------------------------
    # Runtime variables
    # ------------------------------------------------------------------
    steer_out = 0.0
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
    recover_reverse_until     = 0.0
    recover_forward_until     = 0.0
    recover_cooldown_until    = 0.0
    recover_reason            = ""
    recover_trigger_count     = 0   # number of recoveries this stuck episode (longer reverse if >= 2)
    sw_latch_until  = 0.0
    turn_latch_until = 0.0
    turning_active   = False
    wrong_way_print_until = 0.0

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
                # Nearest waypoint index (monotonic)
                d2 = (active_wp[:, 0] - pose_x) ** 2 + (active_wp[:, 1] - pose_y) ** 2
                i_near = int(np.argmin(d2))
                seg_max_idx = max(seg_max_idx, i_near)
                
                # CRITICAL: Proactive wrong-way check: look ahead on path toward roundabouts
                # Check if we're heading toward a roundabout and if our heading matches the path direction
                # START CHECKING EARLIER to prevent wrong-way entry
                for rb_sign in MAP_ROUNDABOUT_SIGNS:
                    dist_to_rb = math.hypot(pose_x - rb_sign.x, pose_y - rb_sign.y)
                    if 0.8 < dist_to_rb < 4.0:  # Start checking earlier (was 1.0-2.5m, now 0.8-4.0m)
                        # Look ahead on path to see direction we should be heading
                        look_ahead_idx = min(i_near + 8, active_wp.shape[0] - 1)  # Look further ahead
                        if look_ahead_idx > i_near:
                            wp_ahead = active_wp[look_ahead_idx]
                            dx_ahead = wp_ahead[0] - pose_x
                            dy_ahead = wp_ahead[1] - pose_y
                            path_heading = math.atan2(dy_ahead, dx_ahead)
                            heading_to_path = wrap_pi(path_heading - pose_th)
                            
                            # CRITICAL: Check if we're heading LEFT (wrong way) - must keep RIGHT!
                            # Negative heading error means heading left (wrong way for keep-right roundabout)
                            if heading_to_path < -RB_WRONG_WAY_THRESHOLD:  # Heading left = wrong way
                                stanley.set_waypoint_index(i_near)
                                if debug_print and t > wrong_way_print_until:
                                    wrong_way_print_until = t + 2.0
                                    print(f"[NAV] CRITICAL: Wrong-way detected near RB (heading LEFT)! Resync wpi->{i_near} "
                                          f"heading_err={math.degrees(heading_to_path):.1f}° dist_to_rb={dist_to_rb:.2f}")
                                break
                            # Also check if heading significantly away from path direction
                            elif abs(heading_to_path) > RB_WRONG_WAY_THRESHOLD:
                                stanley.set_waypoint_index(i_near)
                                if debug_print and t > wrong_way_print_until:
                                    wrong_way_print_until = t + 2.0
                                    print(f"[NAV] Proactive wrong-way prevention near RB: resync wpi->{i_near} "
                                          f"heading_err={math.degrees(heading_to_path):.1f}° dist_to_rb={dist_to_rb:.2f}")
                                break

                goal_x = float(active_wp[-1, 0])
                goal_y = float(active_wp[-1, 1])
                dist_to_goal = float(math.hypot(pose_x - goal_x, pose_y - goal_y))

                n_wp = active_wp.shape[0]
                seg_complete = bool(
                    (dist_to_goal <= SEG_GOAL_RADIUS_M)
                    and (
                        (seg_max_idx >= max(0, n_wp - SEG_TAIL_WP_COUNT))
                        or (seg_max_idx >= max(0, int(0.75 * n_wp)))
                    )
                )

                if (t >= segment_hold_until) and (seg_complete or bool(stanley.pathComplete)):
                    segment_idx += 1
                    recover_trigger_count = 0  # reset so next segment starts fresh
                    if segment_idx >= len(route_segments):
                        route_done = True
                    else:
                        active_name, active_wp = route_segments[segment_idx]
                        seg_max_idx = 0
                        stanley.updatePath(active_wp.T, cyclic=False)
                        stanley.maxSteeringAngle = 0.42
                        segment_hold_until = t + SEGMENT_HOLD_S
                        if debug_print:
                            print(f"[NAV] Segment switch -> {active_name}")

            # ----------------------------------------------------------
            # 4. Stanley steering
            # ----------------------------------------------------------
            stop_reason = ""
            target_steer = 0.0

            if route_done:
                step_name = "DONE"
                speed_cmd = 0.0
            else:
                step_name = active_name
                v_ctrl = max(0.20, dr_speed)
                target_steer = float(stanley.update(
                    np.array([pose_x, pose_y], dtype=np.float64), pose_th, v_ctrl))
                target_steer = clamp(target_steer, -0.42, 0.42)

                # Wrong-way detection: only resync if heading is truly backwards (>90°)
                # Trust Stanley path following - waypoints are correct!
                heading_err = math.atan2(
                    math.sin(stanley.th_ref - pose_th), math.cos(stanley.th_ref - pose_th))
                
                # Only resync if heading is significantly wrong (>90° = truly backwards)
                # Don't be too aggressive - trust the waypoint path
                wrong_way_threshold = 1.57  # 90° - only resync if truly backwards
                if abs(heading_err) > wrong_way_threshold:
                    stanley.set_waypoint_index(i_near)
                    if debug_print and t > wrong_way_print_until:
                        wrong_way_print_until = t + 2.0
                        print(f"[NAV] wrong-way resync wpi->{i_near} heading_err_deg={math.degrees(heading_err):.0f}")

                goal_x = float(active_wp[-1, 0])
                goal_y = float(active_wp[-1, 1])
                dist_to_goal = float(math.hypot(pose_x - goal_x, pose_y - goal_y))

                speed_cmd = float(max_speed_mps)
                if dist_to_goal < 1.60:
                    speed_cmd = min(speed_cmd, 0.55)
                if dist_to_goal < 0.90:
                    speed_cmd = min(speed_cmd, 0.35)

                abs_plan_steer = abs(target_steer)
                if abs_plan_steer > 0.30:
                    speed_cmd = min(speed_cmd, 0.35)
                elif abs_plan_steer > 0.20:
                    speed_cmd = min(speed_cmd, 0.50)
                elif abs_plan_steer > 0.12:
                    speed_cmd = min(speed_cmd, 0.75)

                # Cross-track error speed limiter
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(stanley.p_ref)))
                
                # CRITICAL: Limit cross-track error to prevent extreme deviations
                # If cross-track error is too large, we might be heading toward sidewalk
                MAX_CTE_M = 0.50  # Maximum allowed cross-track error (meters)
                if cte > MAX_CTE_M:
                    # Cross-track error too large - reduce speed significantly
                    speed_cmd = min(speed_cmd, 0.15)
                    if debug_print and t > wrong_way_print_until:
                        wrong_way_print_until = t + 2.0
                        print(f"[NAV] Large cross-track error: {cte:.2f}m - reducing speed")
                if cte > 0.25:
                    speed_cmd = min(speed_cmd, 0.25)
                elif cte > 0.15:
                    speed_cmd = min(speed_cmd, 0.40)

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
                            math.sin(stanley.th_ref - pose_th), math.cos(stanley.th_ref - pose_th))
                        if abs(heading_err_rb) > RB_WRONG_WAY_THRESHOLD:
                            stanley.set_waypoint_index(i_near)
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
                if trig:
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
            sidewalk_fp = bool(
                lane_ok and lane_conf > 0.90 and center_front > 1.20
                and (swC > 0.70 or swN > 0.65)  # Very high sidewalk detection but clear path ahead
                and abs(swB) < 0.15  # Not biased toward sidewalk
            )
            if sidewalk_fp:
                swC = max(0.0, swC - 0.3)  # Reduce but don't eliminate
                swN = max(0.0, swN - 0.3)
            
            # Use balanced thresholds - sensitive but not overly so
            sidewalk_strong = bool(swC > SW_STOP or swN > SW_NEAR_STOP)
            sidewalk_soft   = bool(swC > SW_SLOW or swN > 0.5 * SW_NEAR_STOP)
            
            # Only treat bias as detection if it's significant AND we're close to boundary
            if abs(swB) > 0.4 and (swC > 0.01 or swN > 0.01):  # More conservative
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

            # ----------------------------------------------------------
            # 9. Steering mixer: Stanley + Lane + Roundabout bias
            # ----------------------------------------------------------
            # CRITICAL: In roundabouts, disable lane following - use Stanley path only
            # Lane lines in roundabouts are confusing (center lines point wrong way)
            # CRITICAL: Include approach zone to start keep-right EARLY
            in_roundabout_zone = bool(rb_in_roundabout or rb_entry_zone or rb_exit_zone or rb_approach_zone or t < rb_active_until)
            
            # Get distance to closest roundabout for bias calculation
            min_dist_to_rb_steer = float("inf")
            if in_roundabout_zone:
                for rb_sign in MAP_ROUNDABOUT_SIGNS:
                    dist_to_rb = math.hypot(pose_x - rb_sign.x, pose_y - rb_sign.y)
                    if dist_to_rb < min_dist_to_rb_steer:
                        min_dist_to_rb_steer = dist_to_rb
            
            if in_roundabout_zone:
                # ROUNDABOUT MODE: Use Stanley path only, ignore lane detection
                # Lane lines in roundabouts are misleading (center lines point into center)
                steer_raw = clamp(target_steer, -0.42, 0.42)
                
                # CRITICAL: Only apply roundabout bias if NO sidewalk is detected
                # Sidewalk safety takes absolute priority over roundabout navigation
                if not sidewalk_detected:
                    # CRITICAL: Apply STRONG keep-right bias - KEY RULE: KEEP RIGHT AT ALL TIMES!
                    # Check if we're actively in the roundabout (not just approaching)
                    if rb_in_zone or (t < rb_active_until and min_dist_to_rb_steer < RB_LANE_DISABLE_DIST):
                        # VERY strong bias when actively traversing roundabout
                        steer_raw = clamp(
                            0.20 * steer_raw + 0.80 * RB_KEEP_RIGHT_STEER,  # Stronger bias (was 0.30/0.70)
                            -0.45, 0.45)
                    elif rb_entry_zone:
                        # Strong bias when approaching entry - prevent wrong-way entry
                        steer_raw = clamp(
                            0.25 * steer_raw + 0.75 * RB_KEEP_RIGHT_STEER,  # Stronger bias (was 0.40/0.60)
                            -0.45, 0.45)
                    elif rb_approach_zone:
                        # NEW: Moderate bias when far approaching - start keeping right early
                        steer_raw = clamp(
                            0.35 * steer_raw + 0.65 * RB_KEEP_RIGHT_STEER,
                            -0.45, 0.45)
                    
                    # Reduce speed in roundabouts for better control
                    if not stop_reason:
                        if rb_in_zone:
                            speed_cmd = min(speed_cmd, 0.60)  # Slower when actively in roundabout
                        elif rb_entry_zone:
                            speed_cmd = min(speed_cmd, 0.70)  # Moderate speed approaching
                        elif rb_approach_zone:
                            speed_cmd = min(speed_cmd, 0.80)  # Slightly slower when approaching
                else:
                    # Sidewalk detected in roundabout - slow down and steer away
                    # Don't hard stop unless extreme danger
                    if swC > 0.05 or swN > 0.04:  # Very high detection
                        speed_cmd = min(speed_cmd, 0.20)  # Very slow but moving
                    else:
                        speed_cmd = min(speed_cmd, 0.40)  # Moderate speed
                    if not stop_reason:
                        stop_reason = "sidewalk_rb"
                    # Steer away from sidewalk but maintain some roundabout navigation
                    if swB > 0.15:  # Sidewalk on right
                        steer_raw = clamp(0.60 * 0.35 + 0.40 * steer_raw, -0.45, 0.45)  # Left bias
                    elif swB < -0.15:  # Sidewalk on left
                        steer_raw = clamp(0.60 * (-0.35) + 0.40 * steer_raw, -0.45, 0.45)  # Right bias
                    else:
                        steer_raw = clamp(steer_raw + 0.30 * swB, -0.45, 0.45)
                
                rb_bias_applied = True
            else:
                # NORMAL MODE: Use lane + Stanley blend
                if lane_ok:
                    lane_w_eff = W_LANE
                    stanley_w_eff = W_STANLEY
                    if turning_active or abs(target_steer) >= TURN_ACTIVE_STEER_ON:
                        lane_w_eff = TURN_LANE_BLEND_MAX
                        stanley_w_eff = 1.0 - lane_w_eff

                    steer_raw = clamp(
                        stanley_w_eff * target_steer + lane_w_eff * lane_steer,
                        -0.45, 0.45)

                    # Lane-only takeover on straights with strong confidence
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
            # SIDEWALK SAFETY - Balanced approach: prevent contact while allowing movement
            # ============================================================
            # Only stop if CONFIRMED danger AND actually heading toward sidewalk
            # Otherwise, slow down and steer away but keep moving
            # CRITICAL: Don't let sidewalk detection block recovery maneuvers
            
            # Skip sidewalk checks during recovery - recovery takes priority
            in_recovery = bool(t < recover_reverse_until or t < recover_forward_until)
            
            if sidewalk_confirmed and not in_recovery:
                # Sidewalk confirmed - check if we're actually heading toward it
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(stanley.p_ref)))
                
                # Only hard stop if confirmed AND we're significantly off path toward sidewalk
                if swC > 0.05 or swN > 0.04:  # Very high sidewalk detection
                    if cte > 0.30:  # Significantly off path
                        speed_cmd = 0.0  # Hard stop only if real danger
                        sidewalk_stop_streak_s += dt
                        if not stop_reason:
                            stop_reason = "sidewalk_stop"
                        
                        # Steer away from sidewalk
                        if swB > 0.15:
                            steer_raw = 0.40  # Strong left
                        elif swB < -0.15:
                            steer_raw = -0.40  # Strong right
                        else:
                            steer_raw = clamp(0.50 * swB, -0.45, 0.45)
                        
                        # Recovery after brief stop
                        if center_front > 1.20 and sidewalk_stop_streak_s > 0.3:
                            speed_cmd = SW_PROBE_SPEED
                            if not stop_reason:
                                stop_reason = "sidewalk_recover"
                            steer_raw = clamp(0.60 * steer_raw + 0.40 * target_steer, -0.45, 0.45)
                    else:
                        # Following path - just slow down and steer away
                        speed_cmd = min(speed_cmd, 0.25)
                        sidewalk_stop_streak_s = 0.0
                        if not stop_reason:
                            stop_reason = "sidewalk_caution"
                        steer_raw = clamp(target_steer + 0.30 * swB, -0.45, 0.45)
                else:
                    # Confirmed but not extreme - slow down
                    speed_cmd = min(speed_cmd, 0.30)
                    sidewalk_stop_streak_s = 0.0
                    if not stop_reason:
                        stop_reason = "sidewalk_caution"
                    steer_raw = clamp(target_steer + 0.25 * swB, -0.45, 0.45)
                    
            elif sidewalk_strong and not in_recovery:
                # Strong detection - slow down significantly but don't stop
                speed_cmd = min(speed_cmd, 0.30)  # Slow but moving
                sidewalk_stop_streak_s = 0.0
                if not stop_reason:
                    stop_reason = "sidewalk_strong"
                
                # Steer away from sidewalk but maintain path following
                if swB > 0.2:
                    steer_raw = clamp(0.50 * 0.35 + 0.50 * target_steer, -0.45, 0.45)
                elif swB < -0.2:
                    steer_raw = clamp(0.50 * (-0.35) + 0.50 * target_steer, -0.45, 0.45)
                else:
                    steer_raw = clamp(target_steer + 0.30 * swB, -0.45, 0.45)
                    
            elif sidewalk_soft and not in_recovery:
                # Soft detection - moderate speed reduction
                speed_cmd = min(speed_cmd, 0.50)  # Moderate speed
                sidewalk_stop_streak_s = 0.0
                if not stop_reason:
                    stop_reason = "sidewalk_soft"
                
                # Slight steering bias away from sidewalk
                steer_raw = clamp(target_steer + 0.20 * swB, -0.45, 0.45)
            else:
                sidewalk_stop_streak_s = 0.0

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
                    np.array([pose_x, pose_y]) - np.array(stanley.p_ref)))
                # Only worry if VERY significantly off path AND front blocked
                if cte > 0.50 and front_valid and min_front < SLOW_FRONT_M:  # Increased threshold
                    speed_cmd = min(speed_cmd, LANE_LOST_SLOW_SPEED)
                    if not stop_reason:
                        stop_reason = "lane_lost"
                    steer_raw = clamp(target_steer + 0.20 * swB + 0.30 * obs_bias, -0.35, 0.35)
                    # NEVER hard stop for lane loss - just slow down
                    # Trust path following will get us back on track
                else:
                    # Trust path following - lane detection is just a helper
                    lane_lost_streak_s = 0.0

            # Lane departure guardrail - TRUST PATH FOLLOWING
            # Only use lane departure if we're also off the waypoint path
            # Waypoints are correct - trust them over lane detection
            if not turning_active and lane_ok and not in_roundabout_zone:
                abs_lane_err = abs(float(lane_err))
                cte = float(np.linalg.norm(
                    np.array([pose_x, pose_y]) - np.array(stanley.p_ref)))
                
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
                        speed_cmd = min(speed_cmd, 0.20)
                        if not stop_reason:
                            stop_reason = "lane_depart_creep"
                elif abs_lane_err >= LANE_DEPART_ERR_SLOW and cte > 0.20:
                    lane_depart_stop_streak_s = 0.0
                    speed_cmd = min(speed_cmd, 0.35)
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
                speed_cmd = min(speed_cmd, 0.35)
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
                # Only hard stop if obstacle is VERY close
                if center_front < 0.30 or min_front < 0.25:  # Very close - real danger
                    speed_cmd = 0.0
                    stop_reason = "obstacle"
                else:
                    # Slow down but keep moving
                    speed_cmd = min(speed_cmd, 0.15)
                    if not stop_reason:
                        stop_reason = "obstacle_slow"
            elif front_valid and (center_front < CENTER_SLOW_M or min_front < 0.55):
                speed_cmd *= 0.6  # Less aggressive slowdown (was 0.5)

            # High-steer speed cap
            abs_steer = abs(float(steer_raw))
            if abs_steer > 0.30:
                speed_cmd = min(speed_cmd, HIGH_STEER_SPEED_CAP)
            elif abs_steer > 0.20:
                speed_cmd = min(speed_cmd, MOD_STEER_SPEED_CAP)

            # Reverse recovery when wedged against curb / wall
            # Use world-transform heading vs path tangent to steer toward path (reorient, not just rock).
            if RECOVER_ENABLE:
                if t < recover_reverse_until:
                    # Back up: steer to align pose_th toward path (stanley.th_ref) so we reorient.
                    speed_cmd = -float(RECOVER_REVERSE_SPEED)
                    if recover_reason:
                        stop_reason = recover_reason
                    align_err = math.atan2(
                        math.sin(stanley.th_ref - pose_th), math.cos(stanley.th_ref - pose_th))
                    align_steer = clamp(float(align_err) * 1.0, -0.45, 0.45)
                    steer_raw = clamp(
                        align_steer + 0.35 * float(obs_bias) + 0.30 * float(swB),
                        -0.45, 0.45,
                    )

                elif t < recover_forward_until:
                    # Forward: keep aligning to path so we drive out the right way.
                    speed_cmd = float(RECOVER_FORWARD_SPEED)
                    if recover_reason:
                        stop_reason = recover_reason
                    align_err = math.atan2(
                        math.sin(stanley.th_ref - pose_th), math.cos(stanley.th_ref - pose_th))
                    align_steer = clamp(float(align_err) * 0.9, -0.45, 0.45)
                    steer_raw = clamp(
                        align_steer + 0.25 * float(obs_bias) + 0.20 * float(swB),
                        -0.45, 0.45,
                    )

                else:
                    # Include recover-related reasons so we keep trying after sidewalk/lane_depart creep
                    blocked_reason = stop_reason in (
                        "obstacle", "sidewalk_stop", "lane_lost_stop", "lane_depart_stop",
                        "sidewalk_recover", "lane_depart_recover",
                    )
                    # More lenient stuck detection - only if REALLY stuck
                    stuck_now = bool((speed_cmd <= 0.05) and blocked_reason and (dr_speed < 0.05))  # More lenient
                    front_blocked = bool(front_valid and (center_front < 0.70 or min_front < 0.45))  # More lenient
                    
                    # Check for wrong-way in roundabout - if heading error is large and in roundabout, prioritize recovery
                    in_roundabout = t < rb_active_until
                    heading_err_recover = math.atan2(
                        math.sin(stanley.th_ref - pose_th), math.cos(stanley.th_ref - pose_th))
                    wrong_way_in_rb = bool(in_roundabout and abs(heading_err_recover) > 1.2)
                    
                    if t < recover_cooldown_until:
                        recover_stuck_streak_s = 0.0
                    elif (stuck_now and front_blocked) or wrong_way_in_rb:
                        if wrong_way_in_rb:
                            # Wrong-way in roundabout: more aggressive recovery
                            recover_stuck_streak_s += dt * 1.5
                        else:
                            recover_stuck_streak_s += dt
                        
                        confirm_time = RECOVER_STUCK_CONFIRM_S * (0.6 if wrong_way_in_rb else 1.0)
                        if recover_stuck_streak_s >= confirm_time:
                            recover_stuck_streak_s = 0.0
                            recover_trigger_count += 1
                            # Longer reverse after 2+ recoveries or if wrong-way in roundabout
                            wrong_way_mult = 1.4 if wrong_way_in_rb else 1.0
                            rev_s = float(RECOVER_REVERSE_S) * (1.65 if recover_trigger_count >= 2 else 1.0) * wrong_way_mult
                            fwd_s = float(RECOVER_FORWARD_S) * (1.2 if recover_trigger_count >= 2 else 1.0)
                            recover_reverse_until = t + rev_s
                            recover_forward_until = recover_reverse_until + fwd_s
                            recover_cooldown_until = t + float(RECOVER_COOLDOWN_S)
                            recover_reason = f"recover_{stop_reason}" + ("_wrongway_rb" if wrong_way_in_rb else "")

                            if debug_print:
                                print(
                                    "[RECOVER] rock "
                                    f"rev={rev_s:.2f}s fwd={fwd_s:.2f}s #={recover_trigger_count} reason={stop_reason} "
                                    f"cF={center_front:.2f} minF={min_front:.2f} "
                                    f"swB={swB:+.2f} obsB={obs_bias:+.2f}"
                                    + (" WRONGWAY_RB" if wrong_way_in_rb else "")
                                )
                    else:
                        recover_stuck_streak_s = 0.0
                    # Reset recovery count once we've been moving again for a while
                    if t > recover_cooldown_until + 4.0 and dr_speed > 0.08:
                        recover_trigger_count = 0

            # ----------------------------------------------------------
            # 11. Actuate
            # ----------------------------------------------------------
            throttle  = speed_to_throttle(speed_cmd)
            steer_out = float(STEER_OUTPUT_SIGN * steer_raw)

            # LED colour (segment-aware)
            if t < start_magenta_until or step_name in ("HUB_WAIT", "DONE"):
                led_arr = np.array([0, 0, 0, 0, 1, 0, 1, 1], dtype=np.float64)
                _led_rgb = (1.0, 0.0, 1.0)
            elif step_name == "TO_DROPOFF":
                led_arr = np.array([0, 0, 0, 0, 0, 1, 1, 1], dtype=np.float64)
                _led_rgb = (0.0, 0.0, 1.0)
            elif step_name == "TO_HUB":
                led_arr = np.array([1, 1, 1, 1, 0, 0, 1, 1], dtype=np.float64)
                _led_rgb = (1.0, 0.5, 0.0)
            else:
                led_arr = np.array([0, 0, 0, 0, 0, 0, 1, 1], dtype=np.float64)
                _led_rgb = (0.0, 1.0, 0.0)

            car.read_write_std(throttle=throttle, steering=steer_out, LEDs=led_arr)
            if _led_ctrl.connected and _led_rgb != _prev_led_rgb:
                if _led_ctrl.set_color(*_led_rgb):
                    _prev_led_rgb = _led_rgb

            # ----------------------------------------------------------
            # 12. HUD
            # ----------------------------------------------------------
            if bgr is not None and bgr.size:
                action = ("REV" if speed_cmd < -1e-3
                          else ("STOP" if speed_cmd <= 1e-3
                                else ("SLOW" if speed_cmd < 0.5 * max_speed_mps else "GO")))
                lines = [
                    f"seg={step_name} idx={segment_idx+1}/{len(route_segments)}",
                    f"pos=({pose_x:+.3f},{pose_y:+.3f}) th={math.degrees(pose_th):+.1f}deg  wt_ok={int(wt_ok)}",
                    f"lane_ok={int(lane_ok)} conf={lane_conf:.2f} lane_steer={lane_steer:+.2f} err={lane_err:+.2f}",
                    f"stanley={target_steer:+.2f}  steer_raw={steer_raw:+.2f}  sent={steer_out:+.2f}",
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
    ap.add_argument("--speed", type=float, default=1.6)
    ap.add_argument("--no-print", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--no-realsense", action="store_true")
    ap.add_argument("--sign-model", required=True)
    ap.add_argument("--sign-labels", required=True)
    ap.add_argument("--sign-conf", type=float, default=0.40)
    ap.add_argument("--sign-device", default="cuda")
    args = ap.parse_args()

    run_scenario(
        waypoints_file=args.waypoints,
        actor_number=args.actor,
        sample_rate_hz=args.rate,
        max_speed_mps=args.speed,
        debug_print=(not args.no_print),
        use_lidar=(not args.no_lidar),
        use_realsense=(not args.no_realsense),
        sign_model=args.sign_model,
        sign_labels=args.sign_labels,
        sign_conf=args.sign_conf,
        sign_device=args.sign_device,
    )


if __name__ == "__main__":
    main()
