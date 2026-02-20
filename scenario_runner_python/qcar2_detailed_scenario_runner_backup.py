"""qcar2_detailed_scenario_runner.py (EKF + Stanley waypoint following + lane correction)

Goal: navigate the full route (hub -> pickup -> dropoff -> hub) using a waypoint
map as the primary steering source, with lane detection as a secondary correction.

Design:
- Localization uses QCarEKF (bicycle-model Extended Kalman Filter with gyro
  heading correction) for accurate dead-reckoning without GPS.
- Steering comes primarily from a StanleyController following the waypoint path.
- Lane detection provides secondary centering correction when available.
- Sidewalk/curb detection + depth/LiDAR act as safety layers.
- Step advancement uses StanleyController.pathComplete to detect when the car
  has actually reached the end of each route segment.
- Speed is profiled: slower on curves, near obstacles, and near sidewalks.

"""

#from __future__ import annotations

import argparse
import math
import socket
import struct
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import cv2
import numpy as np

from pal.products.qcar import QCar, QCarLidar, QCarRealSense

# global pose from QCarGPS (more reliable segment completion than pure odometry)
try:
    from pal.products.qcar import QCarGPS  # type: ignore
except Exception:  # pragma: no cover
    QCarGPS = None  # type: ignore

from pal.utilities.vision import Camera2D
from hal.utilities.estimation import EKF, KalmanFilter
from hal.utilities.control import StanleyController


class _QLabsLEDDirect:
    """Send LED-strip colour commands directly to the QLabs TCP server.
    """
    _QCAR2_CLASS_ID      = 161
    _FCN_LED_UNIFORM      = 30
    _BASE_CONTAINER_SIZE  = 13   # 4+4+4+1 (containerSize, classID, actorNumber, actorFunction)

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
        """Set the full LED strip to a uniform (R, G, B) colour (0-1 scale)."""
        if self._sock is None:
            return False
        payload = struct.pack(">fff", float(r), float(g), float(b))
        csz = self._BASE_CONTAINER_SIZE + len(payload)        # 25
        pkt = (
            struct.pack("<i", 1 + csz)
            + struct.pack(">BiiiB", 123, csz,
                          self._QCAR2_CLASS_ID, self._actor,
                          self._FCN_LED_UNIFORM)
            + payload
        )
        try:
            self._sock.sendall(pkt)
            # Drain any ACK bytes so the buffer doesn't fill up
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


# ---------------------------
# Utils
# ---------------------------

def now() -> float:
    return time.time()


def clamp(x: float, lo: float, hi: float) -> float:
    return float(max(lo, min(hi, x)))


def wrap_pi(a: float) -> float:
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


def classify_sign_color(bgr_full: np.ndarray, xyxy: np.ndarray,
                        red_thresh: float = 0.28) -> str:
    """Use colour analysis to decide if a detected sign is STOP or YIELD.

    Stop signs (red octagon) have a large fraction of red pixels.
    Yield signs (inverted triangle, red border + white centre) have
    less red and more white.  Returns ``"stop"`` or ``"yield"``.
    """
    try:
        h, w = bgr_full.shape[:2]
        x1 = max(0, int(xyxy[0]))
        y1 = max(0, int(xyxy[1]))
        x2 = min(w, int(xyxy[2]))
        y2 = min(h, int(xyxy[3]))
        crop = bgr_full[y1:y2, x1:x2]
        if crop.size == 0:
            return "stop"  # safety default
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        # Red hue wraps around 0/180 in OpenCV HSV
        lo_red = ((hsv[:, :, 0] < 12) | (hsv[:, :, 0] > 168)) & \
                 (hsv[:, :, 1] > 70) & (hsv[:, :, 2] > 50)
        red_ratio = float(np.count_nonzero(lo_red)) / max(crop.shape[0] * crop.shape[1], 1)
        return "stop" if red_ratio >= red_thresh else "yield"
    except Exception:
        return "stop"


# ---------------------------
# Waypoints
# ---------------------------

def load_waypoints_txt(path: str) -> Dict[str, np.ndarray]:
    txt = open(path, "r", encoding="utf-8").read()
    out: Dict[str, np.ndarray] = {}

    current_name = None
    current_lines = []
    for raw in txt.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.endswith(":") and line.lower().startswith("path_"):
            if current_name and current_lines:
                out[current_name] = _parse_xy_lines(current_lines)
            current_name = line[:-1].strip()
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
        # Strip comments
        if '#' in line:
            line = line[:line.index('#')]
        line = line.replace(";", "").replace(",", " ")
        parts = [p for p in line.split() if p]
        if len(parts) >= 2:
            pts.append((float(parts[0]), float(parts[1])))
    return np.array(pts, dtype=np.float64)


def path_arc_length(pts: np.ndarray) -> float:
    """Total arc-length of a polyline (same coordinate system as waypoints)."""
    if pts.ndim != 2 or pts.shape[0] < 2:
        return 0.0
    diffs = np.diff(pts, axis=0)
    return float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))


def interpolate_waypoints(pts: np.ndarray, spacing: float = 1.0) -> np.ndarray:
    """Resample a polyline at uniform arc-length spacing for smoother following."""
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


# ---------------------------
# Dead Reckoning & Path Tracking (No Odometry Coordinates)
# ---------------------------

class DeadReckoning:
    """Tracks total distance traveled based on commanded speed and time."""
    def __init__(self):
        self.total_dist = 0.0

    def update(self, speed_mps: float, dt: float):
        if speed_mps > 0:
            self.total_dist += speed_mps * dt
        return self.total_dist

class PathTracker:
    """Tracks progress along a path segment based on distance traveled."""
    def __init__(self, segment_length: float):
        self.total_length = segment_length
        self.start_dist = 0.0
        self.initialized = False

    def reset(self, current_total_dist: float):
        self.start_dist = current_total_dist
        self.initialized = True

    def update(self, current_total_dist: float) -> Tuple[float, float]:
        """Returns (dist_traveled_on_segment, dist_remaining)."""
        if not self.initialized:
            self.reset(current_total_dist)
        
        traveled = current_total_dist - self.start_dist
        remaining = max(0.0, self.total_length - traveled)
        return traveled, remaining


class OdometryPoseEstimator:
    """Dead-reckoned 2D pose with gyro + bicycle-rate fusion."""

    def __init__(self, x0: float, y0: float, th0: float, wheelbase_m: float = 0.20):
        self.x = float(x0)
        self.y = float(y0)
        self.th = float(th0)
        self.L = float(max(0.05, wheelbase_m))

    def update(self, speed_mps: float, steer_rad: float, gyro_z: float | None, dt: float) -> Tuple[float, float, float]:
        v = float(speed_mps) if np.isfinite(speed_mps) else 0.0
        delta = clamp(float(steer_rad) if np.isfinite(steer_rad) else 0.0, -0.6, 0.6)
        dt = float(max(1e-3, dt))

        yaw_rate_bicycle = v * math.tan(delta) / self.L
        if gyro_z is not None and np.isfinite(gyro_z):
            yaw_rate = 0.90 * float(yaw_rate_bicycle) + 0.10 * float(gyro_z)
        else:
            yaw_rate = float(yaw_rate_bicycle)

        self.th = wrap_pi(self.th + yaw_rate * dt)
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        return self.x, self.y, self.th


# ---------------------------
# Perception: Lane centering (explicit)
# ---------------------------


class LaneController:
    """Fit yellow/white boundaries and command lane-center steering.

    Sign convention: steer_raw positive = LEFT (internal).
    Output sign can be flipped with STEER_OUTPUT_SIGN.
    """

    def __init__(self):
        self.kernel = np.ones((5, 5), np.uint8)
        self.prev_err = 0.0

        # lane geometry estimate
        self.lane_width_px = 360.0
        # err = (mid - x_center) / mid, so positive err -> steer left.
        # x_center = geometric_center + bias_right_px.
        # To keep the car in the RIGHT portion of its lane, we want the
        # setpoint (x_center) to sit LEFT of image mid, i.e. bias_right_px < 0.
        # This makes err positive -> controller steers left -> car moves right
        # relative to the lane centre until it is correctly offset.
        # Tune this value: more negative = car sits further right in lane.
        self.bias_right_px = -28.0

        # control gains
        self.k_lat = 0.65
        self.k_head = 0.25
        self.kd = 0.06
        self.max_steer = 0.34

        # last debug
        self.last_dbg = None

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
    def _strip_x_range(mask: np.ndarray, y1: int, y2: int, x1: int, x2: int, *, min_pix: int = 120) -> float | None:
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
    def _boundary_pose(cls, mask: np.ndarray, roi_h: int) -> Tuple[float | None, Tuple[float, float] | None]:
        # Near-field x at the bottom, plus a direction vector (vx, vy) from two
        # horizontal strips. This is much more stable than a global line fit on
        # curved boundaries.
        x_bot = cls._strip_x(mask, roi_h - 26, roi_h - 2)
        x_mid = cls._strip_x(mask, roi_h - 92, roi_h - 64)
        if x_bot is None:
            return None, None
        if x_mid is None:
            return x_bot, None
        vx = float(x_mid - x_bot)
        vy = float((roi_h - 78) - (roi_h - 14))  # negative (upwards)
        # Match fitLine convention: make vy positive so "vertical" => +pi/2.
        if vy < 0.0:
            vx, vy = -vx, -vy
        # Avoid degenerate vectors.
        if abs(vy) < 1e-3:
            return x_bot, None
        return x_bot, (vx, vy)

    @classmethod
    def _boundary_pose_range(
        cls, mask: np.ndarray, roi_h: int, x1: int, x2: int, *, min_pix: int = 120
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

    def step(self, bgr: np.ndarray, dt: float, debug: bool = False):
        if bgr is None or bgr.size == 0:
            return 0.0, False, 0.0, 0.0, None, None, None, None

        if not np.isfinite(dt) or dt <= 0.0:
            dt = 1.0 / 60.0

        H, W = bgr.shape[:2]
        y0 = int(0.62 * H)
        roi = bgr[y0:H, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Yellow boundary
        y_mask = cv2.inRange(hsv, (10, 70, 90), (45, 255, 255))
        # White boundary
        h, s, v = cv2.split(hsv)
        w_mask = ((s < 70) & (v > 180)).astype(np.uint8) * 255

        # Clean up
        y_mask = cv2.morphologyEx(y_mask, cv2.MORPH_OPEN, self.kernel)
        y_mask = cv2.morphologyEx(y_mask, cv2.MORPH_CLOSE, self.kernel)
        w_mask = cv2.morphologyEx(w_mask, cv2.MORPH_OPEN, self.kernel)
        w_mask = cv2.morphologyEx(w_mask, cv2.MORPH_CLOSE, self.kernel)

        # Fit using only near-field pixels (bottom of ROI). This avoids bad
        # line direction estimates when the boundary curves across the image
        # (common at corners / when the yellow divider is near the center).
        roi_h, roi_w = roi.shape[:2]

        y_eval = float(roi_h - 6)

        # Side-aware boundary picks to avoid locking onto the taxi hub island
        # curb/markings. For right-hand driving we expect:
        #   - yellow (divider) mostly in LEFT half
        #   - white (curb) mostly in RIGHT half
        mid_x = int(0.5 * roi_w)

        y_xL, y_vL = self._boundary_pose_range(y_mask, roi_h, 0, mid_x)
        y_xR, y_vR = self._boundary_pose_range(y_mask, roi_h, mid_x, roi_w)
        w_xL, w_vL = self._boundary_pose_range(w_mask, roi_h, 0, mid_x)
        w_xR, w_vR = self._boundary_pose_range(w_mask, roi_h, mid_x, roi_w)

        # Default to the expected halves, but keep global fallbacks.
        y_x, y_v = (y_xL, y_vL) if (y_xL is not None) else (y_xR, y_vR)
        w_x, w_v = (w_xR, w_vR) if (w_xR is not None) else (w_xL, w_vL)

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

        # Prefer right-hand driving lane.
        # If both boundaries are seen and yellow appears to the RIGHT of white,
        # that usually means we've locked onto the LEFT lane (white outer edge
        # on the left, yellow divider on the right). In that case, use the
        # yellow divider as the LEFT boundary of the RIGHT lane.
        if (y_x is not None) and (w_x is not None):
            yxf = float(y_x)
            wxf = float(w_x)

            # Update lane width from the observed separation (either lane).
            w_est = float(abs(wxf - yxf))
            if 170.0 < w_est < 900.0:
                self.lane_width_px = 0.92 * self.lane_width_px + 0.08 * w_est

            if yxf <= wxf:
                # Right lane observed: yellow is left boundary, white is right.
                lane_mode = "R"
                xl = yxf
                xr = wxf
                x_center = 0.5 * (xl + xr) + float(self.bias_right_px)
            else:
                # Left lane observed: force right-lane center from the divider.
                lane_mode = "L->R"
                xl = yxf
                xr = xl + float(self.lane_width_px)
                x_center = float(xl + 0.5 * self.lane_width_px + self.bias_right_px)
                # Heading from the divider only (more stable than mixing).
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

        # Lane bounds (in ROI pixel coordinates)
        xl_use = float(np.clip(xl if xl is not None else (x_center - 0.5 * self.lane_width_px), 0.0, roi_w - 1.0))
        xr_use = float(np.clip(xr if xr is not None else (x_center + 0.5 * self.lane_width_px), 0.0, roi_w - 1.0))
        if xl_use > xr_use:
            xl_use, xr_use = xr_use, xl_use

        # lateral error normalized: + means car is right of center -> steer left
        mid = 0.5 * roi_w
        err = float((mid - x_center) / max(1.0, mid))
        derr = float((err - self.prev_err) / max(1e-3, dt))
        self.prev_err = err

        # heading error (use whichever boundary has a direction vector; average
        # if we have both).
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

        # simple confidence: how many pixels we see
        # Confidence from near-field coverage (what we actually use to steer).
        y_nf = y_mask[max(0, roi_h - 120) : roi_h, :]
        w_nf = w_mask[max(0, roi_h - 120) : roi_h, :]
        lane_pix = float((y_nf > 0).mean() + (w_nf > 0).mean())
        conf = clamp(lane_pix / 0.07, 0.0, 1.0)
        valid = bool(conf > 0.25)

        dbg = None
        if debug:
            dbg = roi.copy()
            cv2.circle(dbg, (int(x_center), int(y_eval)), 7, (0, 0, 255), -1)
            if xl is not None:
                cv2.line(dbg, (int(xl), 0), (int(xl), roi_h - 1), (0, 255, 255), 2)
            if xr is not None:
                cv2.line(dbg, (int(xr), 0), (int(xr), roi_h - 1), (255, 255, 255), 2)
            if lane_mode:
                cv2.putText(dbg, f"mode={lane_mode}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(dbg, f"conf={conf:.2f} err={err:+.2f} head={head_err:+.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(dbg, f"conf={conf:.2f} err={err:+.2f} head={head_err:+.2f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.last_dbg = dbg

        return float(steer), bool(valid), float(conf), float(err), dbg, float(xl_use), float(xr_use), int(y0)


# ---------------------------
# Sidewalk/curb guardrail from RGB (near-ground only)
# ---------------------------


class SidewalkGuard:
    def __init__(self):
        self.kernel = np.ones((5, 5), np.uint8)

    def step(self, bgr: np.ndarray, lane_y0: int | None, xl: float | None, xr: float | None) -> Tuple[float, float, float, np.ndarray | None]:
        if bgr is None or bgr.size == 0:
            return 0.0, 0.0, 0.0, None

        H, W = bgr.shape[:2]
        # Use the same ROI as lane detection if available.
        y0 = int(lane_y0) if (lane_y0 is not None) else int(0.72 * H)
        roi = bgr[y0:H, :]
        rH, rW = roi.shape[:2]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        s = hsv[:, :, 1].astype(np.float32)
        v = hsv[:, :, 2].astype(np.float32)
        L = lab[:, :, 0].astype(np.float32)

        # adaptive road reference from bottom-center
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

        # Conservative thresholds to avoid classifying the asphalt lane as
        # sidewalk under bright exposure.
        sw_rel = (L > (L_med + 52.0)) & (s < min(55.0, s_med + 18.0)) & (v > (v_med + 34.0))
        sw_abs = (L > 220.0) & (s < 45.0) & (v > 205.0)
        sw = (sw_rel | sw_abs).astype(np.uint8) * 255
        sw = cv2.morphologyEx(sw, cv2.MORPH_OPEN, self.kernel, iterations=1)
        sw = cv2.morphologyEx(sw, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        sw_bin = (sw > 0).astype(np.float32)

        # Compute sidewalk metrics relative to the detected lane corridor.
        # If lane bounds are missing, fall back to conservative zeros.
        if xl is None or xr is None or not np.isfinite(xl) or not np.isfinite(xr):
            sw_center, sw_bias, sw_near = 0.0, 0.0, 0.0
        else:
            xl_i = int(np.clip(xl, 0, rW - 1))
            xr_i = int(np.clip(xr, 0, rW - 1))
            if xl_i > xr_i:
                xl_i, xr_i = xr_i, xl_i
            pad = int(0.03 * rW)
            xL = int(np.clip(xl_i + pad, 0, rW - 1))
            xR = int(np.clip(xr_i - pad, 0, rW - 1))
            if xR <= xL + 10:
                sw_center, sw_bias, sw_near = 0.0, 0.0, 0.0
            else:
                band0 = int(0.45 * rH)
                near = sw_bin[band0:, :]
                # Sidewalk OUTSIDE the lane corridor (used for bias/repulsion)
                out_left = float(near[:, :xL].mean()) if xL > 5 else 0.0
                out_right = float(near[:, xR:].mean()) if xR < (rW - 5) else 0.0
                denom = out_left + out_right + 1e-6
                sw_bias = clamp((out_right - out_left) / denom, -1.0, 1.0)

                # Sidewalk INTRUSION inside lane corridor (used for stop)
                lane_w = max(1, xR - xL)
                core_pad = int(max(2, 0.18 * lane_w))
                cxL = xL + core_pad
                cxR = xR - core_pad
                if cxR <= cxL + 8:
                    cxL, cxR = xL, xR

                in_lane = near[:, cxL:cxR]
                sw_center = float(in_lane.mean())
                near0 = int(0.75 * rH)
                sw_near = float(sw_bin[near0:, cxL:cxR].mean())

        dbg = cv2.cvtColor(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
        dbg[sw > 0] = (0, 0, 255)  # red overlay for detected sidewalk/curb pixels
        if xl is not None and xr is not None and np.isfinite(xl) and np.isfinite(xr):
            xl_i = int(np.clip(xl, 0, rW - 1))
            xr_i = int(np.clip(xr, 0, rW - 1))
            cv2.line(dbg, (xl_i, 0), (xl_i, rH - 1), (0, 255, 255), 2)
            cv2.line(dbg, (xr_i, 0), (xr_i, rH - 1), (0, 255, 255), 2)
        cv2.putText(dbg, f"swC={sw_center:.2f} swN={sw_near:.2f} swB={sw_bias:+.2f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return float(sw_center), float(sw_bias), float(sw_near), dbg


# ---------------------------
# Depth/LiDAR safety
# ---------------------------


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
    # Exclude very bottom rows (hood/near-ground artifacts) which often
    # produce persistent false near obstacles at curves.
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
        # Reject invalid fills (near-zero) and very-near noise floor.
        win = win[np.isfinite(win) & (win > 0.32)]
        total_valid += int(win.size)
        mins.append(float(np.percentile(win, 8)) if win.size else float("inf"))

    left, center, right = mins
    min_front = float(min(left, center, right))
    # Require a minimum amount of valid depth samples; otherwise treat as invalid.
    valid = bool(np.isfinite(min_front) and (min_front > 0.32) and (min_front < 30.0) and (total_valid > 1200))

    invL = 0.0 if not np.isfinite(left) else 1.0 / max(left, 0.08)
    invR = 0.0 if not np.isfinite(right) else 1.0 / max(right, 0.08)
    bias = clamp(0.35 * (invR - invL), -0.35, 0.35)
    center_front = float(center)
    return float(min_front), float(center_front), float(bias), valid


def lidar_front(angles: np.ndarray, dist: np.ndarray) -> Tuple[float, float, float, float, bool]:
    if angles is None or dist is None:
        return float("inf"), float("inf"), 0.0, 0.0, False

    a = (-angles + np.pi)
    a = (a + np.pi) % (2.0 * np.pi) - np.pi

    d = dist.astype(np.float32, copy=False)
    ok = np.isfinite(d)

    f = ok & (np.abs(a) < math.radians(70.0))
    if not np.any(f):
        return float("inf"), float("inf"), 0.0, 0.0, False

    a = a[f]
    d = d[f]
    x = d * np.cos(a)
    y = d * np.sin(a)

    f2 = (x > 0.18) & (d < 8.0)
    if not np.any(f2):
        return float("inf"), float("inf"), 0.0, 0.0, False

    a = a[f2]
    y = y[f2]
    d = d[f2]

    # Robust "front" distance: use a narrower wedge and a low percentile
    # rather than the absolute min over a wide FOV, which can be dominated by
    # curbs/walls at the extreme left/right and freeze the vehicle.
    front = d[np.abs(a) < math.radians(35.0)]
    if front.size:
        min_front = float(np.percentile(front, 5))
    else:
        min_front = float(np.min(d))

    valid = bool(np.isfinite(min_front) and min_front > 0.05)

    # Center distance for hard-stop decisions
    center = d[np.abs(a) < math.radians(12.0)]
    center_front = float(np.percentile(center, 10)) if center.size else float("inf")

    left = d[y > 0]
    right = d[y < 0]
    dL = float(np.percentile(left, 10)) if left.size else float("inf")
    dR = float(np.percentile(right, 10)) if right.size else float("inf")

    invL = 0.0 if not np.isfinite(dL) else 1.0 / max(dL, 0.08)
    invR = 0.0 if not np.isfinite(dR) else 1.0 / max(dR, 0.08)
    bias = clamp(0.25 * (invR - invL), -0.30, 0.30)

    conf = 0.0
    if np.isfinite(dL) and np.isfinite(dR):
        conf = clamp(1.0 - abs(dL - dR) / max(dL + dR, 1e-3), 0.0, 1.0)

    return float(min_front), float(center_front), float(bias), float(conf), valid

    


# ---------------------------
# Actuation
# ---------------------------


def speed_to_throttle(speed_mps: float, brake: bool = False) -> float:
    """Convert desired speed to throttle.
    QCar2 has no separate brake channel — negative throttle drives in reverse.
    So we return 0.0 when we want to stop (coast to stop in simulation)."""
    if speed_mps <= 1e-3:
        return 0.0               # coast to stop; NEVER send negative (= reverse)
    speed_mps = max(0.0, float(speed_mps))
    return clamp(0.05 + 0.08 * speed_mps, 0.06, 0.30)


def render_hud(bgr: np.ndarray, title: str, lines: list[str]) -> np.ndarray:
    img = bgr.copy()
    cv2.putText(img, title, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 255), 3)
    y = 75
    for ln in lines:
        cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        y += 22
    return img


# ---------------------------
# Runner
# ---------------------------


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
    # Steering output sign (kept aligned with internal convention:
    # steer_raw > 0 means LEFT)
    STEER_OUTPUT_SIGN = 1.0

    paths = load_waypoints_txt(waypoints_file)
    required = ["path_to_pickup", "path_to_dropoff", "path_to_hub"]
    for r in required:
        if r not in paths:
            raise RuntimeError(f"Missing {r} in {waypoints_file}. Found: {list(paths.keys())}")

    P_pick_raw = paths["path_to_pickup"]
    P_drop_raw = paths["path_to_dropoff"]
    P_hub_raw = paths["path_to_hub"]

    # Interpolate waypoints at ~1 FS-unit spacing for smooth Pure-Pursuit
    P_pick = interpolate_waypoints(P_pick_raw, spacing=1.0)
    P_drop = interpolate_waypoints(P_drop_raw, spacing=1.0)
    P_hub  = interpolate_waypoints(P_hub_raw,  spacing=1.0)

    if debug_print:
        print(f"  interpolated waypoints: pickup={P_pick.shape[0]}  dropoff={P_drop.shape[0]}  hub={P_hub.shape[0]}")

    # [EKF and Stanley removed for reactive nav]
    
    lane = LaneController()
    swg = SidewalkGuard()

    car = QCar(readMode=1, frequency=int(sample_rate_hz))
    cam = Camera2D(cameraId="3@tcpip://localhost:18964", frameWidth=820, frameHeight=410, frameRate=sample_rate_hz)

    # QLabs LED strip control 
    _led_ctrl = _QLabsLEDDirect(host="localhost", port=18000, actor=actor_number)
    if debug_print:
        if _led_ctrl.connected:
            print("[INFO] QLabs LED strip TCP connection OK")
        else:
            print("[WARN] QLabs LED strip TCP connection failed – LEDs won't change colour")
    _prev_led_rgb = (-1.0, -1.0, -1.0)  # force first write

    rs = QCarRealSense(mode="RGB, Depth") if use_realsense else None
    lidar = QCarLidar(numMeasurements=1000, rangingDistanceMode=2, interpolationMode=0) if use_lidar else None
    lidar_warn_t = 0.0

    # Sign detector (used only for roundabout entry cue).
    # This is intentionally lightweight: run at low rate and only latch the
    # roundabout cue for a short time window.
    sign_model_obj = None
    sign_names: list[str] = []
    sign_stride = 6  # 30Hz -> 5Hz
    sign_frame = 0
    rb_cue_until = 0.0
    rb_entry_until = 0.0
    rb_exit_until = 0.0
    rb_dist_m = 0.0
    rb_last_score = 0.0
    rb_sign_events = 0
    rb_first_sequence_active = False
    dropoff_sign_triggered = False

    # --- Sign action state (stop / yield / traffic lights) ---
    STOP_SIGN_DWELL_S   = 2.0   # required *stopped* time at a stop sign
    YIELD_SLOW_S        = 2.0   # slow period after yield sign
    YIELD_SPEED         = 0.6   # max speed while yielding (m/s)
    TL_HOLD_S           = 4.0   # traffic light action hold
    TL_YELLOW_SPEED     = 0.55  # max speed on yellow
    SIGN_COOLDOWN_S     = 12.0  # re-trigger cooldown per sign

    stop_sign_active    = False # True while car must stay stopped at stop sign
    stop_sign_stopped_t = 0.0   # accumulated seconds the car has been stopped
    stop_sign_detect_t  = 0.0   # wall-time when stop sign was detected
    yield_slow_until    = 0.0   # yield slow until this time
    tl_red_until        = 0.0   # TL red stop until
    tl_yellow_until     = 0.0   # TL yellow slow until
    tl_green_until      = 0.0   # TL green (overrides red/yellow)
    last_stop_t         = -999.0
    last_yield_t        = -999.0
    last_tl_red_t       = -999.0
    last_tl_yellow_t    = -999.0
    last_tl_green_t     = -999.0
    try:
        if sign_model and sign_labels:
            sign_names = [ln.strip() for ln in open(sign_labels, "r", encoding="utf-8").read().splitlines() if ln.strip()]
            from ultralytics import YOLO  # type: ignore

            sign_model_obj = YOLO(sign_model)
    except Exception:
        sign_model_obj = None
        sign_names = []


    # --- Route planner: odometry + Stanley on waypoint segments ---
    WAYPOINT_SCALE_M = 0.10  # map exported at x10 scale
    route_segments: list[tuple[str, np.ndarray]] = [
        ("TO_PICKUP", np.array(P_pick, dtype=np.float64) * WAYPOINT_SCALE_M),
        ("TO_DROPOFF", np.array(P_drop, dtype=np.float64) * WAYPOINT_SCALE_M),
        ("TO_HUB", np.array(P_hub, dtype=np.float64) * WAYPOINT_SCALE_M),
    ]
    segment_idx = 0
    segment_hold_until = 0.0
    SEGMENT_HOLD_S = 2.0  # Full stop at pickup/dropoff per competition rules
    route_done = False

    first_wp = route_segments[0][1]
    if first_wp.shape[0] < 2:
        raise RuntimeError("Waypoint route needs at least 2 points in first segment.")

    # Use actual spawn heading from Setup_Real_Scenario_fullscale_x10.py: -44.7 degrees
    # This must match the car's real initial orientation, not the waypoint direction
    SPAWN_HEADING_DEG = -44.7
    th0 = math.radians(SPAWN_HEADING_DEG)
    odom = OdometryPoseEstimator(x0=float(first_wp[0, 0]), y0=float(first_wp[0, 1]), th0=float(th0), wheelbase_m=0.20)

    gps = None
    if QCarGPS is not None:
        try:
            gps = QCarGPS(initialPose=np.array([odom.x, odom.y, odom.th], dtype=np.float64))
        except Exception:
            gps = None

    active_name, active_wp = route_segments[segment_idx]
    seg_max_idx = 0  # monotonic progress along current segment (pose-based)
    stanley = StanleyController(waypoints=active_wp.T, k=1.15, cyclic=False)
    stanley.maxSteeringAngle = 0.42

    # Dead Reckoning
    dr = DeadReckoning()
    dr.total_dist = 0.0



    # Speed cap slider
    # speed_ui_ready = False
    # speed_cap_mps = float(max_speed_mps)
    # speed_cap_max_mps = float(max(0.5, max_speed_mps))
    # speed_trackbar_max = int(round(speed_cap_max_mps * 100.0))
    # speed_trackbar_name = "speed_cap_x100"

    def _noop(_v: int):
        return

    # Guardrail thresholds (strict)
    STOP_FRONT_M = 0.75
    SLOW_FRONT_M = 1.60
    CENTER_STOP_M = 0.55
    CENTER_SLOW_M = 1.00
    SIDE_HARD_STOP_M = 0.30
    SIDE_BYPASS_CLEAR_M = 1.50
    HIGH_STEER_SPEED_CAP = 0.45
    MOD_STEER_SPEED_CAP = 0.70
    OBSTACLE_STOP_CONFIRM_S = 0.22

    # If lane is lost, crawl forward slowly to re-acquire (still respects
    # obstacle + sidewalk stops). Without this, the car can deadlock at start.
    LANE_ACQUIRE_SPEED = 0.35
    LANE_LOST_SLOW_SPEED = 0.18
    LANE_LOST_STOP_S = 1.20
    LANE_DEPART_ERR_SLOW = 0.45
    LANE_DEPART_ERR_STOP = 0.70
    LANE_DEPART_STEER_MAX = 0.38
    LANE_DEPART_ERR_LANE_ONLY = 0.90
    LANE_DEPART_RECOVER_SPEED = 0.20
    LANE_DEPART_RECOVER_CLEAR_M = 1.20
    LANE_DEPART_STOP_GRACE_S = 0.75
    TURN_ACTIVE_STEER_ON = 0.14
    TURN_ACTIVE_HOLD_S = 0.70

    # Sidewalk thresholds – relaxed so the car doesn't freeze on
    # bright concrete, curb edges, or hub markings.
    SW_SLOW = 0.10
    SW_STOP = 0.18
    SW_NEAR_STOP = 0.12

    # If sidewalk detector trips but center path is clear, creep/steer away
    # instead of dead-stopping (prevents getting stuck hugging a curb/median).
    SW_RECOVER_BIAS = 0.18
    SW_RECOVER_CLEAR_M = 1.20
    SW_RECOVER_SPEED = 0.22
    SW_RECOVER_LANE_ERR = 0.55
    SW_RECOVER_LANE_STEER = 0.18
    SW_ESCAPE_LANE_ERR = 0.16
    SW_STRONG_CONFIRM_S = 0.35
    SW_PROBE_SPEED = 0.16
    SW_PROBE_CLEAR_M = 1.10

    # weights
    W_SW = 0.55
    W_OBS = 0.90

    # Roundabout helper: encourage counter-clockwise circulation when inside
    # the roundabout region (lane markings can be ambiguous at entry/exit).
    # Center derived from DetailedScenario_cursor/Setup_Real_Scenario_fullscale_x10.py
    RB_CX = 1.03
    RB_CY = 2.99
    RB_R_IN = 0.85
    RB_R_OUT = 3.20
    W_RB = 0.55

    # If we detect a roundabout sign, force a keep-right bias regardless of
    # dead-reckoning. This is the primary protection against entering the
    # roundabout the wrong way.
    RB_CUE_HOLD_S = 5.0
    RB_ENTRY_HOLD_S = 1.6
    RB_KEEP_RIGHT_STEER = -0.16
    RB_ENTRY_STEER = -0.24
    RB_EXIT_DIST_M = 2.4
    RB_EXIT_HOLD_S = 1.1
    RB_POST_EXIT_HOLD_S = 2.0
    RB_EXIT_STEER = -0.38

    # Pure Pursuit is now replaced by StanleyController; these old waypoint-
    # heading bias weights are superseded but kept as fallback constants.
    W_WP_BASE = 0.00
    W_WP_TURN = 0.0   # disabled: Stanley handles this
    WP_LOOKAHEAD = 22
    WP_TURN_ENABLE_RAD = math.radians(12.0)

    # Blending weights: Stanley (primary) + Lane (secondary)
    W_STANLEY = 0.97  # weight for Stanley waypoint steering
    W_LANE    = 0.03  # weight for lane-centering correction
    TURN_LANE_BLEND_MAX = 0.01
    LANE_ONLY_CONF_MIN = 0.72
    LANE_ONLY_MAX_PLAN_STEER = 0.12

    # smoothing at stop
    steer_filt = 0.0
    steer_out  = 0.0   # last steering command sent (used by EKF on next tick)
    # Kinematic/odometry sign convention uses steer_raw (>0 means LEFT).
    # steer_out may be inverted for hardware, so do NOT feed steer_out into the bicycle model.
    steer_model_prev = 0.0

    dt_nom = 1.0 / float(sample_rate_hz)
    t_prev = now()
    sidewalk_stop_streak_s = 0.0
    sidewalk_strong_streak_s = 0.0
    lane_lost_streak_s = 0.0
    lane_depart_stop_streak_s = 0.0
    obstacle_stop_streak_s = 0.0
    sw_latch_until = 0.0
    turn_latch_until = 0.0
    # IO timeout/backoff handling (QLabs sockets can intermittently time out)
    last_bgr = None
    cam_fail = 0
    car_fail = 0
    rs_fail = 0
    next_cam_try_t = 0.0
    next_car_try_t = 0.0
    next_rs_try_t = 0.0

    def _is_timeout_exc(e: BaseException) -> bool:
        msg = str(e).lower()
        return (
            isinstance(e, (TimeoutError, OSError))
            and ("timed out" in msg or "timeout" in msg or "10060" in msg)
        )

    if debug_print:
        print(f"[INFO] RESTART runner starting segment={active_name}")

    speed_cmd = 0.0
    START_MAGENTA_S = 1.2
    start_magenta_until = now() + START_MAGENTA_S
    try:
        while True:
            t = now()
            dt = t - t_prev
            t_prev = t
            if not np.isfinite(dt) or dt <= 0.0:
                dt = dt_nom

            # --- Device reads with timeout/backoff
            if t >= next_car_try_t:
                try:
                    car.read()
                    car_fail = 0
                except Exception as e:
                    if _is_timeout_exc(e):
                        car_fail += 1
                        next_car_try_t = t + min(1.0, 0.05 * car_fail)
                    else:
                        raise

            bgr = None
            if t >= next_cam_try_t:
                try:
                    cam.read()
                    bgr = cam.imageData
                    if bgr is not None and getattr(bgr, "size", 0):
                        last_bgr = bgr
                    cam_fail = 0
                except Exception as e:
                    if _is_timeout_exc(e):
                        cam_fail += 1
                        next_cam_try_t = t + min(1.5, 0.08 * cam_fail)
                        # If camera is repeatedly timing out, try reconnecting.
                        if cam_fail in (25, 60):
                            try:
                                if hasattr(cam, "terminate"):
                                    cam.terminate()
                            except Exception:
                                pass
                            try:
                                cam = Camera2D(
                                    cameraId="3@tcpip://localhost:18964",
                                    frameWidth=820,
                                    frameHeight=410,
                                    frameRate=sample_rate_hz,
                                )
                            except Exception:
                                pass
                    else:
                        raise

            if bgr is None or not getattr(bgr, "size", 0):
                bgr = last_bgr

            depth_px = None
            if rs is not None:
                if t >= next_rs_try_t:
                    try:
                        rs.read_RGB()
                        rs.read_depth(dataMode="PX")
                        rs_fail = 0
                    except Exception as e:
                        if _is_timeout_exc(e):
                            rs_fail += 1
                            next_rs_try_t = t + min(1.5, 0.08 * rs_fail)
                        else:
                            raise
                depth_px = getattr(rs, "imageBufferDepthPX", None)

            lidar_angles = None
            lidar_dist = None
            if lidar is not None:
                try:
                    lidar.read()
                    lidar_angles = getattr(lidar, "angles", None)
                    lidar_dist = getattr(lidar, "distances", None)
                except Exception as e:
                    if debug_print and (t - lidar_warn_t) > 2.0:
                        lidar_warn_t = t
                        print(f"[WARN] LiDAR read failed ({type(e).__name__}: {e}). Skipping frame.")
                    lidar_angles = None
                    lidar_dist = None

            # Get raw tachometer speed and gyro for odometry/navigation.
            v_est = float(getattr(car, "motorTach", 0.0))
            dr_speed = max(0.0, v_est) if np.isfinite(v_est) else 0.0
            dr.update(dr_speed, dt)

            gyro = getattr(car, "gyroscope", None)
            gyro_z = None
            if gyro is not None and np.size(gyro) >= 3:
                try:
                    gyro_z = float(gyro[2])
                except Exception:
                    gyro_z = None
            # Pose update: prefer QCarGPS when available (more reliable segment completion),
            # fall back to odometry (tach + gyro + last steering).
            gps_ok = False
            if gps is not None:
                try:
                    gps_ok = bool(gps.readGPS())
                except Exception:
                    gps_ok = False

            if gps_ok:
                try:
                    pose_x = float(gps.position[0])
                    pose_y = float(gps.position[1])
                    pose_th = float(gps.orientation[2]) if hasattr(gps, "orientation") else float(odom.th)
                    # Keep odom aligned so downstream code has consistent state.
                    odom.x, odom.y, odom.th = pose_x, pose_y, pose_th
                except Exception:
                    gps_ok = False

            if not gps_ok:
                pose_x, pose_y, pose_th = odom.update(
                    speed_mps=dr_speed,
                    steer_rad=steer_model_prev,
                    gyro_z=gyro_z,
                    dt=dt,
                )
            # Segment transition: use pose-based completion as the primary trigger.
            # (StanleyController.pathComplete can be brittle if waypoint formatting or
            # pose integration is slightly off.)
            if not route_done:
                # nearest waypoint index (monotonic progress)
                try:
                    d2 = (active_wp[:, 0] - pose_x) ** 2 + (active_wp[:, 1] - pose_y) ** 2
                    i_near = int(np.argmin(d2))
                    seg_max_idx = max(seg_max_idx, i_near)
                except Exception:
                    i_near = 0

                goal_x = float(active_wp[-1, 0])
                goal_y = float(active_wp[-1, 1])
                dist_to_goal_now = float(math.hypot(pose_x - goal_x, pose_y - goal_y))

                # Consider the segment complete if we have progressed to the last few
                # waypoints AND we are within a goal radius.
                SEG_GOAL_RADIUS_M = 0.45
                SEG_TAIL_WP_COUNT = 5
                seg_pose_complete = bool(
                    (seg_max_idx >= max(0, active_wp.shape[0] - SEG_TAIL_WP_COUNT))
                    and (dist_to_goal_now <= SEG_GOAL_RADIUS_M)
                )

                if (t >= segment_hold_until) and (seg_pose_complete or bool(stanley.pathComplete)):
                    segment_idx += 1
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

            action_type = "follow"
            action_val = 0.0
            stop_reason = ""
            turning_active = False

            if route_done:
                step_name = "DONE"
                target_steer = 0.0
                speed_cmd = 0.0
                action_type = "done"
                turn_latch_until = t
            else:
                step_name = active_name
                v_ctrl = max(0.20, dr_speed)
                target_steer = float(stanley.update(np.array([pose_x, pose_y], dtype=np.float64), pose_th, v_ctrl))
                target_steer = clamp(target_steer, -0.42, 0.42)

                goal_x = float(active_wp[-1, 0])
                goal_y = float(active_wp[-1, 1])
                dist_to_goal = float(math.hypot(pose_x - goal_x, pose_y - goal_y))

                speed_cmd = float(max_speed_mps)
                if dist_to_goal < 1.60:
                    speed_cmd = min(speed_cmd, 0.65)
                if dist_to_goal < 0.90:
                    speed_cmd = min(speed_cmd, 0.42)

                abs_plan_steer = abs(target_steer)
                if abs_plan_steer > 0.30:
                    speed_cmd = min(speed_cmd, 0.45)
                elif abs_plan_steer > 0.20:
                    speed_cmd = min(speed_cmd, 0.65)

                if abs_plan_steer >= TURN_ACTIVE_STEER_ON:
                    turn_latch_until = max(turn_latch_until, t + TURN_ACTIVE_HOLD_S)
                turning_active = bool(t < turn_latch_until)

                if t < segment_hold_until:
                    speed_cmd = 0.0
                    action_type = "hold"

                # Competition rule: start in magenta at the taxi hub before
                # switching to green to depart toward pickup.
                if t < start_magenta_until:
                    speed_cmd = 0.0
                    action_type = "hub_wait"

            # speed slider
            # if not speed_ui_ready:
            #     try:
            #         cv2.namedWindow("sign_debug", cv2.WINDOW_NORMAL)
            #         cv2.createTrackbar(
            #             speed_trackbar_name,
            #             "sign_debug",
            #             int(round(speed_cap_mps * 100.0)),
            #             max(1, speed_trackbar_max),
            #             _noop,
            #         )
            #         speed_ui_ready = True
            #     except Exception:
            #         speed_ui_ready = False

            # if speed_ui_ready:
            #     try:
            #         pos = int(cv2.getTrackbarPos(speed_trackbar_name, "sign_debug"))
            #         speed_cap_mps = clamp(pos / 100.0, 0.0, speed_cap_max_mps)
            #     except Exception:
            #         pass

            # lane
            lane_steer, lane_ok, lane_conf, lane_err, lane_dbg, xl, xr, lane_y0 = lane.step(bgr, dt, debug=True)

            # --- Sign detection logic (reused) ---
            SIGN_MIN_AREA_FRAC_STOP  = 0.008   # ~0.8% for stop / TL
            SIGN_MIN_AREA_FRAC_YIELD = 0.010   # ~1.0% for yield
            YIELD_AS_STOP_AREA_FRAC = 0.018   # ~1.8% box -> treat yield as stop
            _bgr_area = float((bgr.shape[0] * bgr.shape[1]) if bgr is not None else 1)

            rb_sign_active = False
            if sign_model_obj is not None and bgr is not None and getattr(bgr, "size", 0):
                try:
                    sign_frame += 1
                    if (sign_frame % int(max(1, sign_stride))) == 0:
                        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                        results = sign_model_obj.predict(source=rgb, verbose=False, conf=float(sign_conf), device=str(sign_device))
                        r0 = results[0]
                        if getattr(r0, "boxes", None) is not None and len(r0.boxes):
                            clss  = r0.boxes.cls.cpu().numpy().astype(int)
                            confs = r0.boxes.conf.cpu().numpy().astype(float)
                            xyxys = r0.boxes.xyxy.cpu().numpy()  # shape (N,4)
                            for cid, score, xyxy in zip(clss, confs, xyxys):
                                sname = sign_names[cid] if (0 <= int(cid) < len(sign_names)) else str(cid)
                                sc = float(score)
                                if sc < float(sign_conf):
                                    continue

                                # bounding box area fraction
                                bw = float(xyxy[2] - xyxy[0])
                                bh = float(xyxy[3] - xyxy[1])
                                box_frac = (bw * bh) / max(_bgr_area, 1.0)

                                if sname == "roundabout":
                                    rb_last_score = sc
                                    new_rb_event = bool(t >= rb_cue_until)
                                    if new_rb_event:
                                        rb_sign_events += 1
                                        if rb_sign_events == 1:
                                            rb_first_sequence_active = True
                                    if t >= rb_cue_until:
                                        rb_dist_m = 0.0
                                        rb_exit_until = 0.0
                                    rb_cue_until = max(rb_cue_until, t + RB_CUE_HOLD_S)
                                    rb_entry_until = max(rb_entry_until, t + RB_ENTRY_HOLD_S)

                                elif sname in ("stop", "yield"):
                                    if box_frac >= SIGN_MIN_AREA_FRAC_STOP:
                                        real_cls = classify_sign_color(bgr, xyxy)
                                        if real_cls == "stop":
                                            if (t - last_stop_t) > SIGN_COOLDOWN_S:
                                                stop_sign_active = True
                                                stop_sign_stopped_t = 0.0
                                                stop_sign_detect_t = t
                                                last_stop_t = t
                                                if debug_print:
                                                    print(f"[SIGN] STOP (model={sname} color=stop conf={sc:.2f} box={box_frac:.4f}) -> full stop {STOP_SIGN_DWELL_S:.1f}s")
                                        else:  # real_cls == "yield"
                                            if (t - last_yield_t) > SIGN_COOLDOWN_S:
                                                yield_slow_until = max(yield_slow_until, t + YIELD_SLOW_S)
                                                last_yield_t = t
                                                if debug_print:
                                                    print(f"[SIGN] YIELD (model={sname} color=yield conf={sc:.2f} box={box_frac:.4f}) -> slowing {YIELD_SLOW_S:.1f}s")

                                elif sname == "traffic_light_red":
                                    if box_frac >= SIGN_MIN_AREA_FRAC_STOP:
                                        if (t - last_tl_red_t) > SIGN_COOLDOWN_S:
                                            tl_red_until = max(tl_red_until, t + TL_HOLD_S)
                                            tl_green_until = 0.0
                                            last_tl_red_t = t
                                            if debug_print:
                                                print(f"[SIGN] TL RED detected (conf={sc:.2f} box={box_frac:.4f})")

                                elif sname == "traffic_light_yellow":
                                    if box_frac >= SIGN_MIN_AREA_FRAC_STOP:
                                        if (t - last_tl_yellow_t) > SIGN_COOLDOWN_S:
                                            tl_yellow_until = max(tl_yellow_until, t + TL_HOLD_S)
                                            last_tl_yellow_t = t
                                            if debug_print:
                                                print(f"[SIGN] TL YELLOW detected (conf={sc:.2f} box={box_frac:.4f})")

                                elif sname == "traffic_light_green":
                                    if box_frac >= SIGN_MIN_AREA_FRAC_STOP:
                                        # Green clears any active red/yellow hold
                                        tl_red_until = 0.0
                                        tl_yellow_until = 0.0
                                        tl_green_until = max(tl_green_until, t + TL_HOLD_S)
                                        last_tl_green_t = t
                                        if debug_print:
                                            print(f"[SIGN] TL GREEN detected (conf={sc:.2f} box={box_frac:.4f})")
                except Exception:
                    pass

            if t < rb_cue_until:
                rb_sign_active = True

            # Roundabout bias logic (Simplified without Pose)
            rb_w = 0.0
            rb_bias = 0.0
            if rb_sign_active:
                rb_w = 1.0
                rb_bias = float(RB_KEEP_RIGHT_STEER)
                if t < rb_entry_until:
                    rb_bias = float(RB_ENTRY_STEER)
                elif t < rb_exit_until:
                    rb_bias = float(RB_EXIT_STEER)


            # Check for sidewalk
            swC, swB, swN, sw_dbg = swg.step(bgr, lane_y0, xl, xr)
            
            # --- Steering Mixer ---
            if lane_ok:
                # During turns/intersections, rely almost entirely on waypoint
                # tracking; lane markings are often ambiguous there.
                lane_w_eff = W_LANE
                stanley_w_eff = W_STANLEY
                if turning_active or (abs(float(target_steer)) >= TURN_ACTIVE_STEER_ON):
                    lane_w_eff = TURN_LANE_BLEND_MAX
                    stanley_w_eff = 1.0 - lane_w_eff

                steer_raw = clamp((stanley_w_eff * target_steer) + (lane_w_eff * lane_steer), -0.45, 0.45)

                # Lane-only takeover is allowed only on near-straight, strong-
                # confidence segments to avoid grabbing the wrong lane at intersections.
                if (
                    (abs(float(lane_err)) > LANE_DEPART_ERR_LANE_ONLY)
                    and (lane_conf >= LANE_ONLY_CONF_MIN)
                    and (abs(float(target_steer)) <= LANE_ONLY_MAX_PLAN_STEER)
                    and (not turning_active)
                ):
                    steer_raw = clamp(lane_steer, -0.34, 0.34)
            else:
                steer_raw = clamp(target_steer, -0.40, 0.40)

            if rb_w > 0.0:
                steer_raw = clamp((1.0 - rb_w) * steer_raw + rb_w * rb_bias, -0.45, 0.45)

            # Treat sustained steering demand as active turning so lane-loss /
            # lane-departure stop logic does not deadlock the vehicle in curves.
            # Consider both mixed steer and planner steer when detecting
            # active turning. This avoids false "straight" classification
            # when lane cues are weak at curves.
            if max(abs(float(steer_raw)), abs(float(target_steer))) >= TURN_ACTIVE_STEER_ON:
                turn_latch_until = max(turn_latch_until, t + TURN_ACTIVE_HOLD_S)
            turning_active = bool((t < turn_latch_until) or rb_sign_active)
            
            # --- Safety Overrides ---

            # Lidar/Depth safety first so sidewalk logic can decide whether to
            # hard-stop or use a controlled creep/recovery.
            depth_min, depth_center_front, depth_bias, depth_valid = depth_front(depth_px) if depth_px is not None else (99.0, float("inf"), 0.0, False)
            lidar_min, lidar_center_front, lidar_bias, _, lidar_valid = lidar_front(lidar_angles, lidar_dist) if (lidar_angles is not None and lidar_dist is not None) else (99.0, float("inf"), 0.0, 0.0, False)

            min_front = min(depth_min, lidar_min)
            center_front = min(depth_center_front, lidar_center_front)
            front_valid = bool(depth_valid or lidar_valid)

            # Fused obstacle-side bias: + steer left, - steer right.
            obs_bias_terms = []
            if depth_valid:
                obs_bias_terms.append(float(depth_bias))
            if lidar_valid:
                obs_bias_terms.append(float(lidar_bias))
            obs_bias = float(sum(obs_bias_terms) / len(obs_bias_terms)) if obs_bias_terms else 0.0

            # False-positive guard: if sidewalk detector saturates the entire
            # lane corridor while lane fit is strong and front is clear, treat
            # it as an exposure/segmentation failure for this frame.
            sidewalk_fp = bool(
                lane_ok
                and (lane_conf > 0.72)
                and (center_front > 1.05)
                and (swC > 0.80)
                and (swN > 0.72)
            )
            if sidewalk_fp:
                swC = 0.0
                swN = 0.0

            # Sidewalk guard (strict): never continue when lane corridor is
            # heavily intruded by sidewalk/curb pixels.
            sidewalk_strong = bool((swC > SW_STOP) or (swN > SW_NEAR_STOP))
            sidewalk_soft = bool((swC > SW_SLOW) or (swN > 0.5 * SW_NEAR_STOP))

            if sidewalk_strong:
                sidewalk_strong_streak_s += dt
            else:
                sidewalk_strong_streak_s = 0.0
            sidewalk_confirmed = bool(sidewalk_strong and (sidewalk_strong_streak_s >= SW_STRONG_CONFIRM_S))

            # Latch: once sidewalk intrusion is detected, stay in recovery for a short
            # window (prevents oscillating on the curb boundary).
            if sidewalk_confirmed:
                sw_latch_until = max(sw_latch_until, t + 1.0)
            if t < sw_latch_until:
                sidewalk_soft = True

            startup_unstick = bool(
                sidewalk_strong
                and lane_ok
                and (dr.total_dist < 0.20)
                and (center_front > 1.30)
            )

            if startup_unstick:
                speed_cmd = min(speed_cmd, 0.22)
                stop_reason = "startup_unstick"
                steer_raw = clamp(lane_steer if lane_ok else 0.0, -0.35, 0.35)
            elif sidewalk_confirmed:
                sidewalk_stop_streak_s += dt

                # If we are hard-stopped by sidewalk for too long while front
                # is clear, creep forward very slowly to re-acquire lane/ROI.
                if (
                    ((lane_ok and (sidewalk_stop_streak_s > 0.6)) or turning_active or (sidewalk_stop_streak_s > 1.2))
                    and (center_front > 1.20)
                ):
                    speed_cmd = min(speed_cmd, 0.20)
                    stop_reason = "sidewalk_curve_recover" if turning_active else "sidewalk_unstick"
                    if lane_ok:
                        steer_raw = clamp(lane_steer + 0.35 * swB, -0.35, 0.35)
                    else:
                        steer_raw = clamp(target_steer + 0.25 * swB, -0.38, 0.38)
                else:
                    speed_cmd = 0.0
                    stop_reason = "sidewalk_strong"
                    if swB > 0:
                        steer_raw = 0.45
                    elif swB < 0:
                        steer_raw = -0.45
            elif sidewalk_strong and (center_front > SW_PROBE_CLEAR_M):
                sidewalk_stop_streak_s = 0.0
                speed_cmd = min(speed_cmd, SW_PROBE_SPEED)
                if not stop_reason:
                    stop_reason = "sidewalk_probe"
                if lane_ok:
                    steer_raw = clamp(lane_steer + 0.25 * swB, -0.30, 0.30)
                else:
                    steer_raw = clamp(target_steer + 0.20 * swB, -0.32, 0.32)
            elif sidewalk_soft:
                sidewalk_stop_streak_s = 0.0
                speed_cmd = min(speed_cmd, 0.35)
                steer_raw = clamp(steer_raw + 0.35 * swB, -0.45, 0.45)
            else:
                sidewalk_stop_streak_s = 0.0

            # Lane-keeping guardrail: if lane is lost, never run full speed.
            if lane_ok:
                lane_lost_streak_s = 0.0
            else:
                lane_lost_streak_s += dt

            if (not turning_active) and (not lane_ok):
                speed_cmd = min(speed_cmd, LANE_LOST_SLOW_SPEED)
                if not stop_reason:
                    stop_reason = "lane_lost_slow"
                # Keep steering mild and biased away from nearby obstacles/curbs.
                steer_raw = clamp(0.20 * swB + 0.65 * obs_bias, -0.24, 0.24)

                # After sustained lane loss, stop unless front is confidently clear.
                if lane_lost_streak_s > LANE_LOST_STOP_S:
                    if (not front_valid) or (min_front < SLOW_FRONT_M):
                        speed_cmd = 0.0
                        stop_reason = "lane_lost_stop"

            # Lane departure guardrail: if we are far from lane center, recover hard.
            if (not turning_active) and lane_ok:
                abs_lane_err = abs(float(lane_err))
                if abs_lane_err >= LANE_DEPART_ERR_STOP:
                    lane_depart_stop_streak_s += dt
                    steer_raw = clamp(lane_steer, -LANE_DEPART_STEER_MAX, LANE_DEPART_STEER_MAX)

                    # Recover by creeping when center path is clear; only full-stop
                    # briefly to avoid driving away with unstable perception.
                    if front_valid and (center_front > LANE_DEPART_RECOVER_CLEAR_M):
                        speed_cmd = min(speed_cmd, LANE_DEPART_RECOVER_SPEED)
                        stop_reason = "lane_departure_recover"
                    elif lane_depart_stop_streak_s <= LANE_DEPART_STOP_GRACE_S:
                        speed_cmd = 0.0
                        stop_reason = "lane_departure_stop"
                    else:
                        speed_cmd = min(speed_cmd, 0.16)
                        stop_reason = "lane_departure_creep"
                elif abs_lane_err >= LANE_DEPART_ERR_SLOW:
                    lane_depart_stop_streak_s = 0.0
                    speed_cmd = min(speed_cmd, 0.25)
                    if not stop_reason:
                        stop_reason = "lane_departure_slow"
                    steer_raw = clamp(lane_steer, -0.35, 0.35)
                else:
                    lane_depart_stop_streak_s = 0.0
            else:
                lane_depart_stop_streak_s = 0.0
            # If only side sectors are close but center is clear, bypass slowly
            # instead of dead-stopping.
            side_obstacle_only = bool(
                front_valid
                and lane_ok
                and (lane_conf > 0.70)
                and (center_front > SIDE_BYPASS_CLEAR_M)
                and (min_front < 0.55)
            )
            if side_obstacle_only:
                speed_cmd = min(speed_cmd, 0.35)
                steer_raw = clamp(steer_raw + 0.22 * obs_bias, -0.35, 0.35)
                if not stop_reason:
                    stop_reason = "side_obstacle_bypass"

            # Hard obstacle stop/slow with debounce + sensor agreement.
            hard_stop_raw = False
            if front_valid:
                both_valid = bool(depth_valid and lidar_valid)
                # Require stronger evidence when only depth is valid because
                # depth is more prone to near-ground false positives.
                if both_valid:
                    hard_stop_raw = bool(
                        (center_front < CENTER_STOP_M)
                        or ((min_front < SIDE_HARD_STOP_M) and (center_front < CENTER_SLOW_M))
                    )
                elif lidar_valid:
                    hard_stop_raw = bool(
                        (lidar_center_front < CENTER_STOP_M)
                        or ((lidar_min < SIDE_HARD_STOP_M) and (lidar_center_front < CENTER_SLOW_M))
                    )
                elif depth_valid:
                    hard_stop_raw = bool(
                        (depth_center_front < (CENTER_STOP_M - 0.15))
                        or ((depth_min < (SIDE_HARD_STOP_M - 0.10)) and (depth_center_front < (CENTER_SLOW_M - 0.15)))
                    )

            if hard_stop_raw:
                obstacle_stop_streak_s += dt
            else:
                obstacle_stop_streak_s = 0.0

            if front_valid and (hard_stop_raw and (obstacle_stop_streak_s >= OBSTACLE_STOP_CONFIRM_S)):
                speed_cmd = 0.0
                stop_reason = "obstacle"
            elif front_valid and ((center_front < CENTER_SLOW_M) or (min_front < 0.55)):
                speed_cmd *= 0.5
            
            # Cap speed
            # speed_cmd = min(speed_cmd, speed_cap_mps)

            # Anti-dart guard: high steering must run at low speed to avoid
            # sudden curb/sidewalk departures.
            abs_steer = abs(float(steer_raw))
            if abs_steer > 0.30:
                speed_cmd = min(speed_cmd, HIGH_STEER_SPEED_CAP)
            elif abs_steer > 0.20:
                speed_cmd = min(speed_cmd, MOD_STEER_SPEED_CAP)

            # Map to actuators
            throttle = speed_to_throttle(speed_cmd)
            steer_out = float(STEER_OUTPUT_SIGN * steer_raw)
            steer_model_prev = float(steer_raw)

            # LED Logic (segment-aware)
            #   TO_PICKUP        -> Green    (0, 1, 0)
            #   TO_DROPOFF       -> Blue     (0, 0, 1)
            #   TO_HUB           -> Orange   (1, 0.5, 0)
            #   HUB_WAIT / DONE  -> Magenta  (1, 0, 1)
            if (t < start_magenta_until) or (step_name in ("HUB_WAIT", "DONE")):
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


            # HUD
            if bgr is not None and bgr.size:
                action = "STOP" if speed_cmd <= 1e-3 else ("SLOW" if speed_cmd < 0.5 * max_speed_mps else "GO")
                lines = [
                    f"segment={step_name} idx={segment_idx+1}/{len(route_segments)} mode={action_type}",
                    f"odom=({pose_x:+.2f},{pose_y:+.2f}) th={pose_th:+.2f} dr_dist={dr.total_dist:.2f}",
                    f"lane_ok={int(lane_ok)} conf={lane_conf:.2f} lane_steer={lane_steer:+.2f} err={lane_err:+.2f}",
                    f"plan_steer={target_steer:+.2f} hold={max(0.0, segment_hold_until-t):.1f}s",
                    f"rbB={rb_bias:+.2f} rbW={rb_w:.2f} rbCue={int(rb_sign_active)} rbScore={rb_last_score:.2f} rbDist={rb_dist_m:.1f}",
                    f"signs: stop={'ON' if stop_sign_active else 'off'}({stop_sign_stopped_t:.1f}/{STOP_SIGN_DWELL_S:.1f}s) yield={max(0.0,yield_slow_until-t):.1f}s tlR={max(0.0,tl_red_until-t):.1f}s tlY={max(0.0,tl_yellow_until-t):.1f}s tlG={max(0.0,tl_green_until-t):.1f}s",
                    f"swC={swC:.2f} swN={swN:.2f} swB={swB:+.2f}",
                    f"minF={min_front:.2f} cF={center_front:.2f}",
                    # f"speed={speed_cmd:.2f} cap={speed_cap_mps:.2f}",
                    f"steer_raw={steer_raw:+.2f} sent={steer_out:+.2f} v={dr_speed:.2f}",
                    f"stop_reason={stop_reason}",
                ]
                hud = render_hud(bgr, f"action: {action}", lines)
                cv2.imshow("sign_debug", hud)
            if lane_dbg is not None:
                cv2.imshow("lane_fit_debug", lane_dbg)
            if sw_dbg is not None:
                cv2.imshow("lane_sw_debug", sw_dbg)

            cv2.waitKey(1)

            # pacing
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
            if gps is not None and hasattr(gps, 'terminate'):
                gps.terminate()
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--waypoints", default="waypoints.txt")
    ap.add_argument("--actor", type=int, default=0)
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=2.2)
    ap.add_argument("--no-print", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--no-realsense", action="store_true")

    # compatibility args (not used in this restart)
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
