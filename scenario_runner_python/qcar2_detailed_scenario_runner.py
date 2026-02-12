#!/usr/bin/env python3
"""qcar2_detailed_scenario_runner.py (RESTART: lane-centering + hard sidewalk guard)

Goal: stay on-road in QLabs and complete the detailed scenario without sidewalk driving.

Design rules (simple + strict):
- Steering comes ONLY from lane boundaries (yellow/white) + safety biases.
- Waypoints are used ONLY for step progression (pickup/dropoff/hub), never for steering.
- Sidewalk/curb detection hard-limits steering and speed to prevent climbing the curb.
- Depth + LiDAR are used as safety layers (slow/stop + steer away) when valid.
- Manual speed cap slider is on the existing OpenCV `sign_debug` window.

CLI is kept compatible with `run_qcar_with_signs.ps1`.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import cv2
import numpy as np

from pal.products.qcar import QCar, QCarLidar, QCarRealSense
from pal.utilities.vision import Camera2D


# ---------------------------
# Utils
# ---------------------------

def now() -> float:
    return time.time()


def clamp(x: float, lo: float, hi: float) -> float:
    return float(max(lo, min(hi, x)))


def wrap_pi(a: float) -> float:
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


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
        line = line.replace(";", "").replace(",", " ")
        parts = [p for p in line.split() if p]
        if len(parts) >= 2:
            pts.append((float(parts[0]), float(parts[1])))
    return np.array(pts, dtype=np.float64)


# ---------------------------
# Pose (dead reckoning) for step progress only
# ---------------------------


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class DeadReckoner:
    def __init__(self, pose0: Pose2D):
        self.pose = Pose2D(float(pose0.x), float(pose0.y), float(pose0.yaw))

    def step(self, v_est: float, yaw_rate_rps: float, dt: float) -> Pose2D:
        if not np.isfinite(dt) or dt <= 0.0:
            dt = 1.0 / 60.0
        self.pose.yaw = wrap_pi(self.pose.yaw + float(yaw_rate_rps) * dt)
        self.pose.x += float(v_est) * math.cos(self.pose.yaw) * dt
        self.pose.y += float(v_est) * math.sin(self.pose.yaw) * dt
        return self.pose


class ProgressTracker:
    def __init__(self, waypoints_xy: np.ndarray):
        self.W = waypoints_xy
        self.i = 0
        self.search_win = 40

    def update(self, x: float, y: float) -> Tuple[int, float]:
        if self.W.size == 0:
            return 0, float("inf")
        N = self.W.shape[0]
        lo = max(0, self.i - self.search_win)
        hi = min(N, self.i + self.search_win + 1)
        seg = self.W[lo:hi]
        d2 = (seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2
        nearest = int(lo + int(np.argmin(d2)))
        if nearest > self.i:
            self.i = nearest
        dist_goal = float(np.hypot(self.W[-1, 0] - x, self.W[-1, 1] - y))
        return int(self.i), dist_goal


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
        # Positive shifts target right; negative shifts left.
        # For right-hand driving, keep a small RIGHT bias so we stay in the
        # correct lane (the sidewalk guard still prevents curb climbing).
        self.bias_right_px = 28.0

        # control gains
        self.k_lat = 0.65
        self.k_head = 0.25
        self.kd = 0.06
        self.max_steer = 0.45

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
                cv2.putText(dbg, f"mode={lane_mode}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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

        sw_rel = (L > (L_med + 34.0)) & (s < min(80.0, s_med + 30.0)) & (v > (v_med + 18.0))
        sw_abs = (L > 192.0) & (s < 65.0) & (v > 170.0)
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
                in_lane = near[:, xL:xR]
                sw_center = float(in_lane.mean())
                near0 = int(0.75 * rH)
                sw_near = float(sw_bin[near0:, xL:xR].mean())

        dbg = cv2.cvtColor(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
        dbg[sw > 0] = (0, 0, 255)
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
    y1 = int(0.50 * H)
    y2 = int(0.98 * H)

    rois = [
        (int(0.15 * W), int(0.42 * W)),
        (int(0.42 * W), int(0.58 * W)),
        (int(0.58 * W), int(0.85 * W)),
    ]

    mins = []
    total_valid = 0
    for x1, x2 in rois:
        win = d[y1:y2, x1:x2]
        # Reject invalid fills (near-zero) and noise.
        win = win[np.isfinite(win) & (win > 0.25)]
        total_valid += int(win.size)
        mins.append(float(np.percentile(win, 5)) if win.size else float("inf"))

    left, center, right = mins
    min_front = float(min(left, center, right))
    # Require a minimum amount of valid depth samples; otherwise treat as invalid.
    valid = bool(np.isfinite(min_front) and (min_front > 0.25) and (min_front < 30.0) and (total_valid > 1500))

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


def speed_to_throttle(speed_mps: float) -> float:
    speed_mps = max(0.0, float(speed_mps))
    if speed_mps <= 1e-3:
        return 0.0
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

    P_pick = paths["path_to_pickup"]
    P_drop = paths["path_to_dropoff"]
    P_hub = paths["path_to_hub"]

    # initial pose guess
    x0, y0 = float(P_pick[0, 0]), float(P_pick[0, 1])
    dx0 = float(P_pick[1, 0] - P_pick[0, 0])
    dy0 = float(P_pick[1, 1] - P_pick[0, 1])
    yaw0 = float(math.atan2(dy0, dx0))

    dr = DeadReckoner(Pose2D(x0, y0, yaw0))

    lane = LaneController()
    swg = SidewalkGuard()

    car = QCar(readMode=1, frequency=int(sample_rate_hz))
    cam = Camera2D(cameraId="3@tcpip://localhost:18964", frameWidth=820, frameHeight=410, frameRate=sample_rate_hz)

    rs = QCarRealSense(mode="RGB, Depth") if use_realsense else None
    lidar = QCarLidar(numMeasurements=1000, rangingDistanceMode=2, interpolationMode=0) if use_lidar else None
    lidar_warn_t = 0.0

    # Optional sign detector (used only for roundabout entry cue).
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
    try:
        if sign_model and sign_labels:
            sign_names = [ln.strip() for ln in open(sign_labels, "r", encoding="utf-8").read().splitlines() if ln.strip()]
            from ultralytics import YOLO  # type: ignore

            sign_model_obj = YOLO(sign_model)
    except Exception:
        sign_model_obj = None
        sign_names = []

    steps = [
        ("HUB_WAIT", P_pick, 0.0, 1.0),
        ("TO_PICKUP", P_pick, max_speed_mps, 0.0),
        ("PICKUP", P_pick, 0.0, 2.0),
        ("TO_DROPOFF", P_drop, max_speed_mps, 0.0),
        ("DROPOFF", P_drop, 0.0, 2.0),
        ("TO_HUB", P_hub, max_speed_mps, 0.0),
        ("DONE", P_hub, 0.0, 999.0),
    ]

    step_idx = 0
    step_name, active_path, target_speed, dwell_s = steps[step_idx]
    prog = ProgressTracker(active_path)
    dwell_until: float | None = None

    # Speed cap slider
    speed_ui_ready = False
    speed_cap_mps = float(max_speed_mps)
    speed_cap_max_mps = float(max(0.5, max_speed_mps))
    speed_trackbar_max = int(round(speed_cap_max_mps * 100.0))
    speed_trackbar_name = "speed_cap_x100"

    def _noop(_v: int):
        return

    # Guardrail thresholds (strict)
    STOP_FRONT_M = 0.75
    SLOW_FRONT_M = 1.60

    # If lane is lost, crawl forward slowly to re-acquire (still respects
    # obstacle + sidewalk stops). Without this, the car can deadlock at start.
    LANE_ACQUIRE_SPEED = 0.35

    SW_SLOW = 0.06
    # Earlier curb/sidewalk intervention to prevent climbing the sidewalk.
    SW_STOP = 0.20
    SW_NEAR_STOP = 0.18

    # If sidewalk detector trips but center path is clear, creep/steer away
    # instead of dead-stopping (prevents getting stuck hugging a curb/median).
    SW_RECOVER_BIAS = 0.18
    SW_RECOVER_CLEAR_M = 1.20
    SW_RECOVER_SPEED = 0.22
    SW_RECOVER_LANE_ERR = 0.55
    SW_RECOVER_LANE_STEER = 0.18
    SW_ESCAPE_LANE_ERR = 0.16

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

    # Waypoint heading bias (for route choice at junctions).
    # IMPORTANT: Only enable this when the waypoint path itself *turns*.
    # If the waypoint path is locally straight (e.g., through a roundabout area
    # where waypoints are only for progress), do NOT let it fight lane-following.
    W_WP_BASE = 0.00
    W_WP_TURN = 0.28
    WP_LOOKAHEAD = 22
    WP_TURN_ENABLE_RAD = math.radians(12.0)

    # smoothing at stop
    steer_filt = 0.0

    dt_nom = 1.0 / float(sample_rate_hz)
    t_prev = now()

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
        print(f"[INFO] RESTART runner step={step_name} pose0≈({x0:.2f},{y0:.2f},{yaw0:.2f})")

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

            # pose for step progress
            v_est = float(getattr(car, "motorTach", 0.0))
            yaw_rate = float(car.gyroscope[2]) if hasattr(car, "gyroscope") else 0.0
            pose = dr.step(v_est=v_est, yaw_rate_rps=yaw_rate, dt=dt)

            prog.W = active_path
            idx_wp, dist_goal = prog.update(pose.x, pose.y)

            # Waypoint-based heading (used as a small steering bias to choose
            # the correct road branch at junctions; lane still does centering).
            #
            # Key guard: only enable waypoint bias when the waypoint *path* is
            # locally turning. This prevents straight waypoint segments from
            # pulling steering the wrong way on curved geometry (roundabouts).
            wp_head_err = 0.0
            wp_bias = 0.0
            wp_w_raw = float(W_WP_BASE)
            try:
                if active_path is not None and active_path.size and active_path.shape[0] >= 3:
                    i0 = int(np.clip(int(idx_wp), 0, active_path.shape[0] - 2))
                    i1 = int(min(i0 + 2, active_path.shape[0] - 1))
                    j = int(min(i0 + WP_LOOKAHEAD, active_path.shape[0] - 1))

                    dx0 = float(active_path[i1, 0] - active_path[i0, 0])
                    dy0 = float(active_path[i1, 1] - active_path[i0, 1])
                    dx1 = float(active_path[j, 0] - active_path[i0, 0])
                    dy1 = float(active_path[j, 1] - active_path[i0, 1])

                    if (dx0 * dx0 + dy0 * dy0) > 1e-8 and (dx1 * dx1 + dy1 * dy1) > 1e-8:
                        local_yaw = float(math.atan2(dy0, dx0))
                        desired_yaw = float(math.atan2(dy1, dx1))
                        path_turn = float(abs(wrap_pi(desired_yaw - local_yaw)))

                        # Only enable waypoint bias when the waypoint path has
                        # an actual turn coming up.
                        if path_turn > WP_TURN_ENABLE_RAD:
                            wp_w_raw = float(W_WP_TURN)
                            wp_head_err = float(wrap_pi(desired_yaw - float(pose.yaw)))
                            # Convert heading error to a gentle steer bias.
                            wp_bias = float(clamp(0.34 * wp_head_err, -0.20, 0.20))
            except Exception:
                wp_head_err = 0.0
                wp_bias = 0.0
                wp_w_raw = float(W_WP_BASE)

            # speed slider
            if not speed_ui_ready:
                try:
                    cv2.namedWindow("sign_debug", cv2.WINDOW_NORMAL)
                    cv2.createTrackbar(
                        speed_trackbar_name,
                        "sign_debug",
                        int(round(speed_cap_mps * 100.0)),
                        max(1, speed_trackbar_max),
                        _noop,
                    )
                    speed_ui_ready = True
                except Exception:
                    speed_ui_ready = False

            if speed_ui_ready:
                try:
                    pos = int(cv2.getTrackbarPos(speed_trackbar_name, "sign_debug"))
                    speed_cap_mps = clamp(pos / 100.0, 0.0, speed_cap_max_mps)
                except Exception:
                    pass

            # lane
            lane_steer, lane_ok, lane_conf, lane_err, lane_dbg, xl, xr, lane_y0 = lane.step(bgr, dt, debug=True)

            # Sign cue (roundabout): latch for a short time.
            rb_sign_active = False
            if sign_model_obj is not None and bgr is not None and getattr(bgr, "size", 0):
                try:
                    sign_frame += 1
                    if (sign_frame % int(max(1, sign_stride))) == 0:
                        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                        results = sign_model_obj.predict(source=rgb, verbose=False, conf=float(sign_conf), device=str(sign_device))
                        r0 = results[0]
                        if getattr(r0, "boxes", None) is not None and len(r0.boxes):
                            clss = r0.boxes.cls.cpu().numpy().astype(int)
                            confs = r0.boxes.conf.cpu().numpy().astype(float)
                            for cid, score in zip(clss, confs):
                                name = sign_names[cid] if (0 <= int(cid) < len(sign_names)) else str(cid)
                                if name == "roundabout" and float(score) >= float(sign_conf):
                                    rb_last_score = float(score)
                                    # If this is a new cue (was previously inactive), reset phase.
                                    if t >= rb_cue_until:
                                        rb_dist_m = 0.0
                                        rb_exit_until = 0.0
                                    rb_cue_until = max(rb_cue_until, t + RB_CUE_HOLD_S)
                                    rb_entry_until = max(rb_entry_until, t + RB_ENTRY_HOLD_S)
                                    break
                except Exception:
                    pass

            if t < rb_cue_until:
                rb_sign_active = True

            # Accumulate distance traveled while in roundabout mode (used to
            # force the first exit instead of looping).
            if rb_sign_active:
                rb_dist_m += max(0.0, float(v_est)) * float(dt)

                # Trigger a short, strong "take the first exit" right-steer.
                if (rb_exit_until <= t) and (t >= rb_entry_until) and (rb_dist_m >= RB_EXIT_DIST_M):
                    rb_exit_until = t + RB_EXIT_HOLD_S
                    # After we commit to exiting, shorten the remaining cue to
                    # a brief post-exit keep-right hold (prevents taking the 2nd exit).
                    rb_cue_until = min(rb_cue_until, t + RB_POST_EXIT_HOLD_S)

            # sidewalk (lane-relative)
            swC, swB, swN, sw_dbg = swg.step(bgr, lane_y0, xl, xr)

            # Defaults (may be overridden later once center_front is known).
            swC_use = float(swC)
            swN_use = float(swN)
            roundabout_fp = False

            # depth/lidar
            depth_min, depth_center, depth_bias, depth_ok = (float("inf"), float("inf"), 0.0, False)
            if depth_px is not None and isinstance(depth_px, np.ndarray) and depth_px.size:
                depth_min, depth_center, depth_bias, depth_ok = depth_front(depth_px)

            lidar_min, lidar_center, lidar_bias, corrC, lidar_ok = (float("inf"), float("inf"), 0.0, 0.0, False)
            if lidar_angles is not None and lidar_dist is not None:
                lidar_min, lidar_center, lidar_bias, corrC, lidar_ok = lidar_front(lidar_angles, lidar_dist)

            # Consistency: if depth reports something extremely close but LiDAR
            # is clearly open, treat depth as invalid for this frame.
            if depth_ok and lidar_ok and (depth_center < 0.45) and (lidar_center > 1.00):
                depth_ok = False

            fronts = []
            if depth_ok:
                fronts.append(depth_min)
            if lidar_ok:
                fronts.append(lidar_min)
            min_front = float(min(fronts)) if fronts else float("inf")

            centers = []
            if depth_ok:
                centers.append(depth_center)
            if lidar_ok:
                centers.append(lidar_center)
            center_front = float(min(centers)) if centers else float("inf")

            # Roundabout handling (requires center_front):
            # The hub/roundabout island is bright concrete and can saturate the
            # sidewalk-in-lane detector (swC/swN -> 1.0), causing a dead-stop.
            # If lane fit is very confident and forward-center is clear-ish,
            # treat this as a false positive and keep-right while creeping.
            if lane_ok and (lane_conf > 0.88) and (swC > 0.85) and (swN > 0.85):
                center_clearish = (not np.isfinite(center_front)) or (center_front > 0.80)
                # On a real roundabout, lane steering will be non-trivial.
                # Keep this loose so we don't freeze due to curvature.
                lane_stable = (abs(lane_err) < 0.35) and (abs(lane_steer) < 0.28)
                if center_clearish and lane_stable:
                    roundabout_fp = True
                    swC_use = min(swC_use, 0.08)
                    swN_use = min(swN_use, 0.08)

            # Apply sidewalk-based gating for waypoint steering AFTER swC_use/swN_use finalized.
            wp_w = float(wp_w_raw)
            # Safety gating: if sidewalk detector is hot (or bias indicates the
            # waypoint turn would steer TOWARD the sidewalk side), disable wp.
            # Note: swB sign is such that +swB => sidewalk more on right.
            if (swC_use > SW_SLOW) or (swN_use > SW_SLOW):
                wp_w = 0.0
            elif (abs(swB) > 0.15) and (swB * wp_bias > 0.0):
                wp_w = 0.0

            # If roundabout sign cue is active, never allow waypoint bias.
            if rb_sign_active:
                wp_w = 0.0

            # Roundabout bias:
            #  1) Primary: sign-cued keep-right bias (guarantees correct entry)
            #  2) Secondary: pose-based CCW tangent helper when dead-reckoning is usable
            rb_w = 0.0
            rb_bias = 0.0
            try:
                if rb_sign_active:
                    rb_w = 1.0
                    rb_bias = float(RB_KEEP_RIGHT_STEER)
                    if t < rb_entry_until:
                        rb_bias = float(RB_ENTRY_STEER)
                    elif t < rb_exit_until:
                        rb_bias = float(RB_EXIT_STEER)
                else:
                    dxr = float(pose.x - RB_CX)
                    dyr = float(pose.y - RB_CY)
                    rr = float(math.hypot(dxr, dyr))
                    if (rr > RB_R_IN) and (rr < RB_R_OUT):
                        # CCW tangent to radius vector r = (dxr, dyr)
                        tx = float(-dyr)
                        ty = float(dxr)
                        if (tx * tx + ty * ty) > 1e-8:
                            desired_rb_yaw = float(math.atan2(ty, tx))
                            rb_head_err = float(wrap_pi(desired_rb_yaw - float(pose.yaw)))
                            rb_bias = float(clamp(0.30 * rb_head_err, -0.20, 0.20))
                            rb_w = float(clamp(0.25 + 0.75 * (1.0 - lane_conf), 0.25, 1.0))
            except Exception:
                rb_w = 0.0
                rb_bias = 0.0

            # --- Unified steering: lane + hard guardrails
            steer_raw = float(lane_steer)
            stop_reason = ""

            # Lane is the only *steering* source, but if it's temporarily lost we
            # allow a slow crawl to re-acquire while using safety biases.
            if not lane_ok:
                # base crawl
                speed_cmd = float(min(target_speed, speed_cap_mps, LANE_ACQUIRE_SPEED))
                # steer only from safety biases (keep small)
                steer_raw = float(
                    clamp(
                        W_OBS * (depth_bias + lidar_bias)
                        + W_SW * swB
                        + (wp_w * wp_bias)
                        + (W_RB * rb_w * rb_bias),
                        -0.25,
                        0.25,
                    )
                )
                stop_reason = "lane_lost"
            else:
                # add obstacle bias
                steer_raw = float(
                    clamp(
                        steer_raw
                        + W_OBS * (depth_bias + lidar_bias)
                        + W_SW * swB
                        + (wp_w * wp_bias)
                        + (W_RB * rb_w * rb_bias),
                        -0.45,
                        0.45,
                    )
                )

                # HARD: do not steer toward the sidewalk when near-field sidewalk is high
                if swN_use > 0.18:
                    if swB > 0.05:
                        steer_raw = max(steer_raw, 0.18)
                    elif swB < -0.05:
                        steer_raw = min(steer_raw, -0.18)

                # HARD: clamp maximum steering if center sidewalk is present
                if swC_use > 0.18:
                    steer_raw = clamp(steer_raw, -0.35, 0.35)

                # In sign-cued roundabout mode, bias to keep-right and do not
                # allow large left turns that can enter/circulate the wrong way.
                if rb_sign_active:
                    steer_raw = float(min(steer_raw, 0.10))

                # --- Speed
                speed_cmd = float(target_speed)
                speed_cmd = min(speed_cmd, float(speed_cap_mps))

                # slow for steering
                speed_cmd *= clamp(1.0 - 0.65 * abs(steer_raw) / 0.45, 0.35, 1.0)

                # slow/stop for sidewalk intrusion
                if swC_use > SW_SLOW or swN_use > SW_SLOW:
                    speed_cmd *= clamp(1.0 - 1.4 * max(swC_use, swN_use), 0.10, 0.60)
                if swC_use > 0.12 or swN_use > 0.12:
                    speed_cmd = min(speed_cmd, 0.18)

                # Proactive steer-away from sidewalk side (before hard stop).
                # Sign convention: +steer is left, -steer is right.
                if swB > 0.08:
                    steer_raw = max(steer_raw, float(clamp(0.16 + 0.85 * swB, 0.16, 0.42)))
                elif swB < -0.08:
                    steer_raw = min(steer_raw, float(clamp(-0.16 + 0.85 * swB, -0.42, -0.16)))

                if (swC_use > SW_STOP) or (swN_use > SW_NEAR_STOP):
                    # Recovery: if forward-center is clear and either
                    # (a) sidewalk is mostly on one side, or
                    # (b) lane controller is demanding a strong correction,
                    # creep instead of freezing.
                    lane_wants_turn = bool(lane_ok and (abs(lane_err) > SW_RECOVER_LANE_ERR or abs(lane_steer) > SW_RECOVER_LANE_STEER))
                    center_clear = bool((not np.isfinite(center_front)) or (center_front > SW_RECOVER_CLEAR_M))
                    lane_escape_turn = bool(lane_ok and lane_conf > 0.65 and abs(lane_err) > SW_ESCAPE_LANE_ERR)
                    escape_ok = bool((((not np.isfinite(center_front)) or (center_front > 0.90))) and (abs(swB) > 0.15 or lane_escape_turn))
                    # Sidewalk detector can saturate on bright concrete/walls.
                    # If lane fit is very confident, bias is near-balanced, and
                    # forward center is clear, treat as false positive.
                    sidewalk_false_positive = bool(
                        lane_ok
                        and (lane_conf > 0.85)
                        and (abs(lane_err) < 0.60)
                        and (abs(swB) < 0.35)
                        and (swC > 0.75)
                        and (swN > 0.75)
                        and (not np.isfinite(center_front) or center_front > 0.95)
                    )
                    # If sidewalk is intruding in-lane (swC high), do NOT creep.
                    if sidewalk_false_positive:
                        speed_cmd = max(speed_cmd, 0.16)
                        stop_reason = stop_reason or "sidewalk_fp"
                    elif roundabout_fp:
                        # Roundabout concrete false-positive: keep moving and keep right.
                        speed_cmd = max(speed_cmd, 0.16)
                        steer_raw = float(clamp(steer_raw - 0.12, -0.45, 0.45))
                        stop_reason = stop_reason or "roundabout_fp"
                    elif (swC <= SW_STOP + 0.05) and center_clear and ((abs(swB) >= SW_RECOVER_BIAS) or lane_wants_turn):
                        speed_cmd = float(min(speed_cmd, SW_RECOVER_SPEED))
                        # steer away using whichever signal is available
                        steer_raw = float(clamp(steer_raw + (0.35 * swB) + (0.25 * lane_steer), -0.45, 0.45))
                        stop_reason = "sidewalk_recover"
                    elif escape_ok:
                        # Hard escape from curb: force steer away and crawl.
                        speed_cmd = 0.12
                        if abs(swB) > 0.12:
                            steer_raw = 0.45 if swB > 0.0 else -0.45
                        else:
                            steer_raw = 0.45 if lane_err > 0.0 else -0.45
                        stop_reason = "sidewalk_escape"
                    else:
                        speed_cmd = 0.0
                        stop_reason = "sidewalk"

                # slow/stop for obstacles
                if np.isfinite(min_front) and min_front < SLOW_FRONT_M:
                    obs_scale = float(clamp((min_front - 0.45) / (SLOW_FRONT_M - 0.45), 0.0, 1.0))
                    # If the *center* is clear (no hard-stop), don't let a very
                    # close off-center return collapse speed to exactly zero.
                    if (obs_scale <= 1e-3) and (not np.isfinite(center_front) or center_front >= STOP_FRONT_M):
                        obs_scale = 0.25
                        stop_reason = stop_reason or "side_obstacle"
                    speed_cmd *= obs_scale
                if np.isfinite(center_front) and center_front < STOP_FRONT_M:
                    speed_cmd = 0.0
                    stop_reason = "obstacle"

            # Always enforce the same hard-stops even during lane acquire
            if (swC_use > SW_STOP) or (swN_use > SW_NEAR_STOP):
                lane_wants_turn = bool(lane_ok and (abs(lane_err) > SW_RECOVER_LANE_ERR or abs(lane_steer) > SW_RECOVER_LANE_STEER))
                center_clear = bool((not np.isfinite(center_front)) or (center_front > SW_RECOVER_CLEAR_M))
                lane_escape_turn = bool(lane_ok and lane_conf > 0.65 and abs(lane_err) > SW_ESCAPE_LANE_ERR)
                escape_ok = bool((((not np.isfinite(center_front)) or (center_front > 0.90))) and (abs(swB) > 0.15 or lane_escape_turn))
                sidewalk_false_positive = bool(
                    lane_ok
                    and (lane_conf > 0.85)
                    and (abs(lane_err) < 0.60)
                    and (abs(swB) < 0.35)
                    and (swC > 0.75)
                    and (swN > 0.75)
                    and (not np.isfinite(center_front) or center_front > 0.95)
                )
                if sidewalk_false_positive:
                    speed_cmd = max(speed_cmd, 0.16)
                    stop_reason = stop_reason or "sidewalk_fp"
                elif roundabout_fp:
                    speed_cmd = max(speed_cmd, 0.16)
                    steer_raw = float(clamp(steer_raw - 0.12, -0.45, 0.45))
                    stop_reason = stop_reason or "roundabout_fp"
                elif (swC <= SW_STOP + 0.05) and center_clear and ((abs(swB) >= SW_RECOVER_BIAS) or lane_wants_turn):
                    speed_cmd = float(min(speed_cmd, SW_RECOVER_SPEED))
                    steer_raw = float(clamp(steer_raw + (0.35 * swB) + (0.25 * lane_steer), -0.45, 0.45))
                    stop_reason = stop_reason or "sidewalk_recover"
                elif escape_ok:
                    speed_cmd = max(speed_cmd, 0.12)
                    if abs(swB) > 0.12:
                        steer_raw = 0.45 if swB > 0.0 else -0.45
                    else:
                        steer_raw = 0.45 if lane_err > 0.0 else -0.45
                    stop_reason = stop_reason or "sidewalk_escape"
                else:
                    speed_cmd = 0.0
                    stop_reason = stop_reason or "sidewalk"
            if np.isfinite(center_front) and center_front < STOP_FRONT_M:
                speed_cmd = 0.0
                stop_reason = stop_reason or "obstacle"

            # Final hard guard: never steer toward the sidewalk side.
            # swB > 0 => sidewalk is more on right -> force leftward steering.
            if swB > 0.04:
                steer_raw = max(steer_raw, float(clamp(0.10 + 0.75 * swB, 0.10, 0.45)))
            elif swB < -0.04:
                steer_raw = min(steer_raw, float(clamp(-0.10 + 0.75 * swB, -0.45, -0.10)))

            # map to actuators
            throttle = speed_to_throttle(speed_cmd)

            if throttle <= 1e-3:
                steer_filt = 0.85 * steer_filt + 0.15 * steer_raw
                if abs(steer_filt) < 0.02:
                    steer_filt = 0.0
                steer_out = float(STEER_OUTPUT_SIGN * steer_filt)
            else:
                steer_out = float(STEER_OUTPUT_SIGN * steer_raw)

            car.read_write_std(throttle=throttle, steering=steer_out, LEDs=np.array([0, 0, 0, 0, 0, 0, 1, 1]))

            # step transitions
            at_goal = (dist_goal < 3.4) or (idx_wp >= (active_path.shape[0] - 3))
            if step_name in ("HUB_WAIT", "PICKUP", "DROPOFF", "DONE"):
                if dwell_until is None:
                    dwell_until = t + float(dwell_s)
                if t >= dwell_until:
                    dwell_until = None
                    step_idx = min(step_idx + 1, len(steps) - 1)
                    step_name, active_path, target_speed, dwell_s = steps[step_idx]
                    prog = ProgressTracker(active_path)
                    if debug_print:
                        print(f"[STEP] -> {step_name}")
            else:
                if at_goal:
                    step_idx = min(step_idx + 1, len(steps) - 1)
                    step_name, active_path, target_speed, dwell_s = steps[step_idx]
                    prog = ProgressTracker(active_path)
                    if debug_print:
                        print(f"[STEP] -> {step_name}")

            # HUD
            if bgr is not None and bgr.size:
                action = "STOP" if speed_cmd <= 1e-3 else ("SLOW" if speed_cmd < 0.5 * max_speed_mps else "GO")
                lines = [
                    f"step={step_name} idx={idx_wp} dist={dist_goal:.1f}",
                    f"lane_ok={int(lane_ok)} conf={lane_conf:.2f} lane_steer={lane_steer:+.2f} err={lane_err:+.2f}",
                    f"wpHead={wp_head_err:+.2f} wpB={wp_bias:+.2f} wpW={wp_w:.2f}",
                    f"rbB={rb_bias:+.2f} rbW={rb_w:.2f} rbCue={int(rb_sign_active)} rbScore={rb_last_score:.2f} rbDist={rb_dist_m:.1f}",
                    (f"swC={swC:.2f}->{swC_use:.2f} swN={swN:.2f}->{swN_use:.2f} swB={swB:+.2f}" if roundabout_fp else f"swC={swC:.2f} swN={swN:.2f} swB={swB:+.2f}"),
                    f"depthMin={depth_min:.2f} depthC={depth_center:.2f} dOK={int(depth_ok)} lidarMin={lidar_min:.2f} lidarC={lidar_center:.2f} lOK={int(lidar_ok)} minF={min_front:.2f} cF={center_front:.2f}",
                    f"speed={speed_cmd:.2f} cap={speed_cap_mps:.2f}",
                    f"steer_raw={steer_raw:+.2f} sent={steer_out:+.2f} v={v_est:.2f}",
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
