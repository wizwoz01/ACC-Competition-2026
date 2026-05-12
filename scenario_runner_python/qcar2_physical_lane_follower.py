"""qcar2_physical_lane_follower.py – Physical QCar2 Autonomous Lane Follower

                Beach Autonomous Systems - CSULB CECS

Localization : LiDAR odometry-based (scan matching & IMU integration).
Lane detection: Canny edge detection (yellow/white lane markings).
Path following: Pure Pursuit on waypoints + lane centering correction.
LiDAR mapping : Real-time occupancy grid for obstacle avoidance.

This is the PHYSICAL hardware version, adapted from the QLabs simulation
to work with the actual QCar2 robot:
- No QLabs TCP (WorldTransform, LED direct, camera TCPIP)
- Physical camera via USB or on-board
- Physical LiDAR + IMU for odometry
- Lane-following primary; waypoints as fallback
"""

import argparse
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import cv2
import numpy as np

from pal.products.qcar import QCar, QCarLidar, QCarRealSense
from pal.utilities.vision import Camera2D
from hal.utilities.control import PurePursuitController

# ===================================================================
#  LiDAR-based Odometry
# ===================================================================

class LidarOdometry:
    """Simple LiDAR odometry: accumulate motion vectors from frame-to-frame scans.
    Uses angle histogram matching and motion estimation for basic dead reckoning."""
    
    def __init__(self, max_range: float = 8.0):
        self.max_range = float(max_range)
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # [x, y, theta]
        self.last_scan_xy = None
        self.scan_lock = threading.Lock()
        self.imu_theta_offset = 0.0  # Calibration offset
        
    def _angle_hist_match(self, scan1: np.ndarray, scan2: np.ndarray,
                         angle_step: float = 0.05) -> float:
        """Estimate rotation between two scans using angle histogram correlation."""
        if scan1 is None or scan2 is None or scan1.shape[0] < 10 or scan2.shape[0] < 10:
            return 0.0
        
        # Convert to angles
        a1 = np.arctan2(scan1[:, 1], scan1[:, 0])
        a2 = np.arctan2(scan2[:, 1], scan2[:, 0])
        
        # Histogram correlation for rotation estimate
        angles = np.linspace(-np.pi, np.pi, 360)
        h1 = np.histogram(a1, bins=angles)[0].astype(np.float32)
        h2 = np.histogram(a2, bins=angles)[0].astype(np.float32)
        
        # Find best rotation offset
        max_corr = -1e9
        best_offset = 0.0
        for offset in np.linspace(-np.pi/12, np.pi/12, 30):
            h2_rot = np.roll(h2, int(offset / angle_step))
            corr = np.sum(h1 * h2_rot)
            if corr > max_corr:
                max_corr = corr
                best_offset = offset
        
        return float(best_offset)
    
    def update(self, lidar_angles: np.ndarray, lidar_distances: np.ndarray,
               imu_yaw: float = None) -> Tuple[np.ndarray, bool]:
        """Update pose from LiDAR scan and optional IMU yaw.
        Returns (pose, ok) where ok=True if update succeeded."""
        
        if lidar_angles is None or lidar_distances is None:
            return self.pose.copy(), False
        
        # Convert polar to cartesian (car frame)
        a_raw = (-lidar_angles + np.pi).astype(np.float32)
        a = np.arctan2(np.sin(a_raw), np.cos(a_raw))
        d = lidar_distances.astype(np.float32)
        valid = np.isfinite(d) & (d > 0.01) & (d <= self.max_range)
        
        if not np.any(valid):
            return self.pose.copy(), False
        
        a = a[valid]
        d = d[valid]
        scan_xy = np.column_stack([d * np.cos(a), d * np.sin(a)]).astype(np.float64)
        
        with self.scan_lock:
            ok = True
            dx, dy = 0.0, 0.0
            dtheta = 0.0
            
            # FM update if we have previous scan
            if self.last_scan_xy is not None and len(scan_xy) > 10:
                # Estimate rotation via angle histogram
                dtheta = self._angle_hist_match(self.last_scan_xy, scan_xy)
                
                # Estimate translation via centroid shift
                # Simple heuristic: use nearest-neighbor and median displacement
                if len(self.last_scan_xy) >= 5 and len(scan_xy) >= 5:
                    # Use center-based translation estimate
                    c1 = np.median(self.last_scan_xy, axis=0)
                    c2 = np.median(scan_xy, axis=0)
                    dc = c2 - c1
                    
                    # Apply rotation to get world-frame displacement
                    theta_now = self.pose[2]
                    dx = dc[0] * np.cos(theta_now) - dc[1] * np.sin(theta_now)
                    dy = dc[0] * np.sin(theta_now) + dc[1] * np.cos(theta_now)
                    
                    # Damping: only use update if displacement is reasonable (< 0.2m per frame at 30Hz)
                    disp = math.hypot(dx, dy)
                    if disp > 0.15:
                        dx *= 0.5
                        dy *= 0.5
                        ok = False  # Failed update alert
            
            # Update pose
            self.pose[0] += dx
            self.pose[1] += dy
            self.pose[2] += dtheta
            
            # If IMU yaw is available, blend it in
            if imu_yaw is not None:
                # Soft blend: trust dead reckoning mostly, IMU for drift correction
                self.pose[2] = 0.95 * self.pose[2] + 0.05 * (imu_yaw + self.imu_theta_offset)
            
            # Normalize heading
            self.pose[2] = float((self.pose[2] + np.pi) % (2.0 * np.pi) - np.pi)
            
            # Store current scan for next frame
            self.last_scan_xy = scan_xy.copy()
        
        return self.pose.copy(), ok


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
#  Lane Controller  (yellow / white boundary fitting)
# ===================================================================

class LaneController:
    """Canny + HoughLinesP lane detection for REAL ROADS.
    
    Adapted for actual road markings (more robust, less tuning).
    Return signature: (steer, ok, conf, err, dbg, xl, xr, y0)
    """

    def __init__(self):
        # --- ROI trapezoid (adjusted for real road perspective) ---
        # Real roads: horizon is higher, focus on road ahead
        self.roi_top_y_ratio = 0.25     # Start higher (more road surface visible)
        self.roi_bottom_y_ratio = 1.00
        self.roi_top_width_ratio = 0.70   # Wider FOV for lane detection
        self.roi_bottom_width_ratio = 1.05

        # --- Hough params (tuned for real road conditions) ---
        self.hough_rho = 2
        self.hough_theta = np.pi / 180
        self.hough_thresh = 45        # Higher threshold for real road noise
        self.hough_min_line_len = 30
        self.hough_max_line_gap = 60

        self.lane_width_px = 380.0    # Real roads: wider lanes than QLabs

        # --- Control gains (reduced for physical stability) ---
        self.k_lat = 0.70     # Reduced for smoother steering
        self.k_head = 0.15    # Conservative heading correction
        self.kd = 0.04        # Modest damping
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
        self.max_bound_age_s = 1.5   # Shorter persistence for real roads
        self.bias_right_px = 0.0
        self.target_offset_frac = 0.25  # Bias toward center
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
        """Canny+HoughLinesP lane detection for REAL ROADS."""
        t_now = time.time()
        if bgr is None or bgr.size == 0:
            return 0.0, False, 0.0, 0.0, None, None, None, 0

        if not np.isfinite(dt) or dt <= 0:
            dt = 1.0 / 30.0

        H, W = bgr.shape[:2]
        mid_x = 0.5 * W
        y0_roi = int(self.roi_top_y_ratio * H)

        # CLAHE for contrast
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(12, 12))
        gray_clahe = clahe.apply(gray)

        # Detect yellow (road markings) and white (lane dividers)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Real road white: brighter, more varied
        white_mask = cv2.inRange(hsv, np.array((0, 0, 180)), np.array((180, 40, 255)))
        # Real road yellow: typical dashed/solid line color
        yellow_mask = cv2.inRange(hsv, np.array((15, 60, 100)), np.array((35, 255, 255)))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Morphology cleanup
        km = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, km, iterations=1)

        # Canny edge detection (adapted for real images)
        blur = cv2.GaussianBlur(gray_clahe, (7, 7), 1.5)
        try:
            med = float(np.median(blur[color_mask > 0])) if np.count_nonzero(color_mask) else float(np.median(blur))
        except Exception:
            med = float(np.median(blur))
        sigma = 0.45
        low = int(max(10, (1.0 - sigma) * med))
        high = int(min(255, (1.0 + sigma) * med))
        if low >= high:
            low = max(10, int(0.5 * high))

        edges = cv2.Canny(blur, low, high)
        edges = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations=1)

        # Mask to color regions (for real roads)
        if int(np.count_nonzero(color_mask)) > 100:
            km2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            color_mask_d = cv2.dilate(color_mask, km2, iterations=2)
            edges = cv2.bitwise_and(edges, edges, mask=color_mask_d)

        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)), iterations=1)

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
                m = dy / dx if dx != 0 else 1e6
                if abs(m) < 0.25 or abs(m) > 7.0:  # Lane slopes: nearly vertical
                    continue
                # Left lane: negative slope, left of center
                if m < 0 and min(x1, x2) < int(0.50 * W):
                    left_segs.append((x1, y1, x2, y2))
                # Right lane: positive slope, right of center
                elif m > 0 and max(x1, x2) > int(0.50 * W):
                    right_segs.append((x1, y1, x2, y2))

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

        if have_xL and have_xR and (xR_use > xL_use + 100):
            w_est = float(xR_use - xL_use)
            if 200.0 < w_est < 1000.0:
                self.lane_width_px = 0.90 * self.lane_width_px + 0.10 * w_est
            offset_px = self.bias_right_px if abs(self.bias_right_px) > 1e-6 else (self.target_offset_frac * w_est)
            x_center = 0.5 * (xL_use + xR_use) + offset_px
            mode = "LR"
            conf = clamp((wL + wR) / 1200.0, 0.0, 1.0)
            angL = math.atan(mL) if mL is not None else 0.0
            angR = math.atan(mR) if mR is not None else 0.0
            ang = 0.5 * (angL + angR)
        elif have_xR:
            bias_term = self.bias_right_px if self.bias_right_px > 0.0 else (self.target_offset_frac * self.lane_width_px)
            x_center = xR_use - 0.5 * self.lane_width_px + bias_term
            mode = "R"
            conf = clamp(wR / 800.0, 0.0, 1.0)
            ang = math.atan(mR) if mR is not None else math.radians(70.0)
        elif have_xL:
            bias_term = self.bias_right_px if self.bias_right_px > 0.0 else (self.target_offset_frac * self.lane_width_px)
            x_center = xL_use + 0.5 * self.lane_width_px + bias_term
            mode = "L"
            conf = clamp(wL / 800.0, 0.0, 1.0)
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
        # Smoothing for stability
        self.steer_smooth = 0.88 * self.steer_smooth + 0.12 * steer
        steer = self.steer_smooth

        lane_ok = bool(conf > 0.40)
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
                y1b = int(0.25 * H)
                y2b = H - 1
                x1b = int(self._x_at_y(m_v, b_v, y1b))
                x2b = int(self._x_at_y(m_v, b_v, y2b))
                if 0 <= x1b < W and 0 <= x2b < W:
                    cv2.line(dbg, (x1b, y1b), (x2b, y2b), color, 3)

            _draw_line(mL, bL, (255, 80, 0))
            _draw_line(mR, bR, (0, 255, 80))
            cv2.circle(dbg, (int(x_center), H - 6), 7, (0, 0, 255), -1)
            cv2.putText(dbg, f"conf={conf:.2f} err={err:+.2f} head={math.degrees(head_err):+.1f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.putText(dbg, f"mode={mode} steer={steer:+.2f}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        return float(steer), bool(lane_ok), float(conf), float(err), dbg, xl_ret, xr_ret, y0_roi


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
#  Main Physical Lane Follower
# ===================================================================

def run_physical_lane_follower(
    sample_rate_hz: float = 30.0,
    max_speed_mps: float = 0.35,
    debug_print: bool = True,
    use_lidar: bool = True,
    use_realsense: bool = False,
    roadmap_file: str = "",
):
    """Run the physical QCar2 with lane-following control + LiDAR mapping.
    
    Args:
        sample_rate_hz: Control loop frequency
        max_speed_mps: Maximum driving speed
        debug_print: Enable console output
        use_lidar: Enable LiDAR-based odometry
        use_realsense: Enable RealSense depth camera
        roadmap_file: Path to waypoints file (for fallback Pure Pursuit)
    """
    
    # --- Initialize hardware ---
    car = QCar(readMode=1, frequency=int(sample_rate_hz))
    car.read_write_std(throttle=0.0, steering=0.0)
    
    # Physical camera (USB or on-board)
    # Adjust cameraId and resolution based on your hardware
    cam = None
    try:
        # Try different camera IDs until one works
        for cam_id in [0, 1, 2, "3@tcpip://192.168.1.10:18964"]:
            try:
                cam = Camera2D(cameraId=str(cam_id),
                              frameWidth=640, frameHeight=480,
                              frameRate=sample_rate_hz)
                if debug_print:
                    print(f"[CAM] Camera {cam_id} initialized")
                break
            except Exception:
                continue
    except Exception as e:
        if debug_print:
            print(f"[WARN] No camera available: {e}")
    
    # LiDAR + optional RealSense
    lidar = QCarLidar(numMeasurements=1000, rangingDistanceMode=2,
                     interpolationMode=0) if use_lidar else None
    rs = QCarRealSense(mode="RGB, Depth") if use_realsense else None
    
    # Initialize odometry
    odom = LidarOdometry(max_range=8.0)
    lidar_mapper = LidarMapper(map_size_m=(12.0, 12.0), px_per_m=50.0)
    
    # Lane controller
    lane = LaneController()
    
    # Optional: waypoint-based Pure Pursuit (fallback)
    pure_pursuit = None
    route_segments = []
    segment_idx = 0
    if roadmap_file and os.path.exists(roadmap_file):
        try:
            paths = load_waypoints_txt(roadmap_file)
            if "path_to_hub" in paths:
                active_wp = interpolate_waypoints(paths["path_to_hub"], spacing=0.5)
                pure_pursuit = PurePursuitController(
                    waypoints=active_wp.T, lookahead=0.35, cyclic=False)
                if debug_print:
                    print(f"[NAV] Loaded {active_wp.shape[0]} waypoints for Pure Pursuit")
        except Exception as e:
            if debug_print:
                print(f"[WARN] Could not load roadmap: {e}")
    
    # Runtime state
    dt_nom = 1.0 / float(sample_rate_hz)
    t_prev = now()
    pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # [x, y, theta]
    
    steer_smooth = 0.0
    speed_cmd = 0.0
    last_bgr = None
    
    # Logging
    import csv
    log_dir = "paper_logs"
    os.makedirs(log_dir, exist_ok=True)
    
    try:
        log_file = open(os.path.join(log_dir, "physical_run.csv"), "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow([
            "time_s", "pose_x", "pose_y", "pose_theta", 
            "lane_steer", "speed_cmd", "throttle",
            "lane_conf", "lane_ok", "odom_ok"
        ])
    except Exception:
        log_file = None
    
    if debug_print:
        print("[INFO] Physical QCar2 lane follower starting...")
        print("[INFO] Press Ctrl+C to stop")
    
    try:
        frame_count = 0
        start_time = now()
        
        while True:
            t = now()
            dt = t - t_prev
            t_prev = t
            if not np.isfinite(dt) or dt <= 0.0:
                dt = dt_nom
            
            # --- Read sensors ---
            
            # Camera
            bgr = None
            if cam is not None:
                try:
                    cam.read()
                    bgr = cam.imageData
                    if bgr is not None and getattr(bgr, "size", 0):
                        last_bgr = bgr
                except Exception as e:
                    if debug_print and frame_count % 100 == 0:
                        print(f"[WARN] Camera read failed: {e}")
            
            if bgr is None:
                bgr = last_bgr
            
            # LiDAR
            lidar_angles = None
            lidar_dist = None
            odom_ok = False
            if lidar is not None:
                try:
                    lidar.read()
                    lidar_angles = getattr(lidar, "angles", None)
                    lidar_dist = getattr(lidar, "distances", None)
                    
                    # Update odometry
                    pose, odom_ok = odom.update(lidar_angles, lidar_dist)
                    
                    # Update LiDAR mapper
                    pts_xy = _polar_to_xy_car(lidar_angles, lidar_dist)
                    lidar_mapper.accumulate(pts_xy, pose)
                except Exception as e:
                    if debug_print and frame_count % 200 == 0:
                        print(f"[WARN] LiDAR error: {e}")
            
            # Car (speed, steering feedback)
            try:
                car.read()
            except Exception as e:
                if debug_print and frame_count % 200 == 0:
                    print(f"[WARN] Car read failed: {e}")
            
            # --- Lane detection ---
            lane_steer = 0.0
            lane_ok = False
            lane_conf = 0.0
            
            if bgr is not None:
                lane_steer, lane_ok, lane_conf, _, _, _, _, _ = lane.step(bgr, dt, debug=False)
            
            # --- Control ---
            target_steer = lane_steer
            
            # Blend with Pure Pursuit if available
            if pure_pursuit is not None:
                try:
                    pp_steer = pure_pursuit.update(pose[:2], pose[2], 0.2)
                    # Lane-following primary (0.85), Pure Pursuit fallback (0.15)
                    if lane_ok:
                        target_steer = 0.85 * lane_steer + 0.15 * pp_steer
                    else:
                        target_steer = pp_steer
                except Exception:
                    pass
            
            target_steer = clamp(target_steer, -0.60, 0.60)
            
            # Speed control (reduce when steering hard)
            speed_cmd = max_speed_mps
            if abs(target_steer) > 0.35:
                speed_cmd = 0.15
            elif abs(target_steer) > 0.20:
                speed_cmd = 0.25
            
            # Convert speed to throttle (simple mapping)
            throttle = speed_cmd / max_speed_mps if max_speed_mps > 0 else 0.0
            throttle = clamp(throttle, -1.0, 1.0)
            
            # --- Output control ---
            try:
                car.read_write_std(throttle=float(throttle), steering=float(target_steer))
            except Exception as e:
                if debug_print and frame_count % 200 == 0:
                    print(f"[WARN] Car write failed: {e}")
            
            # --- Logging ---
            if log_file is not None:
                try:
                    log_writer.writerow([
                        t - start_time,
                        float(pose[0]), float(pose[1]), float(pose[2]),
                        float(lane_steer), float(speed_cmd), float(throttle),
                        float(lane_conf), int(lane_ok), int(odom_ok)
                    ])
                except Exception:
                    pass
            
            # --- Debug output ---
            if debug_print and frame_count % 30 == 0:
                elapsed = t - start_time
                print(f"[{elapsed:6.1f}s] pos=({pose[0]:+.2f},{pose[1]:+.2f}) "
                      f"θ={math.degrees(pose[2]):+.0f}° "
                      f"lane_ok={lane_ok} conf={lane_conf:.2f} "
                      f"steer={target_steer:+.2f} speed={speed_cmd:.2f}m/s")
            
            frame_count += 1
    
    except KeyboardInterrupt:
        if debug_print:
            print("\n[INFO] Shutting down...")
    
    finally:
        # Stop vehicle
        try:
            car.read_write_std(throttle=0.0, steering=0.0)
        except Exception:
            pass
        
        if log_file is not None:
            log_file.close()
        
        if debug_print:
            print("[INFO] Done.")


# ===================================================================
#  Entry Point
# ===================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Physical QCar2 lane follower")
    parser.add_argument("--speed", type=float, default=0.35, help="Max speed (m/s)")
    parser.add_argument("--rate", type=float, default=30.0, help="Control rate (Hz)")
    parser.add_argument("--roadmap", default="", help="Waypoints file for Pure Pursuit fallback")
    parser.add_argument("--no-lidar", action="store_true", help="Disable LiDAR")
    parser.add_argument("--with-realsense", action="store_true", help="Enable RealSense")
    parser.add_argument("--quiet", action="store_true", help="Suppress debug output")
    
    args = parser.parse_args()
    
    # Import os if needed
    import os
    
    run_physical_lane_follower(
        sample_rate_hz=args.rate,
        max_speed_mps=args.speed,
        debug_print=not args.quiet,
        use_lidar=not args.no_lidar,
        use_realsense=args.with_realsense,
        roadmap_file=args.roadmap,
    )
