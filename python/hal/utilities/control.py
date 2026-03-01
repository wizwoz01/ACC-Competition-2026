"""This module contains general implementations for various controller types.
"""
import numpy as np
from pal.utilities.math import wrap_to_pi

class PID:
    """A proportional-integral-derivative (PID) controller.

    A proportional-integral-derivative (PID) controller that produces a control
    signal based on the tracking error and specified gain terms: Kp, Ki, and
    Kd.
    """

    def __init__(self, Kp=0, Ki=0, Kd=0, uLimits=None):
        """Creates a PID Controller Instance

        Args:
            Kp (float): Proportional gain (default 0).
            Ki (float): Integral gain (default 0).
            Kd (float): Derivative gain (default 0).
            uLimits (tuple): Upper and lower limits on the output signal,
                (u_min, u_max). Defaults to None.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.uLimits = uLimits
        self.prev_e = None
        self.reset()

    def reset(self):
        """Reset the controller.

        This method resets numerical integrator and derivative.
        """
        self.ei = 0
        self.prev_e = None

    def update(self, r, y, dt):
        """Update the controller output.

        This method updates the control output based on the current set-point,
        measured value, and time since last update.

        Args:
            r (float): The set-point.
            y (float): The measured value.
            dt (float): The time elapsed since the last update.

        Returns:
            float: The controller output.
        """
        # Validate dt to prevent numerical issues
        if dt <= 0:
            # Invalid dt - return previous output or zero
            return 0.0 if self.prev_e is None else self.Kp * (r - y)
        
        # Clamp dt to prevent instability from very large time steps
        dt = min(dt, 1.0)  # Cap at 1 second
        
        # Calculate the error (e)
        e = r - y

        # Calculate derivative of the error
        if self.prev_e is None or dt < 0.001:
            de = 0
        else:
            de = (e - self.prev_e) / dt
        self.prev_e = e

        # Calculate proportional and derivative terms
        u_p = self.Kp * e
        u_d = self.Kd * de
        
        # Calculate what the output would be without integral term
        u_without_i = u_p + u_d
        
        # Check if we would saturate without integral term
        would_saturate = False
        if self.uLimits is not None:
            if u_without_i >= self.uLimits[1] or u_without_i <= self.uLimits[0]:
                would_saturate = True
        
        # Only accumulate integral if we won't saturate (anti-windup)
        if not would_saturate:
            self.ei += dt * e
        
        # Calculate the full control signal
        u = u_p + self.Ki * self.ei + u_d
        
        # Apply limits if specified
        if self.uLimits is None:
            return u
        
        return np.clip(u, self.uLimits[0], self.uLimits[1])


class StanleyController:
    """A Stanley controller for following a path of waypoints.

    The controller determines the desired heading based on the closest point on
    the path, and computes the steering angle required to track that heading.

    Args:
        waypoints (numpy.ndarray): An array of waypoints as (x, y) tuples.
        k (float): The gain for the cross track error (default 1).
        cyclic (bool): Whether the path is cyclic (default True).
    """
    def __init__(self, waypoints, k=1, cyclic=True):
        self.updatePath(waypoints, cyclic)
        self.maxSteeringAngle = np.pi/6
        self.k = k
        self.p_ref = (0, 0)
        self.th_ref = 0

    def updatePath(self, waypoints, cyclic):
        """Update the path of waypoints.

        This method updates the array of waypoints and resets the internal
        state of the controller.

        Args:
            waypoints (numpy.ndarray): An array of waypoints as (x, y) tuples.
                Accepts both Nx2 (row-per-point) and 2xN (column-per-point) formats.
            cyclic (bool): Whether the path is cyclic.
        
        Raises:
            ValueError: If waypoints array is empty or has fewer than 2 points.
        """
        self.wp = np.array(waypoints)
        
        # Validate input
        if self.wp.size == 0:
            raise ValueError("Waypoints array cannot be empty")
        
        # Detect Nx2 format (rows > 2, columns == 2) and transpose to 2xN
        if self.wp.ndim == 2 and self.wp.shape[1] == 2 and self.wp.shape[0] > 2:
            self.wp = self.wp.T
        
        # N = number of waypoints (columns in 2xN format)
        self.N = self.wp.shape[1]
        
        # Require at least 2 waypoints to form a segment
        if self.N < 2:
            raise ValueError(f"Waypoints array must have at least 2 points, got {self.N}")

        self.wpi = 0
        self.cyclic = cyclic
        self.pathComplete = False

    def set_waypoint_index(self, i):
        """Set current waypoint index (e.g. to resync when wrong-way).
        
        Args:
            i (int): Desired waypoint index. Will be clamped to valid range.
        """
        if self.N < 2:
            self.wpi = 0
        else:
            self.wpi = min(max(0, int(i)), max(0, self.N - 2))
        self.pathComplete = False

    def update(self, p, th, speed):
        """Update the controller output.

        This method updates the controller output based on the current position
        and heading.

        Args:
            p (tuple or numpy.ndarray): The current position as an (x, y) tuple or array.
            th (float): The current heading in radians.
            speed (float): The current speed (must be >= 0).

        Returns:
            float: The steering angle in radians.
        """
        # Convert p to numpy array if needed
        p = np.array(p, dtype=np.float64)
        
        # Validate inputs
        if self.N < 2:
            return 0.0
        
        # Ensure speed is non-negative to avoid division issues
        speed = max(0.0, float(speed))
        
        # Handle waypoint indexing safely
        if self.N == 1:
            return 0.0
        
        # Use modulo to handle cyclic paths, but ensure we don't divide by zero
        seg_idx = self.wpi % max(1, self.N - 1)
        next_idx = (self.wpi + 1) % max(1, self.N - 1)
        
        wp_1 = self.wp[:, seg_idx]
        wp_2 = self.wp[:, next_idx]

        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        
        # Handle zero-length segments (duplicate waypoints)
        if v_mag < 1e-6:
            # Advance to next segment if possible
            if self.cyclic or self.wpi < self.N - 2:
                self.wpi = (self.wpi + 1) % max(1, self.N - 1)
            else:
                self.pathComplete = True
            return 0.0
        
        v_uv = v / v_mag
        tangent = np.arctan2(v_uv[1], v_uv[0])

        s = np.dot(p - wp_1, v_uv)
        # Clamp s to segment so we never steer toward a point behind us (wrong-way)
        s = np.clip(s, 0.0, v_mag)

        if s >= v_mag:
            if self.cyclic or self.wpi < self.N - 2:
                self.wpi += 1
                # Ensure wpi doesn't exceed bounds
                if self.cyclic:
                    self.wpi = self.wpi % max(1, self.N - 1)
                else:
                    self.wpi = min(self.wpi, self.N - 2)
            else:
                self.pathComplete = True

        ep = wp_1 + v_uv * s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent - th)

        self.p_ref = tuple(ep)
        self.th_ref = tangent

        # Avoid division by zero when speed is very small
        # Use a minimum speed threshold for the arctan2 calculation
        speed_threshold = max(speed, 0.1)  # Minimum 0.1 m/s to avoid numerical issues
        
        steering = wrap_to_pi(psi + np.arctan2(self.k * ect, speed_threshold))
        
        return np.clip(
            steering,
            -self.maxSteeringAngle,
            self.maxSteeringAngle
        )


class PurePursuitController:
    """Pure pursuit path follower for waypoints.

    Steers toward a lookahead point on the path. Same interface as StanleyController
    for drop-in replacement: updatePath, set_waypoint_index, update(p, th, speed),
    pathComplete, p_ref, th_ref, maxSteeringAngle.
    """
    def __init__(self, waypoints, lookahead=0.6, cyclic=False):
        self.updatePath(waypoints, cyclic)
        self.maxSteeringAngle = np.pi / 6
        self.lookahead = float(lookahead)
        self.p_ref = (0.0, 0.0)
        self.th_ref = 0.0

    def updatePath(self, waypoints, cyclic):
        self.wp = np.array(waypoints, dtype=np.float64)
        if self.wp.size == 0:
            raise ValueError("Waypoints array cannot be empty")
        if self.wp.ndim == 2 and self.wp.shape[1] == 2 and self.wp.shape[0] > 2:
            self.wp = self.wp.T
        self.N = self.wp.shape[1]
        if self.N < 2:
            raise ValueError(f"Waypoints array must have at least 2 points, got {self.N}")
        self.wpi = 0
        self.cyclic = cyclic
        self.pathComplete = False

    def set_waypoint_index(self, i):
        if self.N < 2:
            self.wpi = 0
        else:
            self.wpi = min(max(0, int(i)), max(0, self.N - 2))
        self.pathComplete = False

    def update(self, p, th, speed):
        p = np.array(p, dtype=np.float64)
        if self.N < 2:
            return 0.0
        seg_idx = self.wpi % max(1, self.N - 1)
        next_idx = (self.wpi + 1) % max(1, self.N - 1)
        wp_1 = self.wp[:, seg_idx]
        wp_2 = self.wp[:, next_idx]
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        if v_mag < 1e-6:
            if self.cyclic or self.wpi < self.N - 2:
                self.wpi = (self.wpi + 1) % max(1, self.N - 1)
            else:
                self.pathComplete = True
            return 0.0
        v_uv = v / v_mag
        s = np.dot(p - wp_1, v_uv)
        s = np.clip(s, 0.0, v_mag)
        ep = wp_1 + v_uv * s
        # Lookahead point: from ep, advance along path by lookahead distance
        cur_pt = ep.copy()
        rem = float(self.lookahead)
        idx = seg_idx
        seg_wp1 = wp_1
        seg_wp2 = wp_2
        seg_v = v
        seg_v_mag = v_mag
        seg_v_uv = v_uv
        seg_s = s
        while rem > 1e-6:
            to_end = seg_v_mag - (np.dot(cur_pt - seg_wp1, seg_v_uv))
            to_end = max(0.0, to_end)
            if to_end >= rem:
                lookahead_pt = cur_pt + seg_v_uv * rem
                rem = 0.0
                break
            rem -= to_end
            cur_pt = seg_wp2.copy()
            if not self.cyclic and idx >= self.N - 2:
                lookahead_pt = seg_wp2.copy()
                rem = 0.0
                break
            idx = (idx + 1) % max(1, self.N - 1)
            nxt = (idx + 1) % max(1, self.N - 1)
            seg_wp1 = self.wp[:, idx]
            seg_wp2 = self.wp[:, nxt]
            seg_v = seg_wp2 - seg_wp1
            seg_v_mag = np.linalg.norm(seg_v)
            if seg_v_mag < 1e-6:
                lookahead_pt = cur_pt
                break
            seg_v_uv = seg_v / seg_v_mag
        else:
            lookahead_pt = cur_pt
        self.p_ref = tuple(lookahead_pt)
        dx = lookahead_pt[0] - p[0]
        dy = lookahead_pt[1] - p[1]
        self.th_ref = np.arctan2(dy, dx)
        alpha = wrap_to_pi(self.th_ref - th)
        ld = max(0.3, np.hypot(dx, dy))
        L_wb = 0.2
        steering = np.arctan2(2.0 * L_wb * np.sin(alpha), ld)
        steering = np.clip(steering, -self.maxSteeringAngle, self.maxSteeringAngle)
        if s >= v_mag - 1e-6:
            if self.cyclic or self.wpi < self.N - 2:
                self.wpi = (self.wpi + 1) % max(1, self.N - 1)
            else:
                self.pathComplete = True
        return float(steering)
