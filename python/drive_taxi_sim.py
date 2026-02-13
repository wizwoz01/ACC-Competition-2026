"""
drive_taxi_sim.py -- Drive the QCar2 through the taxi mission in QLabs (no QUARC).

Ports the FSM + pure-pursuit waypoint follower from taxi_stack_sfun.m and
controls the car entirely through the qvl Python API.

Usage
-----
Terminal 1:  python Setup_Real_Scenario_fullscale_x10.py   (sets up world -- keep running)
Terminal 2:  python drive_taxi_sim.py                       (drives the car)
"""

import os
import sys
import time
import math
import numpy as np

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime

# =============================================================================
#  Tunable parameters
# =============================================================================
LOOP_HZ       = 20                       # control-loop frequency (Hz)
LOOP_DT       = 1.0 / LOOP_HZ

CRUISE_SPEED  = 1.0                      # qvl forward speed while cruising
INIT_HOLD_SEC = 3.0                      # seconds to wait before first move
STOP_HOLD_SEC = 2.0                      # dwell time at pickup / dropoff

COORD_SCALE   = 10.0                     # must match Setup script
ARRIVAL_R     = 0.30 * COORD_SCALE       # arrival radius (scaled)

# Pure pursuit
WHEELBASE     = 0.256 * COORD_SCALE      # full-scale wheelbase in sim coords
LOOKAHEAD     = 3.0                      # lookahead distance (scaled coords)
MAX_STEER     = 0.5                      # max steering angle (rad)

# PID gains on steering
KP = 1.2
KI = 0.05
KD = 0.15

# Speed adaptation in sharp turns
SPEED_TURN_FACTOR = 0.4
STEER_THRESH_SLOW = 0.2

# Key locations (small-scale, auto-scaled)
HUB_XY     = np.array([-1.205, -0.830]) * COORD_SCALE
PICKUP_XY  = np.array([ 0.125,  4.395]) * COORD_SCALE
DROPOFF_XY = np.array([-0.905,  0.800]) * COORD_SCALE

# LED colours [R, G, B] in 0..1
LED_RED    = [1.0, 0.0, 0.0]
LED_GREEN  = [0.0, 1.0, 0.0]
LED_BLUE   = [0.0, 0.0, 1.0]
LED_ORANGE = [1.0, 0.647, 0.0]

# =============================================================================
#  FSM states
# =============================================================================
S_INIT      = 0
S_GO_PICK   = 1
S_STOP_PICK = 2
S_GO_DROP   = 3
S_STOP_DROP = 4
S_RET_HUB   = 5
S_WAIT      = 6

STATE_NAME = {
    S_INIT:      "INIT",
    S_GO_PICK:   "GO_PICKUP",
    S_STOP_PICK: "STOP_PICKUP",
    S_GO_DROP:   "GO_DROPOFF",
    S_STOP_DROP: "STOP_DROPOFF",
    S_RET_HUB:   "RETURN_HUB",
    S_WAIT:      "WAIT_AT_HUB",
}

# =============================================================================
#  Waypoints  (pre-scaled x10, copied from taxi_stack_sfun.m)
# =============================================================================
WP_PICKUP = np.array([
    [-12.05,  -8.30], [-10.50,  -9.00], [ -8.00, -10.00], [ -6.00, -10.50],
    [ -4.00, -10.80], [ -2.00, -10.90], [  0.00, -10.95], [  2.00, -10.95],
    [  4.00, -10.90], [  6.00, -10.85], [  8.00, -10.85], [ 10.00, -10.90],
    [ 12.00, -10.92], [ 14.00, -10.92], [ 16.00, -10.50], [ 18.00,  -9.00],
    [ 19.50,  -7.50], [ 20.50,  -5.50], [ 21.00,  -4.00], [ 21.50,  -3.00],
    [ 21.80,  -1.50], [ 22.00,   0.00], [ 22.10,   2.00], [ 22.15,   4.00],
    [ 22.15,   6.00], [ 22.15,   8.00], [ 22.15,  10.00], [ 22.15,  12.00],
    [ 22.15,  14.00], [ 22.15,  16.00], [ 22.15,  18.00], [ 22.10,  20.00],
    [ 22.10,  22.00], [ 22.10,  24.00], [ 22.10,  26.00], [ 22.10,  28.00],
    [ 22.05,  30.00], [ 22.00,  32.00], [ 21.90,  34.00], [ 21.70,  36.00],
    [ 21.40,  38.00], [ 21.00,  40.00], [ 20.50,  41.50], [ 19.80,  43.00],
    [ 18.80,  44.00], [ 17.50,  44.50], [ 16.00,  44.80], [ 14.00,  44.90],
    [ 12.00,  44.92], [ 10.00,  44.92], [  8.00,  44.92], [  6.00,  44.92],
    [  4.00,  44.92], [  2.00,  44.92], [  1.25,  44.50], [  1.25,  43.95],
])

WP_DROPOFF = np.array([
    [  1.25,  43.95], [  0.00,  44.92], [ -2.00,  44.92], [ -4.00,  44.92],
    [ -6.00,  44.92], [ -8.00,  44.92], [-10.00,  44.92], [-12.00,  44.92],
    [-14.00,  44.92], [-16.00,  44.50], [-17.50,  43.00], [-18.50,  41.00],
    [-19.20,  39.00], [-19.70,  37.00], [-19.95,  35.00], [-20.05,  33.00],
    [-20.08,  31.00], [-20.08,  29.00], [-20.08,  27.00], [-20.08,  25.00],
    [-20.08,  23.00], [-20.08,  21.00], [-20.08,  19.00], [-19.50,  17.00],
    [-19.00,  15.00], [-18.40,  13.00], [-17.60,  11.50], [-16.50,  10.00],
    [-15.00,   9.00], [-13.00,   8.40], [-11.00,   8.20], [ -9.05,   8.00],
])

WP_HUB = np.array([
    [ -9.05,   8.00], [ -7.00,   7.50], [ -5.00,   7.20], [ -3.00,   7.10],
    [ -1.00,   6.00], [  0.00,   4.50], [  0.00,   2.50], [  0.00,   0.50],
    [  0.00,  -1.50], [  0.00,  -3.50], [  0.50,  -5.50], [  1.00,  -7.00],
    [  1.50,  -8.50], [  1.20, -10.00], [  0.00, -10.50], [ -2.00, -10.70],
    [ -4.00, -10.50], [ -6.00, -10.00], [ -8.00,  -9.50], [-10.00,  -8.80],
    [-11.00,  -8.50], [-12.05,  -8.30],
])


# =============================================================================
#  Pure-pursuit helpers
# =============================================================================
def _find_lookahead_point(pos, waypoints, start_idx, la):
    """Line-circle intersection along the path from *start_idx* onward."""
    n = len(waypoints)
    for i in range(max(0, start_idx), n - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        d  = p2 - p1
        f  = p1 - pos
        a  = float(np.dot(d, d))
        b  = 2.0 * float(np.dot(f, d))
        c  = float(np.dot(f, f)) - la * la
        disc = b * b - 4.0 * a * c
        if disc >= 0:
            sq = math.sqrt(disc)
            t2 = (-b + sq) / (2.0 * a)
            t1 = (-b - sq) / (2.0 * a)
            if 0.0 <= t2 <= 1.0:
                return p1 + t2 * d
            if 0.0 <= t1 <= 1.0:
                return p1 + t1 * d
    # Fallback: aim a few waypoints ahead (or the last one)
    return waypoints[min(start_idx + 3, n - 1)]


def pure_pursuit(pos, heading, waypoints, wp_idx, la, wb):
    """Return (steering_angle, updated_wp_idx)."""
    n = len(waypoints)
    wp_idx = max(0, min(wp_idx, n - 1))

    # Advance past waypoints that are behind or too close
    while wp_idx < n - 1:
        wp  = waypoints[wp_idx]
        dx  = wp[0] - pos[0]
        dy  = wp[1] - pos[1]
        dist = math.hypot(dx, dy)
        ang  = math.atan2(dy, dx)
        hdiff = abs(math.atan2(math.sin(ang - heading),
                               math.cos(ang - heading)))
        if dist < la * 0.5 or (dist < la and hdiff > math.pi / 2):
            wp_idx += 1
        else:
            break

    la_pt = _find_lookahead_point(pos, waypoints, wp_idx, la)

    dx = la_pt[0] - pos[0]
    dy = la_pt[1] - pos[1]

    # Transform to vehicle-local frame
    cos_h = math.cos(-heading)
    sin_h = math.sin(-heading)
    local_x = dx * cos_h - dy * sin_h
    local_y = dx * sin_h + dy * cos_h

    L = math.hypot(local_x, local_y)
    if L < 0.01:
        return 0.0, wp_idx

    kappa = 2.0 * local_y / (L * L)
    steer = math.atan(wb * kappa)
    steer = max(-MAX_STEER, min(MAX_STEER, steer))
    return steer, wp_idx


# =============================================================================
#  Main
# =============================================================================
def main():
    os.system("cls" if os.name == "nt" else "clear")

    # ── Connect to QLabs ─────────────────────────────────────────────────────
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    if not qlabs.open("localhost"):
        print("ERROR: Unable to connect to QLabs.")
        print("Make sure Quanser Interactive Labs is running.")
        return
    print("Connected to QLabs")

    # Terminate the QUARC real-time model so it doesn't fight our commands
    try:
        QLabsRealTime().terminate_all_real_time_models()
        print("Terminated QUARC real-time models")
    except Exception as exc:
        print(f"Note: could not terminate RT models ({exc})")

    # Reference the already-spawned QCar2 (actorNumber=0 from Setup script)
    car = QLabsQCar2(qlabs)
    car.actorNumber = 0

    # Read initial state (car should already exist from Setup script)
    ok, loc, rot, f_hit, r_hit = car.set_velocity_and_request_state(
        forward=0, turn=0,
        headlights=True, leftTurnSignal=False, rightTurnSignal=False,
        brakeSignal=False, reverseSignal=False,
    )

    if not ok:
        print("WARNING: no response from QCar2 (actorNumber=0).")
        print("Make sure Setup_Real_Scenario_fullscale_x10.py ran first.")

    pos     = np.array([loc[0], loc[1]]) if ok else HUB_XY.copy()
    heading = rot[2] if ok else 0.0
    print(f"Start pos: ({pos[0]:.1f}, {pos[1]:.1f})  heading: {math.degrees(heading):.1f} deg")

    # ── FSM variables ────────────────────────────────────────────────────────
    state      = S_INIT
    prev_state = -1
    stop_t0    = -1.0
    wp_idx     = 0
    s_integral = 0.0
    s_prev_err = 0.0
    t_start    = time.time()
    iteration  = 0

    car.set_led_strip_uniform(color=LED_RED)

    print(f"\n{'=' * 60}")
    print(f"  Taxi Mission Running   (Ctrl+C to abort)")
    print(f"{'=' * 60}\n")

    try:
        while True:
            t_loop    = time.time()
            t_elapsed = t_loop - t_start

            # ── Distance checks ──────────────────────────────────────────────
            d_pickup  = np.linalg.norm(pos - PICKUP_XY)
            d_dropoff = np.linalg.norm(pos - DROPOFF_XY)
            d_hub     = np.linalg.norm(pos - HUB_XY)

            # ── FSM ─────────────────────────────────────────────────────────
            v_cmd     = 0.0
            led       = LED_RED
            active_wp = None

            if state == S_INIT:
                led = LED_RED
                if t_elapsed >= INIT_HOLD_SEC:
                    state      = S_GO_PICK
                    wp_idx     = 0
                    s_integral = 0.0
                    s_prev_err = 0.0

            elif state == S_GO_PICK:
                v_cmd     = CRUISE_SPEED
                led       = LED_GREEN
                active_wp = WP_PICKUP
                if d_pickup <= ARRIVAL_R:
                    state   = S_STOP_PICK
                    stop_t0 = -1.0

            elif state == S_STOP_PICK:
                led = LED_BLUE
                if stop_t0 < 0:
                    stop_t0 = t_elapsed
                if t_elapsed - stop_t0 >= STOP_HOLD_SEC:
                    state      = S_GO_DROP
                    wp_idx     = 0
                    s_integral = 0.0
                    s_prev_err = 0.0
                    stop_t0    = -1.0

            elif state == S_GO_DROP:
                v_cmd     = CRUISE_SPEED
                led       = LED_GREEN
                active_wp = WP_DROPOFF
                if d_dropoff <= ARRIVAL_R:
                    state   = S_STOP_DROP
                    stop_t0 = -1.0

            elif state == S_STOP_DROP:
                led = LED_ORANGE
                if stop_t0 < 0:
                    stop_t0 = t_elapsed
                if t_elapsed - stop_t0 >= STOP_HOLD_SEC:
                    state      = S_RET_HUB
                    wp_idx     = 0
                    s_integral = 0.0
                    s_prev_err = 0.0
                    stop_t0    = -1.0

            elif state == S_RET_HUB:
                v_cmd     = CRUISE_SPEED
                led       = LED_GREEN
                active_wp = WP_HUB
                if d_hub <= ARRIVAL_R:
                    state = S_WAIT

            else:  # S_WAIT
                led = LED_RED

            # ── Steering (pure pursuit + PID) ────────────────────────────────
            steer_cmd = 0.0
            if v_cmd > 0 and active_wp is not None:
                steer_pp, wp_idx = pure_pursuit(
                    pos, heading, active_wp, wp_idx, LOOKAHEAD, WHEELBASE)

                # PID smoothing
                err         = steer_pp
                s_integral += err * LOOP_DT
                max_int     = MAX_STEER / max(KI, 0.01)
                s_integral  = max(-max_int, min(max_int, s_integral))
                d_err       = (err - s_prev_err) / max(LOOP_DT, 0.001)
                s_prev_err  = err

                steer_cmd = KP * err + KI * s_integral + KD * d_err
                steer_cmd = max(-MAX_STEER, min(MAX_STEER, steer_cmd))

                # Speed adaptation for sharp turns
                abs_s = abs(steer_cmd)
                if abs_s > STEER_THRESH_SLOW:
                    factor = 1.0 - (1.0 - SPEED_TURN_FACTOR) * min(
                        1.0,
                        (abs_s - STEER_THRESH_SLOW)
                        / (MAX_STEER - STEER_THRESH_SLOW),
                    )
                    v_cmd *= factor

            # ── Send command & read state ────────────────────────────────────
            ok, loc, rot, f_hit, r_hit = car.set_velocity_and_request_state(
                forward=v_cmd,
                turn=steer_cmd,
                headlights=True,
                leftTurnSignal=False,
                rightTurnSignal=False,
                brakeSignal=(v_cmd == 0),
                reverseSignal=False,
            )
            if ok:
                pos     = np.array([loc[0], loc[1]])
                heading = rot[2]

            # ── LED on state change ──────────────────────────────────────────
            if state != prev_state:
                car.set_led_strip_uniform(color=led)
                print(
                    f"[{t_elapsed:6.1f}s]  >> {STATE_NAME[state]:15s}  "
                    f"pos=({pos[0]:7.1f}, {pos[1]:7.1f})  "
                    f"hdg={math.degrees(heading):6.1f} deg"
                )
                prev_state = state

            # ── Periodic status (every 5 s while driving) ────────────────────
            if v_cmd > 0 and iteration % (5 * LOOP_HZ) == 0:
                print(
                    f"[{t_elapsed:6.1f}s]     {STATE_NAME[state]:15s}  "
                    f"pos=({pos[0]:7.1f}, {pos[1]:7.1f})  "
                    f"v={v_cmd:.2f}  steer={math.degrees(steer_cmd):5.1f} deg  "
                    f"wp={wp_idx}"
                )

            # ── Mission complete ─────────────────────────────────────────────
            if state == S_WAIT:
                car.set_velocity_and_request_state(
                    forward=0, turn=0,
                    headlights=False, leftTurnSignal=False,
                    rightTurnSignal=False, brakeSignal=True,
                    reverseSignal=False,
                )
                car.set_led_strip_uniform(color=LED_RED)
                print(f"\n{'=' * 60}")
                print(f"  Mission Complete!  Car returned to hub.")
                print(f"{'=' * 60}")
                break

            # ── Loop timing ──────────────────────────────────────────────────
            iteration += 1
            elapsed = time.time() - t_loop
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nAborted by user -- stopping car.")
        car.set_velocity_and_request_state(
            forward=0, turn=0,
            headlights=False, leftTurnSignal=False,
            rightTurnSignal=False, brakeSignal=True,
            reverseSignal=False,
        )

    qlabs.close()
    print("QLabs connection closed.  Done.")


if __name__ == "__main__":
    main()
