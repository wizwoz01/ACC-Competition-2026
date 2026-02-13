"""
drive_taxi_sim.py -- Drive the QCar2 through the taxi mission using lane detection.

Uses the front CSI camera to detect white/yellow lane markings and stay
centred in the lane.  Falls back to heading-toward-goal when no lanes are
visible.  Opens a live OpenCV window showing the detected lanes.

Usage
-----
Terminal 1:  python Setup_Real_Scenario_fullscale_x10.py   (sets up world -- keep running)
Terminal 2:  python drive_taxi_sim.py                       (drives the car)

Press 'q' in the debug window to close it (car keeps driving).
Press Ctrl+C in the terminal to stop the car.
"""

import os
import sys
import time
import math
import numpy as np
import cv2

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

# Steering gains
K_LANE        = 2.0                      # proportional gain on lane offset
K_GOAL        = 1.5                      # gain for heading-to-goal fallback
SMOOTH_ALPHA  = 0.3                      # lane-offset EMA  (0 = freeze, 1 = raw)
MAX_STEER     = 0.5                      # max steering angle (rad)

# Speed adaptation in sharp turns
SPEED_TURN_FACTOR = 0.4
STEER_THRESH_SLOW = 0.2

# Key locations  (small-scale coords, auto-scaled by COORD_SCALE)
HUB_XY     = np.array([-1.205, -0.830]) * COORD_SCALE
PICKUP_XY  = np.array([ 0.125,  4.395]) * COORD_SCALE
DROPOFF_XY = np.array([-0.905,  0.800]) * COORD_SCALE

# LED colours [R, G, B] in 0..1
LED_RED    = [1.0, 0.0, 0.0]
LED_GREEN  = [0.0, 1.0, 0.0]
LED_BLUE   = [0.0, 0.0, 1.0]
LED_ORANGE = [1.0, 0.647, 0.0]

# Lane-detection thresholds
WHITE_THRESH    = 200
YELLOW_H_LOW    = 15       # OpenCV HSV hue range (0-180)
YELLOW_H_HIGH   = 40
YELLOW_S_LOW    = 80
YELLOW_V_LOW    = 80
MIN_LANE_PIXELS = 20
ROI_TOP_FRAC    = 0.5      # only look at bottom half of image

# How many consecutive no-lane frames before switching to goal steering
NO_LANE_LIMIT  = 5

# Set to False to disable the debug window
SHOW_DEBUG_WINDOW = True

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
#  Lane detection  (ported from detect_lanes.m / lane_detect_sfun.m)
# =============================================================================
def detect_lane_offset(bgr_image):
    """Detect lane-centre offset from a BGR camera frame.

    Returns
    -------
    offset : float
        -1 .. +1   (negative = car is left of centre)
    confidence : float
        0.0  no lanes, 0.5  one lane, 1.0  both lanes
    debug : dict
        Intermediate data for the debug visualisation.
    """
    h, w = bgr_image.shape[:2]

    # ROI: bottom portion of image (where lane markings are closest)
    roi_top = int(h * ROI_TOP_FRAC)
    roi = bgr_image[roi_top:, :]
    roi_h, roi_w = roi.shape[:2]

    # ── White lane detection (grayscale threshold) ──
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, white_mask = cv2.threshold(gray, WHITE_THRESH, 255, cv2.THRESH_BINARY)

    # ── Yellow lane detection (HSV) ──
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(
        hsv,
        np.array([YELLOW_H_LOW, YELLOW_S_LOW, YELLOW_V_LOW]),
        np.array([YELLOW_H_HIGH, 255, 255]),
    )

    # ── Combined mask ──
    lane_mask = cv2.bitwise_or(white_mask, yellow_mask)

    # Focus on the bottom half of the ROI for reliability
    bottom_start = roi_h // 2
    bottom = lane_mask[bottom_start:, :]
    bw = bottom.shape[1]
    centre = bw / 2.0

    cols = np.where(bottom > 0)[1]

    left_x  = None
    right_x = None
    offset  = 0.0
    conf    = 0.0

    if len(cols) > 0:
        left_cols  = cols[cols < centre]
        right_cols = cols[cols > centre]

        if len(left_cols) >= MIN_LANE_PIXELS and len(right_cols) >= MIN_LANE_PIXELS:
            left_x  = float(np.median(left_cols))
            right_x = float(np.median(right_cols))
            lane_centre = (left_x + right_x) / 2.0
            offset = float(np.clip((lane_centre - centre) / w, -1.0, 1.0))
            conf   = 1.0
        elif len(left_cols) >= MIN_LANE_PIXELS:
            left_x = float(np.median(left_cols))
            offset = -0.15
            conf   = 0.5
        elif len(right_cols) >= MIN_LANE_PIXELS:
            right_x = float(np.median(right_cols))
            offset  =  0.15
            conf    = 0.5

    debug = dict(
        lane_mask=lane_mask,
        roi_top=roi_top,
        bottom_start=bottom_start,
        left_x=left_x,
        right_x=right_x,
        centre=centre,
    )
    return offset, conf, debug


def draw_lane_debug(bgr_image, debug, offset, confidence,
                    steer_deg, mode, state_name):
    """Draw lane-detection overlay on a copy of the camera image.

    Returns the annotated BGR image.
    """
    vis = bgr_image.copy()
    h, w = vis.shape[:2]

    roi_top      = debug["roi_top"]
    bottom_start = debug["bottom_start"]
    lane_mask    = debug["lane_mask"]
    left_x       = debug["left_x"]
    right_x      = debug["right_x"]
    centre       = debug["centre"]

    # ── ROI boundary (cyan dashed line) ──
    cv2.line(vis, (0, roi_top), (w, roi_top), (255, 255, 0), 1)

    # ── Overlay lane mask in green on the ROI ──
    roi_slice = vis[roi_top:, :]
    mask_colour = np.zeros_like(roi_slice)
    mask_colour[:, :, 1] = lane_mask   # green channel
    vis[roi_top:] = cv2.addWeighted(roi_slice, 0.7, mask_colour, 0.3, 0)

    # Vertical reference: image centre (thin white)
    cx = int(centre)
    y_start = roi_top + bottom_start
    cv2.line(vis, (cx, y_start), (cx, h), (255, 255, 255), 1)

    # ── Left lane position (blue) ──
    if left_x is not None:
        lx = int(left_x)
        cv2.line(vis, (lx, y_start), (lx, h), (255, 0, 0), 2)

    # ── Right lane position (red) ──
    if right_x is not None:
        rx = int(right_x)
        cv2.line(vis, (rx, y_start), (rx, h), (0, 0, 255), 2)

    # ── Detected lane centre (green) ──
    if left_x is not None and right_x is not None:
        lcx = int((left_x + right_x) / 2.0)
        cv2.line(vis, (lcx, y_start), (lcx, h), (0, 255, 0), 2)

    # ── Text overlay ──
    font  = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thick = 1
    white = (255, 255, 255)
    black = (0, 0, 0)
    y_txt = 20

    def put(text, y):
        cv2.putText(vis, text, (6, y), font, scale, black, thick + 1)
        cv2.putText(vis, text, (5, y), font, scale, white, thick)

    put(f"State: {state_name}", y_txt)
    put(f"Mode: {mode}", y_txt + 20)
    put(f"Offset: {offset:+.3f}  Conf: {confidence:.1f}", y_txt + 40)
    put(f"Steer: {steer_deg:+.1f} deg", y_txt + 60)

    return vis


# =============================================================================
#  Helpers
# =============================================================================
def angle_wrap(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


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

    # ── Reference the already-spawned QCar2 (actorNumber=0 from Setup) ─────
    car = QLabsQCar2(qlabs)
    car.actorNumber = 0

    # Read initial state
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
    state           = S_INIT
    prev_state      = -1
    stop_t0         = -1.0
    t_start         = time.time()
    iteration       = 0
    smoothed_offset = 0.0
    no_lane_count   = 0
    show_window     = SHOW_DEBUG_WINDOW

    car.set_led_strip_uniform(color=LED_RED)

    if show_window:
        cv2.namedWindow("Lane Detection", cv2.WINDOW_NORMAL)

    print(f"\n{'=' * 60}")
    print(f"  Taxi Mission (Lane Detection)   Ctrl+C to abort")
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
            v_cmd   = 0.0
            led     = LED_RED
            goal_xy = HUB_XY            # default target for fallback steering

            if state == S_INIT:
                led = LED_RED
                if t_elapsed >= INIT_HOLD_SEC:
                    state           = S_GO_PICK
                    smoothed_offset = 0.0
                    no_lane_count   = 0

            elif state == S_GO_PICK:
                v_cmd   = CRUISE_SPEED
                led     = LED_GREEN
                goal_xy = PICKUP_XY
                if d_pickup <= ARRIVAL_R:
                    state   = S_STOP_PICK
                    stop_t0 = -1.0

            elif state == S_STOP_PICK:
                led     = LED_BLUE
                goal_xy = PICKUP_XY
                if stop_t0 < 0:
                    stop_t0 = t_elapsed
                if t_elapsed - stop_t0 >= STOP_HOLD_SEC:
                    state           = S_GO_DROP
                    stop_t0         = -1.0
                    smoothed_offset = 0.0
                    no_lane_count   = 0

            elif state == S_GO_DROP:
                v_cmd   = CRUISE_SPEED
                led     = LED_GREEN
                goal_xy = DROPOFF_XY
                if d_dropoff <= ARRIVAL_R:
                    state   = S_STOP_DROP
                    stop_t0 = -1.0

            elif state == S_STOP_DROP:
                led     = LED_ORANGE
                goal_xy = DROPOFF_XY
                if stop_t0 < 0:
                    stop_t0 = t_elapsed
                if t_elapsed - stop_t0 >= STOP_HOLD_SEC:
                    state           = S_RET_HUB
                    stop_t0         = -1.0
                    smoothed_offset = 0.0
                    no_lane_count   = 0

            elif state == S_RET_HUB:
                v_cmd   = CRUISE_SPEED
                led     = LED_GREEN
                goal_xy = HUB_XY
                if d_hub <= ARRIVAL_R:
                    state = S_WAIT

            else:  # S_WAIT
                led = LED_RED

            # ── Steering ────────────────────────────────────────────────────
            steer_cmd       = 0.0
            lane_confidence = 0.0
            lane_offset_raw = 0.0
            debug_info      = None

            if v_cmd > 0:
                # Grab front-camera frame
                try:
                    img_ok, camera_image = car.get_image(
                        camera=car.CAMERA_CSI_FRONT)
                except Exception:
                    img_ok = False
                    camera_image = None

                if (img_ok
                        and camera_image is not None
                        and camera_image.size > 0):
                    # Strip alpha channel if present
                    if camera_image.ndim == 3 and camera_image.shape[2] == 4:
                        camera_image = camera_image[:, :, :3]

                    lane_offset_raw, lane_confidence, debug_info = \
                        detect_lane_offset(camera_image)

                    if lane_confidence > 0:
                        smoothed_offset = (SMOOTH_ALPHA * lane_offset_raw
                                           + (1 - SMOOTH_ALPHA) * smoothed_offset)
                        steer_cmd     = -K_LANE * smoothed_offset
                        no_lane_count = 0
                    else:
                        no_lane_count += 1
                else:
                    no_lane_count += 1

                # Fallback: heading-toward-goal when lanes lost for a while
                if no_lane_count > NO_LANE_LIMIT:
                    dx = goal_xy[0] - pos[0]
                    dy = goal_xy[1] - pos[1]
                    heading_err = angle_wrap(math.atan2(dy, dx) - heading)
                    steer_cmd       = K_GOAL * heading_err
                    smoothed_offset *= 0.9      # decay stale offset

                # Clamp
                steer_cmd = max(-MAX_STEER, min(MAX_STEER, steer_cmd))

                # Slow down in sharp turns
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

            # ── Debug visualisation window ───────────────────────────────────
            if show_window and debug_info is not None:
                mode = "LANE" if no_lane_count <= NO_LANE_LIMIT else "GOAL"
                vis = draw_lane_debug(
                    camera_image, debug_info,
                    lane_offset_raw, lane_confidence,
                    math.degrees(steer_cmd), mode, STATE_NAME[state],
                )
                cv2.imshow("Lane Detection", vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    show_window = False
                    cv2.destroyWindow("Lane Detection")

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
                mode = "LANE" if no_lane_count <= NO_LANE_LIMIT else "GOAL"
                print(
                    f"[{t_elapsed:6.1f}s]     {STATE_NAME[state]:15s}  "
                    f"pos=({pos[0]:7.1f}, {pos[1]:7.1f})  "
                    f"v={v_cmd:.2f}  steer={math.degrees(steer_cmd):5.1f} deg  "
                    f"mode={mode}  conf={lane_confidence:.1f}"
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

    cv2.destroyAllWindows()
    qlabs.close()
    print("QLabs connection closed.  Done.")


if __name__ == "__main__":
    main()
