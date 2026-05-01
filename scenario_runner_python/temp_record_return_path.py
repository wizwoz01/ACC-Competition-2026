"""
Temporary manual-drive recorder for collecting three waypoint paths.

Controls (Pygame keyboard window must be focused):
  - W/S/A/D : drive
  - SPACE   : toggle recording on/off for current path
  - I       : switch current target path (pickup -> dropoff -> hub)
  - Q       : save and quit
  - ESC     : quit without saving
"""

import argparse
import math
import socket
import struct
import time
from pathlib import Path


def now() -> float:
    return time.time()


class _QLabsWorldTransform:
    _QCAR2_CLASS_ID = 161
    _FCN_REQUEST_WORLD_TRANSFORM = 3
    _FCN_RESPONSE_WORLD_TRANSFORM = 4
    _BASE_CONTAINER_SIZE = 13

    def __init__(self, host: str = "localhost", port: int = 18000, actor: int = 0, timeout: float = 5.0):
        self._actor = int(actor)
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock: socket.socket | None = None
        self._connect()

    def _connect(self) -> None:
        self.close()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self._timeout)
            s.connect((self._host, self._port))
            s.settimeout(self._timeout)
            self._sock = s
        except Exception:
            self._sock = None

    def _send_container(self, class_id: int, actor: int, func: int, payload: bytes = b"") -> bool:
        if self._sock is None:
            return False
        container_size = self._BASE_CONTAINER_SIZE + len(payload)
        pkt = (
            struct.pack("<i", 1 + container_size)
            + struct.pack(">BiiiB", 123, container_size, class_id, actor, func)
            + payload
        )
        try:
            self._sock.sendall(pkt)
            return True
        except Exception:
            return False

    def _wait_for_container(self, class_id: int, actor: int, func: int) -> bytes | None:
        if self._sock is None:
            return None
        deadline = time.time() + self._timeout
        recv_buf = bytearray()
        while time.time() < deadline:
            try:
                self._sock.settimeout(max(0.05, deadline - time.time()))
                chunk = self._sock.recv(4096)
                if chunk:
                    recv_buf.extend(chunk)
            except socket.timeout:
                pass
            except Exception:
                return None

            while len(recv_buf) >= 5:
                pkt_len_raw = struct.unpack("<I", recv_buf[0:4])[0]
                total_pkt = 4 + pkt_len_raw
                if len(recv_buf) < total_pkt:
                    break
                if recv_buf[4] != 123:
                    recv_buf = recv_buf[1:]
                    continue

                idx = 5
                found_payload = None
                while idx + self._BASE_CONTAINER_SIZE <= total_pkt:
                    c_size = struct.unpack(">I", recv_buf[idx:idx + 4])[0]
                    c_cls = struct.unpack(">I", recv_buf[idx + 4:idx + 8])[0]
                    c_act = struct.unpack(">I", recv_buf[idx + 8:idx + 12])[0]
                    c_func = recv_buf[idx + 12]
                    payload_start = idx + self._BASE_CONTAINER_SIZE
                    payload_end = idx + c_size
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

    def get_world_transform(self):
        location = [0.0, 0.0, 0.0]
        rotation = [0.0, 0.0, 0.0]
        scale = [0.0, 0.0, 0.0]
        if self._sock is not None:
            self._sock.setblocking(False)
            try:
                while self._sock.recv(4096):
                    pass
            except Exception:
                pass
            self._sock.setblocking(True)
            self._sock.settimeout(self._timeout)

        if not self._send_container(self._QCAR2_CLASS_ID, self._actor, self._FCN_REQUEST_WORLD_TRANSFORM):
            return False, location, rotation, scale

        payload = self._wait_for_container(self._QCAR2_CLASS_ID, self._actor, self._FCN_RESPONSE_WORLD_TRANSFORM)
        if payload is not None and len(payload) == 36:
            vals = struct.unpack(">fffffffff", payload[0:36])
            location = [vals[0], vals[1], vals[2]]
            rotation = [vals[3], vals[4], vals[5]]
            scale = [vals[6], vals[7], vals[8]]
            return True, location, rotation, scale
        return False, location, rotation, scale

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None


def total_path_length(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    dist = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        dist += math.hypot(dx, dy)
    return dist


def _append_path_block(lines: list[str], path_name: str, points: list[tuple[float, float]]) -> None:
    lines.extend([f"{path_name}:", "", "["])
    for idx, (x, y) in enumerate(points):
        suffix = ";" if idx < len(points) - 1 else ""
        lines.append(f"{x:.4f}, {y:.4f}{suffix}")
    lines.extend(["]", ""])


def write_path_file(out_file: Path, path_points: dict[str, list[tuple[float, float]]]) -> None:
    lines: list[str] = []
    for name in ("path_to_pickup", "path_to_dropoff", "path_to_hub"):
        _append_path_block(lines, name, path_points[name])
    out_file.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    # Parse CLI first so --help does not initialize QCar hardware libraries.
    ap = argparse.ArgumentParser()
    ap.add_argument("--actor", type=int, default=0)
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--max-throttle", type=float, default=0.12)
    ap.add_argument("--max-steer", type=float, default=0.30)
    ap.add_argument("--spacing-m", type=float, default=0.10)
    ap.add_argument("--out", default="waypoints_return_test.txt")
    args = ap.parse_args()

    dt = 1.0 / max(5.0, float(args.rate))
    out_path = Path(args.out).resolve()

    from pal.products.qcar import QCar
    from pal.utilities.keyboard import PygameKeyboard, PygameKeyboardDrive

    keyb = PygameKeyboard()
    drive = PygameKeyboardDrive(maxThrottle=float(args.max_throttle), maxSteer=float(args.max_steer))
    car = QCar(readMode=0)
    qlabs = _QLabsWorldTransform(actor=int(args.actor), timeout=0.2)

    path_names = ["path_to_pickup", "path_to_dropoff", "path_to_hub"]
    path_points: dict[str, list[tuple[float, float]]] = {name: [] for name in path_names}
    active_path_idx = 0
    recording = False
    prev_space = False
    prev_i = False

    print("Recorder started.")
    print("Focus the keyboard window and drive with W/S/A/D.")
    print("Press SPACE to start/stop recording, I to switch path, Q to save+quit, ESC to quit without saving.")
    print("Target paths: path_to_pickup, path_to_dropoff, path_to_hub")
    print(f"Current path: {path_names[active_path_idx]}")
    print(f"Output file: {out_path}")

    try:
        while True:
            t0 = now()
            keyb.read()

            if keyb.k_esc:
                print("ESC pressed. Exiting without saving.")
                break

            if keyb.k_q:
                missing = [name for name in path_names if len(path_points[name]) < 2]
                if missing:
                    missing_txt = ", ".join(missing)
                    print(f"Cannot save yet. Need at least 2 points in: {missing_txt}")
                    break
                write_path_file(out_path, path_points)
                print(f"Saved all three paths to {out_path}")
                for name in path_names:
                    pts = path_points[name]
                    length_m = total_path_length(pts)
                    print(
                        f"{name}: n={len(pts)} length={length_m:.2f}m "
                        f"start=({pts[0][0]:+.3f},{pts[0][1]:+.3f}) "
                        f"end=({pts[-1][0]:+.3f},{pts[-1][1]:+.3f})"
                    )
                break

            if keyb.k_space and not prev_space:
                recording = not recording
                active_name = path_names[active_path_idx]
                print(f"Recording {'ON' if recording else 'OFF'} for {active_name}")
            prev_space = keyb.k_space

            if keyb.k_i and not prev_i:
                recording = False
                active_path_idx = (active_path_idx + 1) % len(path_names)
                active_name = path_names[active_path_idx]
                print(f"Switched to {active_name} (recording OFF)")
            prev_i = keyb.k_i

            steer, throttle = drive.update(keyb)
            car.read_write_std(throttle=float(throttle), steering=float(steer))

            ok, loc, _, _ = qlabs.get_world_transform()
            if ok and recording:
                x = float(loc[0]) * 0.10
                y = float(loc[1]) * 0.10
                active_name = path_names[active_path_idx]
                points = path_points[active_name]
                if not points:
                    points.append((x, y))
                else:
                    dx = x - points[-1][0]
                    dy = y - points[-1][1]
                    if math.hypot(dx, dy) >= float(args.spacing_m):
                        points.append((x, y))

            sleep_dt = dt - (now() - t0)
            if sleep_dt > 0.0:
                time.sleep(sleep_dt)

    finally:
        try:
            car.read_write_std(throttle=0.0, steering=0.0)
        except Exception:
            pass
        try:
            qlabs.close()
        except Exception:
            pass
        try:
            car.terminate()
        except Exception:
            pass
        try:
            keyb.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
