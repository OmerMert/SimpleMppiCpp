"""Plumbing shared by the competitor wrappers.

Config, reference path and obstacle loading, the UDP protocol, nearest-waypoint tracking,
end-of-path detection, solve timing and the summary all live here, because they are
identical across the wrappers. What stays in each run_*.py is only the part specific to
that library: its model, its cost and its controller setup.

UDP protocol, identical to MppiCpp.exe:
  recv  StatePacket   "dddddi" = time, x, y, yaw, v, valid            (from the bridge)
  send  ControlPacket "ddddi"  = time, steer, accel, solve_ms, reset  (to the bridge)
"""
import json
import os
import socket
import struct
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
if PROJ not in sys.path:
    sys.path.insert(0, PROJ)

from obstacles import load_obstacle_circles          # noqa: E402
from scenario import OBSTACLES                       # noqa: E402

# Same forward window as the reference implementation (MizuhoAOKI _get_nearest_waypoint).
SEARCH_FWD = 200

STATE_FMT, CTRL_FMT = "dddddi", "ddddi"
STATE_SZ = struct.calcsize(STATE_FMT)


def load_setup():
    """config.json, the reference path and the obstacle circles, from the shared source."""
    with open(os.path.join(PROJ, "config.json")) as f:
        cfg = json.load(f)
    ref_path = np.genfromtxt(os.path.join(PROJ, cfg["REF_PATH_FILE"]),
                             delimiter=",", skip_header=1)              # (N,4)
    circles = (np.array(load_obstacle_circles(OBSTACLES), dtype=float)
               if OBSTACLES else np.zeros((0, 3)))
    return cfg, ref_path, circles


def footprint(cfg):
    """Vehicle body box from config VEHICLE_FOOTPRINT -> effective (width, length)."""
    fp = cfg.get("VEHICLE_FOOTPRINT", {})
    m = float(fp.get("safety_margin_rate", 1.0))
    return float(fp.get("width", 1.9)) * m, float(fp.get("length", 4.5)) * m


def serve(tag, cfg, ref_path, solve, listen_port, send_port, banner=()):
    """UDP loop: receive state, solve, send control. Only the solve call is timed.

    solve(x, y, yaw, v, idx) -> (steer, accel)
        idx is the nearest waypoint index, computed here with the forward-200 window.
        It may raise IndexError at the end of the path, as the reference MPPI does.
    """
    max_accel = float(cfg["max_accel_abs"])
    path_xy = ref_path[:, :2]
    n_path = len(ref_path)
    idx = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", listen_port))
    sock.settimeout(180.0)      # only fires if the bridge has actually died
    bridge_addr = ("127.0.0.1", send_port)

    for line in banner:
        print(f"[{tag}] {line}")
    print(f"[{tag}] UDP listen {listen_port} -> send {send_port} | waiting for first state...")

    timings, step = [], 0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                print(f"[{tag}] No state from the bridge (timeout). Exiting.")
                break
            if len(data) != STATE_SZ:
                continue
            t_stamp, x, y, yaw, v, valid = struct.unpack(STATE_FMT, data)
            if valid == 0:
                print(f"[{tag}] Stop signal from the bridge (valid=0). Exiting.")
                break

            # nearest waypoint, forward-only like the reference
            seg = path_xy[idx:min(n_path, idx + SEARCH_FWD)]
            idx += int(np.argmin((seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2))

            done = idx >= n_path - 4
            if not done:
                try:
                    t0 = time.perf_counter()
                    steer, accel = solve(x, y, yaw, v, idx)
                    solve_ms = (time.perf_counter() - t0) * 1000.0
                    timings.append(solve_ms)
                except IndexError:
                    done = True

            if done:
                print(f"[{tag}] End of path - done.")
                sock.sendto(struct.pack(CTRL_FMT, t_stamp, 0.0, -max_accel, 0.0, 1),
                            bridge_addr)
                break

            sock.sendto(struct.pack(CTRL_FMT, t_stamp, float(steer), float(accel),
                                    solve_ms, 0), bridge_addr)
            if step % 20 == 0:
                print(f"[{step:4d}] solve={solve_ms:8.2f} ms | pos=({x:+6.1f},{y:+6.1f}) "
                      f"v={v:4.2f} u=({steer:+.2f},{accel:+.2f})")
            step += 1
    except KeyboardInterrupt:
        print(f"\n[{tag}] Ctrl+C")
    finally:
        sock.close()
        if timings:
            a = np.array(timings)
            print(f"\n[{tag}] SOLVE TIME (ms) over {len(a)} steps: mean {a.mean():.2f} | "
                  f"median {np.median(a):.2f} | p95 {np.percentile(a, 95):.2f} | "
                  f"max {a.max():.2f}")
        print(f"[{tag}] (per-step solve_ms is written under runs/ by the bridge)")


def ports():
    """The <listen_port> <send_port> arguments the bridge passes in."""
    return (int(sys.argv[1]) if len(sys.argv) > 1 else 5005,
            int(sys.argv[2]) if len(sys.argv) > 2 else 5006)
