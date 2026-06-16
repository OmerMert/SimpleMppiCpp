"""
Run command:
$env:PYTHONUNBUFFERED="1"; $env:TEMP="D:\isaac_tmp"; $env:TMP="D:\isaac_tmp"
D:\isaacsim\_build\windows-x86_64\release\python.bat isaacsim_bridge.py
"""

"""
isaacsim_bridge.py - Isaac Sim <-> C++ MPPI bridge (Phase 1: kinematic bicycle)

Drives a vehicle in Isaac Sim under the existing C++ MPPI controller. The
"physics" here is the SAME kinematic bicycle model the MPPI itself uses (see
Vehicle.cpp), integrated in Python. The vehicle therefore lives directly in
MPPI coordinates, so there is NO world<->MPPI frame transform. This gives a
zero model-mismatch sanity check of the whole pipeline (UDP <-> C++ MPPI,
reference path, control) inside Isaac Sim's 3D scene. Physics fidelity (PhysX
vehicle) is a later phase.

The C++ side is UNCHANGED: it is launched in "isaac" mode, which is just an
"external simulator over UDP" mode (SimulateIsaac in main.cpp).

UDP protocol (see UDP.h):
  Python -> C++  StatePacket   struct "dddddi" = (time, x, y, yaw, v, valid)  44B
  C++ -> Python  ControlPacket struct "dddi"   = (time, steer, accel, reset)  28B

Run with Isaac Sim's bundled Python (PYTHONUNBUFFERED=1 recommended for live logs):
  D:\isaacsim\_build\windows-x86_64\release\python.bat isaacsim_bridge.py
Options:
  --headless            run without the GUI window
  <path.csv>            override REF_PATH_FILE from config.json (relative to repo)
"""
import os
import sys
import math
import time
import json
import csv
import socket
import struct
import subprocess
import traceback


def log(msg):
    """Print and flush immediately (Isaac's stdout is block-buffered to a file)."""
    print(msg, flush=True)


# --------------------------------------------------------------------------
# Config / paths  (read BEFORE SimulationApp so we can fail fast with a clear
# message instead of after a slow Isaac Sim startup).
# --------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def repo(rel):
    return os.path.join(SCRIPT_DIR, rel)


with open(repo("config.json"), "r") as _f:
    CFG = json.load(_f)

DT         = float(CFG["delta_t"])          # 0.05 s  (20 Hz, matches MPPI)
WHEEL_BASE = float(CFG["wheel_base"])       # L [m]
MAX_STEER  = float(CFG["max_steer_abs"])    # [rad]
MAX_ACCEL  = float(CFG["max_accel_abs"])    # [m/s^2]

HEADLESS = "--headless" in sys.argv
_pos_args = [a for a in sys.argv[1:] if not a.startswith("--")]
REF_PATH_FILE = _pos_args[0] if _pos_args else CFG["REF_PATH_FILE"]

# Visual-only vehicle box dimensions (length along +x, width along +y).
_vc = CFG.get("VEHICLE_CONFIG", {})
VEH_L = float(_vc.get("L", 4.0))
VEH_W = float(_vc.get("W", 2.0))
VEH_H = 1.5

CPP_LISTEN_PORT = 5005   # C++ binds here -> Python sends state here
PY_LISTEN_PORT  = 5006   # C++ sends here -> Python listens here
MPPI_EXE = repo("MppiCpp.exe")

# End-of-path detection (matches SimulateIsaac conventions in main.cpp).
END_RADIUS_M  = 5.0
MIN_END_STEPS = 100

STATE_FMT, CTRL_FMT = "dddddi", "dddi"
CTRL_SZ = struct.calcsize(CTRL_FMT)


def load_path_csv(filepath):
    pts = []
    with open(filepath, "r") as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"])))
    return pts


PATH = load_path_csv(repo(REF_PATH_FILE))
log(f"[Bridge] Path loaded: {len(PATH)} points from {REF_PATH_FILE}")

# --------------------------------------------------------------------------
# Launch Isaac Sim. SimulationApp MUST be created before importing any
# omni.* / isaacsim.* modules.
# --------------------------------------------------------------------------
from isaacsim import SimulationApp                     # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np                                     # noqa: E402
from isaacsim.core.api import World                    # noqa: E402
from isaacsim.core.api.objects import VisualCuboid     # noqa: E402
from pxr import UsdGeom, Gf                            # noqa: E402
import omni.usd                                        # noqa: E402

# Optional helpers (wrapped: tolerate API-name drift across Isaac versions).
try:
    from isaacsim.core.utils.viewports import set_camera_view
except Exception:
    set_camera_view = None

try:
    from isaacsim.util.debug_draw import _debug_draw
    _draw = _debug_draw.acquire_debug_draw_interface()
except Exception:
    _draw = None


def yaw_to_quat_wxyz(yaw):
    """Quaternion (w, x, y, z) for a rotation of `yaw` about +Z."""
    h = 0.5 * yaw
    return np.array([math.cos(h), 0.0, 0.0, math.sin(h)])


def pack_state(t, x, y, yaw, v, valid):
    return struct.pack(STATE_FMT, t, x, y, yaw, max(0.0, v), valid)


def build_ref_path_curve(stage, pts):
    """Draw the reference path as a static green BasisCurves polyline."""
    curves = UsdGeom.BasisCurves.Define(stage, "/World/ref_path")
    curves.CreateTypeAttr("linear")
    curves.CreateCurveVertexCountsAttr([len(pts)])
    curves.CreatePointsAttr([Gf.Vec3f(float(x), float(y), 0.05) for (x, y) in pts])
    curves.CreateWidthsAttr([0.25] * len(pts))
    curves.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
    curves.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.9, 0.2)])


def make_socket():
    """UDP socket bound to the bridge's listen port.

    NOTE: Python's socket.ioctl does NOT support SIO_UDP_CONNRESET, so we cannot
    disable the Windows "ICMP port-unreachable -> WinError 10054 on next recv"
    behaviour here. Instead we simply CATCH ConnectionResetError in the handshake
    and main loop (it fires when we send to the C++ port before the exe has
    bound it) and retry -- which is sufficient."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", PY_LISTEN_PORT))
    return s


def launch_mppi():
    """Launch the C++ MPPI exe in 'isaac' (external-sim-over-UDP) mode.

    C++ arg order (main.cpp): <cpp_listen_port> <py_send_port> <mode>.
    cwd must be the repo so it finds config.json and data/ by relative path.
    """
    if not os.path.isfile(MPPI_EXE):
        log(f"[Bridge] ERROR: MPPI exe not found: {MPPI_EXE}")
        return None
    cmd = [MPPI_EXE, str(CPP_LISTEN_PORT), str(PY_LISTEN_PORT), "isaac"]
    log(f"[Bridge] Launching MPPI: {' '.join(cmd)}")
    return subprocess.Popen(cmd, cwd=SCRIPT_DIR)


def handshake(sock, cpp_addr, x, y, yaw, v, t0, timeout_s=30.0):
    """Resend the initial state until the C++ exe (which may still be loading
    config / path / costmap and binding its socket) replies with the first
    control. Tolerates dropped first packets and Windows connection-reset."""
    log("[Bridge] Handshake: waiting for C++ MPPI to come online...")
    sock.settimeout(0.5)
    deadline = time.time() + timeout_s
    attempts = 0
    while time.time() < deadline:
        attempts += 1
        try:
            sock.sendto(pack_state(time.time() - t0, x, y, yaw, v, 1), cpp_addr)
            data, _ = sock.recvfrom(1024)
            if len(data) == CTRL_SZ:
                _, steer, accel, reset = struct.unpack(CTRL_FMT, data)
                log(f"[Bridge] Handshake OK after {attempts} attempt(s): "
                    f"first u=(s{steer:+.3f}, a{accel:+.2f})")
                return (steer, accel, reset)
        except socket.timeout:
            pass
        except ConnectionResetError:
            time.sleep(0.2)  # exe not bound yet -> back off and retry
    return None


def main():
    # ---- scene ----
    world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)
    world.scene.add_default_ground_plane()

    vehicle = world.scene.add(VisualCuboid(
        prim_path="/World/vehicle",
        name="vehicle",
        position=np.array([PATH[0][0], PATH[0][1], 0.5 * VEH_H]),
        scale=np.array([VEH_L, VEH_W, VEH_H]),
        color=np.array([0.1, 0.5, 1.0]),
    ))

    stage = omni.usd.get_context().get_stage()
    build_ref_path_curve(stage, PATH)

    world.reset()

    # Frame the camera on the path (from behind/above).
    if set_camera_view is not None:
        xs = [p[0] for p in PATH]
        ys = [p[1] for p in PATH]
        cx, cy = 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys))
        span = max(max(xs) - min(xs), max(ys) - min(ys), 20.0)
        try:
            set_camera_view(eye=[cx - 0.2 * span, cy - 0.6 * span, 0.8 * span],
                            target=[cx, cy, 0.0])
        except Exception as e:
            log(f"[Bridge] camera framing skipped: {e}")

    # ---- UDP + C++ exe ----
    sock = make_socket()
    cpp_addr = ("127.0.0.1", CPP_LISTEN_PORT)
    log(f"[Bridge] UDP listening on 127.0.0.1:{PY_LISTEN_PORT}")

    mppi_proc = launch_mppi()
    if mppi_proc is None:
        simulation_app.close()
        return

    # Vehicle state in the MPPI frame: [x, y, yaw, v]. Start at path[0].
    x, y, yaw, v = PATH[0][0], PATH[0][1], 0.0, 0.0
    last_x, last_y = PATH[-1]
    trajectory = []
    t0 = time.time()
    step = 0

    # First contact (handles slow exe startup + Windows UDP quirks).
    first = handshake(sock, cpp_addr, x, y, yaw, v, t0)
    if first is None:
        log("[Bridge] ERROR: no response from MPPI exe (handshake timeout).")
        if mppi_proc.poll() is None:
            mppi_proc.terminate()
        simulation_app.close()
        return
    steer, accel, reset = first
    sock.settimeout(5.0)
    pending_control = first   # apply the handshake's control as the first step

    log("[Bridge] MPPI loop starting...")
    try:
        while simulation_app.is_running():
            loop_start = time.perf_counter()

            # Use the control we already hold (from handshake or previous recv).
            steer, accel, reset = pending_control
            if reset:
                log("[Bridge] C++ signalled done/reset (end of path).")
                break

            # integrate the kinematic bicycle model -- EXACT mirror of
            # Vehicle::update() in Vehicle.cpp (rear-axle, forward Euler).
            steer = max(-MAX_STEER, min(MAX_STEER, steer))
            accel = max(-MAX_ACCEL, min(MAX_ACCEL, accel))
            x   += v * math.cos(yaw) * DT
            y   += v * math.sin(yaw) * DT
            yaw += v / WHEEL_BASE * math.tan(steer) * DT
            v   += accel * DT
            if v < 0.0:
                v = 0.0
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # wrap to [-pi, pi]

            # move the vehicle visual + draw trajectory + render one frame
            vehicle.set_world_pose(np.array([x, y, 0.5 * VEH_H]),
                                   yaw_to_quat_wxyz(yaw))
            trajectory.append((x, y))
            if _draw is not None and len(trajectory) > 1:
                pts = [(px, py, 0.1) for (px, py) in trajectory]
                _draw.clear_lines()
                _draw.draw_lines(pts[:-1], pts[1:],
                                 [(1.0, 0.1, 0.1, 1.0)] * (len(pts) - 1),
                                 [3.0] * (len(pts) - 1))
            world.step(render=True)
            step += 1

            # end-of-path check (within END_RADIUS_M of the last waypoint).
            if step > MIN_END_STEPS and math.hypot(x - last_x, y - last_y) < END_RADIUS_M:
                log(f"\n[Bridge] PATH COMPLETED: {step} steps, {time.time() - t0:.1f}s")
                for _ in range(3):
                    sock.sendto(pack_state(time.time() - t0, x, y, yaw, v, 0), cpp_addr)
                    time.sleep(0.02)
                break

            if step % 20 == 0:
                log(f"[{step:4d}] t={time.time()-t0:5.1f}s "
                    f"pos=({x:+7.2f},{y:+7.2f}) yaw={math.degrees(yaw):+6.1f} "
                    f"v={v:4.1f} | u=(s{steer:+.3f}, a{accel:+.2f})")

            # send the new state, get the next control
            try:
                sock.sendto(pack_state(time.time() - t0, x, y, yaw, v, 1), cpp_addr)
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                log("[Bridge] C++ command timeout - is MppiCpp.exe alive?")
                break
            except ConnectionResetError:
                # transient on Windows; skip this exchange and resend next loop
                pending_control = (0.0, 0.0, 0)
                continue
            if len(data) != CTRL_SZ:
                continue
            _, ns, na, nr = struct.unpack(CTRL_FMT, data)
            pending_control = (ns, na, nr)

            # pace to ~real time (20 Hz) for smooth viewing.
            remaining = DT - (time.perf_counter() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        log("\n[Bridge] Ctrl+C")
    except Exception:
        log("[Bridge] EXCEPTION in main loop:")
        log(traceback.format_exc())
    finally:
        try:
            sock.close()
        except Exception:
            pass
        if mppi_proc is not None and mppi_proc.poll() is None:
            log("[Bridge] Terminating MPPI exe...")
            mppi_proc.terminate()
            try:
                mppi_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                mppi_proc.kill()
        log(f"[Bridge] Total steps: {step}")
        simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        log("[Bridge] FATAL:")
        log(traceback.format_exc())
        try:
            simulation_app.close()
        except Exception:
            pass
