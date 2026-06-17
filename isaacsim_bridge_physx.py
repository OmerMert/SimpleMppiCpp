"""
isaacsim_bridge_physx.py - Isaac Sim <-> C++ MPPI bridge (Phase 2: PhysX vehicle)

Phase 2 counterpart of isaacsim_bridge.py. Instead of integrating the kinematic
model in Python, we drive a REAL PhysX vehicle -- NVIDIA's Leatherback Ackermann
RC car (from the Isaac asset server) -- with full rigid-body dynamics: contact
friction, suspension, momentum, lateral tire slip. This is the sim-to-real
relevant setup.

The C++ side is UNCHANGED: launched in "isaac" mode (external-sim-over-UDP).
The bridge: reads the real chassis pose/velocity from PhysX, sends it to the
MPPI, receives (steer, accel), and maps them to the car via the Ackermann
controller (steer -> front-wheel angles, accel -> target speed -> wheel spin).

Frame: we build the scene directly in MPPI coordinates (path is in MPPI frame,
car spawned at path[0] facing +x). The Leatherback's forward axis is +x and a
positive steer turns left (+yaw) -- both match the MPPI convention -- so NO
world<->MPPI transform is needed (verified empirically).

UDP protocol (see UDP.h): identical to Phase 1.
  Python -> C++  StatePacket   "dddddi" = (time, x, y, yaw, v, valid)  44B
  C++ -> Python  ControlPacket "dddi"   = (time, steer, accel, reset)  28B

Run with Isaac Sim's bundled Python:
  set PYTHONUNBUFFERED=1
  D:\isaacsim\_build\windows-x86_64\release\python.bat isaacsim_bridge_physx.py
Options:
  --headless          run without the GUI window
  <path.csv>          override REF_PATH_FILE from config.json (relative to repo)

NOTE: requires internet on first run (downloads the Leatherback from the public
Isaac S3 asset bucket). config.json wheel_base should match the car (~0.322 m).
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
    print(msg, flush=True)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def repo(rel):
    return os.path.join(SCRIPT_DIR, rel)


with open(repo("config.json"), "r") as _f:
    CFG = json.load(_f)

DT         = float(CFG["delta_t"])          # MPPI control period (0.05 s -> 20 Hz)
WHEEL_BASE = float(CFG["wheel_base"])        # used by MPPI; should match the car
MAX_STEER  = float(CFG["max_steer_abs"])     # [rad]
MAX_ACCEL  = float(CFG["max_accel_abs"])     # [m/s^2]

HEADLESS = "--headless" in sys.argv
_pos = [a for a in sys.argv[1:] if not a.startswith("--")]
REF_PATH_FILE = _pos[0] if _pos else CFG["REF_PATH_FILE"]

# ---- Leatherback specifics (measured via inspection / drive test) ----
ASSET_SUBPATH = "/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd"
STEER_JOINTS = ["Knuckle__Upright__Front_Left", "Knuckle__Upright__Front_Right"]
WHEEL_JOINTS = ["Wheel__Knuckle__Front_Left", "Wheel__Knuckle__Front_Right",
                "Wheel__Upright__Rear_Left", "Wheel__Upright__Rear_Right"]  # FL,FR,BL,BR
CAR_WHEEL_BASE = 0.322     # m (front-rear axle)
CAR_TRACK      = 0.244     # m (left-right)
CAR_WHEEL_R    = 0.052     # m (refined at runtime)
CAR_MAX_STEER  = 0.6       # rad (controller clamp; >= MAX_STEER so MPPI passes through)

# Longitudinal: MPPI accel -> short-horizon target speed -> Ackermann wheel spin.
PREVIEW = 0.3              # s
V_MAX   = 4.0              # m/s safety cap

CPP_LISTEN_PORT = 5005
PY_LISTEN_PORT  = 5006
MPPI_EXE = repo("MppiCpp.exe")

END_RADIUS_M, MIN_END_STEPS = 1.0, 100   # 1 m suits the small RC oval
STATE_FMT, CTRL_FMT = "dddddi", "dddi"
CTRL_SZ = struct.calcsize(CTRL_FMT)

# physics substeps per control tick (physics at 60 Hz, control at 1/DT Hz)
PHYS_HZ = 60.0
SUBSTEPS = max(1, round(DT * PHYS_HZ))


def load_path_csv(filepath):
    pts = []
    with open(filepath, "r") as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"])))
    return pts


PATH = load_path_csv(repo(REF_PATH_FILE))
log(f"[Bridge] Path loaded: {len(PATH)} points from {REF_PATH_FILE}")

# --------------------------------------------------------------------------
from isaacsim import SimulationApp                       # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np                                       # noqa: E402
import omni.usd                                          # noqa: E402
from isaacsim.core.api import World                      # noqa: E402
from isaacsim.core.api.robots import Robot               # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction      # noqa: E402
from isaacsim.storage.native import get_assets_root_path      # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.controllers import AckermannController  # noqa: E402
from pxr import UsdGeom, Gf, UsdLux                      # noqa: E402

try:
    from isaacsim.core.utils.viewports import set_camera_view
except Exception:
    set_camera_view = None
try:
    from isaacsim.util.debug_draw import _debug_draw
    _draw = _debug_draw.acquire_debug_draw_interface()
except Exception:
    _draw = None


def pack_state(t, x, y, yaw, v, valid):
    return struct.pack(STATE_FMT, t, x, y, yaw, max(0.0, v), valid)


def quat_to_yaw(q):
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def build_ref_path_curve(stage, pts):
    curves = UsdGeom.BasisCurves.Define(stage, "/World/ref_path")
    curves.CreateTypeAttr("linear")
    curves.CreateCurveVertexCountsAttr([len(pts)])
    curves.CreatePointsAttr([Gf.Vec3f(float(x), float(y), 0.05) for (x, y) in pts])
    curves.CreateWidthsAttr([0.03] * len(pts))
    curves.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
    curves.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.9, 0.2)])


def add_lighting(stage):
    """Bright dome (ambient fill) + a tilted distant light (sun) so the car is
    clearly visible. The default scene has no adequate light -> very dark."""
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1200.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 10.0, 0.0))


def make_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", PY_LISTEN_PORT))
    return s


def launch_mppi():
    if not os.path.isfile(MPPI_EXE):
        log(f"[Bridge] ERROR: MPPI exe not found: {MPPI_EXE}")
        return None
    cmd = [MPPI_EXE, str(CPP_LISTEN_PORT), str(PY_LISTEN_PORT), "isaac"]
    log(f"[Bridge] Launching MPPI: {' '.join(cmd)}")
    return subprocess.Popen(cmd, cwd=SCRIPT_DIR)


def handshake(sock, cpp_addr, x, y, yaw, v, t0, timeout_s=30.0):
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
                log(f"[Bridge] Handshake OK after {attempts} attempt(s)")
                return (steer, accel, reset)
        except socket.timeout:
            pass
        except ConnectionResetError:
            time.sleep(0.2)
    return None


def main():
    assets_root = get_assets_root_path()
    if not assets_root:
        log("[Bridge] ERROR: Isaac asset root unreachable (need internet for Leatherback).")
        simulation_app.close()
        return
    asset = assets_root + ASSET_SUBPATH
    log(f"[Bridge] Vehicle asset: {asset}")

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / PHYS_HZ, rendering_dt=1.0 / PHYS_HZ)
    world.scene.add_default_ground_plane()

    x0, y0 = PATH[0]
    add_reference_to_stage(usd_path=asset, prim_path="/World/Leatherback")
    car = world.scene.add(Robot(prim_path="/World/Leatherback", name="car",
                                position=np.array([x0, y0, 0.05])))

    stage = omni.usd.get_context().get_stage()
    add_lighting(stage)
    if _draw is None:  # no debug_draw -> fall back to a thin static reference curve
        build_ref_path_curve(stage, PATH)
    world.reset()

    # settle the car on the ground, then refine wheel radius from a front wheel height
    for _ in range(30):
        world.step(render=False)
    global CAR_WHEEL_R
    for prim in stage.Traverse():
        if prim.GetName() == "Wheel_Front_Left":
            z = float(omni.usd.get_world_transform_matrix(prim).ExtractTranslation()[2])
            if z > 0.02:
                CAR_WHEEL_R = z
            break
    log(f"[Bridge] wheel_radius={CAR_WHEEL_R:.4f}  wheel_base(car)={CAR_WHEEL_BASE} "
        f"track={CAR_TRACK}  (MPPI wheel_base={WHEEL_BASE})")

    names = list(car.dof_names)
    steer_idx = np.array([names.index(n) for n in STEER_JOINTS])
    wheel_idx = np.array([names.index(n) for n in WHEEL_JOINTS])

    ack = AckermannController(wheel_base=CAR_WHEEL_BASE, track_width=CAR_TRACK,
                              front_wheel_radius=CAR_WHEEL_R, back_wheel_radius=CAR_WHEEL_R,
                              max_wheel_rotation_angle=CAR_MAX_STEER)

    if set_camera_view is not None:
        xs = [p[0] for p in PATH]; ys = [p[1] for p in PATH]
        cx, cy = 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys))
        span = max(max(xs) - min(xs), max(ys) - min(ys), 20.0)
        try:
            set_camera_view(eye=[cx - 0.2 * span, cy - 0.6 * span, 0.8 * span], target=[cx, cy, 0.0])
        except Exception:
            pass

    sock = make_socket()
    cpp_addr = ("127.0.0.1", CPP_LISTEN_PORT)
    log(f"[Bridge] UDP listening on 127.0.0.1:{PY_LISTEN_PORT}")
    mppi_proc = launch_mppi()
    if mppi_proc is None:
        simulation_app.close(); return

    def read_state():
        pos, quat = car.get_world_pose()
        yaw = quat_to_yaw(quat)
        vel = car.get_linear_velocity()
        v = float(vel[0]) * math.cos(yaw) + float(vel[1]) * math.sin(yaw)  # forward speed
        return float(pos[0]), float(pos[1]), yaw, v

    def apply_control(steer, accel, v_meas):
        v_target = max(0.0, min(v_meas + accel * PREVIEW, V_MAX))
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        pos_cmd, vel_cmd = ack.forward([steer, 0.0, v_target, 0.0, DT])
        if pos_cmd is None:
            return
        car.apply_action(ArticulationAction(joint_positions=np.array(pos_cmd), joint_indices=steer_idx))
        car.apply_action(ArticulationAction(joint_velocities=np.array(vel_cmd), joint_indices=wheel_idx))

    last_x, last_y = PATH[-1]
    trajectory = []
    t0 = time.time()
    step = 0

    x, y, yaw, v = read_state()
    first = handshake(sock, cpp_addr, x, y, yaw, v, t0)
    if first is None:
        log("[Bridge] ERROR: no response from MPPI exe (handshake timeout).")
        if mppi_proc.poll() is None:
            mppi_proc.terminate()
        simulation_app.close(); return
    sock.settimeout(5.0)
    pending = first

    # Precompute the green reference as thin line segments, drawn each frame with
    # the SAME debug_draw method as the red trace -> both look like thin lines
    # (only the colour differs), instead of a fat green ribbon.
    green_a = [(float(px), float(py), 0.04) for (px, py) in PATH[:-1]]
    green_b = [(float(px), float(py), 0.04) for (px, py) in PATH[1:]]
    green_col = [(0.1, 0.9, 0.2, 1.0)] * len(green_a)
    green_w = [2.0] * len(green_a)

    log("[Bridge] MPPI loop starting...")
    try:
        while simulation_app.is_running():
            loop_start = time.perf_counter()
            steer, accel, reset = pending
            if reset:
                log("[Bridge] C++ signalled done/reset (end of path).")
                break

            # read REAL state, apply control, advance physics (substepped)
            x, y, yaw, v = read_state()
            apply_control(steer, accel, v)
            for s in range(SUBSTEPS):
                world.step(render=(s == SUBSTEPS - 1))
            step += 1

            trajectory.append((x, y))
            if _draw is not None:
                _draw.clear_lines()
                _draw.draw_lines(green_a, green_b, green_col, green_w)  # thin green reference
                if len(trajectory) > 1:
                    rpts = [(px, py, 0.1) for (px, py) in trajectory]
                    _draw.draw_lines(rpts[:-1], rpts[1:], [(1.0, 0.1, 0.1, 1.0)] * (len(rpts) - 1),
                                     [3.0] * (len(rpts) - 1))

            if step > MIN_END_STEPS and math.hypot(x - last_x, y - last_y) < END_RADIUS_M:
                log(f"\n[Bridge] PATH COMPLETED: {step} steps, {time.time() - t0:.1f}s")
                for _ in range(3):
                    sock.sendto(pack_state(time.time() - t0, x, y, yaw, v, 0), cpp_addr)
                    time.sleep(0.02)
                break

            if step % 20 == 0:
                log(f"[{step:4d}] t={time.time()-t0:5.1f}s pos=({x:+7.2f},{y:+7.2f}) "
                    f"yaw={math.degrees(yaw):+6.1f} v={v:4.1f} | u=(s{steer:+.3f}, a{accel:+.2f})")

            # send new state, get next control
            try:
                sock.sendto(pack_state(time.time() - t0, x, y, yaw, v, 1), cpp_addr)
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                log("[Bridge] C++ command timeout."); break
            except ConnectionResetError:
                pending = (0.0, 0.0, 0); continue
            if len(data) != CTRL_SZ:
                continue
            _, ns, na, nr = struct.unpack(CTRL_FMT, data)
            pending = (ns, na, nr)

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
