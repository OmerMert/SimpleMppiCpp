"""
beamng_bridge.py - BeamNG.tech <-> C++ MPPI bridge script

Flow:
  1) Automatically launches the C++ MPPI exe (MppiCpp.exe) in "beamng" mode
     (no need to run it separately by hand)
  2) Opens BeamNG.tech, spawns the vehicle at the start of the reference path
  3) In a loop:
     - Reads the real vehicle state from BeamNG (x, y, yaw, v)
     - Sends the state to C++ via UDP
     - Waits for a control command (steer, accel) from C++
     - Converts it to BeamNG control() format and applies it to the vehicle
     - Advances physics by one tick

Important conventions (identified from hello-world test):
  - In BeamNG, rot_quat=(0,0,0,1) -> vehicle faces the -y direction
  - In MPPI, yaw=0 -> +x direction is forward
  - Therefore we spawn the vehicle facing +x (rotate -90 deg around z axis)
  - Steer sign: positive steer is CCW (left turn) on both sides -> consistent
"""
import math
import os
import sys
import socket
import struct
import subprocess
import time
import csv
import json

from beamngpy import BeamNGpy, Scenario, Vehicle, ProceduralCylinder, ProceduralCube

from scenario import OBSTACLES

# ============ USER SETTINGS ============
BNG_HOME = r"D:\BeamNG.tech.v0.38.5.0"   # folder containing tech.key
BNG_USER = r"D:\BeamNg"                   # user folder (must not contain spaces)

# C++ MPPI executable.
MPPI_EXE = "MppiCpp.exe"

CPP_LISTEN_PORT = 5005
PY_LISTEN_PORT  = 5006

# All per-step run logs go here (one csv per controller/mode), keeping the project root clean.
RUNS_DIR = "runs"

# --- Controller under test (SWAPPABLE for the MPPI speed benchmark) ---
# The harness (this bridge, BeamNG, path, obstacles, metrics) stays identical; only the
# "brain" behind the UDP protocol changes, so the comparison is apples-to-apples.
# Usage:  python beamng_bridge.py [controller] [mode]
#
# controller (arg 1, or MPPI_CONTROLLER env; default "cpp"):
#   cpp    -> our C++/CUDA MPPI (MppiCpp.exe)
#   python -> competitor python_simple_mppi (MizuhoAOKI)
#   jax    -> competitor jax-mppi (jlehtomaa, Williams 2017)
#   torch  -> competitor pytorch_mppi (UM-ARM-Lab, Williams 2017)
CONTROLLER = (sys.argv[1] if len(sys.argv) > 1
              else os.environ.get("MPPI_CONTROLLER", "cpp")).lower()

# mode (arg 2, or MPPI_MODE env; default "step"):
#   step     -> benchmark mode. The sim is paused while the controller thinks, then advanced
#               exactly one tick (a true 20 Hz) per control. This is deterministic and
#               independent of wall-clock speed, so a slow and a fast controller face the
#               same dynamics. Without it BeamNG free-runs during the solve: the car drifts
#               by ~v*t, a slow controller goes unstable, and a fast one ends up controlling
#               at a lower, wall-clock-dependent rate - measured ~5 Hz for C++ against 20 Hz
#               for Python.
#   realtime -> demo mode. No pausing; the simulation runs at real time and the controller
#               has to keep up, needing solve < 50 ms. This shows whether a controller is
#               genuinely real-time capable, but it is not a fair basis for benchmarking.
MODE = (sys.argv[2] if len(sys.argv) > 2
        else os.environ.get("MPPI_MODE", "step")).lower()
if MODE not in ("step", "realtime"):
    print(f"[Bridge] WARNING: unknown mode '{MODE}', falling back to 'step'.")
    MODE = "step"
PAUSE_DURING_SOLVE = (MODE == "step")

# Live red-trajectory drawing costs an extra BeamNG round-trip every few steps. In realtime
# the loop period is what limits the control rate, so drawing is off there by default; in
# step mode it is free (the sim is paused anyway) and useful to watch. Override with
# MPPI_DRAW=1/0.
DRAW_TRAJECTORY = os.environ.get("MPPI_DRAW", "1" if MODE == "step" else "0") == "1"
# Print a breakdown of where each loop iteration's wall-clock time goes (poll / control /
# step / draw / controller). Set MPPI_PROFILE=0 to silence.
PROFILE_LOOP = os.environ.get("MPPI_PROFILE", "1") == "1"

# The pure-Python competitor is ~1000x slower per solve, so it needs a generous
# control-reply timeout. JAX is fast (~12 ms) but its first call JIT-compiles (~0.6 s),
# so give it some slack too. C++ is fast, so 5 s comfortably catches real hangs.
RECV_TIMEOUT = {"python": 60.0, "jax": 30.0, "torch": 30.0}.get(CONTROLLER, 5.0)


with open("config.json", 'r') as f:
            data = json.load(f)

# --- PATH SELECTION ---
PATH_CSV = data["REF_PATH_FILE"]

# Obstacles come from the same source as the C++ CBF and the obstacle costmap
# (scenario.py OBSTACLES, imported above), in the MPPI frame as [x, y, r] circles or
# [x, y, w, h] rectangles. They are spawned physically here so that the MPPI, which
# avoids them via the CBF, and the real car, which collides with them, agree.

# Effective steering angle [rad] at full lock (steering=1.0) for the BeamNG etk800.
MAX_STEER_RAD = data["max_steer_abs"]  # rad 
MAX_ACCEL = data["max_accel_abs"]      # m/s^2

# Safety: stop if the vehicle deviates more than this distance from the path
MAX_PATH_DEVIATION = 20.0   # metres

# Speed ceiling (SAFETY) - clamp the target speed to prevent runaway.
V_TARGET_MAX = 5.0          # m/s

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def load_path_csv(filepath):
    points = []
    with open(filepath, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row["x"]), float(row["y"])))
    return points


def accel_to_throttle_brake(accel_cmd, v, _state=[0.0]):
    """
    Converts the MPPI acceleration command to BeamNG throttle/brake.

    The accel_cmd (m/s^2) produced by MPPI is used to compute a target
    speed for the next step; a PI controller then generates BeamNG
    throttle/brake actuator commands to track that target speed.

    This correctly maps MPPI's longitudinal command to the BeamNG vehicle;
    the earlier approach that bypassed MPPI with a pure PI has been removed.
    """
    dt = 0.05
    # Convert the MPPI acceleration command to a speed target, with a preview horizon so
    # that a brake command actually brakes. Throttle is deliberately gentle and clamped by
    # THROTTLE_CAP: otherwise the etk800 gets near-full throttle from standstill, spins up,
    # and the speed reading goes haywire, leaving the controller blind.
    PREVIEW = 0.25         # s (tunable)
    THROTTLE_CAP = 0.40    # upper throttle limit - prevents launch/wheelspin
    v_meas = max(0.0, v)   # guard against glitchy/negative reading (spin) blinding throttle
    v_target = v_meas + accel_cmd * PREVIEW
    v_target = max(0.0, min(v_target, V_TARGET_MAX))

    error = v_target - v_meas
    _state[0] += error * dt
    _state[0] = max(-1.0, min(_state[0], 1.0))   # anti-windup

    Kp = 0.35
    Ki = 0.10
    u = Kp * error + Ki * _state[0]

    if u >= 0:
        throttle = min(u, THROTTLE_CAP)
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(-u, 1.0)
    return throttle, brake

# Launches the selected controller, either the C++ exe or a Python competitor. They speak
# the same UDP protocol, so the rest of the harness is unaffected by the choice.
def launch_mppi(cpp_listen_port, py_send_port):

    script_dir = os.path.dirname(os.path.abspath(__file__))

    if CONTROLLER in ("python", "jax", "torch"):
        wrapper_name = {"python": "run_python_mppi.py", "jax": "run_jax_mppi.py",
                        "torch": "run_torch_mppi.py"}[CONTROLLER]
        label = {"python": "python_simple_mppi (MizuhoAOKI)",
                 "jax": "jax-mppi (jlehtomaa, Williams 2017 / JAX)",
                 "torch": "pytorch_mppi (UM-ARM-Lab, Williams 2017 / PyTorch)"}[CONTROLLER]
        wrapper = os.path.join(script_dir, "competitors", wrapper_name)
        if not os.path.isfile(wrapper):
            print(f"[Bridge] ERROR: competitor wrapper not found: {wrapper}")
            return None
        cmd = [sys.executable, wrapper, str(cpp_listen_port), str(py_send_port)]
        print(f"[Bridge] Controller = {label} [COMPETITOR]")
    else:
        exe_path = os.path.join(script_dir, MPPI_EXE)
        if not os.path.isfile(exe_path):
            print(f"[Bridge] ERROR: MPPI exe not found: {exe_path}")
            print("[Bridge] Build the C++ project first (e.g. build_and_run.bat).")
            return None
        cmd = [exe_path, str(cpp_listen_port), str(py_send_port), "beamng"]
        print("[Bridge] Controller = C++/CUDA (MppiCpp.exe) [OURS]")

    print(f"[Bridge] Launching: {' '.join(cmd)}")
    # stdout/stderr shared with this terminal; controller logs appear here.
    return subprocess.Popen(cmd, cwd=script_dir)


def full_stop(vehicle, bng, ticks=40):
    """Bring the car to a firm, latched stop and keep it from driving off.

    Two BeamNG automatic-gearbox gotchas this guards against:
      * HOLDING the brake at a standstill makes the realistic automatic shift to
        REVERSE and drive backward forever (the brake input acts as reverse when
        stopped) -> so once stopped we RELEASE the brake.
      * to be certain it cannot drive itself in any direction, we force NEUTRAL
        (gear=0; in neutral no torque reaches the wheels) and latch the PARKING
        BRAKE.  gear: -1 reverse, 0 neutral, 1.. forward.
    """
    for i in range(ticks):
        brake = 1.0 if i < 12 else 0.0   # brief active brake to stop, then release
        vehicle.control(throttle=0.0, brake=brake, steering=0.0,
                        parkingbrake=1.0, gear=0)
        bng.control.step(1, wait=True)
    # latch parked: neutral + parking brake, brake released
    vehicle.control(throttle=0.0, brake=0.0, steering=0.0, parkingbrake=1.0, gear=0)


def main():
    print(f"[Bridge] MODE = {MODE.upper()}  "
          + ("(sim paused during the solve -> true 20 Hz, deterministic, fair benchmark)"
             if MODE == "step" else
             "(sim runs in real time; the controller must keep up - solve < 50 ms. DEMO)"))
    # Log the switches that affect timing results, so it stays clear afterwards which run
    # was taken under which settings (see Log/13 on frame sync).
    print(f"[Bridge] DRAW_TRAJECTORY = {int(DRAW_TRAJECTORY)} | "
          f"PAUSE_DURING_SOLVE = {int(PAUSE_DURING_SOLVE)} | PROFILE = {int(PROFILE_LOOP)}")

    # --- Launch C++ MPPI exe ---
    mppi_proc = launch_mppi(CPP_LISTEN_PORT, PY_LISTEN_PORT)
    if mppi_proc is None:
        return

    # --- UDP ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", PY_LISTEN_PORT))
    sock.settimeout(RECV_TIMEOUT)
    cpp_addr = ("127.0.0.1", CPP_LISTEN_PORT)
    print(f"[Bridge] UDP listening on 127.0.0.1:{PY_LISTEN_PORT}")

    bng = None
    vehicle = None
    step_count = 0
    trajectory_world = []

    try:
        # --- Path ---
        try:
            path_points = load_path_csv(PATH_CSV)
            print(f"[Bridge] Path loaded: {len(path_points)} points")
        except Exception as e:
            print(f"[Bridge] ERROR: path could not be loaded: {e}")
            return

        # --- BeamNG ---
        bng = BeamNGpy("localhost", 25252, home=BNG_HOME, user=BNG_USER)
        bng.open()
        print("[Bridge] BeamNG.tech connected")

        scenario = Scenario("tech_ground", "mppi_run")
        vehicle = Vehicle("ego", model="etk800", license="MPPI")
        # No Electrics sensor: only the built-in 'state' sensor (pos/vel/rotation) is ever
        # read, and attaching Electrics made sensors.poll() issue an extra request per loop
        # for data nobody uses, which is pure latency in the control loop.
        # Spawned at identity; calibration happens below.
        scenario.add_vehicle(vehicle, pos=(0.0, 0.0, 0.5),
                             rot_quat=(0.0, 0.0, 0.0, 1.0))

        # --- Physical obstacles, which must be added before scenario.make ---
        # config OBSTACLES are in the MPPI frame; map to BeamNG world with the
        # nominal spawn transform (spawn at origin, identity quaternion ->
        # pos_rotation=+90deg, so MPPI (x,y) -> world (y, -x)). Circle->cylinder,
        # rectangle->cube. The +90deg rotation swaps the rect's w/h extents.
        OBS_HEIGHT = 2.0
        for oi, obs in enumerate(OBSTACLES):
            ox, oy = obs[0], obs[1]
            wx, wy = oy, -ox                      # MPPI -> BeamNG world
            if len(obs) == 3:                     # circle: [x, y, r]
                scenario.add_procedural_mesh(ProceduralCylinder(
                    pos=(wx, wy, OBS_HEIGHT / 2.0), radius=obs[2],
                    height=OBS_HEIGHT, name=f"obstacle_{oi}"))
            elif len(obs) == 4:                   # rectangle: [x, y, w, h]
                w, h = obs[2], obs[3]
                scenario.add_procedural_mesh(ProceduralCube(
                    pos=(wx, wy, OBS_HEIGHT / 2.0), size=(h, w, OBS_HEIGHT),
                    name=f"obstacle_{oi}"))
        if OBSTACLES:
            print(f"[Bridge] Spawned {len(OBSTACLES)} physical obstacle(s) in BeamNG")

        scenario.make(bng)

        # DETERMINISTIC for the whole startup (load, settle, calibration, teleport) in BOTH
        # modes: those phases drive the sim with bng.control.step(...), which requires the
        # deterministic/paused clock. Realtime only switches over right before the main loop
        # (see "realtime switch" below) - switching here froze the startup sequence.
        bng.settings.set_deterministic(20)  # 20 Hz = MPPI delta_t 0.05
        bng.scenario.load(scenario)
        bng.scenario.start()

        vehicle.control(throttle=0.0, brake=0.0, steering=0.0)
        bng.control.step(10, wait=True)

        # --- Calibration ---
        # There are two separate transforms here, and they are easy to conflate:
        #
        # 1) Position (world x,y -> MPPI x,y). Vehicle forward is +y in world and must be
        #    +x in MPPI, so the world x,y is rotated by -(pi/2 + yaw0).
        #
        # 2) Yaw (world rotation angle -> MPPI yaw). VEHICLE_FORWARD_OFFSET appears in both
        #    the source and the target frame and cancels algebraically, so subtracting
        #    yaw0_world alone is enough.
        #
        # For the etk800, forward is +y at the identity quaternion.
        VEHICLE_FORWARD_OFFSET = -math.pi / 2

        vehicle.sensors.poll()
        s0 = vehicle.sensors["state"]
        x0_world, y0_world, _ = s0["pos"]
        qx, qy, qz, qw = s0["rotation"]
        yaw0_world = quat_to_yaw(qx, qy, qz, qw)
        YAW_OFFSET = -yaw0_world
        pos_rotation = -(VEHICLE_FORWARD_OFFSET + yaw0_world)
        # YAW: only zero out the world rotation
        # POSITION: the vehicle's forward (+y) must align with MPPI's +x
        # World CCW rotation angle: -(VEHICLE_FORWARD_OFFSET + yaw0_world)
        #   = -(-pi/2 + 0) = +pi/2 (i.e., +90 deg CCW)
        # BUT since we want to "map points going in +y to +x", we rotate -90 deg CCW.
        # This is consistent: a +y point rotated by -90 deg becomes +x.

        print(f"[Bridge] Spawn calibration:")
        print(f"         world pos = ({x0_world:+.2f}, {y0_world:+.2f})")
        print(f"         world yaw = {math.degrees(yaw0_world):+.2f} deg")
        print(f"         vehicle forward offset = {math.degrees(VEHICLE_FORWARD_OFFSET):+.1f} deg")
        print(f"         yaw offset = {math.degrees(YAW_OFFSET):+.2f} deg")
        print(f"         pos rotation = {math.degrees(pos_rotation):+.2f} deg")

        def world_to_mppi(wx, wy, wyaw):
            dx = wx - x0_world
            dy = wy - y0_world
            c = math.cos(pos_rotation)
            s = math.sin(pos_rotation)
            mx = c * dx - s * dy
            my = s * dx + c * dy
            #  the BeamNG quaternion yaw runs opposite (CW+) to the MPPI
            myaw = -(wyaw + YAW_OFFSET)
            while myaw > math.pi: myaw -= 2 * math.pi
            while myaw < -math.pi: myaw += 2 * math.pi
            return mx, my, myaw

        def mppi_to_world_point(mx, my, z=0.5):
            c = math.cos(-pos_rotation)
            s = math.sin(-pos_rotation)
            dx = c * mx - s * my
            dy = s * mx + c * my
            return (x0_world + dx, y0_world + dy, z)

        # --- Draw path in BeamNG scene (BLUE) ---
        try:
            path_world = [mppi_to_world_point(mx, my, z=0.3)
                          for (mx, my) in path_points]
            # RGBA: (blue, green, red, alpha) - BeamNG sometimes reads it reversed;
            # docs say RGBA but some examples use BGR; test both if colors look wrong.
            bng.debug.add_polyline(path_world,
                                   rgba_color=(0.1, 0.4, 1.0, 1.0),
                                   cling=True, offset=0.1)
            print(f"[Bridge] Path drawn in BeamNG (blue, {len(path_world)} points)")
        except AttributeError:
            print("[Bridge] WARNING: bng.debug API not available, drawing skipped")
        except Exception as e:
            print(f"[Bridge] Path drawing error: {e} (continuing)")

        print("\n[Bridge] Ready. Running 30-tick calibration test first...")
        print("         (with straight throttle the vehicle should move in +x in MPPI frame)")

        # --- CALIBRATION TEST ---
        # Apply straight throttle for 30 ticks, bypassing MPPI.
        # The vehicle should move in the +x direction in MPPI frame (> 1m forward).
        '''
        for i in range(30):
            vehicle.control(steering=0.0, throttle=0.3, brake=0.0)
            bng.control.step(1, wait=True)

        vehicle.sensors.poll()
        s_test = vehicle.sensors["state"]
        px_t, py_t, _ = s_test["pos"]
        mx_t, my_t, myaw_t = world_to_mppi(px_t, py_t,
                                             quat_to_yaw *s_test["rotation"]))
        print(f"[Bridge] Calibration result: after 30 ticks MPPI=({mx_t:+.2f}, {my_t:+.2f}) "
              f"yaw={math.degrees(myaw_t):+.1f}deg")
        # Check both position and yaw
        pos_ok = (mx_t > 1.0 and abs(my_t) < 2.0)
        yaw_ok = (abs(math.degrees(myaw_t)) < 15.0)  # should be ~0 since going straight
        if pos_ok and yaw_ok:
            print("[Bridge] ✓ Calibration CORRECT (position +x, yaw ~0)")
        else:
            print("[Bridge] ✗ WARNING: Calibration incorrect!")
            print(f"         Position {'✓' if pos_ok else '✗'}: mx={mx_t:.2f} (>1), my={my_t:.2f} (|.|<2)")
            print(f"         Yaw      {'✓' if yaw_ok else '✗'}: {math.degrees(myaw_t):+.1f}deg (|.|<15)")
            print("         Waiting 3 seconds (press Ctrl+C to abort)...")
            time.sleep(3)

        # Brake to stop
        vehicle.control(steering=0.0, throttle=0.0, brake=1.0)
        for _ in range(20):
            bng.control.step(1, wait=True)
'''
        # Teleport the vehicle back to (0,0) in the MPPI frame. The calibration test may
        # have changed its orientation, so the spawn quaternion is re-applied to reset the
        # heading too; reset=True also clears velocity and damage.
        print("[Bridge] Teleporting vehicle to (0,0) + original orientation...")
        vehicle.teleport(pos=(x0_world, y0_world, 0.5),
                         rot_quat=(qx, qy, qz, qw),
                         reset=True)

        # Run a few empty ticks after teleport to let the physics settle
        for _ in range(30):
            vehicle.control(steering=0.0, throttle=0.0, brake=0.0)
            bng.control.step(1, wait=True)

        # Teleport verification: is the vehicle actually at the original position?
        vehicle.sensors.poll()
        s_check = vehicle.sensors["state"]
        px_c, py_c, _ = s_check["pos"]
        qx_c, qy_c, qz_c, qw_c = s_check["rotation"]
        yaw_c_world = quat_to_yaw(qx_c, qy_c, qz_c, qw_c)
        mx_c, my_c, myaw_c = world_to_mppi(px_c, py_c, yaw_c_world)
        print(f"[Bridge] Post-teleport check: MPPI=({mx_c:+.2f},{my_c:+.2f}) "
              f"yaw={math.degrees(myaw_c):+.1f}deg")
        if abs(mx_c) > 0.5 or abs(my_c) > 0.5 or abs(math.degrees(myaw_c)) > 5:
            print("[Bridge] WARNING: Vehicle is not at the zero point after teleport!")
            print("[Bridge] Continuing anyway (MPPI will use the real state)")

        # --- Is the C++ MPPI exe still running? ---
        if mppi_proc.poll() is not None:
            print(f"[Bridge] ERROR: MPPI exe exited unexpectedly "
                  f"(exit={mppi_proc.returncode}).")
            print("[Bridge] Check config.json, data/ovalpath.csv and data/costmap.csv.")
            return

        # --- realtime switch (startup above ran deterministic on purpose) ---
        # Now hand the clock back to the simulator:
        #  * finer step rate: every beamngpy round-trip (poll waits for fresh data, control
        #    waits for its ack) syncs to the sim step rate; 20 sps means ~50 ms EACH and caps
        #    the loop at ~10 Hz. 50 sps makes those waits ~20 ms (same wall-clock speed).
        #  * non-deterministic + resume: step() "assumes the sim is paused", and the settle
        #    loops above leave it paused. The realtime main loop never calls step(), so
        #    without resume() the sim stays frozen and the car never moves.
        if MODE == "realtime":
            bng.settings.set_steps_per_second(50)
            bng.settings.set_nondeterministic()
            bng.control.resume()
            print("[Bridge] Realtime: the sim runs on its own clock (50 sps, resumed)")

        print("[Bridge] MPPI loop starting...\n")

        # State for steer slew-rate limiter (closure)
        _bridge_state = {}

        t_start = time.time()
        last_print_step = 0

        # --- Per-step trajectory log (diagnostics only; no effect on driving) ---
        # Name the log by controller so the benchmark runs don't overwrite each other.
        # realtime is a demo mode -> separate file, so it never clobbers benchmark data.
        # MPPI_LAMBDA (torch competitor only) also gets its own file, so the "identical
        # config" run and the "lambda tuned for its parameterisation" run both survive.
        _lam_tag = ""
        if CONTROLLER == "torch" and os.environ.get("MPPI_LAMBDA"):
            _lam_tag = "_lam" + os.environ["MPPI_LAMBDA"].replace(".", "p")
        os.makedirs(RUNS_DIR, exist_ok=True)
        run_log_name = os.path.join(
            RUNS_DIR, f"run_log_{CONTROLLER}{_lam_tag}.csv" if MODE == "step"
            else f"run_log_{CONTROLLER}{_lam_tag}_realtime.csv")
        run_log = open(run_log_name, "w", newline="")
        # step_ms = how long bng.control.step(1) took in the PREVIOUS iteration, i.e. how
        # long BeamNG needed to advance one tick and present a frame. In step mode the C++
        # solve queues behind that frame on the GPU, so solve_ms tracks step_ms - logging
        # both makes that correlation measurable instead of inferred (see Log/13).
        run_log.write("step,t,solve_ms,mx,my,myaw_deg,v,min_dist,ref_x,ref_y,"
                      "steer_rad,accel_cmd,throttle,brake,z,vz,step_ms\n")

        timeout_count = 0     # consecutive C++ command timeouts
        last_draw_idx = 0     # last trajectory index drawn live in BeamNG
        last_step_ms = 0.0    # previous iteration's bng.control.step() wall time
        left_start = False    # car has driven clear of the start (closed-loop path guard)

        # Where each loop iteration's wall-clock goes (ms, accumulated for the periodic print)
        prof = {"pause": 0.0, "poll": 0.0, "ctrl_wait": 0.0, "apply": 0.0,
                "step": 0.0, "draw": 0.0, "n": 0}

        while True:
            _t_iter = time.perf_counter()
            # Freeze the sim BEFORE reading state, so it stays frozen through the (slow)
            # solve wait below -> the car does not drift while the controller thinks.
            # step(1, wait=True) later advances exactly one tick from this paused state.
            if PAUSE_DURING_SOLVE:
                _t = time.perf_counter()
                bng.control.pause()
                prof["pause"] += (time.perf_counter() - _t) * 1000
            _t = time.perf_counter()
            vehicle.sensors.poll("state")   # only what we use -> one request, less latency
            prof["poll"] += (time.perf_counter() - _t) * 1000
            s = vehicle.sensors["state"]
            px_w, py_w, pz_w = s["pos"]
            vx, vy, vz_w = s["vel"]
            qx, qy, qz, qw = s["rotation"]
            yaw_w = quat_to_yaw(qx, qy, qz, qw)

            mx, my, myaw = world_to_mppi(px_w, py_w, yaw_w)
            # Signed longitudinal velocity (negative = reversing).
            # The velocity has to be rotated into the MPPI frame by pos_rotation, exactly
            # as world_to_mppi does, before projecting onto the MPPI heading. Projecting
            # raw world velocity onto yaw_w instead reads ~0 or negative while the car is
            # moving forward, because the etk800's forward axis is offset from the
            # quaternion yaw by VEHICLE_FORWARD_OFFSET. Under the max(0,v) clamp that fed
            # v=0 to the MPPI and blinded it, since its model ignores steering at v=0.
            vmx = math.cos(pos_rotation) * vx - math.sin(pos_rotation) * vy
            vmy = math.sin(pos_rotation) * vx + math.cos(pos_rotation) * vy
            v = vmx * math.cos(myaw) + vmy * math.sin(myaw)
            if abs(v) < 0.1:
                v = 0.0

            trajectory_world.append((px_w, py_w))

            # Has the end of the path been reached? Count as "done" if within 5m.
            # The oval is CLOSED (start == end), so we must first confirm the car actually
            # drove away: a step-count guard alone falsely reports "completed" when the car
            # never moves (e.g. sim frozen) - it just sits on the end point.
            last_x, last_y = path_points[-1]
            dist_to_end = math.hypot(mx - last_x, my - last_y)
            if dist_to_end > 15.0:
                left_start = True
            if dist_to_end < 5.0 and step_count > 100 and left_start:
                print(f"\n[Bridge] PATH COMPLETED: {dist_to_end:.2f}m from end point "
                      f"({step_count} steps, {time.time()-t_start:.1f}s)")
                # Send "stop" signal to C++ (valid=0)
                stop_bytes = struct.pack("dddddi",
                                          time.time() - t_start,
                                          mx, my, myaw, v, 0)
                for _ in range(3):
                    sock.sendto(stop_bytes, cpp_addr)
                    time.sleep(0.05)
                full_stop(vehicle, bng)   # firm latched stop (parking brake on)
                break

            # Distance to the nearest point on the path. This scans the whole path; an
            # earlier version stopped at the first 600 waypoints and gave wrong results on
            # long straights. At 1501 waypoints the full scan costs ~0.5 ms.
            _nearest = min(path_points,
                           key=lambda p: math.hypot(mx - p[0], my - p[1]))
            ref_x, ref_y = _nearest
            min_dist = math.hypot(mx - ref_x, my - ref_y)

            if min_dist > MAX_PATH_DEVIATION:
                print(f"\n[Bridge] SAFETY STOP: {min_dist:.1f}m deviation from path "
                      f"(limit {MAX_PATH_DEVIATION}m)")
                # Send "stop" signal to C++ (valid=0)
                stop_bytes = struct.pack("dddddi",
                                          time.time() - t_start,
                                          mx, my, myaw, v, 0)
                for _ in range(3):
                    sock.sendto(stop_bytes, cpp_addr)
                    time.sleep(0.05)
                full_stop(vehicle, bng)   # firm latched stop (parking brake on)
                break

            # ========================================================
            # MPPI (C++) CONTROL
            # ========================================================
            # Send state to C++. v must not be negative: the kinematic model does
            # not model reverse motion and a negative v would invert the yaw
            # response (v/L*tan(steer)).
            v_send = max(0.0, v)
            state_bytes = struct.pack("dddddi",
                                       time.time() - t_start,
                                       mx, my, myaw, v_send, 1)
            sock.sendto(state_bytes, cpp_addr)

            # Wait for control command
            _t = time.perf_counter()
            try:
                data, _ = sock.recvfrom(1024)
                prof["ctrl_wait"] += (time.perf_counter() - _t) * 1000
                timeout_count = 0
            except socket.timeout:
                # MPPI connection lost: stop the car instead of leaving it
                # uncontrolled (it would otherwise roll backward indefinitely).
                if mppi_proc.poll() is not None:
                    print("[Bridge] MPPI exe exited -> stopping vehicle.")
                    full_stop(vehicle, bng)
                    break
                timeout_count += 1
                print(f"[Bridge] C++ command timeout ({timeout_count})")
                if timeout_count >= 3:
                    print("[Bridge] No MPPI response -> stopping vehicle.")
                    full_stop(vehicle, bng)
                    break
                continue

            if len(data) != struct.calcsize("ddddi"):
                continue
            # solve_ms = the controller's own MPPI compute time for this cycle (C++ <chrono>
            # / Python perf_counter). Logged into run_log so each MPPI has ONE csv with it.
            ctrl_time, steer_rad, accel_cmd, solve_ms, reset_flag = struct.unpack("ddddi", data)

            if reset_flag:
                break

            # BeamNG etk800: a +steering command turns in the opposite direction
            # to MPPI's +steer convention, hence the negation.
            steer_raw = -steer_rad / MAX_STEER_RAD
            steer_raw = max(-1.0, min(1.0, steer_raw))
            steer_target = steer_raw

            # Symmetric slew-rate limiter. Tightening it to damp the weave (0.10 was tried)
            # adds steering lag and the car stops making corners in time, so it stays loose
            # at 0.30. The weave is better reduced at its source, the MPPI sampling sigma,
            # than by a hard external rate cap.
            if 'prev_steer' not in _bridge_state:
                _bridge_state['prev_steer'] = 0.0
            prev = _bridge_state['prev_steer']
            max_delta = 0.30
            delta = steer_target - prev
            delta = max(-max_delta, min(delta, max_delta))
            steer_norm = prev + delta
            _bridge_state['prev_steer'] = steer_norm

            # Apply MPPI's accel command to BeamNG (no bypass)
            throttle, brake = accel_to_throttle_brake(accel_cmd, v)
            debug_extra = f"u=(s{steer_rad:+.2f},a{accel_cmd:+.2f})"

            _t = time.perf_counter()
            vehicle.control(steering=steer_norm, throttle=throttle, brake=brake)
            prof["apply"] += (time.perf_counter() - _t) * 1000

            run_log.write(f"{step_count},{time.time()-t_start:.3f},{solve_ms:.4f},"
                          f"{mx:.3f},{my:.3f},"
                          f"{math.degrees(myaw):.2f},{v:.3f},{min_dist:.3f},"
                          f"{ref_x:.3f},{ref_y:.3f},{steer_rad:.4f},{accel_cmd:.4f},"
                          f"{throttle:.3f},{brake:.3f},{pz_w:.3f},{vz_w:.3f},"
                          f"{last_step_ms:.3f}\n")
            run_log.flush()

            # Only DRIVE the sim clock in deterministic (step) mode. In realtime the sim runs
            # on its own clock, so calling step(1, wait=True) here would just burn ~88 ms/iter
            # waiting out a tick and throttle the control rate to ~6.5 Hz.
            if MODE == "step":
                _t = time.perf_counter()
                bng.control.step(1, wait=True)
                last_step_ms = (time.perf_counter() - _t) * 1000
                prof["step"] += last_step_ms
            step_count += 1

            # RED trajectory drawn live in BeamNG (extra round-trip; off in realtime)
            if DRAW_TRAJECTORY and len(trajectory_world) - last_draw_idx >= 3:
                _t = time.perf_counter()
                try:
                    seg = [(x, y, 0.3) for (x, y) in trajectory_world[last_draw_idx:]]
                    bng.debug.add_polyline(seg, rgba_color=(1.0, 0.1, 0.1, 1.0),
                                           cling=True, offset=0.2)
                    last_draw_idx = len(trajectory_world) - 1   # overlap 1 to connect
                except Exception:
                    pass
                prof["draw"] += (time.perf_counter() - _t) * 1000

            prof["n"] += 1
            prof["iter"] = prof.get("iter", 0.0) + (time.perf_counter() - _t_iter) * 1000
            if PROFILE_LOOP and prof["n"] >= 20:
                n = prof["n"]
                print(f"[PROFILE] loop {prof['iter']/n:6.1f} ms/step = "
                      f"pause {prof['pause']/n:5.1f} + poll {prof['poll']/n:5.1f} + "
                      f"kontrol-bekle {prof['ctrl_wait']/n:6.1f} + uygula {prof['apply']/n:5.1f} + "
                      f"step {prof['step']/n:5.1f} + ciz {prof['draw']/n:5.1f}")
                for k in prof:
                    prof[k] = 0.0
                prof["n"] = 0

            if step_count - last_print_step >= 10:
                dt = time.time() - t_start
                print(f"[{step_count:4d}] t={dt:5.2f}s "
                      f"mppi=({mx:+6.2f},{my:+6.2f}) "
                      f"yaw={math.degrees(myaw):+5.1f} "
                      f"v={v:4.1f} d={min_dist:4.1f}m | "
                      f"{debug_extra} "
                      f"->t{throttle:.2f}b{brake:.2f}")
                last_print_step = step_count
    except KeyboardInterrupt:
        print("\n[Bridge] Ctrl+C")
    except Exception as e:
        print(f"[Bridge] ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            run_log.close()
            print(f"[Bridge] {run_log_name} written")
        except Exception:
            pass
        # Park the car, which also covers Ctrl+C and exception exits: neutral plus parking
        # brake, with the brake released. Holding the brake here would make the automatic
        # shift into reverse at a standstill and drive backwards.
        try:
            if vehicle is not None:
                vehicle.control(throttle=0.0, brake=0.0, steering=0.0,
                                parkingbrake=1.0, gear=0)
        except Exception:
            pass


        sock.close()
        if bng is not None:
            try:
                bng.close()
            except Exception:
                pass

        # Terminate C++ MPPI exe (if it hasn't exited on its own)
        if mppi_proc.poll() is None:
            print("[Bridge] Terminating MPPI exe...")
            mppi_proc.terminate()
            try:
                mppi_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                mppi_proc.kill()

        print(f"[Bridge] Total steps: {step_count}")


if __name__ == "__main__":
    main()
