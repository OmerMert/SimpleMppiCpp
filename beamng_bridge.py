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
import socket
import struct
import subprocess
import time
import csv
import json

from beamngpy import BeamNGpy, Scenario, Vehicle, ProceduralCylinder, ProceduralCube
from beamngpy.sensors import Electrics

from scenario import OBSTACLES

# ============ USER SETTINGS ============
BNG_HOME = r"D:\BeamNG.tech.v0.38.5.0"   # folder containing tech.key
BNG_USER = r"D:\BeamNg"                   # user folder (must not contain spaces)

# C++ MPPI executable.
MPPI_EXE = "MppiCpp.exe"

CPP_LISTEN_PORT = 5005
PY_LISTEN_PORT  = 5006


with open("config.json", 'r') as f:
            data = json.load(f)

# --- PATH SELECTION ---
PATH_CSV = data["REF_PATH_FILE"]

# Obstacles: SAME source the C++ CBF and the obstacle costmap are built from
# (scenario.py OBSTACLES, imported above). MPPI-frame: [x, y, r] circle or
# [x, y, w, h] rectangle. Spawned physically here so MPPI (avoids via CBF) and
# the real car (collides) share one source.

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
    # Convert MPPI acceleration command to a speed target (with preview horizon so that
    # a brake command actually brakes). NOTE: throttle is intentionally kept gentle
    # and clamped by THROTTLE_CAP. Otherwise the etk800 receives a near-full-throttle
    # command from standstill, spins/launches, the speed sensor (v) goes haywire,
    # and the controller goes blind (20-30+ mph runaway). Gentle throttle prevents
    # this initial wheelspin.
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

# Launches the C++ MPPI exe in "beamng" mode
def launch_mppi(cpp_listen_port, py_send_port):

    script_dir = os.path.dirname(os.path.abspath(__file__))
    exe_path = os.path.join(script_dir, MPPI_EXE)
    if not os.path.isfile(exe_path):
        print(f"[Bridge] ERROR: MPPI exe not found: {exe_path}")
        print("[Bridge] Build the C++ project first (e.g. build_and_run.bat).")
        return None

    cmd = [exe_path, str(cpp_listen_port), str(py_send_port), "beamng"]
    print(f"[Bridge] Launching MPPI: {' '.join(cmd)}")
    # stdout/stderr shared with this terminal; C++ logs appear here.
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
    print("[Bridge] Controller: MPPI (C++)")

    # --- Launch C++ MPPI exe ---
    mppi_proc = launch_mppi(CPP_LISTEN_PORT, PY_LISTEN_PORT)
    if mppi_proc is None:
        return

    # --- UDP ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", PY_LISTEN_PORT))
    sock.settimeout(5.0)
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
        vehicle.sensors.attach("electrics", Electrics())
        # IDENTITY spawn - calibration is performed below
        scenario.add_vehicle(vehicle, pos=(0.0, 0.0, 0.5),
                             rot_quat=(0.0, 0.0, 0.0, 1.0))

        # --- Physical obstacles (must be added BEFORE scenario.make) ---
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

        bng.settings.set_deterministic(20)  # 20 Hz = MPPI delta_t 0.05
        bng.scenario.load(scenario)
        bng.scenario.start()

        vehicle.control(throttle=0.0, brake=0.0, steering=0.0)
        bng.control.step(10, wait=True)

        # --- CALIBRATION ---
        # THERE ARE TWO CRITICAL TRANSFORMS - do not confuse them:
        #
        # 1) POSITION transform (world x,y -> MPPI x,y):
        #    Vehicle forward in world is +y; in MPPI it must be +x.
        #    So we rotate the world x,y by -(pi/2 + yaw0).
        #
        # 2) YAW transform (world rotation angle -> MPPI yaw):
        #    Algebraically, VEHICLE_FORWARD_OFFSET appears in both
        #    source and target frames simultaneously and cancels out.
        #    Subtracting only yaw0_world is sufficient.
        #
        # For etk800: forward = +y in the identity quaternion
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
        # Teleport the vehicle back to (0,0) in MPPI frame.
        # NOTE: the calibration test may have changed the vehicle's orientation.
        # Re-apply the spawn quaternion to also reset the heading.
        # reset=True: also clears velocity and damage.
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

        print("[Bridge] MPPI loop starting...\n")

        # State for steer slew-rate limiter (closure)
        _bridge_state = {}

        t_start = time.time()
        last_print_step = 0

        # --- Per-step trajectory log (diagnostics only; no effect on driving) ---
        run_log = open("run_log.csv", "w", newline="")
        run_log.write("step,t,mx,my,myaw_deg,v,min_dist,ref_x,ref_y,"
                      "steer_rad,accel_cmd,throttle,brake,z,vz\n")

        timeout_count = 0     # consecutive C++ command timeouts
        last_draw_idx = 0     # last trajectory index drawn live in BeamNG

        while True:
            vehicle.sensors.poll()
            s = vehicle.sensors["state"]
            px_w, py_w, pz_w = s["pos"]
            vx, vy, vz_w = s["vel"]
            qx, qy, qz, qw = s["rotation"]
            yaw_w = quat_to_yaw(qx, qy, qz, qw)

            mx, my, myaw = world_to_mppi(px_w, py_w, yaw_w)
            # Signed longitudinal velocity (negative = reversing).
            # IMPORTANT: project the velocity in the MPPI frame (rotated by
            # pos_rotation, exactly like world_to_mppi) onto the MPPI heading.
            # Projecting raw world velocity onto yaw_w is WRONG: the etk800's
            # forward axis is offset by VEHICLE_FORWARD_OFFSET from the quaternion
            # yaw, so that formula reads ~0/negative while the car is actually
            # moving forward. After the max(0,v) clamp that sent v=0 to MPPI and
            # blinded it (steering has no effect in its model when v=0).
            vmx = math.cos(pos_rotation) * vx - math.sin(pos_rotation) * vy
            vmy = math.sin(pos_rotation) * vx + math.cos(pos_rotation) * vy
            v = vmx * math.cos(myaw) + vmy * math.sin(myaw)
            if abs(v) < 0.1:
                v = 0.0

            trajectory_world.append((px_w, py_w))

            # Has the end of the path been reached? Count as "done" if within 5m.
            # step_count > 100: minimum progress guard to avoid triggering early
            # at the start point of a closed oval path.
            last_x, last_y = path_points[-1]
            dist_to_end = math.hypot(mx - last_x, my - last_y)
            if dist_to_end < 5.0 and step_count > 100:
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

            # Distance to the nearest point on the path.
            # IMPORTANT: scan the full path (was previously limited to the first 600 wp
            # which gave incorrect results on long straight paths).
            # 1501 wp x 1 calculation ~ 0.5ms, no issue.
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
            try:
                data, _ = sock.recvfrom(1024)
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

            if len(data) != struct.calcsize("dddi"):
                continue
            ctrl_time, steer_rad, accel_cmd, reset_flag = struct.unpack("dddi", data)

            if reset_flag:
                break

            # BeamNG etk800: a +steering command turns in the opposite direction
            # to MPPI's +steer convention, hence the negation.
            steer_raw = -steer_rad / MAX_STEER_RAD
            steer_raw = max(-1.0, min(1.0, steer_raw))
            steer_target = steer_raw

            # Symmetric slew-rate limiter. NOTE: tightening this to damp the weave
            # (tried 0.10) adds steering lag and the car can't corner in time ->
            # keep it loose (0.30). The weave is better reduced at the source (MPPI
            # sampling noise sigma) than by a hard external rate cap.
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

            vehicle.control(steering=steer_norm, throttle=throttle, brake=brake)

            run_log.write(f"{step_count},{time.time()-t_start:.3f},{mx:.3f},{my:.3f},"
                          f"{math.degrees(myaw):.2f},{v:.3f},{min_dist:.3f},"
                          f"{ref_x:.3f},{ref_y:.3f},{steer_rad:.4f},{accel_cmd:.4f},"
                          f"{throttle:.3f},{brake:.3f},{pz_w:.3f},{vz_w:.3f}\n")
            run_log.flush()

            bng.control.step(1, wait=True)
            step_count += 1

            # RED trajectory drawn live in BeamNG
            if len(trajectory_world) - last_draw_idx >= 3:
                try:
                    seg = [(x, y, 0.3) for (x, y) in trajectory_world[last_draw_idx:]]
                    bng.debug.add_polyline(seg, rgba_color=(1.0, 0.1, 0.1, 1.0),
                                           cling=True, offset=0.2)
                    last_draw_idx = len(trajectory_world) - 1   # overlap 1 to connect
                except Exception:
                    pass

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
            print("[Bridge] run_log.csv written")
        except Exception:
            pass
        # Make sure the car is parked (also covers Ctrl+C / exception exits): NEUTRAL
        # + parking brake, brake released. Do NOT hold the brake here - at a standstill
        # that makes the automatic shift to reverse and drive backward.
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
