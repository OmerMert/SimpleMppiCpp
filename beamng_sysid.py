"""
beamng_sysid.py - Kinematic bicycle model calibration for etk800

Purpose: Align the bicycle model parameters used in MPPI's internal rollout
with the real dynamics of the BeamNG vehicle.

Kinematic bicycle equation:
    yaw_rate = (v / L_eff) * tan(delta_eff)
    delta_eff = steer_norm * max_delta_eff

2 unknowns: L_eff (effective wheelbase), max_delta_eff (effective max steer)

Method:
    1) Hold vehicle at constant speed (PI controller)
    2) Drive with different fixed steer inputs
    3) Measure steady-state yaw_rate
    4) Fit L and max_delta via least squares
"""
import math
import time
import json
import numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Electrics

# ============ USER SETTINGS ============
BNG_HOME = r"D:\BeamNG.tech.v0.38.5.0"   # folder containing tech.key
BNG_USER = r"D:\BeamNg"                   # user folder (must not contain spaces)

CONFIG_FILE_OUT = "config_calibrated.json"
ORIGINAL_CONFIG = "config.json"

# Test matrix
TEST_SPEEDS = [3.0, 5.0, 7.0]               # m/s
TEST_STEERS = [-1.0, -0.75, -0.50, -0.25,
                0.25, 0.50, 0.75, 1.0]      # BeamNG normalized
STEADY_STATE_TIME = 5.0   # seconds to hold steady
SETTLE_TIME = 3.0         # seconds to reach target speed first

PHYSICS_HZ = 50
# =============================================


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def drive_constant(bng, vehicle, v_target, steer_norm, duration_s,
                    Kp=0.5, Ki=0.2):
    """Drive vehicle at constant speed + constant steer. Record last 2 seconds of state.

    Returns:
        dict: {'v_mean', 'yaw_rate_mean', 'v_std', 'yaw_rate_std'}
    """
    n_ticks = int(duration_s * PHYSICS_HZ)
    dt = 1.0 / PHYSICS_HZ

    # PI state for speed control
    integral = 0.0

    # Collect last 2 seconds of data (steady-state)
    record_from_tick = n_ticks - int(2.0 * PHYSICS_HZ)
    yaws = []
    times = []
    vs = []

    for tick in range(n_ticks):
        vehicle.sensors.poll()
        s = vehicle.sensors["state"]
        vx, vy, _ = s["vel"]
        v = math.hypot(vx, vy)
        qx, qy, qz, qw = s["rotation"]
        yaw = quat_to_yaw(qx, qy, qz, qw)

        # Speed PI
        error = v_target - v
        integral += error * dt
        integral = max(-2.0, min(integral, 2.0))
        u = Kp * error + Ki * integral

        if u >= 0:
            throttle = min(u, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-u, 1.0)

        vehicle.control(steering=steer_norm, throttle=throttle, brake=brake)
        bng.control.step(1, wait=True)

        if tick >= record_from_tick:
            yaws.append(yaw)
            times.append(tick * dt)
            vs.append(v)

    # Yaw unwrap (in case the -pi/+pi boundary was crossed)
    yaws_unwrapped = np.unwrap(yaws)
    times_arr = np.array(times)

    # Linear fit: yaw(t) = a*t + b, slope = yaw_rate
    if len(yaws_unwrapped) > 2:
        A = np.vstack([times_arr, np.ones(len(times_arr))]).T
        yaw_rate, _ = np.linalg.lstsq(A, yaws_unwrapped, rcond=None)[0]
    else:
        yaw_rate = 0.0

    return {
        'v_mean': np.mean(vs),
        'v_std': np.std(vs),
        'yaw_rate': yaw_rate,
        'yaw_samples': len(yaws),
    }


def fit_bicycle_parameters(measurements):
    """Fit L_eff and max_delta from a list of (v, steer_norm, yaw_rate) measurements.

    Model: yaw_rate = (v / L) * tan(steer_norm * max_delta)

    Two-unknown nonlinear least squares.
    Solved via simple grid search (robust, no gradient needed).
    """
    # Data arrays
    vs = np.array([m['v'] for m in measurements])
    ss = np.array([m['steer_norm'] for m in measurements])
    yrs = np.array([m['yaw_rate'] for m in measurements])

    # Grid search range
    L_candidates = np.linspace(1.5, 4.5, 61)         # L in [1.5, 4.5] m
    max_delta_candidates = np.linspace(0.2, 0.9, 71) # max_delta in [0.2, 0.9] rad

    best_err = float('inf')
    best_L = None
    best_max_delta = None

    for L in L_candidates:
        for max_delta in max_delta_candidates:
            pred = (vs / L) * np.tan(ss * max_delta)
            err = np.sum((pred - yrs) ** 2)
            if err < best_err:
                best_err = err
                best_L = L
                best_max_delta = max_delta

    # Residual analysis
    pred = (vs / best_L) * np.tan(ss * best_max_delta)
    residuals = yrs - pred
    rms_err = np.sqrt(np.mean(residuals ** 2))
    rel_err = rms_err / (np.mean(np.abs(yrs)) + 1e-6)

    return {
        'L_eff': best_L,
        'max_delta_eff': best_max_delta,
        'rms_err': rms_err,
        'rel_err': rel_err,
    }


def main():
    print("=" * 60)
    print("BeamNG etk800 Kinematic Bicycle Model Calibration")
    print("=" * 60)
    print()

    bng = BeamNGpy("localhost", 25252, home=BNG_HOME, user=BNG_USER)
    bng.open()
    print("[SysID] Connected to BeamNG")

    scenario = Scenario("tech_ground", "sysid")
    vehicle = Vehicle("ego", model="etk800", license="CALIB")
    vehicle.sensors.attach("electrics", Electrics())
    scenario.add_vehicle(vehicle, pos=(0, 0, 0.5), rot_quat=(0, 0, 0, 1))
    scenario.make(bng)

    bng.settings.set_deterministic(PHYSICS_HZ)
    bng.scenario.load(scenario)
    bng.scenario.start()

    print("[SysID] Scene loaded, letting engine stabilize...")
    vehicle.control(throttle=0.0, brake=0.0, steering=0.0)
    bng.control.step(20, wait=True)

    measurements = []
    total_tests = len(TEST_SPEEDS) * len(TEST_STEERS)
    test_idx = 0

    for v_target in TEST_SPEEDS:
        for steer_norm in TEST_STEERS:
            test_idx += 1
            print(f"\n[{test_idx}/{total_tests}] v={v_target} m/s, steer={steer_norm:+.2f}")

            # Teleport back to origin (no accumulating drift)
            vehicle.teleport(pos=(0, 0, 0.5),
                             rot_quat=(0, 0, 0, 1),
                             reset=True)
            vehicle.control(throttle=0.0, brake=0.0, steering=0.0)
            bng.control.step(15, wait=True)

            # 1) Reach target speed (steer=0)
            drive_constant(bng, vehicle, v_target, 0.0, SETTLE_TIME)

            # 2) Apply steer and capture steady-state
            result = drive_constant(bng, vehicle, v_target, steer_norm,
                                     STEADY_STATE_TIME)

            print(f"    v_mean={result['v_mean']:.2f} m/s  "
                  f"yaw_rate={result['yaw_rate']:+.3f} rad/s  "
                  f"samples={result['yaw_samples']}")

            measurements.append({
                'v': result['v_mean'],
                'steer_norm': steer_norm,
                'yaw_rate': result['yaw_rate'],
                'v_std': result['v_std'],
            })

    # Stop vehicle
    vehicle.control(throttle=0.0, brake=1.0, steering=0.0)
    bng.control.step(20, wait=True)

    print("\n" + "=" * 60)
    print("Calibration Fit Results")
    print("=" * 60)

    fit = fit_bicycle_parameters(measurements)

    print(f"\nFitted parameters:")
    print(f"  L_eff (effective wheelbase):  {fit['L_eff']:.3f} m")
    print(f"  max_delta_eff (effective max steer): {fit['max_delta_eff']:.4f} rad "
          f"({math.degrees(fit['max_delta_eff']):.2f} deg)")
    print(f"\nFit quality:")
    print(f"  RMS error:      {fit['rms_err']:.4f} rad/s")
    print(f"  Relative error: {fit['rel_err']*100:.1f}%")

    # Predicted vs actual table for each measurement
    print(f"\nMeasurement | v    | steer_norm | yaw_rate_actual | yaw_rate_pred | err")
    print("-" * 75)
    for m in measurements:
        pred = (m['v'] / fit['L_eff']) * math.tan(m['steer_norm'] * fit['max_delta_eff'])
        err = m['yaw_rate'] - pred
        print(f"            | {m['v']:4.1f} | {m['steer_norm']:+.2f}       "
              f"| {m['yaw_rate']:+.3f}           "
              f"| {pred:+.3f}         | {err:+.3f}")

    # Write calibrated config file
    print(f"\n[SysID] Generating calibrated config file...")
    with open(ORIGINAL_CONFIG, "r") as f:
        cfg = json.load(f)

    cfg['wheel_base'] = round(fit['L_eff'], 3)
    cfg['max_steer_abs'] = round(fit['max_delta_eff'], 4)
    cfg['_calibration_info'] = {
        'source': 'beamng_sysid.py',
        'vehicle': 'etk800',
        'num_measurements': len(measurements),
        'rms_error_rad_s': round(fit['rms_err'], 4),
        'relative_error_pct': round(fit['rel_err'] * 100, 2),
    }

    with open(CONFIG_FILE_OUT, "w") as f:
        json.dump(cfg, f, indent=4)

    print(f"[SysID] New config: {CONFIG_FILE_OUT}")
    print(f"[SysID] Changes:")
    print(f"         wheel_base: {cfg['wheel_base']} m")
    print(f"         max_steer_abs: {cfg['max_steer_abs']} rad")

    # Save raw data (for thesis plots)
    np.savetxt("sysid_measurements.csv",
               np.array([[m['v'], m['steer_norm'], m['yaw_rate']]
                          for m in measurements]),
               delimiter=",",
               header="v,steer_norm,yaw_rate",
               comments="")
    print("[SysID] Raw data saved: sysid_measurements.csv")

    bng.close()
    print("\n[SysID] Done.")


if __name__ == "__main__":
    main()
