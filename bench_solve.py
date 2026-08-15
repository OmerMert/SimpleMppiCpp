"""Runs the four MPPIs around the same track without BeamNG.

In place of the simulator there is a headless plant driving the same reference path
(data/ovalpath.csv) and the same obstacles: the kinematic bicycle model each controller
already assumes internally. Every controller therefore drives the identical lap and only
the "brain" differs.

The reason for a simulator-free run is that with BeamNG live our solve time swings between
3.68 and 15.46 ms across runs of the same exe and config. The simulator shares both the
CPU (physics threads) and the GPU (rendering) with us, and the CPU-based competitors are
affected far less, so an in-loop measurement systematically penalises whoever shares the
GPU. Without the simulator the number is repeatable and free of that contention.

Each controller is measured on solve time, tracking deviation and obstacle clearance.
Results go to runs/bench_log_<controller>.csv using the same column layout as the BeamNG
runs, so analyze_run.py and compare_mppi.py can read either.

Usage:
  python bench_solve.py                    # all controllers, full lap
  python bench_solve.py cpp jax            # selected controllers
  python bench_solve.py --steps 300 cpp    # cap the number of steps
"""
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "competitors"))
import _harness as H                                    # noqa: E402

PROJ = os.path.dirname(os.path.abspath(__file__))
LISTEN, SEND = 5005, 5006          # controller listens / sends back to us
OLD_CTRL_FMT = "dddi"              # c73e871 and earlier: no solve_ms field
# MPPI_FORCE_RTT=1 makes even a current build report round-trip time instead of its own
# solve_ms, matching how the older builds are measured.
FORCE_RTT = os.environ.get("MPPI_FORCE_RTT", "0") == "1"
# MPPI_GAP_MS inserts an idle wait between solves. In BeamNG's step mode the sim is paused
# during the solve, so nothing feeds the GPU and a ~50 ms gap opens up between solves; this
# flag reproduces that gap here, without BeamNG or rendering.
GAP_MS = float(os.environ.get("MPPI_GAP_MS", "0"))
MAX_STEPS = 2000                   # a full lap is ~1200 steps; this is a safety cap
PY_MAX_STEPS = 120                 # pure Python runs ~4.2 s/step, so a full lap is ~85 min
WARMUP = 1                         # leading steps excluded from the statistics

CONTROLLERS = {
    "cpp":    ("C++/CUDA (ours)",        None,                             5.0),
    "jax":    ("JAX (jax-mppi)",         "competitors/run_jax_mppi.py",   30.0),
    "torch":  ("PyTorch (pytorch_mppi)", "competitors/run_torch_mppi.py", 30.0),
    "python": ("Python (reference)",     "competitors/run_python_mppi.py", 60.0),
}


def free_port():
    """Clear any listener or exe left from a previous run, so packets do not hit a dead socket."""
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    f"Get-NetUDPEndpoint -LocalPort {LISTEN} -EA SilentlyContinue | "
                    "%{ Stop-Process -Id $_.OwningProcess -Force -EA SilentlyContinue }"],
                   capture_output=True)
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Stop-Process -Name MppiCpp -Force -EA SilentlyContinue"],
                   capture_output=True)
    time.sleep(0.6)


# Headless plant: the kinematic bicycle each controller assumes, which is also the plant
# the reference repo uses in its own demos. This is not BeamNG's full vehicle physics; the
# goal here is to run the same task the same way, not to validate vehicle dynamics.
def plant_step(x, y, yaw, v, steer, accel, dt, L, max_steer, max_accel):
    steer = max(-max_steer, min(max_steer, steer))
    accel = max(-max_accel, min(max_accel, accel))
    return (x + v * math.cos(yaw) * dt,
            y + v * math.sin(yaw) * dt,
            yaw + v / L * math.tan(steer) * dt,
            v + accel * dt)


def body_points(x, y, yaw, vw, vl):
    """The footprint's 9 body points, same layout as mppi_core.cu compute_cbf_cost."""
    bx = 0.5 * vl * np.array([-1., -1., -1., 0., 0., 0., 1., 1., 1.])
    by = 0.5 * vw * np.array([-1., 0., 1., 1., -1., 0., 1., 0., -1.])
    c, s = math.cos(yaw), math.sin(yaw)
    return x + bx * c - by * s, y + bx * s + by * c


def run(key, cfg, ref_path, circles, max_steps):
    """Start the controller, run it closed-loop against the headless plant, return metrics."""
    label, wrapper, _ = CONTROLLERS[key]
    free_port()
    if wrapper is None:
        # MPPI_EXE points at a different build, e.g. an older commit's exe, to test claims
        # like "the same algorithm used to be faster".
        exe = os.environ.get("MPPI_EXE") or os.path.join(PROJ, "MppiCpp.exe")
        cmd = [exe, str(LISTEN), str(SEND)]
    else:
        cmd = [sys.executable, "-u", os.path.join(PROJ, wrapper), str(LISTEN), str(SEND)]

    proc = subprocess.Popen(cmd, cwd=PROJ, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    # Drain the controller's output continuously; otherwise the pipe fills and the
    # controller blocks on its next write, hanging silently mid-run.
    ready = threading.Event()
    tail = []

    def _drain():
        for line in proc.stdout:
            tail.append(line.rstrip())
            del tail[:-40]
            if line.startswith("[PHASE]"):      # MPPI_PHASE_PROFILE=1 dump
                print("      " + line.rstrip(), flush=True)
            # Readiness handshake: must stay in sync with the banner _harness.serve prints.
            if "waiting for first state" in line:
                ready.set()
        ready.set()

    threading.Thread(target=_drain, daemon=True).start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", SEND))
    sock.settimeout(CONTROLLERS[key][2] * 4)   # generous for the first reply (JIT, allocation)

    if wrapper is None:
        time.sleep(1.5)                        # let the exe open its socket
    elif not ready.wait(180.0):                # wait for the wrapper to report ready
        print(f"    {label}: FAILED TO START\n      " + "\n      ".join(tail[-8:]))
        sock.close(); proc.terminate(); return None

    dt = float(cfg["delta_t"]); L = float(cfg["wheel_base"])
    max_steer = float(cfg["max_steer_abs"]); max_accel = float(cfg["max_accel_abs"])
    vw, vl = H.footprint(cfg)
    path_xy = ref_path[:, :2]
    n_path = len(ref_path)

    # Start at the path's first point with its reference speed. At v=0 the kinematic model
    # gives yaw_rate = v/L*tan(d) = 0 and ignores steering, so the car must start moving.
    x, y, yaw, v = float(ref_path[0, 0]), float(ref_path[0, 1]), \
                   float(ref_path[0, 2]), float(ref_path[0, 3])

    idx = 0
    rows, solves = [], []
    finished = False
    for step in range(max_steps):
        if GAP_MS:
            time.sleep(GAP_MS / 1000.0)
        t_send = time.perf_counter()
        sock.sendto(struct.pack("dddddi", step * dt, x, y, yaw, v, 1), ("127.0.0.1", LISTEN))
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            print(f"    {label}: no reply (timeout) at step {step}\n      "
                  + "\n      ".join(tail[-6:]))
            break
        rtt_ms = (time.perf_counter() - t_send) * 1000.0
        if step == 0:
            sock.settimeout(CONTROLLERS[key][2])
        if FORCE_RTT and len(data) != struct.calcsize(OLD_CTRL_FMT):
            _, steer, accel, _sm, reset = struct.unpack("ddddi", data)
            solve_ms = rtt_ms          # reduce to the same measure the old builds allow
        elif len(data) == struct.calcsize(OLD_CTRL_FMT):
            # Builds up to c73e871 do not carry solve_ms in ControlPacket, so round-trip
            # time stands in. The localhost UDP overhead is the same for both builds, so
            # the comparison stays fair.
            _, steer, accel, reset = struct.unpack(OLD_CTRL_FMT, data)
            solve_ms = rtt_ms
        else:
            _, steer, accel, solve_ms, reset = struct.unpack("ddddi", data)
        if reset:                              # controller reports end of path
            finished = True
            break

        # metrics: nearest waypoint (forward window), deviation, obstacle clearance
        seg = path_xy[idx:min(n_path, idx + H.SEARCH_FWD)]
        idx += int(np.argmin((seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2))
        dev = float(math.hypot(x - path_xy[idx, 0], y - path_xy[idx, 1]))
        if len(circles):
            gx, gy = body_points(x, y, yaw, vw, vl)
            d = np.hypot(gx[:, None] - circles[None, :, 0],
                         gy[:, None] - circles[None, :, 1]) - circles[None, :, 2]
            min_dist = float(d.min())
        else:
            min_dist = float("nan")

        solves.append(solve_ms)
        # In the bridge the min_dist column means deviation from the path
        # (beamng_bridge.py:548) and the analysis tools read it that way, so obstacle
        # clearance goes at the end as an extra column.
        rows.append((step, step * dt, solve_ms, x, y, math.degrees(yaw), v, dev,
                     path_xy[idx, 0], path_xy[idx, 1], steer, accel, 0.0, 0.0, 0.0, 0.0,
                     min_dist))

        if step % 200 == 0 and step:
            print(f"      [{step:4d}] {100.0 * idx / (n_path - 1):.0f}% of path | "
                  f"solve {np.median(solves):.2f} ms | deviation {dev:.2f} m", flush=True)
        if idx >= n_path - 4:                  # lap complete
            finished = True
            break

        x, y, yaw, v = plant_step(x, y, yaw, v, steer, accel, dt, L, max_steer, max_accel)

    sock.sendto(struct.pack("dddddi", 0, x, y, yaw, v, 0), ("127.0.0.1", LISTEN))
    time.sleep(0.4)
    proc.terminate()
    sock.close()

    if len(rows) <= WARMUP:
        return None
    os.makedirs(os.path.join(PROJ, "runs"), exist_ok=True)
    out = os.path.join(PROJ, "runs", f"bench_log_{key}.csv")
    with open(out, "w", newline="") as f:
        f.write("step,t,solve_ms,mx,my,myaw_deg,v,min_dist,ref_x,ref_y,"
                "steer_rad,accel_cmd,throttle,brake,z,vz,obs_clear\n")
        for r in rows:
            f.write(",".join(f"{c:.6f}" for c in r) + "\n")

    a = np.array(rows[WARMUP:], dtype=float)
    return {"solve": a[:, 2], "dev": a[:, 7],
            "min_dist": a[:, 16], "v": a[:, 6], "steer": a[:, 10],
            "steps": len(rows), "idx": idx, "n_path": n_path,
            "finished": finished, "log": out}


def main():
    args = [a.lower() for a in sys.argv[1:]]
    max_steps = MAX_STEPS
    if "--steps" in args:
        i = args.index("--steps")
        max_steps = int(args[i + 1]); del args[i:i + 2]
    sel = [a for a in args if a in CONTROLLERS] or list(CONTROLLERS)

    cfg, ref_path, circles = H.load_setup()
    print("=" * 78)
    print(f"  SAME TRACK, NO SIMULATOR  |  K={cfg['number_of_samples_K']} "
          f"T={cfg['horizon_step_T']} dt={cfg['delta_t']} | path {len(ref_path)} points | "
          f"{len(circles)} obstacles")
    print("=" * 78)

    res = {}
    for key in sel:
        label = CONTROLLERS[key][0]
        n = min(max_steps, PY_MAX_STEPS) if key == "python" else max_steps
        note = f"  (capped at {n} steps: a full lap would take ~{n_full_min(ref_path):.0f} min)" \
               if key == "python" and n < max_steps else ""
        print(f"\n  running {label}...{note}", flush=True)
        r = run(key, cfg, ref_path, circles, n)
        if r is None:
            print("    no measurement"); continue
        res[key] = r
        s, d = r["solve"], r["dev"]
        pct = 100.0 * r["idx"] / (r["n_path"] - 1)
        print(f"    {r['steps']:4d} steps | {pct:.0f}% of path | "
              f"{'LAP COMPLETE' if r['finished'] else 'unfinished'}")
        print(f"    solve    : median {np.median(s):8.2f} ms | mean {s.mean():8.2f} | "
              f"p95 {np.percentile(s, 95):8.2f} | max {s.max():8.2f}")
        print(f"    deviation: mean {d.mean():.3f} m | p95 {np.percentile(d, 95):.3f} | "
              f"max {d.max():.3f}")
        if np.isfinite(r["min_dist"]).any():
            md = np.nanmin(r["min_dist"])
            print(f"    obstacle : closest clearance {md:+.2f} m "
                  f"{'(body overlaps obstacle)' if md < 0 else ''}")

    if not res:
        return
    print("\n" + "=" * 78)
    print(f"  {'':24} {'solve med':>10} {'ratio':>8} {'dev mean':>10} {'p95':>8} {'lap':>10}")
    print("-" * 78)
    base = np.median(res["cpp"]["solve"]) if "cpp" in res else None
    for key in CONTROLLERS:
        if key not in res:
            continue
        r = res[key]
        m = np.median(r["solve"])
        ratio = f"{m / base:7.1f}x" if base else "      -"
        print(f"  {CONTROLLERS[key][0]:24} {m:9.2f}ms {ratio:>8} "
              f"{r['dev'].mean():9.3f}m {np.percentile(r['dev'], 95):7.3f}m "
              f"{('FULL' if r['finished'] else 'partial'):>10}")
    print("\n  20 Hz budget (%.0f ms):" % (1000 * float(H.load_setup()[0]["delta_t"])))
    for key in CONTROLLERS:
        if key in res:
            m = np.median(res[key]["solve"])
            print(f"    {CONTROLLERS[key][0]:24}: " +
                  (f"UNDER ({50 / m:.0f}x margin)" if m < 50
                   else f"OVER (exceeds by {m / 50:.0f}x)"))
    print(f"\n  Logs: runs/bench_log_<controller>.csv")


def n_full_min(ref_path):
    """Roughly how many minutes a full pure-Python lap would take, at ~4.2 s/step."""
    return len(ref_path) * 4.2 / 60.0


if __name__ == "__main__":
    main()
