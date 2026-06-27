"""
analyze_run.py - Analyze a BeamNG-MPPI run (run_log.csv vs the reference path).

Answers: does it follow the path? where/how much does it deviate? why?
(correlates cross-track error with path curvature, speed, steering command)
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- load ----
path = np.loadtxt("data/ovalpath.csv", delimiter=",", skiprows=1)  # x,y,yaw,ref_v
px, py, pref_v = path[:, 0], path[:, 1], path[:, 3]

log = np.genfromtxt("run_log.csv", delimiter=",", names=True)
step = log["step"]; t = log["t"]
mx, my = log["mx"], log["my"]
yaw = log["myaw_deg"]; v = log["v"]; dev = log["min_dist"]
steer = log["steer_rad"]; accel = log["accel_cmd"]
thr, brk = log["throttle"], log["brake"]
n = len(step)
print(f"=== BeamNG-MPPI run analysis ===  ({n} steps, {t[-1]:.1f}s)\n")

# ---- path curvature (per waypoint) ----
dx = np.gradient(px); dy = np.gradient(py)
ddx = np.gradient(dx); ddy = np.gradient(dy)
curv = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2 + 1e-9)**1.5  # 1/R

# nearest path index + local curvature/ref_v for each logged pose
near_idx = np.array([np.argmin((px - mx[i])**2 + (py - my[i])**2) for i in range(n)])
loc_curv = curv[near_idx]
loc_refv = pref_v[near_idx]
loc_R = 1.0 / (loc_curv + 1e-9)

# ---- deviation stats ----
print("Cross-track error (distance to nearest path point):")
print(f"  mean {dev.mean():.2f} m | median {np.median(dev):.2f} | "
      f"p95 {np.percentile(dev,95):.2f} | max {dev.max():.2f} m")
prog = np.hypot(mx[-1]-mx[0], my[-1]-my[0])
laps_done = near_idx[-1] / len(px)
print(f"  final pos MPPI=({mx[-1]:.1f},{my[-1]:.1f})  nearest wp {near_idx[-1]}/{len(px)} "
      f"(~{laps_done*100:.0f}% of path)")
print(f"  speed: mean {v.mean():.2f} m/s (ref ~{loc_refv.mean():.2f}) | max {v.max():.2f}")
print(f"  steer_rad: mean|.| {np.abs(steer).mean():.3f} | max|.| {np.abs(steer).max():.3f} "
      f"(cap 0.30) | accel mean {accel.mean():.2f}")

# straight vs corner split
straight = loc_R > 25      # gentle
corner = loc_R <= 25
if corner.any() and straight.any():
    print(f"\nDeviation by section:")
    print(f"  straight (R>25m): mean {dev[straight].mean():.2f} m  ({straight.sum()} steps)")
    print(f"  corner  (R<=25m): mean {dev[corner].mean():.2f} m  ({corner.sum()} steps)")
    # is steering saturating in corners?
    sat = (np.abs(steer) > 0.28).mean() * 100
    print(f"  steering near cap (>0.28) {sat:.0f}% of the time"
          + ("  <- SATURATING" if sat > 5 else ""))

# correlation hints
if n > 10:
    c_cv = np.corrcoef(loc_curv, dev)[0, 1]
    c_v = np.corrcoef(v, dev)[0, 1]
    print(f"\nCorrelation with deviation:  curvature {c_cv:+.2f} | speed {c_v:+.2f}")

# ---- plots ----
fig, ax = plt.subplots(2, 2, figsize=(14, 9))
a = ax[0, 0]
a.plot(px, py, "b-", lw=1, label="reference path")
sc = a.scatter(mx, my, c=dev, cmap="inferno", s=10, label="vehicle (color=dev)")
a.scatter([mx[0]], [my[0]], c="g", s=80, marker="o", label="start", zorder=5)
a.scatter([mx[-1]], [my[-1]], c="r", s=80, marker="x", label="end", zorder=5)
plt.colorbar(sc, ax=a, label="cross-track err [m]")
a.set_aspect("equal"); a.set_title("Trajectory vs path"); a.legend(fontsize=8); a.grid(alpha=0.3)

a = ax[0, 1]
a.plot(t, dev, "r-", lw=1)
a.axhline(dev.mean(), color="k", ls=":", label=f"mean {dev.mean():.2f} m")
a.set_xlabel("t [s]"); a.set_ylabel("cross-track err [m]")
a.set_title("Deviation over time"); a.legend(fontsize=8); a.grid(alpha=0.3)

a = ax[1, 0]
a.plot(t, v, "g-", lw=1, label="actual v")
a.plot(t, loc_refv, "k--", lw=1, label="ref v")
a.set_xlabel("t [s]"); a.set_ylabel("speed [m/s]")
a.set_title("Speed tracking"); a.legend(fontsize=8); a.grid(alpha=0.3)

a = ax[1, 1]
a.plot(t, steer, "b-", lw=1, label="steer_rad")
a.axhline(0.30, color="r", ls=":"); a.axhline(-0.30, color="r", ls=":", label="cap")
a.plot(t, loc_curv * 5, "m-", lw=0.8, alpha=0.6, label="path curv x5")
a.set_xlabel("t [s]"); a.set_ylabel("steer [rad]")
a.set_title("Steering command vs path curvature"); a.legend(fontsize=8); a.grid(alpha=0.3)

fig.tight_layout()
fig.savefig("run_analysis.png", dpi=120)
print("\nsaved run_analysis.png")
