"""
plot_calibration.py - Thesis figure: BeamNG etk800 vs MPPI kinematic model.

Left : measured BeamNG yaw-rate vs the calibrated kinematic-bicycle model
       (physical L = 2.82 m, gain G from sysid).
Right : deployment consistency - real BeamNG yaw response to an MPPI command,
        OLD bridge (MAX_STEER_RAD=0.3, ~3.1x oversteer) vs NEW (G*L, 1:1).
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

cfg = json.load(open("config.json"))
L = cfg["wheel_base"]
cap = cfg["max_steer_abs"]
G = cfg["beamng_calibration"]["yaw_rate_gain_G"]
MAX_STEER_RAD_NEW = G * L
MAX_STEER_RAD_OLD = 0.3

d = np.loadtxt("sysid_measurements.csv", delimiter=",", skiprows=1)
v, sn, yr = d[:, 0], d[:, 1], d[:, 2]
md = float(np.arctan(G * L))   # derived full-lock angle for the model curve

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

# ---- Left: data vs model, per speed ----
for vt, col in zip([3.0, 5.0, 7.0], ["#1f77b4", "#ff7f0e", "#2ca532"]):
    m = np.abs(v - vt) < 0.6
    ax1.scatter(sn[m], yr[m], color=col, s=45, zorder=3,
                label=f"BeamNG  v≈{vt:.0f} m/s")
    sx = np.linspace(-1, 1, 200)
    ax1.plot(sx, (vt / L) * np.tan(sx * md), color=col, lw=1.6, alpha=0.8)
ax1.axvspan(-cap / MAX_STEER_RAD_NEW, cap / MAX_STEER_RAD_NEW,
            color="gray", alpha=0.12, label="MPPI operating range")
ax1.set_xlabel("steer_norm  (BeamNG input)")
ax1.set_ylabel("yaw rate [rad/s]")
ax1.set_title(f"Steady-state yaw: BeamNG (dots) vs model (lines)\n"
              f"L={L} m (physical), G={G}")
ax1.grid(alpha=0.3); ax1.legend(fontsize=8)

# ---- Right: deployment mismatch ratio = real_yaw / model_yaw, per command ----
# Each config is self-consistent: OLD (L=1.5, scale 0.3), NEW (L=2.82, scale G*L).
L_OLD = 1.5
sr = np.linspace(0.01, cap, 100)
ratio_old = (G * (sr / MAX_STEER_RAD_OLD)) / ((1 / L_OLD) * np.tan(sr))
ratio_new = (G * (sr / MAX_STEER_RAD_NEW)) / ((1 / L) * np.tan(sr))
ax2.axhline(1.0, color="k", lw=1, ls=":")
ax2.plot(sr, ratio_old, "r--", lw=2.2,
         label=f"OLD  (L=1.5, MAX_STEER_RAD=0.3)  →  ~{ratio_old.mean():.1f}x")
ax2.plot(sr, ratio_new, "g-", lw=2.2,
         label=f"NEW  (L=2.82, MAX_STEER_RAD=G·L)  →  ~{ratio_new.mean():.2f}x")
ax2.set_xlabel("MPPI command  steer_rad [rad]")
ax2.set_ylabel("real BeamNG yaw / model yaw")
ax2.set_title("Deployment mismatch\n(1.0 = car turns exactly as MPPI plans)")
ax2.set_ylim(0, 3.6)
ax2.grid(alpha=0.3); ax2.legend(fontsize=8)

fig.tight_layout()
fig.savefig("calibration_check.png", dpi=130)
print("saved calibration_check.png")
print(f"OLD mismatch (L=1.5, scale 0.3): {ratio_old.mean():.2f}x")
print(f"NEW mismatch (L=2.82, scale G·L): {ratio_new.mean():.2f}x")
