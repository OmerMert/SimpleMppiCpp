"""
check_ackermann.py - Does the kinematic bicycle (= lumped Ackermann) model hold
for the etk800, or is a richer model needed?

The MPPI model  yaw_rate = (v/L)*tan(delta)  IS the lumped Ackermann kinematic
model (single centre wheel). The real question for fidelity is NOT left/right
Ackermann split (that averages out at the CoG) but whether the *gain* stays
constant across speed. If gain drops with speed -> tyre slip / understeer ->
the kinematic model breaks and you'd need a DYNAMIC bicycle, not Ackermann.
"""
import numpy as np

L = 2.82           # physical wheelbase
TRACK = 1.6        # approx front track width of etk800 [m]

d = np.loadtxt("sysid_measurements.csv", delimiter=",", skiprows=1)
v, sn, yr = d[:, 0], d[:, 1], d[:, 2]
G = yr / (v * sn)   # per-point gain

print("Per-point steering gain  G = yaw_rate / (v * steer_norm)")
print("(kinematic/Ackermann model => G must be CONSTANT across v and steer)\n")
print("steer_norm |   v=3    |   v=5    |   v=7   | spread across speed")
print("-" * 64)
for s in sorted(set(np.abs(np.round(sn, 2)))):
    row = []
    for vt in [3.0, 5.0, 7.0]:
        m = (np.abs(np.abs(sn) - s) < 0.01) & (np.abs(v - vt) < 0.6)
        row.append(np.mean(np.abs(G[m])) if m.any() else np.nan)
    spread = (np.nanmax(row) - np.nanmin(row)) / np.nanmean(row) * 100
    flag = "  <- LINEAR (we operate here)" if s <= 0.25 else (
           "  <- understeer/saturation" if spread > 12 else "")
    print(f"   {s:.2f}    |  {row[0]:.3f}  |  {row[1]:.3f}  |  {row[2]:.3f} "
          f"|  {spread:4.1f}%{flag}")

# Operating point: how big is the left/right Ackermann angle split we are
# "ignoring" by using a single bicycle delta?
print("\nLeft/right Ackermann split at our MAX operating steer (steer_norm~0.17):")
G0 = np.mean(np.abs(G[np.abs(sn) <= 0.25]))
for vt in [3.0]:
    sn_max = 0.17
    yaw = G0 * vt * sn_max
    R = vt / yaw                       # turn radius of the CoG/centre
    di = np.arctan(L / (R - TRACK/2))  # inner wheel angle
    do = np.arctan(L / (R + TRACK/2))  # outer wheel angle
    dc = np.arctan(L / R)              # bicycle (centre) angle
    print(f"  v={vt}: R={R:.1f} m | inner={np.degrees(di):.2f}d  "
          f"centre(bicycle)={np.degrees(dc):.2f}d  outer={np.degrees(do):.2f}d")
    print(f"          inner-outer split = {np.degrees(di-do):.2f} deg "
          f"(bicycle delta sits exactly between -> averaged out)")
