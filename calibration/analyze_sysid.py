"""
analyze_sysid.py - Re-analyze BeamNG etk800 sysid data to ground the MPPI
kinematic model in the REAL vehicle (BeamNG = reference).

Reads sysid_measurements.csv (v, steer_norm, yaw_rate) and answers:
  1) Reproduce the original fit (L free) -> does L rail to the grid floor 1.5?
  2) Identifiability: is only the gain max_delta/L observable in the linear region?
  3) Fit with L PINNED to the physical wheelbase (2.82 m from jbeam).
  4) Composite deployment check: with the CURRENT bridge (MAX_STEER_RAD=0.3,
     L_model=1.5), how far off is MPPI's internal yaw-rate model from reality?
  5) Derive the corrected bridge scale so the model matches BeamNG 1:1.
"""
import numpy as np

PHYS_L = 2.82          # physical etk800 wheelbase from jbeam nodes [m]
CUR_L_MODEL = 1.5      # config.json wheel_base
CUR_MAX_STEER_RAD = 0.3   # beamng_bridge.py rad->norm divisor
CUR_MAX_STEER_ABS = 0.3   # config.json max_steer_abs (MPPI command cap)

d = np.loadtxt("sysid_measurements.csv", delimiter=",", skiprows=1)
v, sn, yr = d[:, 0], d[:, 1], d[:, 2]


def fit_L_md(L_grid, md_grid):
    best = (np.inf, None, None)
    for L in L_grid:
        for md in md_grid:
            pred = (v / L) * np.tan(sn * md)
            e = np.sum((pred - yr) ** 2)
            if e < best[0]:
                best = (e, L, md)
    _, L, md = best
    pred = (v / L) * np.tan(sn * md)
    rms = np.sqrt(np.mean((yr - pred) ** 2))
    rel = rms / np.mean(np.abs(yr))
    return L, md, rms, rel


print("=" * 70)
print("BeamNG etk800 sysid re-analysis  (BeamNG = reference)")
print("=" * 70)
print(f"data points: {len(v)}   speeds: {sorted(set(np.round(v)))}")

# ---- 1) Original fit, L free (same grid as beamng_sysid.py) ----
L, md, rms, rel = fit_L_md(np.linspace(1.5, 4.5, 61), np.linspace(0.2, 0.9, 71))
print("\n[1] Original fit  (L free in [1.5, 4.5]):")
print(f"    L_eff = {L:.3f} m   max_delta_eff = {md:.3f} rad ({np.degrees(md):.1f} deg)")
print(f"    RMS = {rms:.3f} rad/s   rel = {rel*100:.1f}%")
print(f"    --> L railed to grid FLOOR (1.5)? {'YES' if abs(L-1.5)<1e-6 else 'no'}")

# widen grid downward to see where L really wants to go
L2, md2, rms2, rel2 = fit_L_md(np.linspace(0.5, 4.5, 81), np.linspace(0.2, 1.6, 141))
print(f"    With wider grid L in [0.5,4.5]: L*={L2:.3f}  md*={md2:.3f}  rel={rel2*100:.1f}%")

# ---- 2) Identifiability: linear-region gain G = yaw/(v*sn) ----
mask = np.abs(sn) <= 0.5     # linear region
G = np.mean(yr[mask] / (v[mask] * sn[mask]))
Gstd = np.std(yr[mask] / (v[mask] * sn[mask]))
print("\n[2] Linear-region steering gain  G = yaw_rate / (v * steer_norm):")
print(f"    G = {G:.4f} +/- {Gstd:.4f}  (per (m/s . steer_norm))")
print(f"    Only the RATIO max_delta/L is identifiable here.")
print(f"    current (1.5, 0.86): md/L = {0.86/1.5:.3f}   |  G = {G:.3f}")
print(f"    => design constraint:  MAX_STEER_RAD = G * L_model")

# ---- 3) Fit with L pinned to physical wheelbase ----
md_grid = np.linspace(0.2, 1.6, 281)
errs = [np.sum(((v/PHYS_L)*np.tan(sn*m) - yr)**2) for m in md_grid]
md_phys = md_grid[int(np.argmin(errs))]
pred = (v/PHYS_L)*np.tan(sn*md_phys)
rms_p = np.sqrt(np.mean((yr-pred)**2)); rel_p = rms_p/np.mean(np.abs(yr))
print(f"\n[3] Fit with L PINNED = {PHYS_L} m (physical):")
print(f"    max_delta_eff = {md_phys:.3f} rad ({np.degrees(md_phys):.1f} deg)")
print(f"    RMS = {rms_p:.3f} rad/s   rel = {rel_p*100:.1f}%")

# ---- 4) Composite deployment mismatch with CURRENT setup ----
# MPPI commands steer_rad; bridge -> steer_norm = steer_rad / MAX_STEER_RAD.
# MPPI model believes yaw = (v/L_model)*tan(steer_rad).
# Reality = measured BeamNG yaw at that steer_norm.
print("\n[4] CURRENT deployment mismatch (L_model=1.5, MAX_STEER_RAD=0.3):")
print("    steer_rad | steer_norm | model yaw/v | REAL yaw/v (fit) | real/model")
for steer_rad in [0.05, 0.10, 0.20, 0.30]:
    snc = steer_rad / CUR_MAX_STEER_RAD
    model_over_v = (1/CUR_L_MODEL) * np.tan(steer_rad)
    real_over_v = G * snc                      # linear approx from measured gain
    ratio = real_over_v / model_over_v
    print(f"      {steer_rad:.2f}    |   {snc:.3f}    |   {model_over_v:.3f}     "
          f"|     {real_over_v:.3f}      |   {ratio:.2f}x")

# ---- 5) Corrected bridge scale ----
print("\n[5] CORRECTED design (model matches BeamNG 1:1 in linear region):")
for Lm in [CUR_L_MODEL, PHYS_L]:
    msr = G * Lm
    print(f"    if L_model = {Lm:.2f} m  ->  MAX_STEER_RAD = G*L = {msr:.3f}")
print(f"    (current MAX_STEER_RAD=0.3 is ~{G*CUR_L_MODEL/0.3:.1f}x too small -> "
      f"car oversteers vs model)")
