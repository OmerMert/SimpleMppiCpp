"""terrain_preview.py - Offline check of the heightmap the bridge will import.

Rebuilds exactly what import_roughness_terrain() sends to BeamNG, using the NOMINAL
spawn transform (spawn at the origin with the identity quaternion), and draws it with
the reference path and the obstacles on top. Run this before burning a BeamNG run:
if the bumps do not sit under the slalom here, they will not sit there in the sim.

    python terrain_preview.py
"""
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from scenario import OBSTACLES, ROUGHNESS
from roughness import load_roughness_zones, terrain_height
from obstacles import load_obstacle_circles

PROJ = os.path.dirname(os.path.abspath(__file__))
cfg = json.load(open(os.path.join(PROJ, "config.json")))
TER = cfg.get("TERRAIN", {})

size = int(TER.get("size", 256))
scale = float(TER.get("resolution", 1.0))
zclamp = float(TER.get("z_max", 1.0))
N = size - 1


def _bounding_square(s):
    t = 64
    for _ in range(20):
        t *= 2
        if t > s:
            return t
    return None


# Nominal spawn transform, identical to the bridge's world_to_mppi at spawn:
# yaw0_world = 0 and VEHICLE_FORWARD_OFFSET = -pi/2  ->  pos_rotation = +pi/2
POS_ROT = math.pi / 2.0


def world_to_mppi(wx, wy, wyaw=0.0):
    c, s = math.cos(POS_ROT), math.sin(POS_ROT)
    return c * wx - s * wy, s * wx + c * wy, wyaw


zones = load_roughness_zones(ROUGHNESS)
if not zones:
    raise SystemExit("scenario.ROUGHNESS is empty - nothing to preview.")

bmp = _bounding_square(N)
half = 0.5 * bmp * scale

# Build the grid the same way the bridge does, then map each cell back to MPPI
# coordinates for plotting.
H = np.zeros((N, N))
MX = np.zeros((N, N))
MY = np.zeros((N, N))
for i in range(N):
    wx = -half + i * scale
    for j in range(N):
        wy = -half + j * scale
        mx, my, _ = world_to_mppi(wx, wy)
        MX[i, j], MY[i, j] = mx, my
        H[i, j] = max(-zclamp, min(zclamp, terrain_height(mx, my, zones)))

print(f"grid {N}x{N} @ {scale} m  |  extent +-{half:.0f} m (world)")
print(f"relief: min {H.min():+.3f}  max {H.max():+.3f}  peak-to-trough {H.max()-H.min():.3f} m")
nz = np.abs(H) > 1e-6
print(f"rough cells: {nz.sum()} / {H.size} ({100*nz.sum()/H.size:.1f}%)")
if nz.any():
    print(f"rough area in MPPI frame: x [{MX[nz].min():+.1f}, {MX[nz].max():+.1f}] "
          f"y [{MY[nz].min():+.1f}, {MY[nz].max():+.1f}]")

path = np.loadtxt(os.path.join(PROJ, cfg["REF_PATH_FILE"]), delimiter=",", skiprows=1)
circles = np.array(load_obstacle_circles(OBSTACLES), dtype=float)

fig, ax = plt.subplots(figsize=(7.5, 5.5))
lim = max(abs(MX[nz]).max(), abs(MY[nz]).max()) + 12 if nz.any() else 40
m = ax.pcolormesh(MX, MY, H, cmap="terrain", shading="auto",
                  vmin=-abs(H).max(), vmax=abs(H).max())
fig.colorbar(m, ax=ax, label="terrain height [m]")
ax.plot(path[:, 0], path[:, 1], "k--", lw=1.2, label="reference path")
for cx, cy, r in circles:
    ax.add_patch(patches.Circle((cx, cy), r, fc="0.3", ec="k", lw=0.6, zorder=5))
for (zx, zy, R, amp, wl) in zones:
    ax.add_patch(patches.Circle((zx, zy), R, fc="none", ec="r", ls=":", lw=1.2, zorder=6))
ax.set_aspect("equal")
ax.set_xlim(-lim, lim)
ax.set_ylim(min(-10, MY[nz].min() - 10) if nz.any() else -10,
            max(45, MY[nz].max() + 10) if nz.any() else 45)
ax.set_xlabel("MPPI x [m]")
ax.set_ylabel("MPPI y [m]")
ax.set_title("Heightmap the bridge will import (red dotted = roughness zones)")
ax.legend(loc="upper right", fontsize=8)
fig.tight_layout()
out = os.path.join(PROJ, "data", "terrain_preview.png")
fig.savefig(out, dpi=130)
print(f"[OK] {out}")
