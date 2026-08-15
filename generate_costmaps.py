"""Builds the obstacle costmap from scenario.py and exports the raw list the C++ side reads.

Input: scenario.py OBSTACLES. Outputs, matching the *_FILE keys in config.json:
  - data/obstacle_costmap.csv           cost grid, additive to the CBF (see obstacles.py)
  - data/obstacles.json                 raw list for the C++ CBF and the BeamNG spawn,
                                        which need exact circles rather than a grid
  - data/obstacle_costmap_preview.png

Grid bounds come from the reference path plus GRID_MARGIN; main.cpp ReadConfig assumes
the same convention.
"""
import csv
import json
import os

import numpy as np

from obstacles import load_obstacle_circles, obstacle_cost
from scenario import OBSTACLES


def load_cfg(path="config.json"):
    with open(path, "r") as f:
        return json.load(f)


def load_ref_path(filepath):
    xs, ys = [], []
    with open(filepath, "r") as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
    return np.array(xs), np.array(ys)


def grid_bounds(resolution, margin, ref_xs, ref_ys):
    x_min, x_max = ref_xs.min() - margin, ref_xs.max() + margin
    y_min, y_max = ref_ys.min() - margin, ref_ys.max() + margin
    cols = int(np.ceil((x_max - x_min) / resolution))
    rows = int(np.ceil((y_max - y_min) / resolution))
    return rows, cols, (x_min, x_max, y_min, y_max)


def build_obstacle_grid(circles, influence_radius, cbf_weight, decay_rate,
                        resolution, rows, cols, bounds):
    x_min, _, y_min, _ = bounds
    grid = np.zeros((rows, cols), dtype=float)
    for r in range(rows):
        wy = y_min + (r + 0.5) * resolution
        for c in range(cols):
            wx = x_min + (c + 0.5) * resolution
            grid[r, c] = obstacle_cost(wx, wy, circles, influence_radius,
                                       cbf_weight, decay_rate)
    return grid


def save_grid(grid, filepath):
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    np.savetxt(filepath, grid, fmt="%.5f", delimiter=",")


def save_preview(grid, ref_xs, ref_ys, bounds, title, filepath, circles=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("[WARNING] matplotlib not available, skipping the preview")
        return

    x_min, x_max, y_min, y_max = bounds
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(grid, origin="lower", extent=[x_min, x_max, y_min, y_max],
                   cmap="magma", interpolation="bilinear", aspect="equal")
    ax.plot(ref_xs, ref_ys, "b--", linewidth=1.6, label="Reference path")
    ax.plot(0, 0, "g^", markersize=11, label="Start")
    if circles:
        for (cx, cy, r) in circles:
            ax.add_patch(patches.Circle((cx, cy), r, fill=False,
                                        edgecolor="black", linewidth=1.4))
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(filepath, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Preview: {filepath}")


def main():
    print("=" * 60)
    print("  OBSTACLE COSTMAP GENERATOR")
    print("=" * 60)
    cfg = load_cfg()
    resolution = cfg["GRID_RESOLUTION"]
    margin = cfg["GRID_MARGIN"]
    ref_xs, ref_ys = load_ref_path(cfg["REF_PATH_FILE"])
    rows, cols, bounds = grid_bounds(resolution, margin, ref_xs, ref_ys)
    print(f"Grid: {rows}x{cols} @ {resolution} m/cell, margin {margin} m")

    # --- obstacle costmap ---
    circles = load_obstacle_circles(OBSTACLES)
    cbf = cfg["CBF_PARAMETERS"]
    # CBF_PARAMETERS.influence_radius is tuned for the analytic CBF and is narrower than
    # GRID_RESOLUTION, so reusing it here would put the whole falloff band inside one cell
    # and leave the costmap almost blank. The costmap therefore takes its own, wider
    # radius; the CBF keeps its narrow one.
    obstacle_influence_radius = cfg.get("OBSTACLE_COSTMAP_INFLUENCE_RADIUS", cbf["influence_radius"])
    print(f"\nObstacles: {len(circles)} (costmap influence_radius={obstacle_influence_radius} m)")
    for (cx, cy, r) in circles:
        print(f"  centre=({cx},{cy}) r={r}")
    obs_grid = build_obstacle_grid(circles, obstacle_influence_radius, cbf["cbf_weight"],
                                   cbf["decay_rate"], resolution, rows, cols, bounds)
    obstacle_costmap_file = cfg.get("OBSTACLE_COSTMAP_FILE", "data/obstacle_costmap.csv")
    save_grid(obs_grid, obstacle_costmap_file)
    print(f"[OK] Obstacle costmap: {obstacle_costmap_file} "
          f"(cost {obs_grid.min():.4f}..{min(obs_grid.max(), 1e6):.4f})")
    save_preview(obs_grid, ref_xs, ref_ys, bounds,
                 "Obstacle costmap (additional to the CBF, not a replacement)",
                 obstacle_costmap_file.replace(".csv", "_preview.png"),
                 circles=circles)

    # --- raw obstacle list, for the C++ CBF and the BeamNG spawn ---
    obstacles_file = cfg.get("OBSTACLES_FILE", "data/obstacles.json")
    os.makedirs(os.path.dirname(obstacles_file), exist_ok=True)
    with open(obstacles_file, "w") as f:
        json.dump(OBSTACLES, f, indent=2)
    print(f"[OK] Raw obstacle list: {obstacles_file}")

    print("\n" + "=" * 60)
    print("  DONE. Re-run the C++ side (config.json is unchanged, only the")
    print("  grid and JSON files were regenerated).")
    print("=" * 60)


if __name__ == "__main__":
    main()
