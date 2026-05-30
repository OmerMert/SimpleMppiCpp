"""
generate_costmap.py - costmap.csv generator

Usage:
    Run this script to generate the costmap.csv file.
    Add obstacles to the OBSTACLES list below:
        (x, y, radius)      -> circular obstacle
        (x, y, w, h)        -> rectangular obstacle (width, height)

    Coordinates are in the MPPI world frame (metres).
    Same frame as the reference path coordinates.

Example:
    OBSTACLES = [
        (8.0, 5.0, 4.0),          # circle: center=(8,5), radius=4m
        (18.0, 25.0, 3.0),        # circle: center=(18,25), radius=3m
        (-10.0, 15.0, 6.0, 2.0),  # rectangle: center=(-10,15), 6m wide, 2m tall
    ]

Output:
    data/costmap.csv       (0/1 grid, compatible with COSTMAP_FILE in config.json)
    costmap_preview.png    (visual preview)
"""

import numpy as np
import json
import csv
import os

# ================================================================
#  OBSTACLE DEFINITIONS - ADD / EDIT HERE
# ================================================================
# Format:
#   (x, y, radius)           -> circular obstacle
#   (x, y, width, height)    -> rectangular obstacle

OBSTACLES = [
    (8.0, 5.0, 4.0),           # Circle: center=(8, 5), r=4m
    (18.0, -5.0, 4.0),         # Circle: center=(18, -5), r=4m
]

# ================================================================


def load_config(config_path="config.json"):
    """Read map parameters from config."""
    with open(config_path, 'r') as f:
        cfg = json.load(f)

    resolution = cfg["COSTMAP_RESOLUTION"]
    margin = cfg["COSTMAP_MARGIN"]
    costmap_file = cfg["COSTMAP_FILE"]
    ref_path_file = cfg["REF_PATH_FILE"]

    return resolution, margin, costmap_file, ref_path_file


def load_ref_path(filepath):
    """Load reference path and compute map bounds."""
    xs, ys = [], []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return np.array(xs), np.array(ys)


def generate_costmap(obstacles, resolution, margin, ref_xs, ref_ys):
    """
    Generate a costmap grid from an obstacle list.

    Returns:
        grid: numpy array (rows x cols), 0/1
        x_min, y_min: map origin (metres)
    """
    # Map bounds (reference path + margin)
    x_min = ref_xs.min() - margin
    x_max = ref_xs.max() + margin
    y_min = ref_ys.min() - margin
    y_max = ref_ys.max() + margin

    cols = int(np.ceil((x_max - x_min) / resolution))
    rows = int(np.ceil((y_max - y_min) / resolution))

    grid = np.zeros((rows, cols), dtype=int)

    for obs in obstacles:
        if len(obs) == 3:
            # Circular obstacle: (x, y, radius)
            cx, cy, radius = obs
            count = 0
            for r in range(rows):
                for c in range(cols):
                    # Cell centre -> world coordinates
                    wx = x_min + (c + 0.5) * resolution
                    wy = y_min + (r + 0.5) * resolution
                    dist = np.sqrt((wx - cx)**2 + (wy - cy)**2)
                    if dist <= radius:
                        grid[r, c] = 1
                        count += 1
            print(f"  Circle ({cx}, {cy}, r={radius}): {count} cells")

        elif len(obs) == 4:
            # Rectangular obstacle: (x, y, width, height)
            cx, cy, w, h = obs
            count = 0
            for r in range(rows):
                for c in range(cols):
                    wx = x_min + (c + 0.5) * resolution
                    wy = y_min + (r + 0.5) * resolution
                    if (abs(wx - cx) <= w / 2.0) and (abs(wy - cy) <= h / 2.0):
                        grid[r, c] = 1
                        count += 1
            print(f"  Rectangle ({cx}, {cy}, {w}x{h}): {count} cells")
        else:
            print(f"  WARNING: Unknown obstacle format: {obs}")

    return grid, x_min, y_min, x_max, y_max


def save_costmap(grid, filepath):
    """Save grid as CSV."""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    np.savetxt(filepath, grid, fmt='%d', delimiter=',')
    print(f"\n[OK] Costmap saved: {filepath}")
    print(f"     Grid size: {grid.shape[0]} x {grid.shape[1]}")
    print(f"     Obstacle cells: {np.sum(grid)} / {grid.size}")


def save_preview(grid, ref_xs, ref_ys, obstacles, x_min, x_max, y_min, y_max, filepath):
    """Save visual preview."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches

        fig, ax = plt.subplots(1, 1, figsize=(14, 7))

        # Cost map
        ax.imshow(grid, origin='lower', extent=[x_min, x_max, y_min, y_max],
                  cmap='Reds', alpha=0.8, vmin=0, vmax=1, interpolation='nearest')

        # Reference path
        ax.plot(ref_xs, ref_ys, 'b--', linewidth=2, label='Reference path')

        # Obstacle outlines
        for obs in obstacles:
            if len(obs) == 3:
                cx, cy, r = obs
                circle = patches.Circle((cx, cy), r, fill=False,
                                       edgecolor='red', linewidth=2, linestyle='-')
                ax.add_patch(circle)
                ax.annotate(f'({cx},{cy}) r={r}', (cx, cy),
                           textcoords="offset points", xytext=(10, 10),
                           fontsize=9, color='red', fontweight='bold')
            elif len(obs) == 4:
                cx, cy, w, h = obs
                rect = patches.Rectangle((cx - w/2, cy - h/2), w, h,
                                        fill=False, edgecolor='red', linewidth=2)
                ax.add_patch(rect)
                ax.annotate(f'({cx},{cy}) {w}x{h}', (cx, cy),
                           textcoords="offset points", xytext=(10, 10),
                           fontsize=9, color='red', fontweight='bold')

        # Start point
        ax.plot(0, 0, 'g^', markersize=12, label='Start (0,0)')

        ax.set_xticks(np.arange(x_min, x_max + 1, 10))
        ax.set_yticks(np.arange(y_min, y_max + 1, 10))
        ax.grid(True, alpha=0.3, linestyle=':')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title(f'costmap.csv - {grid.shape[0]}x{grid.shape[1]} grid, '
                     f'{np.sum(grid)} obstacle cells')
        ax.set_aspect('equal')
        ax.legend(loc='upper left')

        plt.tight_layout()
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"[OK] Preview saved: {filepath}")

    except ImportError:
        print("[WARNING] matplotlib not found, skipping preview")


def main():
    print("=" * 60)
    print("  COSTMAP GENERATOR")
    print("=" * 60)

    # Read config
    resolution, margin, costmap_file, ref_path_file = load_config()
    print(f"Resolution: {resolution} m/cell")
    print(f"Margin: {margin} m")
    print(f"Output: {costmap_file}")
    print(f"Ref path: {ref_path_file}")
    print(f"Obstacle count: {len(OBSTACLES)}")
    print()

    # Load reference path
    ref_xs, ref_ys = load_ref_path(ref_path_file)
    print(f"Reference path: {len(ref_xs)} points, "
          f"X[{ref_xs.min():.1f}, {ref_xs.max():.1f}], "
          f"Y[{ref_ys.min():.1f}, {ref_ys.max():.1f}]")
    print()

    # Build grid
    print("Building obstacles:")
    grid, x_min, y_min, x_max, y_max = generate_costmap(
        OBSTACLES, resolution, margin, ref_xs, ref_ys)

    # Save
    save_costmap(grid, costmap_file)

    # Preview
    preview_path = costmap_file.replace('.csv', '_preview.png')
    save_preview(grid, ref_xs, ref_ys, OBSTACLES,
                 x_min, x_max, y_min, y_max, preview_path)

    print("\n" + "=" * 60)
    print("  DONE! You can now run MppiCpp.exe.")
    print("=" * 60)


if __name__ == "__main__":
    main()
