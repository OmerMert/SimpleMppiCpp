"""
generate_costmaps.py - scenario.py'deki engel + engebe tanimlarindan IKI costmap
uretir + C++ tarafinin okuyacagi ham engel listesini disari yazar.

Girdi (tek kaynak): scenario.py OBSTACLES + ROUGHNESS.
Ciktilar (config.json'daki *_FILE anahtarlariyla eslesir):
  - data/obstacle_costmap.csv   : engel maliyet grid'i (MPPI'daki CBF'ye EK, onu
                                   degistirmez - bkz. obstacles.py docstring)
  - data/roughness_costmap.csv  : engebe (roughness) YUMUSAK maliyet grid'i
  - data/obstacles.json         : ham engel listesi (C++ CBF + BeamNG fiziksel
                                   spawn icin - bunlar costmap DEGIL, tam gecen
                                   cemberler/dikdortgenler gerektirir)
  - data/obstacle_costmap_preview.png, data/roughness_costmap_preview.png

Grid sinirlari (x_min..x_max, y_min..y_max) referans yol + GRID_MARGIN'dan
gelir ve iki costmap de AYNI sinirlari/cozunurlugu kullanir (main.cpp bu
varsayimla hizalanir - bkz. main.cpp ReadConfig).
"""
import csv
import json
import os

import numpy as np

from obstacles import load_obstacle_circles, obstacle_cost
from roughness import load_roughness_zones, roughness_severity, terrain_height
from scenario import OBSTACLES, ROUGHNESS


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


def build_roughness_grids(zones, resolution, rows, cols, bounds):
    x_min, _, y_min, _ = bounds
    severity = np.zeros((rows, cols), dtype=float)
    elevation = np.zeros((rows, cols), dtype=float)
    for r in range(rows):
        wy = y_min + (r + 0.5) * resolution
        for c in range(cols):
            wx = x_min + (c + 0.5) * resolution
            severity[r, c] = roughness_severity(wx, wy, zones)
            elevation[r, c] = terrain_height(wx, wy, zones)
    return severity, elevation


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


def save_preview(grid, ref_xs, ref_ys, bounds, title, filepath, cmap="Reds",
                 circles=None, zones=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("[WARNING] matplotlib yok, onizleme atlandi")
        return

    x_min, x_max, y_min, y_max = bounds
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(grid, origin="lower", extent=[x_min, x_max, y_min, y_max],
                   cmap=cmap, interpolation="bilinear", aspect="equal")
    ax.plot(ref_xs, ref_ys, "b--", linewidth=1.6, label="Referans yol")
    ax.plot(0, 0, "g^", markersize=11, label="Baslangic")
    if circles:
        for (cx, cy, r) in circles:
            ax.add_patch(patches.Circle((cx, cy), r, fill=False,
                                        edgecolor="black", linewidth=1.4))
    if zones:
        for (cx, cy, R, amp, wl) in zones:
            ax.add_patch(patches.Circle((cx, cy), R, fill=False,
                                        edgecolor="black", linewidth=1.4, linestyle="--"))
            ax.annotate(f"amp={amp} wl={wl}", (cx, cy), textcoords="offset points",
                        xytext=(6, 6), fontsize=8, fontweight="bold")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(filepath, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"[OK] Onizleme: {filepath}")


def main():
    print("=" * 60)
    print("  COSTMAP GENERATOR (engel + engebe)")
    print("=" * 60)
    cfg = load_cfg()
    resolution = cfg["GRID_RESOLUTION"]
    margin = cfg["GRID_MARGIN"]
    ref_xs, ref_ys = load_ref_path(cfg["REF_PATH_FILE"])
    rows, cols, bounds = grid_bounds(resolution, margin, ref_xs, ref_ys)
    print(f"Grid: {rows}x{cols} @ {resolution} m/hucre, margin {margin} m")

    # --- Engebe (roughness) costmap ---
    zones = load_roughness_zones(ROUGHNESS)
    print(f"\nEngebe bolgesi sayisi: {len(zones)}")
    for z in zones:
        print(f"  merkez=({z[0]},{z[1]}) R={z[2]} amp={z[3]} wl={z[4]}")
    severity, elevation = build_roughness_grids(zones, resolution, rows, cols, bounds)
    roughness_file = cfg.get("ROUGHNESS_FILE", "data/roughness_costmap.csv")
    save_grid(severity, roughness_file)
    print(f"[OK] Roughness costmap: {roughness_file} "
          f"(siddet {severity.min():.4f}..{severity.max():.4f}, "
          f"yukseklik {elevation.min():+.3f}..{elevation.max():+.3f} m)")
    save_preview(severity, ref_xs, ref_ys, bounds,
                "Engebe (roughness) YUMUSAK maliyet",
                roughness_file.replace(".csv", "_preview.png"),
                cmap="Reds", zones=zones)

    # --- Engel (obstacle) costmap ---
    circles = load_obstacle_circles(OBSTACLES)
    cbf = cfg["CBF_PARAMETERS"]
    # NOT: CBF_PARAMETERS.influence_radius (0.2 m) analitik CBF icin ayarli - GRID_RESOLUTION
    # (1.0 m) ile kullanilirsa yumusak gecis bandi hucre boyutundan kucuk kalir ve costmap'te
    # neredeyse hic gorunmez. Bu yuzden costmap ayrica (ve daha genis) bir etki yaricapi
    # kullanir; CBF'nin kendi (dar/hassas) yaricapina DOKUNMAZ.
    obstacle_influence_radius = cfg.get("OBSTACLE_COSTMAP_INFLUENCE_RADIUS", cbf["influence_radius"])
    print(f"\nEngel sayisi: {len(circles)} (costmap influence_radius={obstacle_influence_radius} m)")
    for (cx, cy, r) in circles:
        print(f"  merkez=({cx},{cy}) r={r}")
    obs_grid = build_obstacle_grid(circles, obstacle_influence_radius, cbf["cbf_weight"],
                                   cbf["decay_rate"], resolution, rows, cols, bounds)
    obstacle_costmap_file = cfg.get("OBSTACLE_COSTMAP_FILE", "data/obstacle_costmap.csv")
    save_grid(obs_grid, obstacle_costmap_file)
    print(f"[OK] Obstacle costmap: {obstacle_costmap_file} "
          f"(maliyet {obs_grid.min():.4f}..{min(obs_grid.max(), 1e6):.4f})")
    save_preview(obs_grid, ref_xs, ref_ys, bounds,
                "Engel (obstacle) EK maliyet (CBF'ye ek, onun yerine degil)",
                obstacle_costmap_file.replace(".csv", "_preview.png"),
                cmap="magma", circles=circles)

    # --- Ham engel listesi (C++ CBF + BeamNG fiziksel spawn icin) ---
    obstacles_file = cfg.get("OBSTACLES_FILE", "data/obstacles.json")
    os.makedirs(os.path.dirname(obstacles_file), exist_ok=True)
    with open(obstacles_file, "w") as f:
        json.dump(OBSTACLES, f, indent=2)
    print(f"[OK] Ham engel listesi: {obstacles_file}")

    print("\n" + "=" * 60)
    print("  BITTI. C++ tarafini yeniden calistir (config.json degismedi,")
    print("  sadece grid/JSON dosyalari guncellendi).")
    print("=" * 60)


if __name__ == "__main__":
    main()
