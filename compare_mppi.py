"""
compare_mppi.py - Uc MPPI'i AYNI BeamNG tezgahinda karsilastirir.

Girdi : run_log_<controller>.csv  (bridge yazar; solve_ms kolonu dahil)
Cikti : konsol tablosu + mppi_comparison.png

Kontrolculer:
  cpp    - bizim C++/CUDA MPPI
  jax    - jax-mppi (jlehtomaa), Williams et al. 2017, JAX/XLA
  python - python_simple_mppi (MizuhoAOKI), saf NumPy (bizim C++'in referansi)

Not: hepsi ayni config.json'u (K, T, dt, L, sigma, agirliklar, engeller, yol) kullanir;
tek fark implementasyon. Kismi kosular (yol tamamlanmamis) tabloda isaretlenir.
"""
import csv
import json
import os
import sys

import numpy as np

RUNS_DIR = "runs"

ALL_CONTROLLERS = [
    ("cpp", "C++/CUDA (bizim)", "#2ca02c"),
    ("torch", "PyTorch (pytorch_mppi)", "#ff7f0e"),
    ("jax", "JAX (jax-mppi)", "#1f77b4"),
    ("python", "Python (referans)", "#d62728"),
]

# Hangi kontrolcüler karsilastirilacak: komut satirindan sec, yoksa logu olan hepsi.
#   python compare_mppi.py            -> mevcut tum loglar
#   python compare_mppi.py cpp python -> sadece bu ikisi
_sel = [a.lower() for a in sys.argv[1:]]
# --bench : BeamNG kosusu yerine bench_solve.py'nin SIMULATORSUZ kosularini karsilastir
# (ayni pist, ayni engeller, ama render/kare senkronu yok -> temiz hiz sayisi).
BENCH = "--bench" in _sel
_sel = [a for a in _sel if a != "--bench"]
PREFIX = "bench_log_" if BENCH else "run_log_"
CONTROLLERS = [c for c in ALL_CONTROLLERS if (not _sel or c[0] in _sel)]
OUT_PNG = (("bench_comparison" if BENCH else "mppi_comparison")
           + ("" if not _sel else "_" + "_".join(_sel)) + ".png")


def load(name):
    fn = os.path.join(RUNS_DIR, f"{PREFIX}{name}.csv")
    if not os.path.exists(fn):
        return None
    rows = list(csv.DictReader(open(fn)))
    if not rows:
        return None
    g = lambda k: np.array([float(r[k]) for r in rows])
    d = dict(n=len(rows), mx=g("mx"), my=g("my"), v=g("v"), dev=g("min_dist"),
             steer=g("steer_rad"), t=g("t"), myaw_deg=g("myaw_deg"))
    d["solve"] = g("solve_ms") if "solve_ms" in rows[0] else None
    d["dist"] = float(np.sum(np.hypot(np.diff(d["mx"]), np.diff(d["my"]))))
    d["lap"] = bool(d["mx"].min() < -25 and d["mx"].max() > 25 and d["my"].max() > 25)
    return d


def clearances(d, obstacles, cfg):
    """Her engele en yakin gecis acikligi.

    Iki olcu: merkez-bazli (arac bir nokta sayilirsa) ve AYAK IZI bazli - aracin 9
    govde noktasi yaw'a gore dondurulur, mppi_core.cu compute_cbf_cost ile ayni.
    Carpisma kararinda gecerli olan ayak izi olcusudur (<0 ise carpma).
    """
    fp = cfg.get("VEHICLE_FOOTPRINT", {})
    m = float(fp.get("safety_margin_rate", 1.0))
    vw, vl = float(fp.get("width", 1.9)) * m, float(fp.get("length", 4.5)) * m
    bx = 0.5 * vl * np.array([-1., -1., -1., 0., 0., 0., 1., 1., 1.])
    by = 0.5 * vw * np.array([-1., 0., 1., 1., -1., 0., 1., 0., -1.])
    yaw = np.radians(d["myaw_deg"])
    gx = d["mx"][:, None] + bx[None, :] * np.cos(yaw)[:, None] - by[None, :] * np.sin(yaw)[:, None]
    gy = d["my"][:, None] + bx[None, :] * np.sin(yaw)[:, None] + by[None, :] * np.cos(yaw)[:, None]

    center, foot = [], []
    for o in obstacles:
        ox, oy, r = o[0], o[1], (o[2] if len(o) == 3 else 0.5 * np.hypot(o[2], o[3]))
        center.append(float(np.min(np.hypot(d["mx"] - ox, d["my"] - oy)) - r))
        foot.append(float(np.min(np.hypot(gx - ox, gy - oy)) - r))
    return center, foot


def main():
    cfg = json.load(open("config.json"))
    obstacles = json.load(open(cfg["OBSTACLES_FILE"]))
    dt = float(cfg["delta_t"])
    print("=" * 78)
    print(f"  MPPI KARSILASTIRMASI  |  K={cfg['number_of_samples_K']} T={cfg['horizon_step_T']} "
          f"dt={dt} L={cfg['wheel_base']} | {len(obstacles)} engel "
          f"| CBF weight={cfg['CBF_PARAMETERS']['cbf_weight']}")
    print("=" * 78)

    data = {}
    for key, label, _ in CONTROLLERS:
        d = load(key)
        if d is None:
            print(f"\n  {label}: log YOK ({PREFIX}{key}.csv)")
            continue
        data[key] = d
        s = np.sort(d["solve"][1:]) if d["solve"] is not None and d["n"] > 1 else None
        cl, clf = clearances(d, obstacles, cfg)
        print(f"\n  {label}   {'[TAM TUR]' if d['lap'] else '[KISMI KOSU]'}")
        print(f"    adim / sim zamani : {d['n']:5d}  /  {d['n']*dt:6.1f} s")
        print(f"    katedilen mesafe  : {d['dist']:6.1f} m")
        print(f"    hiz ort / std     : {d['v'].mean():5.2f} / {d['v'].std():4.2f} m/s")
        print(f"    SAPMA ort/p95/maks: {d['dev'].mean():5.3f} / "
              f"{np.percentile(d['dev'],95):5.3f} / {d['dev'].max():5.3f} m")
        # Saturasyon esigi config'den gelir; sabit yazilirsa max_steer_abs degisince sessizce bozulur.
        _sat = 0.97 * float(cfg["max_steer_abs"])
        print(f"    direksiyon satur. : {100*(np.abs(d['steer'])>_sat).mean():4.1f} %")
        print(f"    engel aciklik min : {min(clf):+5.2f} m (ayak izi) / "
              f"{min(cl):+5.2f} m (merkez)  {'(CARPMA!)' if min(clf) < 0 else ''}")
        if s is not None:
            print(f"    SOLVE ms med/ort  : {np.median(s):8.2f} / {s.mean():8.2f}   "
                  f"(p95 {np.percentile(s,95):.2f}, maks {s.max():.1f})")
            print(f"    20 Hz (50 ms)     : {'ALTINDA - real-time OK' if np.median(s) < 50 else 'USTUNDE - real-time YOK'}")

    # --- hiz orani ---
    if "cpp" in data and data["cpp"]["solve"] is not None:
        base = np.median(np.sort(data["cpp"]["solve"][1:]))
        print("\n" + "-" * 78)
        print("  HIZ ORANI (bizim C++'e gore, medyan solve" +
              (", SIMULATORSUZ):" if BENCH else ", BeamNG in-loop):"))
        for key, label, _ in CONTROLLERS:
            if key in data and data[key]["solve"] is not None:
                m = np.median(np.sort(data[key]["solve"][1:]))
                print(f"    {label:22}: {m:8.2f} ms   ({m/base:6.1f}x)")

    plot(data, obstacles, cfg)


def plot(data, obstacles, cfg):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("\n[uyari] matplotlib yok, grafik atlandi")
        return
    if not data:
        return

    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.1, 1])

    # 1) solve time PER CYCLE (line, log y) - x ekseni her bir kontrol dongusu
    ax = fig.add_subplot(gs[0, 0])
    any_solve = False
    for key, label, col in CONTROLLERS:
        if key in data and data[key]["solve"] is not None:
            s = data[key]["solve"]
            # ilk adim JIT/CUDA isinmasi -> cizgide gosterme (medyani da bozmasin)
            steps = np.arange(1, len(s))
            ax.plot(steps, s[1:], color=col, lw=0.9, alpha=0.85,
                    label=f"{label.split(' (')[0]} (med {np.median(s[1:]):.2f} ms)")
            ax.axhline(np.median(s[1:]), color=col, ls=":", lw=1.0, alpha=0.6)
            any_solve = True
    if any_solve:
        meds = sorted(float(np.median(data[k]["solve"][1:]))
                      for k, _, _ in CONTROLLERS
                      if k in data and data[k]["solve"] is not None)
        spread = meds[-1] / meds[0] if meds[0] > 0 else 1.0
        # Buyukluk mertebesi farki varsa log sart (yoksa hizli olan tabanda duz cizgi olur
        # ve kendi degisimi hic gorunmez). Yakinsalar dogrusal ms daha okunakli.
        use_log = spread > 20
        if use_log:
            ax.set_yscale("log")
        ax.set_xlabel("kontrol dongusu (cycle)")
        ax.set_ylabel("solve suresi [ms]" + (" (log)" if use_log else ""))

        # 20 Hz gercek-zaman butcesi: altta kalan bolge = real-time mumkun
        ax.axhspan(ax.get_ylim()[0], 50, color="green", alpha=0.06)
        ax.axhline(50, color="gray", ls="--", lw=1.3)
        ax.text(0.99, 0.02, "20 Hz butcesi (50 ms) - altinda kalan real-time surebilir",
                color="gray", fontsize=8, ha="right", va="bottom", transform=ax.transAxes)

        ax.legend(fontsize=8, loc="center right")
        ax.grid(alpha=0.3, which="both")


    # 2) ozet tablo - her kontrolcu icin tek satir
    ax = fig.add_subplot(gs[0, 1])
    ax.axis("off")
    header = ["", "solve md\n[ms]", "sapma ort\n[m]", "sapma maks\n[m]", "adim"]
    rows, row_colors, vals, full = [], [], [], []
    for key, label, colr in CONTROLLERS:
        if key not in data:
            continue
        d = data[key]
        s = d["solve"]
        med = np.median(s[1:]) if s is not None and d["n"] > 1 else float("nan")
        rows.append([label.split(" (")[0],
                     f"{med:.2f}" if med < 100 else f"{med:.0f}",
                     f"{d['dev'].mean():.3f}",
                     f"{d['dev'].max():.3f}",
                     f"{d['n']}" + ("" if d["lap"] else "*")])
        vals.append([med, d["dev"].mean(), d["dev"].max()])
        row_colors.append(colr)
        full.append(d["lap"])

    # En iyi degeri kirmizi yaz. Kalan sayisal sutunlarin hepsinde kucuk = iyi.
    # Yarim kalan kosular yarisa girmez: yolun tamamini surmeden dusuk sapma almak kolay.
    a = np.array(vals, dtype=float)
    elig = np.array(full) if any(full) else np.ones(len(full), bool)
    best = {}
    for c in range(a.shape[1]):
        col_v = np.where(elig, a[:, c], np.nan)
        if np.all(np.isnan(col_v)):
            continue
        best[c + 1] = int(np.nanargmin(col_v))

    tbl = ax.table(cellText=rows, colLabels=header, cellLoc="center", loc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1, 1.9)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#cccccc")
        if r == 0:                                  # baslik satiri
            cell.set_facecolor("#f0f0f0")
            cell.set_text_props(fontweight="bold")
        elif c == 0:                                # kontrolcu adi - yorunge rengiyle ayni
            cell.set_text_props(color=row_colors[r - 1], fontweight="bold")
            cell.set_facecolor("#fafafa")
        elif best.get(c) == r - 1:                  # sutunun en iyisi
            cell.set_text_props(color="red", fontweight="bold")

    # 3) trajectories
    ax = fig.add_subplot(gs[1, :])
    p = np.loadtxt(cfg["REF_PATH_FILE"], delimiter=",", skiprows=1)
    ax.plot(p[:, 0], p[:, 1], "k--", lw=1.2, label="referans yol")
    for o in obstacles:
        r = o[2] if len(o) == 3 else 0.5 * np.hypot(o[2], o[3])
        ax.add_patch(patches.Circle((o[0], o[1]), r, facecolor="red",
                                    edgecolor="darkred", alpha=0.85, zorder=3))
    for key, label, col in CONTROLLERS:
        if key in data:
            d = data[key]
            tag = "" if d["lap"] else " [kismi]"
            ax.plot(d["mx"], d["my"], color=col, lw=1.4, alpha=0.85,
                    label=f"{label.split(' (')[0]}{tag}")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="center", fontsize=9)
    ax.set_xlabel("MPPI X [m]"); ax.set_ylabel("MPPI Y [m]")

    fig.suptitle("MPPI Karsilastirmasi", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(OUT_PNG, dpi=135, bbox_inches="tight")
    print(f"\n[OK] grafik: {OUT_PNG}")


if __name__ == "__main__":
    main()
