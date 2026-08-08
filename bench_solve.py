"""
bench_solve.py - Dort MPPI'yi AYNI PISTTE, BeamNG OLMADAN kosturur.

BeamNG yerine ayni referans yolu (data/ovalpath.csv) ve ayni engelleri kullanan
BASSIZ bir tesis (plant) vardir: kontrolcunun kendi varsaydigi kinematik bisiklet
modeli. Yani her kontrolcu gercekten ayni turu surer - fark yalnizca "beyin".

Neden BeamNG'siz bir kosu: BeamNG canliyken bizim solve suremiz kosudan kosuya
3.68 <-> 15.46 ms arasi oynuyor (ayni exe, ayni config). Simulator hem CPU'yu
(fizik is parcaciklari) hem GPU'yu (render) bizimle paylasiyor; CPU tabanli
rakipler bundan daha az etkileniyor -> in-loop olcum GPU/CPU'yu paylasan tarafi
haksiz yere yavas gosterir. Burada simulator yok: tekrarlanabilir, donanim
paylasimindan arinmis rakam.

Olculenler (her kontrolcu icin): solve suresi + takip sapmasi + engel acikligi.
Sonuclar runs/bench_log_<kontrolcu>.csv icine BeamNG kosulariyla AYNI sutun
duzeninde yazilir (analyze_run.py / compare_mppi.py okuyabilsin diye).

Kullanim:
  python bench_solve.py                    # hepsi, tam tur
  python bench_solve.py cpp jax            # secili kontrolcu(ler)
  python bench_solve.py --steps 300 cpp    # adim siniri
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
LISTEN, SEND = 5005, 5006          # kontrolcu dinler / bize gonderir
OLD_CTRL_FMT = "dddi"              # c73e871 ve oncesi: solve_ms alani yok
# MPPI_FORCE_RTT=1: yeni build'de de kontrolcunun bildirdigi solve_ms yerine gidis-donus
# suresini kullan -> eski build'lerle BIREBIR ayni olcu (elmayla elma).
FORCE_RTT = os.environ.get("MPPI_FORCE_RTT", "0") == "1"
# MPPI_GAP_MS: her solve arasina bos bekleme koy. BeamNG step modunda sim solve boyunca
# DURDURULDUGU icin GPU'ya kimse is vermez ve iki solve arasinda ~50 ms bosluk olusur;
# bu bayrak o boslugu simulatorsuz taklit eder (BeamNG'siz, render'siz).
GAP_MS = float(os.environ.get("MPPI_GAP_MS", "0"))
MAX_STEPS = 2000                   # tam tur ~1200 adim; guvenlik siniri
PY_MAX_STEPS = 120                 # saf-Python ~4.2 s/adim -> tam tur ~85 dk, kisitli
WARMUP = 1                         # ilk adim(lar) istatistige girmez

CONTROLLERS = {
    "cpp":    ("C++/CUDA (bizim)",       None,                             5.0),
    "jax":    ("JAX (jax-mppi)",         "competitors/run_jax_mppi.py",   30.0),
    "torch":  ("PyTorch (pytorch_mppi)", "competitors/run_torch_mppi.py", 30.0),
    "python": ("Python (referans)",      "competitors/run_python_mppi.py", 60.0),
}


def free_port():
    """Onceki kosudan kalmis dinleyici/exe varsa temizle (paketler olu sokete gitmesin)."""
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    f"Get-NetUDPEndpoint -LocalPort {LISTEN} -EA SilentlyContinue | "
                    "%{ Stop-Process -Id $_.OwningProcess -Force -EA SilentlyContinue }"],
                   capture_output=True)
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Stop-Process -Name MppiCpp -Force -EA SilentlyContinue"],
                   capture_output=True)
    time.sleep(0.6)


# --------------------------------------------------------------------------------------
# Bassiz tesis: kontrolcunun varsaydigi kinematik bisiklet (referans repo'nun kendi
# demolarinda kullandigi tesisin ayni). BeamNG'nin tam arac fizigi degil - amac
# "ayni gorevi ayni sekilde kostur", arac dinamigini dogrulamak degil (o BeamNG'nin isi).
# --------------------------------------------------------------------------------------
def plant_step(x, y, yaw, v, steer, accel, dt, L, max_steer, max_accel):
    steer = max(-max_steer, min(max_steer, steer))
    accel = max(-max_accel, min(max_accel, accel))
    return (x + v * math.cos(yaw) * dt,
            y + v * math.sin(yaw) * dt,
            yaw + v / L * math.tan(steer) * dt,
            v + accel * dt)


def body_points(x, y, yaw, vw, vl):
    """Ayak izinin 9 govde noktasi (mppi_core.cu compute_cbf_cost ile ayni yerlesim)."""
    bx = 0.5 * vl * np.array([-1., -1., -1., 0., 0., 0., 1., 1., 1.])
    by = 0.5 * vw * np.array([-1., 0., 1., 1., -1., 0., 1., 0., -1.])
    c, s = math.cos(yaw), math.sin(yaw)
    return x + bx * c - by * s, y + bx * s + by * c


def run(key, cfg, ref_path, circles, max_steps):
    """Kontrolcuyu baslat, bassiz tesisle kapali cevrimde kostur, olculeri dondur."""
    label, wrapper, _ = CONTROLLERS[key]
    free_port()
    if wrapper is None:
        # MPPI_EXE ile baska bir build olculebilir (ornek: eski bir commit'in exe'si,
        # "ayni algoritma once daha hizliydi" iddiasini test etmek icin).
        exe = os.environ.get("MPPI_EXE") or os.path.join(PROJ, "MppiCpp.exe")
        cmd = [exe, str(LISTEN), str(SEND)]
    else:
        cmd = [sys.executable, "-u", os.path.join(PROJ, wrapper), str(LISTEN), str(SEND)]

    proc = subprocess.Popen(cmd, cwd=PROJ, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    # Kontrolcunun ciktisini SUREKLI bosalt: aksi halde boru dolar ve kontrolcu
    # stdout'a yazarken bloklanir (kosu ortasinda sessiz kilitlenme).
    ready = threading.Event()
    tail = []

    def _drain():
        for line in proc.stdout:
            tail.append(line.rstrip())
            del tail[:-40]
            if line.startswith("[PHASE]"):      # MPPI_PHASE_PROFILE=1 dokumu
                print("      " + line.rstrip(), flush=True)
            if "ilk durum bekleniyor" in line:
                ready.set()
        ready.set()

    threading.Thread(target=_drain, daemon=True).start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", SEND))
    sock.settimeout(CONTROLLERS[key][2] * 4)   # ilk cevap (JIT/allocation) icin genis

    if wrapper is None:
        time.sleep(1.5)                        # exe soketi acsin
    elif not ready.wait(180.0):                # sarmalayici hazir diyene kadar (JIT dahil)
        print(f"    {label}: BASLATILAMADI\n      " + "\n      ".join(tail[-8:]))
        sock.close(); proc.terminate(); return None

    dt = float(cfg["delta_t"]); L = float(cfg["wheel_base"])
    max_steer = float(cfg["max_steer_abs"]); max_accel = float(cfg["max_accel_abs"])
    vw, vl = H.footprint(cfg)
    path_xy = ref_path[:, :2]
    n_path = len(ref_path)

    # Baslangic: yolun ilk noktasi, referans hiziyla (v=0'da yaw_rate=v/L*tan(d)=0 ->
    # kinematik model direksiyona sagir kalir, o yuzden hareket halinde basliyoruz).
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
            print(f"    {label}: cevap gelmedi (timeout) - adim {step}\n      "
                  + "\n      ".join(tail[-6:]))
            break
        rtt_ms = (time.perf_counter() - t_send) * 1000.0
        if step == 0:
            sock.settimeout(CONTROLLERS[key][2])
        if FORCE_RTT and len(data) != struct.calcsize(OLD_CTRL_FMT):
            _, steer, accel, _sm, reset = struct.unpack("ddddi", data)
            solve_ms = rtt_ms          # eski build'le ayni olcuye indirgemek icin
        elif len(data) == struct.calcsize(OLD_CTRL_FMT):
            # Eski build'ler (c73e871 ve oncesi) ControlPacket'te solve_ms TASIMIYOR -
            # o surumleri olcebilmek icin gidis-donus suresini kullan. Localhost UDP
            # ek yuku her iki build icin ayni oldugundan karsilastirma adil kalir.
            _, steer, accel, reset = struct.unpack(OLD_CTRL_FMT, data)
            solve_ms = rtt_ms
        else:
            _, steer, accel, solve_ms, reset = struct.unpack("ddddi", data)
        if reset:                              # kontrolcu "yol bitti" diyor
            finished = True
            break

        # olcum: en yakin waypoint (ileri pencere), sapma, engel acikligi
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
        # min_dist sutunu bridge'te YOLDAN SAPMA anlamina gelir (beamng_bridge.py:548);
        # analiz araclari onu oyle okuyor. Engel acikligi en sona ek sutun olarak.
        rows.append((step, step * dt, solve_ms, x, y, math.degrees(yaw), v, dev,
                     path_xy[idx, 0], path_xy[idx, 1], steer, accel, 0.0, 0.0, 0.0, 0.0,
                     min_dist))

        if step % 200 == 0 and step:
            print(f"      [{step:4d}] yolun %{100.0 * idx / (n_path - 1):.0f}'i | "
                  f"solve {np.median(solves):.2f} ms | sapma {dev:.2f} m", flush=True)
        if idx >= n_path - 4:                  # tur tamam
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
    print(f"  AYNI PIST, SIMULATORSUZ  |  K={cfg['number_of_samples_K']} "
          f"T={cfg['horizon_step_T']} dt={cfg['delta_t']} | yol {len(ref_path)} nokta | "
          f"engel {len(circles)}")
    print("=" * 78)

    res = {}
    for key in sel:
        label = CONTROLLERS[key][0]
        n = min(max_steps, PY_MAX_STEPS) if key == "python" else max_steps
        note = f"  (adim siniri {n}: tam tur ~{n_full_min(ref_path):.0f} dk surerdi)" \
               if key == "python" and n < max_steps else ""
        print(f"\n  {label} kosuyor...{note}", flush=True)
        r = run(key, cfg, ref_path, circles, n)
        if r is None:
            print("    olcum alinamadi"); continue
        res[key] = r
        s, d = r["solve"], r["dev"]
        pct = 100.0 * r["idx"] / (r["n_path"] - 1)
        print(f"    {r['steps']:4d} adim | yolun %{pct:.0f}'i | "
              f"{'TUR TAMAM' if r['finished'] else 'yarim kaldi'}")
        print(f"    solve  : medyan {np.median(s):8.2f} ms | ort {s.mean():8.2f} | "
              f"p95 {np.percentile(s, 95):8.2f} | maks {s.max():8.2f}")
        print(f"    sapma  : ort {d.mean():.3f} m | p95 {np.percentile(d, 95):.3f} | "
              f"maks {d.max():.3f}")
        if np.isfinite(r["min_dist"]).any():
            md = np.nanmin(r["min_dist"])
            print(f"    engel  : en yakin aciklik {md:+.2f} m "
                  f"{'(CARPMA!)' if md < 0 else ''}")

    if not res:
        return
    print("\n" + "=" * 78)
    print(f"  {'':24} {'solve md':>10} {'kat':>8} {'sapma ort':>10} {'p95':>8} {'tur':>10}")
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
              f"{('TAM' if r['finished'] else 'yarim'):>10}")
    print("\n  20 Hz butcesi (%.0f ms):" % (1000 * float(H.load_setup()[0]["delta_t"])))
    for key in CONTROLLERS:
        if key in res:
            m = np.median(res[key]["solve"])
            print(f"    {CONTROLLERS[key][0]:24}: " +
                  (f"ALTINDA ({50 / m:.0f}x emniyet payi)" if m < 50
                   else f"USTUNDE ({m / 50:.0f}x asiyor)"))
    print(f"\n  Loglar: runs/bench_log_<kontrolcu>.csv")


def n_full_min(ref_path):
    """Saf-Python tam tur kac dakika surerdi (kabaca, 4.2 s/adim)."""
    return len(ref_path) * 4.2 / 60.0


if __name__ == "__main__":
    main()
