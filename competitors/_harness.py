"""
_harness.py - Rakip sarmalayicilarinin ORTAK iskeleti.

Amac: her sarmalayici dosyasinda SADECE o kutuphaneye ozel olan sey kalsin
(model + maliyet + kontrolcu kurulumu). Config/yol/engel yukleme, UDP protokolu,
en yakin waypoint takibi, yol-sonu tespiti, sure olcumu ve ozet burada - uc
sarmalayicida birebir ayni oldugu icin tek yerde durur.

UDP protokolu (MppiCpp.exe ile birebir ayni):
  recv  StatePacket   "dddddi" = time, x, y, yaw, v, valid            (bridge'ten)
  send  ControlPacket "ddddi"  = time, steer, accel, solve_ms, reset  (bridge'e)
"""
import json
import os
import socket
import struct
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
if PROJ not in sys.path:
    sys.path.insert(0, PROJ)

from obstacles import load_obstacle_circles          # noqa: E402
from scenario import OBSTACLES                       # noqa: E402

# Referans implementasyonla (MizuhoAOKI _get_nearest_waypoint) AYNI ileri pencere.
SEARCH_FWD = 200

STATE_FMT, CTRL_FMT = "dddddi", "ddddi"
STATE_SZ = struct.calcsize(STATE_FMT)


def load_setup():
    """config.json + referans yol + engel cemberleri (hepsi TEK kaynak)."""
    with open(os.path.join(PROJ, "config.json")) as f:
        cfg = json.load(f)
    ref_path = np.genfromtxt(os.path.join(PROJ, cfg["REF_PATH_FILE"]),
                             delimiter=",", skip_header=1)              # (N,4)
    circles = (np.array(load_obstacle_circles(OBSTACLES), dtype=float)
               if OBSTACLES else np.zeros((0, 3)))
    return cfg, ref_path, circles


def footprint(cfg):
    """Arac govde kutusu (config VEHICLE_FOOTPRINT) -> etkin (genislik, uzunluk)."""
    fp = cfg.get("VEHICLE_FOOTPRINT", {})
    m = float(fp.get("safety_margin_rate", 1.0))
    return float(fp.get("width", 1.9)) * m, float(fp.get("length", 4.5)) * m


def serve(tag, cfg, ref_path, solve, listen_port, send_port, banner=()):
    """UDP dongusu: durum al -> solve -> kontrol gonder. Sadece solve zamanlanir.

    solve(x, y, yaw, v, idx) -> (steer, accel)
        idx : en yakin waypoint indeksi (ileri-200 pencere ile burada hesaplanir).
        Yol sonuna gelindiginde IndexError firlatabilir (referans MPPI boyle yapar).
    """
    max_accel = float(cfg["max_accel_abs"])
    path_xy = ref_path[:, :2]
    n_path = len(ref_path)
    idx = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", listen_port))
    sock.settimeout(180.0)      # sadece bridge gercekten oldugunde devreye girer
    bridge_addr = ("127.0.0.1", send_port)

    for line in banner:
        print(f"[{tag}] {line}")
    print(f"[{tag}] UDP dinle {listen_port} -> gonder {send_port} | ilk durum bekleniyor...")

    timings, step = [], 0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                print(f"[{tag}] Bridge'ten durum gelmedi (timeout). Cikiliyor.")
                break
            if len(data) != STATE_SZ:
                continue
            t_stamp, x, y, yaw, v, valid = struct.unpack(STATE_FMT, data)
            if valid == 0:
                print(f"[{tag}] Bridge dur sinyali (valid=0). Cikiliyor.")
                break

            # en yakin waypoint (referansla ayni: sadece ileri)
            seg = path_xy[idx:min(n_path, idx + SEARCH_FWD)]
            idx += int(np.argmin((seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2))

            done = idx >= n_path - 4
            if not done:
                try:
                    t0 = time.perf_counter()
                    steer, accel = solve(x, y, yaw, v, idx)
                    solve_ms = (time.perf_counter() - t0) * 1000.0
                    timings.append(solve_ms)
                except IndexError:
                    done = True

            if done:
                print(f"[{tag}] Yol sonu - tamam.")
                sock.sendto(struct.pack(CTRL_FMT, t_stamp, 0.0, -max_accel, 0.0, 1),
                            bridge_addr)
                break

            sock.sendto(struct.pack(CTRL_FMT, t_stamp, float(steer), float(accel),
                                    solve_ms, 0), bridge_addr)
            if step % 20 == 0:
                print(f"[{step:4d}] solve={solve_ms:8.2f} ms | pos=({x:+6.1f},{y:+6.1f}) "
                      f"v={v:4.2f} u=({steer:+.2f},{accel:+.2f})")
            step += 1
    except KeyboardInterrupt:
        print(f"\n[{tag}] Ctrl+C")
    finally:
        sock.close()
        if timings:
            a = np.array(timings)
            print(f"\n[{tag}] SOLVE SURESI (ms) / {len(a)} adim: ort {a.mean():.2f} | "
                  f"medyan {np.median(a):.2f} | p95 {np.percentile(a, 95):.2f} | "
                  f"maks {a.max():.2f}")
        print(f"[{tag}] (per-step solve_ms bridge tarafindan runs/ altina yaziliyor)")


def ports():
    """Bridge'in verdigi <listen_port> <send_port> argumanlari."""
    return (int(sys.argv[1]) if len(sys.argv) > 1 else 5005,
            int(sys.argv[2]) if len(sys.argv) > 2 else 5006)
