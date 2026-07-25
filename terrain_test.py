"""
terrain_test.py - BeamNG runtime heightmap (engebe) DOGRULAMA testi.

Amac: beamngpy 1.35'teki Terrain_Importer.import_heightmap API'sinin bizim
mevcut sahnemizde (tech_ground) calisip calismadigini ve aracin engebeli zemine
FIZIKSEL tepki verip vermedigini (zipliyor/egiliyor mu) dogrulamak.

Bu betik MPPI/bridge'den BAGIMSIZDIR - hicbir seyi degistirmez. Sadece:
  1) BeamNG'yi acar, tech_ground'a etk800 spawn eder
  2) Sentetik DALGALI bir heightmap uretir (sin tabanli tumsekler)
  3) import_heightmap ile zemini kabartir
  4) Araci uzerinden duz surer, dikey tepkiyi (z, vz, pitch) olcer

Sonuc: z genligi > ~0.2 m ve |vz| ani sicramalar + pitch degisimi varsa
       -> zemin fiziksel olarak calisiyor demektir.

NOT: import_heightmap cok uzun sure (>~30 sn) yanit vermezse, muhtemelen bu
     level duzenlenebilir bir TerrainBlock'a sahip degildir; Ctrl+C ile durdur.
"""
import math
import os
import importlib.util

import beamngpy
from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Electrics

# ---- BeamNG kurulum yolu (beamng_bridge.py ile ayni) ----
BNG_HOME = r"D:\BeamNG.tech.v0.38.5.0"
BNG_USER = r"D:\BeamNg"

# ---- Heightmap parametreleri ----
# N=255 => Lua findBoundingSquare 256'ya (2'nin kuvveti) yuvarlar; data 255x255
# fill ederek terrainin NEREDEYSE TAMAMI engebeli olur (128 secseydik bmp=256 olur,
# veri sadece bir kadrani doldururdu ve arac duz kisma kayardi).
N        = 255     # data boyutu (final bmp 256)
SCALE    = 1.0     # metre/piksel
AMP      = 0.25    # tumsek genligi [m] -> relief 4*amp = 1.0 m
PERIOD   = 8       # dalga boyu [piksel] = 8 m
DRIVE_TICKS = 200  # ~10 s surus (20 Hz)
THROTTLE = 0.40    # engebede hareket icin biraz gaz


def load_terrain_importer():
    """Terrain_Importer'i IZOLE yukle (beamngpy.tools/__init__ fastapi cekiyor)."""
    p = os.path.join(os.path.dirname(beamngpy.__file__), "tools", "terrain_import.py")
    spec = importlib.util.spec_from_file_location("bng_terrain_import", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Terrain_Importer


def bounding_square(s):
    """Lua findBoundingSquare ile ayni: s'den buyuk en kucuk 2'nin kuvveti."""
    t = 64
    for _ in range(20):
        t *= 2
        if t > s:
            return t
    return None


def make_bumpy(n, amp, period):
    """0-TABANLI dict-of-dict: data[i][j], i,j in 0..n-1.

    Lua tarafi (terrainImporter.lua:53-56) veriyi 'for x=0,xSize do data[x]...'
    seklinde 0-TABANLI okur. Python nested-list -> msgpack -> Lua'da 1-tabanli
    tablo olur ve data[0] nil dondurur (onceki cokme). 0-tabanli integer anahtarli
    dict gonderince Lua data[0], data[0][0] ... dogru calisir.
    """
    data = {}
    for i in range(n):
        row = {}
        for j in range(n):
            row[j] = float(amp * (math.sin(2 * math.pi * i / period) +
                                  math.sin(2 * math.pi * j / period)) + 2.0 * amp)
        data[i] = row
    return data


def quat_to_pitch(qx, qy, qz, qw):
    s = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    return math.degrees(math.asin(s))


def main():
    TI = load_terrain_importer()
    print("[Test] Terrain_Importer yuklendi.")

    bng = BeamNGpy("localhost", 25252, home=BNG_HOME, user=BNG_USER)
    bng.open()
    print("[Test] BeamNG baglandi.")

    veh = None
    try:
        scenario = Scenario("tech_ground", "terrain_test")
        veh = Vehicle("ego", model="etk800", license="TERRAIN")
        veh.sensors.attach("electrics", Electrics())
        scenario.add_vehicle(veh, pos=(0.0, 0.0, 0.5), rot_quat=(0.0, 0.0, 0.0, 1.0))
        scenario.make(bng)

        bng.settings.set_deterministic(20)   # 20 Hz
        bng.scenario.load(scenario)
        bng.scenario.start()
        veh.control(throttle=0.0, brake=0.0, steering=0.0)
        bng.control.step(20, wait=True)

        # --- DUZ zemin referansi ---
        veh.sensors.poll()
        s = veh.sensors["state"]
        z_flat = s["pos"][2]
        print(f"[Test] Duz zemin z = {z_flat:.3f} m")

        # --- Heightmap uret + import et ---
        data = make_bumpy(N, AMP, PERIOD)
        zmin, zmax = 0.0, 4.0 * AMP
        print(f"[Test] {N}x{N} heightmap import ediliyor "
              f"(relief {zmax:.2f} m, dalga boyu {PERIOD*SCALE:.0f} m, scale {SCALE} m/px)...")
        print("[Test] (BeamNG yanit verene kadar bekleniyor - takilirsa Ctrl+C)")
        try:
            TI.import_heightmap(bng, data, N, N, scale=SCALE,
                                zMin=zmin, zMax=zmax, isYFlipped=True)
            print("[Test] >>> import_heightmap BASARILI (ack alindi) <<<")
        except Exception as e:
            print(f"[Test] >>> import_heightmap BASARISIZ: {e!r}")
            print("[Test] Bu level runtime terrain import'u desteklemiyor olabilir.")
            return

        # --- Terraini ORIGIN'e ortala ---
        # importHeightmap terrain kosesini origin'de birakir; ortalamazsak bumpy bolge
        # tek kadranda kalir ve arac (origin'de) kenara/duz kisma dusebilir.
        bmp = bounding_square(N)                 # 256
        half = 0.5 * bmp * SCALE                 # 128 m
        try:
            bng.control.queue_lua_command(
                "if core_terrain.getTerrain() then "
                f"core_terrain.getTerrain():setPosition(vec3({-half},{-half},0)); "
                "be:reloadCollision() end")
            print(f"[Test] Terrain origin'e ortalandi (kose offset -{half:.0f} m, "
                  f"kapsam [-{half:.0f},{half:.0f}] m).")
        except Exception as e:
            print(f"[Test] Ortalama atlandi ({e}); yine de denenecek.")
        for _ in range(10):
            bng.control.step(1, wait=True)

        # --- Araci engebenin ustune birak, otur ---
        veh.teleport(pos=(0.0, 0.0, z_flat + zmax + 1.0),
                     rot_quat=(0.0, 0.0, 0.0, 1.0), reset=True)
        for _ in range(50):
            veh.control(throttle=0.0, brake=0.0, steering=0.0)
            bng.control.step(1, wait=True)
        veh.sensors.poll()
        s = veh.sensors["state"]
        print(f"[Test] Oturma sonrasi z = {s['pos'][2]:.3f} m "
              f"(duz zemine gore fark {s['pos'][2]-z_flat:+.2f} m)")

        # --- Duz sur, dikey tepkiyi olc ---
        print(f"[Test] {DRIVE_TICKS} tik duz surus, dikey tepki olculuyor...\n")
        zs, vzs, pitches = [], [], []
        x0 = s["pos"][0]
        for k in range(DRIVE_TICKS):
            veh.control(throttle=THROTTLE, brake=0.0, steering=0.0)
            bng.control.step(1, wait=True)
            veh.sensors.poll()
            s = veh.sensors["state"]
            x, y, z = s["pos"]
            vx, vy, vz = s["vel"]
            pitch = quat_to_pitch(*s["rotation"])
            zs.append(z); vzs.append(vz); pitches.append(pitch)
            if k % 20 == 0:
                print(f"[{k:3d}] pos=({x:6.1f},{y:6.1f},{z:5.2f}) "
                      f"vz={vz:+5.2f} pitch={pitch:+5.1f} deg")

        dist = s["pos"][0] - x0
        z_span = max(zs) - min(zs)
        vz_max = max(abs(v) for v in vzs)
        pitch_span = max(pitches) - min(pitches)
        print("\n================ SONUC ================")
        print(f"katedilen x mesafesi : {dist:+.1f} m")
        print(f"z genligi (span)     : {z_span:.2f} m")
        print(f"|vz| maks (dikey hiz): {vz_max:.2f} m/s")
        print(f"pitch araligi        : {pitch_span:.1f} deg")
        if dist < 2.0:
            print(">>> UYARI: arac neredeyse hic ilerlememis (engebede takildi?).")
        if z_span > 0.2 and vz_max > 0.3:
            print(">>> BASARILI: arac engebeye FIZIKSEL tepki veriyor (zipliyor/egiliyor).")
            print(">>> Yani import_heightmap bu sahnede calisiyor - costmap->terrain yolu acik.")
        else:
            print(">>> Dikey tepki zayif: zemin degismemis ya da genlik cok dusuk olabilir.")
        print("======================================")

    except KeyboardInterrupt:
        print("\n[Test] Ctrl+C - durduruldu.")
    except Exception as e:
        print(f"[Test] HATA: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if veh is not None:
                veh.control(throttle=0.0, brake=0.0, steering=0.0,
                            parkingbrake=1.0, gear=0)
        except Exception:
            pass
        try:
            bng.close()
        except Exception:
            pass
        print("[Test] Kapandi.")


if __name__ == "__main__":
    main()
