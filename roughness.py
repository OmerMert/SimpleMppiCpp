"""
roughness.py - Ortak engebe (offroad) modeli.

scenario.py "ROUGHNESS" -> engebe BOLGELERI (MPPI cercevesi, metre):
    [x, y, radius, amplitude, wavelength]
      x, y       : bolge merkezi
      radius     : etki yaricapi (bu yaricapta engebe 0'a yumusar - kenarda duvar OLMAZ)
      amplitude  : tumsek yuksekligi [m]
      wavelength : tumsek dalga boyu [m]

Tek kaynaktan IKI turev uretilir:
  1) terrain_height(x, y)     -> BeamNG icin GERCEK yukseklik alani (import_heightmap)
  2) roughness_severity(x, y) -> MPPI icin YUMUSAK maliyet (purufsuz rotayi tercih et)

Bu modul generate_costmaps.py (grid + onizleme) ve beamng_bridge.py (heightmap)
tarafindan ortak kullanilir - engellerdeki (obstacles.py) tek-kaynak felsefesiyle ayni.
Bolgelerin kendisi artik config.json'da degil, scenario.py ROUGHNESS'te.
"""
import math


def load_roughness_zones(raw_list):
    """scenario.py ROUGHNESS (ham liste) -> [(x, y, R, amp, wl), ...]."""
    zones = []
    for z in raw_list:
        if len(z) >= 5:
            zones.append(tuple(float(v) for v in z[:5]))
    return zones


def _window(d, R):
    """Radyal raised-cosine pencere: merkezde 1 -> kenarda (d=R) 0.

    Bolge sinirinda yuksekligi yumusakca 0'a indirir; aksi halde ani yukseklik
    basamagi olusur ve bu FIZIKSEL BIR DUVAR gibi davranir (istemedigimiz sey).
    """
    if d >= R:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * d / R))


def terrain_height(x, y, zones):
    """MPPI (x, y) noktasindaki yukseklik [m] - tum bolgelerin pencereli toplamı.

    Not: ripple sin tabanli oldugu icin +/- (tepe ve cukur) uretir; bu normaldir.
    import_heightmap oncesi cagiran taraf min'i 0'a otelemeli (zMin).
    """
    z = 0.0
    for (cx, cy, R, amp, wl) in zones:
        d = math.hypot(x - cx, y - cy)
        if d < R:
            w = _window(d, R)
            ripple = 0.5 * (math.sin(2.0 * math.pi * x / wl) +
                            math.sin(2.0 * math.pi * y / wl))
            z += amp * w * ripple
    return z


def roughness_severity(x, y, zones):
    """MPPI (x, y) noktasindaki purefsuzluk siddeti [>=0] - yumusak maliyet icin.

    Siddet = pencere * amplitude / wavelength  (egim proxy'si: yuksek+sik tumsek = kotu).
    Ust uste binen bolgelerde en kotusu (max) alinir. Disarida 0.
    """
    s = 0.0
    for (cx, cy, R, amp, wl) in zones:
        d = math.hypot(x - cx, y - cy)
        if d < R:
            s = max(s, _window(d, R) * amp / wl)
    return s
