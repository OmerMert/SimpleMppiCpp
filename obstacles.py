"""
obstacles.py - Paylasilan engel maliyet modeli.

scenario.py OBSTACLES -> cemberler (dikdortgen -> cevreleyen cember, mppi_core.cu
compute_cbf_cost'taki ile ayni yaklasim). obstacle_cost() formulu de ayni CBF
falloff'unu (exp(-decay_rate*h), yuzeyden h<0 -> sert maliyet) tekrar eder ki
uretilen costmap, C++ CBF'nin gordugu maliyetle GORSEL/kavramsal olarak tutarli
olsun.

ONEMLI: bu costmap MPPI'daki analitik CBF'nin (footprint+yaw farkinda, mppi_core.cu
compute_cbf_cost) YERINE GECMEZ - ona EK bir yumusak maliyet katmanidir (bkz.
config.json OBSTACLE_COSTMAP_WEIGHT, varsayilan 0 = kapali). CBF nokta-bazli degil,
aracin 9 govde noktasini yaw'a gore donduruyor; bu modul ise sadece (x,y) nokta
yaklasimi kullanir - kucuk bir yardimci sinyal icin yeterli, hassas carpisma
kontrolu icin degil.
"""
import math


def load_obstacle_circles(raw_list):
    """[x,y,r] veya [x,y,w,h] -> [(x,y,r), ...] (dikdortgen -> cevreleyen cember)."""
    circles = []
    for o in raw_list:
        if len(o) == 3:
            circles.append((float(o[0]), float(o[1]), float(o[2])))
        elif len(o) == 4:
            w, h = float(o[2]), float(o[3])
            r = 0.5 * math.hypot(w, h)
            circles.append((float(o[0]), float(o[1]), r))
    return circles


def obstacle_cost(x, y, circles, influence_radius, cbf_weight, decay_rate):
    """(x,y) noktasindaki engel maliyeti - mppi_core.cu compute_cbf_cost'un nokta-bazli hali.

    h = engel YUZEYINE mesafe. h<=0 (cember icinde) -> sabit sert maliyet (1e6,
    CSV/onizlemede okunakli kalsin diye CBF'nin 1e9'undan kucuk). 0 < h < influence_radius
    -> cbf_weight * exp(-decay_rate*h) (en kotu/en yakin engel gecerli). Disarida 0.
    """
    cost = 0.0
    for (cx, cy, r) in circles:
        d = math.hypot(x - cx, y - cy)
        h = d - r
        if h <= 0.0:
            return 1e6
        if h < influence_radius:
            cost = max(cost, cbf_weight * math.exp(-decay_rate * h))
    return cost
