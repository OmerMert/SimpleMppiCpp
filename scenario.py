"""
scenario.py - Tek kaynak: engel yerlesimleri (MPPI cercevesi, metre).

Daha once config.json icinde duran OBSTACLES dizisi artik burada.
generate_costmaps.py bu veriden obstacle_costmap.csv + data/obstacles.json (C++ CBF ve
BeamNG spawn icin ham liste) uretir. beamng_bridge.py bu modulu dogrudan import eder.
"""

# HARD engeller: [x, y, r] cember, ya da [x, y, w, h] dikdortgen
# (dikdortgen -> cevreleyen cembere donusturulur, bkz. obstacles.py).
#
# Yerlesim: HER IKI duz seritte de donusumlu slalom (arac aralarindan slalom yapar).
# Alt serit y=0 (arac +x yonunde gider), ust serit y=30 (arac -x yonunde doner).
#
# Ofset secimi (MANEVRAYI ZORUNLU KILAR): merkez +/-2.2 m, r=1.5 -> engel kenari yol
# eksenine 0.7 m. Arac yari-genisligi 1.9/2 = 0.95 m oldugundan duz gitmek CARPISMADIR
# (0.95 > 0.7); arac en az ~0.25 m yana kaymak ZORUNDA (+ CBF payi).
# (Onceki +/-2.5 ofsette kenar 1.0 m idi -> arac dumduz gecip siyiriyordu.)
OBSTACLES = [
    # --- alt serit (gidis) ---
    [11.0,  2.2, 1.5],   # ustte
    [16.5, -2.2, 1.5],   # altta
    # --- ust serit (donus, arac -x yonunde ilerler) ---
    # NOT: viraj cikisi x=+16.1. Engeller ONCE x=9'daydi -> cikisa sadece 7 m; alt seritte
    # ise 27 m serbest mesafe var. Bu asimetri, virajdan genis cikan bir kontrolcuyu
    # (JAX) toparlanamadan engele sokup v=0 kilidine dusuruyordu. 6 m kaydirildi -> 13 m.
    [3.0,  32.2, 1.5],   # disarida (yolun ustunde)
    [-4.0,  27.8, 1.5],  # iceride (yolun altinda)
    [-11.0, 32.2, 1.5],  # disarida
]
