"""
scenario.py - Tek kaynak: engel yerlesimleri (MPPI cercevesi, metre).

Daha once config.json icinde duran OBSTACLES dizisi artik burada.
generate_costmaps.py bu veriden obstacle_costmap.csv + data/obstacles.json (C++ CBF ve
BeamNG spawn icin ham liste) uretir. beamng_bridge.py bu modulu dogrudan import eder.
"""

# HARD engeller: [x, y, r] cember, ya da [x, y, w, h] dikdortgen
# (dikdortgen -> cevreleyen cembere donusturulur, bkz. obstacles.py).
OBSTACLES = [
    [4.0, 2.5, 1.5],
    [11.0, -2.5, 1.5],
    [18.0, 2.5, 1.5],
]
