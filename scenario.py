"""
scenario.py - Tek kaynak: engel ve engebe (roughness) yerlesimleri (MPPI cercevesi, metre).

Daha once config.json icinde duran OBSTACLES / ROUGHNESS dizileri artik burada.
generate_costmaps.py bu veriden IKI costmap uretir (obstacle_costmap.csv,
roughness_costmap.csv) + data/obstacles.json (C++ CBF ve BeamNG spawn icin ham
liste). beamng_bridge.py bu modulu dogrudan import eder.
"""

# HARD engeller: [x, y, r] cember, ya da [x, y, w, h] dikdortgen
# (dikdortgen -> cevreleyen cembere donusturulur, bkz. obstacles.py).
OBSTACLES = [
    [4.0, 2.5, 1.5],
    [11.0, -2.5, 1.5],
    [18.0, 2.5, 1.5],
]

# SOFT engebe (roughness) bolgeleri: [x, y, radius, amplitude, wavelength]
ROUGHNESS = [
    [0.0, 30.0, 8.0, 0.5, 5.0],
    [-10.0, 0.0, 6.0, 0.4, 4.0],
]
