"""Obstacle and terrain layout in the MPPI frame, in metres. Single source for every consumer.

generate_costmaps.py turns this into data/obstacle_costmap.csv and data/obstacles.json
(the raw list the C++ CBF and the BeamNG spawn need). beamng_bridge.py imports it directly.
"""
import json
import os

# Entries are circles [x, y, r] or rectangles [x, y, w, h]; obstacles.py converts
# rectangles to their circumscribing circle.
#
# All six are circles of r=1.5 m, the size that worked on the old oval. Rectangles were
# tried and dropped: a 3x2 m box is planned around as a 1.80 m radius disc, i.e. 3.6 m
# wide against a prediction horizon that only reaches T*dt*v = 2.5 m at the reference
# speed. pytorch_mppi duly deadlocked 2.36 m short of the first box and sat there for
# 579 s with v=0 - and its solve times were fine at the time (median 32.9 ms, 2.4% over
# the 50 ms budget), so it was a cost-landscape trap, not a timing failure: once the
# obstacle enters the horizon there is no collision-free rollout left and stopping
# becomes the cheapest action.
#
# Laid out along data/figure8.csv (236.0 m, min turn radius 12.8 m), alternating sides
# so the car has to swap direction repeatedly.
#
# The path was widened from a=30 (157 m, R_min 9.5 m) to a=45 after jax-mppi failed to
# track the tighter version at all: it was already 1.76 m off the reference at the FIRST
# obstacle and deadlocked at the second, in PAUSED mode, so nothing to do with solve
# time. The figure eight is curving everywhere - unlike the old oval it has no straights
# to recover on - and 9.5 m turns left jax permanently wide. R_min 12.8 m is close to the
# oval's ~15 m, where jax tracked at 0.503 m mean deviation.
#
# The lateral offset is what forces the manoeuvre, and on a curved track it cannot be a
# constant. A 4.5 m long body tangent to a radius-R curve sweeps its outer corner past
# the centreline by
#     W_eff(R) = sqrt((R + W/2)^2 + (L/2)^2) - R
# which is 1.19 m at R=9.5 m against 0.95 m on a straight. The first layout used the flat
# 0.70 m clearance from the old oval and the car duly clipped two obstacles (-0.07 m on
# the circle at s=15 m, -0.04 m on the box at s=75 m). Offsets are now
#     r_eff + W_eff(R_local) - 0.25
# so the forced lateral escape stays 0.25 m everywhere, curvature included. On a straight
# this reduces exactly to the old r_eff + 0.70.
#
# Which SIDE an obstacle goes on is not free either. jax-mppi takes bends from the
# outside - measured here at 0.354 m to the right of the reference through left-hand
# turns - so an obstacle on the outside sits exactly where it drifts. Four of the six
# were outside in the first a=45 layout and jax deadlocked at the first of them (s=42 m)
# even though its tracking was fine by then (0.357 m mean deviation). Every obstacle is
# now placed on the INSIDE of its local bend, so running wide moves the car away from
# the obstacle and the forced escape is in the direction the vehicle already wants to go.
# Sides still alternate over a lap because the figure eight reverses curvature at each
# crossing.
#
# Every entry was also checked against the WHOLE path, not just its own pass: on a
# self-crossing track an obstacle can sit near one branch and block the other. All six
# clear that test, and none is closer than 17 m of arc to the crossings at s=59 / s=177 m.
OBSTACLES_FIGURE8 = [
    [13.23, 13.63, 1.5],          # s=22 m,  inside (left)
    [9.31, 30.93, 1.5],           # s=42 m,  inside (left)
    [-10.84, 62.34, 1.5],         # s=80 m,  inside (right)
    [-6.16, 85.91, 1.5],          # s=110 m, inside (right)
    [13.44, 76.27, 1.5],          # s=140 m, inside (right)
    [-11.72, 26.04, 1.5],         # s=200 m, inside (left)
]

# The oval scenario of the TOK 2026 paper: five r=1.5 m circles, alternating left/right of
# the centreline along the two straights so the car has to slalom. These are the exact
# positions every run in runs/ (run_log_*.csv) and Table 1's "Oval yol" block were produced
# with - do not re-derive them, they are the published layout.
OBSTACLES_OVAL = [
    [11.0, 2.2, 1.5],
    [16.5, -2.2, 1.5],
    [3.0, 32.2, 1.5],
    [-4.0, 27.8, 1.5],
    [-11.0, 32.2, 1.5],
]

# Which set is live follows config.json's REF_PATH_FILE, so the path and the obstacles can
# never disagree: switching the scenario is ONE edit (REF_PATH_FILE) and everything that
# imports OBSTACLES - generate_costmaps.py, beamng_bridge.py, competitors/_harness.py -
# follows. Scripts that always mean one specific scenario (paper/make_*_figs.py) import the
# named list instead, so they cannot be silently redirected by the switch.
_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
with open(_CFG, "r") as _f:
    _REF = json.load(_f).get("REF_PATH_FILE", "")
OBSTACLES = OBSTACLES_FIGURE8 if "figure8" in _REF else OBSTACLES_OVAL

# Uneven-terrain zones: [x, y, radius, amplitude, wavelength], all metres.
# roughness.py turns these into the BeamNG heightmap (import_heightmap) that the bridge
# loads after spawn calibration.
#
# IMPORTANT - these enter the SIMULATOR ONLY, never a cost function. The four compared
# controllers keep the identical cost they use on flat ground, so the comparison stays
# apples-to-apples; the terrain acts purely as a disturbance that widens the gap between
# the kinematic model and the soft-body vehicle. (The offroad branch also feeds roughness
# into the MPPI stage cost - deliberately NOT done here, see Log/16.)
#
# Placement: one zone over each lobe of the figure eight. The self-crossing at
# (0, 30) falls between the two zones and stays flat, so the hardest bit of geometry is
# not also the bumpiest and the two effects can be told apart.
# Amplitude is peak height; the ripple spans +/-amplitude and is windowed to zero at
# the zone edge, so there is no step (which would act as a wall).
# Wavelength is 10 m against the 1 m heightmap grid, i.e. ten samples per wave. At the
# earlier 5 m the terrain came out blocky rather than wavy; the grid has to stay at 1 m
# because the track now reaches MPPI y=90 and the extent is 0.5 * 256 * resolution.
ROUGHNESS = [
    [0.0, 22.5, 19.0, 0.25, 10.0],   # lower lobe
    [0.0, 67.5, 19.0, 0.25, 10.0],   # upper lobe
]
