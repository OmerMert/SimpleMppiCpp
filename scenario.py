"""Obstacle layout in the MPPI frame, in metres. Single source for every consumer.

generate_costmaps.py turns this into data/obstacle_costmap.csv and data/obstacles.json
(the raw list the C++ CBF and the BeamNG spawn need). beamng_bridge.py imports it directly.
"""

# Entries are circles [x, y, r] or rectangles [x, y, w, h]; obstacles.py converts
# rectangles to their circumscribing circle.
#
# Alternating slalom on both straights: the lower one at y=0 (car travels +x) and the
# upper one at y=30 (car travels -x).
#
# The lateral offset is what forces the manoeuvre. At centre +/-2.2 m with r=1.5 the
# obstacle edge sits 0.7 m from the path axis while the car's half-width is 0.95 m, so
# driving straight through collides and the car must move at least ~0.25 m aside, plus
# the CBF margin. An earlier +/-2.5 m offset left a 1.0 m gap and the car passed without
# steering at all.
OBSTACLES = [
    # lower straight (outbound)
    [11.0,  2.2, 1.5],   # above the axis
    [16.5, -2.2, 1.5],   # below
    # upper straight (return leg, car travels -x). The corner exit is at x=+16.1, so
    # these sit 13 m past it. At their original x=9 only 7 m were left, against 27 m on
    # the lower straight; that asymmetry pushed a controller running wide out of the
    # corner (JAX) into the obstacle before it could recover, stalling it at v=0.
    [3.0,  32.2, 1.5],   # outside (above the path)
    [-4.0,  27.8, 1.5],  # inside (below the path)
    [-11.0, 32.2, 1.5],  # outside
]
