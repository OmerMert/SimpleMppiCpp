"""Shared obstacle cost model.

Turns scenario.py's OBSTACLES into circles (rectangles become their circumscribing
circle, as mppi_core.cu compute_cbf_cost does) and reproduces the same CBF falloff,
exp(-decay_rate * h) with a hard cost inside the surface, so the generated costmap stays
consistent with what the C++ CBF actually sees.

This costmap does not replace the analytic CBF in mppi_core.cu; it is an additional soft
layer on top of it (config.json OBSTACLE_COSTMAP_WEIGHT, 0 by default). The CBF rotates
the car's nine body points by yaw, whereas this module only evaluates a single (x, y)
point - enough for a coarse guidance signal, not for collision checking.
"""
import math


def load_obstacle_circles(raw_list):
    """[x,y,r] or [x,y,w,h] -> [(x,y,r), ...], rectangles via their circumscribing circle."""
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
    """Point-based version of mppi_core.cu compute_cbf_cost.

    h is the distance to the obstacle surface. Inside the circle (h <= 0) the cost is a
    flat 1e6 - smaller than the CBF's 1e9 so the CSV and the preview stay readable. Within
    the influence radius it is cbf_weight * exp(-decay_rate * h) for the nearest obstacle,
    and zero beyond.
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
