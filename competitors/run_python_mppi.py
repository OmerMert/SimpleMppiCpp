"""Competitor #1: MizuhoAOKI/python_simple_mppi, the reference our C++ was ported from.

Pure NumPy with a K x T double loop rather than a vectorised one, run as published. The
shared plumbing (UDP, path and obstacle loading, timing) is in _harness.py; all that is
left here is constructing the reference MPPI with parameters matched to our config.
"""
import numpy as np

import _harness as H

# The reference MPPI class, unmodified. Its module imports Vehicle from
# pathtracking_kbm_obav at the top, but only the repo's own demo uses that.
import sys, os                                                        # noqa: E402
sys.path.insert(0, os.path.join(H.HERE, "python_simple_mppi", "scripts"))
from mppi_pathtracking_obav import MPPIControllerForPathTracking      # noqa: E402


def main():
    cfg, ref_path, circles = H.load_setup()
    vw, vl = H.footprint(cfg)

    mppi = MPPIControllerForPathTracking(
        delta_t=cfg["delta_t"],
        wheel_base=cfg["wheel_base"],
        vehicle_width=vw,
        vehicle_length=vl,
        max_steer_abs=cfg["max_steer_abs"],
        max_accel_abs=cfg["max_accel_abs"],
        ref_path=ref_path,
        horizon_step_T=cfg["horizon_step_T"],
        number_of_samples_K=cfg["number_of_samples_K"],
        param_exploration=cfg["param_exploration"],
        param_lambda=cfg["param_lambda"],
        param_alpha=cfg["param_alpha"],
        sigma=np.array(cfg["sigma"], dtype=float),
        stage_cost_weight=np.array(cfg["stage_cost_weight"], dtype=float),
        terminal_cost_weight=np.array(cfg["terminal_cost_weight"], dtype=float),
        visualize_optimal_traj=False,      # drawing off, so only solve time is measured
        visualze_sampled_trajs=False,
        obstacle_circles=circles,
        collision_safety_margin_rate=1.0,  # the scaling is already applied in H.footprint
    )

    def solve(x, y, yaw, v, idx):
        # The reference does its own nearest-waypoint search, so idx is unused, and it
        # raises IndexError at the end of the path, which the harness reads as "finished".
        u0, _, _, _ = mppi.calc_control_input(np.array([x, y, yaw, v], dtype=float))
        return float(u0[0]), float(u0[1])

    H.serve("PyMPPI", cfg, ref_path, solve, *H.ports(), banner=(
        "Competitor: python_simple_mppi (MizuhoAOKI) - pure NumPy",
        f"K={mppi.K} T={mppi.T} dt={mppi.delta_t} L={mppi.wheel_base} "
        f"| obstacles={len(circles)} | ref_path {ref_path.shape}",
    ))


if __name__ == "__main__":
    main()
