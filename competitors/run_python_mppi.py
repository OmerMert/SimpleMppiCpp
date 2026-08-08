"""
Rakip #1: MizuhoAOKI/python_simple_mppi - bizim C++'in port edildigi ASIL referans.
Saf NumPy, K x T cift dongusu (vektorize DEGIL); OLDUGU GIBI kosturulur.

Ortak iskelet (UDP, yol/engel yukleme, sure olcumu) _harness.py'de.
Burada SADECE: referans MPPI'yi bizim config'e esitlenmis parametrelerle kurmak.
"""
import numpy as np

import _harness as H

# Referans MPPI sinifi (kodu DEGISTIRILMEDI). Modul tepesinde pathtracking_kbm_obav'dan
# Vehicle cekiliyor ama MPPI sinifi onu kullanmiyor - sadece repo'nun kendi demosu kullanir.
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
        visualize_optimal_traj=False,      # gorsel kapali -> saf solve zamani olculur
        visualze_sampled_trajs=False,
        obstacle_circles=circles,
        collision_safety_margin_rate=1.0,  # olcek H.footprint'te uygulandi
    )

    def solve(x, y, yaw, v, idx):
        # Referans kendi en yakin waypoint aramasini icerde yapar (idx kullanilmaz);
        # yol sonunda IndexError firlatir -> iskelet bunu "tamamlandi" olarak alir.
        u0, _, _, _ = mppi.calc_control_input(np.array([x, y, yaw, v], dtype=float))
        return float(u0[0]), float(u0[1])

    H.serve("PyMPPI", cfg, ref_path, solve, *H.ports(), banner=(
        "Rakip: python_simple_mppi (MizuhoAOKI) - saf NumPy",
        f"K={mppi.K} T={mppi.T} dt={mppi.delta_t} L={mppi.wheel_base} "
        f"| engel={len(circles)} | ref_path {ref_path.shape}",
    ))


if __name__ == "__main__":
    main()
