"""
Rakip #3: UM-ARM-Lab/pytorch_mppi (Williams et al. 2017 MPPI, PyTorch).

Rakibin kontrolcusu DEGISTIRILMEZ; ona sadece bizim model + maliyetimiz verilir:
kinematik bisiklet, ayni asama/terminal agirliklari, ileri-200 pencereli waypoint
aramasi ve SERT carpisma cezasi (referanstaki _is_collided'in karsiligi). Bizim CBF
yumusak bariyerimiz BILEREK verilmez.

Cihaz: MPPI_DEVICE=cuda|cpu (varsayilan: CUDA varsa cuda, yoksa cpu).
Ortak iskelet (UDP, yol/engel yukleme, sure olcumu) _harness.py'de.
"""
import os

import numpy as np
import torch
from pytorch_mppi import MPPI

import _harness as H

DTYPE = torch.float32

# pytorch_mppi'de `alpha` YOK; lambda_ hem softmax sicakligi HEM kontrol maliyeti
# agirligidir (action_cost = lambda_ * noise @ Sigma^-1, yani gamma = lambda). Bizde ve
# referansta gamma = lambda*(1-alpha) = 100*(1-0.98) = 2. Config'deki 100 aynen verilirse
# kontrol maliyeti 50x agir kalir: OLCULDU -> ivme ort +0.41 (limit 2.5), arac hizlanamiyor.
# lambda=2 ile ayni testte 2.29 m/s. Parametrelendirme farki, implementasyon kusuru degil;
# hiz metrigi lambda'dan etkilenmez. Birebir-config kosusu icin: MPPI_LAMBDA=100.
LAMBDA_MATCHED_GAMMA = 2.0


def build_problem(cfg, ref_path, circles, device):
    """Bizim model + maliyet -> pytorch_mppi'nin bekledigi fonksiyonlar.

    Durum: [x, y, yaw, v, wp_idx] (nx=5). wp_idx pencereli aramayi rollout boyunca
    ilerletir (C++ ve referans da prev_idx'i boyle tasir).
    """
    dt, L = float(cfg["delta_t"]), float(cfg["wheel_base"])
    w = torch.tensor(cfg["stage_cost_weight"], dtype=DTYPE, device=device)
    wT = torch.tensor(cfg["terminal_cost_weight"], dtype=DTYPE, device=device)

    path = torch.tensor(ref_path, dtype=DTYPE, device=device)      # (N,4)
    path_xy = path[:, :2]
    n_path = path.shape[0]
    win = torch.arange(H.SEARCH_FWD, device=device)

    has_obs = len(circles) > 0
    if has_obs:
        obs_xy = torch.tensor(circles[:, :2], dtype=DTYPE, device=device)
        obs_r = torch.tensor(circles[:, 2], dtype=DTYPE, device=device)

    vw, vl = H.footprint(cfg)
    body_x = 0.5 * vl * torch.tensor([-1., -1., -1., 0., 0., 0., 1., 1., 1.],
                                     dtype=DTYPE, device=device)
    body_y = 0.5 * vw * torch.tensor([-1., 0., 1., 1., -1., 0., 1., 0., -1.],
                                     dtype=DTYPE, device=device)

    def nearest_idx(idx, x, y):
        cand = (idx.unsqueeze(1).long() + win.unsqueeze(0)).clamp_(0, n_path - 1)
        pts = path_xy[cand]
        d2 = (pts[..., 0] - x.unsqueeze(1)) ** 2 + (pts[..., 1] - y.unsqueeze(1)) ** 2
        return torch.gather(cand, 1, d2.argmin(dim=1, keepdim=True)).squeeze(1)

    def collision_cost(x, y, yaw):
        """Referansin _is_collided * 1e10 karsiligi (9 govde noktasi). CBF YOK."""
        if not has_obs:
            return torch.zeros_like(x)
        c, s = torch.cos(yaw), torch.sin(yaw)
        gpx = x.unsqueeze(1) + body_x * c.unsqueeze(1) - body_y * s.unsqueeze(1)
        gpy = y.unsqueeze(1) + body_x * s.unsqueeze(1) + body_y * c.unsqueeze(1)
        d = torch.sqrt((gpx.unsqueeze(2) - obs_xy[:, 0]) ** 2 +
                       (gpy.unsqueeze(2) - obs_xy[:, 1]) ** 2)
        return torch.where((d - obs_r).amin(dim=1) <= 0.0, 1e9, 0.0).sum(dim=1)

    def track_cost(weights, x, y, yaw, v, idx):
        ref = path[idx]
        dyaw = torch.atan2(torch.sin(yaw - ref[:, 2]), torch.cos(yaw - ref[:, 2]))
        return (weights[0] * (x - ref[:, 0]) ** 2 + weights[1] * (y - ref[:, 1]) ** 2 +
                weights[2] * dyaw ** 2 + weights[3] * (v - ref[:, 3]) ** 2)

    def dynamics(state, action):
        """Kinematik bisiklet - C++ update_state_gpu ile birebir."""
        x, y, yaw, v, idx = state.unbind(dim=1)
        steer, accel = action[:, 0], action[:, 1]
        nx = x + v * torch.cos(yaw) * dt
        ny = y + v * torch.sin(yaw) * dt
        return torch.stack((nx, ny, yaw + v / L * torch.tan(steer) * dt, v + accel * dt,
                            nearest_idx(idx, nx, ny).to(DTYPE)), dim=1)

    def running_cost(state, action):
        x, y, yaw, v, idx = state.unbind(dim=1)
        return track_cost(w, x, y, yaw, v, idx.long()) + collision_cost(x, y, yaw)

    def terminal_cost(states, actions):
        x, y, yaw, v, idx = states[0, :, -1, :].unbind(dim=1)
        return track_cost(wT, x, y, yaw, v, idx.long())

    return dynamics, running_cost, terminal_cost


def main():
    cfg, ref_path, circles = H.load_setup()

    want = os.environ.get("MPPI_DEVICE", "cuda" if torch.cuda.is_available() else "cpu")
    if want == "cuda" and not torch.cuda.is_available():
        print("[TorchMPPI] UYARI: CUDA yok (CPU-only torch), cpu'ya dusuluyor.")
        want = "cpu"
    device = torch.device(want)
    on_gpu = device.type == "cuda"

    dynamics, running_cost, terminal_cost = build_problem(cfg, ref_path, circles, device)
    max_steer, max_accel = float(cfg["max_steer_abs"]), float(cfg["max_accel_abs"])
    lam = float(os.environ.get("MPPI_LAMBDA", LAMBDA_MATCHED_GAMMA))

    ctrl = MPPI(
        dynamics=dynamics, running_cost=running_cost, nx=5,
        # pytorch_mppi noise_sigma'yi KOVARYANS alir -> config'deki sigma dogrudan girer
        noise_sigma=torch.tensor(cfg["sigma"], dtype=DTYPE, device=device),
        num_samples=int(cfg["number_of_samples_K"]),
        horizon=int(cfg["horizon_step_T"]),
        lambda_=lam, device=device, terminal_state_cost=terminal_cost,
        u_min=torch.tensor([-max_steer, -max_accel], dtype=DTYPE, device=device),
        u_max=torch.tensor([max_steer, max_accel], dtype=DTYPE, device=device),
    )

    def solve(x, y, yaw, v, idx):
        act = ctrl.command(torch.tensor([x, y, yaw, v, float(idx)],
                                        dtype=DTYPE, device=device))
        if on_gpu:
            torch.cuda.synchronize()      # async kernel'leri bekle -> dogru sure olcumu
        a = act.detach().cpu().numpy()
        return float(a[0]), float(a[1])

    # isinma (tahsis/derleme benchmark'a girmesin)
    solve(0.0, 0.0, 0.0, 0.0, 0)
    ctrl.reset()

    our_gamma = float(cfg["param_lambda"]) * (1 - float(cfg["param_alpha"]))
    H.serve("TorchMPPI", cfg, ref_path, solve, *H.ports(), banner=(
        "Rakip: pytorch_mppi (UM-ARM-Lab) - Williams et al. 2017, PyTorch",
        f"torch {torch.__version__} | cihaz={device}"
        + (f" ({torch.cuda.get_device_name(0)})" if on_gpu else " (CPU)"),
        f"K={ctrl.K} T={ctrl.T} dt={cfg['delta_t']} L={cfg['wheel_base']} "
        f"| engel={len(circles)} | ref_path {ref_path.shape}",
        f"lambda={lam:g} -> gamma={lam:g} (bizim gamma={our_gamma:g})",
    ))


if __name__ == "__main__":
    main()
