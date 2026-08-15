"""Competitor #3: UM-ARM-Lab/pytorch_mppi (Williams et al. 2017 MPPI in PyTorch).

The competitor's controller is left unmodified; it is only given our model and cost:
kinematic bicycle, the same stage and terminal weights, the forward-200 waypoint search
and the hard collision penalty that mirrors the reference's _is_collided. Our soft CBF
barrier is deliberately withheld.

Device comes from MPPI_DEVICE=cuda|cpu, defaulting to cuda when it is available. Shared
plumbing is in _harness.py.
"""
import os

import numpy as np
import torch
from pytorch_mppi import MPPI

import _harness as H

DTYPE = torch.float32

# pytorch_mppi has no `alpha`: lambda_ is both the softmax temperature and the control
# cost weight (action_cost = lambda_ * noise @ Sigma^-1, so gamma = lambda). Ours and the
# reference use gamma = lambda*(1-alpha) = 100*(1-0.98) = 2. Passing the config's 100
# straight through leaves the control cost 50x too heavy; measured, that caps mean accel
# at +0.41 against a limit of 2.5 and the car never gets up to speed, versus 2.29 m/s at
# lambda=2. This is a parameterisation difference, not a flaw in the implementation, and
# the timing metric is unaffected by lambda. Use MPPI_LAMBDA=100 for a literal-config run.
LAMBDA_MATCHED_GAMMA = 2.0


def build_problem(cfg, ref_path, circles, device):
    """Our model and cost, packaged as the functions pytorch_mppi expects.

    State is [x, y, yaw, v, wp_idx] (nx=5). wp_idx carries the windowed search forward
    through the rollout, the same way the C++ and the reference carry prev_idx.
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
        """Equivalent of the reference's _is_collided * 1e10 over the 9 body points. No CBF."""
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
        """Kinematic bicycle, identical to the C++ update_state_gpu."""
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
        print("[TorchMPPI] WARNING: no CUDA (CPU-only torch build), falling back to cpu.")
        want = "cpu"
    device = torch.device(want)
    on_gpu = device.type == "cuda"

    dynamics, running_cost, terminal_cost = build_problem(cfg, ref_path, circles, device)
    max_steer, max_accel = float(cfg["max_steer_abs"]), float(cfg["max_accel_abs"])
    lam = float(os.environ.get("MPPI_LAMBDA", LAMBDA_MATCHED_GAMMA))

    ctrl = MPPI(
        dynamics=dynamics, running_cost=running_cost, nx=5,
        # pytorch_mppi treats noise_sigma as a covariance, so config's sigma goes in as-is
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
            torch.cuda.synchronize()      # kernels are async; wait so timing is real
        a = act.detach().cpu().numpy()
        return float(a[0]), float(a[1])

    # warm-up, so allocation and compilation stay out of the benchmark
    solve(0.0, 0.0, 0.0, 0.0, 0)
    ctrl.reset()

    our_gamma = float(cfg["param_lambda"]) * (1 - float(cfg["param_alpha"]))
    H.serve("TorchMPPI", cfg, ref_path, solve, *H.ports(), banner=(
        "Competitor: pytorch_mppi (UM-ARM-Lab) - Williams et al. 2017, PyTorch",
        f"torch {torch.__version__} | device={device}"
        + (f" ({torch.cuda.get_device_name(0)})" if on_gpu else " (CPU)"),
        f"K={ctrl.K} T={ctrl.T} dt={cfg['delta_t']} L={cfg['wheel_base']} "
        f"| obstacles={len(circles)} | ref_path {ref_path.shape}",
        f"lambda={lam:g} -> gamma={lam:g} (ours: gamma={our_gamma:g})",
    ))


if __name__ == "__main__":
    main()
