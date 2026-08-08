"""
Rakip #2: jlehtomaa/jax-mppi (Williams et al. 2017 MPPI, JAX: scan + vmap + jit).

Rakibin kontrolcusu DEGISTIRILMEZ; ona sadece bizim model + maliyetimiz verilir:
kinematik bisiklet, ayni asama/terminal agirliklari, ileri-200 pencereli waypoint
aramasi ve SERT carpisma cezasi (referanstaki _is_collided'in karsiligi). Bizim CBF
yumusak bariyerimiz BILEREK verilmez.

Backend: JAX hangi cihaz varsa orada kosar. Native Windows'ta jaxlib CPU-only'dir
(GPU icin WSL2) -> "XLA-CPU" olarak raporlanir; kod degismeden GPU'da da kosar.
Ortak iskelet (UDP, yol/engel yukleme, sure olcumu) _harness.py'de.
"""
import sys
import types

import jax
import jax.numpy as jnp
import numpy as np

import _harness as H

sys.path.insert(0, f"{H.HERE}/jax_mppi")

# jax_mppi.controllers.__init__ CEMPPI'yi de ceker, o da utils -> `gym`. gym yalnizca
# repo'nun pendulum demosu icin lazim; biz kendi modelimizi kullandigimiz icin hic
# cagrilmaz. Rakibin KODUNA DOKUNMAMAK icin gym'i bos bir modulle stub'liyoruz.
sys.modules.setdefault("gym", types.ModuleType("gym"))

from jax_mppi.controllers import MPPI                 # noqa: E402
from jax_mppi.rollout import build_rollout_fn         # noqa: E402


class _EnvShim:
    """jax-mppi Controller sadece action_space.shape/high/low ister (gym'e gerek yok)."""
    def __init__(self, max_steer, max_accel):
        self.action_space = types.SimpleNamespace(
            low=np.array([-max_steer, -max_accel]),
            high=np.array([max_steer, max_accel]),
            shape=(2,))


def build_step_fn(cfg, ref_path, circles):
    """Bizim model + maliyet -> jax-mppi'nin bekledigi step fonksiyonu.

    Durum: [x, y, yaw, v, wp_idx, t]. wp_idx pencereli aramayi rollout boyunca
    ilerletir; t terminal maliyeti son adimda eklemek icin sayac.
    """
    dt, L = float(cfg["delta_t"]), float(cfg["wheel_base"])
    T = int(cfg["horizon_step_T"])
    w = jnp.asarray(cfg["stage_cost_weight"], dtype=jnp.float32)
    wT = jnp.asarray(cfg["terminal_cost_weight"], dtype=jnp.float32)

    path = jnp.asarray(ref_path, dtype=jnp.float32)      # (N,4)
    path_xy = path[:, :2]
    max_start = max(path.shape[0] - H.SEARCH_FWD, 0)

    has_obs = len(circles) > 0
    if has_obs:
        obs_xy = jnp.asarray(circles[:, :2], dtype=jnp.float32)
        obs_r = jnp.asarray(circles[:, 2], dtype=jnp.float32)

    vw, vl = H.footprint(cfg)
    body_x = 0.5 * vl * jnp.array([-1., -1., -1., 0., 0., 0., 1., 1., 1.], dtype=jnp.float32)
    body_y = 0.5 * vw * jnp.array([-1., 0., 1., 1., -1., 0., 1., 0., -1.], dtype=jnp.float32)

    def nearest_idx(idx, x, y):
        start = jnp.clip(idx, 0, max_start).astype(jnp.int32)
        seg = jax.lax.dynamic_slice(path_xy, (start, 0), (H.SEARCH_FWD, 2))
        d2 = (seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2
        return start + jnp.argmin(d2).astype(jnp.int32)

    def collision_cost(x, y, yaw):
        """Referansin _is_collided * 1e10 karsiligi (9 govde noktasi). CBF YOK."""
        if not has_obs:
            return 0.0
        c, s = jnp.cos(yaw), jnp.sin(yaw)
        gpx = x + body_x * c - body_y * s
        gpy = y + body_x * s + body_y * c
        d = jnp.sqrt((gpx[:, None] - obs_xy[None, :, 0]) ** 2 +
                     (gpy[:, None] - obs_xy[None, :, 1]) ** 2)
        return jnp.sum(jnp.where(jnp.min(d - obs_r[None, :], axis=0) <= 0.0, 1e9, 0.0))

    def track_cost(weights, x, y, yaw, v, idx):
        rx, ry, ryaw, rv = path[idx, 0], path[idx, 1], path[idx, 2], path[idx, 3]
        dyaw = jnp.arctan2(jnp.sin(yaw - ryaw), jnp.cos(yaw - ryaw))
        return (weights[0] * (x - rx) ** 2 + weights[1] * (y - ry) ** 2 +
                weights[2] * dyaw ** 2 + weights[3] * (v - rv) ** 2)

    def step(state, act):
        x, y, yaw, v, idx_f, t = state
        steer, accel = act[0], act[1]
        # kinematik bisiklet - C++ update_state_gpu ile birebir
        nx = x + v * jnp.cos(yaw) * dt
        ny = y + v * jnp.sin(yaw) * dt
        nyaw = yaw + v / L * jnp.tan(steer) * dt
        nv = v + accel * dt
        nidx = nearest_idx(idx_f.astype(jnp.int32), nx, ny)
        cost = track_cost(w, nx, ny, nyaw, nv, nidx) + collision_cost(nx, ny, nyaw)
        cost += jnp.where(t >= T - 1, track_cost(wT, nx, ny, nyaw, nv, nidx), 0.0)
        return jnp.array([nx, ny, nyaw, nv, nidx.astype(jnp.float32), t + 1.0]), cost

    def wrapper_step(carry, action):
        next_state, cost = step(carry[0], action)
        return (next_state,), (next_state, cost)

    return wrapper_step


def main():
    cfg, ref_path, circles = H.load_setup()

    sigma = np.array(cfg["sigma"], dtype=float)
    ctrl_cfg = {
        "seed": 42,
        "horizon": int(cfg["horizon_step_T"]),
        "n_samples": int(cfg["number_of_samples_K"]),
        # jax-mppi: w ~ exp(temperature*(min-S)) | bizim: exp(-(S-rho)/lambda)
        "temperature": 1.0 / float(cfg["param_lambda"]),
        # jax-mppi gurultuyu STD ile carpar; bizim sigma KOVARYANS -> std = sqrt(diag)
        "noise_sigma": np.sqrt(np.diag(sigma)),
    }
    ctrl = MPPI(_EnvShim(float(cfg["max_steer_abs"]), float(cfg["max_accel_abs"])), ctrl_cfg)
    ctrl.rollout_fn = build_rollout_fn(build_step_fn(cfg, ref_path, circles))

    def solve(x, y, yaw, v, idx):
        act = ctrl.get_action(np.array([x, y, yaw, v, float(idx), 0.0]))
        a = np.asarray(jax.block_until_ready(act))   # async dispatch -> dogru sure olcumu
        return float(a[0]), float(a[1])

    solve(0.0, 0.0, 0.0, 0.0, 0)   # JIT isinmasi (benchmark'a girmesin)
    ctrl.reset()

    H.serve("JaxMPPI", cfg, ref_path, solve, *H.ports(), banner=(
        "Rakip: jax-mppi (jlehtomaa) - Williams et al. 2017, JAX",
        f"JAX {jax.__version__} | backend={jax.default_backend()} | {jax.devices()}",
        f"K={ctrl_cfg['n_samples']} T={ctrl_cfg['horizon']} dt={cfg['delta_t']} "
        f"L={cfg['wheel_base']} | engel={len(circles)} | ref_path {ref_path.shape}",
    ))


if __name__ == "__main__":
    main()
