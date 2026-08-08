# Competitors — MPPI speed benchmark

Other MPPI implementations run inside OUR BeamNG harness (same track, vehicle,
obstacles, metrics) so we can compare computation speed against our C++/CUDA MPPI.
Each competitor plugs into the SAME UDP protocol `MppiCpp.exe` uses:

    recv  StatePacket   "dddddi" = time, x, y, yaw, v, valid            (from beamng_bridge.py)
    send  ControlPacket "ddddi"  = time, steer, accel, solve_ms, reset  (to beamng_bridge.py)

Each controller times its OWN `calc_control_input` (pure compute, no UDP/BeamNG overhead)
and returns it as `solve_ms` in the control packet. The bridge writes it into that MPPI's
single `run_log_<controller>.csv` (the `solve_ms` column) — no separate timing file. Params
are matched to `config.json` so both solve the identical problem; only the implementation differs.

### Where to take the speed number from

`../bench_solve.py` drives the SAME controllers around the SAME track with a headless
kinematic plant (no simulator) and writes `runs/bench_log_<controller>.csv`. **Use those
numbers.** In BeamNG's `step` mode every control step forces a rendered frame, so a CUDA
controller gets serviced roughly once per 60 Hz frame — our solve reads ~16 ms instead of
~2 ms, while CPU-based competitors are barely affected (measured inflation: C++ 7.0x,
JAX 1.34x, Python 1.08x). BeamNG runs stay useful for real vehicle dynamics and visual
verification, not for timing.

## #1 — MizuhoAOKI/python_simple_mppi  (the reference our C++ is a port of)

The cloned repo is git-ignored (external, ~15 MB). Reproduce with:

```bash
git clone --depth 1 https://github.com/MizuhoAOKI/python_simple_mppi.git \
    competitors/python_simple_mppi
```

Then the wrapper `run_python_mppi.py` imports its `MPPIControllerForPathTracking`
(pure NumPy, unchanged) and drives it from BeamNG. Early measurement: ~4.7 s/solve
at K=1000, T=20 (naive Python K×T double loop) vs our C++/CUDA at ~1-2 ms.

## #2 — jlehtomaa/jax-mppi  (Williams et al. 2017 MPPI in JAX)

Cloned repo (git-ignored):
```bash
git clone --depth 1 https://github.com/jlehtomaa/jax-mppi.git competitors/jax_mppi
```
`run_jax_mppi.py` imports its `MPPI` unchanged and supplies our model+cost as its
`step_fn` (jax.lax.scan + vmap + jit). JAX picks whatever backend exists; on native
Windows jaxlib is CPU-only (GPU needs WSL2), so it is reported as XLA-CPU.

## #3 — UM-ARM-Lab/pytorch_mppi  (Williams et al. 2017 MPPI in PyTorch)

```bash
pip install pytorch-mppi
```
MIT. No dedicated paper; the authors ask to cite RUMI (Zhong, Fazeli, Berenson,
IEEE T-RO 2025, arXiv:2408.10450). `run_torch_mppi.py` passes our `dynamics` and
`running_cost` (plus `terminal_state_cost`) to its `MPPI` unchanged. `noise_sigma` is
taken as a covariance, so config's sigma goes in directly. Device via `MPPI_DEVICE`
(cuda|cpu); with a CPU-only torch build it falls back to CPU automatically.

## Structure

`_harness.py` holds everything the three wrappers share: config / reference-path /
obstacle loading, the UDP protocol, the forward-200 nearest-waypoint tracking,
end-of-path detection, solve timing and the summary. Each `run_*.py` therefore contains
ONLY what is specific to that library - building its controller and one `solve()`
callback. That keeps "what we added" small and visible:

| file | lines | library-specific content |
|---|---|---|
| `_harness.py` | 132 | shared plumbing (no algorithm) |
| `run_python_mppi.py` | 59 | constructor args mapped from our config |
| `run_jax_mppi.py` | 142 | model + cost as a JAX `step_fn`, env shim, gym stub |
| `run_torch_mppi.py` | 149 | model + cost as torch `dynamics`/`running_cost`, device, lambda note |

## Fairness rules applied to every competitor

* The competitor's controller code is NEVER edited (gym is stubbed for jax-mppi only
  because its demo import pulls it in).
* All read OUR `config.json`: same K, T, dt, wheelbase, sigma, lambda, cost weights,
  steer/accel limits, path and obstacles.
* Same problem definition: kinematic bicycle, forward-200 waypoint search, 9-point
  footprint HARD collision penalty. Our CBF soft barrier is deliberately NOT given to
  them (and is currently disabled in config for us too, so all three are identical).
* Only `calc_control_input` / `get_action` / `command` is timed (pure compute).
