# Solver-tuning benchmark

Multi-stage NMPC sweep for the maglev system: identifies a baseline cost
weighting (Q, R, terminal) plus a Pareto-optimal solver / integrator /
horizon configuration, separately for the **10-state** (yaw-projected)
and **12-state** (full) controllers. Both controllers run against a
shared 12-state plant compiled at `params.magnet.n = 30`.

## Running it

```matlab
cd experiments/solver_tuning
verify_setup;            % ~1 h, sanity checks (correction-factor n-independence,
                          %        build_dynamics signature, equilibrium check,
                          %        baseline closed-loop run)
runFullBenchmark();      % ~12 h, both models, all stages, resumable on crash
```

Then refresh the index and produce figures:

```matlab
dump_index('experiments/solver_tuning/results');
```

```bash
cd analysis
python plot_pareto.py  --results-dir ../results
python plot_heatmap.py --results-dir ../results
```

`runFullBenchmark` prints the exact `dump_index(...)` call to use when
it finishes.

## Stages

| Stage | Purpose | Runs / model |
|-------|---------|-------------|
| P-1   | Build & cache 12-state plant @ n=30 | 1 (shared) |
| P0    | Integrator pre-screen (IRK stages × steps; ERK steps; GNSF feasibility) | 12–17 |
| 1a    | Seed solver pick | 1 |
| 0     | Cost-weight baseline (Bryson + sequential α_R, α_we) | 10 |
| 1b    | Full QP × integrator × NLP × Hessian sweep | ~22 |
| 2     | Horizon × Tf × controller `magnet.n`, top-2 of 1b | ~72 |
| 3a    | Robustness on top-1 (5 directions × 3 magnitudes + yaw probe) | 21 |
| 3b    | Timing stability (5 reps × top-3) | 15 |

See `C:\Users\mariujf\.claude\plans\i-am-currently-in-atomic-valley.md`
for the full design rationale.

## Layout

```
experiments/solver_tuning/
├── README.md                  this file
├── runFullBenchmark.m         orchestrator (entry point)
├── verify_setup.m             pre-launch sanity checks
├── build_all_configs.m        cfg lists per stage
├── analysis/
│   ├── plot_pareto.py         Pareto + top-3 LaTeX
│   └── plot_heatmap.py        Stage-2 N × controller_n heatmaps
└── results/                   per-run .mat files (gitignored)
    ├── reduced10/<stage>/*.mat
    ├── full12/<stage>/*.mat
    └── _checkpoints/progress.mat
```

The shared NMPC harness library lives at
`NMPCProject/nmpc_lib/` (`build_dynamics.m`, `build_model.m`,
`build_ocp.m`, `build_plant.m`, `run_sim.m`, `compute_metrics.m`,
`bryson_weights.m`, `get_or_build_model.m`, `project_setup.m`,
`runOneConfig.m`, `dump_index.m`). New experiments should reuse those
and add their own `build_all_configs.m` / orchestrator under
`experiments/<name>/`.
