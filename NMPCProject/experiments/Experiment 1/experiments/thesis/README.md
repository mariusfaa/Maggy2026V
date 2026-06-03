# Thesis experiment framework

Multi-stage NMPC experiment pipeline for thesis-quality solver and OCP
tuning analysis of the magnetic-levitation system. Replaces the previous
auto-generated framework under `experiments/solver_tuning/`.

The methodology, staged design, and rationale are documented in the
approved plan at `~/.claude/plans/in-this-repository-i-toasty-lake.md`.

## Quick reference

- **Baselines** (validation truth):
  - [`workingSimulator.m`](../../workingSimulator.m) — 12-state, N=20,
    Ts=0.01, IRK 4/10, SQP_RTI, PARTIAL_CONDENSING_HPIPM.
  - [`workingSimulatorReducedOrder.m`](../../workingSimulatorReducedOrder.m)
    — 10-state, N=10, Ts=0.01, OCP IRK 2/5 vs plant IRK 4/10
    (asymmetric on purpose), includes z-axis disturbance at t=5 s.

- **Matched-plant policy**: every closed-loop run rebuilds the plant
  simulator with the OCP integrator's `(type, stages, steps)`. The only
  documented exception is the Stage-A reproduction of
  `workingSimulatorReducedOrder.m`, which sets explicit
  `cfg.plant_override.*` fields.

- **`params.magnet.n = 10`** throughout; plant `magnet_n = 10`. The
  previous framework's silent `n=30` plant is gone.

## Layout

```
experiments/thesis/
├── README.md                  this file
├── config/
│   ├── baseline_full12.m      cfg that reproduces workingSimulator.m
│   ├── baseline_reduced10.m   cfg that reproduces workingSimulatorReducedOrder.m
│   └── default_ranges.m       factor ranges used in Stages B–E
├── stages/
│   ├── stageA_validate.m      baseline reproduction gate
│   ├── stageB_solver_factors.m
│   ├── stageC_integrator_depth.m
│   ├── stageD_horizon_ts.m
│   ├── stageE_cost_tuning.m
│   ├── stageF_model_order.m
│   ├── stageG_robustness.m
│   └── stageH_synthesis.m
├── analysis/
│   ├── make_all_figures.m
│   ├── make_all_tables.m
│   └── generate_report.m
├── tests/
│   └── run_framework_tests.m  pre-flight tests (schema, hash, classifier)
└── results/                   .gitignored
    ├── _meta/                 env_info snapshots, schema, manifest
    ├── _checkpoints/          per-stage progress files
    └── stageA/ ... stageH/    per-stage result records
```

## Running an experiment

The library and stages are MATLAB-only. From the repo root:

```matlab
% First-time setup (project paths, acados env vars, etc.):
project_setup_thesis;

% Pre-flight tests — always green before any stage.
run experiments/thesis/tests/run_framework_tests.m

% Stage A is the validation gate. Until it passes, no other stage runs.
run experiments/thesis/stages/stageA_validate.m

% Then proceed stage-by-stage:
run experiments/thesis/stages/stageB_solver_factors.m
run experiments/thesis/stages/stageC_integrator_depth.m
% ...
```

Each stage runner:

1. Calls `validate_cfg(cfg)` for every planned configuration.
2. Checks the result cache (`results/<stage>/<order>/<exp_id>__<hash>.mat`).
   If a matching cfg_hash exists, the run is skipped.
3. Builds OCP + plant (with matched-plant policy), runs the closed loop,
   classifies the outcome, computes metrics, saves a `result_record`.
4. Updates `results/_checkpoints/<stage>.mat` with the completed hash.
5. On crash, the next invocation resumes from the checkpoint.

## Status (implementation phases)

| Phase | Scope | Status |
|---|---|---|
| P1 | Scaffolding + schema + env_info + hash_cfg | done (this commit) |
| P2 | Build pipeline (dynamics, model, equilibrium, plant, ocp) | pending |
| P3 | Closed loop + classification + metrics + result_record | pending |
| P4 | Stage A baseline reproduction (validation gate) | pending |
| P5 | Stages B–C (solver factors, integrator depth) | pending |
| P6 | Stages D–E (horizon/Ts, cost tuning) | pending |
| P7 | Stages F–G (model order, robustness) | pending |
| P8 | Stage H synthesis + thesis artefacts | pending |

## Differences from the previous `experiments/solver_tuning/`

| Aspect | Old framework | This framework |
|---|---|---|
| Plant integrator | Hardcoded IRK 4/10 | Matches OCP integrator (matched-plant policy) |
| Plant `magnet.n` | 30 (silently) | 10 (matching the baselines) |
| Failure semantics | Diverged/not | Seven-class taxonomy with human-readable reason |
| Reproducibility | UUID + timestamp only | `cfg_hash`, `params_hash`, `git_commit`, `schema_version`, full `env_info` |
| Sweep design | Cartesian products, no pruning | Hierarchical: anchor at baseline, one-factor-at-a-time, viable-region propagation |
| Cost weights | Locked after Stage 0 against one integrator | Re-tuned per model order in Stage E (after solver selection) |
| Analysis stack | MATLAB + Python (CSV bridge) | MATLAB-only |
| Stage outputs | Heatmaps, top-K Pareto | Research-question-driven figures (one figure per question) |

## Legacy results

The previous framework's `.mat` results under
`experiments/solver_tuning/results/` are preserved for reference but are
**not** consumed by this pipeline. They will be moved to
`experiments/_legacy_solver_tuning_results/` when the old framework is
fully decommissioned (after Phase P4 passes its validation gate).
