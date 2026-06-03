# nmpc_lib — thesis NMPC experiment library

Thesis-grade helper library for the acados magnetic-levitation NMPC
experiments. Replaces the auto-generated `nmpc_lib/` of the previous
framework. Plan file: `../in-this-repository-i-toasty-lake.md`
(in `~/.claude/plans/`).

## Layout

```
nmpc_lib/
├── cfg_schema.m           authoritative schema for every cfg field
├── core/                  build pipeline (P2) + closed loop (P3)
│   ├── build_dynamics.m
│   ├── build_model.m
│   ├── compute_equilibrium.m
│   ├── build_plant.m
│   ├── build_ocp.m
│   ├── run_closed_loop.m
│   └── classify_outcome.m
├── metrics/               metric reduction (P3)
│   ├── timing_stats.m
│   ├── tracking_metrics.m
│   ├── disturbance_metrics.m
│   └── convergence_metrics.m
├── io/                    I/O + reproducibility (P1)
│   ├── env_info.m
│   ├── hash_cfg.m
│   ├── validate_cfg.m
│   ├── result_record.m
│   ├── save_result.m
│   ├── load_result.m
│   └── index_results.m
├── plotting/              publication-style figures (P5+)
│   ├── pub_style.m
│   ├── plot_factor_main_effects.m
│   ├── plot_pareto_2d.m
│   ├── plot_pareto_3d.m
│   ├── plot_stability_region.m
│   ├── plot_timing_distribution.m
│   └── plot_trajectory_overlay.m
└── README.md
```

## Core principles

1. **Anchor to the baseline.** Every experiment starts from a cfg that
   reproduces `workingSimulator.m` (full12) or
   `workingSimulatorReducedOrder.m` (reduced10) and varies one element at
   a time.
2. **Matched-plant policy.** The plant simulator's integrator (type,
   stages, steps) is rebuilt to exactly mirror the OCP integrator each
   run. Asymmetry requires explicit `cfg.plant_override.*` and a
   `plant_override.reason` justification string; the result record is
   then flagged `asymmetric_plant = true`.
3. **No silent defaults.** `validate_cfg` enforces `cfg_schema` at the
   entry of every stage. Missing required fields are an error; missing
   optional fields print the default that was applied.
4. **Failed runs are data.** `classify_outcome` returns one of seven
   classes (`CONVERGED_STABLE`, `CONVERGED_DEGRADED`,
   `DIVERGED_PHYSICAL`, `QP_INFEASIBLE`, `NLP_MAX_ITER`, `NUMERIC_NAN`,
   `TIMEOUT_PER_STEP`, `BUILD_FAIL`) plus a human-readable reason.
5. **Content-addressed results.** Each result file is named
   `<exp_id>__<cfg_hash[:8]>.mat`. Result records carry `cfg_hash`,
   `params_hash`, `git_commit`, `schema_version`, `env_info` so a future
   reader can determine exactly what produced the file.

## Status (implementation phases)

| Phase | Scope | Status |
|---|---|---|
| P1 | Scaffolding + schema + env_info + hash_cfg + validate_cfg | done (this commit) |
| P2 | Build pipeline (dynamics, model, equilibrium, plant, ocp) | pending |
| P3 | Closed loop + classification + metrics + result_record | pending |
| P4 | Stage A baseline reproduction (validation gate) | pending |
| P5 | Stages B–C (solver factors, integrator depth) | pending |
| P6 | Stages D–E (horizon/Ts, cost tuning) | pending |
| P7 | Stages F–G (model order, robustness) | pending |
| P8 | Stage H synthesis + thesis artefacts | pending |

## Conventions

- All public functions have a help block usable via `help nmpc_lib/...`.
- File paths in error messages are absolute; line numbers reference the
  current state of the file.
- No `try`/`catch` swallows errors silently. The only place silence is
  acceptable is `env_info`, which must never throw.
- All char comparisons use `strcmp` / `ismember` rather than `==`.
- All numeric tolerances are named constants in `cfg_schema` or stage
  configs, never inline magic numbers.
