function ok = verify_setup()
% VERIFY_SETUP  Sanity checks before launching the solver_tuning benchmark.
%
%   ok = verify_setup()
%
% Self-bootstraps: adds nmpc_lib/ and this experiment's folder to the
% MATLAB path so the user can simply run `cd experiments/solver_tuning;
% verify_setup` from any starting CWD.
%
% Checks
%   1. computeSolenoidRadiusCorrectionFactor independence of magnet.n
%      (or, if it does depend, document by how much).
%   2. build_dynamics produces the expected 12-state CasADi function.
%   3. build_model returns consistent equilibria for both model_kinds.
%   4. A single short closed-loop run via runOneConfig completes without
%      diverging from the baseline IC.

    bootstrap_paths();
    paths = project_setup();
    ok    = true;

    fprintf('\n=== verify_setup (solver_tuning) ===\n\n');

    % --- Check 1: correction factor n-independence ---
    fprintf('[1/4] Checking computeSolenoidRadiusCorrectionFactor(n=10) vs (n=30)...\n');
    params = struct();                                      %#ok<NASGU>
    parameters_maggy_V4;                                    %#ok<NODEF>
    p10 = params; p10.magnet.n = 10;
    p30 = params; p30.magnet.n = 30;
    cf10 = computeSolenoidRadiusCorrectionFactor(p10, 'fast');
    cf30 = computeSolenoidRadiusCorrectionFactor(p30, 'fast');
    rel_diff = abs(cf10 - cf30) / max(abs(cf10), eps);
    fprintf('       cf(n=10) = %.10f\n', cf10);
    fprintf('       cf(n=30) = %.10f\n', cf30);
    fprintf('       rel diff = %.3e\n', rel_diff);
    if rel_diff < 1e-3
        fprintf('       PASS: n-independent within 0.1%%.\n\n');
    else
        fprintf('       NOTE: correction factor depends on n. build_dynamics already\n');
        fprintf('       recomputes per-n so this is fine, but document the dependence.\n\n');
    end

    % --- Check 2: build_dynamics returns a CasADi function with right signature ---
    fprintf('[2/4] Checking build_dynamics for n=10, n=30...\n');
    [f10, ~] = build_dynamics(10);
    [f30, ~] = build_dynamics(30);
    if ~isa(f10, 'casadi.Function') || ~isa(f30, 'casadi.Function')
        error('build_dynamics did not return a CasADi function.');
    end
    if f10.n_in() ~= 2 || f10.n_out() ~= 1
        error('Unexpected build_dynamics signature.');
    end
    fprintf('       PASS: both return casadi.Function with (x,u)->dx.\n\n');

    % --- Check 3: build_model equilibrium consistency ---
    fprintf('[3/4] Checking build_model equilibria...\n');
    M_red  = build_model('reduced10', 10, 30);
    M_full = build_model('full12',    10, 30);
    if abs(M_red.xEq_plant(3) - M_full.xEq_plant(3)) > 1e-9
        error('Equilibria differ between reduced10 and full12.');
    end
    if length(M_red.xEq_ctrl) ~= 10
        error('Reduced model xEq_ctrl is %d-dim, expected 10.', length(M_red.xEq_ctrl));
    end
    if length(M_full.xEq_ctrl) ~= 12
        error('Full model xEq_ctrl is %d-dim, expected 12.', length(M_full.xEq_ctrl));
    end
    fprintf('       PASS: zEq=%.6f m, dims correct.\n\n', M_red.xEq_plant(3));

    % --- Check 4: a one-shot runOneConfig with the legacy baseline cfg ---
    fprintf('[4/4] Running a baseline closed-loop sanity run...\n');
    cfg = baseline_cfg();
    cfg.save_dir = fullfile(experiment_dir(), 'results');
    res = runOneConfig(cfg);
    if res.summary.diverged
        warning('Baseline run diverged. Inspect results before launching the full sweep.');
        ok = false;
    else
        fprintf('       PASS: no divergence, t_mean=%.2f ms, J=%.3e\n', ...
            res.summary.t_mean_ms, res.summary.J_integrated);
    end

    if ok
        fprintf('\n=== verify_setup: ALL PASSED ===\n');
    else
        fprintf('\n=== verify_setup: ONE OR MORE FAILED ===\n');
    end
end

function bootstrap_paths()
% Add nmpc_lib/ and this experiment's dir to MATLAB path so subsequent
% function calls resolve. Idempotent.
    this_dir   = experiment_dir();
    proj_root  = fileparts(fileparts(this_dir));     % .../NMPCProject
    addpath(this_dir);
    addpath(fullfile(proj_root, 'nmpc_lib'));
end

function d = experiment_dir()
    d = fileparts(mfilename('fullpath'));
end

function cfg = baseline_cfg()
% Mirror of the legacy build_ocp settings used in runExperimentB.
    cfg.stage           = 'verify';
    cfg.exp_id          = 'baseline';
    cfg.model_kind      = 'reduced10';
    cfg.controller_n    = 10;
    cfg.plant_n         = 30;
    cfg.N               = 20;
    cfg.Tf              = 0.2;
    cfg.integ_type      = 'IRK';
    cfg.n_stages        = 4;
    cfg.n_steps         = 5;
    cfg.nlp_solver      = 'SQP_RTI';
    cfg.qp_solver       = 'PARTIAL_CONDENSING_HPIPM';
    cfg.hessian_approx  = 'GAUSS_NEWTON';
    cfg.Q               = diag([1e2 1e2 1e3 1e3 1e3 1e1 1e1 1e1 1e1 1e1]);
    cfg.R               = eye(4) * 1.0;
    cfg.alpha_we        = 50;
    cfg.alpha_R         = 1.0;
    cfg.ic_label        = 'baseline';
    cfg.ic_scale        = 0.05;
    cfg.ic_direction    = [0.005;-0.008;0.010; 0.15;-0.10; 0; 0.01;-0.01;0.0; 0.2;-0.3; 0];
    cfg.sim_steps       = 200;
    cfg.n_repeats       = 1;
end
