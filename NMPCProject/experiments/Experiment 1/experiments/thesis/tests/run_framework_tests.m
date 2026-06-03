function failures = run_framework_tests(varargin)
% RUN_FRAMEWORK_TESTS  Pre-flight tests for the thesis framework library.
%
%   failures = run_framework_tests()
%   failures = run_framework_tests('Verbose', true)
%   failures = run_framework_tests('Phase', 'P1' | 'P2' | 'all')   (default 'all')
%
% Tests are added per implementation phase:
%
%   P1 (active): cfg_schema, validate_cfg, hash_cfg, env_info.
%   P2 (active): baseline cfgs, build_dynamics, compute_equilibrium,
%                build_model, build_plant, build_ocp. NOTE: P2 builds
%                acados solvers and takes a few minutes the first time.
%   P3 (pending): run_closed_loop sanity + classify_outcome spot-checks.
%   P4 (pending): bit-exact baseline reproduction.
%
% Each test prints PASS/FAIL with a short reason. Returns the number of
% failures (0 = all green).

    p = inputParser();
    p.addParameter('Verbose', false, @islogical);
    p.addParameter('Phase',   'all', @(x) ischar(x) && ismember(lower(x),{'p1','p2','p3','all'}));
    p.parse(varargin{:});
    verbose = p.Results.Verbose;
    phase   = lower(p.Results.Phase);

    % This file lives at experiments/thesis/tests/. Add the thesis config
    % directory so project_setup_thesis is findable, then run it for the
    % full path/env bootstrap.
    here = fileparts(mfilename('fullpath'));
    repo = fileparts(fileparts(fileparts(here)));
    addpath(fullfile(repo, 'experiments', 'thesis', 'config'));
    project_setup_thesis();

    fprintf('\n=== nmpc_lib pre-flight tests ===\n');
    fprintf('repo_root: %s\n', repo);
    fprintf('phase:     %s\n\n', phase);

    failures = 0;
    if any(strcmp(phase, {'p1','all'}))
        failures = failures + run_block('P1', @() p1_tests(verbose));
    end
    if any(strcmp(phase, {'p2','all'}))
        failures = failures + run_block('P2', @() p2_tests(verbose));
    end
    if any(strcmp(phase, {'p3','all'}))
        failures = failures + run_block('P3', @() p3_tests(verbose));
    end

    fprintf('\n=== Summary ===\n');
    if failures == 0
        fprintf('  ALL GREEN\n\n');
    else
        fprintf('  %d FAILURE(S)\n\n', failures);
    end
end

% ---------------------------------------------------------------------- %
% P1 tests
% ---------------------------------------------------------------------- %

function n_fail = p1_tests(verbose)
    n_fail = 0;

    n_fail = n_fail + tcase('cfg_schema returns a non-empty struct', @() ...
        check(isstruct(cfg_schema()) && ~isempty(fieldnames(cfg_schema())), ...
              'cfg_schema() returned an empty/non-struct'), verbose);

    n_fail = n_fail + tcase('schema has every required top-level field', @() ...
        check_required_fields_listed(), verbose);

    n_fail = n_fail + tcase('validate_cfg rejects a missing required field', @() ...
        expect_error(@() validate_cfg(struct()), 'validate_cfg:missing_required'), verbose);

    n_fail = n_fail + tcase('validate_cfg fills optional defaults', @() ...
        check_defaults_applied(), verbose);

    n_fail = n_fail + tcase('validate_cfg rejects unjustified plant_override asymmetry', @() ...
        check_plant_override_unjustified(), verbose);

    n_fail = n_fail + tcase('validate_cfg accepts justified plant_override asymmetry', @() ...
        check_plant_override_justified(), verbose);

    n_fail = n_fail + tcase('SQP_RTI forces nlp_solver_max_iter = 1', @() ...
        check_rti_forces_one_iter(), verbose);

    n_fail = n_fail + tcase('hash_cfg returns a 64-char lowercase hex string', @() ...
        check_hash_format(), verbose);

    n_fail = n_fail + tcase('hash_cfg is order-independent', @() ...
        check_hash_field_order(), verbose);

    n_fail = n_fail + tcase('hash_cfg distinguishes different cfgs', @() ...
        check_hash_distinguishes(), verbose);

    n_fail = n_fail + tcase('env_info returns all expected fields with non-empty values', @() ...
        check_env_info(), verbose);
end

function check_required_fields_listed()
    S    = cfg_schema();
    must = {'stage','exp_id','order','N','Tf','integrator_type', ...
            'sim_method_num_stages','sim_method_num_steps','nlp_solver_type', ...
            'qp_solver','hessian_approx','regularize_method','Q','R','alpha_we', ...
            'lbu','ubu','idxbx','lbx','ubx','idxsbx','Zl','Zu','zl','zu', ...
            'sim_steps','ic_descriptor','warmstart_strategy'};
    missing = setdiff(must, fieldnames(S));
    if ~isempty(missing)
        error('schema is missing required fields: %s', strjoin(missing, ', '));
    end
end

function check_defaults_applied()
    cfg = baseline_min_cfg();
    out = validate_cfg(cfg, 'PrintDefaults', false);
    if out.qp_solver_iter_max ~= 200
        error('qp_solver_iter_max default not applied (got %g)', out.qp_solver_iter_max);
    end
    if out.qp_solver_warm_start ~= 1
        error('qp_solver_warm_start default not applied');
    end
    if abs(out.nlp_solver_tol_stat - 1e-4) > 0
        error('nlp_solver_tol_stat default not applied');
    end
    if out.plant_override.magnet_n ~= 10
        error('plant_override.magnet_n default not applied');
    end
end

function check_plant_override_unjustified()
    cfg = baseline_min_cfg();
    cfg.plant_override.sim_method_num_stages = 4;  % differs from cfg.sim_method_num_stages=2
    cfg.plant_override.sim_method_num_steps  = 10; % differs from cfg.sim_method_num_steps=5
    % No reason supplied -> must error.
    expect_error(@() validate_cfg(cfg, 'PrintDefaults', false), ...
                 'validate_cfg:plant_override_unjustified');
end

function check_plant_override_justified()
    cfg = baseline_min_cfg();
    cfg.plant_override.sim_method_num_stages = 4;
    cfg.plant_override.sim_method_num_steps  = 10;
    cfg.plant_override.reason = 'reduced-baseline reproduction';
    out = validate_cfg(cfg, 'PrintDefaults', false);
    if ~out.plant_override.is_asymmetric
        error('asymmetric plant_override did not set is_asymmetric=true');
    end
end

function check_rti_forces_one_iter()
    cfg = baseline_min_cfg();
    cfg.nlp_solver_type   = 'SQP_RTI';
    cfg.nlp_solver_max_iter = 50;  % should be overridden to 1
    out = validate_cfg(cfg, 'PrintDefaults', false);
    if out.nlp_solver_max_iter ~= 1
        error('SQP_RTI did not force nlp_solver_max_iter=1 (got %g)', out.nlp_solver_max_iter);
    end
end

function check_hash_format()
    cfg = baseline_min_cfg();
    h = hash_cfg(cfg);
    if ~ischar(h) || numel(h) ~= 64
        error('hash_cfg returned a non-64-char string (len=%d)', numel(h));
    end
    if ~all((h>='0'&h<='9') | (h>='a'&h<='f'))
        error('hash_cfg returned non-hex characters');
    end
end

function check_hash_field_order()
    cfg1 = baseline_min_cfg();
    cfg2 = struct();
    fns  = fliplr(fieldnames(cfg1).');
    for i = 1:numel(fns)
        cfg2.(fns{i}) = cfg1.(fns{i});
    end
    if ~strcmp(hash_cfg(cfg1), hash_cfg(cfg2))
        error('hash_cfg is sensitive to field order');
    end
end

function check_hash_distinguishes()
    cfg = baseline_min_cfg();
    h1  = hash_cfg(cfg);
    cfg.N = cfg.N + 1;
    h2  = hash_cfg(cfg);
    if strcmp(h1, h2)
        error('hash_cfg failed to distinguish two distinct cfgs');
    end
end

function check_env_info()
    info = env_info();
    must = {'timestamp','schema_version','matlab_version','casadi_version', ...
            'acados_install_dir','acados_version','repo_root','git_commit', ...
            'host','os','cpu'};
    missing = setdiff(must, fieldnames(info));
    if ~isempty(missing)
        error('env_info missing fields: %s', strjoin(missing, ', '));
    end
    if isempty(info.timestamp); error('env_info.timestamp is empty'); end
    if isempty(info.schema_version); error('env_info.schema_version is empty'); end
end

% ---------------------------------------------------------------------- %
% P2 tests
% ---------------------------------------------------------------------- %

function n_fail = p2_tests(verbose)
    n_fail = 0;

    n_fail = n_fail + tcase('baseline_full12 cfg validates', @() ...
        check_baseline_validates(@baseline_full12, false), verbose);

    n_fail = n_fail + tcase('baseline_reduced10 cfg validates and is asymmetric', @() ...
        check_baseline_validates(@baseline_reduced10, true), verbose);

    n_fail = n_fail + tcase('build_dynamics returns a usable CasADi function', @() ...
        check_build_dynamics(), verbose);

    n_fail = n_fail + tcase('compute_equilibrium yields zEq in [0.015, 0.060] with small residual', @() ...
        check_compute_equilibrium(), verbose);

    n_fail = n_fail + tcase('build_model.full12 has nx_ctrl=12 and ctrl_idx=1:12', @() ...
        check_build_model_full12(), verbose);

    n_fail = n_fail + tcase('build_model.reduced10 has nx_ctrl=10 and projects yaw out', @() ...
        check_build_model_reduced10(), verbose);

    n_fail = n_fail + tcase('build_plant succeeds for full12 baseline (matched-plant)', @() ...
        check_build_plant_full12(), verbose);

    n_fail = n_fail + tcase('build_ocp succeeds for full12 baseline and returns a solver', @() ...
        check_build_ocp_full12(), verbose);

    n_fail = n_fail + tcase('single-step OCP solve returns a finite 4-vector u_0', @() ...
        check_single_step_solve(), verbose);
end

function check_baseline_validates(make_cfg, expect_asymmetric)
    cfg = make_cfg();
    out = validate_cfg(cfg, 'PrintDefaults', false);
    if out.plant_override.is_asymmetric ~= expect_asymmetric
        error('expected is_asymmetric=%d, got %d', ...
              expect_asymmetric, out.plant_override.is_asymmetric);
    end
end

function check_build_dynamics()
    dyn = build_dynamics(10);
    if ~isstruct(dyn) || ~isfield(dyn,'f_func')
        error('build_dynamics did not return a struct with f_func');
    end
    f = dyn.f_func;
    if ~isa(f, 'casadi.Function')
        error('dyn.f_func is not a casadi.Function (class=%s)', class(f));
    end
    if f.n_in() ~= 2 || f.n_out() ~= 1
        error('dyn.f_func has wrong arity: n_in=%d, n_out=%d (expected 2, 1)', ...
              f.n_in(), f.n_out());
    end
    % Evaluate at a benign point and check the output shape.
    out = full(f(zeros(12,1), zeros(4,1)));
    if numel(out) ~= 12
        error('dyn.f_func output has wrong length: %d (expected 12)', numel(out));
    end
end

function check_compute_equilibrium()
    dyn = build_dynamics(10);
    eq  = compute_equilibrium(dyn);
    if eq.zEq < 0.015 || eq.zEq > 0.060
        error('zEq=%.6f outside physical range [0.015, 0.060]', eq.zEq);
    end
    if eq.residual > 1e-6
        error('zEq residual %.3g too large (>1e-6)', eq.residual);
    end
end

function check_build_model_full12()
    M = build_model(struct('order','full12'));
    if M.nx_ctrl ~= 12
        error('full12 nx_ctrl=%d (expected 12)', M.nx_ctrl);
    end
    if ~isequal(M.ctrl_idx, 1:12)
        error('full12 ctrl_idx mismatch');
    end
    if numel(M.xEq_plant) ~= 12 || numel(M.xEq_ctrl) ~= 12
        error('full12 equilibrium has wrong dims');
    end
end

function check_build_model_reduced10()
    M = build_model(struct('order','reduced10'));
    if M.nx_ctrl ~= 10
        error('reduced10 nx_ctrl=%d (expected 10)', M.nx_ctrl);
    end
    if ~isequal(M.ctrl_idx, [1 2 3 4 5 7 8 9 10 11])
        error('reduced10 ctrl_idx mismatch');
    end
    % Lift / project roundtrip on the equilibrium.
    x12 = M.xEq_plant;
    x10 = M.project_to_ctrl(x12);
    x12b = M.lift_to_plant(x10);
    if any(abs(x12b([1:5 7:11]) - x12([1:5 7:11])) > 1e-12)
        error('project/lift roundtrip changed retained components');
    end
end

function check_build_plant_full12()
    cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
    M   = build_model(struct('order','full12'));
    ps  = build_plant(M, cfg);
    if ~isa(ps, 'AcadosSimSolver')
        error('build_plant returned %s, expected AcadosSimSolver', class(ps));
    end
end

function check_build_ocp_full12()
    cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
    M   = build_model(struct('order','full12'));
    [ocp_solver, model_name, meta] = build_ocp(M, cfg);
    if ~isa(ocp_solver, 'AcadosOcpSolver')
        error('build_ocp returned %s, expected AcadosOcpSolver', class(ocp_solver));
    end
    if ~ischar(model_name) || numel(model_name) < 4
        error('model_name looks malformed: "%s"', model_name);
    end
    if ~isfield(meta,'build_time_s') || meta.build_time_s < 0
        error('build_meta.build_time_s missing or invalid');
    end
end

function check_single_step_solve()
    cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
    M   = build_model(struct('order','full12'));
    ocp_solver = build_ocp(M, cfg);

    x0 = M.xEq_plant + cfg.ic_descriptor.scale * cfg.ic_descriptor.direction;
    x0_ctrl = M.project_to_ctrl(x0);
    xEq_ctrl = M.xEq_ctrl;

    % Linear-interpolation warmstart (mirrors workingSimulator.m).
    for k = 0:cfg.N
        a = k / cfg.N;
        ocp_solver.set('x', (1-a)*x0_ctrl + a*xEq_ctrl, k);
    end
    for k = 0:(cfg.N-1)
        ocp_solver.set('u', M.uEq, k);
    end

    ocp_solver.set('constr_x0', x0_ctrl);
    ocp_solver.solve();
    status = ocp_solver.get('status');
    if status ~= 0
        error('OCP solve returned status %d on baseline IC with linear warmstart', status);
    end
    u0 = ocp_solver.get('u', 0);
    if numel(u0) ~= 4 || any(~isfinite(u0))
        error('u0 is malformed: %s', mat2str(u0));
    end
    % All four inputs should be within input bounds (RTI may not respect them
    % strictly, but a single step from a small perturbation should be safe).
    if any(abs(u0) > 1.5)
        error('u0 magnitudes exceed reasonable range: %s', mat2str(u0));
    end
end

% ---------------------------------------------------------------------- %
% P3 tests
% ---------------------------------------------------------------------- %

function n_fail = p3_tests(verbose)
    n_fail = 0;

    % These tests share an expensive setup; build once via a shared closure.
    state = struct();

    n_fail = n_fail + tcase('full12 baseline closed-loop runs to completion', @() ...
        setup_and_run(state, 'shared'), verbose);

    n_fail = n_fail + tcase('classify_outcome on baseline -> CONVERGED_STABLE', @() ...
        check_classify_stable(), verbose);

    n_fail = n_fail + tcase('timing_stats produces finite values for baseline', @() ...
        check_timing_stats(), verbose);

    n_fail = n_fail + tcase('tracking_metrics has finite settling time for baseline', @() ...
        check_tracking_metrics(), verbose);

    n_fail = n_fail + tcase('disturbance_metrics returns has_disturbance=false on full12', @() ...
        check_disturbance_metrics(), verbose);

    n_fail = n_fail + tcase('convergence_metrics reports all_converged=true', @() ...
        check_convergence_metrics(), verbose);

    n_fail = n_fail + tcase('result_record has all required top-level fields', @() ...
        check_result_record(), verbose);

    n_fail = n_fail + tcase('save_result + load_result roundtrip preserves cfg_hash', @() ...
        check_save_load_roundtrip(), verbose);

    n_fail = n_fail + tcase('NaN injection -> classify_outcome returns NUMERIC_NAN', @() ...
        check_nan_classifier(), verbose);

    n_fail = n_fail + tcase('index_results builds a table containing the saved row', @() ...
        check_index_results(), verbose);
end

% --- Shared expensive setup, memoized in a persistent var -------------- %
function [R, M, cfg, build_meta] = p3_state()
% Build everything once for the P3 block: cfg -> M -> ocp/plant solvers ->
% closed-loop run. Subsequent calls return the cached state.
    persistent S
    if ~isempty(S); R = S.R; M = S.M; cfg = S.cfg; build_meta = S.build_meta; return; end

    cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
    M   = build_model(struct('order','full12'));
    [ocp_solver, ~, build_meta] = build_ocp(M, cfg);
    plant_solver = build_plant(M, cfg);

    % Use the baseline's full 200-step horizon so the system has time to
    % settle (matches workingSimulator.m exactly). The closed-loop run is
    % ~1 second of simulated time and a handful of seconds of wall clock.
    R = run_closed_loop(ocp_solver, plant_solver, M, cfg);

    S.R = R;
    S.M = M;
    S.cfg = cfg;
    S.build_meta = build_meta;
end

function setup_and_run(~, ~)
    [R, ~, cfg, ~] = p3_state();
    if R.n_actual ~= cfg.sim_steps
        error('closed loop stopped early: n_actual=%d < sim_steps=%d', R.n_actual, cfg.sim_steps);
    end
    if R.diverged
        error('baseline run flagged as diverged');
    end
    if any(~isfinite(R.timing.tot)) || any(~isfinite(R.timing.wall))
        error('timing arrays contain NaN/Inf');
    end
    if any(~isfinite(R.X_plant(:)))
        error('X_plant contains NaN/Inf');
    end
end

function check_classify_stable()
    [R, M, cfg, ~] = p3_state();
    [class, reason] = classify_outcome(R, M, cfg);
    if ~strcmp(class,'CONVERGED_STABLE')
        error('expected CONVERGED_STABLE, got %s (%s)', class, reason);
    end
end

function check_timing_stats()
    [R, ~, cfg, ~] = p3_state();
    S = timing_stats(R, cfg);
    must = {'t_mean_ms','t_p95_ms','t_p99_ms','t_max_ms','embedded_headroom_ms'};
    for i = 1:numel(must)
        if ~isfield(S, must{i}) || ~isfinite(S.(must{i}))
            error('timing_stats.%s is missing or non-finite', must{i});
        end
    end
end

function check_tracking_metrics()
    [R, M, cfg, ~] = p3_state();
    S = tracking_metrics(R, M, cfg);
    if ~isfinite(S.peak_pos_mm); error('peak_pos_mm not finite'); end
    if ~isfinite(S.final_err_norm); error('final_err_norm not finite'); end
    if ~isfinite(S.J_integrated);   error('J_integrated not finite'); end
    if ~isfinite(S.peak_ang_deg);   error('peak_ang_deg not finite'); end
    % Note: settling_*_ms is intentionally NOT asserted here. For full-12,
    % the whole-state error norm never reaches a tight fraction of initial
    % because yaw is uncontrollable. Stage A will instead validate that the
    % framework's CONTROLLABLE-state trajectory matches workingSimulator.m
    % bit-for-bit.
    %
    % Sanity bound: position peak should be within the OCP's soft box
    % (+ slack), say < 30 mm. If the closed loop were truly diverging the
    % peak would be hundreds of mm or NaN.
    if S.peak_pos_mm > 30
        error('peak_pos_mm = %.2f mm exceeds 30 mm sanity bound', S.peak_pos_mm);
    end
end

function check_disturbance_metrics()
    [R, M, cfg, ~] = p3_state();
    S = disturbance_metrics(R, M, cfg);
    if S.has_disturbance
        error('expected has_disturbance=false for full12 baseline');
    end
end

function check_convergence_metrics()
    [R, ~, ~, ~] = p3_state();
    S = convergence_metrics(R);
    if ~S.all_converged
        error('baseline run not all-converged (pct_status_0=%.1f)', S.pct_status_0);
    end
end

function check_result_record()
    [R, M, cfg, bm] = p3_state();
    [class, reason] = classify_outcome(R, M, cfg);
    rec = result_record(cfg, M, R, class, reason, bm);

    must = {'schema_version','timestamp','run_id','cfg_hash','params_hash', ...
            'classification','reason','env_info','cfg','model','build_meta','run','summary'};
    missing = setdiff(must, fieldnames(rec));
    if ~isempty(missing)
        error('result_record missing fields: %s', strjoin(missing, ', '));
    end
    if numel(rec.cfg_hash) ~= 64
        error('cfg_hash has wrong length: %d (expected 64)', numel(rec.cfg_hash));
    end
end

function check_save_load_roundtrip()
    [R, M, cfg, bm] = p3_state();
    [class, reason] = classify_outcome(R, M, cfg);
    rec = result_record(cfg, M, R, class, reason, bm);

    tmp = fullfile(tempdir, sprintf('thesis_tests_%s', rec.run_id(1:8)));
    if ~exist(tmp, 'dir'); mkdir(tmp); end
    cleanup = onCleanup(@() rmdir(tmp, 's'));

    path = save_result(rec, tmp);
    if ~exist(path, 'file')
        error('save_result did not produce file at %s', path);
    end
    rec2 = load_result(path);
    if ~strcmp(rec.cfg_hash, rec2.cfg_hash)
        error('cfg_hash differs after roundtrip');
    end
    if ~strcmp(rec.run_id, rec2.run_id)
        error('run_id differs after roundtrip');
    end
end

function check_nan_classifier()
    [R, M, cfg, ~] = p3_state();
    R_bad = R;
    R_bad.X_plant(3, 50) = NaN;
    [class, ~] = classify_outcome(R_bad, M, cfg);
    if ~strcmp(class, 'NUMERIC_NAN')
        error('NaN-injected run was classified as %s (expected NUMERIC_NAN)', class);
    end
end

function check_index_results()
    [R, M, cfg, bm] = p3_state();
    [class, reason] = classify_outcome(R, M, cfg);
    rec = result_record(cfg, M, R, class, reason, bm);

    tmp = fullfile(tempdir, sprintf('thesis_index_%s', rec.run_id(1:8)));
    if ~exist(tmp, 'dir'); mkdir(tmp); end
    cleanup = onCleanup(@() rmdir(tmp, 's'));

    save_result(rec, tmp);
    T = index_results(tmp);
    if isempty(T)
        error('index_results returned empty table');
    end
    if ~any(strcmp(T.cfg_hash, rec.cfg_hash))
        error('saved row not found in index (cfg_hash=%s)', rec.cfg_hash);
    end
end

% ---------------------------------------------------------------------- %
% Test infrastructure
% ---------------------------------------------------------------------- %

function n = run_block(name, fn)
    fprintf('--- %s ---\n', name);
    n = fn();
    fprintf('\n');
end

function n = tcase(label, fn, verbose)
    try
        fn();
        fprintf('  PASS  %s\n', label);
        n = 0;
    catch ME
        fprintf('  FAIL  %s\n', label);
        fprintf('        %s\n', ME.message);
        if verbose
            for k = 1:numel(ME.stack)
                fprintf('          at %s:%d\n', ME.stack(k).file, ME.stack(k).line);
            end
        end
        n = 1;
    end
end

function check(cond, msg)
    if ~cond; error('%s', msg); end
end

function expect_error(fn, identifier)
    threw = false;
    try
        fn();
    catch ME
        threw = true;
        if ~strcmp(ME.identifier, identifier)
            error('expected error "%s", got "%s" (%s)', identifier, ME.identifier, ME.message);
        end
    end
    if ~threw
        error('expected error "%s" but no error was raised', identifier);
    end
end

% ---------------------------------------------------------------------- %
% A minimal cfg that passes validation, used as a base for hash/validate
% tests. Mirrors the reduced-order baseline so it is feasible-by-shape.
% ---------------------------------------------------------------------- %

function cfg = baseline_min_cfg()
    cfg = struct();
    cfg.stage  = 'tests';
    cfg.exp_id = 'min';
    cfg.order  = 'reduced10';

    cfg.N  = 10;
    cfg.Tf = 0.1;

    cfg.integrator_type        = 'IRK';
    cfg.sim_method_num_stages  = 2;
    cfg.sim_method_num_steps   = 5;

    cfg.nlp_solver_type    = 'SQP_RTI';
    cfg.qp_solver          = 'FULL_CONDENSING_HPIPM';
    cfg.hessian_approx     = 'GAUSS_NEWTON';
    cfg.regularize_method  = 'CONVEXIFY';

    cfg.Q        = [1e2 1e2 1e3 1e3 1e3 1e1 1e1 1e1 1e1 1e1];
    cfg.R        = [1 1 1 1];
    cfg.alpha_we = 50;

    cfg.lbu    = -ones(4,1);
    cfg.ubu    =  ones(4,1);
    cfg.idxbx  = [0 1 2 3 4];
    cfg.lbx    = [-0.025; -0.025; 0.015; -0.35; -0.35];
    cfg.ubx    = [ 0.025;  0.025; 0.055;  0.35;  0.35];
    cfg.idxsbx = [0 1 2 3 4];
    cfg.Zl     = 1e3 * ones(5,1);
    cfg.Zu     = 1e3 * ones(5,1);
    cfg.zl     = 1e2 * ones(5,1);
    cfg.zu     = 1e2 * ones(5,1);

    cfg.sim_steps = 1000;
    cfg.ic_descriptor = struct( ...
        'scale',     0.05, ...
        'direction', [0.005; -0.008; 0.010; 0.15; -0.10; 0.3; ...
                      0.01;  -0.01;  0.0;   0.2;  -0.3;  0.0]);
    cfg.warmstart_strategy = 'linear_to_eq';
end
