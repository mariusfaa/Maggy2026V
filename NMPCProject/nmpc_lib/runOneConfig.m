function res = runOneConfig(cfg)
% RUNONECONFIG  Single configuration NMPC closed-loop run.
%
%   res = runOneConfig(cfg)
%
% Required cfg fields:
%   .stage           'P0' | '1a' | '0' | '1b' | '2' | '3a' | '3b'
%   .exp_id           short string (used in filenames and model_name)
%   .model_kind      'reduced10' | 'full12'
%   .controller_n    integer (params.magnet.n inside OCP)
%   .plant_n         integer (params.magnet.n inside plant; default 30)
%   .N, .Tf
%   .integ_type      'ERK' | 'IRK' | 'GNSF'
%   .n_stages        sim_method_num_stages
%   .n_steps         sim_method_num_steps
%   .nlp_solver      'SQP' | 'SQP_RTI'
%   .qp_solver
%   .hessian_approx
%   .Q, .R           weight matrices (or diagonals)
%   .alpha_we        terminal-cost multiplier
%   .ic_direction    12-dim vector (ic_label / ic_scale interpret it)
%   .ic_scale        scalar
%   .ic_label        short tag, e.g. 'baseline' | 'd_pos_s0p5'
%
% Optional:
%   .sim_steps       (default: max(200, ceil(3.0 / dt)))
%   .n_repeats       (default: 1; use >1 in Stage 3b for timing stability)
%   .save_dir        (default: NMPCProject/results)
%   .Q_ref, .R_ref   reference weights for J_integrated (default: Bryson)
%
% Output struct res with fields:
%   .saved_path      absolute path to the saved .mat file
%   .summary         the .summary substruct
%   .compile_time_s  OCP compile time (excludes plant compile, which is
%                    typically already cached when this function runs
%                    inside the orchestrator)
%   .skipped         logical, true if save file already existed (resume)

    paths = project_setup();

    % --- Defaults ---
    if ~isfield(cfg, 'plant_n') || isempty(cfg.plant_n);  cfg.plant_n  = 30; end
    if ~isfield(cfg, 'n_repeats') || isempty(cfg.n_repeats); cfg.n_repeats = 1; end
    if ~isfield(cfg, 'save_dir') || isempty(cfg.save_dir)
        cfg.save_dir = fullfile(paths.project_root, 'results');
    end
    if ~isfield(cfg, 'ic_direction') || isempty(cfg.ic_direction)
        cfg.ic_direction = [0.005;-0.008;0.010; 0.15;-0.10; 0; 0.01;-0.01;0.0; 0.2;-0.3; 0];
        cfg.ic_label     = setIfEmpty(cfg, 'ic_label', 'baseline');
    end
    cfg.ic_label = setIfEmpty(cfg, 'ic_label', sprintf('s%g', cfg.ic_scale));
    if ~isfield(cfg, 'ic_scale') || isempty(cfg.ic_scale); cfg.ic_scale = 0.05; end
    dt = cfg.Tf / cfg.N;
    if ~isfield(cfg, 'sim_steps') || isempty(cfg.sim_steps)
        cfg.sim_steps = max(200, ceil(3.0 / dt));
    end

    % --- Output path (skip if already exists) ---
    out_dir = fullfile(cfg.save_dir, cfg.model_kind, cfg.stage);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    fname    = sprintf('%s_%s_%s.mat', cfg.model_kind, cfg.stage, cfg.exp_id);
    out_path = fullfile(out_dir, fname);
    if exist(out_path, 'file')
        loaded = load(out_path, 'summary');
        res.saved_path     = out_path;
        res.summary        = loaded.summary;
        res.compile_time_s = NaN;
        res.skipped        = true;
        return;
    end

    fprintf('\n=== %s/%s/%s (n_ctrl=%d, N=%d, Tf=%.3f, %s/%s/%s/%s) ===\n', ...
        cfg.model_kind, cfg.stage, cfg.exp_id, cfg.controller_n, cfg.N, cfg.Tf, ...
        cfg.integ_type, cfg.nlp_solver, cfg.qp_solver, cfg.hessian_approx);

    % --- Build / fetch model and plant ---
    M = get_or_build_model(cfg.model_kind, cfg.controller_n, cfg.plant_n);
    plant_solver = build_plant(cfg.plant_n, dt, M);

    % --- Reference weights (always Bryson for cross-config comparability) ---
    if ~isfield(cfg, 'Q_ref') || isempty(cfg.Q_ref)
        [Q_ref, R_ref] = bryson_weights(cfg.model_kind);
        cfg.Q_ref = Q_ref; cfg.R_ref = R_ref;
    end

    % --- Build OCP solver (timed) ---
    t_compile = tic;
    [ocp_solver, model_name] = build_ocp(cfg, M);
    compile_time_s = toc(t_compile);
    fprintf('  Compile time: %.2f s (model %s)\n', compile_time_s, model_name);

    % --- Initial condition (12-state) ---
    x0_plant = M.xEq_plant + cfg.ic_scale * cfg.ic_direction(:);

    % --- Run sim n_repeats times ---
    runs = cell(cfg.n_repeats, 1);
    summaries = cell(cfg.n_repeats, 1);
    for r = 1:cfg.n_repeats
        runs{r} = run_sim(ocp_solver, plant_solver, x0_plant, M, cfg);
        summaries{r} = compute_metrics(runs{r}, M, cfg, cfg.Q_ref, cfg.R_ref);
        fprintf('  rep %d/%d: t_mean=%.2f ms  t_max=%.2f ms  J=%.3e  div=%d  n=%d/%d\n', ...
            r, cfg.n_repeats, summaries{r}.t_mean_ms, summaries{r}.t_max_ms, ...
            summaries{r}.J_integrated, summaries{r}.diverged, ...
            summaries{r}.n_actual, cfg.sim_steps);
    end

    % --- Aggregate across repeats ---
    summary = aggregate_summaries(summaries);
    summary.compile_time_s = compile_time_s;
    summary.model_name     = model_name;

    % --- Build save struct ---
    sd                = struct();
    sd.run_id         = uuid_v4();
    sd.timestamp      = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    sd.cfg            = cfg;
    sd.model_kind     = cfg.model_kind;
    sd.stage          = cfg.stage;
    sd.exp_id         = cfg.exp_id;
    sd.controller_n   = cfg.controller_n;
    sd.plant_n        = cfg.plant_n;
    sd.integ_type     = cfg.integ_type;
    sd.n_stages       = cfg.n_stages;
    sd.n_steps        = cfg.n_steps;
    sd.nlp_solver     = cfg.nlp_solver;
    sd.qp_solver      = cfg.qp_solver;
    sd.hessian_approx = cfg.hessian_approx;
    sd.N              = cfg.N;
    sd.Tf             = cfg.Tf;
    sd.dt             = dt;
    sd.alpha_we       = cfg.alpha_we;
    sd.alpha_R        = getfielddef(cfg, 'alpha_R', NaN);
    sd.ic_label       = cfg.ic_label;
    sd.ic_scale       = cfg.ic_scale;
    sd.n_repeats      = cfg.n_repeats;
    sd.runs           = runs;        % full per-repeat trajectories
    sd.summary        = summary;
    sd.xEq_plant      = M.xEq_plant;
    sd.xEq_ctrl       = M.xEq_ctrl;
    sd.uEq            = M.uEq;
    sd.Q              = cfg.Q;
    sd.R              = cfg.R;
    sd.W_e            = cfg.alpha_we * (asMat(cfg.Q, M.nx_ctrl));
    sd.params_ctrl    = M.params_ctrl;
    sd.params_plant   = M.params_plant;

    save(out_path, '-struct', 'sd', '-v7.3');
    fprintf('  Saved: %s\n', out_path);

    res.saved_path     = out_path;
    res.summary        = summary;
    res.compile_time_s = compile_time_s;
    res.skipped        = false;
end

% =========================================================================
% Helpers
% =========================================================================

function v = setIfEmpty(s, fld, default)
    if isfield(s, fld) && ~isempty(s.(fld))
        v = s.(fld);
    else
        v = default;
    end
end

function v = getfielddef(s, fld, default)
    if isfield(s, fld) && ~isempty(s.(fld))
        v = s.(fld);
    else
        v = default;
    end
end

function W = asMat(W_in, n)
    if isvector(W_in); W = diag(W_in(:)); else; W = W_in; end
end

function S = aggregate_summaries(S_list)
% Aggregate per-repeat summary structs into one. Means across repeats for
% scalars; logical OR for diverged; per-repeat values also stored.
    n = numel(S_list);
    fields_mean = {'t_mean_ms','t_p95_ms','t_p99_ms','t_max_ms', ...
                   't_wall_mean_ms','t_wall_p99_ms','t_wall_max_ms', ...
                   'time_lin_mean_ms','time_qp_sol_mean_ms','time_reg_mean_ms', ...
                   'J_integrated','sqp_iter_mean','qp_iter_mean', ...
                   'pct_status_0','pct_saturated','control_effort', ...
                   'peak_pos_mm','peak_ang_deg','peak_yaw_deg','peak_z_dev_mm', ...
                   'settling_1pct_ms','settling_2pct_ms','settling_5pct_ms', ...
                   'n_actual'};
    for k = 1:length(fields_mean)
        f = fields_mean{k};
        vals = arrayfun(@(i) safe_field(S_list{i}, f), 1:n);
        S.(f)             = nanmean(vals);
        S.([f '_per_rep']) = vals;
    end
    div = false;
    n_v = 0; max_v = 0;
    converged = true;
    for i = 1:n
        div = div || S_list{i}.diverged;
        n_v = max(n_v, S_list{i}.n_hard_viol);
        max_v = max(max_v, S_list{i}.max_hard_viol);
        converged = converged && S_list{i}.all_converged;
    end
    S.diverged      = div;
    S.n_hard_viol   = n_v;
    S.max_hard_viol = max_v;
    S.all_converged = converged;
end

function v = safe_field(S, f)
    if isfield(S, f) && ~isempty(S.(f))
        v = S.(f);
    else
        v = NaN;
    end
end

function id = uuid_v4()
    a = dec2hex(randi([0,2^16-1]), 4);
    b = dec2hex(randi([0,2^16-1]), 4);
    c = dec2hex(randi([0,2^16-1]), 4);
    d = dec2hex(randi([0,2^16-1]), 4);
    e = dec2hex(randi([0,2^16-1]), 4);
    f = dec2hex(randi([0,2^16-1]), 4);
    g = dec2hex(randi([0,2^16-1]), 4);
    h = dec2hex(randi([0,2^16-1]), 4);
    id = lower(sprintf('%s%s-%s-%s-%s-%s%s%s', a, b, c, d, e, f, g, h));
end
