function report = stageD_horizon_ts(varargin)
% STAGED_HORIZON_TS  Stage D -- horizon / timestep stability and embedded-
% feasibility map.
%
%   report = stageD_horizon_ts()
%   report = stageD_horizon_ts('Orders', {'full12'})
%
% Sweeps the (N, Ts) design space at the Stage-C-pinned integrator config
% (IRK, sim_method_num_stages=1, sim_method_num_steps=1) -- the only
% Pareto-optimal embedded-feasible point in Stage C. Plant integrator
% follows the matched-plant policy.
%
% Sub-grids (per plan section 3, Stage D):
%   D1 (horizon at baseline Ts):  Ts = 0.01 s fixed
%                                 N in {5, 8, 10, 12, 15, 20, 25, 30, 40}
%   D2 (Ts at three horizons):    N in {10, 20, 30}, Ts in
%                                 {0.003, 0.005, 0.007, 0.010, 0.015,
%                                  0.020, 0.030, 0.050},
%                                 filtered to Tf = N*Ts in [0.05, 0.5] s
%
% D3 (boundary refinement) is NOT automated here -- run it manually once
% the D1+D2 map identifies the stability boundary.
%
% Inputs ('Name', value):
%   'Orders'   cell array, subset of {'full12','reduced10'} (default both)
%   'Force'    logical, re-run if a saved record exists (default false)
%   'Verbose'  logical (default true)
%
% Output struct report:
%   .table         MATLAB table, one row per (N, Ts, order)
%   .results_dir   path

    p = inputParser();
    p.addParameter('Orders',  {'full12','reduced10'}, @iscellstr);
    p.addParameter('Force',   false, @islogical);
    p.addParameter('Verbose', true,  @islogical);
    p.parse(varargin{:});
    opts = p.Results;

    project_setup_thesis();
    repo_root   = repo_root_of_this_file();
    results_dir = fullfile(repo_root, 'experiments','thesis','results');
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    fprintf('\n============================================================\n');
    fprintf('  Stage D -- horizon / timestep grid\n');
    fprintf('  orders: %s\n', strjoin(opts.Orders, ', '));
    fprintf('  integrator pin: IRK (n_stages=1, n_steps=1)\n');
    fprintf('  out:    %s/D/<order>/\n', results_dir);
    fprintf('============================================================\n\n');

    verify_stage_C_passed_(repo_root);

    cfg_list = enumerate_stageD_configs(opts.Orders);
    fprintf('Planned configurations: %d\n\n', numel(cfg_list));

    rows = cell(numel(cfg_list), 1);
    for i = 1:numel(cfg_list)
        c0 = cfg_list(i);
        fprintf('[%3d/%3d] %s | %s | N=%d Ts=%.4f s (Tf=%.3f s)\n', ...
                i, numel(cfg_list), c0.order, c0.exp_id, c0.N, c0.Ts, c0.N*c0.Ts);
        try
            rec = run_one_config_(c0, results_dir, opts);
            rows{i} = row_from_record(rec, c0);
            fprintf('         -> %s (t_p99=%.2f ms, ratio=%.2f)\n', ...
                    rec.classification, rec.summary.t_p99_ms, ...
                    rec.summary.t_p99_ms / (c0.Ts*1000));
        catch ME
            fprintf('         FAIL build/run: %s\n', ME.message);
            rows{i} = build_fail_row(c0, ME);
        end
    end

    T = struct2table(vertcat(rows{~cellfun('isempty', rows)}));
    report = struct();
    report.table       = T;
    report.results_dir = results_dir;

    save(fullfile(results_dir, 'stageD_report.mat'), 'report', '-v7.3'); %#ok<NASGU>
    fprintf('\nSaved roll-up to %s/stageD_report.mat\n', results_dir);

    print_summary_(T);

    % Two figures per order: stability-region heatmap and embedded-feasibility
    try
        fig_dir = fullfile(results_dir, 'D', 'figures');
        if ~exist(fig_dir,'dir'); mkdir(fig_dir); end
        for k = 1:numel(opts.Orders)
            f1 = plot_stability_region(T, opts.Orders{k});
            p1 = fullfile(fig_dir, sprintf('stageD_stability_region_%s.fig', opts.Orders{k}));
            savefig(f1, p1); close(f1);

            f2 = plot_embedded_feasibility(T, opts.Orders{k});
            p2 = fullfile(fig_dir, sprintf('stageD_embedded_feasibility_%s.fig', opts.Orders{k}));
            savefig(f2, p2); close(f2);

            fprintf('  figures: %s, %s\n', p1, p2);
        end
    catch ME
        warning('stageD:figure_failed', 'figure generation failed: %s', ME.message);
    end
end

% ====================================================================== %
% Stage C gate
% ====================================================================== %

function verify_stage_C_passed_(repo_root)
    p = fullfile(repo_root,'experiments','thesis','results','stageC_report.mat');
    if ~exist(p,'file')
        error('stageD:no_stageC', ...
            'Stage C has not been run. Run stageC_integrator_depth first.');
    end
    fprintf('Stage C gate: report present at %s\n\n', p);
end

% ====================================================================== %
% cfg enumeration
% ====================================================================== %

function cfgs = enumerate_stageD_configs(orders)
    % D1: horizon sweep at fixed Ts = 0.01 s
    d1_Ts = 0.01;
    d1_N  = [5 8 10 12 15 20 25 30 40];

    % D2: Ts sweep at three reference horizons
    d2_N   = [10 20 30];
    d2_Ts  = [0.003 0.005 0.007 0.010 0.015 0.020 0.030 0.050];

    cfgs = repmat(empty_cfg_(), 0, 1);
    for ord = orders
        base = stageD_base_cfg(ord{1});

        % D1
        for n = d1_N
            c = base;
            c.stage    = 'D';
            c.N        = n;
            c.Tf       = n * d1_Ts;
            c.sim_steps = max(c.sim_steps, round(2.0 / d1_Ts));
            c.exp_id   = sprintf('D1_N%d_Ts%dus', n, round(d1_Ts*1e6));
            cfgs(end+1) = pack_cfg_(c, n, d1_Ts); %#ok<AGROW>
        end

        % D2
        for n = d2_N
            for ts = d2_Ts
                Tf = n * ts;
                if Tf < 0.05 || Tf > 0.5 + 1e-9; continue; end
                % Skip duplicate of D1 (N=20, Ts=0.01 etc.)
                if abs(ts - d1_Ts) < 1e-12 && ismember(n, d1_N); continue; end

                c = base;
                c.stage    = 'D';
                c.N        = n;
                c.Tf       = Tf;
                % Sim 2 s of physical time but at least 200 steps.
                c.sim_steps = max(200, round(2.0 / ts));
                c.exp_id   = sprintf('D2_N%d_Ts%dus', n, round(ts*1e6));
                cfgs(end+1) = pack_cfg_(c, n, ts); %#ok<AGROW>
            end
        end
    end
end

function c = empty_cfg_()
    c = struct('order','', 'N',0, 'Ts',0, 'exp_id','', 'cfg',[]);
end

function out = pack_cfg_(full_cfg, n, ts)
    out = struct( ...
        'order',  full_cfg.order, ...
        'N',      n, ...
        'Ts',     ts, ...
        'exp_id', full_cfg.exp_id, ...
        'cfg',    full_cfg);
end

function cfg = stageD_base_cfg(order)
    switch order
        case 'full12'
            cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
        case 'reduced10'
            raw = baseline_reduced10();
            raw.plant_override       = struct();
            raw.disturbance_schedule = [];
            cfg = validate_cfg(raw, 'PrintDefaults', false);
        otherwise
            error('stageD:bad_order','order must be full12 or reduced10 (got %s)', order);
    end
    cfg.allow_overwrite       = true;
    % Stage C pin
    cfg.integrator_type       = 'IRK';
    cfg.sim_method_num_stages = 1;
    cfg.sim_method_num_steps  = 1;
    cfg.nlp_solver_type       = 'SQP_RTI';
    cfg.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    cfg.hessian_approx        = 'GAUSS_NEWTON';
    cfg.regularize_method     = 'CONVEXIFY';
    cfg.qp_solver_warm_start  = 1;
end

% ====================================================================== %
% Per-config build + run + save
% ====================================================================== %

function rec = run_one_config_(c0, results_dir, opts)
    cfg = c0.cfg;
    cfg.stage  = 'D';
    cfg.exp_id = c0.exp_id;
    cfg = validate_cfg(cfg, 'PrintDefaults', false);

    target = predict_save_path_(cfg, results_dir);
    if exist(target,'file') && ~opts.Force
        try
            rec = load_result(target);
            return
        catch ME
            warning('stageD:stale_record', ...
                    'Found stale record at %s (%s); re-running.', target, ME.message);
        end
    end

    M = build_model(struct('order', cfg.order));
    [ocp_solver, ~, build_meta] = build_ocp(M, cfg);
    plant_solver = build_plant(M, cfg);
    R = run_closed_loop(ocp_solver, plant_solver, M, cfg);
    [klass, reason] = classify_outcome(R, M, cfg);
    rec = result_record(cfg, M, R, klass, reason, build_meta);
    rec.cfg.allow_overwrite = true;
    save_result(rec, results_dir);
end

function p = predict_save_path_(cfg, results_dir)
    short_hash = hash_cfg(cfg); short_hash = short_hash(1:8);
    fname = sprintf('%s__%s.mat', cfg.exp_id, short_hash);
    p     = fullfile(results_dir, cfg.stage, cfg.order, fname);
end

% ====================================================================== %
% Row builder
% ====================================================================== %

function r = row_from_record(rec, c0)
    s = rec.summary;
    Ts_ms = c0.Ts * 1000;
    r = struct( ...
        'order',          char_(c0.order), ...
        'N',              c0.N, ...
        'Ts',             c0.Ts, ...
        'Tf',             c0.N * c0.Ts, ...
        'exp_id',         char_(c0.exp_id), ...
        'cfg_hash',       char_(rec.cfg_hash), ...
        'classification', char_(rec.classification), ...
        'reason',         char_(rec.reason), ...
        'n_actual',       s.n_actual, ...
        'diverged',       logical(s.diverged), ...
        't_mean_ms',      safe_(s,'t_mean_ms'), ...
        't_p99_ms',       safe_(s,'t_p99_ms'), ...
        't_max_ms',       safe_(s,'t_max_ms'), ...
        't_p99_over_Ts',  safe_(s,'t_p99_ms') / Ts_ms, ...
        'embedded_feasible_p99', logical(safe_(s,'embedded_feasible_p99', false)), ...
        'J_integrated',   safe_(s,'J_integrated'), ...
        'peak_pos_mm',    safe_(s,'peak_pos_mm'), ...
        'peak_ang_deg',   safe_(s,'peak_ang_deg'), ...
        'final_err_norm', safe_(s,'final_err_norm'), ...
        'build_failed',   false);
end

function r = build_fail_row(c0, ME)
    Ts_ms = c0.Ts * 1000;
    r = struct( ...
        'order',          char_(c0.order), ...
        'N',              c0.N, ...
        'Ts',             c0.Ts, ...
        'Tf',             c0.N * c0.Ts, ...
        'exp_id',         char_(c0.exp_id), ...
        'cfg_hash',       '', ...
        'classification', 'BUILD_FAIL', ...
        'reason',         char_(ME.message), ...
        'n_actual',       0, ...
        'diverged',       false, ...
        't_mean_ms',      NaN, ...
        't_p99_ms',       NaN, ...
        't_max_ms',       NaN, ...
        't_p99_over_Ts',  NaN, ...
        'embedded_feasible_p99', false, ...
        'J_integrated',   NaN, ...
        'peak_pos_mm',    NaN, ...
        'peak_ang_deg',   NaN, ...
        'final_err_norm', NaN, ...
        'build_failed',   true);
end

% ====================================================================== %
% Summary
% ====================================================================== %

function print_summary_(T)
    fprintf('\n--- Outcome distribution ---\n');
    disp(groupcounts(T, {'order','classification'}))
    fprintf('\n--- Stable & embedded-feasible at p99 (full12) ---\n');
    sub = T(strcmp(T.order,'full12') & strcmp(T.classification,'CONVERGED_STABLE') ...
            & T.embedded_feasible_p99, {'N','Ts','t_p99_ms','J_integrated'});
    disp(sortrows(sub, {'Ts','N'}))
    fprintf('\n');
end

% ====================================================================== %
% Helpers
% ====================================================================== %

function root = repo_root_of_this_file()
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(fileparts(here)));
end

function v = safe_(s, name, default)
    if nargin < 3; default = NaN; end
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end

function s = char_(x)
    if isstring(x); s = char(x); elseif ischar(x); s = x; else; s = char(string(x)); end
end
