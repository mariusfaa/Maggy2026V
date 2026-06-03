function report = stageC_integrator_depth(varargin)
% STAGEC_INTEGRATOR_DEPTH  Stage C -- integrator discretization-depth study.
%
%   report = stageC_integrator_depth()
%   report = stageC_integrator_depth('Orders', {'full12'})
%   report = stageC_integrator_depth('Force', true)
%
% For each integrator type that survived Stage B at baseline IC (ERK, IRK,
% GNSF), sweep the discretization depth and trace the accuracy-vs-speed
% frontier. Both model orders are swept independently. Plant integrator
% follows the matched-plant policy (no plant_override), so each Pareto
% point represents the closed-loop performance of a single internally
% consistent (OCP, plant) integrator choice.
%
% Sub-studies (per plan section 3, Stage C):
%   C1 IRK   : (n_stages, n_steps) in {1,2,3,4,6} x {1,2,3,5,10} = 25 cfgs
%   C2 ERK   : n_steps in {1,2,3,5,10} at n_stages=4               =  5 cfgs
%   C3 GNSF  : n_steps in {1,2,3,5,10} at n_stages=4               =  5 cfgs
% Total: 35 cfgs per order x 2 orders = 70 runs.
%
% All other solver factors are pinned to the Stage-B viable best:
%   nlp_solver_type      = SQP_RTI
%   qp_solver            = PARTIAL_CONDENSING_HPIPM
%   hessian_approx       = GAUSS_NEWTON
%   regularize_method    = CONVEXIFY
%   qp_solver_warm_start = 1
%
% Inputs ('Name', value):
%   'Orders'   cell array, subset of {'full12','reduced10'} (default both)
%   'Force'    logical, re-run configs even if a saved record exists
%   'Verbose'  logical (default true)
%
% Output struct report:
%   .table          MATLAB table, one row per (integrator, stages, steps, order)
%   .min_viable     struct .full12 / .reduced10 -> per-integrator minimum
%                   (stages, steps) that produced CONVERGED_STABLE at baseline IC
%   .results_dir    path
%
% Stage B's report (stageB_report.mat) is checked first; Stage C refuses to
% run if Stage B was not produced.

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
    fprintf('  Stage C -- integrator depth\n');
    fprintf('  orders: %s\n', strjoin(opts.Orders, ', '));
    fprintf('  out:    %s/C/<order>/\n', results_dir);
    fprintf('============================================================\n\n');

    verify_stage_B_passed_(repo_root);

    cfg_list = enumerate_stageC_configs(opts.Orders);
    fprintf('Planned configurations: %d\n\n', numel(cfg_list));

    rows = cell(numel(cfg_list), 1);
    for i = 1:numel(cfg_list)
        c0 = cfg_list(i);
        fprintf('[%3d/%3d] %s | %s | %s s=%d t=%d\n', i, numel(cfg_list), ...
                c0.order, c0.exp_id, c0.integrator_type, c0.n_stages, c0.n_steps);
        try
            rec = run_one_config_(c0, results_dir, opts);
            rows{i} = row_from_record(rec, c0);
            fprintf('         -> %s (t_p99=%.2f ms, J=%.3g)\n', ...
                    rec.classification, rec.summary.t_p99_ms, ...
                    safe_(rec.summary, 'J_integrated'));
        catch ME
            fprintf('         FAIL build/run: %s\n', ME.message);
            rows{i} = build_fail_row(c0, ME);
        end
    end

    T = struct2table(vertcat(rows{~cellfun('isempty', rows)}));
    report = struct();
    report.table       = T;
    report.min_viable  = extract_min_viable_(T, opts.Orders);
    report.results_dir = results_dir;

    save(fullfile(results_dir, 'stageC_report.mat'), 'report', '-v7.3'); %#ok<NASGU>
    fprintf('\nSaved roll-up to %s/stageC_report.mat\n', results_dir);

    print_summary_(T, report.min_viable);

    % Pareto figures (one panel per integrator, per order).
    try
        fig_dir = fullfile(results_dir, 'C', 'figures');
        if ~exist(fig_dir,'dir'); mkdir(fig_dir); end
        for k = 1:numel(opts.Orders)
            fig = plot_integrator_pareto(T, opts.Orders{k});
            fpath = fullfile(fig_dir, sprintf('stageC_pareto_%s.fig', opts.Orders{k}));
            savefig(fig, fpath);
            close(fig);
            fprintf('  figure: %s\n', fpath);
        end
    catch ME
        warning('stageC:figure_failed', 'pareto figure failed: %s', ME.message);
    end
end

% ====================================================================== %
% Stage B gate
% ====================================================================== %

function verify_stage_B_passed_(repo_root)
    p = fullfile(repo_root,'experiments','thesis','results','stageB_report.mat');
    if ~exist(p,'file')
        error('stageC:no_stageB', ...
            ['Stage B has not been run. Run stageB_solver_factors first ' ...
             'and confirm baseline anchors are CONVERGED_STABLE before launching Stage C.']);
    end
    fprintf('Stage B gate: report present at %s\n\n', p);
end

% ====================================================================== %
% cfg enumeration
% ====================================================================== %

function cfgs = enumerate_stageC_configs(orders)
    % IRK grid
    irk_stages = [1 2 3 4 6];
    irk_steps  = [1 2 3 5 10];

    % ERK / GNSF: 1D sweep over n_steps at fixed n_stages = 4
    erk_steps  = [1 2 3 5 10];
    gnsf_steps = [1 2 3 5 10];

    cfgs = repmat(empty_cfg_(), 0, 1);
    for ord = orders
        base = stageC_base_cfg(ord{1});

        % C1: IRK grid
        for s = irk_stages
            for k = irk_steps
                c = base;
                c.stage              = 'C';
                c.integrator_type    = 'IRK';
                c.sim_method_num_stages = s;
                c.sim_method_num_steps  = k;
                c.exp_id = sprintf('C_IRK_s%d_t%d', s, k);
                cfgs(end+1) = pack_cfg_(c, 'IRK', s, k); %#ok<AGROW>
            end
        end

        % C2: ERK sweep
        for k = erk_steps
            c = base;
            c.stage              = 'C';
            c.integrator_type    = 'ERK';
            c.sim_method_num_stages = 4;
            c.sim_method_num_steps  = k;
            c.exp_id = sprintf('C_ERK_s4_t%d', k);
            cfgs(end+1) = pack_cfg_(c, 'ERK', 4, k); %#ok<AGROW>
        end

        % C3: GNSF sweep
        for k = gnsf_steps
            c = base;
            c.stage              = 'C';
            c.integrator_type    = 'GNSF';
            c.sim_method_num_stages = 4;
            c.sim_method_num_steps  = k;
            c.exp_id = sprintf('C_GNSF_s4_t%d', k);
            cfgs(end+1) = pack_cfg_(c, 'GNSF', 4, k); %#ok<AGROW>
        end
    end
end

function c = empty_cfg_()
    c = struct('order','', 'integrator_type','', 'n_stages',0, 'n_steps',0, ...
               'exp_id','', 'cfg',[]);
end

function out = pack_cfg_(full_cfg, itype, s, k)
    out = struct( ...
        'order',           full_cfg.order, ...
        'integrator_type', itype, ...
        'n_stages',        s, ...
        'n_steps',         k, ...
        'exp_id',          full_cfg.exp_id, ...
        'cfg',             full_cfg);
end

function cfg = stageC_base_cfg(order)
% Stage-C starting cfg: baseline with all solver factors pinned to the
% Stage-B viable best (which IS the baseline). plant_override stripped so
% the matched-plant policy applies.
    switch order
        case 'full12'
            cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
        case 'reduced10'
            raw = baseline_reduced10();
            raw.plant_override        = struct();
            raw.disturbance_schedule  = [];
            cfg = validate_cfg(raw, 'PrintDefaults', false);
        otherwise
            error('stageC:bad_order','order must be full12 or reduced10 (got %s)', order);
    end
    cfg.allow_overwrite       = true;
    cfg.nlp_solver_type       = 'SQP_RTI';
    cfg.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    cfg.hessian_approx        = 'GAUSS_NEWTON';
    cfg.regularize_method     = 'CONVEXIFY';
    cfg.qp_solver_warm_start  = 1;
end

% ====================================================================== %
% Per-config build + run + save (mirror of stageB)
% ====================================================================== %

function rec = run_one_config_(c0, results_dir, opts)
    cfg = c0.cfg;
    cfg.stage  = 'C';
    cfg.exp_id = c0.exp_id;
    cfg = validate_cfg(cfg, 'PrintDefaults', false);

    target = predict_save_path_(cfg, results_dir);
    if exist(target,'file') && ~opts.Force
        try
            rec = load_result(target);
            return
        catch ME
            warning('stageC:stale_record', ...
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
    r = struct( ...
        'order',          char_(c0.order), ...
        'integrator_type',char_(c0.integrator_type), ...
        'n_stages',       c0.n_stages, ...
        'n_steps',        c0.n_steps, ...
        'exp_id',         char_(c0.exp_id), ...
        'cfg_hash',       char_(rec.cfg_hash), ...
        'classification', char_(rec.classification), ...
        'reason',         char_(rec.reason), ...
        'n_actual',       s.n_actual, ...
        'diverged',       logical(s.diverged), ...
        't_mean_ms',      safe_(s,'t_mean_ms'), ...
        't_p99_ms',       safe_(s,'t_p99_ms'), ...
        't_max_ms',       safe_(s,'t_max_ms'), ...
        'embedded_feasible_p99', logical(safe_(s,'embedded_feasible_p99', false)), ...
        'J_integrated',   safe_(s,'J_integrated'), ...
        'peak_pos_mm',    safe_(s,'peak_pos_mm'), ...
        'peak_ang_deg',   safe_(s,'peak_ang_deg'), ...
        'final_err_norm', safe_(s,'final_err_norm'), ...
        'sqp_iter_mean',  safe_(s,'sqp_iter_mean'), ...
        'qp_iter_mean',   safe_(s,'qp_iter_mean'), ...
        'build_failed',   false);
end

function r = build_fail_row(c0, ME)
    r = struct( ...
        'order',          char_(c0.order), ...
        'integrator_type',char_(c0.integrator_type), ...
        'n_stages',       c0.n_stages, ...
        'n_steps',        c0.n_steps, ...
        'exp_id',         char_(c0.exp_id), ...
        'cfg_hash',       '', ...
        'classification', 'BUILD_FAIL', ...
        'reason',         char_(ME.message), ...
        'n_actual',       0, ...
        'diverged',       false, ...
        't_mean_ms',      NaN, ...
        't_p99_ms',       NaN, ...
        't_max_ms',       NaN, ...
        'embedded_feasible_p99', false, ...
        'J_integrated',   NaN, ...
        'peak_pos_mm',    NaN, ...
        'peak_ang_deg',   NaN, ...
        'final_err_norm', NaN, ...
        'sqp_iter_mean',  NaN, ...
        'qp_iter_mean',   NaN, ...
        'build_failed',   true);
end

% ====================================================================== %
% Min-viable extraction
% ====================================================================== %

function mv = extract_min_viable_(T, orders)
% For each (order, integrator_type), find the (stages, steps) entry with
% the smallest steps*stages product that classified CONVERGED_STABLE.
    mv = struct();
    for k = 1:numel(orders)
        o = orders{k};
        sub = T(strcmp(T.order, o) & strcmp(T.classification, 'CONVERGED_STABLE'), :);
        types = unique(sub.integrator_type, 'stable');
        per_type = struct();
        for ti = 1:numel(types)
            t = types{ti};
            st = sub(strcmp(sub.integrator_type, t), :);
            if isempty(st); continue; end
            [~, idx] = min(st.n_stages .* st.n_steps);
            per_type.(t) = struct( ...
                'n_stages', st.n_stages(idx), ...
                'n_steps',  st.n_steps(idx), ...
                't_p99_ms', st.t_p99_ms(idx), ...
                'J',        st.J_integrated(idx));
        end
        mv.(o) = per_type;
    end
end

% ====================================================================== %
% Pretty summary
% ====================================================================== %

function print_summary_(T, mv)
    fprintf('\n--- Outcome distribution ---\n');
    disp(groupcounts(T, {'order','integrator_type','classification'}))
    fprintf('\n--- Minimum viable (stages, steps) per integrator ---\n');
    orders = fieldnames(mv);
    for i = 1:numel(orders)
        o = orders{i};
        ts = fieldnames(mv.(o));
        for j = 1:numel(ts)
            e = mv.(o).(ts{j});
            fprintf('  %-10s %-5s   stages=%d steps=%d   t_p99=%.2f ms   J=%.3g\n', ...
                    o, ts{j}, e.n_stages, e.n_steps, e.t_p99_ms, e.J);
        end
    end
    fprintf('\n');
end

% ====================================================================== %
% Helpers
% ====================================================================== %

function root = repo_root_of_this_file()
    % this file lives at experiments/thesis/stages/ -- climb three levels
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
