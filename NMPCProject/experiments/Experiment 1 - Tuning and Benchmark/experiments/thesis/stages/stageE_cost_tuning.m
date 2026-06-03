function report = stageE_cost_tuning(varargin)
% STAGEE_COST_TUNING  Stage E -- sequential cost-weight tuning.
%
%   report = stageE_cost_tuning()
%   report = stageE_cost_tuning('Orders', {'full12'})
%
% Sequentially tunes cost-function weights using the Stage-D pinned
% (N, Ts) = (10, 0.005 s) and Stage-C pinned IRK (1,1) integrator. Each
% sub-stage's WINNER (lowest J_integrated among CONVERGED_STABLE configs)
% feeds the next sub-stage as the new baseline.
%
% Sub-stages (per plan section 3, Stage E):
%
%   E1 -- R scaling
%       cfg.R = alpha_R * baseline_R
%       alpha_R in {0.1, 0.3, 1.0, 3.0, 10.0}                (5 cfgs/order)
%
%   E2 -- terminal weight (alpha_we)
%       cfg.alpha_we in {1, 3, 10, 30, 100, 300, 1000}        (7 cfgs/order)
%       at the E1 winner's R.
%
%   E3 -- Q sub-block scaling
%       Three groups: position-Q (idx 1:3), orientation-Q (4:6 of full;
%       4:5 of reduced -- yaw is removed), velocity-Q (7:12 of full;
%       6:10 of reduced).
%       Each group scaled independently by {0.3, 1.0, 3.0} -- 9 cfgs/order.
%       At E2 winner's (R, alpha_we).
%
%   E4 -- slack penalty
%       Zl_scale in {1, 10, 100}, zl_scale in {0.1, 1, 10}.
%       cfg.Zl = Zl_scale * baseline_Zl, cfg.zl = zl_scale * baseline_zl
%       (and likewise for upper). 9 cfgs/order.
%       At E3 winner's (R, alpha_we, Q-scale).
%
% Per plan, "best" is the minimum J_integrated among stable runs that are
% ALSO embedded_feasible_p99 -- if no embedded-feasible run is stable we
% fall back to the minimum-J stable run and emit a warning.
%
% Output struct report:
%   .table          row per run with sub_stage / variant / metrics
%   .winners        struct .full12 / .reduced10 -> per-substage winning cfg
%   .tuned          struct .full12 / .reduced10 -> recommended FINAL
%                   weights (the E4 winner)
%   .results_dir    path

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
    fprintf('  Stage E -- sequential cost tuning\n');
    fprintf('  orders: %s\n', strjoin(opts.Orders, ', '));
    fprintf('  pin:    N=10, Ts=5 ms, IRK(1,1), SQP_RTI + PC_HPIPM\n');
    fprintf('  out:    %s/E/<order>/\n', results_dir);
    fprintf('============================================================\n\n');

    verify_stage_D_passed_(repo_root);

    rows    = struct('row', {}, 'order', {});
    winners = struct();
    tuned   = struct();

    for k = 1:numel(opts.Orders)
        ord = opts.Orders{k};
        fprintf('\n--- ORDER: %s ---\n\n', ord);
        base = stageE_base_cfg(ord);

        % --- E1: R scaling -------------------------------------------------
        fprintf('E1: R scaling\n');
        winners_ord = struct();
        [rows_E1, win_E1] = run_substage_(base, ord, 'E1', ...
            make_E1_variants_(base), results_dir, opts);
        rows = [rows rows_E1]; %#ok<AGROW>
        if isempty(win_E1)
            warning('stageE:E1_no_winner','no winner in E1 for %s; aborting order', ord);
            continue;
        end
        base = win_E1.cfg;
        winners_ord.E1 = win_E1.descriptor;

        % --- E2: terminal weight ------------------------------------------
        fprintf('E2: terminal weight (alpha_we)\n');
        [rows_E2, win_E2] = run_substage_(base, ord, 'E2', ...
            make_E2_variants_(base), results_dir, opts);
        rows = [rows rows_E2]; %#ok<AGROW>
        if ~isempty(win_E2); base = win_E2.cfg; winners_ord.E2 = win_E2.descriptor; end

        % --- E3: Q sub-block scaling --------------------------------------
        fprintf('E3: Q sub-block scaling\n');
        [rows_E3, win_E3] = run_substage_(base, ord, 'E3', ...
            make_E3_variants_(base, ord), results_dir, opts);
        rows = [rows rows_E3]; %#ok<AGROW>
        if ~isempty(win_E3); base = win_E3.cfg; winners_ord.E3 = win_E3.descriptor; end

        % --- E4: slack penalty --------------------------------------------
        fprintf('E4: slack penalty\n');
        [rows_E4, win_E4] = run_substage_(base, ord, 'E4', ...
            make_E4_variants_(base), results_dir, opts);
        rows = [rows rows_E4]; %#ok<AGROW>
        if ~isempty(win_E4); base = win_E4.cfg; winners_ord.E4 = win_E4.descriptor; end

        winners.(ord) = winners_ord;
        tuned.(ord)   = struct('cfg', base, ...
                               'alpha_R',     get_or_default_(winners_ord, 'E1', '-'), ...
                               'alpha_we',    get_or_default_(winners_ord, 'E2', '-'), ...
                               'Q_scale',     get_or_default_(winners_ord, 'E3', '-'), ...
                               'slack_scale', get_or_default_(winners_ord, 'E4', '-'));
    end

    % Flatten rows
    if isempty(rows)
        report.table = table();
    else
        T = struct2table(vertcat(rows.row));
        report.table = T;
    end
    report.winners     = winners;
    report.tuned       = tuned;
    report.results_dir = results_dir;

    save(fullfile(results_dir, 'stageE_report.mat'), 'report', '-v7.3'); %#ok<NASGU>
    fprintf('\nSaved roll-up to %s/stageE_report.mat\n', results_dir);
    print_summary_(report);
end

% ====================================================================== %
% Stage D gate
% ====================================================================== %

function verify_stage_D_passed_(repo_root)
    p = fullfile(repo_root,'experiments','thesis','results','stageD_report.mat');
    if ~exist(p,'file')
        error('stageE:no_stageD', ...
            'Stage D has not been run. Run stageD_horizon_ts first.');
    end
    fprintf('Stage D gate: report present at %s\n\n', p);
end

% ====================================================================== %
% Sub-stage variant builders
% ====================================================================== %

function V = make_E1_variants_(base)
    alpha_R = [0.1 0.3 1.0 3.0 10.0];
    R0      = base.R(:);
    V = struct('label',{},'cfg',{},'descriptor',{});
    for k = 1:numel(alpha_R)
        c = base;
        c.R = alpha_R(k) * R0;
        V(end+1).label     = sprintf('aR_%g', alpha_R(k)); %#ok<AGROW>
        V(end).cfg         = c;
        V(end).descriptor  = struct('alpha_R', alpha_R(k));
    end
end

function V = make_E2_variants_(base)
    alpha_we = [1 3 10 30 100 300 1000];
    we0 = base.alpha_we;
    V = struct('label',{},'cfg',{},'descriptor',{});
    for k = 1:numel(alpha_we)
        c = base;
        c.alpha_we = alpha_we(k);
        V(end+1).label    = sprintf('awe_%g', alpha_we(k)); %#ok<AGROW>
        V(end).cfg        = c;
        V(end).descriptor = struct('alpha_we', alpha_we(k), 'baseline_alpha_we', we0);
    end
end

function V = make_E3_variants_(base, order)
    % Three sub-blocks indexed into the controller-state cost vector.
    if strcmp(order, 'full12')
        idx_pos = 1:3; idx_ori = 4:6; idx_vel = 7:12;
    else  % reduced10: yaw and yaw-rate removed (no idx 6 / idx 12)
        idx_pos = 1:3; idx_ori = 4:5; idx_vel = 6:10;
    end
    Q0 = base.Q(:);
    scales = [0.3 1.0 3.0];
    V = struct('label',{},'cfg',{},'descriptor',{});
    for group = {'pos','ori','vel'}
        switch group{1}
            case 'pos'; idx = idx_pos;
            case 'ori'; idx = idx_ori;
            case 'vel'; idx = idx_vel;
        end
        for s = scales
            c = base;
            c.Q = Q0;
            c.Q(idx) = s * Q0(idx);
            V(end+1).label    = sprintf('Q_%s_%g', group{1}, s); %#ok<AGROW>
            V(end).cfg        = c;
            V(end).descriptor = struct('Q_group', group{1}, 'Q_scale', s);
        end
    end
end

function V = make_E4_variants_(base)
    Zl0 = base.Zl(:); Zu0 = base.Zu(:); zl0 = base.zl(:); zu0 = base.zu(:);
    big_scales = [1 10 100];
    lin_scales = [0.1 1 10];
    V = struct('label',{},'cfg',{},'descriptor',{});
    for B = big_scales
        for L = lin_scales
            c = base;
            c.Zl = B * Zl0;  c.Zu = B * Zu0;
            c.zl = L * zl0;  c.zu = L * zu0;
            V(end+1).label    = sprintf('slack_Z%g_z%g', B, L); %#ok<AGROW>
            V(end).cfg        = c;
            V(end).descriptor = struct('Zl_scale', B, 'zl_scale', L);
        end
    end
end

% ====================================================================== %
% Sub-stage runner
% ====================================================================== %

function [rows, winner] = run_substage_(base, order, sub_stage, V, results_dir, opts)
    rows = struct('row', {}, 'order', {});
    winner = [];
    best.J = inf; best.idx = -1;
    best_fallback.J = inf; best_fallback.idx = -1;

    for k = 1:numel(V)
        v = V(k);
        cfg = v.cfg;
        cfg.stage   = 'E';
        cfg.exp_id  = sprintf('%s_%s', sub_stage, v.label);

        fprintf('  [%2d/%2d] %s | %s | %s\n', k, numel(V), order, sub_stage, v.label);
        try
            rec = run_one_config_(cfg, order, results_dir, opts);
            r   = row_from_record(rec, order, sub_stage, v);
            rows(end+1).row = r; rows(end).order = order; %#ok<AGROW>
            fprintf('         -> %s (t_p99=%.2f ms, J=%.4g)\n', ...
                    rec.classification, rec.summary.t_p99_ms, ...
                    safe_(rec.summary,'J_integrated'));

            if startsWith(rec.classification,'CONVERGED_STABLE')
                Ji = safe_(rec.summary,'J_integrated', inf);
                if rec.summary.embedded_feasible_p99 && Ji < best.J
                    best.J   = Ji;  best.idx = numel(rows);
                end
                if Ji < best_fallback.J
                    best_fallback.J = Ji; best_fallback.idx = numel(rows);
                end
            end
        catch ME
            fprintf('         FAIL build/run: %s\n', ME.message);
            rfail = build_fail_row(cfg, order, sub_stage, v, ME);
            rows(end+1).row = rfail; rows(end).order = order; %#ok<AGROW>
        end
    end

    if best.idx > 0
        winner = struct('cfg', V(best.idx).cfg, 'descriptor', V(best.idx).descriptor);
    elseif best_fallback.idx > 0
        warning('stageE:no_embedded_winner', ...
                'No embedded-feasible stable config in %s for %s; using min-J stable fallback.', ...
                sub_stage, order);
        winner = struct('cfg', V(best_fallback.idx).cfg, ...
                        'descriptor', V(best_fallback.idx).descriptor);
    end
end

% ====================================================================== %
% Per-config build + run + save
% ====================================================================== %

function rec = run_one_config_(cfg, order, results_dir, opts)
    cfg.order = order;
    cfg = validate_cfg(cfg, 'PrintDefaults', false);

    target = predict_save_path_(cfg, results_dir);
    if exist(target,'file') && ~opts.Force
        try
            rec = load_result(target);
            return
        catch ME
            warning('stageE:stale_record', ...
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
% Base cfg / row builders
% ====================================================================== %

function cfg = stageE_base_cfg(order)
    switch order
        case 'full12'
            cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
        case 'reduced10'
            raw = baseline_reduced10();
            raw.plant_override       = struct();
            raw.disturbance_schedule = [];
            cfg = validate_cfg(raw, 'PrintDefaults', false);
        otherwise
            error('stageE:bad_order','order must be full12 or reduced10 (got %s)', order);
    end
    cfg.allow_overwrite       = true;
    % Stage-D pin
    cfg.N         = 10;
    cfg.Tf        = 10 * 0.005;
    % Stage-C pin
    cfg.integrator_type       = 'IRK';
    cfg.sim_method_num_stages = 1;
    cfg.sim_method_num_steps  = 1;
    % Stage-B viable best
    cfg.nlp_solver_type       = 'SQP_RTI';
    cfg.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    cfg.hessian_approx        = 'GAUSS_NEWTON';
    cfg.regularize_method     = 'CONVEXIFY';
    cfg.qp_solver_warm_start  = 1;
end

function r = row_from_record(rec, order, sub_stage, v)
    s = rec.summary;
    r = struct( ...
        'order',          char_(order), ...
        'sub_stage',      sub_stage, ...
        'variant',        v.label, ...
        'descriptor',     dump_descriptor_(v.descriptor), ...
        'exp_id',         char_(rec.cfg.exp_id), ...
        'cfg_hash',       char_(rec.cfg_hash), ...
        'classification', char_(rec.classification), ...
        'reason',         char_(rec.reason), ...
        't_mean_ms',      safe_(s,'t_mean_ms'), ...
        't_p99_ms',       safe_(s,'t_p99_ms'), ...
        't_max_ms',       safe_(s,'t_max_ms'), ...
        'embedded_feasible_p99', logical(safe_(s,'embedded_feasible_p99', false)), ...
        'J_integrated',   safe_(s,'J_integrated'), ...
        'peak_pos_mm',    safe_(s,'peak_pos_mm'), ...
        'peak_ang_deg',   safe_(s,'peak_ang_deg'), ...
        'final_err_norm', safe_(s,'final_err_norm'), ...
        'build_failed',   false);
end

function r = build_fail_row(cfg, order, sub_stage, v, ME)
    r = struct( ...
        'order',          char_(order), ...
        'sub_stage',      sub_stage, ...
        'variant',        v.label, ...
        'descriptor',     dump_descriptor_(v.descriptor), ...
        'exp_id',         char_(cfg.exp_id), ...
        'cfg_hash',       '', ...
        'classification', 'BUILD_FAIL', ...
        'reason',         char_(ME.message), ...
        't_mean_ms',      NaN, ...
        't_p99_ms',       NaN, ...
        't_max_ms',       NaN, ...
        'embedded_feasible_p99', false, ...
        'J_integrated',   NaN, ...
        'peak_pos_mm',    NaN, ...
        'peak_ang_deg',   NaN, ...
        'final_err_norm', NaN, ...
        'build_failed',   true);
end

% ====================================================================== %
% Pretty summary
% ====================================================================== %

function print_summary_(report)
    fprintf('\n--- Winners ---\n');
    orders = fieldnames(report.winners);
    for i = 1:numel(orders)
        o = orders{i};
        fprintf(' %s:\n', o);
        w = report.winners.(o);
        if isfield(w,'E1'); fprintf('   E1 alpha_R          : %g\n', w.E1.alpha_R); end
        if isfield(w,'E2'); fprintf('   E2 alpha_we         : %g\n', w.E2.alpha_we); end
        if isfield(w,'E3'); fprintf('   E3 Q_group / scale  : %s / %g\n', w.E3.Q_group, w.E3.Q_scale); end
        if isfield(w,'E4'); fprintf('   E4 slack Z/z scales : %g / %g\n', w.E4.Zl_scale, w.E4.zl_scale); end
    end
    fprintf('\n');
end

% ====================================================================== %
% Helpers
% ====================================================================== %

function d = dump_descriptor_(s)
% Flatten a small descriptor struct into a single readable string.
    if isempty(s); d = ''; return; end
    fn = fieldnames(s); parts = cell(1, numel(fn));
    for k = 1:numel(fn)
        v = s.(fn{k});
        if ischar(v) || isstring(v); parts{k} = sprintf('%s=%s', fn{k}, v);
        else;                         parts{k} = sprintf('%s=%g', fn{k}, double(v));
        end
    end
    d = strjoin(parts, ', ');
end

function v = get_or_default_(s, fld, default)
    if isfield(s, fld); v = s.(fld); else; v = default; end
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

function root = repo_root_of_this_file()
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(fileparts(here)));
end
