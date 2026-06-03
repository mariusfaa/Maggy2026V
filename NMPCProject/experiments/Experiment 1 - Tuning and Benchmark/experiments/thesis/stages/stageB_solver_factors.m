function report = stageB_solver_factors(varargin)
% STAGEB_SOLVER_FACTORS  Stage B -- solver-factor isolation (main-effect
% screening).
%
%   report = stageB_solver_factors()
%   report = stageB_solver_factors('Orders', {'full12'})
%   report = stageB_solver_factors('Force', true)
%
% Sweeps the six acados solver-side factors ONE AT A TIME starting from
% each model's baseline:
%
%   nlp_solver_type   : SQP, SQP_RTI                       (baseline RTI)
%   qp_solver         : PARTIAL_CONDENSING_HPIPM,
%                       FULL_CONDENSING_HPIPM,
%                       FULL_CONDENSING_QPOASES,
%                       PARTIAL_CONDENSING_QPOASES         (baseline PC-HPIPM)
%   integrator_type   : ERK, IRK, GNSF                      (baseline IRK)
%   hessian_approx    : GAUSS_NEWTON, EXACT                 (baseline GN)
%   regularize_method : CONVEXIFY, MIRROR,
%                       NO_REGULARIZE, PROJECT              (baseline CONVEXIFY)
%   qp_solver_warm_start : 0, 1, 2                          (baseline 1)
%
% For each (factor, level) pair we run THREE initial-condition variants:
%
%   ic_baseline   : scale 0.05, no disturbance (the workingSimulator IC)
%   ic_harder     : scale 0.15, no disturbance (3x perturbation amplitude)
%   ic_dist       : scale 0.05, with the reduced-baseline z-push at t=5s
%
% Both model orders are swept independently. Plant integrator follows the
% matched-plant policy (no plant_override).
%
% Inputs ('Name', value):
%   'Orders'   cell array, subset of {'full12','reduced10'} (default both)
%   'Force'    logical, re-run configs even if a saved result_record
%              already exists (default false; the existing record is
%              reused for resumability)
%   'Verbose'  logical (default true)
%
% Output struct report:
%   .table          MATLAB table, one row per (cfg, order) run with
%                   factor / level / ic / outcome / metrics columns
%   .viable         struct with .full12 and .reduced10, each carrying a
%                   per-factor list of levels that produced
%                   CONVERGED_STABLE on at least the baseline IC (consumed
%                   by stageC)
%   .results_dir    path
%
% Stage A's report (stageA_report.mat) is checked first. Stage B refuses
% to run if any of A1/A2/A3 failed.
%
% Run time: ~108 closed-loop runs (6 factors x 3 levels-after-baseline x
% 3 ICs x 2 orders, approximately, minus duplicates with the baseline).
% Each run is ~5-30 seconds depending on integrator -- expect 30-60 min
% total on first invocation; subsequent calls are instant if Force=false.

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
    fprintf('  Stage B -- solver-factor isolation\n');
    fprintf('  orders: %s\n', strjoin(opts.Orders, ', '));
    fprintf('  out:    %s/B/<order>/\n', results_dir);
    fprintf('============================================================\n\n');

    verify_stage_A_passed_(repo_root);

    cfg_list = enumerate_stageB_configs(opts.Orders);
    fprintf('Planned configurations: %d\n\n', numel(cfg_list));

    rows = cell(numel(cfg_list), 1);
    for i = 1:numel(cfg_list)
        c0 = cfg_list(i);
        fprintf('[%3d/%3d] %s | %s | %s=%s | ic=%s\n', i, numel(cfg_list), ...
                c0.order, c0.exp_id, c0.factor, c0.level, c0.ic_label);
        try
            rec = run_one_config_(c0, results_dir, opts);
            rows{i} = row_from_record(rec, c0);
            fprintf('         -> %s (t_p99=%.2f ms)\n', ...
                    rec.classification, rec.summary.t_p99_ms);
        catch ME
            fprintf('         FAIL build/run: %s\n', ME.message);
            rows{i} = build_fail_row(c0, ME);
        end
    end

    T = struct2table(vertcat(rows{~cellfun('isempty', rows)}));
    report = struct();
    report.table       = T;
    report.viable      = extract_viable_set_(T, opts.Orders);
    report.results_dir = results_dir;

    save(fullfile(results_dir, 'stageB_report.mat'), 'report', '-v7.3'); %#ok<NASGU>
    fprintf('\nSaved roll-up to %s/stageB_report.mat\n', results_dir);

    print_summary_(T, report.viable);

    % Main-effects figure (one panel per factor, per order).
    try
        fig_dir = fullfile(results_dir, 'B', 'figures');
        if ~exist(fig_dir,'dir'); mkdir(fig_dir); end
        for k = 1:numel(opts.Orders)
            fig = plot_main_effects(T, opts.Orders{k});
            fpath = fullfile(fig_dir, sprintf('stageB_main_effects_%s.fig', opts.Orders{k}));
            savefig(fig, fpath);
            close(fig);
            fprintf('  figure: %s\n', fpath);
        end
    catch ME
        warning('stageB:figure_failed', 'main-effects figure failed: %s', ME.message);
    end
end

% ====================================================================== %
% Stage A gate verification
% ====================================================================== %

function verify_stage_A_passed_(repo_root)
    p = fullfile(repo_root, 'experiments','thesis','results','stageA','stageA_report.mat');
    if ~exist(p,'file')
        error('stageB:no_stageA', ...
            ['Stage A has not been run. Run stageA_validate first and confirm ' ...
             'A1, A2, A3 pass before launching Stage B. (A4 is informational.)']);
    end
    S = load(p,'report');
    blockers = {'A1','A2','A3'};
    for k = 1:numel(blockers)
        if ~isfield(S.report, blockers{k})
            error('stageB:incomplete_stageA', 'Stage A is missing sub-test %s', blockers{k});
        end
        if ~S.report.(blockers{k}).passed
            error('stageB:stageA_failed', ...
                  'Stage A %s did NOT pass: %s', blockers{k}, S.report.(blockers{k}).details);
        end
    end
    fprintf('Stage A gate: PASSED (A1, A2, A3).\n\n');
end

% ====================================================================== %
% cfg enumeration
% ====================================================================== %

function cfgs = enumerate_stageB_configs(orders)
% Builds the struct array of all Stage-B cfg variants. The array is FLAT;
% the runner iterates and dispatches.

    factors = make_factor_table();
    ic_variants = make_ic_variants();

    cfgs = repmat(empty_cfg(), 0, 1);

    for ord = orders
        base_cfg = stageB_base_cfg(ord{1});

        for fi = 1:numel(factors)
            f = factors{fi};
            for li = 1:numel(f.levels)
                level = f.levels(li);
                for vi = 1:numel(ic_variants)
                    iv = ic_variants(vi);

                    c = base_cfg;
                    c.stage  = 'B';
                    c.factor = f.field;
                    c.level  = level.label;
                    c.ic_label = iv.label;

                    % Apply the factor variation.
                    c = apply_factor_(c, f.field, level.value);

                    % Apply the IC variant.
                    c.ic_descriptor.scale = iv.scale;
                    if iv.with_disturbance
                        c.disturbance_schedule = struct( ...
                            't_apply', 5.0, ...
                            'kind',    'state_additive', ...
                            'vector',  [0;0;0;0;0;0;-0.005;0.0025;-0.2;0;0;0]);
                        c.sim_steps = max(c.sim_steps, round(7.0 / (c.Tf / c.N))); % >=2s post-dist
                    end

                    c.exp_id = sprintf('B_%s_%s_%s', short_factor(f.field), ...
                                       short_level(level.label), iv.short);
                    c.allow_overwrite = true;  % Stage B is re-runnable

                    cfgs(end+1) = pack_cfg_(c, f.field, level.label, iv.label); %#ok<AGROW>
                end
            end
        end
    end
end

function c = empty_cfg()
    % MUST match pack_cfg_'s field set exactly (and order, to keep MATLAB
    % happy when growing the struct array via cfgs(end+1) = ...).
    c = struct('order','', 'factor','', 'level','', 'ic_label','', ...
               'exp_id','', 'cfg',[]);
end

function out = pack_cfg_(full_cfg, factor, level, ic_label)
% Wraps a full cfg with stage-B metadata for the iterator.
    out = struct( ...
        'order',    full_cfg.order, ...
        'factor',   factor, ...
        'level',    level, ...
        'ic_label', ic_label, ...
        'exp_id',   full_cfg.exp_id, ...
        'cfg',      full_cfg);
end

function c = apply_factor_(c, field, value)
    % Some factors only make sense for SQP; auto-correct.
    switch field
        case 'hessian_approx'
            if strcmp(value, 'EXACT') && strcmp(c.nlp_solver_type, 'SQP_RTI')
                % EXACT Hessian on RTI is non-standard; promote to SQP.
                c.nlp_solver_type = 'SQP';
            end
            c.hessian_approx = value;
        case 'integrator_type'
            c.integrator_type = value;
            if strcmp(value, 'ERK')
                % ERK uses explicit form; CSV num_stages is the RK order
                % (4-stage RK4 is the conventional default).
                c.sim_method_num_stages = 4;
                c.sim_method_num_steps  = c.sim_method_num_steps;
            elseif strcmp(value, 'GNSF')
                % GNSF requires an IRK-like setting; keep baseline stages/steps.
            end
        otherwise
            c.(field) = value;
    end
end

function f = make_factor_table()
    f{1} = struct('field','nlp_solver_type',    'levels', ...
                  struct_array({'SQP','SQP_RTI'}));
    % acados in the user's build accepts:
    %   PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES,
    %   PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    % (PARTIAL_CONDENSING_QPOASES is NOT supported -- it BUILD_FAILs).
    f{2} = struct('field','qp_solver',          'levels', ...
                  struct_array({'PARTIAL_CONDENSING_HPIPM','FULL_CONDENSING_HPIPM', ...
                                'FULL_CONDENSING_QPOASES'}));
    f{3} = struct('field','integrator_type',    'levels', ...
                  struct_array({'ERK','IRK','GNSF'}));
    f{4} = struct('field','hessian_approx',     'levels', ...
                  struct_array({'GAUSS_NEWTON','EXACT'}));
    f{5} = struct('field','regularize_method',  'levels', ...
                  struct_array({'CONVEXIFY','MIRROR','NO_REGULARIZE','PROJECT'}));
    f{6} = struct('field','qp_solver_warm_start','levels', ...
                  num_array_struct([0 1 2]));
end

function sa = struct_array(labels)
    sa = repmat(struct('label','','value',[]), 1, numel(labels));
    for k = 1:numel(labels)
        sa(k).label = labels{k};
        sa(k).value = labels{k};
    end
end

function sa = num_array_struct(values)
    sa = repmat(struct('label','','value',[]), 1, numel(values));
    for k = 1:numel(values)
        sa(k).label = num2str(values(k));
        sa(k).value = values(k);
    end
end

function v = make_ic_variants()
    v(1) = struct('label','ic_baseline',     'short','base', 'scale',0.05, 'with_disturbance',false);
    v(2) = struct('label','ic_harder',       'short','hard', 'scale',0.15, 'with_disturbance',false);
    v(3) = struct('label','ic_disturbance',  'short','dist', 'scale',0.05, 'with_disturbance',true);
end

function cfg = stageB_base_cfg(order)
% The Stage-B starting cfg for one model order. Equal to the framework
% baseline for that order EXCEPT it strips the plant_override (matched-
% plant policy applies from Stage B onward).
    switch order
        case 'full12'
            cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
        case 'reduced10'
            raw = baseline_reduced10();
            % Strip the Stage-A asymmetry so the matched-plant policy
            % governs Stage B onward.
            raw.plant_override = struct();
            raw.disturbance_schedule = [];
            cfg = validate_cfg(raw, 'PrintDefaults', false);
        otherwise
            error('stageB:bad_order','order must be full12 or reduced10 (got %s)', order);
    end
    cfg.allow_overwrite = true;
end

function s = short_factor(field)
    map = containers.Map( ...
        {'nlp_solver_type','qp_solver','integrator_type', ...
         'hessian_approx','regularize_method','qp_solver_warm_start'}, ...
        {'nlp','qp','int','hess','reg','ws'});
    s = map(field);
end

function s = short_level(label)
    % Shorten verbose acados names; keep recognisable.
    s = regexprep(label,'PARTIAL_CONDENSING_', 'PC_');
    s = regexprep(s,    'FULL_CONDENSING_',    'FC_');
    s = regexprep(s,    'GAUSS_NEWTON',        'GN');
    s = regexprep(s,    'NO_REGULARIZE',       'NOREG');
    s = regexprep(s,    'SQP_RTI',             'RTI');
    s = regexprep(s,    '[^A-Za-z0-9_]','_');
    if numel(s) > 16; s = s(1:16); end
end

% ====================================================================== %
% Per-config build + run + save
% ====================================================================== %

function rec = run_one_config_(c0, results_dir, opts)
    cfg = c0.cfg;
    cfg.stage  = 'B';
    cfg.exp_id = c0.exp_id;
    cfg = validate_cfg(cfg, 'PrintDefaults', false);

    % Predict the save path. If a record already exists for this cfg_hash
    % and Force is false, skip building.
    target = predict_save_path_(cfg, results_dir);
    if exist(target,'file') && ~opts.Force
        try
            rec = load_result(target);
            return
        catch ME
            warning('stageB:stale_record', ...
                    'Found stale record at %s (%s); re-running.', target, ME.message);
        end
    end

    M = build_model(struct('order', cfg.order));
    [ocp_solver, ~, build_meta] = build_ocp(M, cfg);

    % Plant solver is fetched via build_plant's own persistent cache;
    % distinct plant settings get distinct codegen subdirectories so a
    % rebuild never collides with a DLL already loaded in this session.
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
% Aggregation
% ====================================================================== %

function r = row_from_record(rec, c0)
    s = rec.summary;
    r = struct( ...
        'order',          char_(c0.order), ...
        'factor',         char_(c0.factor), ...
        'level',          char_(c0.level), ...
        'ic_label',       char_(c0.ic_label), ...
        'exp_id',         char_(c0.exp_id), ...
        'cfg_hash',       char_(rec.cfg_hash), ...
        'classification', char_(rec.classification), ...
        'reason',         char_(rec.reason), ...
        'n_actual',       s.n_actual, ...
        'diverged',       logical(s.diverged), ...
        'all_converged',  logical(getf_(s,'all_converged',false)), ...
        't_mean_ms',      getf_(s,'t_mean_ms',NaN), ...
        't_p99_ms',       getf_(s,'t_p99_ms',NaN), ...
        't_max_ms',       getf_(s,'t_max_ms',NaN), ...
        'embedded_feasible_p99', logical(getf_(s,'embedded_feasible_p99',false)), ...
        'J_integrated',   getf_(s,'J_integrated',NaN), ...
        'peak_pos_mm',    getf_(s,'peak_pos_mm',NaN), ...
        'peak_ang_deg',   getf_(s,'peak_ang_deg',NaN), ...
        'final_err_norm', getf_(s,'final_err_norm',NaN), ...
        'sqp_iter_mean',  getf_(s,'sqp_iter_mean',NaN), ...
        'qp_iter_mean',   getf_(s,'qp_iter_mean',NaN), ...
        'dist_recovery_time_ms', getf_(s,'dist_recovery_time_ms',NaN), ...
        'dist_max_z_deviation_post_mm', getf_(s,'dist_max_z_deviation_post_mm',NaN), ...
        'build_failed',   false);
end

function r = build_fail_row(c0, ME)
    r = struct( ...
        'order',c0.order,'factor',c0.factor,'level',c0.level, ...
        'ic_label',c0.ic_label,'exp_id',c0.exp_id, ...
        'cfg_hash','', 'classification','BUILD_FAIL', 'reason',ME.message, ...
        'n_actual',0,'diverged',true,'all_converged',false, ...
        't_mean_ms',NaN,'t_p99_ms',NaN,'t_max_ms',NaN, ...
        'embedded_feasible_p99',false, ...
        'J_integrated',NaN,'peak_pos_mm',NaN,'peak_ang_deg',NaN, ...
        'final_err_norm',NaN,'sqp_iter_mean',NaN,'qp_iter_mean',NaN, ...
        'dist_recovery_time_ms',NaN,'dist_max_z_deviation_post_mm',NaN, ...
        'build_failed',true);
end

function viable = extract_viable_set_(T, orders)
% For each order x factor x level, the level is "viable" iff the
% baseline-IC run had classification CONVERGED_STABLE.
    viable = struct();
    for ki = 1:numel(orders)
        ord = orders{ki};
        rows_o = T(strcmp(T.order, ord) & strcmp(T.ic_label, 'ic_baseline'), :);
        factors = unique(rows_o.factor);
        S = struct();
        for fi = 1:numel(factors)
            f = factors{fi};
            sub = rows_o(strcmp(rows_o.factor, f), :);
            ok = startsWith(string(sub.classification), 'CONVERGED_STABLE');
            S.(safefield(f)) = string(sub.level(ok));
        end
        viable.(ord) = S;
    end
end

function f = safefield(s)
    f = regexprep(s,'[^A-Za-z0-9_]','_');
end

% ====================================================================== %
% Reporting
% ====================================================================== %

function print_summary_(T, viable)
    fprintf('\n--- Stage B summary ---\n');
    orders = unique(T.order);
    for ki = 1:numel(orders)
        ord = orders{ki};
        rows_o = T(strcmp(T.order, ord), :);
        n_total = height(rows_o);
        n_stable = sum(startsWith(string(rows_o.classification),'CONVERGED_STABLE'));
        fprintf('  %s: %d runs, %d stable (%.1f%%)\n', ord, n_total, n_stable, 100*n_stable/n_total);
    end
    fprintf('\n--- Viable factor levels (baseline IC, per order) ---\n');
    fn = fieldnames(viable);
    for ki = 1:numel(fn)
        ord = fn{ki};
        sub = viable.(ord);
        fprintf('  %s\n', ord);
        ffs = fieldnames(sub);
        for fi = 1:numel(ffs)
            lvls = sub.(ffs{fi});
            if isempty(lvls); s = '(none)';
            else;             s = strjoin(cellstr(lvls), ', '); end
            fprintf('    %-22s : %s\n', ffs{fi}, s);
        end
    end
    fprintf('\n');
end

% ====================================================================== %
% Helpers
% ====================================================================== %

function r = repo_root_of_this_file()
    here = fileparts(mfilename('fullpath'));
    r    = fileparts(fileparts(fileparts(here)));
end

function v = getf_(s, fn, default_)
    if isfield(s, fn) && ~isempty(s.(fn)); v = s.(fn); else; v = default_; end
end

function c = char_(s)
    if isstring(s); c = char(s);
    elseif ischar(s); c = s;
    elseif isnumeric(s); c = num2str(s);
    else; c = ''; end
end
