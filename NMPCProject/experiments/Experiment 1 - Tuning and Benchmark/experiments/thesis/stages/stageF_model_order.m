function report = stageF_model_order(varargin)
% STAGEF_MODEL_ORDER  Stage F -- model-order comparison panel (the central
% thesis comparison: full-12 vs reduced-10).
%
%   report = stageF_model_order()
%
% Assembles a comparison panel of 6 configurations derived from Stages
% A-E, runs each on BOTH model orders with three ICs (baseline / harder /
% disturbance), and produces a head-to-head table and trajectory overlay.
%
% Panel:
%
%   F1  baseline_full12 / baseline_reduced10
%       The Stage-A reproduction configs. Defines the "before-tuning"
%       reference.
%
%   F2  C-winner       (Stage-C integrator pin, otherwise baseline)
%       IRK n_stages=1 n_steps=1, everything else at baseline weights.
%
%   F3  D-winner       (Stage-D horizon/Ts pin)
%       Adds N=10, Ts=5 ms on top of F2.
%
%   F4  E-winner       (Stage-E tuned weights, per order)
%       Adds the per-order tuned cost weights on top of F3. This is the
%       FULLY TUNED config -- the per-order recommendation.
%
%   F5  Aggressive embedded
%       F4 + tightest stable Ts found in Stage D (the embedded-frontier
%       config). Probes "smallest deployable period".
%
%   F6  Reduced-only candidate
%       A deliberately aggressive config (IRK 1/1, very short Ts, slack
%       relaxed) that we expect reduced-10 to survive but full-12 to fail.
%       The point is to demonstrate where reduced-order is the only
%       feasible choice.
%
% For each of the 6 panel entries x 2 orders x 3 IC variants we run a
% closed-loop simulation and aggregate metrics. Output:
%
%   report.table        long-form table with per-(panel,order,ic) row
%   report.summary_tab  wide-form table for the thesis (per-panel rows,
%                       order/IC columns)
%   report.results_dir  path

    p = inputParser();
    p.addParameter('Force',   false, @islogical);
    p.addParameter('Verbose', true,  @islogical);
    p.parse(varargin{:});
    opts = p.Results;

    project_setup_thesis();
    repo_root   = repo_root_of_this_file();
    results_dir = fullfile(repo_root, 'experiments','thesis','results');
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    fprintf('\n============================================================\n');
    fprintf('  Stage F -- model-order panel comparison\n');
    fprintf('  out:    %s/F/<order>/\n', results_dir);
    fprintf('============================================================\n\n');

    verify_stage_E_passed_(repo_root);

    panel = build_panel_(repo_root);
    ic_variants = make_ic_variants_();
    orders = {'full12','reduced10'};

    rows = {};
    for pi = 1:numel(panel)
        P = panel(pi);
        for oi = 1:numel(orders)
            ord = orders{oi};
            for vi = 1:numel(ic_variants)
                iv = ic_variants(vi);

                cfg = apply_panel_(P, ord);
                cfg.stage    = 'F';
                cfg.exp_id   = sprintf('F_%s_%s_%s', P.tag, ord(1:3), iv.short);
                cfg.ic_descriptor.scale = iv.scale;
                if iv.with_disturbance
                    cfg.disturbance_schedule = struct( ...
                        't_apply', 5.0, 'kind','state_additive', ...
                        'vector', [0;0;0;0;0;0;-0.005;0.0025;-0.2;0;0;0]);
                    cfg.sim_steps = max(cfg.sim_steps, round(7.0 / (cfg.Tf/cfg.N)));
                else
                    cfg.disturbance_schedule = [];
                end
                cfg.allow_overwrite = true;

                fprintf('[F.%d:%s | %s | %s]\n', pi, P.tag, ord, iv.label);
                try
                    rec = run_one_config_(cfg, ord, results_dir, opts);
                    rows{end+1} = row_from_record(rec, P, ord, iv); %#ok<AGROW>
                    fprintf('   -> %s  t_p99=%.2f ms  J=%.4g\n', ...
                            rec.classification, rec.summary.t_p99_ms, ...
                            safe_(rec.summary,'J_integrated'));
                catch ME
                    fprintf('   FAIL: %s\n', ME.message);
                    rows{end+1} = build_fail_row(cfg, P, ord, iv, ME); %#ok<AGROW>
                end
            end
        end
    end

    T = struct2table(vertcat(rows{:}));
    report = struct();
    report.table       = T;
    report.summary_tab = build_summary_tab_(T);
    report.results_dir = results_dir;

    save(fullfile(results_dir,'stageF_report.mat'), 'report', '-v7.3'); %#ok<NASGU>
    fprintf('\nSaved roll-up to %s/stageF_report.mat\n', results_dir);

    print_summary_(report);
end

% ====================================================================== %
% Stage E gate
% ====================================================================== %

function verify_stage_E_passed_(repo_root)
    p = fullfile(repo_root,'experiments','thesis','results','stageE_report.mat');
    if ~exist(p,'file')
        error('stageF:no_stageE', ...
            'Stage E has not been run. Run stageE_cost_tuning first.');
    end
    fprintf('Stage E gate: report present at %s\n\n', p);
end

% ====================================================================== %
% Panel & IC variants
% ====================================================================== %

function panel = build_panel_(repo_root)
% Each panel entry knows how to transform a baseline cfg into the panel
% variant; "apply" is a function-handle that takes (cfg, order) and
% returns the modified cfg.
    se = load(fullfile(repo_root,'experiments','thesis','results','stageE_report.mat'),'report');
    if ~isfield(se,'report') || ~isfield(se.report,'tuned')
        error('stageF:bad_stageE','Stage E report missing tuned field');
    end
    tuned = se.report.tuned;

    panel(1) = struct('tag','F1_baseline', 'apply', @(c,o) c);
    panel(2) = struct('tag','F2_Cwinner',  'apply', @apply_Cwinner_);
    panel(3) = struct('tag','F3_Dwinner',  'apply', @apply_Dwinner_);
    panel(4) = struct('tag','F4_Ewinner',  'apply', @(c,o) apply_Ewinner_(c, o, tuned));
    panel(5) = struct('tag','F5_aggressive_embedded', ...
                      'apply', @(c,o) apply_aggressive_embedded_(c, o, tuned));
    panel(6) = struct('tag','F6_reduced_only', ...
                      'apply', @(c,o) apply_reduced_only_probe_(c, o, tuned));
end

function v = make_ic_variants_()
    v(1) = struct('label','ic_baseline',    'short','base', 'scale',0.05, 'with_disturbance',false);
    v(2) = struct('label','ic_harder',      'short','hard', 'scale',0.15, 'with_disturbance',false);
    v(3) = struct('label','ic_disturbance', 'short','dist', 'scale',0.05, 'with_disturbance',true);
end

% --- panel transforms ---

function c = apply_Cwinner_(c, ~)
    c.integrator_type       = 'IRK';
    c.sim_method_num_stages = 1;
    c.sim_method_num_steps  = 1;
end

function c = apply_Dwinner_(c, ~)
    c = apply_Cwinner_(c);
    c.N  = 10;
    c.Tf = 10 * 0.005;
end

function c = apply_Ewinner_(c, order, tuned)
    c = apply_Dwinner_(c);
    if ~isfield(tuned, order); return; end
    tw = tuned.(order).cfg;
    % Lift the cost weights and slacks from the tuned cfg.
    c.R         = tw.R(:);
    c.Q         = tw.Q(:);
    c.alpha_we  = tw.alpha_we;
    c.Zl        = tw.Zl(:);
    c.Zu        = tw.Zu(:);
    c.zl        = tw.zl(:);
    c.zu        = tw.zu(:);
end

function c = apply_aggressive_embedded_(c, order, tuned)
    c = apply_Ewinner_(c, order, tuned);
    % Push Ts down to 3 ms (the tightest in Stage D's grid). Keep Tf in
    % range by also pulling N down.
    c.N  = 20;
    c.Tf = 20 * 0.003;
end

function c = apply_reduced_only_probe_(c, order, tuned)
    c = apply_Ewinner_(c, order, tuned);
    % Aggressive: very low N, very fine Ts, slack relaxed.
    c.N  = 5;
    c.Tf = 5 * 0.003;
    c.Zl = 0.01 * c.Zl;
    c.Zu = 0.01 * c.Zu;
end

% ====================================================================== %
% Per-config build + run + save
% ====================================================================== %

function rec = run_one_config_(cfg, order, results_dir, opts)
    % Apply order first so cfg has the right ic_descriptor / state-size
    % baseline; then validate. (apply_panel already returns the right cfg.)
    cfg.order = order;
    cfg = validate_cfg(cfg, 'PrintDefaults', false);

    target = predict_save_path_(cfg, results_dir);
    if exist(target,'file') && ~opts.Force
        try
            rec = load_result(target);
            return
        catch ME
            warning('stageF:stale_record', ...
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

function cfg = apply_panel_(P, order)
% Build the per-order base cfg and apply the panel transform.
    switch order
        case 'full12'
            cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
        case 'reduced10'
            raw = baseline_reduced10();
            raw.plant_override       = struct();
            raw.disturbance_schedule = [];
            cfg = validate_cfg(raw, 'PrintDefaults', false);
        otherwise
            error('stageF:bad_order','order must be full12 or reduced10');
    end
    cfg.allow_overwrite = true;
    cfg = P.apply(cfg, order);
end

% ====================================================================== %
% Row + summary
% ====================================================================== %

function r = row_from_record(rec, P, order, iv)
    s = rec.summary;
    r = struct( ...
        'panel',          char_(P.tag), ...
        'order',          char_(order), ...
        'ic_label',       char_(iv.label), ...
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
        'settling_2pct_ms', safe_(s,'settling_2pct_ms'), ...
        'final_err_norm',   safe_(s,'final_err_norm'), ...
        'dist_recovery_time_ms', safe_(s,'dist_recovery_time_ms'), ...
        'build_failed',   false);
end

function r = build_fail_row(cfg, P, order, iv, ME)
    r = struct( ...
        'panel',          char_(P.tag), ...
        'order',          char_(order), ...
        'ic_label',       char_(iv.label), ...
        'exp_id',         char_(cfg.exp_id), ...
        'cfg_hash',       '', ...
        'classification', 'BUILD_FAIL', ...
        'reason',         char_(ME.message), ...
        't_mean_ms',      NaN, 't_p99_ms', NaN, 't_max_ms', NaN, ...
        'embedded_feasible_p99', false, ...
        'J_integrated',   NaN, 'peak_pos_mm', NaN, 'peak_ang_deg', NaN, ...
        'settling_2pct_ms', NaN, 'final_err_norm', NaN, ...
        'dist_recovery_time_ms', NaN, ...
        'build_failed',   true);
end

function W = build_summary_tab_(T)
% Reshape T into a per-panel wide table:
%   panel | full12_base_class | full12_base_tp99 | full12_base_J | ...
%   ... x ic_baseline / ic_harder / ic_disturbance
%   ... x full12 / reduced10
    panels = unique(T.panel, 'stable');
    rows = cell(numel(panels), 1);
    for k = 1:numel(panels)
        sub = T(strcmp(T.panel, panels{k}), :);
        row = struct('panel', panels{k});
        for o = {'full12','reduced10'}
            for i = {'ic_baseline','ic_harder','ic_disturbance'}
                m = strcmp(sub.order, o{1}) & strcmp(sub.ic_label, i{1});
                if any(m)
                    tag = sprintf('%s_%s', o{1}, ic_short_(i{1}));
                    row.(sprintf('class_%s', tag))   = char_(sub.classification{find(m,1)});
                    row.(sprintf('tp99_%s',  tag))   = sub.t_p99_ms(find(m,1));
                    row.(sprintf('J_%s',     tag))   = sub.J_integrated(find(m,1));
                end
            end
        end
        rows{k} = row;
    end
    W = struct2table(vertcat(rows{:}));
end

function s = ic_short_(label)
    switch label
        case 'ic_baseline';    s = 'base';
        case 'ic_harder';      s = 'hard';
        case 'ic_disturbance'; s = 'dist';
        otherwise;             s = label;
    end
end

function print_summary_(report)
    fprintf('\n--- Panel outcome by (order, ic) ---\n');
    disp(groupcounts(report.table, {'panel','order','ic_label','classification'}))
    fprintf('\n--- Summary table (one row per panel entry) ---\n');
    disp(report.summary_tab)
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
