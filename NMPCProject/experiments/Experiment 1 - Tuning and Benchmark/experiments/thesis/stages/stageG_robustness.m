function report = stageG_robustness(varargin)
% STAGEG_ROBUSTNESS  Stage G -- robustness / disturbance / timing-rep.
%
%   report = stageG_robustness()
%
% Runs three sub-studies on the top-3 candidates from Stage F:
%
%   G1  IC robustness:        N_LHS Latin-Hypercube perturbations of the
%                              initial state at scales {0.05, 0.10, 0.20,
%                              0.30}. Default N_LHS = 20 (80 runs total per
%                              candidate). Yields a stability-region cloud.
%
%   G2  Disturbance rejection: z-push at magnitudes {0.05, 0.10, 0.20,
%                              0.30, 0.50} x the baseline vector. Recovery
%                              time, max post-disturbance deviation, post-
%                              disturbance solve-time spike.
%
%   G3  Timing repeatability:  N_REPS repetitions on the SAME baseline IC,
%                              same cfg, to characterize jitter. Default
%                              N_REPS = 10.
%
% Top-3 candidates (per Stage F analysis):
%   - F4_Ewinner             : fully tuned per-order recommendation
%   - F5_aggressive_embedded : tightest-Ts deployment frontier
%   - F1_baseline            : pre-tuning reference (what the world looked
%                              like before this thesis)
%
% Each is run on both model orders.
%
% Inputs ('Name', value):
%   'N_LHS'    integer, samples per scale per (candidate, order). Default 20.
%   'N_REPS'   integer, repetitions for G3. Default 10.
%   'Force'    logical, re-run if a record exists. Default false.
%   'Seed'     integer, base seed for the LHS sweep. Default 0.
%
% Output struct report:
%   .G1_table  long-form table for G1 (one row per LHS sample x candidate x order)
%   .G2_table  long-form for G2
%   .G3_table  long-form for G3
%   .results_dir path

    p = inputParser();
    p.addParameter('N_LHS', 20, @(x) isnumeric(x) && x > 0);
    p.addParameter('N_REPS', 10, @(x) isnumeric(x) && x > 0);
    p.addParameter('Force', false, @islogical);
    p.addParameter('Seed', 0, @(x) isnumeric(x));
    p.parse(varargin{:});
    opts = p.Results;

    project_setup_thesis();
    repo_root   = repo_root_of_this_file();
    results_dir = fullfile(repo_root, 'experiments','thesis','results');
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    fprintf('\n============================================================\n');
    fprintf('  Stage G -- robustness / disturbance / timing repeatability\n');
    fprintf('  N_LHS=%d, N_REPS=%d, Seed=%d\n', opts.N_LHS, opts.N_REPS, opts.Seed);
    fprintf('  out:    %s/G/<order>/\n', results_dir);
    fprintf('============================================================\n\n');

    verify_stage_F_passed_(repo_root);
    se = load(fullfile(repo_root,'experiments','thesis','results','stageE_report.mat'),'report');
    tuned = se.report.tuned;

    candidates = build_candidates_(tuned);
    orders = {'full12','reduced10'};

    % --- G1: IC robustness LHS ----------------------------------------
    fprintf('\nG1: IC robustness (LHS, %d samples x 4 scales)\n', opts.N_LHS);
    G1_rows = {};
    scales = [0.05 0.10 0.20 0.30];
    for ci = 1:numel(candidates)
        cand = candidates(ci);
        for oi = 1:numel(orders)
            ord  = orders{oi};
            base = apply_candidate_(cand, ord);
            nx   = numel(base.ic_descriptor.direction);
            base_dir = base.ic_descriptor.direction(:);
            for si = 1:numel(scales)
                scale = scales(si);
                directions = lhs_directions_(opts.N_LHS, nx, ...
                    opts.Seed + 100*si + 10000*oi + 1000000*ci, base_dir);
                for k = 1:opts.N_LHS
                    cfg = base;
                    cfg.stage   = 'G';
                    cfg.exp_id  = sprintf('G1_%s_%s_s%g_k%d', cand.tag, ord(1:3), scale, k);
                    cfg.ic_descriptor.scale     = scale;
                    cfg.ic_descriptor.direction = directions(:, k);
                    cfg.disturbance_schedule    = [];
                    cfg.allow_overwrite = true;
                    fprintf('  G1 [%s|%s|s=%g|k=%2d/%d] ', cand.tag, ord, scale, k, opts.N_LHS);
                    try
                        rec = run_one_(cfg, ord, results_dir, opts);
                        G1_rows{end+1} = row_G1_(rec, cand, ord, scale, k); %#ok<AGROW>
                        fprintf('-> %s J=%.3g\n', rec.classification, safe_(rec.summary,'J_integrated'));
                    catch ME
                        fprintf('FAIL %s\n', ME.message);
                        G1_rows{end+1} = fail_G1_(cfg, cand, ord, scale, k, ME); %#ok<AGROW>
                    end
                end
            end
        end
    end
    G1 = struct2table(vertcat(G1_rows{:}));

    % --- G2: Disturbance magnitude sweep ------------------------------
    fprintf('\nG2: Disturbance magnitude sweep\n');
    G2_rows = {};
    base_dist = [0;0;0;0;0;0;-0.005;0.0025;-0.2;0;0;0];
    dist_mags = [0.05 0.10 0.20 0.30 0.50];
    for ci = 1:numel(candidates)
        cand = candidates(ci);
        for oi = 1:numel(orders)
            ord  = orders{oi};
            base = apply_candidate_(cand, ord);
            base.ic_descriptor.scale = 0.05;
            for mi = 1:numel(dist_mags)
                cfg = base;
                cfg.stage   = 'G';
                cfg.exp_id  = sprintf('G2_%s_%s_m%g', cand.tag, ord(1:3), dist_mags(mi));
                cfg.disturbance_schedule = struct('t_apply', 5.0, ...
                    'kind','state_additive', 'vector', dist_mags(mi) * base_dist);
                cfg.sim_steps = max(cfg.sim_steps, round(8.0 / (cfg.Tf/cfg.N)));
                cfg.allow_overwrite = true;
                fprintf('  G2 [%s|%s|m=%g] ', cand.tag, ord, dist_mags(mi));
                try
                    rec = run_one_(cfg, ord, results_dir, opts);
                    G2_rows{end+1} = row_G2_(rec, cand, ord, dist_mags(mi)); %#ok<AGROW>
                    fprintf('-> %s J=%.3g recovery=%.0f ms\n', rec.classification, ...
                            safe_(rec.summary,'J_integrated'), safe_(rec.summary,'dist_recovery_time_ms'));
                catch ME
                    fprintf('FAIL %s\n', ME.message);
                    G2_rows{end+1} = fail_G2_(cfg, cand, ord, dist_mags(mi), ME); %#ok<AGROW>
                end
            end
        end
    end
    G2 = struct2table(vertcat(G2_rows{:}));

    % --- G3: Timing repeatability -------------------------------------
    fprintf('\nG3: Timing repeatability (%d repetitions)\n', opts.N_REPS);
    G3_rows = {};
    for ci = 1:numel(candidates)
        cand = candidates(ci);
        for oi = 1:numel(orders)
            ord  = orders{oi};
            base = apply_candidate_(cand, ord);
            base.ic_descriptor.scale = 0.05;
            for r = 1:opts.N_REPS
                cfg = base;
                cfg.stage  = 'G';
                cfg.exp_id = sprintf('G3_%s_%s_r%02d', cand.tag, ord(1:3), r);
                cfg.disturbance_schedule = [];
                cfg.allow_overwrite = true;
                fprintf('  G3 [%s|%s|r=%2d/%d] ', cand.tag, ord, r, opts.N_REPS);
                try
                    rec = run_one_(cfg, ord, results_dir, struct('Force', true)); % always re-run for fresh timing
                    G3_rows{end+1} = row_G3_(rec, cand, ord, r); %#ok<AGROW>
                    fprintf('-> %s t_p99=%.2f ms\n', rec.classification, rec.summary.t_p99_ms);
                catch ME
                    fprintf('FAIL %s\n', ME.message);
                    G3_rows{end+1} = fail_G3_(cfg, cand, ord, r, ME); %#ok<AGROW>
                end
            end
        end
    end
    G3 = struct2table(vertcat(G3_rows{:}));

    report = struct();
    report.G1_table = G1;
    report.G2_table = G2;
    report.G3_table = G3;
    report.candidates = candidates;
    report.results_dir = results_dir;

    save(fullfile(results_dir,'stageG_report.mat'), 'report', '-v7.3'); %#ok<NASGU>
    fprintf('\nSaved roll-up to %s/stageG_report.mat\n', results_dir);

    print_summary_(report);
end

% ====================================================================== %
% Stage F gate
% ====================================================================== %

function verify_stage_F_passed_(repo_root)
    p = fullfile(repo_root,'experiments','thesis','results','stageF_report.mat');
    if ~exist(p,'file')
        error('stageG:no_stageF', ...
            'Stage F has not been run. Run stageF_model_order first.');
    end
    fprintf('Stage F gate: report present at %s\n\n', p);
end

% ====================================================================== %
% Candidate definitions
% ====================================================================== %

function cands = build_candidates_(tuned)
% Each candidate carries the per-order cfg transform.
    cands(1) = struct('tag','F4_Ewinner',  ...
                      'apply', @(c,o) apply_Ewinner_(c, o, tuned));
    cands(2) = struct('tag','F5_aggressive', ...
                      'apply', @(c,o) apply_aggressive_(c, o, tuned));
    cands(3) = struct('tag','F1_baseline', 'apply', @(c,o) c);
end

function c = apply_Cwinner_(c)
    c.integrator_type       = 'IRK';
    c.sim_method_num_stages = 1;
    c.sim_method_num_steps  = 1;
end

function c = apply_Dwinner_(c)
    c = apply_Cwinner_(c);
    c.N  = 10; c.Tf = 10 * 0.005;
end

function c = apply_Ewinner_(c, order, tuned)
    c = apply_Dwinner_(c);
    if ~isfield(tuned, order); return; end
    tw = tuned.(order).cfg;
    c.R = tw.R(:); c.Q = tw.Q(:);
    c.alpha_we = tw.alpha_we;
    c.Zl = tw.Zl(:); c.Zu = tw.Zu(:);
    c.zl = tw.zl(:); c.zu = tw.zu(:);
end

function c = apply_aggressive_(c, order, tuned)
    c = apply_Ewinner_(c, order, tuned);
    c.N = 20; c.Tf = 20 * 0.003;
end

function cfg = apply_candidate_(cand, order)
    switch order
        case 'full12'
            cfg = validate_cfg(baseline_full12(), 'PrintDefaults', false);
        case 'reduced10'
            raw = baseline_reduced10();
            raw.plant_override       = struct();
            raw.disturbance_schedule = [];
            cfg = validate_cfg(raw, 'PrintDefaults', false);
        otherwise
            error('stageG:bad_order','order must be full12 or reduced10');
    end
    cfg.allow_overwrite = true;
    cfg = cand.apply(cfg, order);
end

% ====================================================================== %
% LHS directions
% ====================================================================== %

function D = lhs_directions_(N, nx, seed, base_direction)
% Returns nx x N matrix; each column is an LHS-sampled direction *scaled
% to the same Euclidean norm as the canonical baseline direction*. This
% keeps the meaning of cfg.ic_descriptor.scale identical between Stages
% B-F (which use the fixed baseline direction) and Stage G (which uses
% randomized directions): scale=0.05 always injects a perturbation of
% magnitude 0.05 * ||baseline_direction||.
%
% If base_direction is empty or omitted, falls back to unit-norm.
    rng(seed);
    U = (lhsdesign_simple_(N, nx) - 0.5) * 2;   % [-1, 1]
    D = U.';
    if nargin < 4 || isempty(base_direction)
        target_norm = 1;
    else
        target_norm = norm(base_direction(:));
        if target_norm == 0; target_norm = 1; end
    end
    for k = 1:N
        nrm = norm(D(:,k));
        if nrm > 0
            D(:,k) = D(:,k) / nrm * target_norm;
        end
    end
end

function H = lhsdesign_simple_(N, k)
% Minimal LHS implementation (avoids dependency on Statistics Toolbox).
% Each column is a permutation of (1..N)/N then jittered.
    H = zeros(N, k);
    for j = 1:k
        perm = randperm(N);
        H(:, j) = (perm - rand(1, N)).' / N;
    end
end

% ====================================================================== %
% Per-config build + run + save
% ====================================================================== %

function rec = run_one_(cfg, order, results_dir, opts)
    cfg.order = order;
    cfg = validate_cfg(cfg, 'PrintDefaults', false);

    target = predict_save_path_(cfg, results_dir);
    if exist(target,'file') && ~opts.Force
        try
            rec = load_result(target);
            return
        catch ME
            warning('stageG:stale_record', 'stale at %s (%s); rerunning', target, ME.message);
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
% Row builders
% ====================================================================== %

function r = row_G1_(rec, cand, ord, scale, k)
    s = rec.summary;
    r = struct('candidate', char_(cand.tag), 'order', char_(ord), ...
        'scale', scale, 'sample', k, ...
        'classification', char_(rec.classification), 'reason', char_(rec.reason), ...
        't_p99_ms', safe_(s,'t_p99_ms'), 'J_integrated', safe_(s,'J_integrated'), ...
        'peak_pos_mm', safe_(s,'peak_pos_mm'), 'final_err_norm', safe_(s,'final_err_norm'));
end
function r = fail_G1_(cfg, cand, ord, scale, k, ME) %#ok<INUSL>
    r = struct('candidate', char_(cand.tag), 'order', char_(ord), ...
        'scale', scale, 'sample', k, ...
        'classification', 'BUILD_FAIL', 'reason', char_(ME.message), ...
        't_p99_ms', NaN, 'J_integrated', NaN, ...
        'peak_pos_mm', NaN, 'final_err_norm', NaN);
end

function r = row_G2_(rec, cand, ord, mag)
    s = rec.summary;
    r = struct('candidate', char_(cand.tag), 'order', char_(ord), 'dist_magnitude', mag, ...
        'classification', char_(rec.classification), 'reason', char_(rec.reason), ...
        't_p99_ms', safe_(s,'t_p99_ms'), 'J_integrated', safe_(s,'J_integrated'), ...
        'dist_recovery_time_ms', safe_(s,'dist_recovery_time_ms'), ...
        'dist_max_z_deviation_post_mm', safe_(s,'dist_max_z_deviation_post_mm'), ...
        'dist_post_solve_time_spike_ms', safe_(s,'dist_post_solve_time_spike_ms'));
end
function r = fail_G2_(cfg, cand, ord, mag, ME) %#ok<INUSL>
    r = struct('candidate', char_(cand.tag), 'order', char_(ord), 'dist_magnitude', mag, ...
        'classification', 'BUILD_FAIL', 'reason', char_(ME.message), ...
        't_p99_ms', NaN, 'J_integrated', NaN, ...
        'dist_recovery_time_ms', NaN, 'dist_max_z_deviation_post_mm', NaN, ...
        'dist_post_solve_time_spike_ms', NaN);
end

function r = row_G3_(rec, cand, ord, rep)
    s = rec.summary;
    r = struct('candidate', char_(cand.tag), 'order', char_(ord), 'rep', rep, ...
        'classification', char_(rec.classification), ...
        't_mean_ms', safe_(s,'t_mean_ms'), 't_median_ms', safe_(s,'t_median_ms'), ...
        't_p95_ms', safe_(s,'t_p95_ms'), 't_p99_ms', safe_(s,'t_p99_ms'), ...
        't_max_ms', safe_(s,'t_max_ms'), 'jitter_iqr_ms', safe_(s,'jitter_iqr_ms'));
end
function r = fail_G3_(cfg, cand, ord, rep, ME) %#ok<INUSL>
    r = struct('candidate', char_(cand.tag), 'order', char_(ord), 'rep', rep, ...
        'classification', 'BUILD_FAIL', ...
        't_mean_ms', NaN, 't_median_ms', NaN, 't_p95_ms', NaN, 't_p99_ms', NaN, ...
        't_max_ms', NaN, 'jitter_iqr_ms', NaN);
end

% ====================================================================== %
% Summary
% ====================================================================== %

function print_summary_(report)
    fprintf('\n--- G1 stability region (CONVERGED rate by candidate/order/scale) ---\n');
    G1 = report.G1_table;
    G1.stable = startsWith(G1.classification, 'CONVERGED_STABLE');
    G = groupsummary(G1, {'candidate','order','scale'}, 'mean', 'stable');
    disp(G)

    fprintf('\n--- G2 disturbance recovery (per candidate/order/magnitude) ---\n');
    disp(report.G2_table(:, {'candidate','order','dist_magnitude', ...
         'classification','dist_recovery_time_ms','dist_max_z_deviation_post_mm'}))

    fprintf('\n--- G3 timing repeatability (per candidate/order) ---\n');
    G3 = report.G3_table;
    G = groupsummary(G3, {'candidate','order'}, {'mean','std'}, {'t_mean_ms','t_p99_ms','t_max_ms'});
    disp(G)
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
