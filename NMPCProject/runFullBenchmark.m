function runFullBenchmark(varargin)
% RUNFULLBENCHMARK  Orchestrate the entire NMPC benchmark sweep.
%
%   runFullBenchmark()                   % both models, all stages
%   runFullBenchmark('models', {'reduced10'})
%   runFullBenchmark('stages', {'P0','1a','0','1b','2','3a','3b'})
%
% Behavior
%   - Runs Stages P0 -> 1a -> 0 -> 1b -> 2 -> 3a -> 3b for each model.
%   - Picks best configs from earlier stages to fill the cfg context for
%     later stages (top_1b for Stage 2, top_2 for Stage 3a, top_1b_three
%     for Stage 3b).
%   - Checkpoints to results/_checkpoints/progress.mat after each run;
%     restart skips runs whose .mat already exists.
%   - Writes a JSON copy of progress for human inspection.
%
% Stage 3b uses the cold-build / clear-classes timing pattern (separate
% MATLAB session) — this orchestrator schedules it at the very end so its
% effect on caches is contained.

    paths = project_setup();

    p = inputParser();
    p.addParameter('models', {'reduced10', 'full12'}, @(c) iscell(c));
    p.addParameter('stages', {'P0','1a','0','1b','2','3a','3b'}, @(c) iscell(c));
    p.parse(varargin{:});
    models = p.Results.models;
    stages = expand_stage0_passes(p.Results.stages);

    save_dir = fullfile(paths.project_root, 'results');
    if ~exist(save_dir, 'dir'); mkdir(save_dir); end
    ckpt_dir = fullfile(save_dir, '_checkpoints');
    if ~exist(ckpt_dir, 'dir'); mkdir(ckpt_dir); end
    ckpt_file = fullfile(ckpt_dir, 'progress.mat');

    progress = load_progress(ckpt_file);

    fprintf('\n========================================================\n');
    fprintf('  NMPC benchmark — runFullBenchmark\n');
    fprintf('  Models: %s\n', strjoin(models, ', '));
    fprintf('  Stages: %s\n', strjoin(stages, ', '));
    fprintf('  Output: %s\n', save_dir);
    fprintf('========================================================\n');

    for im = 1:length(models)
        model_kind = models{im};
        ctx = init_ctx(progress, model_kind);

        for is = 1:length(stages)
            stage = stages{is};
            fprintf('\n----- Model: %s | Stage: %s -----\n', model_kind, stage);

            cfgs = configs_for_stage(model_kind, stage, ctx);
            if isempty(cfgs)
                fprintf('  (no configs for this stage with current ctx; skipping)\n');
                continue;
            end

            for ic = 1:length(cfgs)
                cfg = cfgs{ic};
                cfg.save_dir = save_dir;
                rid = run_id(cfg);

                if isfield(progress.completed, rid)
                    fprintf('  [%d/%d] %s — already done.\n', ic, length(cfgs), rid);
                    continue;
                end

                try
                    res = runOneConfig(cfg);
                    progress.completed.(rid) = struct( ...
                        'path', res.saved_path, ...
                        'when', datestr(now,'yyyy-mm-dd_HH-MM-SS'), ...
                        'summary', res.summary);
                    save_progress(progress, ckpt_file);
                catch ME
                    fprintf(2, '  ERROR in %s: %s\n', rid, ME.message);
                    progress.failed.(rid) = ME.message;
                    save_progress(progress, ckpt_file);
                end
            end

            % Refresh ctx after each stage so downstream stages see picks.
            ctx = update_ctx_after_stage(ctx, model_kind, stage, save_dir);
        end

        % Persist final ctx per model so re-runs use the same picks.
        progress.ctx.(model_kind) = ctx;
        save_progress(progress, ckpt_file);
    end

    fprintf('\n========================================================\n');
    fprintf('  Done. Run dump_index() to refresh the CSV index.\n');
    fprintf('========================================================\n');
end

% =========================================================================
% Stage routing
% =========================================================================

function cfgs = configs_for_stage(model_kind, stage, ctx)
    all_cfgs = build_all_configs(model_kind, ctx);
    cfgs = {};
    for k = 1:length(all_cfgs)
        if strcmp(all_cfgs{k}.stage, stage)
            cfgs{end+1} = all_cfgs{k}; %#ok<AGROW>
        end
    end
    if strcmp(stage, '0')
        cfgs = filter_stage0(cfgs, ctx);
    end
end

function out = filter_stage0(cfgs, ctx)
% Stage 0 emits both passes; before alpha_R is locked we run only pass 1
% (alpha_R varies, alpha_we=25). Pass 2 runs after pass 1 picks alpha_R.
    out = {};
    for k = 1:length(cfgs)
        nm = cfgs{k}.exp_id;
        if startsWith(nm, 'aR') && contains(nm, '_we25')
            % Pass 1 (varying alpha_R, fixed alpha_we=25)
            out{end+1} = cfgs{k}; %#ok<AGROW>
        elseif startsWith(nm, 'aR_locked')
            % Pass 2 — only run after alpha_R is locked from pass 1
            if isfield(ctx, 'alpha_R_locked') && ctx.alpha_R_locked
                out{end+1} = cfgs{k}; %#ok<AGROW>
            end
        end
    end
end

% =========================================================================
% Context picks (Stage 0/1b/2 -> downstream)
% =========================================================================

function ctx = init_ctx(progress, model_kind)
    if isfield(progress, 'ctx') && isfield(progress.ctx, model_kind)
        ctx = progress.ctx.(model_kind);
    else
        ctx = struct();
    end
end

function ctx = update_ctx_after_stage(ctx, model_kind, stage, save_dir)
    switch stage
        case 'P0'
            picks = pick_P0(save_dir, model_kind);
            ctx.ns_irk    = picks.ns_irk;
            ctx.nstep_irk = picks.nstep_irk;
            ctx.nstep_erk = picks.nstep_erk;
            ctx.gnsf_ok   = picks.gnsf_ok;
            fprintf('  P0 picks: IRK %d/%d, ERK steps=%d, GNSF=%s\n', ...
                ctx.ns_irk, ctx.nstep_irk, ctx.nstep_erk, ...
                ternary_str(ctx.gnsf_ok, 'ok', 'skip'));

        case '0'
            picks = pick_stage0(save_dir, model_kind);
            ctx.alpha_R  = picks.alpha_R;
            ctx.alpha_we = picks.alpha_we;
            ctx.alpha_R_locked = true;
            [Qb, Rb] = bryson_weights(model_kind);
            ctx.Q = Qb; ctx.R = Rb * picks.alpha_R;
            fprintf('  Stage-0 picks: alpha_R=%g, alpha_we=%g\n', ...
                picks.alpha_R, picks.alpha_we);

        case '1b'
            picks = pick_top_1b(save_dir, model_kind);
            ctx.top_1b       = picks.top2;
            ctx.top_1b_three = picks.top3;
            fprintf('  1b top-2: %s, %s\n', picks.top2_ids{1}, picks.top2_ids{2});

        case '2'
            picks = pick_top_2(save_dir, model_kind);
            ctx.top_2 = picks.top1;
            fprintf('  Stage-2 top-1: %s\n', picks.top1_id);

        otherwise
            % no picks needed
    end
end

% =========================================================================
% Per-stage best-of selection (reads .mat summaries)
% =========================================================================

function picks = pick_P0(save_dir, model_kind)
% Pick the smallest IRK (stages, num_steps) that integrates accurately,
% i.e. J_integrated within 1% of the IRK 4/10 reference. Same for ERK.
    [files, summaries] = load_stage(save_dir, model_kind, 'P0');
    if isempty(files)
        picks = struct('ns_irk',4,'nstep_irk',5,'nstep_erk',5,'gnsf_ok',false);
        return;
    end

    % Find IRK reference (4 stages, 10 steps)
    refJ = NaN;
    for k = 1:numel(files)
        if startsWith(files(k).exp_id, 'IRK_s4_n10')
            refJ = summaries{k}.J_integrated; break;
        end
    end
    if isnan(refJ)
        warning('IRK_s4_n10 reference missing; using min J_integrated as ref.');
        Js = cellfun(@(s) s.J_integrated, summaries);
        refJ = min(Js);
    end

    % IRK pick
    best_irk = struct('cost', inf, 'ns', 4, 'nstep', 5);
    for k = 1:numel(files)
        if ~startsWith(files(k).exp_id, 'IRK_s'); continue; end
        if summaries{k}.diverged; continue; end
        J = summaries{k}.J_integrated;
        if abs(J - refJ) / max(refJ, eps) > 0.01; continue; end
        % parse stages/steps from exp_id "IRK_sX_nY"
        toks = regexp(files(k).exp_id, 'IRK_s(\d+)_n(\d+)', 'tokens', 'once');
        if isempty(toks); continue; end
        ns = str2double(toks{1}); ns_step = str2double(toks{2});
        cost = ns * ns_step;     % cheap proxy for compute
        if cost < best_irk.cost
            best_irk = struct('cost', cost, 'ns', ns, 'nstep', ns_step);
        end
    end

    % ERK pick
    best_erk_step = 10;
    for k = 1:numel(files)
        if ~startsWith(files(k).exp_id, 'ERK_n'); continue; end
        if summaries{k}.diverged; continue; end
        J = summaries{k}.J_integrated;
        if abs(J - refJ) / max(refJ, eps) > 0.01; continue; end
        toks = regexp(files(k).exp_id, 'ERK_n(\d+)', 'tokens', 'once');
        if isempty(toks); continue; end
        st = str2double(toks{1});
        if st < best_erk_step; best_erk_step = st; end
    end

    % GNSF
    gnsf_ok = false;
    for k = 1:numel(files)
        if strcmp(files(k).exp_id, 'GNSF_dropin')
            gnsf_ok = ~summaries{k}.diverged && summaries{k}.all_converged;
            break;
        end
    end

    picks.ns_irk    = best_irk.ns;
    picks.nstep_irk = best_irk.nstep;
    picks.nstep_erk = best_erk_step;
    picks.gnsf_ok   = gnsf_ok;
end

function picks = pick_stage0(save_dir, model_kind)
% Pass 1: pick alpha_R minimizing J among the aR-varying runs.
% Pass 2: pick alpha_we minimizing J among the locked-aR runs.
    [files, summaries] = load_stage(save_dir, model_kind, '0');
    if isempty(files)
        picks = struct('alpha_R',1.0,'alpha_we',50);
        return;
    end

    bestR.J = inf; bestR.aR = 1.0;
    bestWe.J = inf; bestWe.aWe = 50;
    for k = 1:numel(files)
        nm = files(k).exp_id;
        if summaries{k}.diverged; continue; end
        J = summaries{k}.J_integrated;
        if startsWith(nm, 'aR_locked_we')
            tok = regexp(nm, 'we(\d+)', 'tokens', 'once');
            if isempty(tok); continue; end
            aWe = str2double(tok{1});
            if J < bestWe.J; bestWe = struct('J',J,'aWe',aWe); end
        elseif startsWith(nm, 'aR') && contains(nm, '_we25')
            tok = regexp(nm, 'aR(\w+?)_we25', 'tokens', 'once');
            if isempty(tok); continue; end
            aR = str2double(strrep(tok{1}, 'p', '.'));
            if J < bestR.J; bestR = struct('J',J,'aR',aR); end
        end
    end

    picks.alpha_R  = bestR.aR;
    picks.alpha_we = bestWe.aWe;
end

function picks = pick_top_1b(save_dir, model_kind)
% Top-K by Pareto front of (t_mean_ms, J_integrated). Falls back to
% lex-min on (J, t_mean) when fewer than K Pareto-optimal points exist.
    [files, summaries] = load_stage(save_dir, model_kind, '1b');
    if isempty(files)
        picks.top2     = {};
        picks.top2_ids = {'',''};
        picks.top3     = {};
        return;
    end

    n = numel(files);
    t = nan(1, n); J = nan(1, n); valid = false(1, n);
    for k = 1:n
        if summaries{k}.diverged; continue; end
        if ~summaries{k}.all_converged; continue; end
        t(k) = summaries{k}.t_mean_ms;
        J(k) = summaries{k}.J_integrated;
        valid(k) = true;
    end

    pareto_idx = pareto_front(t, J, valid);
    score = (J - min(J(valid))) / max(eps, max(J(valid)) - min(J(valid))) ...
          + (t - min(t(valid))) / max(eps, max(t(valid)) - min(t(valid)));
    score(~valid) = inf;
    [~, sorted] = sort(score);
    selection = unique([pareto_idx, sorted], 'stable');

    pickIdx = selection(1:min(end, 3));
    picks.top3 = cell(1, numel(pickIdx));
    for k = 1:numel(pickIdx)
        picks.top3{k} = files(pickIdx(k)).cfg;
    end
    picks.top2     = picks.top3(1:min(end, 2));
    picks.top2_ids = arrayfun(@(i) files(i).exp_id, pickIdx(1:min(end,2)), 'UniformOutput', false);
end

function picks = pick_top_2(save_dir, model_kind)
    [files, summaries] = load_stage(save_dir, model_kind, '2');
    if isempty(files)
        picks.top1    = {};
        picks.top1_id = '';
        return;
    end
    bestJ = inf; bestK = 0;
    for k = 1:numel(files)
        if summaries{k}.diverged || ~summaries{k}.all_converged; continue; end
        if summaries{k}.J_integrated < bestJ
            bestJ = summaries{k}.J_integrated; bestK = k;
        end
    end
    if bestK == 0
        picks.top1    = {};
        picks.top1_id = '';
    else
        picks.top1    = {files(bestK).cfg};
        picks.top1_id = files(bestK).exp_id;
    end
end

function front = pareto_front(t, J, valid)
% Return indices of Pareto-optimal points in (t, J). Lower is better in both.
    n = length(t);
    front = [];
    for i = 1:n
        if ~valid(i); continue; end
        dominated = false;
        for j = 1:n
            if i == j || ~valid(j); continue; end
            if (t(j) <= t(i)) && (J(j) <= J(i)) && ((t(j) < t(i)) || (J(j) < J(i)))
                dominated = true; break;
            end
        end
        if ~dominated; front(end+1) = i; end %#ok<AGROW>
    end
    if isempty(front); return; end
    [~, ord] = sort(t(front));
    front = front(ord);
end

% =========================================================================
% Stage-summary load helpers
% =========================================================================

function [files, summaries] = load_stage(save_dir, model_kind, stage)
    dir_path = fullfile(save_dir, model_kind, stage);
    if ~exist(dir_path, 'dir')
        files = struct([]); summaries = {};
        return;
    end
    listing = dir(fullfile(dir_path, '*.mat'));
    files = struct('exp_id', {}, 'path', {}, 'cfg', {});
    summaries = {};
    for k = 1:length(listing)
        path = fullfile(listing(k).folder, listing(k).name);
        s = load(path, 'summary', 'cfg', 'exp_id');
        files(end+1).exp_id = s.exp_id; %#ok<AGROW>
        files(end).path = path;
        files(end).cfg  = s.cfg;
        summaries{end+1} = s.summary; %#ok<AGROW>
    end
end

% =========================================================================
% Progress checkpoint
% =========================================================================

function progress = load_progress(ckpt_file)
    if exist(ckpt_file, 'file')
        s = load(ckpt_file);
        progress = s.progress;
        if ~isfield(progress, 'completed'); progress.completed = struct(); end
        if ~isfield(progress, 'failed');    progress.failed    = struct(); end
        if ~isfield(progress, 'ctx');       progress.ctx       = struct(); end
        fprintf('Resumed progress: %d completed, %d failed.\n', ...
            numel(fieldnames(progress.completed)), numel(fieldnames(progress.failed)));
    else
        progress = struct('completed', struct(), 'failed', struct(), ...
            'ctx', struct(), 'created', datestr(now));
    end
end

function save_progress(progress, ckpt_file)
    save(ckpt_file, 'progress');
    json_path = strrep(ckpt_file, '.mat', '.json');
    try
        json = jsonencode(rmfield_safe(progress, 'completed'));
        fid = fopen(json_path, 'w'); fprintf(fid, '%s', json); fclose(fid);
    catch
        % jsonencode chokes on certain structs; ignore
    end
end

function s = rmfield_safe(s, fld)
    if isfield(s, fld); s = rmfield(s, fld); end
end

% =========================================================================
% Misc
% =========================================================================

function id = run_id(cfg)
    id = sprintf('%s__%s__%s', cfg.model_kind, cfg.stage, cfg.exp_id);
    id = matlab.lang.makeValidName(id);
end

function s = ternary_str(cond, a, b)
    if cond; s = a; else; s = b; end
end

function out = expand_stage0_passes(stages)
% Stage 0 runs in two passes: pass 1 picks alpha_R, pass 2 picks alpha_we.
% This is encoded by listing Stage '0' twice — the first iteration runs
% pass-1 cfgs (pass 2 is gated by ctx.alpha_R_locked, false initially);
% update_ctx_after_stage('0') sets ctx.alpha_R and ctx.alpha_R_locked=true,
% so the second iteration runs pass-2 cfgs (pass-1 cfgs are skipped because
% their .mat files already exist).
    out = {};
    for k = 1:length(stages)
        out{end+1} = stages{k}; %#ok<AGROW>
        if strcmp(stages{k}, '0')
            out{end+1} = '0';   %#ok<AGROW>  pass 2
        end
    end
end
