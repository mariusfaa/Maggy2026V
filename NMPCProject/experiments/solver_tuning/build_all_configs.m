function cfgs = build_all_configs(model_kind, ctx)
% BUILD_ALL_CONFIGS  Produce the full list of cfg structs for one model.
%
%   cfgs = build_all_configs(model_kind, ctx)
%
% Inputs
%   model_kind   'reduced10' | 'full12'
%   ctx          context struct used to thread Stage 0 -> 1b -> 2 picks:
%                  .alpha_R      locked from Stage 0 (default 1.0)
%                  .alpha_we     locked from Stage 0 (default 50)
%                  .Q            locked Q (default Bryson)
%                  .R            locked R (default identity)
%                  .ns_irk       picked num_stages from P0 (default 4)
%                  .nstep_irk    picked num_steps  from P0 (default 5)
%                  .nstep_erk    picked num_steps  from P0 (default 5)
%                  .gnsf_ok      logical from P0 (default false)
%                  .top_1b       cell array of cfg structs (top-2 of Stage 1b)
%                  .top_2        cell array of cfg structs (top-1 of Stage 2)
%                  .top_1b_three cell array of cfg structs (top-3 of Stage 1b)
%                                used by Stage 3b
%
% NOTE: ctx is allowed to be empty/incomplete on first call. The orchestrator
% builds the cfg list incrementally, refreshing ctx as it learns the picks.
% This function returns a *flat* list of cfgs across all stages but with
% sensible defaults so a partial list can run end-to-end on a fresh setup.

    if nargin < 2 || isempty(ctx); ctx = struct(); end
    ctx = with_defaults(ctx, model_kind);

    cfgs = {};

    cfgs = [cfgs, stage_P0(model_kind)];
    cfgs = [cfgs, stage_1a(model_kind, ctx)];
    cfgs = [cfgs, stage_0( model_kind, ctx)];
    cfgs = [cfgs, stage_1b(model_kind, ctx)];
    cfgs = [cfgs, stage_2( model_kind, ctx)];
    cfgs = [cfgs, stage_3a(model_kind, ctx)];
    cfgs = [cfgs, stage_3b(model_kind, ctx)];
end

% =========================================================================
% Defaults
% =========================================================================

function ctx = with_defaults(ctx, model_kind)
    [Qb, Rb] = bryson_weights(model_kind);
    ctx = setif(ctx, 'alpha_R',     1.0);
    ctx = setif(ctx, 'alpha_we',    50);
    ctx = setif(ctx, 'Q',           Qb);
    ctx = setif(ctx, 'R',           Rb);
    ctx = setif(ctx, 'ns_irk',      4);
    ctx = setif(ctx, 'nstep_irk',   5);
    ctx = setif(ctx, 'nstep_erk',   5);
    ctx = setif(ctx, 'gnsf_ok',     false);
end

function s = setif(s, fld, v)
    if ~isfield(s, fld) || isempty(s.(fld)); s.(fld) = v; end
end

function c = base_cfg(model_kind)
    c.model_kind     = model_kind;
    c.controller_n   = 10;
    c.plant_n        = 30;
    c.N              = 20;
    c.Tf             = 0.2;
    c.nlp_solver     = 'SQP_RTI';
    c.qp_solver      = 'PARTIAL_CONDENSING_HPIPM';
    c.hessian_approx = 'GAUSS_NEWTON';
    c.alpha_R        = 1.0;
    c.alpha_we       = 50;
    [Qb, Rb]         = bryson_weights(model_kind);
    c.Q              = Qb;
    c.R              = Rb;
    c.ic_label       = 'baseline';
    c.ic_scale       = 0.05;
    c.ic_direction   = [0.005;-0.008;0.010; 0.15;-0.10; 0; 0.01;-0.01;0.0; 0.2;-0.3; 0];
    c.n_repeats      = 1;
end

% =========================================================================
% Stage P0 — integrator pre-screen
% =========================================================================

function cfgs = stage_P0(model_kind)
    cfgs = {};
    base = base_cfg(model_kind);
    base.stage = 'P0';

    % IRK: stages x steps grid
    irk_stages = [2 3 4];
    irk_steps  = [2 3 5 10];
    for s = irk_stages
        for st = irk_steps
            c = base;
            c.exp_id  = sprintf('IRK_s%d_n%d', s, st);
            c.integ_type = 'IRK';
            c.n_stages   = s;
            c.n_steps    = st;
            cfgs{end+1} = c; %#ok<AGROW>
        end
    end

    % ERK4: steps grid (stages always 4)
    for st = irk_steps
        c = base;
        c.exp_id     = sprintf('ERK_n%d', st);
        c.integ_type = 'ERK';
        c.n_stages   = 4;
        c.n_steps    = st;
        cfgs{end+1} = c; %#ok<AGROW>
    end

    % GNSF drop-in attempt
    c = base;
    c.exp_id     = 'GNSF_dropin';
    c.integ_type = 'GNSF';
    c.n_stages   = 4;
    c.n_steps    = 5;
    cfgs{end+1} = c;
end

% =========================================================================
% Stage 1a — seed solver pick
% =========================================================================

function cfgs = stage_1a(model_kind, ctx)
    base = base_cfg(model_kind);
    base.stage = '1a';
    base.exp_id = 'seed';
    base.integ_type = 'IRK';
    base.n_stages   = ctx.ns_irk;
    base.n_steps    = ctx.nstep_irk;
    cfgs = {base};
end

% =========================================================================
% Stage 0 — sequential alpha_R then alpha_we
% =========================================================================

function cfgs = stage_0(model_kind, ctx)
    cfgs = {};
    base = base_cfg(model_kind);
    base.stage = '0';
    base.integ_type = 'IRK';
    base.n_stages   = ctx.ns_irk;
    base.n_steps    = ctx.nstep_irk;

    [Qb, Rb] = bryson_weights(model_kind);
    base.Q = Qb;
    base.R = Rb;

    % Pass 1: vary alpha_R, fix alpha_we
    aR_grid = [0.1, 0.3, 1.0, 3.0, 10.0];
    aWe_fixed = 25;
    for k = 1:length(aR_grid)
        c = base;
        c.exp_id   = sprintf('aR%s_we%d', double_to_tag(aR_grid(k)), aWe_fixed);
        c.alpha_R  = aR_grid(k);
        c.alpha_we = aWe_fixed;
        c.R        = Rb * aR_grid(k);
        cfgs{end+1} = c; %#ok<AGROW>
    end

    % Pass 2: at locked alpha_R = ctx.alpha_R (set by orchestrator after pass 1),
    % vary alpha_we. The orchestrator will fill ctx.alpha_R between the two
    % passes; here we emit both grids and let the orchestrator filter / rerun.
    aWe_grid = [1, 5, 25, 100, 500];
    for k = 1:length(aWe_grid)
        c = base;
        c.exp_id   = sprintf('aR_locked_we%d', aWe_grid(k));
        c.alpha_R  = ctx.alpha_R;
        c.alpha_we = aWe_grid(k);
        c.R        = Rb * ctx.alpha_R;
        cfgs{end+1} = c; %#ok<AGROW>
    end
end

% =========================================================================
% Stage 1b — full solver/integrator/SQP/Hessian sweep
% =========================================================================

function cfgs = stage_1b(model_kind, ctx)
    cfgs = {};
    base = base_cfg(model_kind);
    base.stage = '1b';
    [Qb, Rb] = bryson_weights(model_kind);
    base.Q   = Qb;
    base.R   = Rb * ctx.alpha_R;
    base.alpha_R  = ctx.alpha_R;
    base.alpha_we = ctx.alpha_we;

    qp_list   = {'FULL_CONDENSING_HPIPM', 'PARTIAL_CONDENSING_HPIPM', 'FULL_CONDENSING_QPOASES'};
    integ_list = {'ERK', 'IRK'};
    if ctx.gnsf_ok; integ_list{end+1} = 'GNSF'; end
    nlp_list  = {'SQP', 'SQP_RTI'};
    hess_list = {'GAUSS_NEWTON', 'EXACT'};

    for iqp = 1:length(qp_list)
        for ii = 1:length(integ_list)
            for inlp = 1:length(nlp_list)
                for ih = 1:length(hess_list)
                    qp   = qp_list{iqp};
                    intg = integ_list{ii};
                    nlp  = nlp_list{inlp};
                    hess = hess_list{ih};

                    % NOTE: acados's qp_solver string already bundles solver+
                    % condensing, so there's no "qpOASES + partial" combo to
                    % drop. Soft-edge cases (EXACT+RTI, qpOASES+EXACT) are
                    % run as-is; divergence detection surfaces failures.

                    c = base;
                    c.qp_solver       = qp;
                    c.integ_type      = intg;
                    c.nlp_solver      = nlp;
                    c.hessian_approx  = hess;

                    if strcmp(intg, 'ERK')
                        c.n_stages = 4;
                        c.n_steps  = ctx.nstep_erk;
                    elseif strcmp(intg, 'IRK')
                        c.n_stages = ctx.ns_irk;
                        c.n_steps  = ctx.nstep_irk;
                    else  % GNSF
                        c.n_stages = ctx.ns_irk;
                        c.n_steps  = ctx.nstep_irk;
                    end

                    c.exp_id = compact_id_1b(qp, intg, nlp, hess);
                    cfgs{end+1} = c; %#ok<AGROW>
                end
            end
        end
    end
end

function s = compact_id_1b(qp, intg, nlp, hess)
    qpc = struct('FULL_CONDENSING_HPIPM',    'fch', ...
                 'PARTIAL_CONDENSING_HPIPM', 'pch', ...
                 'FULL_CONDENSING_QPOASES',  'fcq');
    nlpc = struct('SQP', 'sqp', 'SQP_RTI', 'rti');
    hessc = struct('GAUSS_NEWTON', 'gn', 'EXACT', 'ex');
    s = sprintf('%s_%s_%s_%s', lower(intg), nlpc.(nlp), qpc.(qp), hessc.(hess));
end

% =========================================================================
% Stage 2 — horizon x Tf x controller_n  for top-2 Stage-1b configs
% =========================================================================

function cfgs = stage_2(model_kind, ctx)
    cfgs = {};
    if ~isfield(ctx, 'top_1b') || isempty(ctx.top_1b)
        % Without picks, skip Stage 2 — orchestrator will refill and re-call.
        return;
    end

    N_grid  = [5 10 15 20 30];
    Tf_grid = [0.05 0.10 0.15 0.20 0.30];
    n_grid  = [5 10 15 20];
    dt_min  = 5e-3;
    dt_max  = 30e-3;

    for it = 1:length(ctx.top_1b)
        top = ctx.top_1b{it};
        for iN = 1:length(N_grid)
            for iT = 1:length(Tf_grid)
                dt = Tf_grid(iT) / N_grid(iN);
                if dt < dt_min - 1e-9 || dt > dt_max + 1e-9; continue; end
                for iN2 = 1:length(n_grid)
                    c = top;
                    c.stage         = '2';
                    c.exp_id        = sprintf('%s_N%d_T%03d_n%d', top.exp_id, ...
                                              N_grid(iN), round(Tf_grid(iT)*1000), n_grid(iN2));
                    c.N             = N_grid(iN);
                    c.Tf            = Tf_grid(iT);
                    c.controller_n  = n_grid(iN2);
                    cfgs{end+1} = c; %#ok<AGROW>
                end
            end
        end
    end
end

% =========================================================================
% Stage 3a — robustness on top-1
% =========================================================================

function cfgs = stage_3a(model_kind, ctx)
    cfgs = {};
    if ~isfield(ctx, 'top_2') || isempty(ctx.top_2)
        return;
    end
    top = ctx.top_2{1};

    % 5 directions x 3 magnitudes = 15  +  yaw probe x 3 = 18 total.
    dirs = robustness_directions();
    scales = [0.2, 0.5, 1.0];

    for d = 1:length(dirs)
        for s = 1:length(scales)
            c = top;
            c.stage     = '3a';
            c.exp_id    = sprintf('%s_%s', dirs{d}.label, scale_tag(scales(s)));
            c.ic_label  = sprintf('%s_s%s', dirs{d}.label, scale_tag(scales(s)));
            c.ic_direction = dirs{d}.vec;
            c.ic_scale     = scales(s);
            cfgs{end+1} = c; %#ok<AGROW>
        end
    end
end

function dirs = robustness_directions()
    dirs = {};
    dirs{end+1} = struct('label', 'd_legacy', 'vec', ...
        [0.005;-0.008;0.010; 0.15;-0.10; 0; 0.01;-0.01;0.0; 0.2;-0.3; 0]);
    dirs{end+1} = struct('label', 'd_pos',    'vec', ...
        [0.005; 0.005; 0.005; 0;     0;    0; 0;    0;   0;   0;   0;   0]);
    dirs{end+1} = struct('label', 'd_ori',    'vec', ...
        [0;     0;     0;    0.175; 0.175; 0; 0;    0;   0;   0;   0;   0]);
    dirs{end+1} = struct('label', 'd_lvel',   'vec', ...
        [0;     0;     0;    0;     0;    0; 0.30;-0.30;-0.20; 0;  0;   0]);
    dirs{end+1} = struct('label', 'd_avel',   'vec', ...
        [0;     0;     0;    0;     0;    0; 0;    0;   0;    1.0;-1.0; 0]);
    dirs{end+1} = struct('label', 'd_yaw',    'vec', ...
        [0.005; 0.005; 0.005; 0.10; 0.10; 0.20; 0; 0; 0; 0; 0; 0.5]);
end

% =========================================================================
% Stage 3b — timing stability for top-3 Stage-1b configs
% =========================================================================

function cfgs = stage_3b(model_kind, ctx)
    cfgs = {};
    if ~isfield(ctx, 'top_1b_three') || isempty(ctx.top_1b_three)
        return;
    end
    for k = 1:length(ctx.top_1b_three)
        c = ctx.top_1b_three{k};
        c.stage     = '3b';
        c.exp_id    = sprintf('%s_timing', c.exp_id);
        c.n_repeats = 5;
        cfgs{end+1} = c; %#ok<AGROW>
    end
end

% =========================================================================
% Helpers
% =========================================================================

function s = double_to_tag(x)
    s = strrep(sprintf('%g', x), '.', 'p');
end

function s = scale_tag(x)
    s = strrep(sprintf('s%g', x), '.', 'p');
end
