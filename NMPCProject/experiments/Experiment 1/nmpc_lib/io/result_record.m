function rec = result_record(cfg, M, R, classification, reason, build_meta)
% RESULT_RECORD  Build the canonical thesis-framework result record.
%
%   rec = result_record(cfg, M, R, classification, reason, build_meta)
%
% Inputs
%   cfg            validated cfg
%   M              build_model output (we keep a slim slice; the full struct
%                  is recomputable from cfg + params and would bloat the .mat)
%   R              run_closed_loop output (FULL trajectory + timing)
%   classification 'CONVERGED_STABLE' | ... (from classify_outcome)
%   reason         human-readable string from classify_outcome
%   build_meta     struct from build_ocp (model_name, build_time_s,
%                  asymmetric_plant, cfg_hash)
%
% Output rec is a self-contained struct with the following layout. Two
% runs with identical cfg_hash and matched params_hash + matched
% schema_version must produce equivalent trajectories (within numerical
% solver noise).
%
%   .schema_version      e.g. "2026.05.13.1"   (from env_info)
%   .timestamp           ISO 8601 local time of record creation
%   .run_id              UUID v4
%   .cfg_hash            deterministic SHA-256 of the validated cfg
%   .params_hash         SHA-256 of M.params_plant (canonical-form)
%   .classification      string
%   .reason              string
%
%   .env_info            full env_info() snapshot at record-creation time
%
%   .cfg                 the validated cfg (full)
%
%   .model               slim model metadata:
%       .order, .nx_ctrl, .nx_plant, .nu, .ctrl_idx
%       .magnet_n_ctrl, .magnet_n_plant
%       .xEq_plant, .xEq_ctrl, .uEq
%
%   .build_meta          from build_ocp
%
%   .run                 raw R from run_closed_loop (full trajectories,
%                        per-step timings, per-step status / iterations)
%
%   .summary             merged metric struct:
%                          timing_stats + tracking_metrics
%                          + disturbance_metrics + convergence_metrics
%                          + .classification, .reason, .n_actual, .diverged

    if nargin < 6 || isempty(build_meta); build_meta = struct(); end

    info  = env_info();
    rec   = struct();

    rec.schema_version = info.schema_version;
    rec.timestamp      = info.timestamp;
    rec.run_id         = uuid_v4();
    rec.cfg_hash       = hash_cfg(cfg);
    % Canonical hash of the USER-PROVIDED params (pre-solenoid-correction),
    % as cached by build_dynamics. The post-correction params are derivable
    % from this hash + magnet_n via the framework's deterministic build.
    rec.params_hash    = M.equilibrium.params_hash;
    rec.classification = classification;
    rec.reason         = reason;

    rec.env_info       = info;
    rec.cfg            = cfg;

    rec.model = struct( ...
        'order',           M.order, ...
        'nx_ctrl',         M.nx_ctrl, ...
        'nx_plant',        M.nx_plant, ...
        'nu',              M.nu, ...
        'ctrl_idx',        M.ctrl_idx, ...
        'magnet_n_ctrl',   M.magnet_n_ctrl, ...
        'magnet_n_plant',  M.magnet_n_plant, ...
        'xEq_plant',       M.xEq_plant, ...
        'xEq_ctrl',        M.xEq_ctrl, ...
        'uEq',             M.uEq, ...
        'equilibrium',     M.equilibrium ...
    );

    rec.build_meta = build_meta;

    rec.run = R;

    % --- Summary: merge the four metric modules ----------------------
    S = struct();
    S = merge_(S, timing_stats(R, cfg));
    S = merge_(S, tracking_metrics(R, M, cfg));
    S = merge_(S, disturbance_metrics(R, M, cfg));
    S = merge_(S, convergence_metrics(R));
    S.classification = classification;
    S.reason         = reason;
    S.n_actual       = R.n_actual;
    S.diverged       = R.diverged;
    rec.summary      = S;
end

% ---------------------------------------------------------------------- %

function S = merge_(S, T)
    fns = fieldnames(T);
    for i = 1:numel(fns)
        if isfield(S, fns{i})
            warning('result_record:dup_metric', ...
                    'Metric "%s" defined in two modules; later one wins.', fns{i});
        end
        S.(fns{i}) = T.(fns{i});
    end
end

function u = uuid_v4()
    try
        u = char(java.util.UUID.randomUUID().toString());
    catch
        % Fallback: 32 hex chars from randi.
        a = dec2hex(randi([0 15], 1, 32) - 1, 1);
        u = lower(a(:).');
    end
end
