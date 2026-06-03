function dyn = build_dynamics(magnet_n, params_in)
% BUILD_DYNAMICS  Build the maglev CasADi dynamics for a given magnet_n.
%
%   dyn = build_dynamics(magnet_n)
%   dyn = build_dynamics(magnet_n, params_in)
%
% Inputs
%   magnet_n   integer >= 3 (default 10 -- baseline). Number of integration
%              points used to discretize the levitating magnet circumference
%              in the force / torque integrals.
%   params_in  optional params struct. If omitted, parameters_maggy_V4 is
%              loaded as the canonical baseline. The magnet.n field of the
%              returned dyn.params is overridden to match magnet_n.
%
% Output (struct dyn)
%   .f_func        CasADi Function f(x, u) -> dx with x in R^12, u in R^4
%   .params        params struct used, with magnet.n = magnet_n and the
%                  solenoid radius scaled by the 'fast' correction factor
%   .magnet_n      the resolved magnet_n
%   .cf            solenoid radius correction factor (computeSolenoidRadiusCorrectionFactor)
%   .params_hash   SHA-256 of params_in (pre-correction). Used as a cache key.
%
% Caching
%   - Session-persistent CasADi Function cache keyed on (magnet_n, params_hash).
%   - Cross-session file cache for the slow correction-factor computation
%     at <cache_root>/dynamics/cf_n<magnet_n>_<hash[:8]>.mat .
%
% The CasADi Function itself is rebuilt on each MATLAB session (cheap and
% guarantees compatibility with the current CasADi build).

    if nargin < 1 || isempty(magnet_n); magnet_n = 10; end
    if nargin < 2 || isempty(params_in)
        params_in = load_default_params();
    end

    validateattributes(magnet_n,  {'numeric'}, {'scalar','integer','>=',3});
    validateattributes(params_in, {'struct'},  {});

    h = hash_cfg(params_in);

    persistent cache
    if isempty(cache)
        cache = containers.Map('KeyType','char','ValueType','any');
    end
    key = sprintf('n%d_%s', magnet_n, h(1:16));
    if isKey(cache, key)
        dyn = cache(key);
        return
    end

    params = params_in;
    params.magnet.n = magnet_n;

    cf = correction_factor(magnet_n, params, h);
    params.solenoids.r = cf * params.solenoids.r;

    import casadi.*
    x = SX.sym('x', 12);
    u = SX.sym('u',  4);
    f_expl = maglevSystemDynamicsCasADi(x, u, params);
    f_func = Function('f', {x, u}, {f_expl});

    dyn = struct( ...
        'f_func',      f_func, ...
        'params',      params, ...
        'magnet_n',    magnet_n, ...
        'cf',          cf, ...
        'params_hash', h ...
    );
    cache(key) = dyn;
end

% ---------------------------------------------------------------------- %

function params = load_default_params()
% Wraps the parameters_maggy_V4 script so its `params` variable is captured
% in this function's workspace and returned as the output.
    parameters_maggy_V4; %#ok<NASGU>  defines `params` in local scope
end

function cf = correction_factor(magnet_n, params, params_hash)
% File-cached solenoid radius correction factor.
    dir_path = cache_root('dynamics');
    fname    = sprintf('cf_n%d_%s.mat', magnet_n, params_hash(1:8));
    fpath    = fullfile(dir_path, fname);

    if exist(fpath, 'file')
        S = load(fpath);
        if isfield(S,'cf') && isfield(S,'params_hash') && isfield(S,'magnet_n') ...
                && strcmp(S.params_hash, params_hash) && S.magnet_n == magnet_n
            cf = S.cf;
            fprintf('[build_dynamics] cache hit: cf=%.9f (magnet_n=%d)\n', cf, magnet_n);
            return
        end
    end

    fprintf(['[build_dynamics] computing correction factor (magnet_n=%d). ' ...
             'This is the slow step; subsequent sessions will reuse the cache.\n'], magnet_n);
    cf = computeSolenoidRadiusCorrectionFactor(params, 'fast'); %#ok<NASGU>
    timestamp = char(datetime('now','Format','yyyy-MM-dd''T''HH:mm:ssXXX','TimeZone','local')); %#ok<NASGU>
    save(fpath, 'cf', 'params_hash', 'magnet_n', 'timestamp');
end
