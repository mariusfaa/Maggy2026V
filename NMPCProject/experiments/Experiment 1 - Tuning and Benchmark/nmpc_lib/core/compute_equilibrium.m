function eq = compute_equilibrium(dyn, opts)
% COMPUTE_EQUILIBRIUM  Find the canonical hover equilibrium (z, with all
% other states zero and zero current) for a given dynamics function.
%
%   eq = compute_equilibrium(dyn)
%   eq = compute_equilibrium(dyn, opts)
%
% Inputs
%   dyn       struct from build_dynamics() (uses dyn.f_func, dyn.magnet_n,
%             dyn.params_hash)
%   opts      optional struct with fields:
%     .lbx          lower bound on zEq (default 0.015)
%     .ubx          upper bound on zEq (default 0.060)
%     .x0           initial guess (default 0.030)
%     .print_level  IPOPT print level (default 0)
%     .use_cache    true | false (default true). When true, results are
%                   cached at <cache_root>/equilibrium/eq_n<n>_<hash[:8]>.mat
%
% Output (struct eq)
%   .zEq        scalar z equilibrium [m]
%   .xEq        12-element equilibrium state [0;0;zEq;zeros(9,1)]
%   .uEq        4-element zero input
%   .residual   |f_z(xEq, uEq)| at the solution (sanity check; should be < 1e-8)
%   .ipopt_status  whatever IPOPT returned via stats.return_status
%
% Caching: the canonical equilibrium depends ONLY on (magnet_n, params_hash)
% via dyn. The same dyn always yields the same equilibrium; the cache
% saves the IPOPT cost on subsequent sessions.

    if nargin < 2; opts = struct(); end
    opts = with_default(opts, 'lbx',         0.015);
    opts = with_default(opts, 'ubx',         0.060);
    opts = with_default(opts, 'x0',          0.030);
    opts = with_default(opts, 'print_level', 0);
    opts = with_default(opts, 'use_cache',   true);

    if opts.use_cache
        cached = try_cache(dyn);
        if ~isempty(cached); eq = cached; return; end
    end

    import casadi.*
    z   = SX.sym('z_eq');
    x12 = [0; 0; z; zeros(9,1)];
    accel = dyn.f_func(x12, zeros(4,1));
    fz    = accel(9);
    nlp   = struct('x', z, 'f', fz^2);
    solver = nlpsol('eq', 'ipopt', nlp, ...
        struct('ipopt', struct('print_level', opts.print_level, ...
                               'sb','yes', 'print_timing_statistics','no')));
    sol = solver('x0', opts.x0, 'lbx', opts.lbx, 'ubx', opts.ubx);

    zEq = full(sol.x);
    xEq = [0; 0; zEq; zeros(9,1)];
    uEq = zeros(4,1);

    % Sanity: evaluate the residual at the solution numerically.
    accel_val = full(dyn.f_func(xEq, uEq));
    residual  = abs(accel_val(9));

    stats = solver.stats();
    ipopt_status = 'unknown';
    if isstruct(stats) && isfield(stats,'return_status')
        ipopt_status = stats.return_status;
    end

    eq = struct( ...
        'zEq',          zEq, ...
        'xEq',          xEq, ...
        'uEq',          uEq, ...
        'residual',     residual, ...
        'ipopt_status', ipopt_status, ...
        'magnet_n',     dyn.magnet_n, ...
        'params_hash',  dyn.params_hash ...
    );

    if opts.use_cache; write_cache(eq); end
end

% ---------------------------------------------------------------------- %

function eq = try_cache(dyn)
    eq = [];
    fpath = cache_path(dyn);
    if exist(fpath,'file')
        S = load(fpath);
        if isfield(S,'eq') && strcmp(S.eq.params_hash, dyn.params_hash) ...
                          && S.eq.magnet_n == dyn.magnet_n
            eq = S.eq;
        end
    end
end

function write_cache(eq)
    fpath = cache_path(eq);
    timestamp = char(datetime('now','Format','yyyy-MM-dd''T''HH:mm:ssXXX','TimeZone','local')); %#ok<NASGU>
    save(fpath, 'eq', 'timestamp');
end

function p = cache_path(dynOrEq)
    dir_path = cache_root('equilibrium');
    p = fullfile(dir_path, sprintf('eq_n%d_%s.mat', ...
        dynOrEq.magnet_n, dynOrEq.params_hash(1:8)));
end

function s = with_default(s, fn, v)
    if ~isfield(s, fn) || isempty(s.(fn)); s.(fn) = v; end
end
