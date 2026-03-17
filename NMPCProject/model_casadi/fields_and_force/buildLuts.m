function luts = buildLuts(params, modelId)
% BUILDLUTS builds CasADi lookup tables for the magnetic field model,
% with automatic caching to avoid recomputation.
%
%   luts = buildLuts(params, modelId)
%
%   modelId selects the field function:
%     MaglevModel.Fast     - Wire-loop (computeFieldCircularWirePolar)
%     MaglevModel.Accurate - Current-sheet (computeFieldCircularCurrentSheetPolar)
%
% Two types of LUT:
%   1. Combined 3D Cartesian LUT for ALL permanent magnets:
%      Input (x,y,z) in global coords -> output (bx,by,bz) total field.
%
%   2. Single 2D polar LUT for solenoids (shared geometry):
%      Input (z,rho) -> output (brho,bz) per unit current, nw pre-baked.
%
% Also creates split single-output interpolants for efficient .map() usage.
%
% Caching: Grid data is saved to lut_cache.mat alongside this file.
%   On subsequent calls, if parameters match, interpolants are rebuilt
%   from cached data (fast) instead of recomputing the field grid (slow).

import casadi.*

    opts = params.lut_opts;

    % Build fingerprint of all parameters that affect LUT data
    fp = buildFingerprint(params, modelId);

    % Check cache
    cache_file = fullfile(fileparts(mfilename('fullpath')), 'lut_cache.mat');
    cache_valid = false;
    if isfile(cache_file)
        cache = load(cache_file, 'fingerprint');
        if isfield(cache, 'fingerprint') && isequal(cache.fingerprint, fp)
            cache_valid = true;
        else
            fprintf('LUT cache invalidated (parameters changed). Rebuilding...\n');
        end
    end

    if cache_valid
        % --- Load from cache ---
        fprintf('Loading LUTs from cache...\n');
        cache = load(cache_file);
        perm_x_vec = cache.perm_x_vec;
        perm_y_vec = cache.perm_y_vec;
        perm_z_vec = cache.perm_z_vec;
        perm_data  = cache.perm_data;
        sol_z_vec   = cache.sol_z_vec;
        sol_rho_vec = cache.sol_rho_vec;
        sol_data    = cache.sol_data;
    else
        % --- Build from scratch ---

        % 3D combined permanent magnet LUT
        fprintf('Building 3D combined permanent magnet LUT (%dx%dx%d grid)...\n', ...
                opts.perm3d_N_xy, opts.perm3d_N_xy, opts.perm3d_N_z);
        [~, perm_x_vec, perm_y_vec, perm_z_vec, perm_data] = ...
            buildCombinedPermanentMagnetLut(params, opts, modelId);
        fprintf('  Done.\n');

        % 2D solenoid LUT (nw pre-baked)
        nw = params.solenoids.nw;
        sol_r = params.solenoids.r(1);
        sol_l = params.solenoids.l(1);
        mu0 = params.physical.mu0;

        fprintf('Building 2D solenoid LUT (r=%.4f, l=%.4f, nw=%d)...\n', ...
                sol_r, sol_l, nw);
        [~, sol_z_vec, sol_rho_vec, sol_data] = ...
            buildSingleLut2D(sol_r, sol_l, mu0, opts, 'sol', nw, modelId);

        % Save cache
        fingerprint = fp; %#ok<NASGU>
        save(cache_file, 'fingerprint', ...
             'perm_x_vec', 'perm_y_vec', 'perm_z_vec', 'perm_data', ...
             'sol_z_vec', 'sol_rho_vec', 'sol_data', '-v7.3');
        fprintf('LUT cache saved.\n');
    end

    % --- Create all interpolants from grid data ---

    % Combined multi-output interpolants (for validation / backward compat)
    luts.perm_3d = interpolant('perm3d', opts.method, ...
        {perm_x_vec, perm_y_vec, perm_z_vec}, perm_data);
    luts.sol_field = interpolant('field_sol', opts.method, ...
        {sol_z_vec, sol_rho_vec}, sol_data);

    % Split single-output interpolants (for .map() usage)
    % perm_data layout: [BX(1),BY(1),BZ(1), BX(2),BY(2),BZ(2), ...]
    luts.perm_bx = interpolant('perm_bx', opts.method, ...
        {perm_x_vec, perm_y_vec, perm_z_vec}, perm_data(1:3:end));
    luts.perm_by = interpolant('perm_by', opts.method, ...
        {perm_x_vec, perm_y_vec, perm_z_vec}, perm_data(2:3:end));
    luts.perm_bz = interpolant('perm_bz', opts.method, ...
        {perm_x_vec, perm_y_vec, perm_z_vec}, perm_data(3:3:end));

    % sol_data layout: [BRHO(1),BZ(1), BRHO(2),BZ(2), ...]
    luts.sol_brho = interpolant('sol_brho', opts.method, ...
        {sol_z_vec, sol_rho_vec}, sol_data(1:2:end));
    luts.sol_bz = interpolant('sol_bz', opts.method, ...
        {sol_z_vec, sol_rho_vec}, sol_data(2:2:end));

    % Store discretization parameters
    luts.n       = params.magnet.n;
    luts.n_axial = params.magnet.n_axial;

    fprintf('LUTs ready (method=%s, model=%s).\n', opts.method, string(modelId));
end

%% Build fingerprint struct of all parameters that affect LUT data
function fp = buildFingerprint(params, modelId)
    fp.modelId = int32(modelId);
    fp.lut_opts = params.lut_opts;
    fp.permanent = params.permanent;
    fp.solenoids_r = params.solenoids.r(1);
    fp.solenoids_l = params.solenoids.l(1);
    fp.solenoids_nw = params.solenoids.nw;
    fp.mu0 = params.physical.mu0;
    fp.magnet_n = params.magnet.n;
    fp.magnet_n_axial = params.magnet.n_axial;
end

%% 3D combined permanent magnet LUT
function [interp_perm, x_vec, y_vec, z_vec, data] = buildCombinedPermanentMagnetLut(params, opts, modelId)
    import casadi.*

    mu0    = params.physical.mu0;
    n_perm = length(params.permanent.r);
    r_perm = params.permanent.r(1);
    l_perm = params.permanent.l(1);
    I_perm = params.permanent.J / mu0 * params.permanent.l(1);

    N_xy   = opts.perm3d_N_xy;
    N_z    = opts.perm3d_N_z;
    xy_max = opts.perm3d_max_xy;

    x_vec = linspace(-xy_max, xy_max, N_xy);
    y_vec = linspace(-xy_max, xy_max, N_xy);

    z_lo = opts.perm3d_max_z(1);
    z_hi = opts.perm3d_max_z(2);
    z_vec = linspace(z_lo, z_hi, N_z);

    fprintf('    Computing field on base grid (%dx%dx%d = %d points)...\n', ...
            N_xy, N_xy, N_z, N_xy*N_xy*N_z);
    [X0, Y0, Z0] = ndgrid(x_vec, y_vec, z_vec);
    BX = zeros(size(X0));
    BY = zeros(size(X0));
    BZ = zeros(size(X0));

    total = numel(X0);
    next_pct = 10;
    for k = 1:total
        xk = X0(k);  yk = Y0(k);  zk = Z0(k);
        bx_sum = 0;  by_sum = 0;  bz_sum = 0;
        for i = 1:n_perm
            dx = xk - params.permanent.x(i);
            dy = yk - params.permanent.y(i);
            dz = zk - params.permanent.z(i);
            rho = sqrt(dx^2 + dy^2);

            switch modelId
                case MaglevModel.Fast
                    [~, brho_k, bz_k] = computeFieldCircularWirePolar( ...
                        0, rho, dz, r_perm, I_perm, mu0);
                otherwise  % Accurate
                    [~, brho_k, bz_k] = computeFieldCircularCurrentSheetPolar( ...
                        0, rho, dz, r_perm, l_perm, I_perm, mu0);
            end

            bz_sum = bz_sum + bz_k;
            if rho > 1e-12
                bx_sum = bx_sum + brho_k * dx / rho;
                by_sum = by_sum + brho_k * dy / rho;
            end
        end
        BX(k) = bx_sum;  BY(k) = by_sum;  BZ(k) = bz_sum;
        if k / total * 100 >= next_pct
            fprintf('    %d%%\n', next_pct);
            next_pct = next_pct + 10;
        end
    end

    data = reshape([BX(:)'; BY(:)'; BZ(:)'], 1, []);
    interp_perm = interpolant('perm3d', opts.method, ...
                              {x_vec, y_vec, z_vec}, data);
end

%% 2D polar LUT for a single source geometry
function [interp_field, z_vec, rho_vec, data] = buildSingleLut2D(r, l, mu0, opts, tag, scale, modelId)
    import casadi.*

    N_rho = opts.sol2d_N_rho;
    N_z = opts.sol2d_N_z;
    rho_max = opts.sol2d_max_rho;
    z_max = opts.sol2d_max_z;

    rho_vec = linspace(0, rho_max, N_rho);
    z_vec = linspace(-l/2, z_max, N_z);

    [ZZ, RHO] = ndgrid(z_vec, rho_vec);
    BRHO = zeros(size(ZZ));
    BZ   = zeros(size(ZZ));

    for k = 1:numel(RHO)
        rho_k = RHO(k);
        z_k   = ZZ(k);

        switch modelId
            case MaglevModel.Fast
                [~, brho_k, bz_k] = computeFieldCircularWirePolar( ...
                    0, rho_k, z_k, r, 1, mu0);
            otherwise  % Accurate
                [~, brho_k, bz_k] = computeFieldCircularCurrentSheetPolar( ...
                    0, rho_k, z_k, r, l, 1, mu0);
        end

        if isnan(brho_k), brho_k = 0; end
        if isnan(bz_k),   bz_k   = 0; end

        BRHO(k) = brho_k;
        BZ(k)   = bz_k;
    end

    % Bake scale factor
    BRHO = BRHO * scale;
    BZ   = BZ   * scale;

    data = reshape([BRHO(:)'; BZ(:)'], 1, []);
    interp_field = interpolant(['field_' tag], opts.method, ...
                               {z_vec, rho_vec}, data);
end
