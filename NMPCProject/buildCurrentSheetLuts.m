function luts = buildCurrentSheetLuts(params, opts)
% BUILDCURRENTSHEETLUTS builds CasADi lookup tables for the current sheet
% magnetic field model. Two types of LUT:
%
%   1. Combined 3D Cartesian LUT for ALL permanent magnets:
%      Input (x,y,z) in global coords → output (bx,by,bz) total field.
%      Since permanent magnets have fixed positions and constant current,
%      their combined field is precomputed. No polar conversion at runtime.
%
%   2. Single 2D polar LUT for solenoids (shared geometry):
%      Input (z,rho) → output (brho,bz) per unit current, nw pre-baked.
%      At runtime, only multiply by u(i).
%
% Inputs:
%   params - Parameter struct
%   opts   - (optional) struct with fields:
%            rho_max   (default 0.1),  z_max  (default 0.06)
%            N_rho     (default 50),   N_z    (default 50, per half)
%            perm3d_xy (default 0.05), perm3d_z (default [0 0.06])
%            perm3d_N  (default 50)
%            method    'bspline' or 'linear' (default 'bspline')

    if nargin < 2, opts = struct(); end
    if ~isfield(opts, 'rho_max'),   opts.rho_max   = 0.1;        end
    if ~isfield(opts, 'z_max'),     opts.z_max     = 0.06;       end
    if ~isfield(opts, 'N_rho'),     opts.N_rho     = 50;         end
    if ~isfield(opts, 'N_z'),       opts.N_z       = 50;         end
    if ~isfield(opts, 'perm3d_xy'), opts.perm3d_xy = 0.05;       end
    if ~isfield(opts, 'perm3d_z'),  opts.perm3d_z  = [0, 0.06];  end
    if ~isfield(opts, 'perm3d_N'),  opts.perm3d_N  = 50;         end
    if ~isfield(opts, 'method'),    opts.method    = 'bspline';   end

    n     = params.magnet.n;
    n_sol = length(params.solenoids.r);

    % --- 3D combined permanent magnet LUT ---
    fprintf('Building 3D combined permanent magnet LUT (%d^3 grid)...\n', ...
            opts.perm3d_N);
    luts.perm_3d = buildCombinedPermanentMagnetLut(params, opts);
    luts.perm_3d_map = luts.perm_3d.map(n);
    fprintf('  Done.\n');

    % --- 2D solenoid LUT (nw pre-baked) ---
    nw = params.solenoids.nw;
    fprintf('Building 2D solenoid LUT (r=%.4f, l=%.4f, nw=%d)...\n', ...
            params.solenoids.r(1), params.solenoids.l(1), nw);
    luts.sol_field = buildSingleLut2D( ...
        params.solenoids.r(1), params.solenoids.l(1), ...
        params.physical.mu0, opts, 'sol', nw);
    luts.sol_field_map = luts.sol_field.map(n * n_sol);
    fprintf('  Done.\n');

    fprintf('LUT construction complete (method=%s).\n', opts.method);
end

%% 3D combined permanent magnet LUT
function interp_perm = buildCombinedPermanentMagnetLut(params, opts)
    import casadi.*

    mu0    = params.physical.mu0;
    n_perm = length(params.permanent.r);
    r_perm = params.permanent.r(1);
    l_perm = params.permanent.l(1);
    I_perm = params.permanent.J / mu0 * params.permanent.l(1);

    N      = opts.perm3d_N;
    xy_max = opts.perm3d_xy;

    x_vec = linspace(-xy_max, xy_max, N);
    y_vec = linspace(-xy_max, xy_max, N);
    z_vec = linspace(opts.perm3d_z(1), opts.perm3d_z(2), N);

    % ndgrid for CasADi interpolant ordering (dim1=x varies fastest)
    [X, Y, Z] = ndgrid(x_vec, y_vec, z_vec);

    BX = zeros(size(X));
    BY = zeros(size(X));
    BZ = zeros(size(X));

    total = numel(X);
    for k = 1:total
        xk = X(k);  yk = Y(k);  zk = Z(k);
        bx_sum = 0;  by_sum = 0;  bz_sum = 0;

        for i = 1:n_perm
            dx = xk - params.permanent.x(i);
            dy = yk - params.permanent.y(i);
            dz = zk - params.permanent.z(i);
            rho = sqrt(dx^2 + dy^2);

            % computeFieldCircularCurrentSheetPolarSmart handles ±z natively
            [brho_k, bz_k] = computeFieldCircularCurrentSheetPolarSmart( ...
                max(rho, 1e-12), dz, r_perm, l_perm, mu0);

            bz_sum = bz_sum + bz_k * I_perm;

            if rho > 1e-12
                bx_sum = bx_sum + brho_k * I_perm * dx / rho;
                by_sum = by_sum + brho_k * I_perm * dy / rho;
            end
        end

        BX(k) = bx_sum;
        BY(k) = by_sum;
        BZ(k) = bz_sum;
    end

    % 3-output interpolant: input [x;y;z] (3×1), output [bx;by;bz] (3×1)
    % Data interleaved: [bx_1, by_1, bz_1, bx_2, by_2, bz_2, ...]
    data = reshape([BX(:)'; BY(:)'; BZ(:)'], 1, []);

    interp_perm = interpolant('perm3d', opts.method, ...
                              {x_vec, y_vec, z_vec}, data);
end

%% 2D polar LUT for a single source geometry
function interp_field = buildSingleLut2D(r, l, mu0, opts, tag, scale)
    import casadi.*

    rho_vec  = linspace(0, opts.rho_max, opts.N_rho);
    N_z_full = 2 * opts.N_z - 1;
    z_vec    = linspace(-opts.z_max, opts.z_max, N_z_full);

    [RHO, ZZ] = meshgrid(rho_vec, z_vec);
    BRHO = zeros(size(RHO));
    BZ   = zeros(size(RHO));

    for k = 1:numel(RHO)
        rho_k = RHO(k);
        z_k   = ZZ(k);

        if abs(z_k) < 1e-12 && rho_k < r
            continue
        end

        [brho_k, bz_k] = computeFieldCircularCurrentSheetPolarSmart( ...
            max(rho_k, 1e-12), z_k, r, l, mu0);

        BRHO(k) = brho_k;
        BZ(k)   = bz_k;
    end

    % Bake scale factor
    BRHO = BRHO * scale;
    BZ   = BZ   * scale;

    % 2-output interpolant: input [z; rho] (2×1), output [brho; bz] (2×1)
    data = reshape([BRHO(:)'; BZ(:)'], 1, []);

    interp_field = interpolant(['field_' tag], opts.method, ...
                               {z_vec, rho_vec}, data);
end
