%% validate_lut_polar_field.m
% Step 1: Validate LUT interpolation vs direct computeFieldCircularCurrentSheetPolarSmart
%
% Tests the 2D solenoid LUT (before nw scaling) by comparing interpolated
% values at random (rho, z) points against the analytical function.
%
% Pass criteria: max relative error < 1% on brho and bz

% clear; clc;
addpath(genpath('..'));

%% Setup
load_params(MaglevModel.Accurate);

luts = buildLuts(params);

%% Generate random test points
rng(42);
nTest = 500;

% Solenoid parameters
r_sol = params.solenoids.r(1);
l_sol = params.solenoids.l(1);
mu0   = params.physical.mu0;
nw    = params.solenoids.nw;

% Random points in valid LUT range, avoiding singularities
rho_test = rand(1, nTest) * params.lut_opts.sol2d_rho_max * 0.95;
z_lo_sol = -l_sol/2;
z_hi_sol = params.lut_opts.sol2d_z_max * 0.95;
z_test   = z_lo_sol + rand(1, nTest) * (z_hi_sol - z_lo_sol);

% Exclude points inside the solenoid (abs(z) < l and rho < r)
% and near the singularity at rho ~= r
% bad = (abs(z_test) < l_sol/2) & (rho_test == r_sol);
% rho_test(bad) = r_sol * 1.5;

%% Evaluate: LUT (includes nw baking)
brho_lut = zeros(1, nTest);
bz_lut   = zeros(1, nTest);

for k = 1:nTest
    result = full(luts.sol_field([z_test(k); rho_test(k)]));
    brho_lut(k) = result(1);
    bz_lut(k)   = result(2);
end

%% Evaluate: Direct analytical (non-smart version, with same nw scaling)
brho_ref = zeros(1, nTest);
bz_ref   = zeros(1, nTest);

for k = 1:nTest
    [~, br, bz_k] = computeFieldCircularCurrentSheetPolar( ...
        0, rho_test(k), z_test(k), r_sol, l_sol, 1, mu0);
    brho_ref(k) = br * nw;
    bz_ref(k)   = bz_k * nw;
end

%% Compute errors
% Mixed relative error: |lut - ref| / (|ref| + eps_abs)
% This handles zero-crossings gracefully — denominator never vanishes.
eps_brho = max(abs(brho_ref)) * 0.01;   % 1% of peak as absolute floor
eps_bz   = max(abs(bz_ref))   * 0.01;

mixed_err_brho = abs(brho_lut - brho_ref) ./ (abs(brho_ref) + eps_brho);
mixed_err_bz   = abs(bz_lut - bz_ref)     ./ (abs(bz_ref)   + eps_bz);

abs_err_brho = abs(brho_lut - brho_ref);
abs_err_bz   = abs(bz_lut - bz_ref);

fprintf('\n=== Step 1: LUT vs Polar Field (Solenoid, non-smart ref) ===\n');
fprintf('Test points: %d\n', nTest);
fprintf('brho - max mixed error: %.4f%%  mean: %.4f%%\n', ...
    max(mixed_err_brho)*100, mean(mixed_err_brho)*100);
fprintf('bz   - max mixed error: %.4f%%  mean: %.4f%%\n', ...
    max(mixed_err_bz)*100, mean(mixed_err_bz)*100);
fprintf('Max absolute errors: brho=%.2e, bz=%.2e\n', max(abs_err_brho), max(abs_err_bz));

% Show worst points
[~, idx_worst_bz] = max(mixed_err_bz);
fprintf('Worst bz point: rho=%.6f, z=%.6f, lut=%.6e, ref=%.6e\n', ...
    rho_test(idx_worst_bz), z_test(idx_worst_bz), bz_lut(idx_worst_bz), bz_ref(idx_worst_bz));

pass = max(mixed_err_brho) < 0.01 && max(mixed_err_bz) < 0.01;
if pass
    fprintf('PASS\n');
else
    fprintf('FAIL\n');
end

%% Error heatmap (dense regular grid)
plot_heatmap = true;
if plot_heatmap
    N_hm = 300;
    rho_hm = linspace(0, params.lut_opts.sol2d_rho_max * 0.95, N_hm);
    z_hm   = linspace(-l_sol/2, params.lut_opts.sol2d_z_max * 0.95, N_hm);
    [RHO_HM, Z_HM] = meshgrid(rho_hm, z_hm);

    ERR_BRHO = zeros(size(RHO_HM));
    ERR_BZ   = zeros(size(RHO_HM));
    FIELD_BRHO = zeros(size(RHO_HM));
    FIELD_BZ   = zeros(size(RHO_HM));

    for k = 1:numel(RHO_HM)
        rk = RHO_HM(k);
        zk = Z_HM(k);

        res = full(luts.sol_field([zk; rk]));
        [~, br, bzk] = computeFieldCircularCurrentSheetPolar( ...
            0, max(rk, 1e-12), zk, r_sol, l_sol, 1, mu0);

        brho_r = br * nw;
        bz_r   = bzk * nw;

        % Mixed error: handles zero-crossings gracefully
        ERR_BRHO(k) = abs(res(1) - brho_r) / (abs(brho_r) + eps_brho);
        ERR_BZ(k)   = abs(res(2) - bz_r)   / (abs(bz_r)   + eps_bz);

        % Store reference field for quiver overlay
        FIELD_BRHO(k) = brho_r;
        FIELD_BZ(k)   = bz_r;
    end

    % Normalized field arrows on a coarser grid
    N_qv = 25;
    skip_r = max(1, round(N_hm / N_qv));
    skip_z = skip_r;
    idx_r = 1:skip_r:N_hm;
    idx_z = 1:skip_z:N_hm;
    QR = RHO_HM(idx_z, idx_r) * 1e3;
    QZ = Z_HM(idx_z, idx_r) * 1e3;
    U  = FIELD_BRHO(idx_z, idx_r);
    V  = FIELD_BZ(idx_z, idx_r);
    mag = sqrt(U.^2 + V.^2) + 1e-20;
    U = U ./ mag;
    V = V ./ mag;

    figure('Name', 'LUT Error Heatmaps');

    subplot(1,2,1);
    imagesc(rho_hm*1e3, z_hm*1e3, ERR_BRHO*100);
    set(gca, 'YDir', 'normal');
    hold on;
    rectangle('Position', [0, -l_sol/2*1e3, r_sol*1e3, l_sol*1e3], ...
              'EdgeColor', 'w', 'LineWidth', 1.5, 'LineStyle', '--');
    quiver(QR, QZ, U, V, 0.4, 'Color', [0.3 0.5 1], 'LineWidth', 0.8);
    colorbar; caxis([0, min(5, max(ERR_BRHO(:)*100))]);
    xlabel('\rho [mm]'); ylabel('z [mm]');
    title('brho relative error [%]');

    subplot(1,2,2);
    imagesc(rho_hm*1e3, z_hm*1e3, ERR_BZ*100);
    set(gca, 'YDir', 'normal');
    hold on;
    rectangle('Position', [0, -l_sol/2*1e3, r_sol*1e3, l_sol*1e3], ...
              'EdgeColor', 'w', 'LineWidth', 1.5, 'LineStyle', '--');
    quiver(QR, QZ, U, V, 0.4, 'Color', [0.3 0.5 1], 'LineWidth', 0.8);
    colorbar; caxis([0, min(5, max(ERR_BZ(:)*100))]);
    xlabel('\rho [mm]'); ylabel('z [mm]');
    title('bz relative error [%]');

    colormap(hot);
end
