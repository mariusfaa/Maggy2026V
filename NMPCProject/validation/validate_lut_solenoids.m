%% validate_lut_solenoids.m
% Step 3: Validate solenoid LUT contribution vs computeFieldBase
%
% Tests the 2D solenoid LUT with polar->cart conversion and u scaling
% against the MATLAB computeFieldBase solenoid contribution.
%
% Pass criteria: max relative error < 1%

% clear; clc;
addpath(genpath('..'));

%% Setup
load_params(MaglevModel.Accurate);

luts = buildLuts(params);

%% Generate random test points and currents
rng(42);
nTest = 200;

l_sol = params.solenoids.l(1);
r_sol = params.solenoids.r(1);
n_sol = length(params.solenoids.r);

% Generate points, rejecting any inside a solenoid
x_test = []; y_test = []; z_test = [];
z_min = params.lut_opts.perm3d_z(1);
z_max = params.lut_opts.perm3d_z(2);

while length(x_test) < nTest
    xc = (2*rand - 1) * params.lut_opts.perm3d_xy;
    yc = (2*rand - 1) * params.lut_opts.perm3d_xy;
    zc = z_min + (z_max - z_min) * rand;

    % Check if inside any solenoid (rho < r AND |dz| < l/2)
    inside = false;
    for i = 1:n_sol
        dx = xc - params.solenoids.x(i);
        dy = yc - params.solenoids.y(i);
        dz = zc - params.solenoids.z(i);
        rho = sqrt(dx^2 + dy^2);
        if rho < r_sol && abs(dz) < l_sol/2
            inside = true;
            break;
        end
    end
    if ~inside
        x_test = [x_test, xc]; %#ok<AGROW>
        y_test = [y_test, yc]; %#ok<AGROW>
        z_test = [z_test, zc]; %#ok<AGROW>
    end
end

% Random currents
u_test = (2*rand(4, 1) - 1);

%% Evaluate: MATLAB reference (solenoid contribution only)
% Total field with solenoids
[bx_total, by_total, bz_total] = computeFieldBase( ...
    x_test, y_test, z_test, u_test, params, MaglevModel.Accurate);

% Permanent magnet field only
u_zero = zeros(4, 1);
[bx_perm, by_perm, bz_perm] = computeFieldBase( ...
    x_test, y_test, z_test, u_zero, params, MaglevModel.Accurate);

% Solenoid contribution = total - permanent
bx_sol_ref = bx_total - bx_perm;
by_sol_ref = by_total - by_perm;
bz_sol_ref = bz_total - bz_perm;

%% Evaluate: LUT-based solenoid field
n_sol = length(params.solenoids.r);
eps_val = 1e-9;

bx_sol_lut = zeros(1, nTest);
by_sol_lut = zeros(1, nTest);
bz_sol_lut = zeros(1, nTest);

for k = 1:nTest
    for i = 1:n_sol
        dx = x_test(k) - params.solenoids.x(i);
        dy = y_test(k) - params.solenoids.y(i);
        dz = z_test(k) - params.solenoids.z(i);
        rho = sqrt(dx^2 + dy^2);

        result = full(luts.sol_field([dz; rho]));
        brho = result(1);
        bz_f = result(2);

        % Polar -> Cartesian
        bx_k = brho * dx / (rho + eps_val);
        by_k = brho * dy / (rho + eps_val);

        % Scale by current
        bx_sol_lut(k) = bx_sol_lut(k) + bx_k * u_test(i);
        by_sol_lut(k) = by_sol_lut(k) + by_k * u_test(i);
        bz_sol_lut(k) = bz_sol_lut(k) + bz_f * u_test(i);
    end
end

%% Compute errors (mixed metric)
fields = {'bx', 'by', 'bz'};
lut_vals = {bx_sol_lut, by_sol_lut, bz_sol_lut};
ref_vals = {bx_sol_ref, by_sol_ref, bz_sol_ref};

fprintf('\n=== Step 3: Solenoid LUT vs MATLAB ===\n');
fprintf('Test points: %d, u = [%.2f, %.2f, %.2f, %.2f]\n', ...
    nTest, u_test(1), u_test(2), u_test(3), u_test(4));

all_pass = true;
for f = 1:3
    eps_f = max(abs(ref_vals{f})) * 0.01;
    mixed_err = abs(lut_vals{f} - ref_vals{f}) ./ (abs(ref_vals{f}) + eps_f);
    fprintf('%s - max mixed error: %.4f%%  mean: %.4f%%\n', ...
        fields{f}, max(mixed_err)*100, mean(mixed_err)*100);
    if max(mixed_err) >= 0.01
        all_pass = false;
    end
end

fprintf('Max absolute errors: bx=%.2e, by=%.2e, bz=%.2e\n', ...
    max(abs(bx_sol_lut-bx_sol_ref)), max(abs(by_sol_lut-by_sol_ref)), ...
    max(abs(bz_sol_lut-bz_sol_ref)));

if all_pass
    fprintf('PASS\n');
else
    fprintf('FAIL\n');
end

%% Diagnostic plots
figure('Name', 'Step 3: Solenoid LUT Validation', 'Position', [100 100 1400 900]);

% --- Plot 1: Scatter of LUT vs reference for each component ---
titles = {'bx solenoid', 'by solenoid', 'bz solenoid'};
for f = 1:3
    subplot(2,3,f);
    plot(ref_vals{f}*1e6, lut_vals{f}*1e6, 'b.', 'MarkerSize', 4);
    hold on;
    lims = [min([ref_vals{f}, lut_vals{f}])*1e6, max([ref_vals{f}, lut_vals{f}])*1e6];
    plot(lims, lims, 'r-', 'LineWidth', 1);
    xlabel('Reference [\muT]'); ylabel('LUT [\muT]');
    title(titles{f});
    grid on; axis equal;
end

% --- Plot 2: Error vs position for each component ---
% Color by mixed error magnitude
for f = 1:3
    subplot(2,3,3+f);
    eps_f = max(abs(ref_vals{f})) * 0.01;
    err = abs(lut_vals{f} - ref_vals{f}) ./ (abs(ref_vals{f}) + eps_f) * 100;
    scatter3(x_test*1e3, y_test*1e3, z_test*1e3, 15, err, 'filled');
    hold on;
    % Draw solenoid positions
    for i = 1:n_sol
        plot3(params.solenoids.x(i)*1e3, params.solenoids.y(i)*1e3, ...
              params.solenoids.z(i)*1e3, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end
    colorbar; colormap(hot);
    xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
    title(sprintf('%s error [%%]', fields{f}));
    view(30, 25); grid on;
end

sgtitle(sprintf('Step 3: Solenoid LUT vs MATLAB (u=[%.2f,%.2f,%.2f,%.2f])', ...
    u_test(1), u_test(2), u_test(3), u_test(4)));

%% Per-solenoid diagnostic: check individual LUT queries
figure('Name', 'Step 3: Per-solenoid LUT query check', 'Position', [100 100 1200 400]);

% Pick one test point with largest bz error
eps_bz = max(abs(bz_sol_ref)) * 0.01;
bz_err = abs(bz_sol_lut - bz_sol_ref) ./ (abs(bz_sol_ref) + eps_bz);
[~, worst_k] = max(bz_err);

fprintf('\nWorst point #%d: x=%.4f, y=%.4f, z=%.4f\n', ...
    worst_k, x_test(worst_k), y_test(worst_k), z_test(worst_k));
fprintf('  bz_lut=%.4e, bz_ref=%.4e\n', bz_sol_lut(worst_k), bz_sol_ref(worst_k));

% Show each solenoid's contribution at worst point
for i = 1:n_sol
    dx = x_test(worst_k) - params.solenoids.x(i);
    dy = y_test(worst_k) - params.solenoids.y(i);
    dz = z_test(worst_k) - params.solenoids.z(i);
    rho = sqrt(dx^2 + dy^2);

    result = full(luts.sol_field([dz; rho]));
    brho_lut_i = result(1);
    bz_lut_i = result(2);

    [~, brho_ref_i, bz_ref_i] = computeFieldCircularCurrentSheetPolar( ...
        0, rho, dz, params.solenoids.r(1), params.solenoids.l(1), 1, params.physical.mu0);
    brho_ref_i = brho_ref_i * params.solenoids.nw;
    bz_ref_i = bz_ref_i * params.solenoids.nw;

    fprintf('  Sol %d: dz=%.4f, rho=%.4f | brho: lut=%.4e ref=%.4e | bz: lut=%.4e ref=%.4e\n', ...
        i, dz, rho, brho_lut_i, brho_ref_i, bz_lut_i, bz_ref_i);
end

% Heatmap: solenoid field over (dz, rho) grid — reference vs LUT with arrows
N_hm = 80;
dz_hm = linspace(-params.solenoids.l(1)/2 * 0.9, params.lut_opts.sol2d_z_max*0.95, N_hm);
rho_hm = linspace(1e-4, params.lut_opts.sol2d_rho_max*0.95, N_hm);
[DZ_HM, RHO_HM] = ndgrid(dz_hm, rho_hm);
BRHO_LUT_HM = zeros(size(DZ_HM));
BZ_LUT_HM   = zeros(size(DZ_HM));
BRHO_REF_HM = zeros(size(DZ_HM));
BZ_REF_HM   = zeros(size(DZ_HM));

for k = 1:numel(DZ_HM)
    res = full(luts.sol_field([DZ_HM(k); RHO_HM(k)]));
    BRHO_LUT_HM(k) = res(1);
    BZ_LUT_HM(k)   = res(2);
    [~, brho_k, bz_k] = computeFieldCircularCurrentSheetPolar( ...
        0, max(RHO_HM(k),1e-12), DZ_HM(k), params.solenoids.r(1), ...
        params.solenoids.l(1), 1, params.physical.mu0);
    BRHO_REF_HM(k) = brho_k * params.solenoids.nw;
    BZ_REF_HM(k)   = bz_k * params.solenoids.nw;
end

% Arrow subsampling
skip = max(1, round(N_hm/15));
idx = 1:skip:N_hm;
QDZ  = DZ_HM(idx,idx)*1e3;
QRHO = RHO_HM(idx,idx)*1e3;

% Reference arrows (blue)
U_ref = BRHO_REF_HM(idx,idx);  V_ref = BZ_REF_HM(idx,idx);
mag_ref = sqrt(U_ref.^2 + V_ref.^2) + 1e-20;
% LUT arrows (red)
U_lut = BRHO_LUT_HM(idx,idx);  V_lut = BZ_LUT_HM(idx,idx);
mag_lut = sqrt(U_lut.^2 + V_lut.^2) + 1e-20;

% Draw solenoid outline helper
sol_r_mm = params.solenoids.r(1)*1e3;
sol_l_mm = params.solenoids.l(1)*1e3;

subplot(1,3,1);
imagesc(dz_hm*1e3, rho_hm*1e3, BZ_REF_HM');
set(gca, 'YDir', 'normal'); colorbar;
hold on;
quiver(QDZ, QRHO, V_ref./mag_ref, U_ref./mag_ref, 0.4, 'Color', [0.3 0.5 1], 'LineWidth', 0.8);
rectangle('Position', [-sol_l_mm/2, 0, sol_l_mm, sol_r_mm], 'EdgeColor', 'w', 'LineWidth', 1.5, 'LineStyle', '--');
xlabel('dz [mm]'); ylabel('rho [mm]');
title('Reference field (arrows=ref)');

subplot(1,3,2);
imagesc(dz_hm*1e3, rho_hm*1e3, BZ_LUT_HM');
set(gca, 'YDir', 'normal'); colorbar;
hold on;
quiver(QDZ, QRHO, V_lut./mag_lut, U_lut./mag_lut, 0.4, 'Color', [1 0.2 0.2], 'LineWidth', 0.8);
rectangle('Position', [-sol_l_mm/2, 0, sol_l_mm, sol_r_mm], 'EdgeColor', 'w', 'LineWidth', 1.5, 'LineStyle', '--');
xlabel('dz [mm]'); ylabel('rho [mm]');
title('LUT field (arrows=LUT)');

subplot(1,3,3);
eps_hm = max(abs(BZ_REF_HM(:))) * 0.01;
ERR_HM = abs(BZ_LUT_HM - BZ_REF_HM) ./ (abs(BZ_REF_HM) + eps_hm) * 100;
imagesc(dz_hm*1e3, rho_hm*1e3, ERR_HM');
set(gca, 'YDir', 'normal'); colorbar;
caxis([0, min(10, max(ERR_HM(:)))]);
hold on;
% Both arrows overlaid: blue=ref, red=LUT
quiver(QDZ, QRHO, V_ref./mag_ref, U_ref./mag_ref, 0.4, 'Color', [0.3 0.5 1], 'LineWidth', 0.8);
quiver(QDZ, QRHO, V_lut./mag_lut, U_lut./mag_lut, 0.4, 'Color', [1 0.2 0.2], 'LineWidth', 0.8);
rectangle('Position', [-sol_l_mm/2, 0, sol_l_mm, sol_r_mm], 'EdgeColor', 'w', 'LineWidth', 1.5, 'LineStyle', '--');
xlabel('dz [mm]'); ylabel('rho [mm]');
title('bz error [%] (blue=ref, red=LUT)');
colormap(hot);
