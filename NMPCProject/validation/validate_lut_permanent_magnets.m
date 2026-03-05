%% validate_lut_permanent_magnets.m
% Step 2: Validate 3D permanent magnet LUT vs computeFieldBase with u=0
%
% Compares the pre-built 3D LUT (combined permanent magnets) against
% MATLAB computeFieldBase with solenoids off.
%
% Pass criteria: max relative error < 1% on bx, by, bz

% clear; clc;
addpath(genpath('..'));

%% Setup
load_params(MaglevModel.Accurate);

luts = buildLuts(params);

%% Generate random test points in the workspace
rng(42);
nTest = 500;

perm3d_xy = params.lut_opts.perm3d_xy;
perm3d_z  = params.lut_opts.perm3d_z;
xy_max = perm3d_xy * 0.9;
z_min  = perm3d_z(1) + 0.002;
z_max  = perm3d_z(2) - 0.002;

x_test = (2*rand(1, nTest) - 1) * xy_max;
y_test = (2*rand(1, nTest) - 1) * xy_max;
z_test = z_min + rand(1, nTest) * (z_max - z_min);

%% Evaluate: 3D LUT
bx_lut = zeros(1, nTest);
by_lut = zeros(1, nTest);
bz_lut = zeros(1, nTest);

for k = 1:nTest
    result = full(luts.perm_3d([x_test(k); y_test(k); z_test(k)]));
    bx_lut(k) = result(1);
    by_lut(k) = result(2);
    bz_lut(k) = result(3);
end

%% Evaluate: MATLAB reference (solenoids off)
u_zero = zeros(length(params.solenoids.r), 1);
[bx_ref, by_ref, bz_ref] = computeFieldBase( ...
    x_test, y_test, z_test, u_zero, params, MaglevModel.Accurate);

%% Compute errors (mixed metric — handles zero-crossings)
fields = {'bx', 'by', 'bz'};
lut_vals = {bx_lut, by_lut, bz_lut};
ref_vals = {bx_ref, by_ref, bz_ref};

fprintf('\n=== Step 2: Permanent Magnet 3D LUT vs MATLAB ===\n');
fprintf('Test points: %d\n', nTest);

all_pass = true;
for f = 1:3
    eps_f = max(abs(ref_vals{f})) * 0.01;
    mixed_err = abs(lut_vals{f} - ref_vals{f}) ./ (abs(ref_vals{f}) + eps_f);
    fprintf('%s - max mixed error: %.4f%%  mean: %.4f%%  (abs_floor=%.2e)\n', ...
        fields{f}, max(mixed_err)*100, mean(mixed_err)*100, eps_f);
    if max(mixed_err) >= 0.01
        all_pass = false;
    end
end

fprintf('Max absolute errors: bx=%.2e, by=%.2e, bz=%.2e\n', ...
    max(abs(bx_lut-bx_ref)), max(abs(by_lut-by_ref)), max(abs(bz_lut-bz_ref)));

% Show worst point
[~, idx_worst] = max(abs(bz_lut - bz_ref));
fprintf('Worst bz point: x=%.4f, y=%.4f, z=%.4f, lut=%.4e, ref=%.4e\n', ...
    x_test(idx_worst), y_test(idx_worst), z_test(idx_worst), ...
    bz_lut(idx_worst), bz_ref(idx_worst));

if all_pass
    fprintf('PASS\n');
else
    fprintf('FAIL\n');
end

%% Error heatmaps
plot_heatmap = true;
if plot_heatmap
    z_eq = z_min+1e-4;
    N_hm = 200;
    u_zero = zeros(length(params.solenoids.r), 1);

    % --- Slice 1: y=0 plane (x-z view) ---
    x_sl = linspace(-perm3d_xy*0.95, perm3d_xy*0.95, N_hm);
    z_sl = linspace(perm3d_z(1)+0.001, perm3d_z(2)-0.001, N_hm);
    [X_SL, Z_SL] = meshgrid(x_sl, z_sl);
    Y_SL = zeros(size(X_SL));

    [bx_r, by_r, bz_r] = computeFieldBase( ...
        X_SL(:)', Y_SL(:)', Z_SL(:)', u_zero, params, MaglevModel.Accurate);

    BZ_REF = reshape(bz_r, N_hm, N_hm);
    BX_REF = reshape(bx_r, N_hm, N_hm);
    BZ_LUT = zeros(N_hm);
    BX_LUT = zeros(N_hm);

    for k = 1:numel(X_SL)
        res = full(luts.perm_3d([X_SL(k); 0; Z_SL(k)]));
        BX_LUT(k) = res(1);
        BZ_LUT(k) = res(3);
    end

    eps_bz = max(abs(BZ_REF(:))) * 0.01;
    ERR_BZ = abs(BZ_LUT - BZ_REF) ./ (abs(BZ_REF) + eps_bz) * 100;

    figure('Name', 'Step 2: Perm Magnet LUT Error');

    subplot(2,2,1);
    imagesc(x_sl*1e3, z_sl*1e3, BZ_REF);
    set(gca, 'YDir', 'normal'); colorbar;
    hold on;
    drawMagnetsXZ(params, 'w');
    xlabel('x [mm]'); ylabel('z [mm]');
    title('bz reference (y=0 slice) [T]');

    subplot(2,2,2);
    imagesc(x_sl*1e3, z_sl*1e3, ERR_BZ);
    set(gca, 'YDir', 'normal'); colorbar;
    caxis([0, min(10, max(ERR_BZ(:)))]);
    hold on;
    drawMagnetsXZ(params, 'w');
    % Add normalized field arrows: reference (blue) and LUT (red)
    skip = max(1, round(N_hm/20));
    idx = 1:skip:N_hm;
    QX = X_SL(idx,idx)*1e3;  QZ = Z_SL(idx,idx)*1e3;
    U_ref = BX_REF(idx,idx);  V_ref = BZ_REF(idx,idx);
    mag_ref = sqrt(U_ref.^2 + V_ref.^2) + 1e-20;
    U_lut = BX_LUT(idx,idx);  V_lut = BZ_LUT(idx,idx);
    mag_lut = sqrt(U_lut.^2 + V_lut.^2) + 1e-20;
    quiver(QX, QZ, U_ref./mag_ref, V_ref./mag_ref, 0.4, 'Color', [0.3 0.5 1], 'LineWidth', 0.8);
    quiver(QX, QZ, U_lut./mag_lut, V_lut./mag_lut, 0.4, 'Color', [1 0.2 0.2], 'LineWidth', 0.8);
    xlabel('x [mm]'); ylabel('z [mm]');
    title('bz mixed error [%] (y=0 slice)');
    colormap(hot);

    % --- Slice 2: z=0.042 plane (x-y at equilibrium height) ---
    xy_sl = linspace(-perm3d_xy*0.95, perm3d_xy*0.95, N_hm);
    [X_EQ, Y_EQ] = meshgrid(xy_sl, xy_sl);
    Z_EQ = z_eq * ones(size(X_EQ));

    [bx_eq, by_eq, bz_eq] = computeFieldBase( ...
        X_EQ(:)', Y_EQ(:)', Z_EQ(:)', u_zero, params, MaglevModel.Accurate);

    BZ_REF_EQ = reshape(bz_eq, N_hm, N_hm);
    BZ_LUT_EQ = zeros(N_hm);

    for k = 1:numel(X_EQ)
        res = full(luts.perm_3d([X_EQ(k); Y_EQ(k); z_eq]));
        BZ_LUT_EQ(k) = res(3);
    end

    ERR_BZ_EQ = abs(BZ_LUT_EQ - BZ_REF_EQ) ./ (abs(BZ_REF_EQ) + eps_bz) * 100;

    subplot(2,2,3);
    imagesc(xy_sl*1e3, xy_sl*1e3, BZ_REF_EQ);
    set(gca, 'YDir', 'normal'); colorbar;
    hold on;
    drawMagnetsXY(params, 'w');
    xlabel('x [mm]'); ylabel('y [mm]');
    title(sprintf('bz reference (z=%.0fmm slice) [T]', z_eq*1e3));

    subplot(2,2,4);
    imagesc(xy_sl*1e3, xy_sl*1e3, ERR_BZ_EQ);
    set(gca, 'YDir', 'normal'); colorbar;
    caxis([0, min(5, max(ERR_BZ_EQ(:)))]);
    hold on;
    drawMagnetsXY(params, 'w');
    xlabel('x [mm]'); ylabel('y [mm]');
    title(sprintf('bz mixed error [%%] (z=%.0fmm slice)', z_eq*1e3));
end

%% Helper: draw permanent magnets as rectangles in x-z view (side cross-section)
function drawMagnetsXZ(params, col)
    for i = 1:length(params.permanent.x)
        xi = params.permanent.x(i) * 1e3;
        zi = params.permanent.z(i) * 1e3;
        ri = params.permanent.r(i) * 1e3;
        li = params.permanent.l(i) * 1e3;
        % Side view: rectangle from x-r to x+r, z-l/2 to z+l/2
        rectangle('Position', [xi-ri, zi-li/2, 2*ri, li], ...
                  'EdgeColor', col, 'LineWidth', 1.5, 'LineStyle', '-');
    end
    % Also draw solenoids (dashed)
    for i = 1:length(params.solenoids.x)
        xi = params.solenoids.x(i) * 1e3;
        zi = params.solenoids.z(i) * 1e3;
        ri = params.solenoids.r(i) * 1e3;
        li = params.solenoids.l(i) * 1e3;
        rectangle('Position', [xi-ri, zi-li/2, 2*ri, li], ...
                  'EdgeColor', col, 'LineWidth', 1, 'LineStyle', '--');
    end
end

%% Helper: draw permanent magnets as circles in x-y view (top-down)
function drawMagnetsXY(params, col)
    theta_c = linspace(0, 2*pi, 64);
    for i = 1:length(params.permanent.x)
        xi = params.permanent.x(i) * 1e3;
        yi = params.permanent.y(i) * 1e3;
        ri = params.permanent.r(i) * 1e3;
        plot(xi + ri*cos(theta_c), yi + ri*sin(theta_c), ...
             '-', 'Color', col, 'LineWidth', 1.5);
    end
    % Also draw solenoids (dashed)
    for i = 1:length(params.solenoids.x)
        xi = params.solenoids.x(i) * 1e3;
        yi = params.solenoids.y(i) * 1e3;
        ri = params.solenoids.r(i) * 1e3;
        plot(xi + ri*cos(theta_c), yi + ri*sin(theta_c), ...
             '--', 'Color', col, 'LineWidth', 1);
    end
end
