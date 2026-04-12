%% nAxialStudy — Effect of n_axial on model accuracy (MATLAB ode15s)
%
% Proves whether n_axial matters for the Accurate model by using the
% MATLAB dynamics (maglevSystemDynamics) which correctly handles
% n_axial > 1 via axial trapezoidal integration.
%
% Sweeps n_axial=[1,3,7,11,21] at n=[8,16,32], compared against
% ode15s reference at n=80, n_axial=21.
%
% Uses 500 random single-step samples, same methodology as magnetNStudy.
%
% Outputs:
%   analysis/naxial_study.mat       — results
%   Master_thesis_2025/figures/     — thesis figure

clear; clc; close all;

%% ================================================================
%% Path setup
%% ================================================================
acados_root  = getenv('ACADOS_INSTALL_DIR');
project_root = getenv('ACADOS_PROJECT_DIR');

assert(~isempty(acados_root),  'ACADOS_INSTALL_DIR not set. Source env.sh first.');
assert(~isempty(project_root), 'ACADOS_PROJECT_DIR not set. Source env.sh first.');

addpath(genpath(fullfile(project_root, 'model_matlab')));
addpath(genpath(fullfile(project_root, 'model_reduced_casadi')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

cd(project_root);

save_path = fullfile('analysis', 'naxial_study.mat');

if exist(save_path, 'file')
    fprintf('=== Loading existing results from %s ===\n', save_path);
    results = load(save_path);
else
    %% ================================================================
    %% Parameters
    %% ================================================================
    modelId = MaglevModel.Accurate;
    parameters_maggy_V4;

    nx = 10;
    nu = 4;

    % Equilibrium
    [zEq, ~, ~, ~] = computeSystemEquilibria(params, modelId);
    xEq_full = [0; 0; zEq(1); zeros(9,1)];
    xEq      = xEq_full([1:5, 7:11]);

    %% ================================================================
    %% Sampling (same seed as magnetNStudy for consistency)
    %% ================================================================
    N_samples = 2000;
    rng(42);

    dx_range = [
        2e-3; 2e-3; 2e-3;
        deg2rad(10); deg2rad(10);
        0.1; 0.1; 0.1;
        5; 5;
    ];
    u_max = 1.0;

    X_samples = xEq + dx_range .* (2*rand(nx, N_samples) - 1);
    U_samples = u_max * (2*rand(nu, N_samples) - 1);

    dt = 0.001;  % MPC-relevant timestep

    %% ================================================================
    %% ode15s reference (n=80, n_axial=21)
    %% ================================================================
    fprintf('=== Computing ode15s reference (n=80, n_axial=21) ===\n');

    params_ref = params;
    params_ref.magnet.n       = 80;
    params_ref.magnet.n_axial = 21;

    f_ode_ref = @(x, u) maglevSystemDynamics(x, u, params_ref, modelId);
    ode_opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-14);

    x_ref = zeros(nx, N_samples);
    tic;
    for i = 1:N_samples
        x10 = X_samples(:, i);
        x12 = [x10(1:5); 0; x10(6:10); 0];
        [~, x_out] = ode15s(@(t, x) f_ode_ref(x, U_samples(:, i)), [0, dt], x12, ode_opts);
        x12_end = x_out(end, :)';
        x_ref(:, i) = x12_end([1:5, 7:11]);
    end
    fprintf('  Done in %.1f s\n', toc);

    %% ================================================================
    %% Sweep n_axial at multiple n values
    %% ================================================================
    n_axial_list = [1, 3, 7, 11, 21];
    n_list       = [8, 10, 16, 32];
    n_nax  = numel(n_axial_list);
    n_nval = numel(n_list);

    fprintf('\n=== Sweeping n_axial at n = [%s] ===\n', num2str(n_list));

    for in = 1:n_nval
        mn = n_list(in);
        for j = 1:n_nax
            nax = n_axial_list(j);
            fprintf('[n=%d, n_axial=%d] ', mn, nax);

            params_test = params;
            params_test.magnet.n       = mn;
            params_test.magnet.n_axial = nax;

            f_ode_test = @(x, u) maglevSystemDynamics(x, u, params_test, modelId);

            x_test = zeros(nx, N_samples);
            tic;
            for i = 1:N_samples
                x10 = X_samples(:, i);
                x12 = [x10(1:5); 0; x10(6:10); 0];
                [~, x_out] = ode15s(@(t, x) f_ode_test(x, U_samples(:, i)), ...
                    [0, dt], x12, ode_opts);
                x12_end = x_out(end, :)';
                x_test(:, i) = x12_end([1:5, 7:11]);
            end
            t_elapsed = toc;

            err = x_test - x_ref;
            pos_err = vecnorm(err(1:3, :), 2, 1) * 1e3;
            ang_err = vecnorm(err(4:5, :), 2, 1) * (180/pi);
            vel_err = vecnorm(err(6:end, :), 2, 1);

            results.data(in, j).n         = mn;
            results.data(in, j).n_axial   = nax;
            results.data(in, j).pos_rmse  = rms(pos_err);
            results.data(in, j).pos_max   = max(pos_err);
            results.data(in, j).ang_rmse  = rms(ang_err);
            results.data(in, j).vel_rmse  = rms(vel_err);
            results.data(in, j).ode_time  = t_elapsed;
            results.data(in, j).pos_err_all = pos_err;

            fprintf('pos_rmse=%.4e mm  (%.1f s)\n', rms(pos_err), t_elapsed);
        end
    end

    results.n_list       = n_list;
    results.n_axial_list = n_axial_list;
    results.dt           = dt;
    results.N_samples    = N_samples;

    cd(project_root);
    save(save_path, '-struct', 'results');
    fprintf('\nResults saved to: %s\n', save_path);
end  % if exist(save_path)

%% ================================================================
%% Extract data
%% ================================================================
n_list       = results.n_list;
n_axial_list = results.n_axial_list;
n_nval       = numel(n_list);
n_nax        = numel(n_axial_list);
dt           = results.dt;
N_samples    = results.N_samples;

pos_rmse_grid = zeros(n_nval, n_nax);
for in = 1:n_nval
    for j = 1:n_nax
        pos_rmse_grid(in, j) = results.data(in, j).pos_rmse;
    end
end

%% ================================================================
%% Print summary table
%% ================================================================
fprintf('\n=== n_axial study via MATLAB ode15s (dt=%.0f us) ===\n', dt*1e6);
for in = 1:n_nval
    mn = n_list(in);
    fprintf('\n  n = %d:\n', mn);
    fprintf('  %-8s  %12s  %12s  %12s  %10s\n', ...
        'n_axial', 'Pos RMSE', 'Pos Max', 'Ang RMSE', 'ODE time');
    fprintf('  %-8s  %12s  %12s  %12s  %10s\n', ...
        '', '(mm)', '(mm)', '(deg)', '(s)');
    fprintf('  %s\n', repmat('-', 1, 60));
    for j = 1:n_nax
        d = results.data(in, j);
        fprintf('  %-8d  %12.4e  %12.4e  %12.4e  %10.1f\n', ...
            n_axial_list(j), d.pos_rmse, d.pos_max, d.ang_rmse, d.ode_time);
    end
end

%% ================================================================
%% Generate figure
%% ================================================================
fprintf('\n=== Generating figure ===\n');

fig_dir = fullfile(project_root, 'Master_thesis_2025', 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

FONT_NAME  = 'Times New Roman';
FONT_AX    = 9;
FONT_LAB   = 10;
FONT_LEG   = 8;
FONT_TITLE = 10;

fig1 = mkfig(17, 7);
hold on; grid on; box on;

colors_n = lines(n_nval);
markers_n = {'o', 's', 'd', '^', 'v'};

for in = 1:n_nval
    plot(n_axial_list, pos_rmse_grid(in, :), ['-' markers_n{in}], ...
        'Color', colors_n(in,:), 'LineWidth', 1.3, 'MarkerSize', 7, ...
        'MarkerFaceColor', colors_n(in,:), ...
        'DisplayName', sprintf('n = %d', n_list(in)));
end

set(gca, 'YScale', 'log');
xlabel('n\_axial', 'FontSize', FONT_LAB);
ylabel('Position RMSE vs. ref (mm)', 'FontSize', FONT_LAB);
title(sprintf('Effect of n\\_axial on model accuracy (MATLAB ode15s, dt = %.0f \\mus)', dt*1e6), ...
    'FontSize', FONT_TITLE, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', FONT_LEG);
style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);

savefig_thesis(fig1, 'magn_naxial_effect', fig_dir);

fprintf('\n=== n_axial study complete ===\n');

%% ================================================================
%% Local helper functions
%% ================================================================

function fig = mkfig(w_cm, h_cm)
    fig = figure('Units', 'centimeters', ...
                 'Position',      [2  2  w_cm h_cm], ...
                 'PaperUnits',    'centimeters', ...
                 'PaperSize',     [w_cm h_cm], ...
                 'PaperPosition', [0   0  w_cm h_cm], ...
                 'Color', 'w');
end

function style_ax(ax, fn, fs_ax, fs_lab)
    set(ax, 'FontName',    fn, ...
            'FontSize',    fs_ax, ...
            'TickDir',     'out', ...
            'TickLength',  [0.015 0.015], ...
            'LineWidth',   0.6, ...
            'Box',         'on');
    set(ax.XLabel, 'FontSize', fs_lab, 'FontName', fn);
    set(ax.YLabel, 'FontSize', fs_lab, 'FontName', fn);
end

function savefig_thesis(fig, name, out_dir)
    pdf_path = fullfile(out_dir, [name '.pdf']);
    exportgraphics(fig, pdf_path, 'ContentType', 'vector');
    fprintf('  Saved: %s\n', pdf_path);
    png_path = fullfile(out_dir, [name '.png']);
    exportgraphics(fig, png_path, 'Resolution', 300);
end
