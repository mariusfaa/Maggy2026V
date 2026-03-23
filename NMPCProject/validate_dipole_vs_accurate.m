%% validate_dipole_vs_accurate — Perturbation comparison around equilibrium
%
% Runs open-loop ode15s simulations from perturbations around equilibrium
% for both Accurate and Dipole models, then compares trajectories.
%
% Perturbations: ±5 mm in x,y,z  and  ±10 deg in roll,pitch
% Input: u = 0 (free response around equilibrium)

clear; clc; close all;

addpath(genpath('model_matlab'));
addpath(genpath('system_parameters'));
addpath(genpath('utilities'));

output_dir = 'figures/dipole_validation';
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

%% --- Setup models ---
params_acc = load_params(MaglevModel.Accurate);
params_dip = load_params(MaglevModel.Dipole);

[zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);
[zEq_dip, ~, ~, ~] = computeSystemEquilibria(params_dip, MaglevModel.Dipole);

xEq_acc = [0; 0; zEq_acc(1); zeros(9,1)];
xEq_dip = [0; 0; zEq_dip(1); zeros(9,1)];

fprintf('Accurate eq: z = %.4f mm\n', zEq_acc(1)*1e3);
fprintf('Dipole eq:   z = %.4f mm\n', zEq_dip(1)*1e3);

f_acc = @(x, u) maglevSystemDynamics(x, u, params_acc, MaglevModel.Accurate);
f_dip = @(x, u) maglevSystemDynamics(x, u, params_dip, MaglevModel.Dipole);

%% --- Define perturbations ---
dx_pos = 0.005;   % 5 mm
da_ang = 10*pi/180; % 10 deg

perturbations = struct([]);
idx = 0;

% Position perturbations
dirs = {'x', 'y', 'z'};
state_idx = [1, 2, 3];
for d = 1:3
    for sgn = [-1, +1]
        idx = idx + 1;
        delta = zeros(12,1);
        delta(state_idx(d)) = sgn * dx_pos;
        perturbations(idx).delta = delta;
        perturbations(idx).name = sprintf('%s %+.0f mm', dirs{d}, sgn*dx_pos*1e3);
        perturbations(idx).group = 'position';
        perturbations(idx).dir = dirs{d};
        perturbations(idx).sign = sgn;
    end
end

% Angle perturbations
dirs_ang = {'roll', 'pitch'};
state_idx_ang = [4, 5];
for d = 1:2
    for sgn = [-1, +1]
        idx = idx + 1;
        delta = zeros(12,1);
        delta(state_idx_ang(d)) = sgn * da_ang;
        perturbations(idx).delta = delta;
        perturbations(idx).name = sprintf('%s %+.0f deg', dirs_ang{d}, sgn*10);
        perturbations(idx).group = 'orientation';
        perturbations(idx).dir = dirs_ang{d};
        perturbations(idx).sign = sgn;
    end
end

nPert = length(perturbations);
fprintf('\n%d perturbation cases\n', nPert);

%% --- Simulate all perturbations ---
T_sim = 0.10;  % 100 ms — enough to see initial dynamics before divergence
ode_opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
u0 = zeros(4, 1);  % free response

results = struct([]);

for ip = 1:nPert
    fprintf('  [%2d/%2d] %s ... ', ip, nPert, perturbations(ip).name);

    x0_acc = xEq_acc + perturbations(ip).delta;
    x0_dip = xEq_dip + perturbations(ip).delta;

    % Accurate
    try
        [t_a, x_a] = ode15s(@(t,x) f_acc(x, u0), [0, T_sim], x0_acc, ode_opts);
        results(ip).t_acc = t_a';
        results(ip).x_acc = x_a';
        results(ip).ok_acc = true;
    catch
        results(ip).ok_acc = false;
        fprintf('ACC FAIL  ');
    end

    % Dipole
    try
        [t_d, x_d] = ode15s(@(t,x) f_dip(x, u0), [0, T_sim], x0_dip, ode_opts);
        results(ip).t_dip = t_d';
        results(ip).x_dip = x_d';
        results(ip).ok_dip = true;
    catch
        results(ip).ok_dip = false;
        fprintf('DIP FAIL  ');
    end

    results(ip).name  = perturbations(ip).name;
    results(ip).group = perturbations(ip).group;
    results(ip).dir   = perturbations(ip).dir;
    results(ip).sign  = perturbations(ip).sign;
    results(ip).delta = perturbations(ip).delta;

    if results(ip).ok_acc && results(ip).ok_dip
        % Compute error on common time grid
        T_end = min(results(ip).t_acc(end), results(ip).t_dip(end));
        t_c = linspace(0, T_end, 2000);
        x_a_i = interp1(results(ip).t_acc, results(ip).x_acc', t_c, 'pchip')';
        x_d_i = interp1(results(ip).t_dip, results(ip).x_dip', t_c, 'pchip')';

        % Subtract respective equilibria so we compare deviations
        err = (x_d_i - xEq_dip) - (x_a_i - xEq_acc);
        results(ip).t_common = t_c;
        results(ip).err = err;
        results(ip).T_end = T_end;

        max_pos_err = max(abs(err(1:3,:)), [], 'all') * 1e3;
        max_ang_err = max(abs(err(4:5,:)), [], 'all') * 180/pi;
        fprintf('pos_err=%.3f mm  ang_err=%.3f deg  (T=%.0f ms)\n', ...
            max_pos_err, max_ang_err, T_end*1e3);
    else
        fprintf('\n');
    end
end

%% === PLOTTING ===
pos_scale = 1e3;
ang_scale = 180/pi;
state_names = {'x','y','z','roll','pitch','yaw','v_x','v_y','v_z','\omega_x','\omega_y','\omega_z'};
units = {'mm','mm','mm','deg','deg','deg','m/s','m/s','m/s','rad/s','rad/s','rad/s'};
scales = [pos_scale*ones(1,3), ang_scale*ones(1,3), ones(1,6)];

col_acc = [0.0 0.45 0.74];   % blue
col_dip = [0.85 0.33 0.10];  % red

%% --- Figure set 1: Per-perturbation trajectory overlay ---
fprintf('\n=== Generating per-perturbation trajectory plots ===\n');

for ip = 1:nPert
    if ~results(ip).ok_acc || ~results(ip).ok_dip, continue; end

    fig = figure('Visible', 'off', 'Position', [50 50 1400 900]);

    % Determine which states to plot based on group
    if strcmp(results(ip).group, 'position')
        plot_states = [1 2 3 7 8 9];  % pos + lin vel
        fig_rows = 2; fig_cols = 3;
    else
        plot_states = [4 5 10 11];    % ang + ang vel
        fig_rows = 2; fig_cols = 2;
    end

    for sp = 1:length(plot_states)
        si = plot_states(sp);
        subplot(fig_rows, fig_cols, sp); hold on; grid on; box on;

        % Plot deviation from equilibrium
        x_acc_dev = (results(ip).x_acc(si,:) - xEq_acc(si)) * scales(si);
        x_dip_dev = (results(ip).x_dip(si,:) - xEq_dip(si)) * scales(si);

        plot(results(ip).t_acc*1e3, x_acc_dev, '-', ...
            'Color', col_acc, 'LineWidth', 1.5, 'DisplayName', 'Accurate');
        plot(results(ip).t_dip*1e3, x_dip_dev, '--', ...
            'Color', col_dip, 'LineWidth', 1.5, 'DisplayName', 'Dipole');

        xlabel('Time (ms)');
        ylabel(sprintf('\\Delta%s (%s)', state_names{si}, units{si}));
        title(sprintf('\\Delta%s', state_names{si}));
        if sp == 1, legend('Location', 'best'); end
    end

    sgtitle(sprintf('Perturbation: %s', results(ip).name), ...
        'FontSize', 14, 'FontWeight', 'bold');

    fname = sprintf('traj_%s_%s%s.png', results(ip).group, results(ip).dir, ...
        ternary(results(ip).sign > 0, '_pos', '_neg'));
    exportgraphics(fig, fullfile(output_dir, fname), 'Resolution', 200);
    close(fig);
    fprintf('  Saved: %s\n', fname);
end

%% --- Figure 2: Error summary — position perturbations ---
fprintf('\n=== Generating error summary plots ===\n');

fig = figure('Visible', 'off', 'Position', [50 50 1400 500]);

% Position perturbation errors
pos_cases = find(strcmp({results.group}, 'position') & [results.ok_acc] & [results.ok_dip]);
colors_pert = lines(length(pos_cases));

subplot(1,2,1); hold on; grid on; box on;
for ii = 1:length(pos_cases)
    ip = pos_cases(ii);
    err_pos = max(abs(results(ip).err(1:3,:)), [], 1) * pos_scale;
    plot(results(ip).t_common*1e3, err_pos, '-', ...
        'Color', colors_pert(ii,:), 'LineWidth', 1.5, ...
        'DisplayName', results(ip).name);
end
xlabel('Time (ms)'); ylabel('mm');
title('Position error |Dipole - Accurate|');
legend('Location', 'northwest', 'FontSize', 8);

subplot(1,2,2); hold on; grid on; box on;
for ii = 1:length(pos_cases)
    ip = pos_cases(ii);
    err_ang = max(abs(results(ip).err(4:5,:)), [], 1) * ang_scale;
    plot(results(ip).t_common*1e3, err_ang, '-', ...
        'Color', colors_pert(ii,:), 'LineWidth', 1.5, ...
        'DisplayName', results(ip).name);
end
xlabel('Time (ms)'); ylabel('deg');
title('Orientation error |Dipole - Accurate|');
legend('Location', 'northwest', 'FontSize', 8);

sgtitle('Modeling Error: Position Perturbations (±5 mm)', ...
    'FontSize', 14, 'FontWeight', 'bold');

exportgraphics(fig, fullfile(output_dir, 'error_summary_position.png'), 'Resolution', 200);
close(fig);
fprintf('  Saved: error_summary_position.png\n');

%% --- Figure 3: Error summary — orientation perturbations ---
fig = figure('Visible', 'off', 'Position', [50 50 1400 500]);

ang_cases = find(strcmp({results.group}, 'orientation') & [results.ok_acc] & [results.ok_dip]);
colors_ang = lines(length(ang_cases));

subplot(1,2,1); hold on; grid on; box on;
for ii = 1:length(ang_cases)
    ip = ang_cases(ii);
    err_pos = max(abs(results(ip).err(1:3,:)), [], 1) * pos_scale;
    plot(results(ip).t_common*1e3, err_pos, '-', ...
        'Color', colors_ang(ii,:), 'LineWidth', 1.5, ...
        'DisplayName', results(ip).name);
end
xlabel('Time (ms)'); ylabel('mm');
title('Position error |Dipole - Accurate|');
legend('Location', 'northwest', 'FontSize', 8);

subplot(1,2,2); hold on; grid on; box on;
for ii = 1:length(ang_cases)
    ip = ang_cases(ii);
    err_ang = max(abs(results(ip).err(4:5,:)), [], 1) * ang_scale;
    plot(results(ip).t_common*1e3, err_ang, '-', ...
        'Color', colors_ang(ii,:), 'LineWidth', 1.5, ...
        'DisplayName', results(ip).name);
end
xlabel('Time (ms)'); ylabel('deg');
title('Orientation error |Dipole - Accurate|');
legend('Location', 'northwest', 'FontSize', 8);

sgtitle('Modeling Error: Orientation Perturbations (±10°)', ...
    'FontSize', 14, 'FontWeight', 'bold');

exportgraphics(fig, fullfile(output_dir, 'error_summary_orientation.png'), 'Resolution', 200);
close(fig);
fprintf('  Saved: error_summary_orientation.png\n');

%% --- Figure 4: Bar chart of max errors at key time points ---
fig = figure('Visible', 'off', 'Position', [50 50 1200 600]);

t_checks = [0.010, 0.030, 0.050, 0.080];  % ms checkpoints
n_checks = length(t_checks);
n_cases = nPert;

pos_err_bar = zeros(n_cases, n_checks);
ang_err_bar = zeros(n_cases, n_checks);
case_names = cell(1, n_cases);

for ip = 1:n_cases
    case_names{ip} = results(ip).name;
    if ~results(ip).ok_acc || ~results(ip).ok_dip, continue; end

    for ic = 1:n_checks
        tc = t_checks(ic);
        if tc > results(ip).T_end, continue; end
        [~, ti] = min(abs(results(ip).t_common - tc));
        pos_err_bar(ip, ic) = max(abs(results(ip).err(1:3, ti))) * pos_scale;
        ang_err_bar(ip, ic) = max(abs(results(ip).err(4:5, ti))) * ang_scale;
    end
end

subplot(2,1,1);
b = bar(pos_err_bar);
set(gca, 'XTickLabel', case_names, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel('mm');
title('Max position error at checkpoints');
legend_labels = arrayfun(@(t) sprintf('t=%.0f ms', t*1e3), t_checks, 'UniformOutput', false);
legend(b, legend_labels, 'Location', 'northwest');
grid on; box on;

subplot(2,1,2);
b = bar(ang_err_bar);
set(gca, 'XTickLabel', case_names, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel('deg');
title('Max orientation error at checkpoints');
legend(b, legend_labels, 'Location', 'northwest');
grid on; box on;

sgtitle('Dipole vs Accurate: Error at Key Timepoints', ...
    'FontSize', 14, 'FontWeight', 'bold');

exportgraphics(fig, fullfile(output_dir, 'error_bar_chart.png'), 'Resolution', 200);
close(fig);
fprintf('  Saved: error_bar_chart.png\n');

%% --- Summary table ---
fprintf('\n\n====== SUMMARY TABLE ======\n');
fprintf('%-20s  %10s  %10s  %10s\n', 'Perturbation', 'T_end (ms)', 'pos err mm', 'ang err deg');
fprintf('%s\n', repmat('-', 1, 60));
for ip = 1:nPert
    if ~results(ip).ok_acc || ~results(ip).ok_dip
        fprintf('%-20s  %10s  %10s  %10s\n', results(ip).name, 'FAIL', '-', '-');
        continue;
    end
    max_pos = max(abs(results(ip).err(1:3,:)), [], 'all') * pos_scale;
    max_ang = max(abs(results(ip).err(4:5,:)), [], 'all') * ang_scale;
    fprintf('%-20s  %10.1f  %10.4f  %10.4f\n', ...
        results(ip).name, results(ip).T_end*1e3, max_pos, max_ang);
end

fprintf('\nAll plots saved to: %s/\n', output_dir);

%% --- Helper ---
function v = ternary(cond, a, b)
    if cond, v = a; else, v = b; end
end
