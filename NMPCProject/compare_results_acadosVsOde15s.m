%% --- COMPARE TWO NMPC SIMULATION RESULTS ---
% Compares simulation outputs from two .mat files (e.g. ode15s vs acados)
% Usage: set the two filenames and labels below, then run.

clear all; clc;

%% --- INPUT: set filenames and display labels ---
file_A   = 'nmpc_results_ode15ssim.mat';
file_B   = 'nmpc_results_acadossim.mat';
label_A  = 'ode15s';
label_B  = 'acados';

%% --- LOAD ---
A = load(file_A);
B = load(file_B);

% Support both save styles: struct fields saved flat, or as a struct
if isfield(A, 'sim_data'), A = A.sim_data; end
if isfield(B, 'sim_data'), B = B.sim_data; end

%% --- ALIGN TIME AXES ---
% Trim to the shorter of the two simulations in case one diverged early
n_steps = min(size(A.x, 2), size(B.x, 2));
tA = A.t(1:n_steps);
tB = B.t(1:n_steps);

xA = A.x(:, 1:n_steps);
xB = B.x(:, 1:n_steps);
uA = A.u(:, 1:min(n_steps, size(A.u,2)));
uB = B.u(:, 1:min(n_steps, size(B.u,2)));

if any(abs(tA - tB) > 1e-10)
    warning('Time vectors differ between files — using time vector from %s', label_A);
end
t   = tA;
t_u = t(1:size(uA,2));

%% --- UNIT CONVERSIONS ---
pos_scale = 1e2;    % m  -> cm
ang_scale = 180/pi; % rad -> deg

state_labels = {'x', 'y', 'z', '\alpha (roll)', '\beta (pitch)', '\gamma (yaw)', ...
                '\dot{x}', '\dot{y}', '\dot{z}', '\omega_x', '\omega_y', '\omega_z'};
state_units  = {'cm','cm','cm','deg','deg','deg', ...
                'cm/s','cm/s','cm/s','deg/s','deg/s','deg/s'};
state_scales = [pos_scale*ones(1,3), ang_scale*ones(1,3), ...
                pos_scale*ones(1,3), ang_scale*ones(1,3)];

%% --- COMPUTE DISCREPANCY ---
diff_x = xA - xB;
diff_u = uA - uB;

rmse_x = sqrt(mean(diff_x.^2, 2));
rmse_u = sqrt(mean(diff_u.^2, 2));
max_x  = max(abs(diff_x), [], 2);
max_u  = max(abs(diff_u), [], 2);

%% --- PRINT SUMMARY TABLE ---
fprintf('\n======================================================\n');
fprintf('  Discrepancy summary: %s  vs  %s\n', label_A, label_B);
fprintf('======================================================\n');
fprintf('%-24s %12s %12s\n', 'State', 'RMSE', 'Max |diff|');
fprintf('------------------------------------------------------\n');
for i = 1:12
    sc = state_scales(i);
    fprintf('%-24s %12.4e %12.4e  [%s]\n', ...
        state_labels{i}, rmse_x(i)*sc, max_x(i)*sc, state_units{i});
end
fprintf('\n%-24s %12s %12s\n', 'Input', 'RMSE', 'Max |diff|');
fprintf('------------------------------------------------------\n');
for i = 1:size(uA,1)
    fprintf('u_%d                      %12.4e %12.4e  [A]\n', i, rmse_u(i), max_u(i));
end
fprintf('======================================================\n\n');

%% --- FIGURE 1: Position & Orientation trajectories + difference ---
figure(1); clf;
set(gcf, 'Name', 'States: trajectory comparison', 'Position', [50 50 1100 800]);

colors = [0.2 0.5 0.9;   % A color
          0.9 0.3 0.2];  % B color

for i = 1:6
    sc = state_scales(i);

    % Top row: trajectories
    subplot(3, 6, i);
    hold on; grid on; box on;
    plot(t, xA(i,:)*sc, 'Color', colors(1,:), 'LineWidth', 1.6, 'DisplayName', label_A);
    plot(t, xB(i,:)*sc, '--', 'Color', colors(2,:), 'LineWidth', 1.6, 'DisplayName', label_B);
    if isfield(A, 'xEq')
        yline(A.xEq(i)*sc, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    title(state_labels{i}, 'Interpreter', 'tex');
    ylabel(state_units{i});
    if i == 1, legend('Location','best'); end
    xlim([t(1) t(end)]);

    % Middle row: absolute difference
    subplot(3, 6, i+6);
    hold on; grid on; box on;
    plot(t, abs(diff_x(i,:))*sc, 'Color', [0.4 0.7 0.3], 'LineWidth', 1.4);
    ylabel(state_units{i});
    title(sprintf('|%s - %s|', label_A, label_B), 'Interpreter', 'none');
    xlim([t(1) t(end)]);
end

% Third row: RMSE bar chart for all 12 states
subplot(3,1,3);
hold on; grid on; box on;
bar_vals = rmse_x .* state_scales(:);
bar(bar_vals, 'FaceColor', [0.4 0.6 0.85]);
xticks(1:12);
xticklabels(state_labels);
ylabel('RMSE (scaled units)');
title('RMSE per state');
set(gca, 'TickLabelInterpreter', 'tex');

sgtitle(sprintf('State comparison: %s vs %s', label_A, label_B), ...
    'FontSize', 13, 'FontWeight', 'bold');

%% --- FIGURE 2: Inputs ---
figure(2); clf;
set(gcf, 'Name', 'Inputs: trajectory comparison', 'Position', [1170 50 700 650]);

for i = 1:size(uA,1)
    subplot(size(uA,1)+1, 1, i);
    hold on; grid on; box on;
    stairs(t_u, uA(i,:), 'Color', colors(1,:), 'LineWidth', 1.6, 'DisplayName', label_A);
    stairs(t_u, uB(i,:), '--', 'Color', colors(2,:), 'LineWidth', 1.6, 'DisplayName', label_B);
    ylabel('A');
    title(sprintf('Solenoid %d', i));
    if i == 1, legend('Location','best'); end
    xlim([t_u(1) t_u(end)]);
end

% Bottom: max absolute input difference over time
subplot(size(uA,1)+1, 1, size(uA,1)+1);
hold on; grid on; box on;
plot(t_u, max(abs(diff_u), [], 1), 'Color', [0.4 0.7 0.3], 'LineWidth', 1.6);
ylabel('A');
title('max_i |u_i^A - u_i^B| over solenoids');
xlim([t_u(1) t_u(end)]);
xlabel('Time (s)');

sgtitle(sprintf('Input comparison: %s vs %s', label_A, label_B), ...
    'FontSize', 13, 'FontWeight', 'bold');