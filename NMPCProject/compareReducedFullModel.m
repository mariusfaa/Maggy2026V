%% NMPC_Controller_Comparison.m
% =========================================================================
% Comparison of Full-Order (12-state) vs. Reduced-Order (10-state) NMPC
% Extended with Data Logging and Worst-Case Analysis for Thesis
% =========================================================================

clearvars; clc; close all;

%% 1. Run Simulations
fprintf('--- Running Full-Order NMPC Simulation ---\n');
% run('workingSimulator.m');
full = load('nmpc_results.mat');

fprintf('\n--- Running Reduced-Order NMPC Simulation ---\n');
% run('workingSimulatorReducedOrder.m');
red = load('nmpc_results_reduced.mat');

%% 2. Data Alignment & Length Check
% Find the shortest simulation length to ensure the arrays match
N_steps = min(size(full.x, 2), size(red.x, 2));
t_common = full.t(1:N_steps);

% Shared state indices for comparison (x, y, z, alpha, beta, vx, vy, vz, wx, wy)
idx_full = [1, 2, 3, 4, 5, 7, 8, 9, 10, 11];

%% 3. Calculate Metrics
% --- Computational Metrics ---
mean_t_full = mean(full.solve_time) * 1000; % ms
mean_t_red  = mean(red.solve_time) * 1000;  % ms
time_reduction = (1 - mean_t_red/mean_t_full) * 100;

% --- Tracking Error (RMSE) ---
err_pos_full = full.x(1:3, 1:N_steps) - full.xEq(1:3);
err_pos_red  = red.x(1:3, 1:N_steps)  - red.xEq(1:3);
rmse_pos_full = sqrt(mean(sum(err_pos_full.^2, 1))) * 1e3;
rmse_pos_red  = sqrt(mean(sum(err_pos_red.^2, 1))) * 1e3;

% --- Worst-Case (Max) Tracking Error ---
max_err_pos_full = max(sqrt(sum(err_pos_full.^2, 1))) * 1e3;
max_err_pos_red  = max(sqrt(sum(err_pos_red.^2, 1))) * 1e3;

% Orientation RMSE (deg)
err_att_full = rad2deg(full.x(4:5, 1:N_steps) - full.xEq(4:5));
err_att_red  = rad2deg(red.x(4:5, 1:N_steps)  - red.xEq(4:5));
rmse_att_full = sqrt(mean(sum(err_att_full.^2, 1)));
rmse_att_red  = sqrt(mean(sum(err_att_red.^2, 1)));

% --- Control Effort ---
effort_full = sum(sum(full.u(:, 1:N_steps-1).^2)) * full.dt;
effort_red  = sum(sum(red.u(:, 1:N_steps-1).^2)) * red.dt;

%% 4. Generate Thesis Table
div_line = repmat('=', 1, 65);
sub_line = repmat('-', 1, 65);

fprintf('\n%s\n', div_line);
fprintf('%-25s | %-18s | %-18s\n', 'Metric', 'Full (12-state)', 'Reduced (10-state)');
fprintf('%s\n', sub_line);
fprintf('%-25s | %-18.2f | %-18.2f\n', 'Avg Solve Time (ms)', mean_t_full, mean_t_red);
fprintf('%-25s | %-18.2f | %-18.2f\n', 'Max Solve Time (ms)', max(full.solve_time)*1000, max(red.solve_time)*1000);
fprintf('%-25s | %-18.2f | %-18.2f\n', 'Avg SQP Iters', mean(full.sqp_iter), mean(red.sqp_iter));
fprintf('%-25s | %-18.4f | %-18.4f\n', 'Pos RMSE (mm)', rmse_pos_full, rmse_pos_red);
fprintf('%-25s | %-18.4f | %-18.4f\n', 'Max Pos Error (mm)', max_err_pos_full, max_err_pos_red);
fprintf('%-25s | %-18.4f | %-18.4f\n', 'Att RMSE (deg)', rmse_att_full, rmse_att_red);
fprintf('%-25s | %-18.4f | %-18.4f\n', 'Control Effort (A^2s)', effort_full, effort_red);
fprintf('%s\n', div_line);
fprintf('CPU Time Reduction: %.2f%%\n', time_reduction);

%% 5. Save Data Functionality
comparison_data = struct();
comparison_data.full = full;
comparison_data.red = red;
comparison_data.metrics.rmse_pos = [rmse_pos_full, rmse_pos_red];
comparison_data.metrics.max_err = [max_err_pos_full, max_err_pos_red];
comparison_data.metrics.time_ms = [mean_t_full, mean_t_red];
save('nmpc_comparison_results.mat', 'comparison_data');

fid = fopen('thesis_comparison_table.txt', 'w');
fprintf(fid, 'NMPC Comparison Results\n%s\n', datestr(now));
fprintf(fid, '%-25s | %-18s | %-18s\n', 'Metric', 'Full (12)', 'Reduced (10)');
fprintf(fid, '%-25s | %-18.4f | %-18.4f\n', 'Avg Solve Time (ms)', mean_t_full, mean_t_red);
fprintf(fid, '%-25s | %-18.4f | %-18.4f\n', 'Pos RMSE (mm)', rmse_pos_full, rmse_pos_red);
fprintf(fid, '%-25s | %-18.4f | %-18.4f\n', 'Control Effort (A^2s)', effort_full, effort_red);
fclose(fid);
fprintf('\nData saved to nmpc_comparison_results.mat and thesis_comparison_table.txt\n');

%% 6. Comparison Visualization
figure('Name', 'NMPC Comparison', 'Color', 'w', 'Position', [100 50 1100 900]);

% --- (1) Z-Position Tracking ---
subplot(3,2,1); hold on; grid on;
plot(t_common*1e3, full.x(3,1:N_steps)*1e3, 'b', 'LineWidth', 1.5);
plot(t_common*1e3, red.x(3,1:N_steps)*1e3, 'r--', 'LineWidth', 1.5);
yline(full.xEq(3)*1e3, 'k:');
title('Z-Axis Stabilization'); ylabel('z [mm]'); xlabel('Time [ms]');
legend('Full', 'Reduced');

% --- (2) Solver Latency ---
subplot(3,2,2); hold on; grid on;
histogram(full.solve_time*1000, 'FaceColor', 'b', 'FaceAlpha', 0.4);
histogram(red.solve_time*1000, 'FaceColor', 'r', 'FaceAlpha', 0.4);
title('Solve Time Distribution'); xlabel('Time [ms]'); ylabel('Samples');

% --- (3) Control Effort (Norm) ---
% Fixed index: u and t_common(1:end-1) match (N points)
subplot(3,2,3); hold on; grid on;
plot(t_common(1:N_steps-1)*1e3, vecnorm(full.u(:,1:N_steps-1)), 'b', 'LineWidth', 1.2);
plot(t_common(1:N_steps-1)*1e3, vecnorm(red.u(:,1:N_steps-1)), 'r--', 'LineWidth', 1.2);
title('Control Input Norm ||u||'); ylabel('Current [A]'); xlabel('Time [ms]');

% --- (4) State Deviation ---
subplot(3,2,4); hold on; grid on;
err_full_total = sum((full.x(idx_full, 1:N_steps) - full.xEq(idx_full)).^2, 1);
err_red_total  = sum((red.x(:, 1:N_steps) - red.xEq).^2, 1);
semilogy(t_common*1e3, err_full_total, 'b', 'LineWidth', 1.5);
semilogy(t_common*1e3, err_red_total, 'r--', 'LineWidth', 1.5);
title('Total State Error Squared (Log)'); ylabel('||x - x_{eq}||^2'); xlabel('Time [ms]');

% --- (5) SQP Iterations (Solver Convergence) ---
% Fixed index: sqp_iter has N points, matching t_common(1:end-1)
subplot(3,2,5); hold on; grid on;
stairs(t_common(1:N_steps-1)*1e3, full.sqp_iter(1:N_steps-1), 'b', 'LineWidth', 1.2);
stairs(t_common(1:N_steps-1)*1e3, red.sqp_iter(1:N_steps-1), 'r--', 'LineWidth', 1.2);
title('SQP Iterations per Step'); ylabel('Iterations'); xlabel('Time [ms]');

% --- (6) Radial Error (XY Plane Tracking) ---
subplot(3,2,6); hold on; grid on;
rad_err_full = sqrt(sum(err_pos_full(1:2,:).^2, 1)) * 1e3;
rad_err_red  = sqrt(sum(err_pos_red(1:2,:).^2, 1)) * 1e3;
plot(t_common*1e3, rad_err_full, 'b', 'LineWidth', 1.5);
plot(t_common*1e3, rad_err_red, 'r--', 'LineWidth', 1.5);
title('Radial Error (X-Y Plane)'); ylabel('Error [mm]'); xlabel('Time [ms]');