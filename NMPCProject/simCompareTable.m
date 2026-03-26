%% simCompareTable — Tabular comparison of simulation results
%
% Loads the same result files as aCompareSimulations and prints a
% formatted summary table to the console. Useful for quick comparison
% without opening figures.

% Uses out_folder from the calling workspace (simRunControllers),
% or falls back to 'results'.
if ~exist('out_folder', 'var')
    out_folder = 'results';
end

files = [
    "res_N30_dt1000us_ERK4s1_nmpc.mat"
    "res_N30_dt1000us_DISC_solmpc.mat"
    "res_N30_dt1000us_DISC_lmpc.mat"
];

%% ==========================================================
%% Load
%% ==========================================================

nFiles = numel(files);
data  = cell(1, nFiles);
names = cell(1, nFiles);

for f = 1:nFiles
    fpath = fullfile(out_folder, files(f));
    if ~isfile(fpath), error('File not found: %s', fpath); end
    data{f} = load(fpath);

    if isfield(data{f}, 'controller')
        names{f} = upper(data{f}.controller);
    else
        [~, n, ~] = fileparts(files(f));
        names{f} = strrep(n, '_', ' ');
    end
    if isfield(data{f}, 'diverged') && data{f}.diverged
        names{f} = [names{f} '*'];
    end
end

%% ==========================================================
%% Compute metrics
%% ==========================================================

% Common time window
T_end = min(cellfun(@(d) d.t(end), data));

fprintf('\n');
fprintf('============================================================\n');
fprintf('  SIMULATION COMPARISON  (T_end = %.4f s)\n', T_end);
fprintf('============================================================\n');

% --- General info ---
fprintf('\n--- Configuration ---\n');
hdr = sprintf('%-14s', 'Metric');
for f = 1:nFiles, hdr = [hdr sprintf('  %14s', names{f})]; end
fprintf('%s\n', hdr);
fprintf('%s\n', repmat('-', 1, 14 + nFiles*16));

row_str = @(label, vals, fmt) fprintf(['%-14s' repmat(['  ' fmt], 1, nFiles) '\n'], label, vals{:});

row_str('N_horizon', ...
    arrayfun(@(f) data{f}.N_horizon, 1:nFiles, 'UniformOutput', false), '%14d');
row_str('dt_mpc (us)', ...
    arrayfun(@(f) data{f}.dt_mpc*1e6, 1:nFiles, 'UniformOutput', false), '%14.0f');
row_str('Diverged', ...
    arrayfun(@(f) {ternary(data{f}.diverged, 'YES', 'no')}, 1:nFiles), '%14s');
row_str('Sim duration', ...
    arrayfun(@(f) data{f}.t(end), 1:nFiles, 'UniformOutput', false), '%14.4f');
row_str('MPC steps', ...
    arrayfun(@(f) numel(data{f}.ocp_time_tot), 1:nFiles, 'UniformOutput', false), '%14d');

% --- Position tracking ---
fprintf('\n--- Position Tracking (vs equilibrium) ---\n');
fprintf('%s\n', hdr);
fprintf('%s\n', repmat('-', 1, 14 + nFiles*16));

for f = 1:nFiles
    mask = data{f}.t <= T_end + 1e-12;
    x_trim = data{f}.x(:, mask);
    xEq_f = data{f}.xEq;

    pos_err = x_trim(1:3,:) - xEq_f(1:3);
    pos_norm = vecnorm(pos_err, 2, 1);
    metrics(f).pos_rms  = rms(pos_norm) * 1e3;
    metrics(f).pos_max  = max(pos_norm) * 1e3;
    metrics(f).pos_final = norm(x_trim(1:3,end) - xEq_f(1:3)) * 1e3;

    % Per-axis max
    metrics(f).x_max = max(abs(pos_err(1,:))) * 1e3;
    metrics(f).y_max = max(abs(pos_err(2,:))) * 1e3;
    metrics(f).z_max = max(abs(pos_err(3,:))) * 1e3;

    % Orientation
    ang_err = x_trim(4:5,:) - xEq_f(4:5);
    metrics(f).ang_max = max(abs(ang_err), [], 'all') * 180/pi;
end

row_str('RMS |pos| (mm)', ...
    arrayfun(@(f) metrics(f).pos_rms, 1:nFiles, 'UniformOutput', false), '%14.4f');
row_str('Max |pos| (mm)', ...
    arrayfun(@(f) metrics(f).pos_max, 1:nFiles, 'UniformOutput', false), '%14.4f');
row_str('Final |pos| (mm)', ...
    arrayfun(@(f) metrics(f).pos_final, 1:nFiles, 'UniformOutput', false), '%14.4f');
row_str('Max |x| (mm)', ...
    arrayfun(@(f) metrics(f).x_max, 1:nFiles, 'UniformOutput', false), '%14.4f');
row_str('Max |y| (mm)', ...
    arrayfun(@(f) metrics(f).y_max, 1:nFiles, 'UniformOutput', false), '%14.4f');
row_str('Max |z| (mm)', ...
    arrayfun(@(f) metrics(f).z_max, 1:nFiles, 'UniformOutput', false), '%14.4f');
row_str('Max angle (deg)', ...
    arrayfun(@(f) metrics(f).ang_max, 1:nFiles, 'UniformOutput', false), '%14.4f');

% --- Cost ---
fprintf('\n--- Cost ---\n');
fprintf('%s\n', hdr);
fprintf('%s\n', repmat('-', 1, 14 + nFiles*16));

for f = 1:nFiles
    n_steps = numel(data{f}.cost);
    metrics(f).cost_cum   = sum(data{f}.cost);
    metrics(f).cost_mean  = mean(data{f}.cost);
    metrics(f).cost_final = data{f}.cost(end);
end

row_str('Cumulative', ...
    arrayfun(@(f) metrics(f).cost_cum, 1:nFiles, 'UniformOutput', false), '%14.4g');
row_str('Mean/step', ...
    arrayfun(@(f) metrics(f).cost_mean, 1:nFiles, 'UniformOutput', false), '%14.4g');
row_str('Final step', ...
    arrayfun(@(f) metrics(f).cost_final, 1:nFiles, 'UniformOutput', false), '%14.4g');

% --- Timing ---
fprintf('\n--- Timing (microseconds) ---\n');
fprintf('%s\n', hdr);
fprintf('%s\n', repmat('-', 1, 14 + nFiles*16));

for f = 1:nFiles
    d = data{f};
    metrics(f).ocp_mean   = mean(d.ocp_time_tot) * 1e6;
    metrics(f).ocp_median = median(d.ocp_time_tot) * 1e6;
    metrics(f).ocp_max    = max(d.ocp_time_tot) * 1e6;
    metrics(f).qp_mean    = mean(d.ocp_time_qp) * 1e6;
    metrics(f).sim_mean   = mean(d.sim_time_tot) * 1e6;
    metrics(f).step_mean  = mean(d.step_time_tot) * 1e6;
    metrics(f).step_max   = max(d.step_time_tot) * 1e6;
    metrics(f).rt_factor  = d.dt_mpc / mean(d.step_time_tot);
end

row_str('OCP mean', ...
    arrayfun(@(f) metrics(f).ocp_mean, 1:nFiles, 'UniformOutput', false), '%14.1f');
row_str('OCP median', ...
    arrayfun(@(f) metrics(f).ocp_median, 1:nFiles, 'UniformOutput', false), '%14.1f');
row_str('OCP max', ...
    arrayfun(@(f) metrics(f).ocp_max, 1:nFiles, 'UniformOutput', false), '%14.1f');
row_str('QP mean', ...
    arrayfun(@(f) metrics(f).qp_mean, 1:nFiles, 'UniformOutput', false), '%14.1f');
row_str('Sim mean', ...
    arrayfun(@(f) metrics(f).sim_mean, 1:nFiles, 'UniformOutput', false), '%14.1f');
row_str('Step mean', ...
    arrayfun(@(f) metrics(f).step_mean, 1:nFiles, 'UniformOutput', false), '%14.1f');
row_str('Step max', ...
    arrayfun(@(f) metrics(f).step_max, 1:nFiles, 'UniformOutput', false), '%14.1f');
row_str('RT factor', ...
    arrayfun(@(f) metrics(f).rt_factor, 1:nFiles, 'UniformOutput', false), '%14.2f');

% --- Solver health ---
fprintf('\n--- Solver Health ---\n');
fprintf('%s\n', hdr);
fprintf('%s\n', repmat('-', 1, 14 + nFiles*16));

for f = 1:nFiles
    d = data{f};
    metrics(f).n_warnings = sum(d.ocp_status ~= 0 & d.ocp_status ~= 2);
    metrics(f).qp_iter_mean = mean(d.ocp_qp_iter);
    metrics(f).qp_iter_max  = max(d.ocp_qp_iter);
end

row_str('Solver warns', ...
    arrayfun(@(f) metrics(f).n_warnings, 1:nFiles, 'UniformOutput', false), '%14d');
row_str('QP iter mean', ...
    arrayfun(@(f) metrics(f).qp_iter_mean, 1:nFiles, 'UniformOutput', false), '%14.1f');
row_str('QP iter max', ...
    arrayfun(@(f) metrics(f).qp_iter_max, 1:nFiles, 'UniformOutput', false), '%14d');

fprintf('\n* = diverged\n');
fprintf('============================================================\n');

%% ==========================================================
%% Helper
%% ==========================================================
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
