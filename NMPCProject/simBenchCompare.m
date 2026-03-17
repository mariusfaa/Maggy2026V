%% simBenchCompare — Compare benchmark sim results against ODE reference
%
% Loads a reference .mat file and all simresults/*.mat files, interpolates
% to a common time grid, computes error metrics, and displays a summary table.

ref_file = 'results_ode_fast.mat';
bench_dir = 'simresults';

%% --- Load reference ---
ref = load(ref_file);
fprintf('Reference: %s  (%d points, T=%.3fs)\n', ref_file, size(ref.x, 2), ref.t(end));

% Expand 10-state to 12-state if needed
if size(ref.x, 1) == 10
    ref.x = ref.x([1:5, 5, 6:10], :);
    ref.x(6, :) = 0;
    ref.xEq = ref.xEq([1:5; 5; 6:10]);
    ref.xEq(6) = 0;
end
nx_ref = size(ref.x, 1);

%% --- Load benchmark files ---
files = dir(fullfile(bench_dir, 'sim_*.mat'));
nFiles = length(files);
fprintf('Found %d benchmark files in %s/\n\n', nFiles, bench_dir);

if nFiles == 0
    error('No benchmark files found in %s/', bench_dir);
end

%% --- State labels ---
state_labels = {'x','y','z','roll','pitch','yaw','vx','vy','vz','wx','wy','wz'};

%% --- Compute errors for each file ---
names     = cell(nFiles, 1);
integrator = cell(nFiles, 1);
n_stages  = zeros(nFiles, 1);
n_steps   = zeros(nFiles, 1);
n_radial  = zeros(nFiles, 1);
t_build   = zeros(nFiles, 1);
t_med     = zeros(nFiles, 1);
t_avg     = zeros(nFiles, 1);
t_max     = zeros(nFiles, 1);
n_sim_steps = zeros(nFiles, 1);
diverged  = false(nFiles, 1);

% Error metrics
max_pos_err  = zeros(nFiles, 1);  % max position error [mm]
max_ang_err  = zeros(nFiles, 1);  % max angle error [deg]
rms_pos_err  = zeros(nFiles, 1);  % RMS position error [mm]
rms_ang_err  = zeros(nFiles, 1);  % RMS angle error [deg]
max_z_err    = zeros(nFiles, 1);  % max z error [mm]
rms_z_err    = zeros(nFiles, 1);  % RMS z error [mm]

for k = 1:nFiles
    d = load(fullfile(bench_dir, files(k).name));
    names{k} = erase(files(k).name, {'.mat', 'sim_'});

    % Extract config
    if isfield(d, 'config')
        integrator{k} = d.config.integrator;
        n_stages(k)   = d.config.num_stages;
        n_steps(k)    = d.config.num_steps;
        n_radial(k)   = d.config.n_radial;
    else
        integrator{k} = '?';
    end

    if isfield(d, 't_build')
        t_build(k) = d.t_build;
    end

    % Timing stats
    if isfield(d, 't_tot')
        t_med(k) = median(d.t_tot) * 1e6;
        t_avg(k) = mean(d.t_tot) * 1e6;
        t_max(k) = max(d.t_tot) * 1e6;
    end
    n_sim_steps(k) = size(d.x, 2);

    % Expand 10-state to 12-state
    x_bench = d.x;
    if size(x_bench, 1) == 10
        x_bench = x_bench([1:5, 5, 6:10], :);
        x_bench(6, :) = 0;
    end

    % Check divergence (didn't run full duration)
    T_ref = ref.t(end);
    T_bench = d.t(end);
    if T_bench < T_ref * 0.99
        diverged(k) = true;
    end

    % Interpolate reference onto benchmark time grid for comparison
    t_common = d.t(d.t <= T_ref);
    n_common = length(t_common);

    x_ref_interp = zeros(nx_ref, n_common);
    for s = 1:nx_ref
        x_ref_interp(s, :) = interp1(ref.t, ref.x(s, :), t_common, 'linear', NaN);
    end
    x_bench_common = x_bench(:, 1:n_common);

    % Position error (states 1:3) in mm
    pos_err = (x_bench_common(1:3, :) - x_ref_interp(1:3, :)) * 1e3;
    pos_err_norm = vecnorm(pos_err, 2, 1);
    max_pos_err(k) = max(pos_err_norm);
    rms_pos_err(k) = rms(pos_err_norm);

    % Z error specifically
    z_err = pos_err(3, :);
    max_z_err(k) = max(abs(z_err));
    rms_z_err(k) = rms(z_err);

    % Angle error (states 4:5, skip yaw=6) in deg
    ang_err = (x_bench_common(4:5, :) - x_ref_interp(4:5, :)) * 180/pi;
    ang_err_norm = vecnorm(ang_err, 2, 1);
    max_ang_err(k) = max(ang_err_norm);
    rms_ang_err(k) = rms(ang_err_norm);
end

%% --- Build and display table ---
T = table(names, integrator, n_stages, n_steps, n_radial, ...
    t_build, t_med, t_avg, t_max, ...
    max_pos_err, rms_pos_err, max_z_err, rms_z_err, ...
    max_ang_err, rms_ang_err, diverged, ...
    'VariableNames', {'Config', 'Integrator', 'Stages', 'Steps', 'nRadial', ...
    'BuildTime_s', 'Median_us', 'Mean_us', 'Max_us', ...
    'MaxPosErr_mm', 'RmsPosErr_mm', 'MaxZErr_mm', 'RmsZErr_mm', ...
    'MaxAngErr_deg', 'RmsAngErr_deg', 'Diverged'});

% Sort by RMS position error
T = sortrows(T, 'RmsPosErr_mm');

fprintf('\n==================== BENCHMARK COMPARISON ====================\n');
fprintf('Reference: %s\n\n', ref_file);
disp(T);

%% --- Highlight best configs ---
fprintf('\n--- Top 5 by accuracy (RMS position error) ---\n');
disp(T(1:min(5, nFiles), {'Config', 'Median_us', 'RmsPosErr_mm', 'MaxZErr_mm', 'RmsAngErr_deg'}));

% Best speed among non-diverged
T_valid = T(~T.Diverged, :);
if ~isempty(T_valid)
    T_speed = sortrows(T_valid, 'Median_us');
    fprintf('--- Top 5 by speed (median step time, non-diverged) ---\n');
    disp(T_speed(1:min(5, height(T_speed)), {'Config', 'Median_us', 'RmsPosErr_mm', 'MaxZErr_mm', 'RmsAngErr_deg'}));
end
