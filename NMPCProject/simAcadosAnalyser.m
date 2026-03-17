%% simAcadosAnalyser — Parse and compare all simulation results in simresults/
%
% Parses filenames of the form: sim_fn_{integrator}_{stages}_{steps}_{dt}.mat
% Loads reference ODE solution for error computation.
% Produces: summary table, timing histograms, error bar charts.

clear; clc; close all;

%% ================================================================
%% 1. Discover and load files
%% ================================================================

sim_dir = 'simresults';
ref_file = 'results_ode_fast.mat';

% Exclude configs that are too inaccurate to compare meaningfully
exclude = {
    'ERK_5ms'   % ERK at 5ms diverges too quickly
};

files = dir(fullfile(sim_dir, 'sim_fn_*.mat'));
nFiles = numel(files);
assert(nFiles > 0, 'No sim_fn_*.mat files found in %s/', sim_dir);
fprintf('Found %d simulation files in %s/\n', nFiles, sim_dir);

% Load reference
has_ref = isfile(ref_file);
if has_ref
    ref = load(ref_file);
    fprintf('Reference: %s (T=%.3fs, %d pts)\n', ref_file, ref.t(end), numel(ref.t));
else
    fprintf('Warning: No reference file (%s) — error metrics unavailable.\n', ref_file);
end

%% ================================================================
%% 2. Parse filenames and compute metrics
%% ================================================================

% Pre-allocate result struct array
R = struct('name',{}, 'integrator',{}, 'stages',{}, 'steps',{}, ...
           'dt_ms',{}, 'dt_val',{}, 'diverged',{}, 'T_end',{}, ...
           'n_steps',{}, 't_tot_mean',{}, 't_tot_median',{}, ...
           't_tot_max',{}, 't_la_mean',{}, 't_ad_mean',{}, ...
           'pos_err_max',{}, 'ang_err_max',{}, ...
           'vel_err_max',{}, 'omg_err_max',{});

for i = 1:nFiles
    filepath = fullfile(files(i).folder, files(i).name);
    [~, fname] = fileparts(files(i).name);

    % Parse: sim_fn_{integrator}_{stages}_{steps}_{dt}
    tokens = regexp(fname, '^sim_fn_(\w+?)_(\d+)_(\d+)_(\d+)ms$', 'tokens');
    if isempty(tokens)
        fprintf('  Skipping (bad name): %s\n', fname);
        continue;
    end
    tok = tokens{1};

    r.name       = fname;
    r.integrator = upper(tok{1});
    r.stages     = str2double(tok{2});
    r.steps      = str2double(tok{3});
    r.dt_ms      = str2double(tok{4});
    r.dt_val     = r.dt_ms * 1e-3;

    % Skip excluded combos
    tag = sprintf('%s_%dms', r.integrator, r.dt_ms);
    if any(strcmp(tag, exclude))
        fprintf('  Excluding: %s\n', fname);
        continue;
    end

    % Load data
    d = load(filepath);

    r.n_steps = numel(d.t);
    r.T_end   = d.t(end);

    % Determine if diverged (shorter than expected full sim)
    T_full = 0.5;  % expected full sim length from simSetup
    r.diverged = r.T_end < T_full - r.dt_val;

    % Timing stats (in microseconds)
    if isfield(d, 't_tot') && ~isempty(d.t_tot)
        r.t_tot_mean   = mean(d.t_tot) * 1e6;
        r.t_tot_median = median(d.t_tot) * 1e6;
        r.t_tot_max    = max(d.t_tot) * 1e6;
    else
        r.t_tot_mean = NaN; r.t_tot_median = NaN; r.t_tot_max = NaN;
    end
    if isfield(d, 't_la') && ~isempty(d.t_la)
        r.t_la_mean = mean(d.t_la) * 1e6;
    else
        r.t_la_mean = NaN;
    end
    if isfield(d, 't_ad') && ~isempty(d.t_ad)
        r.t_ad_mean = mean(d.t_ad) * 1e6;
    else
        r.t_ad_mean = NaN;
    end

    % Error vs reference (interpolate to common grid)
    r.pos_err_max = NaN;
    r.ang_err_max = NaN;
    r.vel_err_max = NaN;
    r.omg_err_max = NaN;

    if has_ref
        T_common = min(r.T_end, ref.t(end));
        if T_common > 0
            % Build uniform grid at this sim's dt
            t_grid = 0:r.dt_val:T_common;
            if numel(t_grid) >= 2
                x_sim = interp1(d.t, d.x', t_grid, 'pchip')';
                x_ref = interp1(ref.t, ref.x', t_grid, 'pchip')';
                err = abs(x_sim - x_ref);
                r.pos_err_max = max(err(1:3, :), [], 'all');
                r.ang_err_max = max(err(4:6, :), [], 'all');
                r.vel_err_max = max(err(7:9, :), [], 'all');
                r.omg_err_max = max(err(10:12, :), [], 'all');
            end
        end
    end

    R = [R, r]; %#ok<AGROW>
end

nR = numel(R);
fprintf('Parsed %d / %d files successfully.\n\n', nR, nFiles);

%% ================================================================
%% 3. Summary table
%% ================================================================

% Sort by: integrator, dt, steps
[~, sortIdx] = sortrows([[R.dt_ms]', ...
                         cellfun(@(s) double(s(1)=='I'), {R.integrator})', ...
                         [R.steps]'], [1, 2, 3]);
R = R(sortIdx);

% Build table
Name       = {R.name}';
Integrator = {R.integrator}';
Stages     = [R.stages]';
Steps      = [R.steps]';
dt_ms      = [R.dt_ms]';
Diverged   = [R.diverged]';
T_end_s    = [R.T_end]';
N_steps    = [R.n_steps]';
Mean_us    = [R.t_tot_mean]';
Median_us  = [R.t_tot_median]';
Max_us     = [R.t_tot_max]';
LA_us      = [R.t_la_mean]';
AD_us      = [R.t_ad_mean]';
PosErr_mm  = [R.pos_err_max]' * 1e3;
AngErr_deg = [R.ang_err_max]' * 180/pi;

T = table(Integrator, Stages, Steps, dt_ms, Diverged, T_end_s, N_steps, ...
          Mean_us, Median_us, Max_us, LA_us, AD_us, PosErr_mm, AngErr_deg);

% Short display name
Labels = cell(nR, 1);
for i = 1:nR
    Labels{i} = sprintf('%s s%d n%d %dms', R(i).integrator, R(i).stages, ...
                        R(i).steps, R(i).dt_ms);
end
T.Properties.RowNames = Labels;

fprintf('============================================================\n');
fprintf('  SIMULATION RESULTS SUMMARY\n');
fprintf('============================================================\n\n');
disp(T);

% Highlight best configs per dt group (longest surviving, then fastest)
dts = unique(dt_ms);
for d = dts'
    mask = dt_ms == d;
    if ~any(mask), continue; end
    subset = T(mask, :);
    [~, best] = max(subset.T_end_s);
    fprintf('Longest (dt=%dms): %s  (T=%.3fs, mean=%.0f us, pos_err=%.4f mm)\n', ...
        d, subset.Properties.RowNames{best}, subset.T_end_s(best), ...
        subset.Mean_us(best), subset.PosErr_mm(best));
end
fprintf('\n');

%% ================================================================
%% 4. Figure 1: Runtime comparison (grouped bar)
%% ================================================================

figure('Name','Runtime Comparison','Position',[50 400 1400 500]);

% Group by dt
for di = 1:numel(dts)
    subplot(1, numel(dts), di); hold on; grid on; box on;

    mask = [R.dt_ms] == dts(di);
    idx = find(mask);
    n = numel(idx);

    erk_mask = cellfun(@(s) strcmp(s,'ERK'), {R(idx).integrator});
    irk_mask = ~erk_mask;

    bar_data = [R(idx).t_tot_median];
    bar_colors = zeros(n, 3);
    bar_colors(erk_mask, :) = repmat([0.2 0.6 1.0], sum(erk_mask), 1);
    bar_colors(irk_mask, :) = repmat([1.0 0.4 0.2], sum(irk_mask), 1);

    b = bar(1:n, bar_data);
    b.FaceColor = 'flat';
    b.CData = bar_colors;

    tick_labels = arrayfun(@(k) sprintf('%s n%d', R(k).integrator, R(k).steps), ...
                           idx, 'UniformOutput', false);
    xticks(1:n);
    xticklabels(tick_labels);
    xtickangle(45);
    ylabel('Median step time (\mus)');
    title(sprintf('dt = %d ms', dts(di)));

    % Budget line
    yline(dts(di) * 1e3, 'r--', 'LineWidth', 1.5, ...
          'Label', 'Real-time budget', 'LabelHorizontalAlignment', 'left');
end
sgtitle('Simulation Step Time (blue=ERK, orange=IRK, dashed red edge=diverged)', ...
        'FontSize', 13, 'FontWeight', 'bold');

%% ================================================================
%% 5. Figure 2: Error comparison (grouped bar)
%% ================================================================

if has_ref
    figure('Name','Survival & Error','Position',[50 50 1400 500]);

    for di = 1:numel(dts)
        subplot(1, numel(dts), di); hold on; grid on; box on;

        mask = [R.dt_ms] == dts(di);
        idx = find(mask);
        n = numel(idx);
        if n == 0, continue; end

        T_ends = [R(idx).T_end];

        yyaxis left;
        b1 = bar((1:n) - 0.15, T_ends, 0.3, 'FaceColor', [0.2 0.6 1.0]);
        ylabel('Survival time (s)');

        yyaxis right;
        pos_errs = [R(idx).pos_err_max] * 1e3;
        pos_errs(isnan(pos_errs)) = 0;
        b2 = bar((1:n) + 0.15, pos_errs, 0.3, 'FaceColor', [1.0 0.6 0.2]);
        ylabel('Max pos error (mm)');

        tick_labels = arrayfun(@(k) sprintf('%s n%d', R(k).integrator, R(k).steps), ...
                               idx, 'UniformOutput', false);
        xticks(1:n);
        xticklabels(tick_labels);
        xtickangle(45);
        title(sprintf('dt = %d ms', dts(di)));
        legend([b1 b2], {'Survival (s)', 'Pos err (mm)'}, 'Location', 'best');
    end
    sgtitle('Survival Time & Max Position Error vs ODE Reference', ...
            'FontSize', 13, 'FontWeight', 'bold');
end

%% ================================================================
%% 6. Figure 3: Timing histograms (ERK vs IRK at each dt)
%% ================================================================

figure('Name','Timing Histograms','Position',[100 200 1400 600]);

plot_idx = 0;
for di = 1:numel(dts)
    for integ = ["ERK", "IRK"]
        plot_idx = plot_idx + 1;
        subplot(numel(dts), 2, plot_idx); hold on; grid on; box on;

        mask = [R.dt_ms] == dts(di) & strcmp({R.integrator}, integ);
        idx = find(mask);
        if isempty(idx)
            title(sprintf('%s dt=%dms (none)', integ, dts(di)));
            continue;
        end

        % Load raw timing data for histograms
        colors = parula(numel(idx) + 1);
        for j = 1:numel(idx)
            filepath = fullfile(sim_dir, [R(idx(j)).name '.mat']);
            d = load(filepath, 't_tot');
            if isfield(d, 't_tot')
                histogram(d.t_tot * 1e6, 30, 'FaceAlpha', 0.5, ...
                    'FaceColor', colors(j,:), ...
                    'DisplayName', sprintf('n%d', R(idx(j)).steps));
            end
        end

        xlabel('Step time (\mus)');
        ylabel('Count');
        title(sprintf('%s  dt=%dms', integ, dts(di)));
        legend('Location', 'best');

        % Budget line
        xline(dts(di) * 1e3, 'r--', 'LineWidth', 1.5);
    end
end
sgtitle('Step Time Distributions (red line = real-time budget)', ...
        'FontSize', 13, 'FontWeight', 'bold');

%% ================================================================
%% 7. Figure 4: Pareto — accuracy vs speed
%% ================================================================

if has_ref
    figure('Name','Pareto: Speed vs Accuracy','Position',[150 100 800 600]);
    hold on; grid on; box on;

    markers = struct('ERK','o', 'IRK','s');
    dt_colors = lines(numel(dts));

    T_max = max([R.T_end]);
    for i = 1:nR
        if isnan(R(i).pos_err_max) || isnan(R(i).t_tot_median), continue; end

        di = find(dts == R(i).dt_ms);
        mk = markers.(R(i).integrator);
        sz = 30 + 120 * (R(i).T_end / T_max);  % bigger = longer survival

        scatter(R(i).t_tot_median, R(i).pos_err_max * 1e3, sz, ...
                dt_colors(di,:), mk, 'filled', 'MarkerEdgeColor', 'k');
        text(R(i).t_tot_median * 1.05, R(i).pos_err_max * 1e3 * 1.05, ...
             sprintf('n%d', R(i).steps), 'FontSize', 7);
    end

    set(gca, 'XScale', 'log', 'YScale', 'log');
    xlabel('Median step time (\mus)');
    ylabel('Max position error (mm)');
    title('Speed vs Accuracy Pareto');

    % Legend entries
    legend_entries = {};
    legend_handles = [];
    for di = 1:numel(dts)
        for integ = ["ERK", "IRK"]
            mk = markers.(integ);
            h = scatter(NaN, NaN, 80, dt_colors(di,:), mk, 'filled', ...
                        'MarkerEdgeColor', 'k');
            legend_handles(end+1) = h; %#ok<AGROW>
            legend_entries{end+1} = sprintf('%s %dms', integ, dts(di)); %#ok<AGROW>
        end
    end
    legend(legend_handles, legend_entries, 'Location', 'best');
end

%% ================================================================
%% 8. Figure 5 & 6: Trajectory comparisons (auto-selected subset)
%% ================================================================
%
% Selection: per (integrator, dt) group pick n=1 (fastest) and the
% longest-surviving config (most accurate). Deduplicate.

if has_ref
    sel_idx = [];
    for d = dts'
        for integ = ["ERK", "IRK"]
            grp = find([R.dt_ms] == d & strcmp({R.integrator}, integ));
            if isempty(grp), continue; end

            % Fastest (n=1 or lowest steps)
            [~, mi] = min([R(grp).steps]);
            sel_idx(end+1) = grp(mi); %#ok<AGROW>

            % Longest surviving
            [~, mi] = max([R(grp).T_end]);
            sel_idx(end+1) = grp(mi); %#ok<AGROW>
        end
    end
    sel_idx = unique(sel_idx, 'stable');
    nSel = numel(sel_idx);

    % Use common time window: shortest of all selected + reference
    T_plot = min([ref.t(end), R(sel_idx).T_end]);

    sel_colors = lines(nSel + 1);

    state_names = {'x','y','z','roll','pitch','yaw'};
    pos_scale = 1e3; ang_scale = 180/pi;

    % --- Figure 5: Position trajectories ---
    figure('Name','Trajectory: Position','Position',[50 300 1400 500]);
    for si = 1:3
        subplot(1,3,si); hold on; grid on; box on;

        % Reference
        ref_mask = ref.t <= T_plot + 1e-12;
        plot(ref.t(ref_mask), ref.x(si, ref_mask) * pos_scale, ...
             'k-', 'LineWidth', 2, 'DisplayName', 'ODE ref');

        for j = 1:nSel
            k = sel_idx(j);
            d = load(fullfile(sim_dir, [R(k).name '.mat']), 't', 'x');
            sim_mask = d.t <= T_plot + 1e-12;
            label = sprintf('%s n%d %dms', R(k).integrator, R(k).steps, R(k).dt_ms);
            plot(d.t(sim_mask), d.x(si, sim_mask) * pos_scale, ...
                 '-', 'Color', sel_colors(j,:), 'LineWidth', 1.2, ...
                 'DisplayName', label);
        end

        xlabel('Time (s)'); ylabel('mm');
        title(state_names{si});
        if si == 1, legend('Location','best','FontSize',7); end
    end
    sgtitle(sprintf('Position Trajectories vs Reference (T=%.3fs)', T_plot), ...
            'FontSize', 13, 'FontWeight', 'bold');

    % --- Figure 6: Orientation trajectories ---
    figure('Name','Trajectory: Orientation','Position',[50 50 1400 500]);
    for si = 1:3
        subplot(1,3,si); hold on; grid on; box on;

        ref_mask = ref.t <= T_plot + 1e-12;
        plot(ref.t(ref_mask), ref.x(si+3, ref_mask) * ang_scale, ...
             'k-', 'LineWidth', 2, 'DisplayName', 'ODE ref');

        for j = 1:nSel
            k = sel_idx(j);
            d = load(fullfile(sim_dir, [R(k).name '.mat']), 't', 'x');
            sim_mask = d.t <= T_plot + 1e-12;
            label = sprintf('%s n%d %dms', R(k).integrator, R(k).steps, R(k).dt_ms);
            plot(d.t(sim_mask), d.x(si+3, sim_mask) * ang_scale, ...
                 '-', 'Color', sel_colors(j,:), 'LineWidth', 1.2, ...
                 'DisplayName', label);
        end

        xlabel('Time (s)'); ylabel('deg');
        title(state_names{si+3});
        if si == 1, legend('Location','best','FontSize',7); end
    end
    sgtitle(sprintf('Orientation Trajectories vs Reference (T=%.3fs)', T_plot), ...
            'FontSize', 13, 'FontWeight', 'bold');

    % --- Figure 7: Position error over time ---
    figure('Name','Trajectory: Position Error','Position',[100 200 1400 500]);
    for si = 1:3
        subplot(1,3,si); hold on; grid on; box on;

        for j = 1:nSel
            k = sel_idx(j);
            d = load(fullfile(sim_dir, [R(k).name '.mat']), 't', 'x');
            T_common = min(d.t(end), ref.t(end));
            t_grid = 0:R(k).dt_val:T_common;
            if numel(t_grid) < 2, continue; end
            x_sim = interp1(d.t, d.x', t_grid, 'pchip')';
            x_ref = interp1(ref.t, ref.x', t_grid, 'pchip')';
            err = abs(x_sim(si,:) - x_ref(si,:)) * pos_scale;

            label = sprintf('%s n%d %dms', R(k).integrator, R(k).steps, R(k).dt_ms);
            plot(t_grid, err, '-', 'Color', sel_colors(j,:), ...
                 'LineWidth', 1.2, 'DisplayName', label);
        end

        xlabel('Time (s)'); ylabel('mm');
        title([state_names{si} ' error']);
        if si == 1, legend('Location','best','FontSize',7); end
    end
    sgtitle('Position Error vs Reference Over Time', ...
            'FontSize', 13, 'FontWeight', 'bold');

    fprintf('\nTrajectory plots: %d configs selected\n', nSel);
    fprintf('  ');
    for j = 1:nSel
        k = sel_idx(j);
        fprintf('%s n%d %dms (T=%.3fs)  ', ...
            R(k).integrator, R(k).steps, R(k).dt_ms, R(k).T_end);
    end
    fprintf('\n');
end

fprintf('Done.\n');
