%% compareExperiments.m
% =========================================================================
%  Loads all nmpc_results_*.mat files from the current directory, builds a
%  comparison table, and prints it to the console.  Re-run after each new
%  experiment to get an updated view.
%
%  Usage:
%    >> compareExperiments              % scans current directory
%    >> compareExperiments('results/')  % scans a specific folder
%
%  The script also saves the table as:
%    - experiment_comparison.mat   (MATLAB struct array)
%    - experiment_comparison.csv   (for LaTeX / Excel import)
%
%  Author: Marius Jullum Faanes
%  Date:   2026-03-29
% =========================================================================

function compareExperiments(results_dir)

if nargin < 1; results_dir = '.'; end

%% --- DISCOVER RESULT FILES ---
files = dir(fullfile(results_dir, 'nmpc_results_*.mat'));
if isempty(files)
    fprintf('No nmpc_results_*.mat files found in %s\n', results_dir);
    return;
end

% Sort by modification time so newest experiments appear last
[~, idx] = sort([files.datenum]);
files = files(idx);

fprintf('Found %d experiment result file(s):\n', length(files));
for k = 1:length(files)
    fprintf('  [%d] %s\n', k, files(k).name);
end
fprintf('\n');

%% --- LOAD AND EXTRACT SUMMARY FROM EACH FILE ---
rows = struct([]);

for k = 1:length(files)
    fpath = fullfile(files(k).folder, files(k).name);
    d = load(fpath);

    r = struct();
    r.filename = files(k).name;

    % --- Experiment label ---
    if isfield(d, 'experiment_name')
        r.name = d.experiment_name;
    else
        r.name = strrep(strrep(files(k).name, 'nmpc_results_', ''), '.mat', '');
    end

    % --- Configuration ---
    if isfield(d, 'config')
        r.nlp_solver   = field_or(d.config, 'nlp_solver_type', '?');
        r.integrator   = field_or(d.config, 'integrator_type', '?');
        r.num_stages   = field_or(d.config, 'sim_method_num_stages', NaN);
        r.num_steps    = field_or(d.config, 'sim_method_num_steps', NaN);
        r.cost_type    = field_or(d.config, 'cost_type', '?');
        r.N            = field_or(d.config, 'N', NaN);
        r.magnet_n     = field_or(d.config, 'magnet_n', NaN);
    else
        r.nlp_solver = '?'; r.integrator = '?'; r.num_stages = NaN;
        r.num_steps = NaN; r.cost_type = '?';
        r.N = field_or(d, 'N', NaN);
        r.magnet_n = NaN;
    end

    % --- Summary statistics ---
    if isfield(d, 'summary')
        s = d.summary;
        r.steps         = field_or(s, 'n_actual', NaN);
        r.all_conv      = field_or(s, 'all_converged', NaN);
        r.pct_ok        = field_or(s, 'pct_status_0', NaN);
        r.t_mean        = field_or(s, 'solve_time_mean_ms', NaN);
        r.t_median      = field_or(s, 'solve_time_median_ms', NaN);
        r.t_std         = field_or(s, 'solve_time_std_ms', NaN);
        r.t_max         = field_or(s, 'solve_time_max_ms', NaN);
        r.t_p95         = field_or(s, 'solve_time_p95_ms', NaN);
        r.t_p99         = field_or(s, 'solve_time_p99_ms', NaN);
        r.t_min         = field_or(s, 'solve_time_min_ms', NaN);
        r.sqp_mean      = field_or(s, 'sqp_iter_mean', NaN);
        r.qp_mean       = field_or(s, 'qp_iter_mean', NaN);
        r.t_lin_ms      = field_or(s, 'time_lin_mean_ms', NaN);
        r.t_sim_ms      = field_or(s, 'time_sim_mean_ms', NaN);
        r.t_qp_ms       = field_or(s, 'time_qp_sol_mean_ms', NaN);
        r.t_reg_ms      = field_or(s, 'time_reg_mean_ms', NaN);
        r.err_final     = field_or(s, 'final_err_norm', NaN);
        r.err_pos_mm    = field_or(s, 'final_err_pos_mm', NaN);
        r.err_ang_deg   = field_or(s, 'final_err_ang_deg', NaN);
        r.settle_1pct   = field_or(s, 'settling_1pct_ms', NaN);
        r.settle_5pct   = field_or(s, 'settling_5pct_ms', NaN);
        r.max_slack     = field_or(s, 'max_soft_slack', NaN);
    else
        % Fall back to computing from raw vectors (backward compat)
        tv = d.solve_time(1:min(end, field_or(d, 'n_actual', size(d.x,2)-1)));
        r.steps     = length(tv);
        r.all_conv  = all(d.status(1:r.steps) == 0);
        r.pct_ok    = 100 * sum(d.status(1:r.steps)==0) / r.steps;
        r.t_mean    = mean(tv)*1e3;
        r.t_median  = median(tv)*1e3;
        r.t_std     = std(tv)*1e3;
        r.t_max     = max(tv)*1e3;
        r.t_p95     = NaN; r.t_p99 = NaN; r.t_min = min(tv)*1e3;
        r.sqp_mean  = mean(d.sqp_iter(1:r.steps));
        r.qp_mean   = NaN;
        r.t_lin_ms  = NaN; r.t_sim_ms = NaN; r.t_qp_ms = NaN; r.t_reg_ms = NaN;
        r.err_final = norm(d.x(:,r.steps+1) - d.xEq);
        r.err_pos_mm = NaN; r.err_ang_deg = NaN;
        r.settle_1pct = NaN; r.settle_5pct = NaN; r.max_slack = NaN;
    end

    if isempty(rows); rows = r; else; rows(end+1) = r; end %#ok<AGROW>
end

n = length(rows);

%% --- COMPUTE SPEEDUP RELATIVE TO BASELINE ---
% The first row with 'Baseline' in the name is the reference.
% If no baseline found, use the first row.
base_idx = find(strcmpi({rows.name}, 'Baseline'), 1);
if isempty(base_idx); base_idx = 1; end
t_base = rows(base_idx).t_mean;

for k = 1:n
    rows(k).speedup = t_base / rows(k).t_mean;
end

%% === PRINT MAIN COMPARISON TABLE ===
sep = repmat('=', 1, 130);
fprintf('\n%s\n', sep);
fprintf('  NMPC EXPERIMENT COMPARISON  (baseline: %s, mean=%.1f ms)\n', ...
    rows(base_idx).name, t_base);
fprintf('%s\n', sep);

% Header
fprintf('%-14s | %7s %7s %7s %7s %7s | %5s | %6s %6s | %8s %8s %8s | %5s\n', ...
    'Experiment', 't_mean', 't_med', 't_p95', 't_max', 't_min', ...
    'Speed', 'SQP', 'QP', ...
    '|x-xEq|', 'SetT1%', 'SetT5%', 'Conv%');
fprintf('%-14s | %7s %7s %7s %7s %7s | %5s | %6s %6s | %8s %8s %8s | %5s\n', ...
    '', 'ms', 'ms', 'ms', 'ms', 'ms', ...
    'x', 'avg', 'avg', ...
    '', 'ms', 'ms', '%');
fprintf('%s\n', repmat('-', 1, 130));

for k = 1:n
    r = rows(k);
    % Highlight if target met
    if r.t_mean <= 5.0
        marker = ' ***';
    elseif r.t_mean <= 10.0
        marker = '  * ';
    else
        marker = '    ';
    end

    fprintf('%-14s | %6.1f %7.1f %7.1f %7.1f %7.1f | %4.1fx | %5.1f %6.1f | %8.1e %7.0f %7.0f | %5.1f%s\n', ...
        r.name, ...
        r.t_mean, r.t_median, r.t_p95, r.t_max, r.t_min, ...
        r.speedup, ...
        r.sqp_mean, r.qp_mean, ...
        r.err_final, r.settle_1pct, r.settle_5pct, ...
        r.pct_ok, marker);
end
fprintf('%s\n\n', sep);

%% === PRINT TIMING BREAKDOWN TABLE ===
fprintf('%-14s | %7s %7s %7s %7s %7s | %7s\n', ...
    'Experiment', 't_tot', 't_lin', 't_sim', 't_qp', 't_reg', 'Other');
fprintf('%-14s | %7s %7s %7s %7s %7s | %7s\n', ...
    '', 'ms', 'ms', 'ms', 'ms', 'ms', 'ms');
fprintf('%s\n', repmat('-', 1, 90));
for k = 1:n
    r = rows(k);
    other = r.t_mean - nansum_([r.t_lin_ms, r.t_qp_ms, r.t_reg_ms]);
    fprintf('%-14s | %6.1f %7.2f %7.2f %7.2f %7.2f | %6.2f\n', ...
        r.name, r.t_mean, r.t_lin_ms, r.t_sim_ms, r.t_qp_ms, r.t_reg_ms, other);
end
fprintf('\n');

%% === PRINT CONFIGURATION TABLE ===
fprintf('%-14s | %-8s %-5s %3s %3s | %-15s | %3s %3s | %s\n', ...
    'Experiment', 'NLP', 'Integ', 'St', 'Stp', 'QP solver', 'N', 'n', 'Cost');
fprintf('%s\n', repmat('-', 1, 90));
for k = 1:n
    r = rows(k);
    fprintf('%-14s | %-8s %-5s %3d %3d | %-15s | %3d %3d | %s\n', ...
        r.name, r.nlp_solver, r.integrator, ...
        r.num_stages, r.num_steps, ...
        truncstr(r.cost_type, 15), ...
        r.N, r.magnet_n, r.cost_type);
end
fprintf('\n');

%% === TARGET ASSESSMENT ===
fprintf('%s\n', sep);
best_idx = 1;
for k = 2:n
    if rows(k).t_mean < rows(best_idx).t_mean; best_idx = k; end
end
fprintf('  Best mean solve time:  %.2f ms  (%s)\n', rows(best_idx).t_mean, rows(best_idx).name);
fprintf('  Target:                5.00 ms\n');
if rows(best_idx).t_mean <= 5.0
    fprintf('  STATUS:                TARGET MET!\n');
else
    fprintf('  Remaining gap:         %.1fx\n', rows(best_idx).t_mean / 5.0);
end
fprintf('%s\n\n', sep);

%% --- SAVE COMPARISON DATA ---
% Struct array (for MATLAB reuse)
save(fullfile(results_dir, 'experiment_comparison.mat'), 'rows');

% CSV (for LaTeX / Excel)
csv_path = fullfile(results_dir, 'experiment_comparison.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, 'Experiment,NLP_solver,Integrator,Stages,Steps,N,magnet_n,Cost_type,');
fprintf(fid, 't_mean_ms,t_median_ms,t_p95_ms,t_max_ms,t_min_ms,Speedup,');
fprintf(fid, 'SQP_mean,QP_mean,t_lin_ms,t_sim_ms,t_qp_ms,t_reg_ms,');
fprintf(fid, 'err_final,settle_1pct_ms,settle_5pct_ms,pct_ok,max_slack\n');
for k = 1:n
    r = rows(k);
    fprintf(fid, '%s,%s,%s,%d,%d,%d,%d,%s,', ...
        r.name, r.nlp_solver, r.integrator, ...
        r.num_stages, r.num_steps, r.N, r.magnet_n, r.cost_type);
    fprintf(fid, '%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,', ...
        r.t_mean, r.t_median, r.t_p95, r.t_max, r.t_min, r.speedup);
    fprintf(fid, '%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,', ...
        r.sqp_mean, r.qp_mean, r.t_lin_ms, r.t_sim_ms, r.t_qp_ms, r.t_reg_ms);
    fprintf(fid, '%.4e,%.1f,%.1f,%.1f,%.4e\n', ...
        r.err_final, r.settle_1pct, r.settle_5pct, r.pct_ok, r.max_slack);
end
fclose(fid);
fprintf('Saved: experiment_comparison.mat, experiment_comparison.csv\n');

%% --- OPTIONAL: COMPARISON PLOTS ---
try
    % Bar chart: mean solve time with 5 ms target line
    figure('Name', 'Experiment Comparison', 'Position', [100 100 900 500]);

    subplot(1,2,1); hold on; grid on;
    t_means = [rows.t_mean];
    bar_h = barh(1:n, t_means, 'FaceColor', [0.3 0.5 0.8]);
    xline(5, '--r', '5 ms target', 'LineWidth', 1.5);
    set(gca, 'YTick', 1:n, 'YTickLabel', {rows.name}, 'YDir', 'reverse');
    xlabel('Mean solve time [ms]');
    title('Solve Time Comparison');
    % Color bars that meet target
    for k = 1:n
        if t_means(k) <= 5.0
            bar_h.FaceColor = 'flat';
            bar_h.CData(k,:) = [0.2 0.7 0.3];
        end
    end

    subplot(1,2,2); hold on; grid on;
    speedups = [rows.speedup];
    barh(1:n, speedups, 'FaceColor', [0.8 0.5 0.3]);
    xline(1, '--k', 'Baseline', 'LineWidth', 1);
    xline(t_base/5, '--r', sprintf('%.0fx needed', t_base/5), 'LineWidth', 1.2);
    set(gca, 'YTick', 1:n, 'YTickLabel', {rows.name}, 'YDir', 'reverse');
    xlabel('Speedup vs Baseline');
    title('Speedup Factor');

    sgtitle('NMPC Experiment Comparison');
    savefig(gcf, fullfile(results_dir, 'experiment_comparison.fig'));
    fprintf('Comparison figure saved.\n');
catch
    fprintf('(Plots skipped — no display available)\n');
end

end  % main function

%% === LOCAL HELPERS ===
function v = field_or(s, fname, default)
    if isfield(s, fname)
        v = s.(fname);
        if isnumeric(v) && isempty(v); v = default; end
    else
        v = default;
    end
end

function s = nansum_(x)
    x(isnan(x)) = 0;
    s = sum(x);
end

function s = truncstr(str, maxlen)
    if length(str) > maxlen
        s = str(1:maxlen);
    else
        s = str;
    end
end