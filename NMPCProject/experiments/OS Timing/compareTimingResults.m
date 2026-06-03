function compareTimingResults(results_dir)
% COMPARETIMINGRESULTS  Aggregate timing_results_*.mat files into thesis-ready
%   tables and plots.
%
%   Usage:
%       compareTimingResults                              % default folder
%       compareTimingResults('path/to/timing_results')
%
%   Inputs:
%       results_dir  (optional) folder containing timing_results_*.mat files.
%                    Defaults to ./timing_results or NMPCProject/timing_results
%                    depending on where MATLAB is started from.
%
%   Outputs (written into results_dir):
%       timing_comparison.csv         summary table, one row per setup
%       timing_comparison.tex         LaTeX tabular block (\input-able)
%       timing_boxplot_wall.pdf       per-step wall-clock distribution
%       timing_boxplot_acados.pdf     per-step acados time_tot distribution
%       timing_compile_bar.pdf        cold-compile time bar chart
%       timing_per_run.pdf            per-run avg solve time (run-to-run noise)
%
%   The script assumes all .mat files were produced by
%   workingSimulatorReducedOrderTiming.m (same field layout).
%
%   Author: Marius Jullum Faanes
% =========================================================================

%% --- LOCATE DATA ---
if nargin < 1 || isempty(results_dir)
    candidates = {'timing_results', fullfile('NMPCProject', 'timing_results')};
    results_dir = '';
    for k = 1:numel(candidates)
        if exist(candidates{k}, 'dir')
            results_dir = candidates{k};
            break;
        end
    end
    if isempty(results_dir)
        error(['No timing_results directory found. ' ...
               'Pass it explicitly: compareTimingResults(''path/to/dir'')']);
    end
end

files = dir(fullfile(results_dir, 'timing_results_*.mat'));
if isempty(files)
    error('No timing_results_*.mat files in %s', results_dir);
end

fprintf('Found %d timing result file(s) in %s\n', numel(files), results_dir);

%% --- LOAD ALL FILES ---
n        = numel(files);
data(n)  = struct();
deadline_ms = NaN;

for k = 1:n
    fpath  = fullfile(files(k).folder, files(k).name);
    s      = load(fpath);
    info   = parseSetupInfo(s.meta);

    data(k).filename       = files(k).name;
    data(k).os             = info.os;
    data(k).mode           = info.mode;
    data(k).label          = info.label;
    data(k).matlab_release = s.meta.matlab_release;
    data(k).compile_time_s = s.compile_time_s;
    data(k).aggregate      = s.aggregate;
    data(k).per_run        = s.per_run;
    data(k).meta           = s.meta;

    % Flatten only the *valid* per-step entries — divergent runs only fill
    % the matrix up to per_run.n_actual(r); the rest are pre-allocated zeros
    % which would otherwise pull averages and the lower whisker of the
    % boxplot toward zero.
    data(k).flat_wall_ms   = flattenValid(s.per_run.solve_time_wall,   s.per_run.n_actual) * 1000;
    data(k).flat_acados_ms = flattenValid(s.per_run.solve_time_acados, s.per_run.n_actual) * 1000;
    if any(s.per_run.diverged)
        warning('Dataset %s has %d diverged run(s); zeros excluded.', ...
                files(k).name, sum(s.per_run.diverged));
    end

    % Use the first dataset's Tf/N as the reference real-time deadline; warn
    % later if any other dataset disagrees (the comparison only makes sense
    % if all setups ran the same OCP).
    this_deadline = (s.meta.Tf / s.meta.N) * 1000;
    if isnan(deadline_ms)
        deadline_ms = this_deadline;
    elseif abs(this_deadline - deadline_ms) > 1e-9
        warning(['Dataset %s has a different per-step deadline (%.2f ms) ' ...
                 'than the reference (%.2f ms). The comparison may not be ' ...
                 'apples-to-apples.'], files(k).name, this_deadline, deadline_ms);
    end
end

%% --- SORT (Windows, WSL, Ubuntu) x (GUI, headless) ---
os_order   = {'Windows', 'WSL', 'Ubuntu'};
mode_order = {'GUI', 'headless'};
sort_key   = zeros(1, n);
for k = 1:n
    o = find(strcmp(os_order,   data(k).os),   1);
    m = find(strcmp(mode_order, data(k).mode), 1);
    if isempty(o), o = numel(os_order)   + 1; end
    if isempty(m), m = numel(mode_order) + 1; end
    sort_key(k) = (o - 1) * 10 + m;
end
[~, sort_idx] = sort(sort_key);
data = data(sort_idx);

%% --- CONSOLE SUMMARY ---
fprintf('\n=== Timing comparison (deadline = %.2f ms) ===\n', deadline_ms);
fprintf('%-22s %-12s %10s %14s %14s %14s %14s %10s\n', ...
    'Setup','MATLAB','Compile [s]', ...
    'Avg ac [ms]','Max ac [ms]','Avg wc [ms]','Max wc [ms]','%>deadl');
fprintf('%s\n', repmat('-', 1, 130));
for k = 1:n
    a = data(k).aggregate;
    pct = 100 * mean(data(k).flat_wall_ms > deadline_ms);
    fprintf('%-22s %-12s %10.2f %6.2f+/-%-5.2f %6.2f+/-%-5.2f %6.2f+/-%-5.2f %6.2f+/-%-5.2f %9.2f%%\n', ...
        data(k).label, data(k).matlab_release, data(k).compile_time_s, ...
        a.mean_avg_acados*1000, a.std_avg_acados*1000, ...
        a.mean_max_acados*1000, a.std_max_acados*1000, ...
        a.mean_avg_wall*1000,   a.std_avg_wall*1000, ...
        a.mean_max_wall*1000,   a.std_max_wall*1000, ...
        pct);
end

%% --- CSV ---
csv_path = fullfile(results_dir, 'timing_comparison.csv');
fid = fopen(csv_path, 'w');
if fid < 0
    error('Cannot open %s for writing', csv_path);
end
fprintf(fid, ['setup,os,mode,matlab_release,compile_time_s,' ...
              'mean_avg_acados_ms,std_avg_acados_ms,' ...
              'mean_max_acados_ms,std_max_acados_ms,' ...
              'mean_avg_wall_ms,std_avg_wall_ms,' ...
              'mean_max_wall_ms,std_max_wall_ms,' ...
              'median_wall_ms,p99_wall_ms,worst_wall_ms,pct_over_deadline\n']);
for k = 1:n
    a   = data(k).aggregate;
    w   = data(k).flat_wall_ms;                        % ms, valid steps only
    med = median(w);
    p99 = prctile(w, 99);
    worst = max(w);
    pct = 100 * mean(w > deadline_ms);
    fprintf(fid, ['"%s",%s,%s,%s,%.4f,' ...
                  '%.4f,%.4f,%.4f,%.4f,' ...
                  '%.4f,%.4f,%.4f,%.4f,' ...
                  '%.4f,%.4f,%.4f,%.4f\n'], ...
        data(k).label, data(k).os, data(k).mode, data(k).matlab_release, ...
        data(k).compile_time_s, ...
        a.mean_avg_acados*1000, a.std_avg_acados*1000, ...
        a.mean_max_acados*1000, a.std_max_acados*1000, ...
        a.mean_avg_wall*1000,   a.std_avg_wall*1000, ...
        a.mean_max_wall*1000,   a.std_max_wall*1000, ...
        med, p99, worst, pct);
end
fclose(fid);
fprintf('\nWrote %s\n', csv_path);

%% --- LATEX TABLE ---
tex_path = fullfile(results_dir, 'timing_comparison.tex');
writeLatexTable(tex_path, data, deadline_ms);
fprintf('Wrote %s\n', tex_path);

%% --- PLOTS ---
fprintf('Rendering boxplot (wall-clock)...\n');
plotBoxchart(data, 'flat_wall_ms',   'Wall-clock solve time [ms]', ...
    deadline_ms, fullfile(results_dir, 'timing_boxplot_wall.pdf'));
fprintf('Rendering boxplot (acados)...\n');
plotBoxchart(data, 'flat_acados_ms', 'acados time\_tot [ms]', ...
    deadline_ms, fullfile(results_dir, 'timing_boxplot_acados.pdf'));
fprintf('Rendering compile-time bar...\n');
plotCompileBar(data, fullfile(results_dir, 'timing_compile_bar.pdf'));
fprintf('Rendering per-run scatter...\n');
plotPerRunAvg(data, deadline_ms, fullfile(results_dir, 'timing_per_run.pdf'));

fprintf('\nDone.\n');

end % function compareTimingResults

% =========================================================================
% LOCAL HELPERS
% =========================================================================

function info = parseSetupInfo(meta)
% Map meta fields to a clean (os, mode, label) triple.
    if meta.is_wsl
        info.os = 'WSL';
    elseif strcmp(meta.computer_arch, 'win64')
        info.os = 'Windows';
    else
        info.os = 'Ubuntu';   % glnxa64 native
    end

    if meta.has_desktop && meta.has_jvm
        info.mode = 'GUI';
    elseif ~meta.has_desktop && ~meta.has_jvm
        info.mode = 'headless';
    elseif ~meta.has_desktop
        info.mode = 'no-desktop';
    else
        info.mode = 'no-JVM';
    end

    info.label = sprintf('%s (%s)', info.os, info.mode);
end

% -------------------------------------------------------------------------

function writeLatexTable(tex_path, data, deadline_ms)
% Emit a self-contained tabular block ready for \input{...}.
    fid = fopen(tex_path, 'w');
    if fid < 0
        error('Cannot open %s', tex_path);
    end
    fprintf(fid, '%% Auto-generated by compareTimingResults.m\n');
    fprintf(fid, '%% Real-time deadline = %.2f ms\n', deadline_ms);
    fprintf(fid, '\\begin{tabular}{lrrrrr}\n');
    fprintf(fid, '\\toprule\n');
    fprintf(fid, ['Setup & Compile [s] & Avg solve [ms] & Max solve [ms] ' ...
                  '& Worst [ms] & Over deadline [\\%%] \\\\\n']);
    fprintf(fid, '\\midrule\n');
    for k = 1:numel(data)
        a   = data(k).aggregate;
        w   = data(k).flat_wall_ms;
        worst = max(w);
        pct = 100 * mean(w > deadline_ms);
        fprintf(fid, '%s & %.1f & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & %.2f & %.2f \\\\\n', ...
            data(k).label, data(k).compile_time_s, ...
            a.mean_avg_wall*1000, a.std_avg_wall*1000, ...
            a.mean_max_wall*1000, a.std_max_wall*1000, ...
            worst, pct);
    end
    fprintf(fid, '\\bottomrule\n');
    fprintf(fid, '\\end{tabular}\n');
    fclose(fid);
end

% -------------------------------------------------------------------------

function plotBoxchart(data, flat_field, ylab, deadline_ms, out_pdf)
% Per-step distribution as grouped boxchart (OS on x, mode as color).
%   flat_field is the name of the precomputed flat-ms vector on `data`,
%   i.e. 'flat_wall_ms' or 'flat_acados_ms'.
    n = numel(data);

    % Long-format vectors (cell arrays grown then converted)
    total = 0;
    for k = 1:n
        total = total + numel(data(k).(flat_field));
    end
    y        = zeros(total, 1);
    os_cat   = cell(total, 1);
    mode_cat = cell(total, 1);
    idx = 1;
    for k = 1:n
        v   = data(k).(flat_field);
        m   = numel(v);
        rng = idx:(idx+m-1);
        y(rng)        = v;
        os_cat(rng)   = {data(k).os};
        mode_cat(rng) = {data(k).mode};
        idx = idx + m;
    end
    os_cat   = categorical(os_cat,   {'Windows','WSL','Ubuntu'}, 'Ordinal', true);
    mode_cat = categorical(mode_cat, {'GUI','headless'},         'Ordinal', true);

    fig = figure('Color', 'w', 'Position', [100 100 720 420]);
    % Hide outlier markers: with ~10k samples per setup, plotting each
    % outlier as a separate vector object makes both rendering and PDF
    % export glacial. The aggregate table reports max / p99 / pct over
    % deadline, which captures the tail information we'd otherwise lose.
    boxchart(os_cat, y, 'GroupByColor', mode_cat, 'MarkerStyle', 'none');
    hold on; grid on;
    yline(deadline_ms, '--r', sprintf('%.0f ms deadline', deadline_ms), ...
          'LabelHorizontalAlignment', 'left', 'LineWidth', 1.2);
    ylabel(ylab, 'Interpreter', 'tex');
    legend('Location','northeastoutside');
    set(gca, 'FontSize', 10);
    savePdf(fig, out_pdf);
end

% -------------------------------------------------------------------------

function plotCompileBar(data, out_pdf)
% Cold-compile wall time per setup.
    n      = numel(data);
    labels = arrayfun(@(d) d.label, data, 'UniformOutput', false);
    values = arrayfun(@(d) d.compile_time_s, data);

    fig = figure('Color','w','Position',[100 100 720 380]);
    b = bar(values, 'FaceColor', 'flat');
    grid on;
    % Color GUI vs headless differently
    cols = lines(2);
    for k = 1:n
        if strcmp(data(k).mode,'GUI')
            b.CData(k,:) = cols(1,:);
        else
            b.CData(k,:) = cols(2,:);
        end
    end
    set(gca, 'XTick', 1:n, 'XTickLabel', labels, 'XTickLabelRotation', 25, ...
        'FontSize', 10);
    ylabel('Cold-compile time [s]');
    title('First-run acados build time');
    % Numeric labels above bars
    for k = 1:n
        text(k, values(k), sprintf(' %.0f s', values(k)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
             'FontSize', 9);
    end
    ylim([0, max(values)*1.15]);
    savePdf(fig, out_pdf);
end

% -------------------------------------------------------------------------

function plotPerRunAvg(data, deadline_ms, out_pdf)
% Per-run average solve time — shows run-to-run consistency.
    n = numel(data);

    fig = figure('Color','w','Position',[100 100 720 420]);
    hold on; grid on;
    cols = lines(n);
    for k = 1:n
        avg_ms = data(k).per_run.avg_solve_time_wall * 1000;
        scatter(k * ones(size(avg_ms)), avg_ms, 50, cols(k,:), 'filled', ...
                'MarkerFaceAlpha', 0.7);
        % Mean as a horizontal tick
        mean_v = mean(avg_ms);
        plot([k-0.25, k+0.25], [mean_v mean_v], 'k-', 'LineWidth', 2);
    end
    yline(deadline_ms, '--r', sprintf('%.0f ms deadline', deadline_ms), ...
          'LineWidth', 1.2);
    set(gca, 'XTick', 1:n, ...
        'XTickLabel', arrayfun(@(d) d.label, data, 'UniformOutput', false), ...
        'XTickLabelRotation', 25, 'FontSize', 10);
    ylabel('Per-run average wall-clock solve time [ms]');
    title('Run-to-run consistency of average solve time');
    xlim([0.5, n+0.5]);
    savePdf(fig, out_pdf);
end

% -------------------------------------------------------------------------

function v = flattenValid(M, n_actual)
% FLATTENVALID  Concatenate row r's first n_actual(r) entries.
%   M is (n_runs x sim_steps); divergent runs only filled the first
%   n_actual(r) columns. Drop the trailing pre-allocated zeros.
    [n_runs, ~] = size(M);
    parts = cell(n_runs, 1);
    for r = 1:n_runs
        parts{r} = M(r, 1:n_actual(r))';
    end
    v = vertcat(parts{:});
end

% -------------------------------------------------------------------------

function savePdf(fig, out_pdf)
% Save figure as a tightly-cropped PDF for thesis inclusion.
%
% Uses `exportgraphics` (R2020a+) — much more reliable than the legacy
% `print -dpdf -vector` pipeline, which is prone to "graphics handshaking"
% timeouts on some Windows MATLAB setups even with simple figures.
% Falls back to a 300 DPI raster PDF if the vector path still fails.
    drawnow;
    try
        exportgraphics(fig, out_pdf, 'ContentType', 'vector');
    catch ME
        warning('Vector export failed (%s); using 300 DPI raster.', ME.message);
        exportgraphics(fig, out_pdf, 'ContentType', 'image', 'Resolution', 300);
    end
    fprintf('Wrote %s\n', out_pdf);
end
