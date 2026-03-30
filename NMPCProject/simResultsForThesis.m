%% simResultsForThesis — Load sweep results and produce thesis figures
%
% Loads all 45 .mat files from the parameter sweep (3 controllers x 5 horizons
% x 3 dt values) and computes:
%   - Settling time (2% criterion on z-axis)
%   - Overshoot (z-axis)
%   - Controller-only timing (excludes plant sim time)
%   - Marginal improvement in settling time between horizon increments
%
% Produces figures saved to Master_thesis_2025/figures/:
%   settle_heatmap.pdf    — settling time across N and dt, per controller
%   timing_bars.pdf       — median controller time vs N, grouped by dt
%   settle_vs_N.pdf       — settling time curves vs N, per controller
%   marginal_heatmap.pdf  — % improvement per horizon increment
%
% Also prints a summary table and saves summary_table.csv.
%
% Usage: run from NMPCProject root after sourcing env.sh, OR set
%        project_root manually below.

%% --- Path setup ---
project_root = getenv('ACADOS_PROJECT_DIR');
if isempty(project_root)
    % Fallback: assume script is run from NMPCProject root
    project_root = fileparts(mfilename('fullpath'));
    if isempty(project_root)
        project_root = pwd;
    end
end

addpath(project_root);  % for getFilename

results_dir = fullfile(project_root, 'results');
fig_dir     = fullfile(project_root, 'Master_thesis_2025', 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

%% --- Index arrays ---
controllers  = {'nmpc',  'lmpc',  'solmpc'};
ctrl_labels  = {'NMPC',  'LMPC',  'SolMPC'};
N_list       = [10, 20, 30, 40, 50];
dt_list_ms   = [1, 2, 3];
dt_list_s    = dt_list_ms * 1e-3;
nC = numel(controllers);
nN = numel(N_list);
nDt = numel(dt_list_ms);

%% --- Pre-allocate metric arrays ---
data          = cell(nC, nN, nDt);
diverged_map  = false(nC, nN, nDt);
settle_ms     = nan(nC, nN, nDt);
overshoot_pct = nan(nC, nN, nDt);
ctrl_med_us   = nan(nC, nN, nDt);
ctrl_max_us   = nan(nC, nN, nDt);
qp_med_us     = nan(nC, nN, nDt);
meets_rt      = false(nC, nN, nDt);

%% --- Load data and compute metrics ---
fprintf('Loading results...\n');
for ic = 1:nC
    for iN = 1:nN
        for iDt = 1:nDt
            fname = getFilename(controllers{ic}, N_list(iN), dt_list_s(iDt));
            fpath = fullfile(results_dir, fname);
            if ~isfile(fpath)
                warning('Missing: %s', fname);
                continue;
            end
            d = load(fpath);
            data{ic, iN, iDt} = d;

            % --- Divergence ---
            div = logical(d.diverged);
            diverged_map(ic, iN, iDt) = div;

            % --- Settling time (2% criterion on z, state index 3) ---
            settle_ms(ic, iN, iDt) = computeSettlingTime(d, 3);

            % --- Overshoot on z ---
            if ~div
                z    = d.x(3, :);
                z_eq = d.xEq(3);
                z0   = z(1);
                z_final = mean(z(max(1,end-9):end));
                dir_sign = sign(z_final - z0);
                if dir_sign > 0
                    os_abs = max(z - z_final);
                else
                    os_abs = min(z - z_final);  % negative if undershot
                end
                denom = abs(z0 - z_final);
                if denom > 1e-9
                    overshoot_pct(ic, iN, iDt) = (os_abs / denom) * 100;
                end
            end

            % --- Controller-only timing (excludes sim_time_tot) ---
            if strcmp(controllers{ic}, 'solmpc') && isfield(d, 'jac_time')
                ctrl_time = d.jac_time + d.ocp_time_tot;
            else
                ctrl_time = d.ocp_time_tot;
            end
            ctrl_med_us(ic, iN, iDt) = median(ctrl_time) * 1e6;
            ctrl_max_us(ic, iN, iDt) = max(ctrl_time) * 1e6;
            qp_med_us(ic, iN, iDt)   = median(d.ocp_time_qp) * 1e6;
            meets_rt(ic, iN, iDt)    = ctrl_max_us(ic, iN, iDt) <= dt_list_ms(iDt) * 1000;
        end
    end
end
fprintf('  Done.\n\n');

%% --- Marginal improvement table ---
% marginal_pct(ic, k, iDt) = % improvement from N_list(k) to N_list(k+1)
marginal_pct = nan(nC, nN-1, nDt);
for ic = 1:nC
    for iDt = 1:nDt
        ts = squeeze(settle_ms(ic, :, iDt));   % 1 x nN
        for k = 1:nN-1
            if ~isnan(ts(k)) && ~isnan(ts(k+1)) && ts(k) > 0
                marginal_pct(ic, k, iDt) = (ts(k) - ts(k+1)) / ts(k) * 100;
            end
        end
    end
end

%% --- Console summary table ---
fprintf('%s\n', repmat('=', 1, 100));
fprintf('%-8s  %-4s  %-6s  %12s  %12s  %12s  %10s  %8s\n', ...
    'Ctrl', 'N', 'dt(ms)', 'T_settle(ms)', 'CtrlMed(us)', 'CtrlMax(us)', 'QPMed(us)', 'MeetsRT');
fprintf('%s\n', repmat('-', 1, 100));
for ic = 1:nC
    for iN = 1:nN
        for iDt = 1:nDt
            if diverged_map(ic, iN, iDt)
                ts_str = 'DIV';
            elseif isnan(settle_ms(ic, iN, iDt))
                ts_str = 'NaN';
            else
                ts_str = sprintf('%.2f', settle_ms(ic, iN, iDt));
            end
            rt_str = ternary(meets_rt(ic, iN, iDt), 'YES', 'no');
            fprintf('%-8s  %-4d  %-6d  %12s  %12.1f  %12.1f  %10.1f  %8s\n', ...
                ctrl_labels{ic}, N_list(iN), dt_list_ms(iDt), ts_str, ...
                ctrl_med_us(ic, iN, iDt), ctrl_max_us(ic, iN, iDt), ...
                qp_med_us(ic, iN, iDt), rt_str);
        end
    end
    fprintf('%s\n', repmat('-', 1, 100));
end

%% --- Marginal improvement table ---
fprintf('\n--- Marginal improvement in settling time (%) ---\n');
trans_labels = {'10->20', '20->30', '30->40', '40->50'};
fprintf('%-12s  %-6s  %10s  %10s  %10s  %10s\n', 'Controller', 'dt(ms)', trans_labels{:});
fprintf('%s\n', repmat('-', 1, 62));
for ic = 1:nC
    for iDt = 1:nDt
        vals = squeeze(marginal_pct(ic, :, iDt));
        val_strs = arrayfun(@(v) ternary(isnan(v), '  N/A', sprintf('%+.1f%%', v)), vals, ...
                            'UniformOutput', false);
        fprintf('%-12s  %-6d  %10s  %10s  %10s  %10s\n', ...
            ctrl_labels{ic}, dt_list_ms(iDt), val_strs{:});
    end
end

%% --- Save CSV ---
csv_path = fullfile(fig_dir, 'summary_table.csv');
fid = fopen(csv_path, 'w');
fprintf(fid, 'Controller,N,dt_ms,T_settle_ms,CtrlMed_us,CtrlMax_us,QPMed_us,MeetsRT,Diverged\n');
for ic = 1:nC
    for iN = 1:nN
        for iDt = 1:nDt
            fprintf(fid, '%s,%d,%d,%.4f,%.2f,%.2f,%.2f,%d,%d\n', ...
                ctrl_labels{ic}, N_list(iN), dt_list_ms(iDt), ...
                settle_ms(ic,iN,iDt), ctrl_med_us(ic,iN,iDt), ...
                ctrl_max_us(ic,iN,iDt), qp_med_us(ic,iN,iDt), ...
                meets_rt(ic,iN,iDt), diverged_map(ic,iN,iDt));
        end
    end
end
fclose(fid);
fprintf('\nCSV saved: %s\n', csv_path);

%% ================================================================
%% Figure style constants
%% ================================================================

W_FULL = 17;   % cm, double-column A4
W_HALF = 8.5;
H_STD  = 7;
H_TALL = 10;

FONT_NAME  = 'Times New Roman';
FONT_AX    = 9;
FONT_LAB   = 10;
FONT_LEG   = 8;
FONT_TITLE = 10;

% One color per controller — distinguishable, printer-friendly
C_NMPC   = [0.00 0.45 0.74];   % blue
C_LMPC   = [0.85 0.33 0.10];   % orange-red
C_SOLMPC = [0.47 0.67 0.19];   % green
CTRL_COLORS = [C_NMPC; C_LMPC; C_SOLMPC];

DT_STYLES  = {'-', '--', ':'};
DT_MARKERS = {'o', 's', '^'};

%% ================================================================
%% Figure 1: Settling time heatmap (one subplot per controller)
%% ================================================================
fprintf('\n--- Figure 1: Settling time heatmap ---\n');

% Shared color axis across all controllers (excluding NaN/diverged)
all_settle = settle_ms(:);
valid_settle = all_settle(isfinite(all_settle) & all_settle > 0);
clim_settle = [0, max(valid_settle) * 1.05];

fig1 = mkfig(W_FULL, H_STD);

for ic = 1:nC
    ax = subplot(1, 3, ic); hold on;

    slice = squeeze(settle_ms(ic, :, :));          % nN x nDt
    div_slice = squeeze(diverged_map(ic, :, :));

    % Replace diverged/NaN with a sentinel for imagesc (will be masked)
    slice_plot = slice;
    slice_plot(~isfinite(slice_plot)) = 0;

    h = imagesc(1:nDt, 1:nN, slice_plot);
    % Mask diverged/NaN cells with gray
    gray_overlay = ~isfinite(slice) | div_slice;
    if any(gray_overlay(:))
        for iN2 = 1:nN
            for iDt2 = 1:nDt
                if gray_overlay(iN2, iDt2)
                    rectangle('Position', [iDt2-0.5, iN2-0.5, 1, 1], ...
                              'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
                end
            end
        end
    end

    % Mark diverged with X
    for iN2 = 1:nN
        for iDt2 = 1:nDt
            if div_slice(iN2, iDt2)
                text(iDt2, iN2, '\bfX', 'HorizontalAlignment', 'center', ...
                     'VerticalAlignment', 'middle', 'Color', 'w', ...
                     'FontSize', 11, 'FontName', FONT_NAME);
            elseif ~isfinite(slice(iN2, iDt2))
                text(iDt2, iN2, '?', 'HorizontalAlignment', 'center', ...
                     'VerticalAlignment', 'middle', 'Color', [0.3 0.3 0.3], ...
                     'FontSize', 9, 'FontName', FONT_NAME);
            else
                text(iDt2, iN2, sprintf('%.0f', slice(iN2, iDt2)), ...
                     'HorizontalAlignment', 'center', ...
                     'VerticalAlignment', 'middle', 'Color', 'k', ...
                     'FontSize', 8, 'FontName', FONT_NAME);
            end
        end
    end

    colormap(ax, parula);
    clim(clim_settle);
    set(ax, 'YDir', 'normal', ...
            'XTick', 1:nDt, 'XTickLabel', arrayfun(@(d) sprintf('%d ms', d), dt_list_ms, 'UniformOutput', false), ...
            'YTick', 1:nN,  'YTickLabel', arrayfun(@(n) sprintf('%d', n),    N_list,     'UniformOutput', false), ...
            'FontName', FONT_NAME, 'FontSize', FONT_AX);
    xlabel('\Delta{}t_{mpc}', 'FontSize', FONT_LAB, 'FontName', FONT_NAME);
    if ic == 1
        ylabel('Horizon N', 'FontSize', FONT_LAB, 'FontName', FONT_NAME);
    end
    title(ctrl_labels{ic}, 'FontSize', FONT_TITLE, 'FontName', FONT_NAME, 'FontWeight', 'bold');
    if ic == nC
        cb = colorbar;
        cb.Label.String = 'Settling time (ms)';
        cb.Label.FontName = FONT_NAME;
        cb.Label.FontSize = FONT_AX;
    end
    style_ax(ax, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig1, 'settle_heatmap', fig_dir);

%% ================================================================
%% Figure 2: Timing bar chart (one subplot per dt_mpc)
%% ================================================================
fprintf('--- Figure 2: Timing comparison bars ---\n');

fig2 = mkfig(W_FULL, H_STD);

for iDt = 1:nDt
    ax = subplot(1, 3, iDt); hold on; grid on; box on;

    % bar_data: nN x nC  (grouped: each N has 3 bars)
    bar_data = squeeze(ctrl_med_us(:, :, iDt))';  % nN x nC

    b = bar(N_list, bar_data, 'grouped');
    for ic = 1:nC
        b(ic).FaceColor = CTRL_COLORS(ic, :);
        b(ic).EdgeColor = 'none';
        b(ic).DisplayName = ctrl_labels{ic};
    end

    % Real-time budget line
    budget_us = dt_list_ms(iDt) * 1000;
    yl = yline(budget_us, 'r--', 'LineWidth', 1.2);
    yl.Label = sprintf('RT budget (%d ms)', dt_list_ms(iDt));
    yl.FontSize = FONT_LEG;
    yl.FontName = FONT_NAME;
    yl.LabelHorizontalAlignment = 'left';
    yl.HandleVisibility = 'off';

    xlabel('Horizon N', 'FontSize', FONT_LAB, 'FontName', FONT_NAME);
    if iDt == 1
        ylabel('Median controller time (\mus)', 'FontSize', FONT_LAB, 'FontName', FONT_NAME);
        legend(ctrl_labels, 'Location', 'northwest', 'FontSize', FONT_LEG, ...
               'FontName', FONT_NAME);
    end
    title(sprintf('\\Delta{}t_{mpc} = %d ms', dt_list_ms(iDt)), ...
          'FontSize', FONT_TITLE, 'FontName', FONT_NAME, 'FontWeight', 'bold');
    style_ax(ax, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig2, 'timing_bars', fig_dir);

%% ================================================================
%% Figure 3: Settling time vs N curves (one subplot per controller)
%% ================================================================
fprintf('--- Figure 3: Settling time vs N ---\n');

fig3 = mkfig(W_FULL, H_STD);

for ic = 1:nC
    ax = subplot(1, 3, ic); hold on; grid on; box on;

    for iDt = 1:nDt
        ts   = squeeze(settle_ms(ic, :, iDt));       % 1 x nN
        div  = squeeze(diverged_map(ic, :, iDt));     % 1 x nN logical
        valid = ~isnan(ts) & ~div;

        if any(valid)
            plot(N_list(valid), ts(valid), ...
                 [DT_STYLES{iDt} DT_MARKERS{iDt}], ...
                 'Color', CTRL_COLORS(ic, :), ...
                 'LineWidth', 1.2, 'MarkerSize', 5, ...
                 'MarkerFaceColor', CTRL_COLORS(ic, :), ...
                 'DisplayName', sprintf('%d ms', dt_list_ms(iDt)));
        end

        % Mark diverged runs with an X at y=0
        div_N = N_list(div);
        if ~isempty(div_N)
            plot(div_N, zeros(1, numel(div_N)), 'rx', ...
                 'MarkerSize', 8, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
    end

    xlabel('Horizon N', 'FontSize', FONT_LAB, 'FontName', FONT_NAME);
    if ic == 1
        ylabel('Settling time (ms)', 'FontSize', FONT_LAB, 'FontName', FONT_NAME);
        legend('Location', 'northeast', 'FontSize', FONT_LEG, 'FontName', FONT_NAME);
    end
    title(ctrl_labels{ic}, 'FontSize', FONT_TITLE, 'FontName', FONT_NAME, 'FontWeight', 'bold');
    style_ax(ax, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig3, 'settle_vs_N', fig_dir);

%% ================================================================
%% Figure 4: Marginal improvement heatmap
%% ================================================================
fprintf('--- Figure 4: Marginal improvement heatmap ---\n');

% Build (nC*nDt) x (nN-1) matrix, rows ordered: NMPC 1ms, NMPC 2ms, ...
n_rows = nC * nDt;
marg_matrix = nan(n_rows, nN-1);
row_labels  = cell(n_rows, 1);
row = 0;
for ic = 1:nC
    for iDt = 1:nDt
        row = row + 1;
        row_labels{row} = sprintf('%s %d ms', ctrl_labels{ic}, dt_list_ms(iDt));
        marg_matrix(row, :) = squeeze(marginal_pct(ic, :, iDt));
    end
end

% Diverging colormap: green (0%) → white → red (high %)
% Values near 0 = diminishing returns (good, green)
% Values near 50%+ = still benefiting strongly from longer horizon (red)
n_cmap = 256;
cmap_lo = [linspace(0.1, 1, n_cmap/2)', linspace(0.6, 1, n_cmap/2)', linspace(0.1, 1, n_cmap/2)'];  % green → white
cmap_hi = [linspace(1, 0.9, n_cmap/2)', linspace(1, 0.1, n_cmap/2)', linspace(1, 0.1, n_cmap/2)'];  % white → red
diverging_cmap = [cmap_lo; cmap_hi];

col_labels = {'N: 10\rightarrow{}20', '20\rightarrow{}30', '30\rightarrow{}40', '40\rightarrow{}50'};

fig4 = mkfig(W_FULL, H_TALL);
ax = axes; hold on;

% Clip NaN to a neutral display value for imagesc (gray it out manually)
marg_display = marg_matrix;
marg_display(isnan(marg_display)) = 0;

imagesc(1:nN-1, 1:n_rows, marg_display);
colormap(ax, diverging_cmap);
clim([0, 50]);
cb = colorbar;
cb.Label.String = 'Improvement in settling time (%)';
cb.Label.FontName = FONT_NAME;
cb.Label.FontSize = FONT_AX;

% Gray out NaN cells
for r = 1:n_rows
    for c = 1:nN-1
        if isnan(marg_matrix(r, c))
            rectangle('Position', [c-0.5, r-0.5, 1, 1], ...
                      'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none');
            text(c, r, 'N/A', 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'middle', 'Color', [0.4 0.4 0.4], ...
                 'FontSize', 7, 'FontName', FONT_NAME);
        else
            text(c, r, sprintf('%.1f%%', marg_matrix(r, c)), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                 'Color', 'k', 'FontSize', 7.5, 'FontName', FONT_NAME);
        end
    end
end

% Add horizontal separator lines between controllers
for ic = 1:nC-1
    yline(ic*nDt + 0.5, 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

set(ax, 'YDir', 'normal', ...
        'XTick', 1:nN-1, 'XTickLabel', col_labels, ...
        'YTick', 1:n_rows, 'YTickLabel', row_labels, ...
        'FontName', FONT_NAME, 'FontSize', FONT_AX, ...
        'TickLength', [0 0]);
xlabel('Horizon transition', 'FontSize', FONT_LAB, 'FontName', FONT_NAME);
title('Marginal improvement in settling time per horizon increment', ...
      'FontSize', FONT_TITLE, 'FontName', FONT_NAME, 'FontWeight', 'bold');
style_ax(ax, FONT_NAME, FONT_AX, FONT_LAB);

savefig_thesis(fig4, 'marginal_heatmap', fig_dir);

%% ================================================================
%% Done
%% ================================================================
fprintf('\n=== All figures exported to %s ===\n', fig_dir);

%% ================================================================
%% Local functions
%% ================================================================

function ts_ms = computeSettlingTime(d, z_idx)
%COMPUTESETTLINGTIME  2% settling time on z-axis.
%   z_idx: 1-based index into state vector (3 for z)
    if d.diverged
        ts_ms = NaN;
        return;
    end

    z = d.x(z_idx, :);
    t = d.t;

    % Final value: mean of last 10 samples (tolerant of model mismatch offset)
    n_end = min(10, length(z));
    z_final = mean(z(end-n_end+1:end));
    z0 = z(1);

    total_excursion = abs(z0 - z_final);
    band = 0.02 * total_excursion;
    band = max(band, 1e-5);  % 10 um floor

    % Find last sample outside the 2% band
    outside = abs(z - z_final) > band;

    if ~any(outside)
        ts_ms = 0;  % already settled at t=0
        return;
    end

    last_out_idx = find(outside, 1, 'last');

    if last_out_idx >= length(t)
        ts_ms = NaN;  % never fully settled
        return;
    end

    ts_ms = t(last_out_idx + 1) * 1e3;
end

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
    if ~isempty(ax.Title.String)
        set(ax.Title, 'FontSize', fs_lab, 'FontWeight', 'bold', 'FontName', fn);
    end
end

function savefig_thesis(fig, name, out_dir)
    pdf_path = fullfile(out_dir, [name '.pdf']);
    exportgraphics(fig, pdf_path, 'ContentType', 'vector');
    fprintf('  Saved: %s\n', pdf_path);
    % Also save PNG as fallback (e.g. heatmaps render better as raster in some viewers)
    png_path = fullfile(out_dir, [name '.png']);
    exportgraphics(fig, png_path, 'Resolution', 300);
end

function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
