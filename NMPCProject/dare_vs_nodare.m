%% dare_vs_nodare — Compare DARE terminal cost vs no terminal cost
%
% Loads matching result files from results_dare/ and results_nodare/
% and produces thesis-quality comparison plots:
%   Fig 1: State trajectories (z-axis) side-by-side per controller
%   Fig 2: Cumulative cost vs N (bar chart, DARE vs no-DARE)
%   Fig 3: Settling time vs N (grouped bar chart)
%   Fig 4: Input trajectories (all 4 solenoids) per controller at N=15
%
% Usage:
%   dare_vs_nodare

clear; clc; close all;

%% ================================================================
%  PATHS
%  ================================================================

project_root = getenv('ACADOS_PROJECT_DIR');
if isempty(project_root), project_root = pwd; end
addpath(genpath(fullfile(project_root, 'utilities')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'model_matlab')));

dir_dare   = fullfile(project_root, 'results_dare');
dir_nodare = fullfile(project_root, 'results_nodare');

%% ================================================================
%  CONFIGURATION
%  ================================================================

controllers  = {'lmpc', 'solnmpc', 'nmpc'};
ctrl_labels  = {'LMPC', 'SolNMPC', 'NMPC'};
N_list       = [10, 20, 30];
dt_mpc       = 0.001;   % 1 ms

% Export options
save_pdf = false;
out_dir  = fullfile(project_root, 'figures', 'terminalcost');
if save_pdf && ~exist(out_dir, 'dir'), mkdir(out_dir); end

%% ================================================================
%  EQUILIBRIUM
%  ================================================================

parameters_maggy_V4;
modelId = MaglevModel.Accurate;
[zEq, ~, ~, ~] = computeSystemEquilibria(params, modelId);
xEq = [0; 0; zEq(1); zeros(7,1)];
uEq = zeros(4, 1);
nx = 10; nu = 4;

%% ================================================================
%  STYLE
%  ================================================================

ctrl_colors = containers.Map();
ctrl_colors('lmpc')   = [0.0  0.45 0.74];   % blue
ctrl_colors('solnmpc') = [0.85 0.33 0.1 ];   % orange
ctrl_colors('nmpc')   = [0.47 0.67 0.19];   % green

state_names = {'x','y','z','\phi','\theta', ...
               '\dot{x}','\dot{y}','\dot{z}','\dot{\phi}','\dot{\theta}'};
state_units = {'mm','mm','mm','deg','deg','m/s','m/s','m/s','rad/s','rad/s'};
state_scale = [1e3, 1e3, 1e3, 180/pi, 180/pi, 1, 1, 1, 1, 1];

% DARE vs no-DARE line styles
STYLE_DARE   = '-';    % solid
STYLE_NODARE = '--';   % dashed

nC = numel(controllers);
nN = numel(N_list);

%% ================================================================
%  LOAD DATA
%  ================================================================

fprintf('Loading results...\n');

data_dare   = cell(nC, nN);
data_nodare = cell(nC, nN);
missing = {};

for ic = 1:nC
    for iN = 1:nN
        fname = getFilename(controllers{ic}, N_list(iN), dt_mpc);

        f_dare = fullfile(dir_dare, fname);
        if isfile(f_dare)
            data_dare{ic, iN} = load(f_dare);
        else
            missing{end+1} = sprintf('  DARE:    %s', fname); %#ok<AGROW>
        end

        f_nodare = fullfile(dir_nodare, fname);
        if isfile(f_nodare)
            data_nodare{ic, iN} = load(f_nodare);
        else
            missing{end+1} = sprintf('  no-DARE: %s', fname); %#ok<AGROW>
        end
    end
end

if ~isempty(missing)
    fprintf('\n--- MISSING result files ---\n');
    for i = 1:numel(missing)
        fprintf('%s\n', missing{i});
    end
    fprintf('----------------------------\n\n');
end
fprintf('Done loading.\n\n');

%% ================================================================
%  COMPUTE METRICS
%  ================================================================

settle_dare   = nan(nC, nN);
settle_nodare = nan(nC, nN);
cost_dare     = nan(nC, nN);
cost_nodare   = nan(nC, nN);

for ic = 1:nC
    for iN = 1:nN
        d = data_dare{ic, iN};
        if ~isempty(d) && ~d.diverged
            settle_dare(ic, iN) = computeSettlingTime(d, 3);
            cost_dare(ic, iN)   = d.cost_cum(end);
        end
        d = data_nodare{ic, iN};
        if ~isempty(d) && ~d.diverged
            settle_nodare(ic, iN) = computeSettlingTime(d, 3);
            cost_nodare(ic, iN)   = d.cost_cum(end);
        end
    end
end

%% ================================================================
%  CONSOLE SUMMARY
%  ================================================================

fprintf('%-8s  %-4s  %14s  %14s  %12s  %12s\n', ...
    'Ctrl', 'N', 'T_s DARE(ms)', 'T_s noDARE(ms)', 'Cost DARE', 'Cost noDARE');
fprintf('%s\n', repmat('-', 1, 75));
for ic = 1:nC
    for iN = 1:nN
        fprintf('%-8s  N=%-2d  %14.2f  %14.2f  %12.4g  %12.4g\n', ...
            ctrl_labels{ic}, N_list(iN), ...
            settle_dare(ic, iN), settle_nodare(ic, iN), ...
            cost_dare(ic, iN), cost_nodare(ic, iN));
    end
end

%% ================================================================
%% FIG 1: z-trajectory comparison (one subplot per controller)
%% ================================================================

fprintf('\n--- Figure 1: z-axis trajectories ---\n');

fig1 = figure('Name','z-axis: DARE vs no-DARE', ...
    'Units','normalized','Position',[0.05 0.15 0.9 0.35]);

for ic = 1:nC
    subplot(1, nC, ic); hold on; grid on; box on;
    col = ctrl_colors(controllers{ic});
    leg_entries = {};

    for iN = 1:nN
        % Shade: lighter for smaller N, darker for larger N
        shade = 0.3 + 0.7 * (iN - 1) / max(nN - 1, 1);
        col_shade = 1 - shade * (1 - col);  % interpolate toward white

        d = data_dare{ic, iN};
        if ~isempty(d) && ~d.diverged
            plot(d.t*1e3, (d.x(3,:) - xEq(3))*1e3, STYLE_DARE, ...
                'Color', col_shade, 'LineWidth', 1.4, ...
                'DisplayName', sprintf('N=%d DARE', N_list(iN)));
        end
        d = data_nodare{ic, iN};
        if ~isempty(d) && ~d.diverged
            plot(d.t*1e3, (d.x(3,:) - xEq(3))*1e3, STYLE_NODARE, ...
                'Color', col_shade, 'LineWidth', 1.4, ...
                'DisplayName', sprintf('N=%d no-DARE', N_list(iN)));
        end
    end

    xlabel('Time (ms)');
    if ic == 1, ylabel('\Deltaz (mm)'); end
    title(ctrl_labels{ic}, 'FontWeight', 'bold');
    if ic == nC, legend('Location','best','FontSize',7); end
end

sgtitle(sprintf('z-axis deviation: DARE vs no terminal cost — \\Deltat_{mpc}=%.0f\\mus', ...
    dt_mpc*1e6), 'FontSize', 12);
saveFig(fig1, out_dir, 'dare_z_trajectories', save_pdf);

%% ================================================================
%% FIG 2: Final cumulative cost — grouped bar (DARE vs no-DARE)
%% ================================================================

fprintf('--- Figure 2: Final cumulative cost ---\n');

fig2 = figure('Name','Cost: DARE vs no-DARE', ...
    'Units','normalized','Position',[0.15 0.2 0.55 0.4]);

% Build bar data: rows = N values, columns = controllers*2 (dare/nodare interleaved)
% We'll use grouped bars: groups = N, bars = controller, with DARE/noDARE side-by-side

% Simpler layout: one subplot per controller
for ic = 1:nC
    subplot(1, nC, ic); hold on; grid on; box on;

    bar_data = [squeeze(cost_dare(ic,:))', squeeze(cost_nodare(ic,:))'];
    b = bar(N_list, bar_data, 'grouped');

    col = ctrl_colors(controllers{ic});
    b(1).FaceColor = col;
    b(1).EdgeColor = 'none';
    b(1).DisplayName = 'DARE';
    b(2).FaceColor = col * 0.5 + 0.5;  % lighter version
    b(2).EdgeColor = col;
    b(2).LineWidth = 1.2;
    b(2).DisplayName = 'No terminal cost';

    xlabel('N_{horizon}');
    if ic == 1, ylabel('Final cumulative cost'); end
    title(ctrl_labels{ic}, 'FontWeight', 'bold');
    if ic == 1, legend('Location', 'best', 'FontSize', 8); end

    set(gca, 'XTick', N_list);
end

sgtitle(sprintf('Final cumulative cost — \\Deltat_{mpc}=%.0f\\mus', dt_mpc*1e6), 'FontSize', 12);
saveFig(fig2, out_dir, 'dare_cost_comparison', save_pdf);

%% ================================================================
%% FIG 3: Settling time — grouped bar (DARE vs no-DARE)
%% ================================================================

fprintf('--- Figure 3: Settling time ---\n');

fig3 = figure('Name','Settling time: DARE vs no-DARE', ...
    'Units','normalized','Position',[0.15 0.2 0.55 0.4]);

for ic = 1:nC
    subplot(1, nC, ic); hold on; grid on; box on;

    bar_data = [squeeze(settle_dare(ic,:))', squeeze(settle_nodare(ic,:))'];
    b = bar(N_list, bar_data, 'grouped');

    col = ctrl_colors(controllers{ic});
    b(1).FaceColor = col;
    b(1).EdgeColor = 'none';
    b(1).DisplayName = 'DARE';
    b(2).FaceColor = col * 0.5 + 0.5;
    b(2).EdgeColor = col;
    b(2).LineWidth = 1.2;
    b(2).DisplayName = 'No terminal cost';

    xlabel('N_{horizon}');
    if ic == 1, ylabel('Settling time (ms)'); end
    title(ctrl_labels{ic}, 'FontWeight', 'bold');
    if ic == 1, legend('Location', 'best', 'FontSize', 8); end

    set(gca, 'XTick', N_list);
end

sgtitle(sprintf('Settling time (2%% criterion on z) — \\Deltat_{mpc}=%.0f\\mus', ...
    dt_mpc*1e6), 'FontSize', 12);
saveFig(fig3, out_dir, 'dare_settling_time', save_pdf);

%% ================================================================
%% FIG 4: Input comparison at smallest N (most visible difference)
%% ================================================================

fprintf('--- Figure 4: Input trajectories at N=%d ---\n', N_list(1));

fig4 = figure('Name','Inputs: DARE vs no-DARE', ...
    'Units','normalized','Position',[0.05 0.1 0.9 0.55]);

iN_show = 1;  % smallest horizon — where terminal cost has the biggest impact

for ic = 1:nC
    col = ctrl_colors(controllers{ic});
    for s = 1:nu
        ax_idx = (ic-1)*nu + s;
        subplot(nC, nu, ax_idx); hold on; grid on; box on;

        d = data_dare{ic, iN_show};
        if ~isempty(d) && ~d.diverged
            plot(d.t*1e3, d.u(s,:), STYLE_DARE, 'Color', col, ...
                'LineWidth', 1.2, 'DisplayName', 'DARE');
        end
        d = data_nodare{ic, iN_show};
        if ~isempty(d) && ~d.diverged
            plot(d.t*1e3, d.u(s,:), STYLE_NODARE, 'Color', col, ...
                'LineWidth', 1.2, 'DisplayName', 'No terminal cost');
        end

        ylim([-1.1 1.1]);
        if ic == nC, xlabel('Time (ms)'); end
        if s == 1, ylabel(sprintf('%s u (A)', ctrl_labels{ic})); end
        if ic == 1, title(sprintf('u_%d', s)); end
        if ic == 1 && s == 1, legend('Location','best','FontSize',7); end
    end
end

sgtitle(sprintf('Input trajectories — N=%d, \\Deltat_{mpc}=%.0f\\mus', ...
    N_list(iN_show), dt_mpc*1e6), 'FontSize', 12);
saveFig(fig4, out_dir, 'dare_inputs', save_pdf);

%% ================================================================
%% FIG 5: Cumulative cost over time — overlay DARE vs no-DARE
%% ================================================================

fprintf('--- Figure 5: Cumulative cost over time ---\n');

fig5 = figure('Name','Cum. cost over time: DARE vs no-DARE', ...
    'Units','normalized','Position',[0.05 0.15 0.9 0.35]);

for ic = 1:nC
    subplot(1, nC, ic); hold on; grid on; box on;
    col = ctrl_colors(controllers{ic});

    for iN = 1:nN
        shade = 0.3 + 0.7 * (iN - 1) / max(nN - 1, 1);
        col_shade = 1 - shade * (1 - col);

        d = data_dare{ic, iN};
        if ~isempty(d) && ~d.diverged
            plot(d.t*1e3, d.cost_cum, STYLE_DARE, 'Color', col_shade, ...
                'LineWidth', 1.4, 'DisplayName', sprintf('N=%d DARE', N_list(iN)));
        end
        d = data_nodare{ic, iN};
        if ~isempty(d) && ~d.diverged
            plot(d.t*1e3, d.cost_cum, STYLE_NODARE, 'Color', col_shade, ...
                'LineWidth', 1.4, 'DisplayName', sprintf('N=%d no-DARE', N_list(iN)));
        end
    end

    xlabel('Time (ms)');
    if ic == 1, ylabel('Cumulative cost'); end
    title(ctrl_labels{ic}, 'FontWeight', 'bold');
    if ic == nC, legend('Location','best','FontSize',7); end
end

sgtitle(sprintf('Cumulative cost evolution — \\Deltat_{mpc}=%.0f\\mus', ...
    dt_mpc*1e6), 'FontSize', 12);
saveFig(fig5, out_dir, 'dare_cost_evolution', save_pdf);

fprintf('\n=== Done. ===\n');
if save_pdf
    fprintf('Figures saved to: %s\n', out_dir);
end

%% ================================================================
%% LOCAL FUNCTIONS
%% ================================================================

function ts_ms = computeSettlingTime(d, z_idx)
%COMPUTESETTLINGTIME  2% settling time on a given state index.
    if d.diverged
        ts_ms = NaN;
        return;
    end

    z = d.x(z_idx, :);
    t = d.t;

    % Final value: mean of last 10 samples
    n_end = min(10, length(z));
    z_final = mean(z(end-n_end+1:end));
    z0 = z(1);

    total_excursion = abs(z0 - z_final);
    band = 0.02 * total_excursion;
    band = max(band, 1e-5);  % 10 um floor

    outside = abs(z - z_final) > band;
    if ~any(outside)
        ts_ms = 0;
        return;
    end

    last_out_idx = find(outside, 1, 'last');
    if last_out_idx >= length(t)
        ts_ms = NaN;
        return;
    end
    ts_ms = t(last_out_idx + 1) * 1e3;
end

function saveFig(fig, out_dir, name, do_save)
%SAVEFIG  Export figure as PDF + PNG if saving is enabled.
    if ~do_save, return; end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    pdf_path = fullfile(out_dir, [name '.pdf']);
    exportgraphics(fig, pdf_path, 'ContentType', 'vector');
    fprintf('  Saved: %s\n', pdf_path);
    png_path = fullfile(out_dir, [name '.png']);
    exportgraphics(fig, png_path, 'Resolution', 300);
end
