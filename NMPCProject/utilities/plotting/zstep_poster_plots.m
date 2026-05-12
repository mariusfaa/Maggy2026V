%% zstep_poster_plots - Poster plots for the z-step controller comparison
%
% Reads the fixed N=20, dt=1 ms results from results_zstep and creates:
%   1) vertical-position trajectory comparison
%   2) x-axis tilt-angle trajectory comparison
%   3) final cumulative cost bar chart
%
% Usage:
%   cd /path/to/NMPCProject
%   run('utilities/plotting/zstep_poster_plots.m')
%
% Optional export:
%   export_figures = true;
%   run('utilities/plotting/zstep_poster_plots.m')

if ~exist('export_figures', 'var')
    export_figures = true;
end

clc; close all;

%% Paths and configuration

project_root = getenv('ACADOS_PROJECT_DIR');
if isempty(project_root)
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
end

addpath(genpath(fullfile(project_root, 'utilities')));

res_dir = fullfile(project_root, 'results_zstep');
out_dir = fullfile(project_root, 'figures', 'poster');

controllers = {'lqr', 'lmpc', 'solnmpc', 'nmpc'};
file_names = {
    'lqr_1ms_N20.mat'
    'lmpc_1ms_N20.mat'
    'solnmpc_1ms_N20.mat'
    'nmpc_1ms_N20.mat'
};

% controllers = {'lmpc', 'solnmpc', 'nmpc'};
% file_names = {
%     'lmpc_1ms_N25.mat'
%     'solnmpc_1ms_N25.mat'
%     'nmpc_1ms_N25.mat'
% };

save_png = true;

if export_figures && ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

if ~exist(res_dir, 'dir')
    error('Result folder not found: %s', res_dir);
end

%% Style

style = thesisPlotStyle();
applyThesisDefaults(style);
ctrl_colors = thesisControllerColors();

%% Load data

data = cell(1, numel(controllers));
for i = 1:numel(controllers)
    f = fullfile(res_dir, file_names{i});
    if ~isfile(f)
        error('Missing z-step result file: %s', f);
    end

    d = load(f);
    d.file = f;

    if ~isfield(d, 'controller') || isempty(d.controller)
        d.controller = controllers{i};
    end
    if strcmp(d.controller, 'solmpc')
        d.controller = 'solnmpc';
    end

    assert(isfield(d, 't') && isfield(d, 'x'), ...
        'Result file must contain t and x: %s', f);

    data{i} = d;
end

T_end = min(cellfun(@(d) d.t(end), data));

%% Figure 1: vertical position trajectory

fig_z = figure('Name', 'Vertical motion tracking', ...
    'Units', 'normalized', 'Position', [0.12 0.18 0.62 0.48]);
hold on; grid on; box on;

for i = 1:numel(data)
    r = data{i};
    idx = getPlotIdx(r, T_end);
    c = ctrl_colors(r.controller);

    plot(r.t(idx) * 1e3, r.x(3, idx) * 1e3, ...
        'Color', c, ...
        'LineWidth', style.LineWidth, ...
        'DisplayName', upperControllerName(r.controller));
end

z_eq = getReferenceZ(data{1});
if ~isnan(z_eq)
    yline(z_eq * 1e3, 'k:', ...
        'LineWidth', style.ThinLineWidth, ...
        'DisplayName', 'Reference');
end

xlabel('Time (ms)');
ylabel('z (mm)');
title('Vertical position response');
legend('Location', 'best', 'FontSize', style.LegendFontSize);
xlim([0, T_end * 1e3]);

formatThesisFigure(fig_z, 'wide', style);
savePosterFigure(fig_z, out_dir, 'poster_vertical_position_response', export_figures, save_png);

%% Figure 2: x-axis tilt trajectory

fig_phi = figure('Name', 'Tilt-angle regulation', ...
    'Units', 'normalized', 'Position', [0.14 0.18 0.62 0.48]);
hold on; grid on; box on;

for i = 1:numel(data)
    r = data{i};
    idx = getPlotIdx(r, T_end);
    c = ctrl_colors(r.controller);

    plot(r.t(idx) * 1e3, r.x(4, idx) * 180 / pi, ...
        'Color', c, ...
        'LineWidth', style.LineWidth, ...
        'DisplayName', upperControllerName(r.controller));
end

phi_ref = getReferenceState(data{1}, 4);
if ~isnan(phi_ref)
    yline(phi_ref * 180 / pi, 'k:', ...
        'LineWidth', style.ThinLineWidth, ...
        'DisplayName', 'Reference');
end

xlabel('Time (ms)');
ylabel('\phi (deg)');
title('Tilt-angle regulation');
legend('Location', 'best', 'FontSize', style.LegendFontSize);
xlim([0, T_end * 1e3]);

formatThesisFigure(fig_phi, 'wide', style);
savePosterFigure(fig_phi, out_dir, 'poster_tilt_angle_regulation', export_figures, save_png);

%% Figure 3: final cumulative cost

final_cost = nan(1, numel(data));
labels = cell(1, numel(data));
bar_colors = nan(numel(data), 3);

for i = 1:numel(data)
    r = data{i};
    final_cost(i) = getFinalCumulativeCost(r);
    labels{i} = upperControllerName(r.controller);
    bar_colors(i, :) = ctrl_colors(r.controller);
end

fig_cost = figure('Name', 'Control effort and tracking cost', ...
    'Units', 'normalized', 'Position', [0.15 0.2 0.48 0.42]);
hold on; grid on; box on;

b = bar(final_cost, 'FaceColor', 'flat');
b.CData = bar_colors;

ylim([0, max(final_cost)*1.1]);

set(gca, 'XTick', 1:numel(labels), 'XTickLabel', labels);
ylabel('Final cumulative cost');
title('Cumulative closed-loop cost');

for i = 1:numel(final_cost)
    if isfinite(final_cost(i))
        text(i, final_cost(i), sprintf('%.3g', final_cost(i)), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'FontSize', style.LegendFontSize);
    end
end

formatThesisFigure(fig_cost, 'single', style);
savePosterFigure(fig_cost, out_dir, 'poster_cumulative_closed_loop_cost', export_figures, save_png);

%% Print summary

fprintf('\nZ-step poster plots\n');
fprintf('Results: %s\n', res_dir);
fprintf('Output:  %s\n\n', out_dir);
fprintf('%-10s  %-12s  %-12s  %-12s\n', 'Controller', 'z_final_mm', 'phi_final_deg', 'final_cost');
for i = 1:numel(data)
    r = data{i};
    fprintf('%-10s  %-12.5g  %-12.5g  %-12.5g\n', ...
        upperControllerName(r.controller), ...
        r.x(3, end) * 1e3, ...
        r.x(4, end) * 180 / pi, ...
        final_cost(i));
end

%% Local helpers

function idx = getPlotIdx(r, T_end)
    n = min(numel(r.t), size(r.x, 2));
    t = r.t(1:n);
    idx = find(t <= T_end + 10 * eps(max(1, abs(T_end))));
    if isempty(idx)
        idx = 1:n;
    end
end

function z_ref = getReferenceZ(r)
    z_ref = getReferenceState(r, 3);
end

function x_ref_i = getReferenceState(r, state_idx)
    x_ref_i = NaN;
    ref_fields = {'xRef', 'x_ref', 'xref', 'x_ref_traj', 'xTarget', 'x_target'};

    for k = 1:numel(ref_fields)
        name = ref_fields{k};
        if isfield(r, name)
            val = r.(name);
            if isnumeric(val) && size(val, 1) >= state_idx
                x_ref_i = val(state_idx, end);
                return;
            end
        end
    end

    if isfield(r, 'xEq') && isnumeric(r.xEq) && numel(r.xEq) >= state_idx
        x_ref_i = r.xEq(state_idx);
    end
end

function J = getFinalCumulativeCost(r)
    if isfield(r, 'cost_cum') && ~isempty(r.cost_cum)
        J = r.cost_cum(end);
    elseif isfield(r, 'cost') && ~isempty(r.cost)
        J = sum(r.cost);
    else
        J = NaN;
    end
end

function label = upperControllerName(controller)
    if strcmp(controller, 'solnmpc')
        label = 'SOL-NMPC';
    else
        label = upper(controller);
    end
end

function savePosterFigure(fig, out_dir, name, do_save, save_png)
    if ~do_save
        return;
    end

    drawnow;

    pdf_file = fullfile(out_dir, [name '.pdf']);
    exportgraphics(fig, pdf_file, 'ContentType', 'vector');
    fprintf('Saved: %s\n', pdf_file);

    if save_png
        png_file = fullfile(out_dir, [name '.png']);
        exportgraphics(fig, png_file, 'ContentType', 'image', 'Resolution', 300);
        fprintf('Saved: %s\n', png_file);
    end
end
