%% magnetFieldComparison — Compare wire-loop vs current-sheet field models
%
% Generates thesis-quality figures showing the 2D magnetic field of a
% single permanent magnet in the x-z plane (y=0 slice, Cartesian):
%   Fig 1: Side-by-side |B| heatmap + streamlines (wire vs current-sheet)
%   Fig 2: Difference heatmap |B_sheet - B_wire|
%
% Usage:
%   magnetFieldComparison

clear; clc; close all;

%% ================================================================
%  Paths
%  ================================================================
project_root = getenv('ACADOS_PROJECT_DIR');
if isempty(project_root), project_root = pwd; end

addpath(genpath(fullfile(project_root, 'model_matlab')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

%% ================================================================
%  Export options
%  ================================================================
save_figs = true;
fig_dir   = fullfile(project_root, 'figures');
if save_figs && ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

%% ================================================================
%  Parameters — single permanent magnet centered at origin
%  ================================================================
parameters_maggy_V4;

mu0 = params.physical.mu0;

% Use the first permanent magnet's geometry
r_mag = params.permanent.r(1);   % radius
l_mag = params.permanent.l(1);   % length
J_mag = params.permanent.J;      % magnetization (T)
I_eq  = J_mag / mu0 * l_mag;    % equivalent current

%% ================================================================
%  Evaluation grid (x-z plane, y=0)
%  ================================================================
n_x = 400;
n_z = 400;

x_range = 4 * r_mag;
z_range = 3 * l_mag;

x_vec = linspace(-x_range, x_range, n_x);
z_vec = linspace(-z_range, z_range, n_z);
[X, Z] = meshgrid(x_vec, z_vec);
Y = zeros(size(X));

%% ================================================================
%  Compute fields (Cartesian: Bx, By, Bz at y=0 slice)
%  ================================================================
fprintf('Computing wire-loop field...\n');
[Bx_wire, ~, Bz_wire] = computeFieldCircularWireCartesian( ...
    X, Y, Z, r_mag, I_eq, mu0);

fprintf('Computing current-sheet field...\n');
[Bx_sheet, ~, Bz_sheet] = computeFieldCircularCurrentSheetCartesian( ...
    X, Y, Z, r_mag, l_mag, I_eq, mu0);

Bmag_wire  = sqrt(Bx_wire.^2  + Bz_wire.^2);
Bmag_sheet = sqrt(Bx_sheet.^2 + Bz_sheet.^2);

%% ================================================================
%  Style settings (consistent with integratorStudy)
%  ================================================================
W_FULL     = 17;    % cm — full page width
H_STD      = 8;     % cm
FONT_NAME  = 'Times New Roman';
FONT_AX    = 9;
FONT_LAB   = 10;
FONT_TITLE = 10;

% Shared |B| colormap limits (log10 scale, Tesla)
B_floor  = -5;    % 10 uT
B_ceil   = -1;    % 0.1 T
n_stream = 30;    % seed points per group for streamlines

% Magnet geometry for overlays
mag_x = [-r_mag  r_mag  r_mag -r_mag -r_mag];
mag_z = [-l_mag/2 -l_mag/2 l_mag/2 l_mag/2 -l_mag/2];

%% ================================================================
%  Figure 1: Side-by-side comparison
%  ================================================================
fig1 = mkfig(W_FULL, H_STD);

ax1 = subplot(1,2,1);
plot_field(ax1, x_vec, z_vec, Bx_wire, Bz_wire, Bmag_wire, ...
    'Wire-loop model', 'wire', r_mag, l_mag, B_floor, B_ceil, n_stream, ...
    FONT_NAME, FONT_AX, FONT_LAB, FONT_TITLE);

ax2 = subplot(1,2,2);
plot_field(ax2, x_vec, z_vec, Bx_sheet, Bz_sheet, Bmag_sheet, ...
    'Current-sheet model', 'sheet', r_mag, l_mag, B_floor, B_ceil, n_stream, ...
    FONT_NAME, FONT_AX, FONT_LAB, FONT_TITLE);

cb = colorbar(ax2, 'eastoutside');
cb.Label.String = 'log_{10} |B| (T)';
cb.Label.FontSize = FONT_AX;
cb.Label.FontName = FONT_NAME;

if save_figs
    savefig_thesis(fig1, 'field_wire_vs_sheet', fig_dir);
end

%% ================================================================
%  Figure 2: Difference heatmap
%  ================================================================
fig2 = mkfig(W_FULL * 0.55, H_STD);
ax3 = axes(fig2);

Bdiff = abs(Bmag_sheet - Bmag_wire);
Bdiff_log = log10(max(Bdiff, 1e-12));

imagesc(ax3, x_vec*1e3, z_vec*1e3, Bdiff_log);
set(ax3, 'YDir', 'normal');
hold(ax3, 'on');
plot(ax3, mag_x*1e3, mag_z*1e3, 'w-', 'LineWidth', 1.5);
colormap(ax3, parula);
cb2 = colorbar(ax3);
cb2.Label.String = 'log_{10} |B_{sheet} - B_{wire}| (T)';
cb2.Label.FontSize = FONT_AX;
cb2.Label.FontName = FONT_NAME;
xlabel(ax3, 'x (mm)', 'FontSize', FONT_LAB);
ylabel(ax3, 'z (mm)', 'FontSize', FONT_LAB);
title(ax3, 'Field difference', 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
style_ax(ax3, FONT_NAME, FONT_AX, FONT_LAB);

if save_figs
    savefig_thesis(fig2, 'field_model_diff', fig_dir);
end

fprintf('\nDone.\n');

%% ================================================================
%% Local functions
%% ================================================================

function plot_field(ax, x_vec, z_vec, Bx, Bz, Bmag, ttl, ...
    model_type, r_mag, l_mag, B_floor, B_ceil, n_stream, ...
    FONT_NAME, FONT_AX, FONT_LAB, FONT_TITLE)

    Bmag_log = log10(max(Bmag, 10^B_floor));

    imagesc(ax, x_vec*1e3, z_vec*1e3, Bmag_log);
    set(ax, 'YDir', 'normal');
    clim(ax, [B_floor, B_ceil]);
    colormap(ax, parula);
    hold(ax, 'on');

    % Streamline seeds: left edge, right edge, and horizontal row through center
    z_seeds = linspace(min(z_vec)*0.9, max(z_vec)*0.9, n_stream);
    seed_x = [ones(1,n_stream)*x_vec(2), ones(1,n_stream)*x_vec(end-1), ...
              linspace(min(x_vec)*0.8, max(x_vec)*0.8, n_stream)];
    seed_z = [z_seeds, z_seeds, zeros(1, n_stream)];

    sl = streamline(ax, x_vec*1e3, z_vec*1e3, Bx, Bz, seed_x*1e3, seed_z*1e3);
    set(sl, 'Color', [1 1 1 0.5], 'LineWidth', 0.4);

    % Magnet representation
    if strcmp(model_type, 'wire')
        % Wire-loop: two dots at (±r, 0) showing the wire cross-section
        plot(ax, [-r_mag r_mag]*1e3, [0 0], 'wo', ...
            'MarkerSize', 7, 'MarkerFaceColor', 'w', 'LineWidth', 1.2);
    else
        % Current-sheet: filled gray rectangle showing the magnet body
        patch(ax, [-r_mag r_mag r_mag -r_mag]*1e3, ...
                  [-l_mag/2 -l_mag/2 l_mag/2 l_mag/2]*1e3, ...
              [0.6 0.6 0.6], 'EdgeColor', 'w', 'LineWidth', 1.2, ...
              'FaceAlpha', 0.7);
    end

    xlabel(ax, 'x (mm)', 'FontSize', FONT_LAB);
    ylabel(ax, 'z (mm)', 'FontSize', FONT_LAB);
    title(ax, ttl, 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
    style_ax(ax, FONT_NAME, FONT_AX, FONT_LAB);
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
end

function savefig_thesis(fig, name, out_dir)
    pdf_path = fullfile(out_dir, [name '.pdf']);
    exportgraphics(fig, pdf_path, 'ContentType', 'vector');
    fprintf('  Saved: %s\n', pdf_path);
    png_path = fullfile(out_dir, [name '.png']);
    exportgraphics(fig, png_path, 'Resolution', 300);
    fprintf('  Saved: %s\n', png_path);
end
