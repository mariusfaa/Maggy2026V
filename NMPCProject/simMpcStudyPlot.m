%%
%% simMpcStudyPlot — Thesis-quality figures for MPC comparison study
%
% Reads results from mpcresults/study_*.mat and generates PDF figures.
% Follows plotForThesis.m conventions (Times New Roman, exportgraphics).

clear; clc; close all;

%% ================================================================
%% Settings
%% ================================================================

out_dir  = 'figures';
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
fmt      = 'pdf';
dpi      = 300;

% Figure sizes (cm)
w_full   = 17;
w_half   = 8.5;
h_single = 7;
h_tall   = 10;

% Fonts
font_name  = 'Times New Roman';
font_ax    = 9;
font_lab   = 10;
font_leg   = 8;
font_title = 11;

% Colors per formulation
c_lmpc = [0.00 0.45 0.74];   % blue
c_rti  = [0.85 0.33 0.10];   % red-orange
c_sqp  = [0.47 0.67 0.19];   % green
form_colors = containers.Map({'LMPC','NMPC_RTI','NMPC_SQP'}, ...
    {c_lmpc, c_rti, c_sqp});
form_labels = containers.Map({'LMPC','NMPC_RTI','NMPC_SQP'}, ...
    {'LMPC', 'NMPC-RTI', 'NMPC-SQP'});

%% ================================================================
%% Data loading
%% ================================================================

data_dir = 'mpcresults';
files = dir(fullfile(data_dir, 'study_*.mat'));
fprintf('Found %d result files.\n', numel(files));

R = struct('tag',{}, 'formulation',{}, 'qp_solver',{}, 'N_horizon',{}, ...
           'pert_mm',{}, 'diverged',{}, 't_settle',{}, 'z_overshoot',{}, ...
           'ss_err',{}, 'med_mpc',{}, 'med_lin',{}, 'med_qp',{}, ...
           'med_reg',{}, 'mean_sqp_iter',{}, 'file',{}, ...
           't',{}, 'x',{}, 'u',{});

for i = 1:numel(files)
    d = load(fullfile(files(i).folder, files(i).name));
    if ~isfield(d, 'config'), continue; end

    r.tag         = files(i).name;
    r.formulation = d.config.formulation;
    r.qp_solver   = d.config.qp_solver;
    r.N_horizon   = d.config.N_horizon;
    r.pert_mm     = d.config.pert_m * 1e3;
    r.diverged    = d.diverged;
    r.t_settle    = d.t_settle;
    r.z_overshoot = d.z_overshoot * 1e3;
    r.ss_err      = d.ss_err * 1e3;
    r.med_mpc     = median(d.t_mpc) * 1e6;
    r.med_lin     = median(d.t_lin) * 1e6;
    r.med_qp      = median(d.t_qp)  * 1e6;
    r.med_reg     = median(d.t_reg) * 1e6;
    r.mean_sqp_iter = mean(d.sqp_iter);
    r.file        = fullfile(files(i).folder, files(i).name);
    r.t           = d.t;
    r.x           = d.x;
    r.u           = d.u;

    R = [R, r]; %#ok<AGROW>
end

nR = numel(R);
fprintf('Loaded %d configurations.\n', nR);

forms = unique({R.formulation});
qps   = unique({R.qp_solver});
Ns    = unique([R.N_horizon]);
perts = unique([R.pert_mm]);

%% ================================================================
%% Helper functions
%% ================================================================

    function fig = mkfig(w_cm, h_cm)
        fig = figure('Units','centimeters','Position',[2 2 w_cm h_cm], ...
                     'PaperUnits','centimeters','PaperSize',[w_cm h_cm], ...
                     'PaperPosition',[0 0 w_cm h_cm], 'Color','w');
    end

    function style_ax(ax, fn, fs_ax, fs_lab)
        set(ax, 'FontName', fn, 'FontSize', fs_ax, ...
                'TickDir', 'out', 'TickLength', [0.015 0.015], ...
                'LineWidth', 0.6, 'Box', 'on');
        set(ax.XLabel, 'FontSize', fs_lab);
        set(ax.YLabel, 'FontSize', fs_lab);
        if ~isempty(ax.Title.String)
            set(ax.Title, 'FontSize', fs_lab, 'FontWeight', 'bold');
        end
    end

    function savefig(fig, name, out_dir, fmt, ~)
        fpath = fullfile(out_dir, [name '.' fmt]);
        if strcmp(fmt, 'png')
            exportgraphics(fig, fpath, 'Resolution', 300);
        else
            exportgraphics(fig, fpath, 'ContentType', 'vector');
        end
        fprintf('  Saved: %s\n', fpath);
    end

%% ================================================================
%% Figure 1: Timing comparison — stacked bars per formulation
%% ================================================================

fprintf('\n--- Figure 1: Timing comparison ---\n');
fig1 = mkfig(w_full, h_single);
ax = axes; hold on; grid on;

% Use N=20, 3mm perturbation, PARTIAL_CONDENSING_HPIPM as representative
ref_qp = 'PARTIAL_CONDENSING_HPIPM';
ref_N  = 20;
ref_p  = 3;

bar_data = [];
bar_labels = {};
for fi = 1:numel(forms)
    mask = strcmp({R.formulation}, forms{fi}) & ...
           [R.N_horizon] == ref_N & ...
           [R.pert_mm] == ref_p & ...
           strcmp({R.qp_solver}, ref_qp);
    idx = find(mask, 1);
    if isempty(idx), continue; end
    bar_data(end+1, :) = [R(idx).med_lin, R(idx).med_qp, ...
                          R(idx).med_mpc - R(idx).med_lin - R(idx).med_qp]; %#ok<AGROW>
    bar_labels{end+1} = form_labels(forms{fi}); %#ok<AGROW>
end

b = bar(bar_data, 'stacked');
b(1).FaceColor = [0.30 0.60 0.90];
b(2).FaceColor = [0.90 0.50 0.20];
b(3).FaceColor = [0.70 0.70 0.70];
xticks(1:numel(bar_labels));
xticklabels(bar_labels);
ylabel('Median solve time ($\mu$s)', 'Interpreter', 'latex');
yline(1000, 'k--', 'LineWidth', 1, 'Label', '1 ms real-time limit', ...
      'FontSize', font_leg, 'LabelHorizontalAlignment', 'left');
legend({'Linearization', 'QP solve', 'Other'}, 'Location', 'northwest', ...
       'FontSize', font_leg);
title(sprintf('Solver timing (N=%d, %dmm pert., PC-HPIPM)', ref_N, ref_p), ...
      'Interpreter', 'latex');
style_ax(ax, font_name, font_ax, font_lab);
savefig(fig1, 'mpc_timing', out_dir, fmt, dpi);

%% ================================================================
%% Figure 2: Trajectory overlay — z, roll vs time (best of each)
%% ================================================================

fprintf('--- Figure 2: Trajectory overlay ---\n');
fig2 = mkfig(w_full, h_tall);

% Pick best (lowest settle time) non-diverged config per formulation at 3mm
for si = 1:2
    ax = subplot(2, 1, si); hold on; grid on;

    for fi = 1:numel(forms)
        mask = strcmp({R.formulation}, forms{fi}) & ...
               [R.pert_mm] == 3 & ~[R.diverged];
        idx = find(mask);
        if isempty(idx), continue; end

        settles = [R(idx).t_settle];
        [~, best] = min(settles);
        k = idx(best);

        col = form_colors(forms{fi});
        label = form_labels(forms{fi});

        if si == 1
            plot(R(k).t * 1e3, R(k).x(3,:) * 1e3, '-', ...
                 'Color', col, 'LineWidth', 1.2, 'DisplayName', label);
        else
            plot(R(k).t * 1e3, R(k).x(4,:) * 180/pi, '-', ...
                 'Color', col, 'LineWidth', 1.2, 'DisplayName', label);
        end
    end

    if si == 1
        ylabel('$z$ (mm)', 'Interpreter', 'latex');
        legend('Location', 'best', 'FontSize', font_leg);
        title('Trajectory comparison (3 mm perturbation, best config per formulation)', ...
              'Interpreter', 'latex');
    else
        ylabel('Roll (deg)', 'Interpreter', 'latex');
        xlabel('Time (ms)', 'Interpreter', 'latex');
    end
    style_ax(ax, font_name, font_ax, font_lab);
end
savefig(fig2, 'mpc_trajectory', out_dir, fmt, dpi);

%% ================================================================
%% Figure 3: Pareto front — solve time vs settling time
%% ================================================================

fprintf('--- Figure 3: Pareto front ---\n');
fig3 = mkfig(w_half, h_single);
ax = axes; hold on; grid on;

leg_h = []; leg_s = {};
for fi = 1:numel(forms)
    mask = strcmp({R.formulation}, forms{fi}) & ~[R.diverged];
    idx = find(mask);
    if isempty(idx), continue; end

    col = form_colors(forms{fi});
    mk = {'o', 's', 'd'};

    h = scatter([R(idx).med_mpc], [R(idx).t_settle] * 1e3, 30, ...
                col, 'filled', 'Marker', mk{fi}, ...
                'DisplayName', form_labels(forms{fi}));
    leg_h(end+1) = h; %#ok<AGROW>
    leg_s{end+1} = form_labels(forms{fi}); %#ok<AGROW>
end

set(gca, 'XScale', 'log');
xlabel('Median solve time ($\mu$s)', 'Interpreter', 'latex');
ylabel('Settling time (ms)', 'Interpreter', 'latex');
legend(leg_h, leg_s, 'Location', 'best', 'FontSize', font_leg);
title('Speed--quality Pareto', 'Interpreter', 'latex');
style_ax(ax, font_name, font_ax, font_lab);
savefig(fig3, 'mpc_pareto', out_dir, fmt, dpi);

%% ================================================================
%% Figure 4: Perturbation robustness — settling time vs pert per form
%% ================================================================

fprintf('--- Figure 4: Perturbation robustness ---\n');
fig4 = mkfig(w_half, h_single);
ax = axes; hold on; grid on;

for fi = 1:numel(forms)
    col = form_colors(forms{fi});
    settle_mean = zeros(1, numel(perts));

    for pi_idx = 1:numel(perts)
        mask = strcmp({R.formulation}, forms{fi}) & ...
               [R.pert_mm] == perts(pi_idx) & ...
               strcmp({R.qp_solver}, ref_qp) & ...
               [R.N_horizon] == ref_N;
        idx = find(mask);
        if isempty(idx) || R(idx).diverged
            settle_mean(pi_idx) = NaN;
        else
            settle_mean(pi_idx) = R(idx).t_settle * 1e3;
        end
    end

    plot(perts, settle_mean, '-o', 'Color', col, 'LineWidth', 1.2, ...
         'MarkerFaceColor', col, 'MarkerSize', 5, ...
         'DisplayName', form_labels(forms{fi}));
end

xlabel('Perturbation (mm)', 'Interpreter', 'latex');
ylabel('Settling time (ms)', 'Interpreter', 'latex');
legend('Location', 'best', 'FontSize', font_leg);
title(sprintf('Robustness (N=%d, PC-HPIPM)', ref_N), 'Interpreter', 'latex');
style_ax(ax, font_name, font_ax, font_lab);
savefig(fig4, 'mpc_robustness', out_dir, fmt, dpi);

%% ================================================================
%% Figure 5: Horizon sensitivity — settling time vs N per formulation
%% ================================================================

fprintf('--- Figure 5: Horizon sensitivity ---\n');
fig5 = mkfig(w_half, h_single);
ax = axes; hold on; grid on;

for fi = 1:numel(forms)
    col = form_colors(forms{fi});
    settle_vs_N = zeros(1, numel(Ns));

    for ni = 1:numel(Ns)
        mask = strcmp({R.formulation}, forms{fi}) & ...
               [R.N_horizon] == Ns(ni) & ...
               [R.pert_mm] == 3 & ...
               strcmp({R.qp_solver}, ref_qp);
        idx = find(mask);
        if isempty(idx) || R(idx).diverged
            settle_vs_N(ni) = NaN;
        else
            settle_vs_N(ni) = R(idx).t_settle * 1e3;
        end
    end

    plot(Ns, settle_vs_N, '-o', 'Color', col, 'LineWidth', 1.2, ...
         'MarkerFaceColor', col, 'MarkerSize', 5, ...
         'DisplayName', form_labels(forms{fi}));
end

xlabel('Prediction horizon $N$', 'Interpreter', 'latex');
ylabel('Settling time (ms)', 'Interpreter', 'latex');
legend('Location', 'best', 'FontSize', font_leg);
title('Horizon sensitivity (3 mm pert., PC-HPIPM)', 'Interpreter', 'latex');
style_ax(ax, font_name, font_ax, font_lab);
savefig(fig5, 'mpc_horizon', out_dir, fmt, dpi);

%% ================================================================
%% Figure 6: QP solver comparison — bars for fixed N=20
%% ================================================================

fprintf('--- Figure 6: QP solver comparison ---\n');
fig6 = mkfig(w_full, h_single);
ax = axes; hold on; grid on;

% Group: formulations × QP solvers, N=20, 3mm
qp_labels_short = {'PC-HPIPM', 'FC-DAQP', 'FC-HPIPM'};
bar_data = NaN(numel(forms), numel(qps));

for fi = 1:numel(forms)
    for qi = 1:numel(qps)
        mask = strcmp({R.formulation}, forms{fi}) & ...
               strcmp({R.qp_solver}, qps{qi}) & ...
               [R.N_horizon] == 20 & ...
               [R.pert_mm] == 3 & ...
               ~[R.diverged];
        idx = find(mask, 1);
        if ~isempty(idx)
            bar_data(fi, qi) = R(idx).med_mpc;
        end
    end
end

b = bar(bar_data);
for qi = 1:numel(qps)
    b(qi).DisplayName = qp_labels_short{qi};
end
xticks(1:numel(forms));
form_display = cellfun(@(f) form_labels(f), forms, 'UniformOutput', false);
xticklabels(form_display);
ylabel('Median solve time ($\mu$s)', 'Interpreter', 'latex');
yline(1000, 'k--', 'LineWidth', 1, 'Label', '1 ms limit', ...
      'FontSize', font_leg, 'LabelHorizontalAlignment', 'left');
legend('Location', 'northeast', 'FontSize', font_leg);
title('QP solver comparison (N=20, 3 mm pert.)', 'Interpreter', 'latex');
style_ax(ax, font_name, font_ax, font_lab);
savefig(fig6, 'mpc_qp_comparison', out_dir, fmt, dpi);

%% ================================================================
%% Done
%% ================================================================

fprintf('\n=== All MPC study figures exported to %s/ ===\n', out_dir);
