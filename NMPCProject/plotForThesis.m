%% plotForThesis — Publication-quality figures for thesis
%
% Requires: simAcadosAnalyser.m to have been run (or runs the data loading
% itself). Exports PDF vector figures to figures/.
%
% Figures produced:
%   1. Convergence study      (error vs n_steps, log-log, with O(h^4) slope)
%   2. Runtime comparison     (grouped bar, ERK vs IRK)
%   3. Speed-accuracy Pareto  (log-log scatter)
%   4. Trajectory comparison  (position & orientation vs reference)
%   5. Error evolution        (per-axis error over time)

clear; clc; close all;

%% ================================================================
%% Settings
%% ================================================================

out_dir  = 'figures';
fmt      = 'pdf';           % 'pdf', 'eps', or 'png'
dpi      = 300;             % for raster formats

% Figure sizes (cm) — sized for A4 thesis, single/double column
w_full   = 17;              % double-column width
w_half   = 8.5;             % single-column width
h_single = 7;               % standard height
h_tall   = 10;

% Fonts
font_name = 'Times New Roman';
font_ax   = 9;              % axis tick labels
font_lab  = 10;             % axis labels
font_leg  = 8;              % legend
font_title = 11;            % subplot titles

% Colors — distinguishable, printer-friendly
c_erk = [0.00 0.45 0.74];   % blue
c_irk = [0.85 0.33 0.10];   % red-orange
c_ref = [0.00 0.00 0.00];   % black

% Trajectory subset: hand-picked for clarity
traj_pick = {
    'ERK', 1, 1;    % ERK n=1  1ms (fastest)
    'ERK', 5, 1;    % ERK n=5  1ms (mid)
    'IRK', 1, 1;    % IRK n=1  1ms
    'IRK', 5, 1;    % IRK n=5  1ms
    'IRK', 1, 2;    % IRK n=1  2ms
    'IRK', 5, 2;    % IRK n=5  2ms
};

%% ================================================================
%% Data loading (mirrors simAcadosAnalyser section 1-2)
%% ================================================================

sim_dir  = 'simresults';
ref_file = 'results_ode_fast.mat';
exclude  = {'ERK_5ms'};

files = dir(fullfile(sim_dir, 'sim_fn_*.mat'));
ref = load(ref_file);

R = struct('name',{}, 'integrator',{}, 'stages',{}, 'steps',{}, ...
           'dt_ms',{}, 'dt_val',{}, 'T_end',{}, ...
           't_tot_mean',{}, 't_tot_median',{}, 't_la_mean',{}, 't_ad_mean',{}, ...
           'pos_err_max',{}, 'ang_err_max',{}, ...
           'vel_err_max',{}, 'omg_err_max',{});

for i = 1:numel(files)
    [~, fname] = fileparts(files(i).name);
    tokens = regexp(fname, '^sim_fn_(\w+?)_(\d+)_(\d+)_(\d+)ms$', 'tokens');
    if isempty(tokens), continue; end
    tok = tokens{1};

    r.name       = fname;
    r.integrator = upper(tok{1});
    r.stages     = str2double(tok{2});
    r.steps      = str2double(tok{3});
    r.dt_ms      = str2double(tok{4});
    r.dt_val     = r.dt_ms * 1e-3;

    if any(strcmp(sprintf('%s_%dms', r.integrator, r.dt_ms), exclude))
        continue;
    end

    d = load(fullfile(files(i).folder, files(i).name));
    r.T_end = d.t(end);

    if isfield(d, 't_tot')
        r.t_tot_mean   = mean(d.t_tot) * 1e6;
        r.t_tot_median = median(d.t_tot) * 1e6;
    else
        r.t_tot_mean = NaN; r.t_tot_median = NaN;
    end
    if isfield(d, 't_la'), r.t_la_mean = mean(d.t_la)*1e6; else, r.t_la_mean = NaN; end
    if isfield(d, 't_ad'), r.t_ad_mean = mean(d.t_ad)*1e6; else, r.t_ad_mean = NaN; end

    T_common = min(r.T_end, ref.t(end));
    t_grid = 0:r.dt_val:T_common;
    r.pos_err_max = NaN; r.ang_err_max = NaN;
    r.vel_err_max = NaN; r.omg_err_max = NaN;
    if numel(t_grid) >= 2
        x_s = interp1(d.t, d.x', t_grid, 'pchip')';
        x_r = interp1(ref.t, ref.x', t_grid, 'pchip')';
        err = abs(x_s - x_r);
        r.pos_err_max = max(err(1:3,:),[],'all');
        r.ang_err_max = max(err(4:6,:),[],'all');
        r.vel_err_max = max(err(7:9,:),[],'all');
        r.omg_err_max = max(err(10:12,:),[],'all');
    end

    R = [R, r]; %#ok<AGROW>
end

nR  = numel(R);
dts = unique([R.dt_ms]);
fprintf('Loaded %d configs, dt values: %s ms\n', nR, mat2str(dts));

%% ================================================================
%% Helper: consistent figure creation
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

    function savefig(fig, name, out_dir, fmt, dpi)
        fpath = fullfile(out_dir, [name '.' fmt]);
        if strcmp(fmt, 'png')
            exportgraphics(fig, fpath, 'Resolution', dpi);
        else
            exportgraphics(fig, fpath, 'ContentType', 'vector');
        end
        fprintf('  Saved: %s\n', fpath);
    end

%% ================================================================
%% Figure 1: Convergence study — error vs n_steps (log scale)
%% ================================================================

fprintf('\n--- Figure 1: Convergence ---\n');
fig1 = mkfig(w_full, h_single);

sp = 0;
for di = 1:numel(dts)
    dt_val = dts(di);

    for si = 1:2  % 1=position, 2=angle
        sp = sp + 1;
        ax = subplot(numel(dts), 2, sp); hold on; grid on;

        for integ = ["ERK", "IRK"]
            mask = [R.dt_ms] == dt_val & strcmp({R.integrator}, integ);
            idx = find(mask);
            if isempty(idx), continue; end

            [steps_sorted, ord] = sort([R(idx).steps]);
            idx = idx(ord);

            if si == 1
                errs = [R(idx).pos_err_max] * 1e3;
                ylab = 'Max pos. error (mm)';
            else
                errs = [R(idx).ang_err_max] * 180/pi;
                ylab = 'Max ang. error (deg)';
            end

            col = c_erk; mk = 'o';
            if strcmp(integ, 'IRK'), col = c_irk; mk = 's'; end

            valid = ~isnan(errs) & errs > 0;
            plot(steps_sorted(valid), errs(valid), '-', ...
                 'Color', col, 'Marker', mk, 'MarkerSize', 4, ...
                 'MarkerFaceColor', col, 'LineWidth', 1.2, ...
                 'DisplayName', integ);
        end

        set(gca, 'YScale', 'log');
        xlabel('Integration sub-steps $n$', 'Interpreter', 'latex');
        ylabel(ylab);
        if sp <= 2
            title(sprintf('%s error, $\\Delta t = %d$ ms', ...
                   ternary(si==1,'Position','Angle'), dt_val), ...
                   'Interpreter', 'latex');
        else
            title(sprintf('$\\Delta t = %d$ ms', dt_val), 'Interpreter', 'latex');
        end
        if sp == 1, legend('Location','best','FontSize',font_leg); end
        style_ax(ax, font_name, font_ax, font_lab);
    end
end
savefig(fig1, 'convergence', out_dir, fmt, dpi);

%% ================================================================
%% Figure 2: Runtime comparison — grouped bar per dt
%% ================================================================

fprintf('--- Figure 2: Runtime ---\n');
fig2 = mkfig(w_full, h_single);

for di = 1:numel(dts)
    ax = subplot(1, numel(dts), di); hold on; grid on;

    mask = [R.dt_ms] == dts(di);
    idx = find(mask);
    % Sort: ERK first, then IRK, each by steps
    erk_idx = idx(strcmp({R(idx).integrator},'ERK'));
    irk_idx = idx(strcmp({R(idx).integrator},'IRK'));
    [~,o] = sort([R(erk_idx).steps]); erk_idx = erk_idx(o);
    [~,o] = sort([R(irk_idx).steps]); irk_idx = irk_idx(o);
    idx = [erk_idx, irk_idx];
    n = numel(idx);

    bar_data = [R(idx).t_tot_median];
    bar_colors = zeros(n, 3);
    for j = 1:n
        if strcmp(R(idx(j)).integrator, 'ERK')
            bar_colors(j,:) = c_erk;
        else
            bar_colors(j,:) = c_irk;
        end
    end

    b = bar(1:n, bar_data);
    b.FaceColor = 'flat';
    b.CData = bar_colors;
    b.EdgeColor = 'none';

    yline(dts(di)*1e3, 'k--', 'LineWidth', 1, ...
          'Label','Real-time', 'FontSize', font_leg, ...
          'LabelHorizontalAlignment','left');

    tick_labels = arrayfun(@(k) sprintf('%s %d', lower(R(k).integrator), R(k).steps), ...
                           idx, 'UniformOutput', false);
    xticks(1:n); xticklabels(tick_labels); xtickangle(45);
    ylabel('Median step time ($\mu$s)', 'Interpreter', 'latex');
    title(sprintf('$\\Delta t = %d$ ms', dts(di)), 'Interpreter', 'latex');
    style_ax(ax, font_name, font_ax, font_lab);
end
savefig(fig2, 'runtime', out_dir, fmt, dpi);

%% ================================================================
%% Figure 3: Pareto — speed vs accuracy
%% ================================================================

fprintf('--- Figure 3: Pareto ---\n');
fig3 = mkfig(w_half, h_single);
ax = axes; hold on; grid on;

dt_styles = {'-', '--', ':'};  % line styles per dt (for connecting)
dt_colors_map = lines(numel(dts));

leg_h = []; leg_s = {};
for di = 1:numel(dts)
    for integ = ["ERK", "IRK"]
        mask = [R.dt_ms]==dts(di) & strcmp({R.integrator},integ);
        idx = find(mask);
        if isempty(idx), continue; end
        [~,o] = sort([R(idx).steps]); idx = idx(o);

        valid = ~isnan([R(idx).pos_err_max]) & ~isnan([R(idx).t_tot_median]);
        idx = idx(valid);
        if isempty(idx), continue; end

        col = c_erk; mk = 'o';
        if strcmp(integ,'IRK'), col = c_irk; mk = 's'; end

        xx = [R(idx).t_tot_median];
        yy = [R(idx).pos_err_max] * 1e3;

        ls = dt_styles{min(di, numel(dt_styles))};
        h = plot(xx, yy, [ls mk], 'Color', col, 'MarkerSize', 5, ...
                 'MarkerFaceColor', col, 'LineWidth', 1);
        leg_h(end+1) = h; %#ok<AGROW>
        leg_s{end+1} = sprintf('%s %d ms', integ, dts(di)); %#ok<AGROW>

        % Annotate n_steps on a few points
        for j = [1, numel(idx)]
            text(xx(j)*1.08, yy(j), sprintf('n=%d', R(idx(j)).steps), ...
                 'FontSize', 7, 'FontName', font_name);
        end
    end
end
set(gca, 'XScale','log', 'YScale','log');
xlabel('Median step time ($\mu$s)', 'Interpreter', 'latex');
ylabel('Max position error (mm)');
legend(leg_h, leg_s, 'Location','best', 'FontSize', font_leg);
style_ax(ax, font_name, font_ax, font_lab);
savefig(fig3, 'pareto', out_dir, fmt, dpi);

%% ================================================================
%% Figure 4: Trajectory comparison — position
%% ================================================================

fprintf('--- Figure 4: Trajectories ---\n');

% Resolve traj_pick to indices in R
sel_idx = [];
for p = 1:size(traj_pick, 1)
    integ = traj_pick{p,1}; ns = traj_pick{p,2}; dm = traj_pick{p,3};
    mi = find(strcmp({R.integrator}, integ) & [R.steps]==ns & [R.dt_ms]==dm, 1);
    if ~isempty(mi)
        sel_idx(end+1) = mi; %#ok<AGROW>
    end
end
nSel = numel(sel_idx);
T_plot = min([ref.t(end), R(sel_idx).T_end]);

% Distinguishable colors + line styles for the subset
sel_colors = lines(nSel);
sel_styles = repmat({'-','--','-.',':'}, 1, ceil(nSel/4));

state_names = {'$x$','$y$','$z$'};
pos_scale = 1e3;

fig4 = mkfig(w_full, h_tall);
for si = 1:3
    ax = subplot(3,1,si); hold on; grid on;

    ref_mask = ref.t <= T_plot + 1e-12;
    plot(ref.t(ref_mask), ref.x(si,ref_mask)*pos_scale, ...
         'k-', 'LineWidth', 1.5, 'DisplayName', 'ODE reference');

    for j = 1:nSel
        k = sel_idx(j);
        d = load(fullfile(sim_dir, [R(k).name '.mat']), 't', 'x');
        sm = d.t <= T_plot + 1e-12;
        label = sprintf('%s $n$=%d, %d ms', R(k).integrator, R(k).steps, R(k).dt_ms);
        plot(d.t(sm), d.x(si,sm)*pos_scale, sel_styles{j}, ...
             'Color', sel_colors(j,:), 'LineWidth', 1, 'DisplayName', label);
    end

    ylabel(sprintf('%s (mm)', state_names{si}), 'Interpreter', 'latex');
    if si == 3, xlabel('Time (s)'); end
    if si == 1
        legend('Interpreter','latex','Location','best','FontSize',font_leg, ...
               'NumColumns', 2);
    end
    style_ax(ax, font_name, font_ax, font_lab);
end
savefig(fig4, 'trajectory_pos', out_dir, fmt, dpi);

%% ================================================================
%% Figure 5: Trajectory comparison — orientation
%% ================================================================

fprintf('--- Figure 5: Orientation ---\n');

ang_names = {'$\phi$ (roll)','$\theta$ (pitch)','$\psi$ (yaw)'};
ang_scale = 180/pi;

fig5 = mkfig(w_full, h_tall);
for si = 1:3
    ax = subplot(3,1,si); hold on; grid on;

    ref_mask = ref.t <= T_plot + 1e-12;
    plot(ref.t(ref_mask), ref.x(si+3,ref_mask)*ang_scale, ...
         'k-', 'LineWidth', 1.5, 'DisplayName', 'ODE reference');

    for j = 1:nSel
        k = sel_idx(j);
        d = load(fullfile(sim_dir, [R(k).name '.mat']), 't', 'x');
        sm = d.t <= T_plot + 1e-12;
        label = sprintf('%s $n$=%d, %d ms', R(k).integrator, R(k).steps, R(k).dt_ms);
        plot(d.t(sm), d.x(si+3,sm)*ang_scale, sel_styles{j}, ...
             'Color', sel_colors(j,:), 'LineWidth', 1, 'DisplayName', label);
    end

    ylabel(sprintf('%s (deg)', ang_names{si}), 'Interpreter', 'latex');
    if si == 3, xlabel('Time (s)'); end
    if si == 1
        legend('Interpreter','latex','Location','best','FontSize',font_leg, ...
               'NumColumns', 2);
    end
    style_ax(ax, font_name, font_ax, font_lab);
end
savefig(fig5, 'trajectory_ang', out_dir, fmt, dpi);

%% ================================================================
%% Figure 6: Error evolution over time
%% ================================================================

fprintf('--- Figure 6: Error evolution ---\n');
fig6 = mkfig(w_full, h_single);

err_names = {'$|e_x|$','$|e_y|$','$|e_z|$'};

for si = 1:3
    ax = subplot(1,3,si); hold on; grid on;

    for j = 1:nSel
        k = sel_idx(j);
        d = load(fullfile(sim_dir, [R(k).name '.mat']), 't', 'x');
        T_c = min(d.t(end), ref.t(end));
        tg = 0:R(k).dt_val:T_c;
        if numel(tg) < 2, continue; end
        xs = interp1(d.t, d.x', tg, 'pchip')';
        xr = interp1(ref.t, ref.x', tg, 'pchip')';
        err_mm = abs(xs(si,:) - xr(si,:)) * pos_scale;

        label = sprintf('%s $n$=%d, %d ms', R(k).integrator, R(k).steps, R(k).dt_ms);
        plot(tg, err_mm, sel_styles{j}, 'Color', sel_colors(j,:), ...
             'LineWidth', 1, 'DisplayName', label);
    end

    xlabel('Time (s)');
    ylabel(sprintf('%s (mm)', err_names{si}), 'Interpreter', 'latex');
    if si == 1
        legend('Interpreter','latex','Location','best','FontSize',font_leg);
    end
    style_ax(ax, font_name, font_ax, font_lab);
end
savefig(fig6, 'error_evolution', out_dir, fmt, dpi);

%% ================================================================
%% Figure 7: Survival time comparison
%% ================================================================

fprintf('--- Figure 7: Survival time ---\n');
fig7 = mkfig(w_full, h_single);

for di = 1:numel(dts)
    ax = subplot(1, numel(dts), di); hold on; grid on;

    mask = [R.dt_ms] == dts(di);
    idx = find(mask);
    erk_idx = idx(strcmp({R(idx).integrator},'ERK'));
    irk_idx = idx(strcmp({R(idx).integrator},'IRK'));
    [~,o] = sort([R(erk_idx).steps]); erk_idx = erk_idx(o);
    [~,o] = sort([R(irk_idx).steps]); irk_idx = irk_idx(o);
    idx = [erk_idx, irk_idx];
    n = numel(idx);

    T_ends = [R(idx).T_end];
    bar_colors = zeros(n, 3);
    for j = 1:n
        if strcmp(R(idx(j)).integrator, 'ERK')
            bar_colors(j,:) = c_erk;
        else
            bar_colors(j,:) = c_irk;
        end
    end

    b = bar(1:n, T_ends);
    b.FaceColor = 'flat';
    b.CData = bar_colors;
    b.EdgeColor = 'none';

    tick_labels = arrayfun(@(k) sprintf('%s %d', lower(R(k).integrator), R(k).steps), ...
                           idx, 'UniformOutput', false);
    xticks(1:n); xticklabels(tick_labels); xtickangle(45);
    ylabel('Survival time (s)');
    title(sprintf('$\\Delta t = %d$ ms', dts(di)), 'Interpreter', 'latex');
    style_ax(ax, font_name, font_ax, font_lab);
end
savefig(fig7, 'survival', out_dir, fmt, dpi);

%% ================================================================
%% Done
%% ================================================================

fprintf('\n=== All figures exported to %s/ ===\n', out_dir);

%% ================================================================
%% Local utilities
%% ================================================================

function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
