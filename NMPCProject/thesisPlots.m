%% thesisPlots — Generate all thesis figures from simulation results
%
% Checks which .mat files exist, warns about missing ones, and generates
% all plots from available data. Edit the configuration per figure below.
%
% Usage:
%   thesisPlots

clear; clc; close all;

%% ================================================================
%  PATHS
%  ================================================================

project_root = getenv('ACADOS_PROJECT_DIR');
if isempty(project_root), project_root = pwd; end
addpath(genpath(fullfile(project_root, 'utilities')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'model_matlab')));

res_dir = fullfile(project_root, 'results');

%% ================================================================
%  PLOT ENABLE FLAGS
%  ================================================================

plot_controller_comparison = true;   % Fig 1: states+inputs+cost, all controllers
plot_horizon_sweep         = true;   % Fig 2: per-controller N sweep trajectories
plot_cost_vs_N             = true;   % Fig 3: final cost vs N_horizon
plot_compute_vs_N          = true;   % Fig 4: compute time vs N_horizon
plot_timing_breakdown      = true;   % Fig 5: stacked timing bar chart
plot_convergence           = false;   % Fig 6: semilogy error norm
plot_min_horizon           = false;   % Fig 7: minimum N for convergence per controller

%% ================================================================
%  FIGURE CONFIGURATIONS — edit per plot
%  ================================================================

% --- Fig 1: Controller comparison (fixed N, dt_mpc) ---
fig1.controllers = {'lqr', 'lmpc', 'solmpc', 'nmpc'};
fig1.N           = 20;
fig1.dt_mpc      = 0.001;

% --- Fig 2: Horizon sweep trajectories (per controller) ---
fig2.controllers = {'lmpc', 'solmpc', 'nmpc'};
fig2.N_list      = [11, 20, 30];
fig2.dt_mpc      = 0.001;

% --- Fig 3: Final cost vs N ---
fig3.controllers = {'lmpc', 'solmpc', 'nmpc'};
fig3.N_list      = [11, 15:5:40];
fig3.dt_mpc      = 0.001;

% --- Fig 4: Compute time vs N ---
fig4.controllers = {'lqr', 'lmpc', 'solmpc', 'nmpc'};
fig4.N_list      = [11, 15:5:40];
fig4.dt_mpc      = 0.001;

% --- Fig 5: Timing breakdown (fixed N) ---
fig5.controllers = {'lqr', 'lmpc', 'solmpc', 'nmpc'};
fig5.N           = 20;
fig5.dt_mpc      = 0.001;

% --- Fig 6: Convergence (fixed N) ---
fig6.controllers = {'lqr', 'lmpc', 'solmpc', 'nmpc'};
fig6.N           = 20;
fig6.dt_mpc      = 0.001;

% --- Fig 7: Minimum horizon for convergence ---
fig7.controllers = {'lmpc', 'solmpc', 'nmpc'};
fig7.N_list      = [11, 15:5:40];
fig7.dt_mpc      = 0.001;

% Input saturation limit (for ylim on solenoid plots)
umax = 1.0;

%% ================================================================
%  EXPORT OPTIONS
%  ================================================================

save_pdf  = false;                          % set false to disable export
out_dir   = fullfile(project_root, 'figures/sc1');  % change to e.g. 'plots/scene_1'
if save_pdf && ~exist(out_dir,'dir'), mkdir(out_dir); end

%% ================================================================
%  EQUILIBRIUM (needed for deviation plots)
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
ctrl_colors('lqr')    = [1 0.08 0.6];
ctrl_colors('lmpc')   = [0.0 0.8 0.2];
ctrl_colors('solmpc') = [1 0.4 0];
ctrl_colors('nmpc')   = [0 0 1];

state_names = {'x','y','z','\phi','\theta', ...
               '\dot{x}','\dot{y}','\dot{z}','\dot{\phi}','\dot{\theta}'};
state_units = {'mm','mm','mm','deg','deg','m/s','m/s','m/s','rad/s','rad/s'};
state_scale = [1e3, 1e3, 1e3, 180/pi, 180/pi, 1, 1, 1, 1, 1];

%% ================================================================
%% FIG 1: Controller Comparison — states, inputs, cost
%% ================================================================

if plot_controller_comparison
    [data, ok] = loadRuns(res_dir, fig1.controllers, fig1.N, fig1.dt_mpc);
    if ok
        fig = figure('Name','Controller Comparison', ...
            'Units','normalized','Position',[0.02 0.02 0.96 0.92]);

        % Row 1: 5 states
        for s = 1:5
            subplot(4,3,s); hold on; grid on; box on;
            for i = 1:numel(data)
                r = data{i};
                col = ctrl_colors(r.controller);
                plot(r.t*1e3, (r.x(s,:)-xEq(s))*state_scale(s), ...
                    'Color',col,'LineWidth',1.4, ...
                    'DisplayName',upper(r.controller));
            end
            ylabel(sprintf('%s (%s)',state_names{s},state_units{s}));
            xlabel('Time (ms)');
            title(state_names{s});
            if s==1, legend('Location','best'); end
        end

        % Slot 6: z absolute
        subplot(4,3,6); hold on; grid on; box on;
        for i = 1:numel(data)
            r = data{i};
            plot(r.t*1e3, r.x(3,:)*1e3, 'Color',ctrl_colors(r.controller), ...
                'LineWidth',1.4,'HandleVisibility','off');
        end
        yline(xEq(3)*1e3,'k--','LineWidth',0.8,'HandleVisibility','off');
        ylabel('z (mm)'); xlabel('Time (ms)'); title('z (absolute)');

        % Row 3: 4 inputs
        for s = 1:4
            subplot(4,3,6+s); hold on; grid on; box on;
            for i = 1:numel(data)
                r = data{i};
                plot(r.t*1e3, r.u(s,:), 'Color',ctrl_colors(r.controller), ...
                    'LineWidth',1.0,'HandleVisibility','off');
            end
            ylim([-umax umax]);
            ylabel(sprintf('u_%d (A)',s)); xlabel('Time (ms)');
            title(sprintf('Solenoid %d',s));
        end

        % Slot 11: cumulative cost
        subplot(4,3,11); hold on; grid on; box on;
        for i = 1:numel(data)
            r = data{i};
            plot(r.t*1e3, r.cost_cum, 'Color',ctrl_colors(r.controller), ...
                'LineWidth',1.4,'HandleVisibility','off');
        end
        xlabel('Time (ms)'); ylabel('Cumulative cost'); title('Cumulative Cost');

        sgtitle(sprintf('Controller Comparison — N=%d, \\Deltat_{mpc}=%.0f\\mus', ...
            fig1.N, fig1.dt_mpc*1e6), 'FontSize',13);
        saveFig(fig, out_dir, 'ctrl_comparison', save_pdf);
    end
end

%% ================================================================
%% FIG 2: Horizon Sweep Trajectories (per controller)
%% ================================================================

if plot_horizon_sweep
    N_cmap = lines(numel(fig2.N_list));

    for ic = 1:numel(fig2.controllers)
        cname = fig2.controllers{ic};
        [data, ok] = loadRuns(res_dir, {cname}, fig2.N_list, fig2.dt_mpc);
        if ~ok, continue; end

        fig = figure('Name', sprintf('N-sweep %s',upper(cname)), ...
            'Units','normalized','Position',[0.02 0.05 0.96 0.75]);

        % Row 1: 5 states
        for s = 1:5
            subplot(2,5,s); hold on; grid on; box on;
            for i = 1:numel(data)
                r = data{i};
                ni = find(fig2.N_list == r.N_horizon);
                col = N_cmap(ni,:);
                plot(r.t*1e3, (r.x(s,:)-xEq(s))*state_scale(s), ...
                    'Color',col,'LineWidth',1.4, ...
                    'DisplayName',sprintf('N=%d',r.N_horizon));
            end
            ylabel(sprintf('%s (%s)',state_names{s},state_units{s}));
            xlabel('Time (ms)'); title(state_names{s});
            if s==1, legend('Location','best'); end
        end

        % Row 2: 4 inputs + cost
        for s = 1:4
            subplot(2,5,5+s); hold on; grid on; box on;
            for i = 1:numel(data)
                r = data{i};
                ni = find(fig2.N_list == r.N_horizon);
                col = N_cmap(ni,:);
                plot(r.t*1e3, r.u(s,:), 'Color',col,'LineWidth',1.0, ...
                    'HandleVisibility','off');
            end
            ylim([-umax umax]);
            ylabel(sprintf('u_%d (A)',s)); xlabel('Time (ms)');
            title(sprintf('Solenoid %d',s));
        end

        subplot(2,5,10); hold on; grid on; box on;
        for i = 1:numel(data)
            r = data{i};
            ni = find(fig2.N_list == r.N_horizon);
            col = N_cmap(ni,:);
            plot(r.t*1e3, r.cost_cum, 'Color',col,'LineWidth',1.4, ...
                'HandleVisibility','off');
        end
        xlabel('Time (ms)'); ylabel('Cum. cost'); title('Cumulative Cost');

        sgtitle(sprintf('%s — Horizon Sweep (\\Deltat_{mpc}=%.0f\\mus)', ...
            upper(cname), fig2.dt_mpc*1e6), 'FontSize',13);
        saveFig(fig, out_dir, sprintf('horizon_sweep_%s', cname), save_pdf);
    end
end

%% ================================================================
%% FIG 3: Final Cumulative Cost vs N_horizon
%% ================================================================

if plot_cost_vs_N
    figure('Name','Cost vs N','Units','normalized','Position',[0.15 0.2 0.55 0.45]);
    hold on; grid on; box on;

    for ic = 1:numel(fig3.controllers)
        cname = fig3.controllers{ic};
        N_vals = []; cost_vals = [];
        for N = fig3.N_list
            f = fullfile(res_dir, getFilename(cname, N, fig3.dt_mpc));
            if ~isfile(f), continue; end
            d = load(f);
            if d.diverged, continue; end  % skip diverged runs
            N_vals(end+1)    = N; %#ok<AGROW>
            cost_vals(end+1) = d.cost_cum(end); %#ok<AGROW>
        end
        if isempty(N_vals), continue; end
        col = ctrl_colors(cname);
        plot(N_vals, cost_vals, '-o', 'Color',col, 'LineWidth',1.4, ...
            'MarkerFaceColor',col, 'MarkerSize',5, ...
            'DisplayName',upper(cname));
    end

    xlabel('N_{horizon}'); ylabel('Final cumulative cost');
    legend('Location','best');
    title(sprintf('Final Cumulative Cost vs Horizon — \\Deltat_{mpc}=%.0f\\mus', fig3.dt_mpc*1e6));
    saveFig(gcf, out_dir, 'cost_vs_N', save_pdf);
end

%% ================================================================
%% FIG 4: Compute Time vs N_horizon
%% ================================================================

if plot_compute_vs_N
    figure('Name','Compute vs N','Units','normalized','Position',[0.15 0.2 0.55 0.45]);
    hold on; grid on; box on;

    for ic = 1:numel(fig4.controllers)
        cname = fig4.controllers{ic};
        N_vals = []; time_mean = [];
        for N = fig4.N_list
            f = fullfile(res_dir, getFilename(cname, N, fig4.dt_mpc));
            if ~isfile(f), continue; end
            d = load(f);
            if d.diverged, continue; end  % skip diverged runs
            t_ctrl = getCtrlTime(d);
            N_vals(end+1)    = N; %#ok<AGROW>
            time_mean(end+1) = mean(t_ctrl)*1e6; %#ok<AGROW>
        end
        if isempty(N_vals), continue; end
        col = ctrl_colors(cname);
        plot(N_vals, time_mean, '-o', 'Color',col, 'LineWidth',1.4, ...
            'MarkerFaceColor',col, 'MarkerSize',5, ...
            'DisplayName',upper(cname));
    end

    yline(fig4.dt_mpc*1e6, 'r--', 'LineWidth',1.5, ...
        'Label','Real-time budget','LabelHorizontalAlignment','left','HandleVisibility','off');
    xlabel('N_{horizon}'); ylabel('Mean controller time (\mus)');
    legend('Location','best');
    title(sprintf('Compute Time vs Horizon — \\Deltat_{mpc}=%.0f\\mus', fig4.dt_mpc*1e6));
    saveFig(gcf, out_dir, 'compute_vs_N', save_pdf);
end

%% ================================================================
%% FIG 5: Timing Breakdown (stacked bar)
%% ================================================================

if plot_timing_breakdown
    [data, ok] = loadRuns(res_dir, fig5.controllers, fig5.N, fig5.dt_mpc);
    if ok
        figure('Name','Timing Breakdown','Units','normalized','Position',[0.2 0.15 0.55 0.45]);
        hold on; grid on; box on;

        names = {};
        t_qp = []; t_lin = []; t_other = []; t_jac = [];
        cols = [];

        for i = 1:numel(data)
            r = data{i};
            names{end+1} = upper(r.controller); %#ok<AGROW>
            cols = [cols; ctrl_colors(r.controller)]; %#ok<AGROW>

            if isfield(r, 'ocp_time_qp')
                qp  = mean(r.ocp_time_qp)*1e6;
                lin = mean(r.ocp_time_lin)*1e6;
                tot = mean(r.ocp_time_tot)*1e6;
                oth = tot - qp - lin;
                jac = 0;
                if isfield(r, 'jac_time')
                    jac = mean(r.jac_time)*1e6;
                end
            elseif isfield(r, 'ctrl_time')
                qp  = 0; lin = 0; jac = 0;
                oth = mean(r.ctrl_time)*1e6;
            else
                qp = 0; lin = 0; jac = 0; oth = 0;
            end
            t_qp(end+1)    = qp; %#ok<AGROW>
            t_lin(end+1)   = lin; %#ok<AGROW>
            t_jac(end+1)   = jac; %#ok<AGROW>
            t_other(end+1) = oth; %#ok<AGROW>
        end

        x_pos = 1:numel(names);
        bar_data = [t_qp; t_lin; t_jac; t_other]';
        b = bar(x_pos, bar_data, 'stacked');

        bar_colors = [0.2 0.6 0.9;   % QP solve
                      0.9 0.5 0.2;   % Linearization (acados)
                      0.6 0.3 0.7;   % Jacobian (CasADi + expm)
                      0.7 0.7 0.7];  % Other
        for k = 1:4
            b(k).FaceColor = bar_colors(k,:);
        end

        set(gca, 'XTick',x_pos, 'XTickLabel',names);
        ylabel('Time (\mus)');
        legend({'QP solve','Linearization (acados)','Jacobian + expm (CasADi)','Other'}, ...
            'Location','best');

        yline(fig5.dt_mpc*1e6, 'r--', 'LineWidth',1.5, ...
            'Label','Real-time budget','LabelHorizontalAlignment','left','HandleVisibility','off');

        title(sprintf('Timing Breakdown — N=%d, \\Deltat_{mpc}=%.0f\\mus', ...
            fig5.N, fig5.dt_mpc*1e6));
        saveFig(gcf, out_dir, 'timing_breakdown', save_pdf);
    end
end

%% ================================================================
%% FIG 6: Convergence — semilogy error norm
%% ================================================================

if plot_convergence
    [data, ok] = loadRuns(res_dir, fig6.controllers, fig6.N, fig6.dt_mpc);
    if ok
        figure('Name','Convergence','Units','normalized','Position',[0.15 0.15 0.6 0.45]);
        hold on; grid on; box on;

        for i = 1:numel(data)
            r = data{i};
            col = ctrl_colors(r.controller);
            % Weighted state error norm (position in mm, angles in deg)
            err = (r.x - xEq) .* state_scale(:);
            err_norm = vecnorm(err, 2, 1);
            semilogy(r.t*1e3, err_norm, 'Color',col, 'LineWidth',1.4, ...
                'DisplayName',upper(r.controller));
        end

        xlabel('Time (ms)'); ylabel('Weighted state error norm (log)');
        legend('Location','best');
        title(sprintf('Convergence — N=%d, \\Deltat_{mpc}=%.0f\\mus', ...
            fig6.N, fig6.dt_mpc*1e6));
        saveFig(gcf, out_dir, 'convergence', save_pdf);
    end
end

%% ================================================================
%% FIG 7: Minimum Horizon for Convergence
%% ================================================================

if plot_min_horizon
    figure('Name','Min Horizon','Units','normalized','Position',[0.15 0.2 0.55 0.45]);

    % Left: bar chart of minimum N per controller
    % Right: convergence status grid (controller x N)

    subplot(1,2,1); hold on; grid on; box on;
    min_N = [];
    bar_names = {};
    bar_cols = [];

    for ic = 1:numel(fig7.controllers)
        cname = fig7.controllers{ic};
        found = false;
        for N = fig7.N_list
            f = fullfile(res_dir, getFilename(cname, N, fig7.dt_mpc));
            if ~isfile(f), continue; end
            d = load(f);
            if ~d.diverged
                min_N(end+1) = N; %#ok<AGROW>
                bar_names{end+1} = upper(cname); %#ok<AGROW>
                bar_cols = [bar_cols; ctrl_colors(cname)]; %#ok<AGROW>
                found = true;
                break;
            end
        end
        if ~found
            min_N(end+1) = NaN; %#ok<AGROW>
            bar_names{end+1} = upper(cname); %#ok<AGROW>
            bar_cols = [bar_cols; ctrl_colors(cname)]; %#ok<AGROW>
        end
    end

    x_pos = 1:numel(bar_names);
    b = bar(x_pos, min_N, 0.5);
    b.FaceColor = 'flat';
    for i = 1:numel(bar_names)
        b.CData(i,:) = bar_cols(i,:);
    end
    set(gca, 'XTick',x_pos, 'XTickLabel',bar_names);
    ylabel('Minimum N_{horizon}');
    title('Minimum Horizon for Convergence');

    % Right: grid showing converged (green) / diverged (red) per (ctrl, N)
    subplot(1,2,2); hold on; box on;

    n_ctrl = numel(fig7.controllers);
    N_list = fig7.N_list;
    n_N = numel(N_list);
    status = nan(n_ctrl, n_N);  % 1=converged, 0=diverged, NaN=missing

    for ic = 1:n_ctrl
        cname = fig7.controllers{ic};
        for iN = 1:n_N
            f = fullfile(res_dir, getFilename(cname, N_list(iN), fig7.dt_mpc));
            if ~isfile(f), continue; end
            d = load(f);
            status(ic, iN) = ~d.diverged;
        end
    end

    imagesc(N_list, 1:n_ctrl, status);
    colormap(gca, [0.9 0.3 0.3; 0.3 0.8 0.3]);  % red=diverged, green=converged
    caxis([0 1]);
    set(gca, 'YTick', 1:n_ctrl, 'YTickLabel', upper(fig7.controllers));
    xlabel('N_{horizon}');
    title('Convergence Status');

    sgtitle(sprintf('Horizon Convergence Study — \\Deltat_{mpc}=%.0f\\mus', ...
        fig7.dt_mpc*1e6), 'FontSize',13);
    saveFig(gcf, out_dir, 'min_horizon', save_pdf);
end

%% ================================================================
%% CONSOLE SUMMARY
%% ================================================================

fprintf('\n%-8s  %-4s  %-8s  %-5s  %12s  %10s  %10s\n', ...
    'Ctrl','N','dt_mpc','Div?','Cost','RMS(mm)','Ctrl(us)');
fprintf('%s\n', repmat('-',1,70));

% Print for fig1 config (the main comparison)
for ic = 1:numel(fig1.controllers)
    cname = fig1.controllers{ic};
    f = fullfile(res_dir, getFilename(cname, fig1.N, fig1.dt_mpc));
    if ~isfile(f), continue; end
    r = load(f);
    div_str = 'no'; if r.diverged, div_str = 'YES'; end
    pos_err = r.x(1:3,:) - xEq(1:3);
    rms_mm  = rms(vecnorm(pos_err,2,1))*1e3;
    ctrl_us = mean(getCtrlTime(r))*1e6;
    fprintf('%-8s  N=%-2d  %4.0f us   %-5s  %12.4g  %10.4f  %10.1f\n', ...
        upper(cname), fig1.N, fig1.dt_mpc*1e6, div_str, ...
        sum(r.cost), rms_mm, ctrl_us);
end

fprintf('\nDone.\n');

%% ================================================================
%% LOCAL FUNCTIONS
%% ================================================================

function [data, ok] = loadRuns(res_dir, controllers, N_list, dt_mpc)
%LOADRUNS  Load result .mat files. Print missing ones and continue.
    data = {};
    missing = {};
    for ic = 1:numel(controllers)
        cname = controllers{ic};
        for N = N_list
            f = fullfile(res_dir, getFilename(cname, N, dt_mpc));
            if isfile(f)
                d = load(f);
                d.file = f;
                data{end+1} = d; %#ok<AGROW>
            else
                missing{end+1} = sprintf('  %s  (run: simMain with %s, N=%d, dt=%.0fus)', ...
                    getFilename(cname, N, dt_mpc), cname, N, dt_mpc*1e6); %#ok<AGROW>
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
    ok = ~isempty(data);
end

function t = getCtrlTime(r)
%GETCTRLTIME  Extract total controller compute time from a result struct.
%   For SOL-NMPC: jac_time (compiled CasADi linearization + expm) + OCP solve.
%   For LMPC/NMPC: OCP solve only.
%   With lin_method='compiled', jac_time is a compiled CasADi C call —
%   comparable to acados internal timers (no MATLAB interpreter overhead).
    if isfield(r, 'jac_time') && isfield(r, 'lin_method') && strcmp(r.lin_method, 'compiled')
        % Compiled CasADi: fair to include jac_time in total
        t = r.jac_time + r.ocp_time_tot;
    elseif isfield(r, 'ocp_time_tot')
        t = r.ocp_time_tot;
    elseif isfield(r, 'ctrl_time')
        t = r.ctrl_time;
    else
        t = 0;
    end
end

function saveFig(fig, out_dir, name, do_save)
%SAVEFIG  Export figure as PDF if saving is enabled.
    if ~do_save, return; end
    fname = fullfile(out_dir, [name '.pdf']);
    exportgraphics(fig, fname, 'ContentType','vector');
    fprintf('Saved: %s\n', fname);
end
