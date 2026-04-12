function simMainPlot(results, xEq, uEq)
%SIMMAINPLOT  Generate comparison plots from simulation results.
%
%   simMainPlot(results, xEq, uEq)
%
%   results  — cell array, each element is a struct with fields from sim_data
%   xEq      — equilibrium state (10x1)
%   uEq      — equilibrium input (4x1)

%% ================================================================
%% PLOT ENABLE FLAGS — set false to skip
%% ================================================================

plot_trajectories = true;   % states + inputs + cumulative cost (combined)
plot_timing       = true;   % bar chart (single config only)
plot_cost_vs_N    = true;   % final cost vs N_horizon (per dt_mpc)
plot_summary      = true;   % console table

% Merge all N_horizon values into one figure per dt_mpc.
% Color = controller type, line style = N_horizon (solid for largest, more
% dashed for smaller).  When false, one figure per (N_horizon, dt_mpc).
merge_N_horizon   = true;

%% ================================================================
%% SETUP
%% ================================================================

nx = numel(xEq);
nR = numel(results);

% Color map for controllers
ctrl_colors = containers.Map();
ctrl_colors('lqr')    = [0.5 0.5 0.5];
ctrl_colors('lmpc')   = [0.0 0.45 0.74];
ctrl_colors('solmpc') = [0.85 0.33 0.1];
ctrl_colors('nmpc')   = [0.47 0.67 0.19];

state_names = {'x','y','z','\phi','\theta', '\dot{x}','\dot{y}','\dot{z}','\dot{\phi}','\dot{\theta}'};
state_units = {'mm','mm','mm','deg','deg','m/s','m/s','m/s','rad/s','rad/s'};
state_scale = [1e3, 1e3, 1e3, 180/pi, 180/pi, 1, 1, 1, 1, 1];

% --- Build line-style map for N_horizon values (largest = solid) ---
all_N = sort(unique(cellfun(@(r) r.N_horizon, results)), 'ascend');
dash_styles = {':', '-.', '--', '-'};  % most-dashed → solid
% Assign from the right so the largest N gets '-'
n_styles = numel(dash_styles);
N_line = containers.Map('KeyType','int32','ValueType','char');
for k = 1:numel(all_N)
    idx = n_styles - (numel(all_N) - k);  % right-align into dash_styles
    idx = max(1, idx);
    N_line(all_N(k)) = dash_styles{idx};
end

% --- Group results by (N_horizon, dt_mpc) ---
configs = struct([]);
for i = 1:nR
    r = results{i};
    key = sprintf('N%d_dt%.0f', r.N_horizon, r.dt_mpc*1e6);
    found = false;
    for j = 1:numel(configs)
        if strcmp(configs(j).key, key)
            configs(j).runs{end+1} = r;
            found = true;
            break;
        end
    end
    if ~found
        c.key       = key;
        c.N_horizon = r.N_horizon;
        c.dt_mpc    = r.dt_mpc;
        c.runs      = {r};
        if isempty(configs)
            configs = c;
        else
            configs(end+1) = c; %#ok<AGROW>
        end
    end
end

% --- Group configs by dt_mpc (for merged mode) ---
dt_mpc_vals = unique(cellfun(@(r) r.dt_mpc, results));

%% ================================================================
%% FIGURE 1: Combined — states, inputs, cumulative cost
%% ================================================================

if plot_trajectories
if merge_N_horizon
    % One figure per (controller, dt_mpc), N_horizon as different colors
    N_cmap = lines(numel(all_N));  % one color per N value

    ctrl_names = unique(cellfun(@(r) r.controller, results, 'UniformOutput', false));
    for i_dt = 1:numel(dt_mpc_vals)
        dt_val = dt_mpc_vals(i_dt);
        for ic = 1:numel(ctrl_names)
            cname = ctrl_names{ic};

            % Collect runs for this controller + dt_mpc
            runs = {};
            for ir = 1:nR
                r = results{ir};
                if r.dt_mpc == dt_val && strcmp(r.controller, cname)
                    runs{end+1} = r; %#ok<AGROW>
                end
            end
            if isempty(runs), continue; end

            fig = figure('Name', sprintf('%s dt=%.0fus', upper(cname), dt_val*1e6), ...
                         'Units','normalized', 'Position',[0.02 0.05 0.96 0.8]);

            plotTrajectoriesMerged(fig, runs, xEq, all_N, N_cmap, ...
                state_names, state_units, state_scale);

            sgtitle(sprintf('%s — \\Deltat_{mpc} = %.0f \\mus', ...
                upper(cname), dt_val*1e6), 'FontSize', 13);
        end
    end
else
    % One figure per (N_horizon, dt_mpc) config
    for ic = 1:numel(configs)
        cfg = configs(ic);

        fig = figure('Name', sprintf('Overview %s', cfg.key), ...
                     'Units','normalized', 'Position',[0.02 0.02 0.96 0.92]);

        plotTrajectories(fig, cfg.runs, xEq, ctrl_colors, N_line, ...
            state_names, state_units, state_scale, false);

        sgtitle(sprintf('N = %d,  \\Deltat_{mpc} = %.0f \\mus', ...
            cfg.N_horizon, cfg.dt_mpc*1e6), 'FontSize', 13);
    end
end
end

%% ================================================================
%% FIGURE 2: Timing comparison (bar chart)
%% ================================================================

if plot_timing && numel(configs) == 1
    cfg = configs(1);
    fig = figure('Name', 'Timing', ...
                 'Units','normalized', 'Position',[0.2 0.15 0.5 0.4]);
    hold on; grid on; box on;

    names = {};
    means = [];
    medians = [];
    maxes = [];
    cols = [];

    for ir = 1:numel(cfg.runs)
        r = cfg.runs{ir};
        names{end+1} = upper(r.controller); %#ok<AGROW>
        cols = [cols; getColor(ctrl_colors, r.controller)]; %#ok<AGROW>

        if isfield(r, 'ocp_time_tot')
            t_ctrl = r.ocp_time_tot;
        elseif isfield(r, 'ctrl_time')
            t_ctrl = r.ctrl_time;
        else
            t_ctrl = zeros(size(r.sim_time_tot));
        end
        means(end+1)   = mean(t_ctrl) * 1e6; %#ok<AGROW>
        medians(end+1) = median(t_ctrl) * 1e6; %#ok<AGROW>
        maxes(end+1)   = max(t_ctrl) * 1e6; %#ok<AGROW>
    end

    x_pos = 1:numel(names);
    b = bar(x_pos, means, 0.5);
    b.FaceColor = 'flat';
    for i = 1:numel(names)
        b.CData(i,:) = cols(i,:);
    end

    % Add max as error bar
    errorbar(x_pos, means, zeros(size(means)), maxes - means, ...
        'k.', 'LineWidth', 1);

    set(gca, 'XTick', x_pos, 'XTickLabel', names);
    ylabel('Control compute time (\mus)');
    title(sprintf('Timing — N = %d,  \\Deltat_{mpc} = %.0f \\mus', ...
        cfg.N_horizon, cfg.dt_mpc*1e6));

    % Real-time budget line
    yline(cfg.dt_mpc * 1e6, 'r--', 'LineWidth', 1.5, ...
        'Label', 'Real-time budget', 'LabelHorizontalAlignment','left');
end

%% ================================================================
%% FIGURE 3: Final cumulative cost vs N_horizon (per dt_mpc)
%% ================================================================

if plot_cost_vs_N
for i_dt = 1:numel(dt_mpc_vals)
    dt_val = dt_mpc_vals(i_dt);
    figure('Name', sprintf('CostVsN dt=%.0fus', dt_val*1e6), ...
           'Units','normalized', 'Position',[0.15 0.2 0.55 0.45]);
    hold on; grid on; box on;

    ctrl_names = unique(cellfun(@(r) r.controller, results, 'UniformOutput', false));
    for ic = 1:numel(ctrl_names)
        cname = ctrl_names{ic};
        N_vals   = [];
        cost_vals = [];
        for ir = 1:nR
            r = results{ir};
            if r.dt_mpc == dt_val && strcmp(r.controller, cname)
                N_vals(end+1)    = r.N_horizon; %#ok<AGROW>
                cost_vals(end+1) = r.cost_cum(end); %#ok<AGROW>
            end
        end
        if isempty(N_vals), continue; end
        [N_vals, si] = sort(N_vals);
        cost_vals = cost_vals(si);
        col = getColor(ctrl_colors, cname);

        plot(N_vals, cost_vals, '-o', 'Color', col, 'LineWidth', 1.4, ...
            'MarkerFaceColor', col, 'MarkerSize', 6, ...
            'DisplayName', upper(cname));
    end

    xlabel('N_{horizon}');
    ylabel('Cumulative cost');
    legend('Location','best');
    title(sprintf('Final Cumulative Cost vs N — \\Deltat_{mpc} = %.0f \\mus', dt_val*1e6));
end
end

%% ================================================================
%% CONSOLE SUMMARY TABLE
%% ================================================================

if plot_summary
fprintf('\n%-8s  %-4s  %-8s  %-5s  %12s  %10s  %10s\n', ...
    'Ctrl', 'N', 'dt_mpc', 'Div?', 'Cost', 'RMS(mm)', 'Ctrl(us)');
fprintf('%s\n', repmat('-', 1, 70));

for i = 1:nR
    r = results{i};
    div_str = 'no';
    if r.diverged, div_str = 'YES'; end

    pos_err = r.x(1:3,:) - xEq(1:3);
    rms_mm  = rms(vecnorm(pos_err, 2, 1)) * 1e3;

    if isfield(r, 'ocp_time_tot')
        ctrl_us = mean(r.ocp_time_tot) * 1e6;
    elseif isfield(r, 'ctrl_time')
        ctrl_us = mean(r.ctrl_time) * 1e6;
    else
        ctrl_us = 0;
    end

    fprintf('%-8s  N=%-2d  %4.0f us   %-5s  %12.4g  %10.4f  %10.1f\n', ...
        upper(r.controller), r.N_horizon, r.dt_mpc*1e6, div_str, ...
        sum(r.cost), rms_mm, ctrl_us);
end
end

end

%% ================================================================
%% LOCAL HELPERS
%% ================================================================

function plotTrajectoriesMerged(fig, runs, xEq, all_N, N_cmap, ...
        state_names, state_units, state_scale)
%PLOTTRAJECTORIESMERGED  2-row layout per controller: 5 states, 5 inputs.
%   Color = N_horizon value, all lines solid.

    figure(fig);
    nRuns = numel(runs);

    % Build N → color map
    N_col = containers.Map('KeyType','int32','ValueType','any');
    for k = 1:numel(all_N)
        N_col(all_N(k)) = N_cmap(k,:);
    end

    % --- Row 1: states (5 subplots) ---
    for s = 1:5
        subplot(2, 5, s); hold on; grid on; box on;
        for ir = 1:nRuns
            r = runs{ir};
            col = N_col(int32(r.N_horizon));
            plot(r.t*1e3, (r.x(s,:) - xEq(s)) * state_scale(s), ...
                'Color', col, 'LineWidth', 1.2, ...
                'DisplayName', sprintf('N=%d', r.N_horizon));
        end
        ylabel(sprintf('%s (%s)', state_names{s}, state_units{s}));
        xlabel('Time (ms)');
        if s == 1, legend('Location','best'); end
        title(state_names{s});
    end

    % --- Row 2: inputs (4 subplots) + cost_cum ---
    for s = 1:4
        subplot(2, 5, 5 + s); hold on; grid on; box on;
        for ir = 1:nRuns
            r = runs{ir};
            col = N_col(int32(r.N_horizon));
            plot(r.t*1e3, r.u(s,:), ...
                'Color', col, 'LineWidth', 1.0, ...
                'HandleVisibility','off');
        end
        ylabel(sprintf('u_%d (A)', s));
        xlabel('Time (ms)');
        title(sprintf('Solenoid %d', s));
    end

    % Cumulative cost in last slot
    subplot(2, 5, 10); hold on; grid on; box on;
    for ir = 1:nRuns
        r = runs{ir};
        col = N_col(int32(r.N_horizon));
        plot(r.t*1e3, r.cost_cum, ...
            'Color', col, 'LineWidth', 1.2, ...
            'HandleVisibility','off');
    end
    xlabel('Time (ms)'); ylabel('Cum. cost');
    title('Cumulative Cost');
end

function plotTrajectories(fig, runs, xEq, ctrl_colors, N_line, ...
        state_names, state_units, state_scale, show_N_in_legend)
%PLOTTRAJECTORIES  4x3 grid: 5 states + z(abs) + 4 inputs + cost.

    figure(fig);
    nRuns = numel(runs);

    for s = 1:5
        subplot(4, 3, s); hold on; grid on; box on;
        for ir = 1:nRuns
            r = runs{ir};
            col = getColor(ctrl_colors, r.controller);
            ls  = getLineStyle(N_line, r.N_horizon);
            lbl = makeLegendLabel(r, show_N_in_legend);
            plot(r.t*1e3, (r.x(s,:) - xEq(s)) * state_scale(s), ...
                'Color', col, 'LineStyle', ls, 'LineWidth', 1.2, ...
                'DisplayName', lbl);
        end
        ylabel(sprintf('%s (%s)', state_names{s}, state_units{s}));
        xlabel('Time (ms)');
        if s == 1, legend('Location','best'); end
        title(state_names{s});
    end

    subplot(4, 3, 6); hold on; grid on; box on;
    for ir = 1:nRuns
        r = runs{ir};
        col = getColor(ctrl_colors, r.controller);
        ls  = getLineStyle(N_line, r.N_horizon);
        plot(r.t*1e3, r.x(3,:)*1e3, ...
            'Color', col, 'LineStyle', ls, 'LineWidth', 1.2, ...
            'HandleVisibility','off');
    end
    yline(xEq(3)*1e3, 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
    ylabel('z (mm)'); xlabel('Time (ms)');
    title('z (absolute)');

    for s = 1:4
        subplot(4, 3, 6 + s); hold on; grid on; box on;
        for ir = 1:nRuns
            r = runs{ir};
            col = getColor(ctrl_colors, r.controller);
            ls  = getLineStyle(N_line, r.N_horizon);
            plot(r.t*1e3, r.u(s,:), ...
                'Color', col, 'LineStyle', ls, 'LineWidth', 1.0, ...
                'HandleVisibility','off');
        end
        ylabel(sprintf('u_%d (A)', s));
        xlabel('Time (ms)');
        title(sprintf('Solenoid %d', s));
    end

    subplot(4, 3, 11); hold on; grid on; box on;
    for ir = 1:nRuns
        r = runs{ir};
        col = getColor(ctrl_colors, r.controller);
        ls  = getLineStyle(N_line, r.N_horizon);
        plot(r.t*1e3, r.cost_cum, ...
            'Color', col, 'LineStyle', ls, 'LineWidth', 1.2, ...
            'HandleVisibility','off');
    end
    xlabel('Time (ms)'); ylabel('Cumulative cost');
    title('Cumulative Cost');
end

function col = getColor(cmap, name)
    if cmap.isKey(name)
        col = cmap(name);
    else
        col = [0 0 0];
    end
end

function ls = getLineStyle(N_line, N)
    key = int32(N);
    if N_line.isKey(key)
        ls = N_line(key);
    else
        ls = '-';
    end
end

function lbl = makeLegendLabel(r, show_N)
    if show_N
        lbl = sprintf('%s N=%d', upper(r.controller), r.N_horizon);
    else
        lbl = upper(r.controller);
    end
end
