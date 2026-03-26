%% simSweepAnalysis — Parse and visualise parameter sweep results
%
% Reads all results produced by simParameterSweep and creates summary
% plots comparing NMPC, LMPC, SOL-MPC across N_horizon and dt_mpc.

clear; clc;

%% ==========================================================
%% Settings — must match simParameterSweep
%% ==========================================================

results_dir = 'results';

N_list     = [10, 20, 30];
dt_list_us = 1000:1000:5000;          % microseconds
dt_list_s  = dt_list_us * 1e-6;       % seconds

controllers = struct( ...
    'name',    {'NMPC',              'LMPC',             'SOL-MPC'}, ...
    'pattern', {'ERK4s1_nmpc',       'DISC_lmpc',        'DISC_solmpc'});

nN  = numel(N_list);
nDt = numel(dt_list_us);
nC  = numel(controllers);

%% ==========================================================
%% Load all results
%% ==========================================================

% Pre-allocate summary tables (NaN = file missing)
diverged    = nan(nC, nN, nDt);
cost_final  = nan(nC, nN, nDt);
pos_err_rms = nan(nC, nN, nDt);   % RMS position error vs equilibrium (m)
pos_err_max = nan(nC, nN, nDt);   % max position error (m)
ocp_mean_us = nan(nC, nN, nDt);   % mean OCP solve time (us)
qp_mean_us  = nan(nC, nN, nDt);   % mean QP solve time (us)
step_mean_us= nan(nC, nN, nDt);   % mean total step time (us)

for ic = 1:nC
    for iN = 1:nN
        for iDt = 1:nDt
            fname = sprintf('res_N%d_dt%dus_%s.mat', ...
                N_list(iN), dt_list_us(iDt), controllers(ic).pattern);
            fpath = fullfile(results_dir, fname);

            if ~isfile(fpath)
                fprintf('  MISSING: %s\n', fname);
                continue;
            end

            d = load(fpath);

            diverged(ic, iN, iDt) = d.diverged;

            % Cost
            if isfield(d, 'cost')
                cost_final(ic, iN, iDt) = sum(d.cost);
            end

            % Position error vs equilibrium (states 1:3)
            if size(d.x,1) == 10
                x10_pos = [1 2 3];
            else
                x10_pos = [1 2 3];
            end
            pos_err = d.x(x10_pos,:) - d.xEq(x10_pos);
            pos_err_rms(ic, iN, iDt) = rms(vecnorm(pos_err, 2, 1));
            pos_err_max(ic, iN, iDt) = max(vecnorm(pos_err, 2, 1));

            % Timing
            ocp_mean_us(ic, iN, iDt)  = mean(d.ocp_time_tot) * 1e6;
            qp_mean_us(ic, iN, iDt)   = mean(d.ocp_time_qp) * 1e6;
            step_mean_us(ic, iN, iDt) = mean(d.step_time_tot) * 1e6;
        end
    end
end

fprintf('\nLoaded %d / %d result files.\n', ...
    sum(~isnan(diverged), 'all'), nC*nN*nDt);

%% ==========================================================
%% Helper: make dt_mpc / N_horizon tick labels
%% ==========================================================

dt_labels = arrayfun(@(d) sprintf('%g', d*1e3), dt_list_s, 'UniformOutput', false);
N_labels  = arrayfun(@(n) sprintf('%d', n), N_list, 'UniformOutput', false);
ctrl_names = {controllers.name};

%% ==========================================================
%% FIGURE 1: Stability map (diverged / converged)
%% ==========================================================

figure(1); clf;
set(gcf, 'Name', 'Stability Map', 'Position', [50 400 1100 350]);

for ic = 1:nC
    subplot(1, nC, ic); hold on;
    data = squeeze(diverged(ic, :, :));   % nN x nDt

    imagesc(1:nDt, 1:nN, data);
    colormap(gca, [0.3 0.75 0.35; 0.85 0.25 0.25]);  % green=ok, red=diverged
    caxis([0 1]);

    % Annotate cells
    for iN = 1:nN
        for iDt = 1:nDt
            if isnan(data(iN, iDt))
                text(iDt, iN, '?', 'HorizontalAlignment', 'center', 'FontSize', 12);
            elseif data(iN, iDt)
                text(iDt, iN, 'DIV', 'HorizontalAlignment', 'center', ...
                    'Color', 'w', 'FontWeight', 'bold', 'FontSize', 9);
            else
                text(iDt, iN, 'OK', 'HorizontalAlignment', 'center', ...
                    'Color', 'w', 'FontWeight', 'bold', 'FontSize', 9);
            end
        end
    end

    set(gca, 'XTick', 1:nDt, 'XTickLabel', dt_labels, ...
             'YTick', 1:nN,  'YTickLabel', N_labels);
    xlabel('dt_{mpc} (ms)'); ylabel('N_{horizon}');
    title(ctrl_names{ic});
    axis tight;
end
sgtitle('Stability Map', 'FontSize', 14, 'FontWeight', 'bold');

%% ==========================================================
%% FIGURE 2: Cumulative cost (bar groups)
%% ==========================================================

figure(2); clf;
set(gcf, 'Name', 'Cumulative Cost', 'Position', [50 50 1200 500]);

for iN = 1:nN
    subplot(1, nN, iN); hold on; grid on; box on;

    costs = squeeze(cost_final(:, iN, :));   % nC x nDt
    % Replace diverged runs with NaN for cleaner plot
    div = squeeze(diverged(:, iN, :));
    costs(div == 1) = NaN;

    b = bar(dt_list_us/1000, costs', 'grouped');
    for ic = 1:nC
        b(ic).DisplayName = ctrl_names{ic};
    end

    xlabel('dt_{mpc} (ms)');
    ylabel('Cumulative cost');
    title(sprintf('N = %d', N_list(iN)));
    set(gca, 'YScale', 'log');
    if iN == 1, legend('Location', 'best'); end
end
sgtitle('Cumulative Cost (converged runs only)', 'FontSize', 14, 'FontWeight', 'bold');

%% ==========================================================
%% FIGURE 3: Position tracking error
%% ==========================================================

figure(3); clf;
set(gcf, 'Name', 'Position Error', 'Position', [100 350 1200 500]);

for iN = 1:nN
    subplot(1, nN, iN); hold on; grid on; box on;

    rms_err = squeeze(pos_err_rms(:, iN, :)) * 1e3;  % mm
    div = squeeze(diverged(:, iN, :));
    rms_err(div == 1) = NaN;

    b = bar(dt_list_us/1000, rms_err', 'grouped');
    for ic = 1:nC
        b(ic).DisplayName = ctrl_names{ic};
    end

    xlabel('dt_{mpc} (ms)');
    ylabel('RMS position error (mm)');
    title(sprintf('N = %d', N_list(iN)));
    if iN == 1, legend('Location', 'best'); end
end
sgtitle('RMS Position Error vs Equilibrium (converged only)', 'FontSize', 14, 'FontWeight', 'bold');

%% ==========================================================
%% FIGURE 4: OCP solve time
%% ==========================================================

figure(4); clf;
set(gcf, 'Name', 'OCP Solve Time', 'Position', [150 250 1200 500]);

for iN = 1:nN
    subplot(1, nN, iN); hold on; grid on; box on;

    t_ocp = squeeze(ocp_mean_us(:, iN, :));   % nC x nDt

    b = bar(dt_list_us/1000, t_ocp', 'grouped');
    for ic = 1:nC
        b(ic).DisplayName = ctrl_names{ic};
    end

    % Draw real-time budget line
    plot(dt_list_us/1000, dt_list_us, 'k--', 'LineWidth', 1.5, ...
        'DisplayName', 'Real-time budget');

    xlabel('dt_{mpc} (ms)');
    ylabel('Mean OCP time (\mus)');
    title(sprintf('N = %d', N_list(iN)));
    if iN == 1, legend('Location', 'best'); end
end
sgtitle('Mean OCP Solve Time', 'FontSize', 14, 'FontWeight', 'bold');

%% ==========================================================
%% FIGURE 5: Real-time factor
%% ==========================================================

figure(5); clf;
set(gcf, 'Name', 'Real-Time Factor', 'Position', [200 150 1200 500]);

for iN = 1:nN
    subplot(1, nN, iN); hold on; grid on; box on;

    t_step = squeeze(step_mean_us(:, iN, :));   % nC x nDt
    rt_factor = dt_list_us ./ t_step;            % >1 means real-time feasible

    b = bar(dt_list_us/1000, rt_factor', 'grouped');
    for ic = 1:nC
        b(ic).DisplayName = ctrl_names{ic};
    end

    yline(1, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

    xlabel('dt_{mpc} (ms)');
    ylabel('Real-time factor');
    title(sprintf('N = %d', N_list(iN)));
    if iN == 1, legend('Location', 'best'); end
end
sgtitle('Real-Time Factor (>1 = feasible)', 'FontSize', 14, 'FontWeight', 'bold');

%% ==========================================================
%% Console summary table
%% ==========================================================

fprintf('\n%-8s  %-4s  %-8s  %-5s  %12s  %10s  %10s  %10s\n', ...
    'Ctrl', 'N', 'dt_mpc', 'Div?', 'Cost', 'RMS(mm)', 'OCP(us)', 'Step(us)');
fprintf('%s\n', repmat('-', 1, 80));

for ic = 1:nC
    for iN = 1:nN
        for iDt = 1:nDt
            div_str = '?';
            if ~isnan(diverged(ic,iN,iDt))
                if diverged(ic,iN,iDt), div_str = 'YES'; else, div_str = 'no'; end
            end
            fprintf('%-8s  N=%-2d  %4.0f us   %-5s  %12.4g  %10.4f  %10.1f  %10.1f\n', ...
                ctrl_names{ic}, N_list(iN), dt_list_us(iDt), div_str, ...
                cost_final(ic,iN,iDt), ...
                pos_err_rms(ic,iN,iDt)*1e3, ...
                ocp_mean_us(ic,iN,iDt), ...
                step_mean_us(ic,iN,iDt));
        end
    end
end
