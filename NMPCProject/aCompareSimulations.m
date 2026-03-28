%% simCompare — Compare simulation results across controllers

% clear; clc;

%% ==========================================================
%% Settings
%% ==========================================================

assert(exist("files", "var"));

T_end = 0;   % 0 = auto

% Error threshold markers: mark points where |error| vs first file exceeds
% these thresholds. Set to 0 or Inf to disable.
err_thresh_mm  = 4;    % position threshold (mm)
err_thresh_deg = Inf;  % orientation threshold (deg) — disabled by default

%% ==========================================================
%% Load files
%% ==========================================================

for i = 1:numel(files)
    files(i) = fullfile(out_folder, files(i));
end

nFiles = numel(files);
assert(nFiles >= 2, 'Need at least 2 files to compare.');

data  = cell(1, nFiles);
names = cell(1, nFiles);

for f = 1:nFiles
    if ~isfile(files(f)), error("File not found: %s", files(f)); end
    data{f} = load(files(f));

    % Use controller field if available, else fall back to filename
    % if isfield(data{f}, 'controller')
    %     names{f} = upper(data{f}.controller);
    % else
        [~, n, ~] = fileparts(files(f));
        names{f} = strrep(n, '_', ' ');
    % end

    if isfield(data{f}, 'diverged') && data{f}.diverged
        fprintf('  WARNING: %s DIVERGED\n', names{f});
        names{f} = [names{f} ' (diverged)'];
    end

    % Expand 10-state (reduced, no yaw) to 12-state for plotting
    if size(data{f}.x, 1) == 10
        N = size(data{f}.x, 2);
        data{f}.x = [data{f}.x(1:5,:); zeros(1,N); ...
                     data{f}.x(6:10,:); zeros(1,N)];
        fprintf('  %s: expanded 10-state -> 12-state (yaw/wz = 0)\n', names{f});
    end
    if isfield(data{f}, 'xEq') && numel(data{f}.xEq) == 10
        data{f}.xEq = [data{f}.xEq(1:5); 0; data{f}.xEq(6:10); 0];
    end
end

xEq = data{1}.xEq;

if T_end == 0
    T_end = min(cellfun(@(d) d.t(end), data));
end

% Trim to common time window
for f = 1:nFiles
    mask = data{f}.t <= T_end + 1e-12;
    data{f}.x = data{f}.x(:, mask);
    data{f}.u = data{f}.u(:, mask);
    data{f}.t = data{f}.t(mask);
    fprintf('%s: %d points, dt=%.6f\n', names{f}, length(data{f}.t), data{f}.dt);
end
fprintf('T_end = %.4f s\n', T_end);

%% ==========================================================
%% Normalize initial state (static offset vs first file)
%% ==========================================================

x0_ref = data{1}.x(:,1);

for f = 2:nFiles
    offset = x0_ref - data{f}.x(:,1);
    data{f}.x = data{f}.x + offset;
end

%% ==========================================================
%% Units & labels
%% ==========================================================

pos_scale = 1e3;
ang_scale = 180/pi;
pos_unit  = 'mm';
ang_unit  = 'deg';

state_names = {'x','y','z','roll','pitch','yaw', ...
               'vx','vy','vz','\omega_x','\omega_y','\omega_z'};

colors = lines(nFiles);

%% ==========================================================
%% FIGURE 1: Position comparison
%% ==========================================================

figure(1); clf;
set(gcf,'Name','Position Comparison','Position',[50 400 1200 500]);

for i = 1:3
    subplot(1,3,i); hold on; grid on; box on;
    for f = 1:nFiles
        plot(data{f}.t, data{f}.x(i,:)*pos_scale, ...
            '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',names{f});
    end
    if isfinite(err_thresh_mm) && err_thresh_mm > 0
        ref_t = data{1}.t;
        for f = 2:nFiles
            xi = interp1(data{f}.t(:), data{f}.x(i,:)', ref_t(:), 'pchip')';
            bad = abs(data{1}.x(i,:) - xi)*pos_scale > err_thresh_mm;
            if any(bad)
                scatter(ref_t(bad), xi(bad)*pos_scale, ...
                    20, colors(f,:), 'x', 'LineWidth', 1.5, 'HandleVisibility','off');
            end
        end
    end
    yline(xEq(i)*pos_scale,'k:','HandleVisibility','off');
    xlim([0 T_end]);
    xlabel('Time (s)'); ylabel(pos_unit); title(state_names{i});
    legend('Location','best');
end
sgtitle('Position Comparison','FontSize',14,'FontWeight','bold');

%% ==========================================================
%% FIGURE 2: Orientation comparison
%% ==========================================================

figure(2); clf;
set(gcf,'Name','Orientation Comparison','Position',[50 50 1200 500]);

for i = 1:3
    subplot(1,3,i); hold on; grid on; box on;
    si = i + 3;
    for f = 1:nFiles
        plot(data{f}.t, data{f}.x(si,:)*ang_scale, ...
            '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',names{f});
    end
    if isfinite(err_thresh_deg) && err_thresh_deg > 0
        ref_t = data{1}.t;
        for f = 2:nFiles
            xi = interp1(data{f}.t(:), data{f}.x(si,:)', ref_t(:), 'pchip')';
            bad = abs(data{1}.x(si,:) - xi)*ang_scale > err_thresh_deg;
            if any(bad)
                scatter(ref_t(bad), xi(bad)*ang_scale, ...
                    20, colors(f,:), 'x', 'LineWidth', 1.5, 'HandleVisibility','off');
            end
        end
    end
    yline(xEq(si)*ang_scale,'k:','HandleVisibility','off');
    xlim([0 T_end]);
    xlabel('Time (s)'); ylabel(ang_unit); title(state_names{si});
    legend('Location','best');
end
sgtitle('Orientation Comparison','FontSize',14,'FontWeight','bold');

%% ==========================================================
%% FIGURE 3: Velocity comparison
%% ==========================================================

figure(3); clf;
set(gcf,'Name','Velocity Comparison','Position',[100 300 1200 500]);

vel_labels = {'m/s','m/s','m/s','rad/s','rad/s','rad/s'};

for i = 1:6
    subplot(2,3,i); hold on; grid on; box on;
    si = i + 6;
    for f = 1:nFiles
        plot(data{f}.t, data{f}.x(si,:), ...
            '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',names{f});
    end
    xlim([0 T_end]);
    xlabel('Time (s)'); ylabel(vel_labels{i}); title(state_names{si});
    if i == 1, legend('Location','best'); end
end
sgtitle('Velocity Comparison','FontSize',14,'FontWeight','bold');

%% ==========================================================
%% FIGURE 4: Error (all vs first file)
%% ==========================================================

figure(4); clf;
set(gcf,'Name','Error','Position',[150 200 1200 600]);

ref = data{1};

subplot(2,2,1); hold on; grid on; box on;
subplot(2,2,2); hold on; grid on; box on;
subplot(2,2,3); hold on; grid on; box on;
subplot(2,2,4); hold on; grid on; box on;

for f = 2:nFiles
    x_interp = interp1(data{f}.t, data{f}.x', ref.t, 'pchip')';
    err   = ref.x - x_interp;
    label = sprintf('%s - %s', names{1}, names{f});

    subplot(2,2,1);
    plot(ref.t, max(abs(err(1:3,:)),[],1)*pos_scale, ...
        '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',label);
    subplot(2,2,2);
    plot(ref.t, max(abs(err(4:6,:)),[],1)*ang_scale, ...
        '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',label);
    subplot(2,2,3);
    plot(ref.t, max(abs(err(7:9,:)),[],1), ...
        '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',label);
    subplot(2,2,4);
    plot(ref.t, max(abs(err(10:12,:)),[],1), ...
        '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',label);

    fprintf('\n=== Error: %s ===\n', label);
    fprintf('Position  max: %.4e m  (%.4e %s)\n', ...
        max(abs(err(1:3,:)),[],'all'), max(abs(err(1:3,:)),[],'all')*pos_scale, pos_unit);
    fprintf('Angle     max: %.4e rad (%.4e %s)\n', ...
        max(abs(err(4:6,:)),[],'all'), max(abs(err(4:6,:)),[],'all')*ang_scale, ang_unit);
    fprintf('Lin vel   max: %.4e m/s\n',  max(abs(err(7:9,:)),  [],'all'));
    fprintf('Ang vel   max: %.4e rad/s\n', max(abs(err(10:12,:)),[],'all'));
end

subplot(2,2,1); xlim([0 T_end]); ylabel(pos_unit);  title('Position error');          legend('Location','best');
subplot(2,2,2); xlim([0 T_end]); ylabel(ang_unit);  title('Orientation error');       legend('Location','best');
subplot(2,2,3); xlim([0 T_end]); xlabel('Time (s)'); ylabel('m/s');    title('Linear velocity error');  legend('Location','best');
subplot(2,2,4); xlim([0 T_end]); xlabel('Time (s)'); ylabel('rad/s');  title('Angular velocity error'); legend('Location','best');
sgtitle(sprintf('Error (relative to %s)', names{1}), 'FontSize',14,'FontWeight','bold');

%% ==========================================================
%% FIGURE 5: Timing comparison
%% ==========================================================

figure(5); clf;
set(gcf,'Name','Timing Comparison','Position',[200 150 1200 600]);

subplot(2,2,1); hold on; grid on; box on; title('Step time (ocp+sim)');
subplot(2,2,2); hold on; grid on; box on; title('OCP solve time');
subplot(2,2,3); hold on; grid on; box on; title('QP solve time');
subplot(2,2,4); hold on; grid on; box on; title('Plant sim time');

fprintf('\n=== Timing summary (acados internal, microseconds) ===\n');
fprintf('%-12s  %8s  %8s  %8s  |  %8s  %8s  %8s  |  %8s\n', ...
    'Controller','mean','median','max','ocp_mean','qp_mean','sim_mean','step_mean');

for f = 1:nFiles
    d = data{f};
    if ~isfield(d, 'step_time_tot'), continue; end

    t_mpc = (0:numel(d.step_time_tot)-1) * d.dt_mpc;

    subplot(2,2,1);
    plot(t_mpc, d.step_time_tot*1e6, '-','Color',colors(f,:),'LineWidth',1,'DisplayName',names{f});
    yline(mean(d.step_time_tot)*1e6, '--','Color',colors(f,:),'LineWidth',1,'HandleVisibility','off');

    subplot(2,2,2);
    plot(t_mpc, d.ocp_time_tot*1e6, '-','Color',colors(f,:),'LineWidth',1,'DisplayName',names{f});

    subplot(2,2,3);
    plot(t_mpc, d.ocp_time_qp*1e6, '-','Color',colors(f,:),'LineWidth',1,'DisplayName',names{f});

    subplot(2,2,4);
    plot(t_mpc, d.sim_time_tot*1e6, '-','Color',colors(f,:),'LineWidth',1,'DisplayName',names{f});

    fprintf('%-12s  %8.1f  %8.1f  %8.1f  |  %8.1f  %8.1f  %8.1f  |  %8.1f\n', ...
        names{f}, ...
        mean(d.ocp_time_tot)*1e6, median(d.ocp_time_tot)*1e6, max(d.ocp_time_tot)*1e6, ...
        mean(d.ocp_time_tot)*1e6, mean(d.ocp_time_qp)*1e6,    mean(d.sim_time_tot)*1e6, ...
        mean(d.step_time_tot)*1e6);
end

subplot(2,2,1); xlim([0 T_end]); xlabel('Time (s)'); ylabel('\mus'); legend('Location','best');
subplot(2,2,2); xlim([0 T_end]); xlabel('Time (s)'); ylabel('\mus'); legend('Location','best');
subplot(2,2,3); xlim([0 T_end]); xlabel('Time (s)'); ylabel('\mus'); legend('Location','best');
subplot(2,2,4); xlim([0 T_end]); xlabel('Time (s)'); ylabel('\mus'); legend('Location','best');
sgtitle('Timing Comparison','FontSize',14,'FontWeight','bold');

%% ==========================================================
%% FIGURE 6: Cost comparison
%% ==========================================================

figure(6); clf;
set(gcf,'Name','Cost Comparison','Position',[250 100 1000 450]);

subplot(1,2,1); hold on; grid on; box on; title('Stage cost per MPC step');
subplot(1,2,2); hold on; grid on; box on; title('Cumulative cost');

fprintf('\n=== Cost summary ===\n');

for f = 1:nFiles
    d = data{f};
    if ~isfield(d, 'cost'), continue; end

    t_mpc = (1:numel(d.cost)) * d.dt_mpc;

    subplot(1,2,1);
    plot(t_mpc, d.cost, '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',names{f});

    subplot(1,2,2);
    plot(t_mpc, d.cost_cum, '-','Color',colors(f,:),'LineWidth',1.5,'DisplayName',names{f});

    fprintf('%-12s  final cost: %.4g   cumulative: %.4g\n', ...
        names{f}, d.cost(end), d.cost_cum(end));
end

subplot(1,2,1); xlim([0 T_end]); xlabel('Time (s)'); ylabel('Cost'); legend('Location','best');
subplot(1,2,2); xlim([0 T_end]); xlabel('Time (s)'); ylabel('Cost'); legend('Location','best');
sgtitle('Cost Comparison','FontSize',14,'FontWeight','bold');

%% ==========================================================
%% FIGURE 7: Solenoid / actuator inputs
%% ==========================================================

figure(7); clf;
set(gcf,'Name','Solenoid Inputs','Position',[300 50 900 700]);

solenoid_names = {'Solenoid 1','Solenoid 2','Solenoid 3','Solenoid 4'};

for i = 1:4
    subplot(4,1,i); hold on; grid on; box on;
    for f = 1:nFiles
        plot(data{f}.t, data{f}.u(i,:), ...
            '-','Color',colors(f,:),'LineWidth',1.2,'DisplayName',names{f});
    end
    xlim([0 T_end]);
    ylabel('A');
    title(solenoid_names{i});
    if i == 1, legend('Location','best'); end
    if i < 4, set(gca,'XTickLabel',[]); end
end
xlabel('Time (s)');
sgtitle('Solenoid Inputs','FontSize',14,'FontWeight','bold');

%% ==========================================================
%% FIGURE 8: Cost breakdown per state/input
%% ==========================================================

% Reconstruct Q, R from getCost (must match getCost.m)
Q_diag = [1e5, 1e5, 1e8, 1e4, 1e4, 1e2, 1e2, 1e2, 1e2, 1e2];
R_diag = [1, 1, 1, 1];
w_diag = [Q_diag, R_diag];

% State indices after 10->12 expansion: remove yaw(6) and wz(12)
x10_idx = [1:5, 7:11];  % indices into the 12-state vector

cost_labels = {'x','y','z','roll','pitch','vx','vy','vz','\omega_x','\omega_y', ...
               'u_1','u_2','u_3','u_4'};

figure(8); clf;
set(gcf,'Name','Cost Breakdown','Position',[350 50 1400 700]);

for f = 1:nFiles
    d = data{f};
    xEq_f = d.xEq;

    % Compute per-component cost contribution at each time step
    x_err = d.x(x10_idx,:) - xEq_f(x10_idx);   % 10 x N
    u_err = d.u - d.uEq;                         % 4 x N
    y_err = [x_err; u_err];                       % 14 x N

    % Weighted squared error per component
    cost_per_comp = y_err.^2 .* w_diag(:);        % 14 x N

    % Downsample to MPC rate for cleaner plot
    n_sub_f = round(d.dt_mpc / d.dt);
    N_mpc_f = floor(size(y_err,2) / n_sub_f);
    t_mpc_f = (0:N_mpc_f-1) * d.dt_mpc;
    cost_mpc = zeros(14, N_mpc_f);
    for k = 1:N_mpc_f
        idx = (k-1)*n_sub_f + 1;
        cost_mpc(:,k) = cost_per_comp(:, idx);
    end

    subplot(1, nFiles, f); hold on; grid on; box on;
    area(t_mpc_f, cost_mpc', 'EdgeColor','none');
    xlim([0 T_end]);
    xlabel('Time (s)'); ylabel('Cost');
    title(names{f});
    set(gca,'YScale','log');
end

subplot(1,nFiles,1);
legend(cost_labels, 'Location','best','FontSize',7);
sgtitle('Cost Breakdown by Component','FontSize',14,'FontWeight','bold');
