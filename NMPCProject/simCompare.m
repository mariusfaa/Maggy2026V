%% --- Compare arbitrary number of simulation result files ---

% clear; clc;

%% ==========================================================
%% Settings
%% ==========================================================

files = [
    "results_ode_fast.mat"
    "results_ode_accurate.mat"
    % "results_ode.mat"
];


files = [
    "results_ode_fast.mat"
    "results_acados_reduced.mat"
    "results_acados_reduced2.mat"
    % "simresults/sim_fn_irk_4_1_1ms.mat"
    % "simresults/sim_fn_irk_4_1_5ms.mat"
    % "simresults/sim_fn_irk_4_1_7ms.mat"
    % "simresults/sim_fn_irk_4_1_10ms.mat"
];


T_end = 0.05;   % 0 = auto

% Error threshold markers: mark points where |error| vs first file exceeds
% these thresholds. Set to 0 or Inf to disable.
err_thresh_mm  = 4;     % position threshold (mm)
err_thresh_deg = Inf;   % orientation threshold (deg) — disabled by default

%% ==========================================================
%% Load files
%% ==========================================================

nFiles = numel(files);
assert(nFiles >= 2, 'Need at least 2 files to compare.');

data  = cell(1, nFiles);
names = cell(1, nFiles);

for f = 1:nFiles
    if ~isfile(files(f)), error("File not found: %s", files(f)); end
    data{f} = load(files(f));
    [~, n, ~] = fileparts(files(f));
    names{f} = strrep(n, '_', ' ');

    % Expand 10-state (reduced, no yaw) to 12-state for compatibility
    if size(data{f}.x, 1) == 10
        N = size(data{f}.x, 2);
        data{f}.x = [data{f}.x(1:5,:); zeros(1,N); ...
                     data{f}.x(6:10,:); zeros(1,N)];
        fprintf('  %s: expanded 10-state -> 12-state (yaw/wz = 0)\n', names{f});
    end
    if numel(data{f}.xEq) == 10
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
    data{f}.t = data{f}.t(mask);
    fprintf('%s: %d points, dt=%.6f\n', names{f}, length(data{f}.t), data{f}.dt);
end
fprintf('T_end = %.4f s\n', T_end);

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
    % Error threshold markers (vs first file)
    if isfinite(err_thresh_mm) && err_thresh_mm > 0
        ref_t = data{1}.t;
        for f = 2:nFiles
            xi = interp1(data{f}.t, data{f}.x(i,:)', ref_t, 'pchip')';
            err_mm = abs(data{1}.x(i,:) - xi) * pos_scale;
            bad = err_mm > err_thresh_mm;
            if any(bad)
                scatter(ref_t(bad), xi(bad)*pos_scale, ...
                    20, colors(f,:), 'x', 'LineWidth', 1.5, 'HandleVisibility','off');
            end
        end
    end
    yline(xEq(i)*pos_scale,'k:','HandleVisibility','off');
    xlabel('Time (s)');
    ylabel(pos_unit);
    title(state_names{i});
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
    % Error threshold markers (vs first file)
    if isfinite(err_thresh_deg) && err_thresh_deg > 0
        ref_t = data{1}.t;
        for f = 2:nFiles
            xi = interp1(data{f}.t, data{f}.x(si,:)', ref_t, 'pchip')';
            err_deg = abs(data{1}.x(si,:) - xi) * ang_scale;
            bad = err_deg > err_thresh_deg;
            if any(bad)
                scatter(ref_t(bad), xi(bad)*ang_scale, ...
                    20, colors(f,:), 'x', 'LineWidth', 1.5, 'HandleVisibility','off');
            end
        end
    end
    yline(xEq(si)*ang_scale,'k:','HandleVisibility','off');
    xlabel('Time (s)');
    ylabel(ang_unit);
    title(state_names{si});
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
    xlabel('Time (s)');
    ylabel(vel_labels{i});
    title(state_names{si});
    if i == 1
        legend('Location','best');
    end
end
sgtitle('Velocity Comparison','FontSize',14,'FontWeight','bold');

%% ==========================================================
%% FIGURE 4: Error (all vs first file)
%% ==========================================================

figure(4); clf;
set(gcf,'Name','Error','Position',[150 200 1200 600]);

% Interpolate all onto first file's time grid, compute errors
ref = data{1};
nErr = nFiles - 1;
err_colors = colors(2:end,:);

subplot(2,2,1); hold on; grid on; box on;
subplot(2,2,2); hold on; grid on; box on;
subplot(2,2,3); hold on; grid on; box on;
subplot(2,2,4); hold on; grid on; box on;

for f = 2:nFiles
    x_interp = interp1(data{f}.t, data{f}.x', ref.t, 'pchip')';
    err = ref.x - x_interp;
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

    % Print summary
    fprintf('\n=== Error: %s ===\n', label);
    fprintf('Position  max: %.4e m  (%.4e %s)\n', ...
        max(abs(err(1:3,:)),[],'all'), ...
        max(abs(err(1:3,:)),[],'all')*pos_scale, pos_unit);
    fprintf('Angle     max: %.4e rad (%.4e %s)\n', ...
        max(abs(err(4:6,:)),[],'all'), ...
        max(abs(err(4:6,:)),[],'all')*ang_scale, ang_unit);
    fprintf('Lin vel   max: %.4e m/s\n', ...
        max(abs(err(7:9,:)),[],'all'));
    fprintf('Ang vel   max: %.4e rad/s\n', ...
        max(abs(err(10:12,:)),[],'all'));
end

subplot(2,2,1); ylabel(pos_unit); title('Position error'); legend('Location','best');
subplot(2,2,2); ylabel(ang_unit); title('Orientation error'); legend('Location','best');
subplot(2,2,3); xlabel('Time (s)'); ylabel('m/s'); title('Linear velocity error'); legend('Location','best');
subplot(2,2,4); xlabel('Time (s)'); ylabel('rad/s'); title('Angular velocity error'); legend('Location','best');

sgtitle(sprintf('Error (relative to %s)', names{1}), ...
    'FontSize',14,'FontWeight','bold');
