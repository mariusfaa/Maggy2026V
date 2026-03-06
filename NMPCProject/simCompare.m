%% --- Compare any two simulation result files ---

% clear; clc;

%% ==========================================================
%% Settings
%% ==========================================================

file1 = "results_acados.mat";
file2 = "results_ode.mat";

T_end = 0.05;   % 0 = auto

%% ==========================================================
%% Load files
%% ==========================================================

if ~isfile(file1), error("File not found: %s", file1); end
if ~isfile(file2), error("File not found: %s", file2); end

data1 = load(file1);
data2 = load(file2);

% Use filename (without extension) for legend
[~, name1, ~] = fileparts(file1);
[~, name2, ~] = fileparts(file2);

% Replace underscores with spaces for legend display
name1 = strrep(name1, '_', ' ');
name2 = strrep(name2, '_', ' ');

xEq = data1.xEq;

if T_end == 0
    T_end = min(data1.t(end), data2.t(end));
end

% Trim both to common time window
mask1 = data1.t <= T_end + 1e-12;
mask2 = data2.t <= T_end + 1e-12;

data1.x = data1.x(:, mask1);
data1.t = data1.t(mask1);

data2.x = data2.x(:, mask2);
data2.t = data2.t(mask2);

fprintf('T_end = %.4f s\n', T_end);
fprintf('%s: %d points, dt=%.6f\n', name1, length(data1.t), data1.dt);
fprintf('%s: %d points, dt=%.6f\n', name2, length(data2.t), data2.dt);

%% ==========================================================
%% Units
%% ==========================================================

pos_scale = 1e3;  
ang_scale = 180/pi;

pos_unit  = 'mm';
ang_unit  = 'deg';

state_names = {'x','y','z','roll','pitch','yaw', ...
               'vx','vy','vz','\omega_x','\omega_y','\omega_z'};

%% ==========================================================
%% FIGURE 1: Position comparison
%% ==========================================================

figure(1); clf;
set(gcf,'Name','Position Comparison','Position',[50 400 1200 500]);

for i = 1:3
    subplot(1,3,i); hold on; grid on; box on;

    plot(data1.t, data1.x(i,:)*pos_scale, ...
        'b-','LineWidth',2.0,'DisplayName',name1);

    plot(data2.t, data2.x(i,:)*pos_scale, ...
        'r-','LineWidth',1.5,'DisplayName',name2);

    yline(xEq(i)*pos_scale,'k:','HandleVisibility','off');

    xlabel('Time (s)');
    ylabel(pos_unit);
    title(state_names{i});
    legend('Location','best');
end

sgtitle(sprintf('Position: %s vs %s', name1, name2), ...
    'FontSize',14,'FontWeight','bold');


%% ==========================================================
%% FIGURE 2: Orientation comparison
%% ==========================================================

figure(2); clf;
set(gcf,'Name','Orientation Comparison','Position',[50 50 1200 500]);

for i = 1:3
    subplot(1,3,i); hold on; grid on; box on;

    si = i + 3;

    plot(data1.t, data1.x(si,:)*ang_scale, ...
        'b-','LineWidth',2.0,'DisplayName',name1);

    plot(data2.t, data2.x(si,:)*ang_scale, ...
        'r-','LineWidth',1.5,'DisplayName',name2);

    yline(xEq(si)*ang_scale,'k:','HandleVisibility','off');

    xlabel('Time (s)');
    ylabel(ang_unit);
    title(state_names{si});
    legend('Location','best');
end

sgtitle(sprintf('Orientation: %s vs %s', name1, name2), ...
    'FontSize',14,'FontWeight','bold');


%% ==========================================================
%% FIGURE 3: Velocity comparison
%% ==========================================================

figure(3); clf;
set(gcf,'Name','Velocity Comparison','Position',[100 300 1200 500]);

vel_labels = {'m/s','m/s','m/s','rad/s','rad/s','rad/s'};

for i = 1:6
    subplot(2,3,i); hold on; grid on; box on;

    si = i + 6;

    plot(data1.t, data1.x(si,:), ...
        'b-','LineWidth',2.0,'DisplayName',name1);

    plot(data2.t, data2.x(si,:), ...
        'r-','LineWidth',1.5,'DisplayName',name2);

    xlabel('Time (s)');
    ylabel(vel_labels{i});
    title(state_names{si});

    if i == 1
        legend('Location','best');
    end
end

sgtitle(sprintf('Velocities: %s vs %s', name1, name2), ...
    'FontSize',14,'FontWeight','bold');


%% ==========================================================
%% FIGURE 4: Error (file1 - file2)
%% ==========================================================

% Interpolate file2 onto file1 time grid
x2_interp = interp1(data2.t, data2.x', data1.t, 'pchip')';
err = data1.x - x2_interp;

figure(4); clf;
set(gcf,'Name','Error','Position',[150 200 1200 600]);

subplot(2,2,1); hold on; grid on; box on;
plot(data1.t, err(1:3,:)*pos_scale,'LineWidth',1.5);
ylabel(pos_unit);
title('Position error');
legend(state_names(1:3),'Location','best');

subplot(2,2,2); hold on; grid on; box on;
plot(data1.t, err(4:6,:)*ang_scale,'LineWidth',1.5);
ylabel(ang_unit);
title('Orientation error');
legend(state_names(4:6),'Location','best');

subplot(2,2,3); hold on; grid on; box on;
plot(data1.t, err(7:9,:),'LineWidth',1.5);
xlabel('Time (s)');
ylabel('m/s');
title('Linear velocity error');
legend(state_names(7:9),'Location','best');

subplot(2,2,4); hold on; grid on; box on;
plot(data1.t, err(10:12,:),'LineWidth',1.5);
xlabel('Time (s)');
ylabel('rad/s');
title('Angular velocity error');
legend(state_names(10:12),'Location','best');

sgtitle(sprintf('Error: %s - %s', name1, name2), ...
    'FontSize',14,'FontWeight','bold');


%% ==========================================================
%% Error Summary
%% ==========================================================

fprintf('\n=== Error summary (%s - %s) ===\n', name1, name2);

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