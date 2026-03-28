%% --- Plot sim data ---
% clear; clc;

%% --- Settings ---
results_file = "results/res_N30_dt1000us_nmpc.mat";
T_end = 0;  % 0 for auto

%% --- Load raw results ---
if ~isfile(results_file)
    error('File not found: %s', results_file);
end

data = load(results_file);

% Expand 10-state (reduced, no yaw) to 12-state for compatibility
if size(data.x, 1) == 10
    N = size(data.x, 2);
    data.x = [data.x(1:5,:); zeros(1,N); data.x(6:10,:); zeros(1,N)];
    fprintf('Expanded 10-state -> 12-state (yaw/wz = 0)\n');
end
if numel(data.xEq) == 10
    data.xEq = [data.xEq(1:5); 0; data.xEq(6:10); 0];
end

xEq = data.xEq;

if T_end == 0
    T_end = data.t(end);
end

% Trim to T_end
mask = data.t <= T_end + 1e-12;
data.x = data.x(:, mask);
data.u = data.u(:, mask);
data.t = data.t(mask);

fprintf('T_end = %.4f s\n', T_end);
fprintf('data: %d points, dt=%.6f s\n', ...
    length(data.t), data.dt);

%% Units
pos_scale = 1e3;      % m -> mm
ang_scale = 180/pi;   % rad -> deg

%% ==========================================================
%% FIGURE 1: Position, Orientation, Input
%% ==========================================================
figure(1); clf;
set(gcf, 'Name', 'States + Input', 'Position', [100 200 1400 450]);

colors = [1 0 0; 0 0.6 0; 0 0 1; 0.5 0 0.5];  % R G B + purple

% --- Subplot 1: Position xyz ---
subplot(3,1,1); hold on; grid on; box on;
for i = 1:3
    plot(data.t, data.x(i,:)*pos_scale, ...
        'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time (s)');
ylabel('mm');
title('Position (x y z)');
legend({'x','y','z'}, 'Location','best');

% --- Subplot 2: Orientation rpy ---
subplot(3,1,2); hold on; grid on; box on;
for i = 1:3
    plot(data.t, data.x(i+3,:)*ang_scale, ...
        'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time (s)');
ylabel('deg');
title('Orientation (roll pitch yaw)');
legend({'roll','pitch','yaw'}, 'Location','best');

% --- Subplot 3: Input currents ---
subplot(3,1,3); hold on; grid on; box on;
for i = 1:4
    plot(data.t, data.u(i,:), ...
        'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time (s)');
ylabel('Current (A)');
title('Input Currents');
legend({'u1','u2','u3','u4'}, 'Location','best');

sgtitle('States and Inputs','FontSize',14,'FontWeight','bold');


%% ==========================================================
%% FIGURE 2: Velocities
%% ==========================================================
figure(2); clf;
set(gcf, 'Name', 'Velocities', 'Position', [200 100 1200 450]);

% --- Subplot 1: Linear velocity ---
subplot(2,1,1); hold on; grid on; box on;
for i = 1:3
    plot(data.t, data.x(i+6,:), ...
        'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time (s)');
ylabel('m/s');
title('Linear Velocity (v_x v_y v_z)');
legend({'v_x','v_y','v_z'}, 'Location','best');

% --- Subplot 2: Angular velocity ---
subplot(2,1,2); hold on; grid on; box on;
for i = 1:3
    plot(data.t, data.x(i+9,:), ...
        'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time (s)');
ylabel('rad/s');
title('\omega (omega_x omega_y omega_z)');
legend({'\omega_x','\omega_y','\omega_z'}, 'Location','best');

sgtitle('Velocity States','FontSize',14,'FontWeight','bold');