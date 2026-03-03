%% --- Compare acados vs ODE reference simulations ---
% Run simODEref.m and simAcados.m first, then run this script.

clear; clc;

%% --- Settings ---
T_end = 0.04;  % Set manually (e.g. 0.01) or leave [] for auto

%% --- Load raw results ---
if ~isfile('results_acados.mat')
    error('Run simAcados.m first to generate results_acados.mat');
end
if ~isfile('results_ode.mat')
    error('Run simODEref.m first to generate results_ode.mat');
end

acados = load('results_acados.mat');
ode    = load('results_ode.mat');

xEq = acados.xEq;

if T_end == 0
    T_end = min(acados.t(end), ode.t(end));
end

% Trim to T_end
acados.x = acados.x(:, acados.t <= T_end + 1e-12);
acados.t = acados.t(acados.t <= T_end + 1e-12);
ode.x    = ode.x(:, ode.t <= T_end + 1e-12);
ode.t    = ode.t(ode.t <= T_end + 1e-12);

fprintf('T_end = %.4f s\n', T_end);
fprintf('acados: %d points, dt=%.6f s, T=[%.4f, %.4f]\n', ...
    length(acados.t), acados.dt, acados.t(1), acados.t(end));
fprintf('ode15s: %d points, dt=%.6f s, T=[%.4f, %.4f]\n', ...
    length(ode.t), ode.dt, ode.t(1), ode.t(end));

% Units
pos_scale = 1e3;  % m -> mm
ang_scale = 180/pi;
pos_unit  = 'mm';
ang_unit  = 'deg';

state_names = {'x', 'y', 'z', 'roll', 'pitch', 'yaw', ...
               'vx', 'vy', 'vz', '\omega_x', '\omega_y', '\omega_z'};

%% ---- FIGURE 1: Position comparison ----
figure(1); clf;
set(gcf, 'Name', 'Position Comparison', 'Position', [50 400 1200 500]);

for i = 1:3
    subplot(1, 3, i); hold on; grid on; box on;
    plot(ode.t, ode.x(i,:)*pos_scale, 'b-',  'LineWidth', 2.0, 'DisplayName', 'ode15s');
    plot(acados.t, acados.x(i,:)*pos_scale, 'r--', 'LineWidth', 1.5, 'DisplayName', 'acados');
    yline(xEq(i)*pos_scale, ':k', 'LineWidth', 1, 'HandleVisibility', 'off');
    xlabel('Time (s)');
    ylabel(pos_unit);
    title(state_names{i});
    legend('Location', 'best');
end
sgtitle('Position: acados vs ode15s', 'FontSize', 14, 'FontWeight', 'bold');

%% ---- FIGURE 2: Orientation comparison ----
figure(2); clf;
set(gcf, 'Name', 'Orientation Comparison', 'Position', [50 50 1200 500]);

for i = 1:3
    subplot(1, 3, i); hold on; grid on; box on;
    si = i + 3;
    plot(ode.t, ode.x(si,:)*ang_scale, 'b-',  'LineWidth', 2.0, 'DisplayName', 'ode15s');
    plot(acados.t, acados.x(si,:)*ang_scale, 'r--', 'LineWidth', 1.5, 'DisplayName', 'acados');
    yline(xEq(si)*ang_scale, ':k', 'LineWidth', 1, 'HandleVisibility', 'off');
    xlabel('Time (s)');
    ylabel(ang_unit);
    title(state_names{si});
    legend('Location', 'best');
end
sgtitle('Orientation: acados vs ode15s', 'FontSize', 14, 'FontWeight', 'bold');

%% ---- FIGURE 3: Velocity comparison ----
figure(3); clf;
set(gcf, 'Name', 'Velocity Comparison', 'Position', [100 300 1200 500]);

vel_labels = {'m/s', 'm/s', 'm/s', 'rad/s', 'rad/s', 'rad/s'};
for i = 1:6
    subplot(2, 3, i); hold on; grid on; box on;
    si = i + 6;
    plot(ode.t, ode.x(si,:), 'b-',  'LineWidth', 2.0, 'DisplayName', 'ode15s');
    plot(acados.t, acados.x(si,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'acados');
    xlabel('Time (s)');
    ylabel(vel_labels{i});
    title(state_names{si});
    if i == 1, legend('Location', 'best'); end
end
sgtitle('Velocities: acados vs ode15s', 'FontSize', 14, 'FontWeight', 'bold');

%% ---- FIGURE 4: Error (acados - ode15s) ----
% Interpolate ode15s onto acados time grid only for error computation
x_o_interp = interp1(ode.t, ode.x', acados.t, 'pchip')';
err = acados.x - x_o_interp;

figure(4); clf;
set(gcf, 'Name', 'Error: acados vs ode15s', 'Position', [150 200 1200 600]);

subplot(2,2,1); hold on; grid on; box on;
plot(acados.t, err(1:3,:)*pos_scale, 'LineWidth', 1.5);
ylabel(pos_unit); title('Position error');
legend(state_names(1:3), 'Location', 'best');

subplot(2,2,2); hold on; grid on; box on;
plot(acados.t, err(4:6,:)*ang_scale, 'LineWidth', 1.5);
ylabel(ang_unit); title('Orientation error');
legend(state_names(4:6), 'Location', 'best');

subplot(2,2,3); hold on; grid on; box on;
plot(acados.t, err(7:9,:), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('m/s'); title('Linear velocity error');
legend(state_names(7:9), 'Location', 'best');

subplot(2,2,4); hold on; grid on; box on;
plot(acados.t, err(10:12,:), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('rad/s'); title('Angular velocity error');
legend(state_names(10:12), 'Location', 'best');

sgtitle('Error: acados - ode15s', 'FontSize', 14, 'FontWeight', 'bold');

% Print summary
fprintf('\n=== Error summary (acados vs ode15s) ===\n');
fprintf('  Position  max error: %.4e m  (%.4e %s)\n', max(abs(err(1:3,:)),[],'all'), max(abs(err(1:3,:)),[],'all')*pos_scale, pos_unit);
fprintf('  Angle     max error: %.4e rad (%.4e %s)\n', max(abs(err(4:6,:)),[],'all'), max(abs(err(4:6,:)),[],'all')*ang_scale, ang_unit);
fprintf('  Lin. vel  max error: %.4e m/s\n', max(abs(err(7:9,:)),[],'all'));
fprintf('  Ang. vel  max error: %.4e rad/s\n', max(abs(err(10:12,:)),[],'all'));
