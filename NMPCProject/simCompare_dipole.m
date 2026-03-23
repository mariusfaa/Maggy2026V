%% simCompare_dipole — Compare ODE reference vs acados Dipole trajectories

clear; clc;

%% Load data
ref = load('results_ode_dipole.mat');
aca = load('results_acados_dipole.mat');

fprintf('ODE ref:  %d points, T = [0, %.4f] s\n', length(ref.t), ref.t(end));
fprintf('acados:   %d points, T = [0, %.4f] s\n', length(aca.t), aca.t(end));

%% Expand 10-state to 12-state for acados data
if size(aca.x, 1) == 10
    N = size(aca.x, 2);
    aca.x = [aca.x(1:5,:); zeros(1,N); aca.x(6:10,:); zeros(1,N)];
    aca.xEq = [aca.xEq(1:5); 0; aca.xEq(6:10); 0];
    fprintf('Expanded acados 10-state -> 12-state\n');
end

%% Common time range
T_end = min(ref.t(end), aca.t(end));
fprintf('Comparing over [0, %.4f] s\n\n', T_end);

% Interpolate both onto common time grid
dt_compare = 1e-4;
t_common = 0:dt_compare:T_end;

x_ref = interp1(ref.t, ref.x', t_common, 'pchip')';
x_aca = interp1(aca.t, aca.x', t_common, 'pchip')';

err = x_ref - x_aca;

%% Error summary
pos_scale = 1e3;
ang_scale = 180/pi;

fprintf('=== Trajectory Error (ODE ref - acados Dipole) ===\n');
fprintf('  Time range: [0, %.4f] s  (%d points)\n', T_end, length(t_common));
fprintf('\n  Position (max |error|):\n');
fprintf('    x:     %.4e m  (%.4e mm)\n', max(abs(err(1,:))), max(abs(err(1,:)))*pos_scale);
fprintf('    y:     %.4e m  (%.4e mm)\n', max(abs(err(2,:))), max(abs(err(2,:)))*pos_scale);
fprintf('    z:     %.4e m  (%.4e mm)\n', max(abs(err(3,:))), max(abs(err(3,:)))*pos_scale);
fprintf('\n  Orientation (max |error|):\n');
fprintf('    roll:  %.4e rad (%.4e deg)\n', max(abs(err(4,:))), max(abs(err(4,:)))*ang_scale);
fprintf('    pitch: %.4e rad (%.4e deg)\n', max(abs(err(5,:))), max(abs(err(5,:)))*ang_scale);
fprintf('\n  Linear velocity (max |error|):\n');
fprintf('    vx:    %.4e m/s\n', max(abs(err(7,:))));
fprintf('    vy:    %.4e m/s\n', max(abs(err(8,:))));
fprintf('    vz:    %.4e m/s\n', max(abs(err(9,:))));
fprintf('\n  Angular velocity (max |error|):\n');
fprintf('    wx:    %.4e rad/s\n', max(abs(err(10,:))));
fprintf('    wy:    %.4e rad/s\n', max(abs(err(11,:))));

fprintf('\n  Overall max position error: %.4e mm\n', max(abs(err(1:3,:)),[],'all')*pos_scale);
fprintf('  Overall max angle error:    %.4e deg\n', max(abs(err(4:6,:)),[],'all')*ang_scale);

%% Plots
state_names = {'x','y','z','roll','pitch','yaw','vx','vy','vz','\omega_x','\omega_y','\omega_z'};

figure(1); clf;
set(gcf,'Name','Position Comparison','Position',[50 400 1200 400]);
for i = 1:3
    subplot(1,3,i); hold on; grid on; box on;
    plot(ref.t, ref.x(i,:)*pos_scale, 'b-', 'LineWidth', 1.5, 'DisplayName', 'ODE ref');
    plot(aca.t, aca.x(i,:)*pos_scale, 'r--', 'LineWidth', 1.5, 'DisplayName', 'acados');
    xlabel('Time (s)'); ylabel('mm');
    title(state_names{i}); legend('Location','best');
end
sgtitle('Position: ODE ref vs acados (Dipole)', 'FontSize', 14, 'FontWeight', 'bold');

figure(2); clf;
set(gcf,'Name','Orientation Comparison','Position',[50 50 800 400]);
for i = 1:2
    subplot(1,2,i); hold on; grid on; box on;
    si = i + 3;
    plot(ref.t, ref.x(si,:)*ang_scale, 'b-', 'LineWidth', 1.5, 'DisplayName', 'ODE ref');
    plot(aca.t, aca.x(si,:)*ang_scale, 'r--', 'LineWidth', 1.5, 'DisplayName', 'acados');
    xlabel('Time (s)'); ylabel('deg');
    title(state_names{si}); legend('Location','best');
end
sgtitle('Orientation: ODE ref vs acados (Dipole)', 'FontSize', 14, 'FontWeight', 'bold');

figure(3); clf;
set(gcf,'Name','Error','Position',[100 250 1200 500]);
subplot(2,2,1); hold on; grid on; box on;
plot(t_common, max(abs(err(1:3,:)),[],1)*pos_scale, 'k-', 'LineWidth', 1.5);
ylabel('mm'); title('Position error (max of x,y,z)');
subplot(2,2,2); hold on; grid on; box on;
plot(t_common, max(abs(err(4:5,:)),[],1)*ang_scale, 'k-', 'LineWidth', 1.5);
ylabel('deg'); title('Orientation error (max of roll,pitch)');
subplot(2,2,3); hold on; grid on; box on;
plot(t_common, max(abs(err(7:9,:)),[],1), 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('m/s'); title('Linear velocity error');
subplot(2,2,4); hold on; grid on; box on;
plot(t_common, max(abs(err(10:11,:)),[],1), 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('rad/s'); title('Angular velocity error');
sgtitle('Error: ODE ref - acados (Dipole)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\nFigures generated.\n');
