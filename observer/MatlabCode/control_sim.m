%% control
% Cost matrices
Qlqr = diag([1e6,1e6,1e4, 1e1,1e1, 1e2,1e2,1e2, 1e2,1e2]);
Qlqr_xred = diag([1e6,1e6,1e4, 1e2,1e2,1e2]);
Rlqr = 1e-0*eye(length(params.solenoids.r));

sysd_lqr = c2d(ss(Ared,Bred,Cred,Dred), dt, 'zoh');
Ad_lqr = sysd_lqr.A;
Bd_lqr = sysd_lqr.B;

% Computing LQR estimate
Kcred = lqr(Ared,Bred,Qlqr,Rlqr);
Kred = dlqr(Ad_lqr,Bd_lqr,Qlqr,Rlqr);

% increasing order of our controller for controlling the real system
%Kc = [Kcred(:,1:5), zeros(4,1), Kcred(:,6:end), zeros(4,1)];
%K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];

%writematrix(Kred, 'feedbackGain.txt');

%% Observer
clear obs;
obs = Observer(2, dt, xLp(1:6));

%% Simulation
% Initial conditions
x0 = xLp(1:10) + [0.0001 -0.0001 0.001 0 0 0 zeros(1, 4)].';
tSpan = 0:dt:1;
N = length(tSpan);

% Preallocate arrays
x = zeros(length(x0), N);
x_est = zeros(length(x0), N);
NIS_values = zeros(1, N);
u = zeros(4, N);
y = zeros(3, N);
x(:,1) = x0;

% Simulation loop - start from k=2 to skip time=0 and simulate from previous to current
for k = 2:N
    % Current time
    t_current = tSpan(k);
    t_previous = tSpan(k-1);
    
    % Simulate continuous plant from previous timestep to current timestep
    [~, x_cont] = ode15s(@(t,xf) maglevSystemDynamics_red(xf, u(:,k-1)), [t_previous t_current], x(:,k-1));
    x(:,k) = x_cont(end,:).';
    
    % Get measurement at current time
    y_ = maglevSystemMeasurements_red(x(:,k), u(:,k-1));  % Using previous control input
    y(:,k) = y_(1:3);
    
    % Estimation
    x_est(:,k) = obs.run(u(:,k-1), y(:,k));
    
    % Compute control input using true state
    u(:,k) = -Kred * (x(:,k) - xLp(1:10)) - uLp;
end

t = tSpan;

%% Plotting
figure(1);
clf;

% Subplot 1: XYZ positions (true and estimated)
subplot(2,2,1);
hold on; grid on; box on;
plot(t, x(1:3,:), 'linewidth', 2, 'LineStyle', '-');
plot(t, x_est(1:3,:), 'linewidth', 2, 'LineStyle', '--');
xlabel('t (s)');
ylabel('Position (m)');
title('XYZ States');
legend({'x true', 'y true', 'z true', 'x est', 'y est', 'z est'}, 'location', 'best');

% Subplot 2: Alpha and Beta angles (true and estimated)
subplot(2,2,2);
hold on; grid on; box on;
plot(t, x(4:5,:), 'linewidth', 2, 'LineStyle', '-');
plot(t, x_est(4:5,:), 'linewidth', 2, 'LineStyle', '--');
xlabel('t (s)');
ylabel('Angle (rad)');
title('Alpha and Beta States');
legend({'α true', 'β true', 'α est', 'β est'}, 'location', 'best');

% Subplot 3: Derivatives of XYZ (velocities)
subplot(2,2,3);
hold on; grid on; box on;
x_dot_idx = 6;
y_dot_idx = 7;
z_dot_idx = 8;
plot(t, x(x_dot_idx:x_dot_idx+2, :), 'linewidth', 2, 'LineStyle', '-');
plot(t, x_est(x_dot_idx:x_dot_idx+2, :), 'linewidth', 2, 'LineStyle', '--');
xlabel('t (s)');
ylabel('Velocity (m/s)');
title('XYZ Derivatives (Velocities)');
legend({'ẋ true', 'ẏ true', 'ż true', 'ẋ est', 'ẏ est', 'ż est'}, 'location', 'best');

% Subplot 4: Derivatives of Alpha and Beta (angular velocities)
subplot(2,2,4);
hold on; grid on; box on;
alpha_dot_idx = 9;
beta_dot_idx = 10;
plot(t, x(alpha_dot_idx:beta_dot_idx, :), 'linewidth', 2, 'LineStyle', '-');
plot(t, x_est(alpha_dot_idx:beta_dot_idx, :), 'linewidth', 2, 'LineStyle', '--');
xlabel('t (s)');
ylabel('Angular Velocity (rad/s)');
title('Alpha and Beta Derivatives');
legend({'α̇ true', 'β̇ true', 'α̇ est', 'β̇ est'}, 'location', 'best');