data = load('nmpc_results.mat');

% Extract variables for easier access
t = data.t;           % Time vector
x = data.x;           % State trajectories (nx x Nsim+1)
u = data.u;           % Control inputs (nu x Nsim)
xEq = data.xEq;       % Equilibrium/Reference states
uEq = data.uEq;       % Equilibrium/Reference controls
dt = data.dt;         % Sampling time

figure('Name', 'NMPC State Trajectories', 'Color', 'w');
nx = size(x, 1);

for i = 1:nx
    subplot(nx, 1, i);
    plot(t, x(i, :), 'b', 'LineWidth', 1.5); hold on;
    % Plot equilibrium/target line
    line([t(1) t(end)], [xEq(i) xEq(i)], 'Color', 'r', 'LineStyle', '--');
    
    ylabel(['x_{', num2str(i), '}']);
    grid on;
    if i == 1, title('State Trajectories vs Equilibrium'); end
end
xlabel('Time [s]');

figure('Name', 'NMPC Control Inputs', 'Color', 'w');
nu = size(u, 1);

for i = 1:nu
    subplot(nu, 1, i);
    % u typically has one less column than x
    stairs(t(1:end-1), u(i, :), 'Color', [0 0.5 0], 'LineWidth', 1.5); hold on;
    % Plot equilibrium control
    line([t(1) t(end)], [uEq(i) uEq(i)], 'Color', 'r', 'LineStyle', '--');
    
    ylabel(['u_{', num2str(i), '}']);
    grid on;
end
xlabel('Time [s]');
sgtitle('Control Inputs (NMPC)');