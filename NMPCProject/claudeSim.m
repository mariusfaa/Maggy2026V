%% Maglev Simulation with acados - Correct API Usage
% This script uses the proper acados MATLAB interface structure
%
% Prerequisites:
% 1. Install acados: https://docs.acados.org/installation/
% 2. Add to MATLAB path: addpath('<acados_root>/interfaces/acados_matlab_octave')
% 3. Ensure maglevSystemDynamicsCasADi.m and ellipke_casadi.m are in path

clear; close all;

% 1. DEFINE PATHS (Double check these once more)
acados_root = 'C:\Users\mariujf\acados'; 
project_root = 'C:\Users\mariujf\maggy26\Maggy2026V\NMPCProject'; 

% 2. THE NUCLEAR FIX FOR PATHS
% We must set these BEFORE importing casadi or creating the solver.
setenv('ACADOS_SOURCE_DIR', acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
% This forces the MEX compiler to look in the acados folder for link_libs.json
setenv('ACADOS_INSTALL_DIR', acados_root); 

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external', 'jsonlab'));
addpath(fullfile(acados_root, 'external', 'casadi-matlab'));

% Check if the folder actually exists
if ~exist(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'), 'dir')
    error('Acados interface path not found. Please check acados_root.');
end

% Project folders
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% Parameters and Equilibrium Setup
fprintf('=== Maglev System Simulation with acados ===\n\n');

% IMPORTANT: Load your params struct here
parameters_maggy_V4;
correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
fprintf('Fast correction factor %.2f\n', correctionFactorFast);
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

% Define equilibrium point
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');
xEq = [0, 0, zEq, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
n_solenoids = length(paramsFast.solenoids.r); % Adjust based on your system
uEq = zeros(n_solenoids, 1);

%% Step 1: Linearization using CasADi
fprintf('Step 1: Linearizing system at equilibrium...\n');

nx = 12; % State dimension
nu = n_solenoids; % Control dimension

% Create symbolic variables
x_sym = SX.sym('x', nx);
u_sym = SX.sym('u', nu);

% Get nonlinear dynamics symbolic expression
f_nl = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast);

% 1. Compute Jacobian symbolic EXPRESSIONS
jac_A_expr = jacobian(f_nl, x_sym);
jac_B_expr = jacobian(f_nl, u_sym);

% 2. Create CasADi FUNCTIONS to evaluate these expressions
% This compiles the symbolic graph into an evaluatable function
A_fun = Function('A_fun', {x_sym, u_sym}, {jac_A_expr});
B_fun = Function('B_fun', {x_sym, u_sym}, {jac_B_expr});

% 3. Evaluate at equilibrium to get NUMERIC matrices
% full() converts the resulting CasADi DM (dense matrix) to a MATLAB double
A = full(A_fun(xEq, uEq));
B = full(B_fun(xEq, uEq));

fprintf('  A matrix: %dx%d (Numeric)\n', size(A));
fprintf('  B matrix: %dx%d (Numeric)\n', size(B));

%% Step 2: LQR Controller Design
fprintf('\nStep 2: Designing LQR controller...\n');

% Remove uncontrollable states (yaw rotation around z-axis)
% Keep states: [x, y, z, roll, pitch, vx, vy, vz, wx, wy]
% Remove: yaw (6) and wz (12)
controllable_indices = [1:5, 7:11];

% Extract submatrices
A_red = A(controllable_indices, controllable_indices);
B_red = B(controllable_indices, :);

% --- IMPORTANT FIX: Convert CasADi objects to MATLAB doubles ---
A_red_numeric = full(double(A_red));
B_red_numeric = full(double(B_red));
% ---------------------------------------------------------------

% Design LQR weights
Q = diag([1e6, 1e6, 1e2, 1e1, 1e1, 1e2, 1e2, 1e2, 1e2, 1e2]);
R = 1e-0 * eye(nu);

% Compute LQR gain using numeric matrices
K_red = lqr(A_red_numeric, B_red_numeric, Q, R);

% Map back to full state space
K = zeros(nu, nx);
K(:, controllable_indices) = K_red;

fprintf('  LQR gain K: %dx%d\n', size(K));

%% Step 3: Setup acados Integrator (v0.5.3 Direct Structure Fix)
fprintf('\nStep 3: Setting up acados integrator...\n');

% 1. Create the formal model object
% This creates the internal .model_struct needed by the solver
model = acados_sim_model();

nx = 12;
nu = n_solenoids;
x = SX.sym('x', nx);
u = SX.sym('u', nu);
xdot = SX.sym('xdot', nx);

% Populate the internal struct directly to bypass 'set' method errors
model.model_struct.name = 'maglev_sim';
model.model_struct.x = x;
model.model_struct.u = u;
model.model_struct.xdot = xdot;

% Dynamics: The v0.5.3 bridge at line 74 looks for 'dyn_expr_f'
f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
model.model_struct.dyn_type = 'implicit';
model.model_struct.dyn_expr_f = xdot - f_expl; % f(x,xdot,u) = 0

% Time step: The bridge at line 162 looks for model.T
model.model_struct.T = 0.01; 

% 2. Setup options
sim_opts = acados_sim_opts();
% Direct assignment to the internal struct to avoid "wrong field" errors
sim_opts.opts_struct.method = 'irk';
sim_opts.opts_struct.num_stages = 3;
sim_opts.opts_struct.num_steps = 3;

% 3. Initialize the simulator
% This will now pass 'make_consistent' because model.model_struct.x exists
sim = acados_sim(model, sim_opts);

fprintf('  Integrator configured successfully.\n');

%% Step 4: Closed-Loop Simulation
fprintf('\nStep 4: Running closed-loop simulation...\n');

% Initial condition (perturbed from equilibrium)
x0 = xEq + [0.001, -0.003, 0.04, pi/5, 0, 0, zeros(1, 6)]';

% Simulation parameters
dt = sim_opts.T;
t_final = 1.0; % [s]
t_vec = 0:dt:t_final;
N_sim = length(t_vec);

% Preallocate trajectory arrays
x_traj = zeros(nx, N_sim);
u_traj = zeros(nu, N_sim);
x_traj(:, 1) = x0;

% Simulation loop
fprintf('  Simulating %d time steps...\n', N_sim-1);
tic;

x_current = x0;
for k = 1:N_sim-1
    % Compute LQR control law: u = -K(x - xEq) + uEq
    u_current = -K * (x_current - xEq) + uEq;
    u_traj(:, k) = u_current;
    
    % Set state and control for simulator
    sim.set('x', x_current);
    sim.set('u', u_current);
    
    % Integrate one step
    sim.solve();
    
    % Retrieve next state
    x_current = sim.get('xn');
    x_traj(:, k+1) = x_current;
end

% Final control
u_traj(:, N_sim) = -K * (x_traj(:, N_sim) - xEq) + uEq;

sim_time = toc;
fprintf('  Simulation completed in %.3f seconds (%.1f ms per step)\n', ...
    sim_time, 1000*sim_time/(N_sim-1));

%% Step 5: Visualization
fprintf('\nStep 5: Generating plots...\n');

% Create main figure for states
fig1 = figure('Name', 'State Trajectories', 'Position', [50, 50, 1400, 900]);

% Position states
subplot(3,4,1); plot(t_vec, 1000*x_traj(1,:), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('X [mm]'); title('X Position'); grid on;

subplot(3,4,2); plot(t_vec, 1000*x_traj(2,:), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Y [mm]'); title('Y Position'); grid on;

subplot(3,4,3); plot(t_vec, 1000*x_traj(3,:), 'LineWidth', 2); hold on;
plot(t_vec, 1000*zEq*ones(size(t_vec)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Z [mm]'); title('Z Position');
legend('Response', 'Equilibrium'); grid on;

% Orientation states
subplot(3,4,4); plot(t_vec, rad2deg(x_traj(4,:)), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Roll [°]'); title('Roll Angle'); grid on;

subplot(3,4,5); plot(t_vec, rad2deg(x_traj(5,:)), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Pitch [°]'); title('Pitch Angle'); grid on;

subplot(3,4,6); plot(t_vec, rad2deg(x_traj(6,:)), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Yaw [°]'); title('Yaw Angle'); grid on;

% Linear velocities
subplot(3,4,7); plot(t_vec, 1000*x_traj(7,:), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('v_x [mm/s]'); title('X Velocity'); grid on;

subplot(3,4,8); plot(t_vec, 1000*x_traj(8,:), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('v_y [mm/s]'); title('Y Velocity'); grid on;

subplot(3,4,9); plot(t_vec, 1000*x_traj(9,:), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('v_z [mm/s]'); title('Z Velocity'); grid on;

% Angular velocities
subplot(3,4,10); plot(t_vec, rad2deg(x_traj(10,:)), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('\omega_x [°/s]'); title('Roll Rate'); grid on;

subplot(3,4,11); plot(t_vec, rad2deg(x_traj(11,:)), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('\omega_y [°/s]'); title('Pitch Rate'); grid on;

subplot(3,4,12); plot(t_vec, rad2deg(x_traj(12,:)), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('\omega_z [°/s]'); title('Yaw Rate'); grid on;

sgtitle('Maglev System: Closed-Loop Response with LQR', ...
    'FontSize', 14, 'FontWeight', 'bold');

% Create figure for control inputs
fig2 = figure('Name', 'Control Inputs', 'Position', [100, 100, 1000, 500]);
colors = lines(nu);
for i = 1:nu
    subplot(2, ceil(nu/2), i);
    plot(t_vec, u_traj(i,:), 'Color', colors(i,:), 'LineWidth', 2);
    hold on; plot(t_vec, uEq(i)*ones(size(t_vec)), 'k--', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel(sprintf('I_%d [A]', i));
    title(sprintf('Solenoid %d Current', i));
    legend('Control', 'Equilibrium'); grid on;
end
sgtitle('Control Input Signals', 'FontSize', 14, 'FontWeight', 'bold');

% 3D trajectory plot
fig3 = figure('Name', '3D Trajectory', 'Position', [150, 150, 800, 600]);
plot3(1000*x_traj(1,:), 1000*x_traj(2,:), 1000*x_traj(3,:), ...
    'b-', 'LineWidth', 2);
hold on;
plot3(1000*x0(1), 1000*x0(2), 1000*x0(3), 'go', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'g');
plot3(1000*xEq(1), 1000*xEq(2), 1000*xEq(3), 'r*', 'MarkerSize', 15, ...
    'LineWidth', 2);
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('3D Position Trajectory');
legend('Trajectory', 'Initial', 'Equilibrium');
grid on; axis equal;
view(45, 30);

fprintf('\n=== Simulation Summary ===\n');
fprintf('Initial error norm: %.4f m\n', norm(x0(1:3) - xEq(1:3)));
fprintf('Final error norm:   %.4f m\n', norm(x_traj(1:3,end) - xEq(1:3)));
fprintf('Simulation time:    %.3f s\n', sim_time);
fprintf('Average step time:  %.2f ms\n', 1000*sim_time/(N_sim-1));
fprintf('\nSimulation complete!\n');

%% Optional: Compare with MATLAB ode15s
compare_with_ode = false; % Set to true to compare

if compare_with_ode
    fprintf('\n=== Comparison with ode15s ===\n');
    
    % Define ODE function for ode15s
    ode_fun = @(t, x) full(maglevSystemDynamicsCasADi(x, -K*(x-xEq)+uEq, paramsFast));
    
    % Solve with ode15s
    fprintf('Running ode15s...\n');
    tic;
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t_ode, x_ode] = ode15s(ode_fun, [0, t_final], x0, options);
    ode_time = toc;
    
    fprintf('ode15s completed in %.3f seconds\n', ode_time);
    fprintf('Speed ratio: acados is %.2fx %s than ode15s\n', ...
        abs(ode_time/sim_time), iff(ode_time>sim_time, 'faster', 'slower'));
    
    % Plot comparison
    figure('Name', 'acados vs ode15s', 'Position', [200, 200, 1200, 400]);
    
    subplot(1,3,1);
    plot(t_vec, 1000*x_traj(1,:), 'b-', 'LineWidth', 2); hold on;
    plot(t_ode, 1000*x_ode(:,1), 'r--', 'LineWidth', 2);
    xlabel('Time [s]'); ylabel('X [mm]'); title('X Position');
    legend('acados', 'ode15s'); grid on;
    
    subplot(1,3,2);
    plot(t_vec, 1000*x_traj(2,:), 'b-', 'LineWidth', 2); hold on;
    plot(t_ode, 1000*x_ode(:,2), 'r--', 'LineWidth', 2);
    xlabel('Time [s]'); ylabel('Y [mm]'); title('Y Position');
    legend('acados', 'ode15s'); grid on;
    
    subplot(1,3,3);
    plot(t_vec, 1000*x_traj(3,:), 'b-', 'LineWidth', 2); hold on;
    plot(t_ode, 1000*x_ode(:,3), 'r--', 'LineWidth', 2);
    xlabel('Time [s]'); ylabel('Z [mm]'); title('Z Position');
    legend('acados', 'ode15s'); grid on;
    
    sgtitle('Comparison: acados vs ode15s', 'FontSize', 14);
end