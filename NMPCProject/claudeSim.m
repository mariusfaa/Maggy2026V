%% Maglev Simulation with acados - FIXED Compilation Issues

% This script uses the NEW acados MATLAB interface (v0.4.0+)
%
% Prerequisites:
% 1. Install acados: https://docs.acados.org/installation/
% 2. Add to MATLAB path: addpath('<acados_root>/interfaces/acados_matlab_octave')
% 3. Ensure maglevSystemDynamicsCasADi.m and ellipke_casadi.m are in path

clear; close all;

% 1. DEFINE PATHS
acados_root = 'C:\Users\mariujf\acados';
project_root = 'C:\Users\mariujf\maggy26\Maggy2026V\NMPCProject';

% 2. SETUP ENVIRONMENT
setenv('ACADOS_SOURCE_DIR', acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
setenv('ACADOS_INSTALL_DIR', acados_root);
setenv('CFLAGS', '-O1');
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
fprintf('=== Maglev System Simulation with acados (FIXED) ===\n\n');

% Load your params struct
parameters_maggy_V4;

correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
fprintf('Fast correction factor %.2f\n', correctionFactorFast);
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

% Define equilibrium point
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');
xEq = [0, 0, zEq, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
n_solenoids = length(paramsFast.solenoids.r);
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

% Compute Jacobian symbolic EXPRESSIONS
jac_A_expr = jacobian(f_nl, x_sym);
jac_B_expr = jacobian(f_nl, u_sym);

% Create CasADi FUNCTIONS to evaluate these expressions
A_fun = Function('A_fun', {x_sym, u_sym}, {jac_A_expr});
B_fun = Function('B_fun', {x_sym, u_sym}, {jac_B_expr});

% Evaluate at equilibrium to get NUMERIC matrices
A = full(A_fun(xEq, uEq));
B = full(B_fun(xEq, uEq));

fprintf(' A matrix: %dx%d (Numeric)\n', size(A));
fprintf(' B matrix: %dx%d (Numeric)\n', size(B));

%% Step 2: LQR Controller Design

fprintf('\nStep 2: Designing LQR controller...\n');

% Remove uncontrollable states (yaw rotation around z-axis)
% Keep states: [x, y, z, roll, pitch, vx, vy, vz, wx, wy]
% Remove: yaw (6) and wz (12)

controllable_indices = [1:5, 7:11];

% Extract submatrices
A_red = A(controllable_indices, controllable_indices);
B_red = B(controllable_indices, :);

% Convert to MATLAB doubles
A_red_numeric = full(double(A_red));
B_red_numeric = full(double(B_red));

% Design LQR weights
Q = diag([1e6, 1e6, 1e2, 1e1, 1e1, 1e2, 1e2, 1e2, 1e2, 1e2]);
R = 1e-0 * eye(nu);

% Compute LQR gain
K_red = lqr(A_red_numeric, B_red_numeric, Q, R);

% Map back to full state space
K = zeros(nu, nx);
K(:, controllable_indices) = K_red;

fprintf(' LQR gain K: %dx%d\n', size(K));

%% Step 3: Setup acados Integrator (FIXED FOR COMPILATION)

fprintf('\nStep 3: Setting up acados integrator...\n');

% 1. Define time step
dt = 0.01; % [s]

% 2. Create symbolic variables
x = SX.sym('x', nx);
u = SX.sym('u', nu);
xdot = SX.sym('xdot', nx);

% 3. Create the Model Object
model = AcadosModel();
model.name = 'maglev_sim';
model.x = x;
model.xdot = xdot;
model.u = u;

% Get explicit dynamics and set as implicit (f_impl = xdot - f_expl = 0)
f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
model.f_impl_expr = xdot - f_expl;

% 4. Create Simulation Object
sim = AcadosSim();
sim.model = model;

% 5. Configure Solver Options
sim.solver_options.Tsim = dt;
sim.solver_options.integrator_type = 'IRK';
sim.solver_options.num_stages = 1;  % Reduced from 3 to speed up compilation
sim.solver_options.num_steps = 1;   % Reduced from 3
sim.solver_options.newton_iter = 5; % Increased from 3 for stability

% 6. CRITICAL: Set code export directory to reuse compiled code
code_gen_dir = fullfile(project_root, 'c_generated_code');
if ~exist(code_gen_dir, 'dir')
    mkdir(code_gen_dir);
end
sim.code_export_directory = code_gen_dir;

% 7. Create the integrator solver object
fprintf(' Creating AcadosSimSolver...\n');
fprintf(' NOTE: First compilation may take 1-3 minutes on Windows.\n');
fprintf('       Please be patient. Subsequent runs will be much faster.\n');

try
    diary('acados_log.txt');
    diary on;
    sim_solver = AcadosSimSolver(sim);
    diary off;
    fprintf(' Integrator configured successfully.\n');
catch ME
    fprintf(' ERROR during solver creation:\n');
    fprintf(' %s\n', ME.message);
    fprintf('\n TROUBLESHOOTING:\n');
    fprintf(' 1. Check that MinGW64 is properly installed and on PATH\n');
    fprintf(' 2. Try closing MATLAB and deleting: %s\n', code_gen_dir);
    fprintf(' 3. Reduce num_stages and num_steps further if needed\n');
    fprintf(' 4. Check acados installation: run acados examples first\n');
    rethrow(ME);
end

%% Step 4: Closed-Loop Simulation

fprintf('\nStep 4: Running closed-loop simulation...\n');

% Initial condition (perturbed from equilibrium)
x0 = xEq + [0.001, -0.003, 0.04, pi/5, 0, 0, zeros(1, 6)]';

% Simulation parameters
t_final = 1.0; % [s]
t_vec = 0:dt:t_final;
N_sim = length(t_vec);

% Preallocate trajectory arrays
x_traj = zeros(nx, N_sim);
u_traj = zeros(nu, N_sim);
x_traj(:, 1) = x0;

% Simulation loop
fprintf(' Simulating %d time steps...\n', N_sim-1);

tic;
x_current = x0;

for k = 1:N_sim-1
    % Compute LQR control
    u_current = -K * (x_current - xEq) + uEq;
    u_traj(:, k) = u_current;
    
    % Set initial state and control
    sim_solver.set('x', x_current);
    sim_solver.set('u', u_current);
    
    % Integrate one step
    status = sim_solver.solve();
    
    % CRITICAL: Check status immediately
    if status ~= 0
        warning('Solver failed at step %d with status %d. Reducing time step may help.', k, status);
        fprintf(' State at failure: x = [%.4f, %.4f, %.4f, ...]\n', x_current(1:3));
        fprintf(' Control at failure: u = [%.4f, %.4f, %.4f, %.4f]\n', u_current);
        break;
    end
    
    % Retrieve next state
    x_current = sim_solver.get('xn');
    x_traj(:, k+1) = x_current;
    
    % Optional: Print progress every 20 steps so you know it's alive
    if mod(k, 20) == 0
        fprintf(' Step %d/%d completed...\n', k, N_sim-1);
    end
end

% Final control
u_traj(:, N_sim) = -K * (x_traj(:, N_sim) - xEq) + uEq;

sim_time = toc;
fprintf(' Simulation completed in %.3f seconds (%.1f ms per step)\n', sim_time, 1000*sim_time/(N_sim-1));

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

sgtitle('Maglev System: Closed-Loop Response with LQR', 'FontSize', 14, 'FontWeight', 'bold');

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
plot3(1000*x_traj(1,:), 1000*x_traj(2,:), 1000*x_traj(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(1000*x0(1), 1000*x0(2), 1000*x0(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(1000*xEq(1), 1000*xEq(2), 1000*xEq(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('3D Position Trajectory');
legend('Trajectory', 'Initial', 'Equilibrium');
grid on; axis equal;
view(45, 30);

fprintf('\n=== Simulation Summary ===\n');
fprintf('Initial error norm: %.4f m\n', norm(x0(1:3) - xEq(1:3)));
fprintf('Final error norm: %.4f m\n', norm(x_traj(1:3,end) - xEq(1:3)));
fprintf('Simulation time: %.3f s\n', sim_time);
fprintf('Average step time: %.2f ms\n', 1000*sim_time/(N_sim-1));

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