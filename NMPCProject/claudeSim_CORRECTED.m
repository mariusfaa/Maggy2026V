%% Maglev Simulation with acados - CORRECTED VERSION

% CHANGES FROM ORIGINAL:
% 1. Fixed Euler angle kinematics in dynamics
% 2. Improved LQR weights (z-position priority)
% 3. Added control saturation
% 4. Smaller initial perturbation for testing
% 5. Added controllability check

clear; close all;

% 1. DEFINE PATHS
acados_root = '/home/mariujf/acados';
project_root = '/home/mariujf/Maggy2026V/NMPCProject';

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
fprintf('=== Maglev System Simulation with acados (CORRECTED) ===\n\n');

% Load your params struct
parameters_maggy_V4;

% Apply correction factor if function exists
if exist('computeSolenoidRadiusCorrectionFactor', 'file')
    correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
    fprintf('Fast correction factor %.2f\n', correctionFactorFast);
    paramsFast = params;
    paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;
else
    fprintf('Warning: computeSolenoidRadiusCorrectionFactor not found, using params as-is\n');
    paramsFast = params;
end

% Define equilibrium point
if exist('computeSystemEquilibria', 'file')
    [zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');
else
    fprintf('Warning: computeSystemEquilibria not found, using manual equilibrium\n');
    zEq = 0.010; % 10mm estimate - adjust based on your system
end

xEq = [0, 0, zEq, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
n_solenoids = length(paramsFast.solenoids.r);
uEq = zeros(n_solenoids, 1);

fprintf('Equilibrium height: z = %.2f mm\n', zEq*1000);

%% Step 1: Linearization using CasADi

fprintf('\nStep 1: Linearizing system at equilibrium...\n');

nx = 12; % State dimension
nu = n_solenoids; % Control dimension

% Create symbolic variables
x_sym = SX.sym('x', nx);
u_sym = SX.sym('u', nu);

% Get nonlinear dynamics symbolic expression
% IMPORTANT: Use the CORRECTED dynamics file!
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

% Check controllability
ctrb_matrix = ctrb(A_red_numeric, B_red_numeric);
rank_ctrb = rank(ctrb_matrix);
fprintf(' Controllability matrix rank: %d (should be %d)\n', rank_ctrb, size(A_red_numeric, 1));

if rank_ctrb < size(A_red_numeric, 1)
    warning('System is not fully controllable!');
else
    fprintf(' System is fully controllable ✓\n');
end

% CORRECTED LQR weights - Z position is HIGHEST priority
% State order in reduced system: [x, y, z, roll, pitch, vx, vy, vz, wx, wy]
Q = diag([1e4,  % x position
          1e4,  % y position  
          1e7,  % z position (HIGHEST - most critical!)
          1e3,  % roll
          1e3,  % pitch
          1e2,  % vx
          1e2,  % vy
          1e3,  % vz (vertical velocity important)
          1e2,  % wx
          1e2]);% wy

R = 1e-0 * eye(nu);

fprintf(' LQR weights:\n');
fprintf('   Q_z = %.0e (vertical position - HIGHEST)\n', Q(3,3));
fprintf('   Q_xy = %.0e (horizontal position)\n', Q(1,1));
fprintf('   Q_angles = %.0e (roll/pitch)\n', Q(4,4));
fprintf('   R = %.0e (control effort)\n', R(1,1));

% Compute LQR gain
K_red = lqr(A_red_numeric, B_red_numeric, Q, R);

% Map back to full state space
K = zeros(nu, nx);
K(:, controllable_indices) = K_red;

fprintf(' LQR gain K: %dx%d\n', size(K));

%% Step 3: Setup acados Integrator

fprintf('\nStep 3: Setting up acados integrator...\n');

% 1. Define time step
dt = 0.01; % [s]

% 2. Create symbolic variables
x = SX.sym('x', nx);
u = SX.sym('u', nu);
xdot = SX.sym('xdot', nx);

% 3. Create the Model Object
model = AcadosModel();
model.name = 'maglev_sim_corrected';
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
sim.solver_options.num_stages = 1;
sim.solver_options.num_steps = 1;
sim.solver_options.newton_iter = 5;

% 6. Set code export directory
code_gen_dir = fullfile(project_root, 'c_generated_code');
if ~exist(code_gen_dir, 'dir')
    mkdir(code_gen_dir);
end
sim.code_export_directory = code_gen_dir;

% 7. Create the integrator solver object
fprintf(' Creating AcadosSimSolver...\n');
fprintf(' NOTE: First compilation may take 1-3 minutes.\n');

try
    sim_solver = AcadosSimSolver(sim);
    fprintf(' Integrator configured successfully.\n');
catch ME
    fprintf(' ERROR during solver creation:\n');
    fprintf(' %s\n', ME.message);
    rethrow(ME);
end

%% Step 4: Closed-Loop Simulation

fprintf('\nStep 4: Running closed-loop simulation...\n');

% CORRECTED: Much smaller initial perturbation for testing
% Original: [0.001, -0.003, 0.04, pi/5, ...]  (36 degree roll!)
% New: Small perturbations only
x0 = xEq + [0, 0, 0.002, 0.01, 0, 0, zeros(1, 6)]';  % 2mm up, 0.6° roll

fprintf(' Initial perturbation:\n');
fprintf('   Δz = %.1f mm\n', (x0(3)-xEq(3))*1000);
fprintf('   Δroll = %.2f deg\n', rad2deg(x0(4)-xEq(4)));

% Control saturation limits [A]
u_min = 0.0;  % Minimum current
u_max = 5.0;  % Maximum current (adjust based on your hardware)

fprintf(' Control saturation: [%.1f, %.1f] A\n', u_min, u_max);

% Simulation parameters
t_final = 2.0; % [s] - increased to see settling
t_vec = 0:dt:t_final;
N_sim = length(t_vec);

% Preallocate trajectory arrays
x_traj = zeros(nx, N_sim);
u_traj = zeros(nu, N_sim);
u_traj_saturated = zeros(nu, N_sim);
x_traj(:, 1) = x0;

% Simulation loop
fprintf(' Simulating %d time steps...\n', N_sim-1);

tic;
x_current = x0;
saturated_count = 0;

for k = 1:N_sim-1
    % Compute LQR control (unsaturated)
    u_raw = -K * (x_current - xEq) + uEq;
    
    % ADDED: Control saturation
    u_current = max(u_min, min(u_max, u_raw));
    
    % Track saturation events
    if any(abs(u_raw - u_current) > 1e-6)
        saturated_count = saturated_count + 1;
    end
    
    u_traj(:, k) = u_raw;
    u_traj_saturated(:, k) = u_current;
    
    % Set initial state and control
    sim_solver.set('x', x_current);
    sim_solver.set('u', u_current);
    
    % Integrate one step
    status = sim_solver.solve();
    
    % Check status
    if status ~= 0
        warning('Solver failed at step %d with status %d.', k, status);
        fprintf(' State at failure: x = [%.4f, %.4f, %.4f, ...]\n', x_current(1:3));
        fprintf(' Control at failure: u = [%.4f, %.4f, %.4f, %.4f]\n', u_current);
        break;
    end
    
    % Retrieve next state
    x_current = sim_solver.get('xn');
    x_traj(:, k+1) = x_current;
    
    % Print progress
    if mod(k, 50) == 0
        fprintf(' Step %d/%d: z=%.2fmm, roll=%.2fdeg\n', ...
            k, N_sim-1, x_current(3)*1000, rad2deg(x_current(4)));
    end
end

% Final control
u_raw = -K * (x_traj(:, N_sim) - xEq) + uEq;
u_traj(:, N_sim) = u_raw;
u_traj_saturated(:, N_sim) = max(u_min, min(u_max, u_raw));

sim_time = toc;
fprintf('\n Simulation completed in %.3f seconds (%.1f ms per step)\n', ...
    sim_time, 1000*sim_time/(N_sim-1));
fprintf(' Control saturated in %d/%d steps (%.1f%%)\n', ...
    saturated_count, N_sim-1, 100*saturated_count/(N_sim-1));

%% Step 5: Performance Metrics

fprintf('\n=== Performance Metrics ===\n');
fprintf('Initial error norm (position): %.4f mm\n', 1000*norm(x0(1:3) - xEq(1:3)));
fprintf('Final error norm (position): %.4f mm\n', 1000*norm(x_traj(1:3,end) - xEq(1:3)));
fprintf('Initial error norm (angles): %.3f deg\n', rad2deg(norm(x0(4:6) - xEq(4:6))));
fprintf('Final error norm (angles): %.3f deg\n', rad2deg(norm(x_traj(4:6,end) - xEq(4:6))));

% Settling time (2% criterion for z position)
z_error = abs(x_traj(3,:) - xEq(3));
z_settled_idx = find(z_error < 0.02*abs(x0(3)-xEq(3)), 1, 'first');
if ~isempty(z_settled_idx)
    fprintf('Z-position settling time (2%%): %.3f s\n', t_vec(z_settled_idx));
else
    fprintf('Z-position did not settle within simulation time\n');
end

% Max control effort
fprintf('Max control current: %.3f A\n', max(max(abs(u_traj_saturated))));

%% Step 6: Visualization

fprintf('\nStep 6: Generating plots...\n');

% Create main figure for states
fig1 = figure('Name', 'State Trajectories - CORRECTED', 'Position', [50, 50, 1400, 900]);

% Position states
subplot(3,4,1); plot(t_vec, 1000*x_traj(1,:), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('X [mm]'); title('X Position'); grid on;

subplot(3,4,2); plot(t_vec, 1000*x_traj(2,:), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Y [mm]'); title('Y Position'); grid on;

subplot(3,4,3); plot(t_vec, 1000*x_traj(3,:), 'LineWidth', 2); hold on;
plot(t_vec, 1000*zEq*ones(size(t_vec)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Z [mm]'); title('Z Position (CRITICAL)');
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

sgtitle('Maglev System: CORRECTED LQR Control', 'FontSize', 14, 'FontWeight', 'bold');

% Create figure for control inputs
fig2 = figure('Name', 'Control Inputs - CORRECTED', 'Position', [100, 100, 1000, 500]);
colors = lines(nu);

for i = 1:nu
    subplot(2, ceil(nu/2), i);
    plot(t_vec, u_traj(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'LineStyle', '--');
    hold on;
    plot(t_vec, u_traj_saturated(i,:), 'Color', colors(i,:), 'LineWidth', 2);
    plot(t_vec, uEq(i)*ones(size(t_vec)), 'k--', 'LineWidth', 1);
    plot(t_vec, u_max*ones(size(t_vec)), 'r:', 'LineWidth', 1);
    plot(t_vec, u_min*ones(size(t_vec)), 'r:', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel(sprintf('I_%d [A]', i));
    title(sprintf('Solenoid %d Current', i));
    legend('Commanded', 'Saturated', 'Equilibrium', 'Limits', 'Location', 'best');
    grid on;
    ylim([u_min-0.5, u_max+0.5]);
end

sgtitle('Control Inputs (with Saturation)', 'FontSize', 14, 'FontWeight', 'bold');

% 3D trajectory plot
fig3 = figure('Name', '3D Trajectory - CORRECTED', 'Position', [150, 150, 800, 600]);
plot3(1000*x_traj(1,:), 1000*x_traj(2,:), 1000*x_traj(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(1000*x0(1), 1000*x0(2), 1000*x0(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(1000*xEq(1), 1000*xEq(2), 1000*xEq(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('3D Position Trajectory');
legend('Trajectory', 'Initial', 'Equilibrium');
grid on; axis equal;
view(45, 30);

fprintf('\nSimulation complete!\n');
fprintf('\n=== NEXT STEPS ===\n');
fprintf('1. If system is now stable, gradually increase initial perturbation\n');
fprintf('2. Try n=20 or n=50 to see effect of discretization\n');
fprintf('3. Tune Q and R weights for better performance\n');
fprintf('4. Consider adding integral action for steady-state error\n');
