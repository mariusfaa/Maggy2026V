%% Maglev Simulation with acados - FIXED Compilation Issues
%
% Prerequisites:
% 1. Install acados: https://docs.acados.org/installation/
% 2. Add to MATLAB path: addpath('<acados_root>/interfaces/acados_matlab_octave')
% 3. Ensure maglevSystemDynamicsCasADi.m and ellipke_casadi.m are in path

clear all; close all; rmdir("build","s"); rmdir("c_generated_code","s");

%% Env setup
acados_root = 'C:\Users\halva\Downloads\acados';
project_root = 'C:\Users\halva\Downloads\Maggy2026V\NMPCProject';

% This sets acados variables, it is present in '<acados_root>/interfaces/acados_matlab_octave'
acados_env_variables_windows();

% Add project folders to path
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% Parameters and Equilibrium Setup

% Load your params struct
parameters_maggy_V4;

% Additional Parameters
dt = 0.0002; % [s]
nx = 12;
nu = 4;

% Correct params
if ~exist("correctionFactorFast",'var')
    correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
end
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

%% Linearizing using hansolini scripts
f = @(x,u) maglevSystemDynamics(x,u,paramsFast,'fast');
h = @(x,u) maglevSystemMeasurements(x,u,paramsFast,'fast');

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');

% Define the point to linearize around
xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(nu,1);

% Linearization
delta = 1e-6; % Step-size used in numerical linearization
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);

% sysc = ss(A, B, C, D); % Create state-space model
% sysd = c2d(sysc, dt, 'zoh');
% [A,B,C,D] = ssdata(sysd);

% Defining reduced order system
I = [1:5,7:11];
Ared = A(I,I);
Bred = B(I,:);
Cred = C(:,I);
Dred = D(:,:);

% Cost matrices
Q = diag([1e4,1e4,1e2, 1e1,1e1, 1e2,1e2,1e2, 1e2,1e2]);
R = 1e-0*eye(length(params.solenoids.r));

% Computing LQR estimate
Kred = lqr(Ared,Bred,Q,R); % Rounding can sometimes be dangerous!
%Kred = round(Kred,3); % Rounding can sometimes be dangerous!


% increasing order of our controller for controlling the real system
K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];



% %% Step 1: Linearization using CasADi
% fprintf('Step 1: Linearizing system at equilibrium...\n');
% 
% 
% % Get nonlinear dynamics symbolic expression
% f_nl = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast);
% 
% % Compute Jacobian symbolic EXPRESSIONS
% jac_A_expr = jacobian(f_nl, x_sym);
% jac_B_expr = jacobian(f_nl, u_sym);
% 
% % Create CasADi FUNCTIONS to evaluate these expressions
% A_fun = Function('A_fun', {x_sym, u_sym}, {jac_A_expr});
% B_fun = Function('B_fun', {x_sym, u_sym}, {jac_B_expr});
% 
% % Evaluate at equilibrium to get NUMERIC matrices
% A = full(A_fun(xLp, uLp));
% B = full(B_fun(xLp, uLp));
% C = zeros(3,12);
% D = zeros(3,4);
% sysc = ss(A, B, C, D); % Create state-space model
% sysd = c2d(sysc, dt, 'zoh');
% [Ad, Bd, Cd, Dd] = ssdata(sysd);
% A = Ad; B = Bd;
% 
% fprintf(' A matrix: %dx%d (Numeric)\n', size(A));
% fprintf(' B matrix: %dx%d (Numeric)\n', size(B));

%% Step 2: LQR Controller Design
% 
% fprintf('\nStep 2: Designing LQR controller...\n');
% 
% % Remove uncontrollable states (yaw rotation around z-axis)
% % Keep states: [x, y, z, roll, pitch, vx, vy, vz, wx, wy]
% % Remove: yaw (6) and wz (12)
% 
% controllable_indices = [1:5, 7:11];
% 
% % Extract submatrices
% A_red = A(controllable_indices, controllable_indices);
% B_red = B(controllable_indices, :);
% 
% % Convert to MATLAB doubles
% A_red_numeric = full(double(A_red));
% B_red_numeric = full(double(B_red));
% 
% % Design LQR weights
% Q = diag([1e6, 1e6, 1e2, 1e1, 1e1, 1e2, 1e2, 1e2, 1e2, 1e2]);
% R = 1e-0 * eye(nu);
% 
% % Compute LQR gain
% K_red = lqr(A_red_numeric, B_red_numeric, Q, R);
% 
% % Map back to full state space
% K = zeros(nu, nx);
% K(:, controllable_indices) = K_red;

fprintf(' LQR gain K: %dx%d\n', size(K));

%% Step 3: Setup acados Integrator (FIXED FOR COMPILATION)

fprintf('\nSetting up acados integrator...\n');

% Create symbolic variables
x_sym = SX.sym('x', nx, 1);
u_sym = SX.sym('u', nu, 1);
xdot_sym = SX.sym('xdot', nx, 1);

f_expl_expr = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast);
f_impl_expr = f_expl_expr - xdot_sym;

% 3. Populate the Model Object
model = AcadosModel();
model.x = x_sym;
model.xdot = xdot_sym;
model.u = u_sym;

model.f_expl_expr = f_expl_expr;
model.f_impl_expr = f_impl_expr;
model.name = 'magglev';

% 4. Create Simulation Object
sim = AcadosSim();
sim.model = model;

sim.solver_options.Tsim = dt;
sim.solver_options.integrator_type = 'ERK';  % 'ERK', 'IRK'
% sim.solver_options.sens_forw = true; % true, false
% sim.solver_options.jac_reuse = false; % true, false
% sim.solver_options.num_stages = 3;
% sim.solver_options.num_steps = 3;
% sim.solver_options.newton_iter = 3;
% sim.solver_options.compile_interface = 'AUTO';

% 7. Create the integrator solver object
fprintf(' Creating AcadosSimSolver...\n');
fprintf(' NOTE: This might take a while...\n');

% create integrator
sim_solver = AcadosSimSolver(sim);

%% Step 4: Closed-Loop Simulation

fprintf('\nStep 4: Running closed-loop simulation...\n');

% Initial condition (perturbed from equilibrium)
%x0 = xLp + [0.001, -0.003, 0.04, pi/5, 0, 0, zeros(1, 6)]';
%x0 = xLp + [-0.0005 0.0005 0.0004 pi/24 0 0 zeros(1, 6)].';
x0 = xLp + [0 0 0.0004 0 0 0 zeros(1, 6)].';

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
umax = 12.5;

for k = 1:N_sim-1
    % Compute LQR control
    u_current = -K * (x_current - xLp) - uLp;
    %u_current = min(umax,max(-umax,u_current));
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
    % 
    % % Optional: Print progress every 20 steps so you know it's alive
    % if mod(k, 20) == 0
    %     fprintf(' Step %d/%d completed...\n', k, N_sim-1);
    % end
end

% Final control
u_traj(:, N_sim) = -K * (x_traj(:, N_sim) - xLp) + uLp;

sim_time = toc;
fprintf(' Simulation completed in %.3f seconds (%.1f ms per step)\n', sim_time, 1000*sim_time/(N_sim-1));

%% Step 5: Visualization

fprintf('\nStep 5: Generating plots...\n');

% Create main figure for states
fig1 = figure('Name', 'State Trajectories', 'Position', [50, 50, 1400, 900]);

% Plot until instability occurs
for i=1:numel(t_vec)
    if (abs(x_traj(1:3,i)) > 0.1)
        i = i - 1;
        x_traj = x_traj(:,1:i);
        u_traj = u_traj(:,1:i);
        t_vec = t_vec(:,1:i);
        break;
    end
end


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
    hold on; plot(t_vec, uLp(i)*ones(size(t_vec)), 'k--', 'LineWidth', 1);
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
plot3(1000*xLp(1), 1000*xLp(2), 1000*xLp(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('3D Position Trajectory');
legend('Trajectory', 'Initial', 'Equilibrium');
grid on; axis equal;
view(45, 30);

fprintf('\n=== Simulation Summary ===\n');
fprintf('Initial error norm: %.4f m\n', norm(x0(1:3) - xLp(1:3)));
fprintf('Final error norm: %.4f m\n', norm(x_traj(1:3,end) - xLp(1:3)));
fprintf('Simulation time: %.3f s\n', sim_time);
fprintf('Average step time: %.2f ms\n', 1000*sim_time/(N_sim-1));

fprintf('\nSimulation complete!\n');

%% Optional: Compare with MATLAB ode15s

compare_with_ode = false; % Set to true to compare

if compare_with_ode
    fprintf('\n=== Comparison with ode15s ===\n');
    
    % Define ODE function for ode15s
    ode_fun = @(t, x) full(maglevSystemDynamicsCasADi(x, -K*(x-xLp)+uLp, paramsFast));
    
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

%clear sim_solver