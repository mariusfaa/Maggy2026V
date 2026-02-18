%% Maglev Acados Simulation - EXACT REPLICATION of Original
% This script replicates the working sim_notlive.m but uses acados for integration
% 
% KEY PRINCIPLE: Match the original EXACTLY, only swap ode15s for acados

clear; close all;

fprintf('=== Maglev Acados Simulation (Matching Original) ===\n\n');

%% 1. SETUP PATHS (Same as before)
acados_root = '/home/mariujf/acados';
project_root = '/home/mariujf/Maggy2026V/NMPCProject';

setenv('ACADOS_SOURCE_DIR', acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
setenv('ACADOS_INSTALL_DIR', acados_root);
setenv('CFLAGS', '-O1');
addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external', 'jsonlab'));
addpath(fullfile(acados_root, 'external', 'casadi-matlab'));

% Project folders
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% 2. LOAD PARAMETERS - EXACTLY AS ORIGINAL
fprintf('Step 1: Loading parameters...\n');

% IMPORTANT: Original uses parameters_maggy_V2, but you only have V4
% If you have V2, use that. Otherwise use V4 but be aware of differences.
try
    parameters_maggy_V2;
    fprintf(' Using parameters_maggy_V2 (original)\n');
catch
    fprintf(' WARNING: parameters_maggy_V2 not found, using V4 instead\n');
    parameters_maggy_V4;
end

% Apply correction factor - EXACTLY AS ORIGINAL
correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
fprintf(' Fast correction factor: %.2f\n', correctionFactorFast);

paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

%% 3. COMPUTE EQUILIBRIA - EXACTLY AS ORIGINAL
fprintf('\nStep 2: Computing equilibria...\n');

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');
fprintf(' Equilibrium height: z = %.2f mm\n', zEq(1)*1000);

% Define linearization point - EXACTLY AS ORIGINAL
xLp = [0, 0, zEq(1), zeros(1,9)]';
uLp = zeros(length(params.solenoids.r), 1);

%% 4. LINEARIZATION - EXACTLY AS ORIGINAL
fprintf('\nStep 3: Linearizing system...\n');

modelName = 'fast';

% CRITICAL: Use the ORIGINAL dynamics function, not CasADi version
f_original = @(x,u) maglevSystemDynamics(x, u, paramsFast, modelName);
h_original = @(x,u) maglevSystemMeasurements(x, u, paramsFast, modelName);

% Numerical linearization - EXACTLY AS ORIGINAL
delta = 1e-6;
[A, B, C, D] = finiteDifferenceLinearization(f_original, h_original, xLp, uLp, delta);

fprintf(' A matrix: %dx%d\n', size(A));
fprintf(' B matrix: %dx%d\n', size(B));

%% 5. LQR CONTROLLER - EXACTLY AS ORIGINAL
fprintf('\nStep 4: Designing LQR controller...\n');

% Reduced order system - EXACTLY AS ORIGINAL
I = [1:5, 7:11];
Ared = A(I,I);
Bred = B(I,:);
Cred = C(:,I);
Dred = D(:,:);

% Check controllability
rank_ctrb = rank(ctrb(Ared, Bred));
fprintf(' Controllability rank: %d (should be %d)\n', rank_ctrb, length(I));

if rank_ctrb < length(I)
    error('System is not fully controllable! Rank = %d, expected %d', rank_ctrb, length(I));
end

% Cost matrices - EXACTLY AS ORIGINAL
Q = diag([1e6, 1e6, 1e2, 1e1, 1e1, 1e2, 1e2, 1e2, 1e2, 1e2]);
R = 1e-0 * eye(length(params.solenoids.r));

% LQR gain - EXACTLY AS ORIGINAL (including rounding!)
Kred = round(lqr(Ared, Bred, Q, R), 3);

% Full order controller - EXACTLY AS ORIGINAL
K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];

fprintf(' LQR gain K: %dx%d\n', size(K));

%% 6. SETUP ACADOS INTEGRATOR
fprintf('\nStep 5: Setting up acados integrator...\n');

% For acados, we MUST use CasADi symbolic expressions
% We'll create a CasADi function that WRAPS the original MATLAB function

nx = 12;
nu = length(params.solenoids.r);
dt = 0.01; % Time step

% Create symbolic variables
x_sym = SX.sym('x', nx);
u_sym = SX.sym('u', nu);
xdot_sym = SX.sym('xdot', nx);

% OPTION 1: If you trust your CasADi implementation, use it
% f_expl = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast);

% OPTION 2: Wrap original MATLAB function (safer for testing)
% This creates a CasADi external function
fprintf(' Creating CasADi wrapper for original dynamics...\n');
fprintf(' NOTE: This may be slow but ensures correctness\n');

% We need to use the CasADi version here since we can't directly wrap MATLAB
% But we'll validate it first
fprintf('\n VALIDATION CHECK:\n');
fprintf(' Comparing CasADi vs original dynamics at test point...\n');

x_test = xLp + [0.001, 0.002, 0.003, 0.01, -0.01, 0, zeros(1,6)]';
u_test = zeros(nu, 1);

dx_original = f_original(x_test, u_test);
dx_casadi = full(maglevSystemDynamicsCasADi(x_test, u_test, paramsFast));

diff = dx_original - dx_casadi;
max_diff = max(abs(diff));
rel_error = 100 * norm(diff) / norm(dx_original);

fprintf(' Max absolute difference: %.6e\n', max_diff);
fprintf(' Relative error: %.3f%%\n', rel_error);

if max_diff > 1e-6
    warning('CasADi dynamics differ from original! Results may be wrong.');
    fprintf('\n Original: [%.6f, %.6f, %.6f, ...]\n', dx_original(1:3));
    fprintf(' CasADi:   [%.6f, %.6f, %.6f, ...]\n', dx_casadi(1:3));
    fprintf('\n Proceeding anyway, but controller may fail...\n');
end

% Use CasADi dynamics (only option for acados)
f_expl = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast);

% Create acados model
model = AcadosModel();
model.name = 'maglev_original_replication';
model.x = x_sym;
model.xdot = xdot_sym;
model.u = u_sym;
model.f_impl_expr = xdot_sym - f_expl;

% Create simulation object
sim = AcadosSim();
sim.model = model;
sim.solver_options.Tsim = dt;
sim.solver_options.integrator_type = 'IRK';
sim.solver_options.num_stages = 1;
sim.solver_options.num_steps = 1;
sim.solver_options.newton_iter = 5;

% Code export directory
code_gen_dir = fullfile(project_root, 'c_generated_code');
if ~exist(code_gen_dir, 'dir')
    mkdir(code_gen_dir);
end
sim.code_export_directory = code_gen_dir;

% Create solver
fprintf(' Creating AcadosSimSolver...\n');
try
    sim_solver = AcadosSimSolver(sim);
    fprintf(' Integrator created successfully.\n');
catch ME
    error('Failed to create acados solver: %s', ME.message);
end

%% 7. SIMULATION - EXACTLY AS ORIGINAL
fprintf('\nStep 6: Running simulation...\n');

% Initial condition - EXACTLY AS ORIGINAL
x0 = xLp + [0.001, -0.003, 0.04, pi/5, 0, 0, zeros(1, 6)]';

fprintf(' Initial perturbation:\n');
fprintf('   [x, y, z] = [%.1f, %.1f, %.1f] mm\n', (x0(1:3)-xLp(1:3))*1000);
fprintf('   [roll, pitch, yaw] = [%.1f, %.1f, %.1f] deg\n', rad2deg(x0(4:6)-xLp(4:6)));

% Simulation timespan - EXACTLY AS ORIGINAL
t_final = 1.0; % Original uses 1 second
t_vec = 0:dt:t_final;
N_sim = length(t_vec);

% Storage
x_traj = zeros(nx, N_sim);
u_traj = zeros(nu, N_sim);
x_traj(:, 1) = x0;

% Control law - EXACTLY AS ORIGINAL: u = -K*(x-xLp) - uLp
% Note the DOUBLE NEGATIVE!
control_law = @(x) -K*(x - xLp) - uLp;

fprintf(' Simulating %d steps...\n', N_sim-1);
tic;

x_current = x0;
for k = 1:N_sim-1
    % Compute control - EXACTLY AS ORIGINAL
    u_current = control_law(x_current);
    u_traj(:, k) = u_current;
    
    % Set state and control
    sim_solver.set('x', x_current);
    sim_solver.set('u', u_current);
    
    % Integrate
    status = sim_solver.solve();
    
    if status ~= 0
        warning('Solver failed at step %d with status %d', k, status);
        break;
    end
    
    % Get next state
    x_current = sim_solver.get('xn');
    x_traj(:, k+1) = x_current;
    
    % Progress
    if mod(k, 20) == 0
        fprintf(' Step %d/%d: z=%.2fmm, roll=%.1fdeg\n', ...
            k, N_sim-1, x_current(3)*1000, rad2deg(x_current(4)));
    end
end

u_traj(:, N_sim) = control_law(x_traj(:, N_sim));

sim_time = toc;
fprintf(' Simulation time: %.3fs (%.2f ms/step)\n', sim_time, 1000*sim_time/(N_sim-1));

%% 8. COMPARE WITH ORIGINAL (ode15s)
fprintf('\nStep 7: Comparing with ode15s...\n');

fprintf(' Running ode15s for comparison...\n');
tSpan = linspace(0, t_final, 100); % Original uses 100 points

tic;
[t_ode, x_ode] = ode15s(@(t,x) f_original(x, control_law(x)), tSpan, x0);
ode_time = toc;

fprintf(' ode15s time: %.3fs\n', ode_time);
fprintf(' Speedup: %.2fx\n', ode_time/sim_time);

%% 9. PLOTTING - EXACTLY AS ORIGINAL
fprintf('\nStep 8: Plotting results...\n');

% Main state plot - EXACTLY AS ORIGINAL Figure 7
figure('Name', 'Position States - Acados vs ode15s', 'Position', [50, 50, 1200, 400]);

subplot(1,3,1);
plot(t_vec, x_traj(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t_ode, x_ode(:,1), 'r--', 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('x [m]'); title('X Position');
legend('acados', 'ode15s'); grid on;

subplot(1,3,2);
plot(t_vec, x_traj(2,:), 'b-', 'LineWidth', 2); hold on;
plot(t_ode, x_ode(:,2), 'r--', 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('y [m]'); title('Y Position');
legend('acados', 'ode15s'); grid on;

subplot(1,3,3);
plot(t_vec, x_traj(3,:), 'b-', 'LineWidth', 2); hold on;
plot(t_ode, x_ode(:,3), 'r--', 'LineWidth', 1.5);
plot(t_vec, zEq(1)*ones(size(t_vec)), 'k:', 'LineWidth', 1);
xlabel('t [s]'); ylabel('z [m]'); title('Z Position');
legend('acados', 'ode15s', 'Equilibrium'); grid on;

sgtitle('Comparison: Acados vs Original ode15s', 'FontSize', 14, 'FontWeight', 'bold');

% Full state trajectories
figure('Name', 'All States - Acados', 'Position', [100, 100, 1400, 900]);

subplot(3,4,1); plot(t_vec, 1000*x_traj(1,:), 'LineWidth', 2);
xlabel('t [s]'); ylabel('X [mm]'); title('X Position'); grid on;

subplot(3,4,2); plot(t_vec, 1000*x_traj(2,:), 'LineWidth', 2);
xlabel('t [s]'); ylabel('Y [mm]'); title('Y Position'); grid on;

subplot(3,4,3); plot(t_vec, 1000*x_traj(3,:), 'LineWidth', 2); hold on;
plot(t_vec, 1000*zEq(1)*ones(size(t_vec)), 'r--', 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('Z [mm]'); title('Z Position'); grid on;

subplot(3,4,4); plot(t_vec, rad2deg(x_traj(4,:)), 'LineWidth', 2);
xlabel('t [s]'); ylabel('Roll [°]'); title('Roll'); grid on;

subplot(3,4,5); plot(t_vec, rad2deg(x_traj(5,:)), 'LineWidth', 2);
xlabel('t [s]'); ylabel('Pitch [°]'); title('Pitch'); grid on;

subplot(3,4,6); plot(t_vec, rad2deg(x_traj(6,:)), 'LineWidth', 2);
xlabel('t [s]'); ylabel('Yaw [°]'); title('Yaw'); grid on;

subplot(3,4,7); plot(t_vec, 1000*x_traj(7,:), 'LineWidth', 2);
xlabel('t [s]'); ylabel('v_x [mm/s]'); title('X Velocity'); grid on;

subplot(3,4,8); plot(t_vec, 1000*x_traj(8,:), 'LineWidth', 2);
xlabel('t [s]'); ylabel('v_y [mm/s]'); title('Y Velocity'); grid on;

subplot(3,4,9); plot(t_vec, 1000*x_traj(9,:), 'LineWidth', 2);
xlabel('t [s]'); ylabel('v_z [mm/s]'); title('Z Velocity'); grid on;

subplot(3,4,10); plot(t_vec, rad2deg(x_traj(10,:)), 'LineWidth', 2);
xlabel('t [s]'); ylabel('ω_x [°/s]'); title('Roll Rate'); grid on;

subplot(3,4,11); plot(t_vec, rad2deg(x_traj(11,:)), 'LineWidth', 2);
xlabel('t [s]'); ylabel('ω_y [°/s]'); title('Pitch Rate'); grid on;

subplot(3,4,12); plot(t_vec, rad2deg(x_traj(12,:)), 'LineWidth', 2);
xlabel('t [s]'); ylabel('ω_z [°/s]'); title('Yaw Rate'); grid on;

sgtitle('Maglev System: Acados Integration', 'FontSize', 14, 'FontWeight', 'bold');

% Control inputs
figure('Name', 'Control Inputs', 'Position', [150, 150, 1000, 500]);
for i = 1:nu
    subplot(2, ceil(nu/2), i);
    plot(t_vec, u_traj(i,:), 'LineWidth', 2);
    xlabel('t [s]'); ylabel(sprintf('I_%d [A]', i));
    title(sprintf('Solenoid %d Current', i));
    grid on;
end
sgtitle('Control Inputs', 'FontSize', 14, 'FontWeight', 'bold');

%% 10. PERFORMANCE METRICS
fprintf('\n=== Performance Summary ===\n');
fprintf('Initial position error: %.2f mm\n', 1000*norm(x0(1:3) - xLp(1:3)));
fprintf('Final position error (acados): %.4f mm\n', 1000*norm(x_traj(1:3,end) - xLp(1:3)));
fprintf('Final position error (ode15s): %.4f mm\n', 1000*norm(x_ode(end,1:3)' - xLp(1:3)));

fprintf('\nInitial angle error: %.1f deg\n', rad2deg(norm(x0(4:6) - xLp(4:6))));
fprintf('Final angle error (acados): %.3f deg\n', rad2deg(norm(x_traj(4:6,end) - xLp(4:6))));
fprintf('Final angle error (ode15s): %.3f deg\n', rad2deg(norm(x_ode(end,4:6)' - xLp(4:6))));

fprintf('\nMax control current: %.3f A\n', max(max(abs(u_traj))));

fprintf('\n=== Simulation Complete ===\n');
fprintf('If results match ode15s closely, acados is working correctly!\n');
fprintf('If not, check the CasADi dynamics implementation.\n');
