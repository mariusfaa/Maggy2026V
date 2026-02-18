%% --- PROJECT SETUP ---
clear all; clc;

% 1. DEFINE PATHS (Double check these once more)
acados_root = '/home/mariujf/acados';
project_root = '/home/mariujf/Maggy2026V/NMPCProject';

% 2. THE NUCLEAR FIX FOR PATHS
% We must set these BEFORE importing casadi or creating the solver.
setenv('ACADOS_SOURCE_DIR', acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
% This forces the MEX compiler to look in the acados folder for link_libs.json
setenv('ACADOS_INSTALL_DIR', acados_root); 

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external', 'jsonlab'));
addpath(fullfile(acados_root, 'external', 'casadi-matlab'));

% Project folders
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

% --- Init OCP structure ---
model_name = 'maglev_nmpc';
ocp = AcadosOcp();
ocp.model.name = model_name;

% --- Model definitions ---
nx = 12;
nu = 4;
x = SX.sym('x', nx); 
u = SX.sym('u', nu);
xdot = SX.sym('xdot', nx);

parameters_maggy_V4;
correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');
uEq = 0.5 * ones(nu, 1);
% If it doesn't return uEq directly, you need to solve for it.
% For a symmetric 4-solenoid system at the hover point, 
% typically u1=u2=u3=u4=uEq_scalar. Try:
u_test_range = linspace(-5, 5, 1000);
for k = 1:length(u_test_range)
    u_test = u_test_range(k) * ones(4,1);
    f_test = maglevSystemDynamics([0;0;zEq;zeros(9,1)], u_test, paramsFast, 'fast');
    if abs(f_test(9)) < 0.01  % near-zero z-acceleration
        fprintf('Approximate equilibrium current: %.4f A\n', u_test_range(k));
        break;
    end
end

f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);

ocp.model.x = x;
ocp.model.u = u;
ocp.model.xdot = xdot;
ocp.model.f_impl_expr = xdot - f_expl;

% % --- THE FIX: Force External Layout ---
% ocp.code_gen_opts.code_export_directory = fullfile(project_root, 'c_generated_code');
% % This tells acados NOT to look in the current folder for the lib folder
% ocp.code_gen_opts.acados_lib_path = fullfile(acados_root, 'lib');
% ocp.code_gen_opts.acados_include_path = fullfile(acados_root, 'include');

% Solver Options
ocp.solver_options.N_horizon = 10;
ocp.solver_options.tf = 0.5;
ocp.solver_options.integrator_type = 'IRK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps = 10;
ocp.solver_options.nlp_solver_type = 'SQP'; % 'SQP_RTI' for real-time
ocp.solver_options.nlp_solver_max_iter = 100;
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.globalization = 'MERIT_BACKTRACKING';

% --- Cost (Corrected Concatenation) ---
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.cost.cost_type_0 = 'NONLINEAR_LS';

% Weights
Q = diag([1 1 10, 0.1 0.1 0.1, 0.01 0.01 0.01, 0.01 0.01 0.01]); 
R = eye(nu) * 1.0;
ocp.cost.W = blkdiag(Q, R);
ocp.cost.W_0  = blkdiag(Q, R);
ocp.cost.W_e = Q;

% Cost expressions
ocp.model.cost_y_expr = [x; u];
ocp.model.cost_y_expr_0 = [x; u];
ocp.model.cost_y_expr_e = x; 

% References
yref = [0; 0; zEq; zeros(9,1); uEq];
yref_e = yref(1:nx);

ocp.cost.yref = yref;
ocp.cost.yref_0 = yref;
ocp.cost.yref_e = yref_e;

% Constraints
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -5 * ones(nu,1); 
ocp.constraints.ubu =  5 * ones(nu,1);
ocp.constraints.x0 = [0; 0; zEq - 0.005; zeros(9,1)];

x_test = ocp.constraints.x0;
u_test = zeros(nu, 1);
% Create the CasADi function
f_func = casadi.Function('f', {x, u}, {f_expl});
% Evaluate it at the test point and convert to numeric
f_test = full(f_func(x_test, u_test));
disp('Dynamics at x0, u=0:');
disp(f_test);
if any(isnan(f_test)) || any(isinf(f_test))
    error('NaN/Inf in dynamics at initial condition! Root cause found here.');
end

% Generate and Build Solver
ocp_solver = AcadosOcpSolver(ocp);

uEq = 0.5 * ones(nu, 1);
x_init = [0; 0; zEq; zeros(9,1)];
u_init = uEq;  % or uEq if you have it
for k = 0:ocp.solver_options.N_horizon
    ocp_solver.set('x', x_init, k);
end
for k = 0:ocp.solver_options.N_horizon-1
    ocp_solver.set('u', uEq, k);
end

% Simulate
% --- Simulation Setup ---
nsim = 100;
x_current = ocp.constraints.x0;

% Initialize history for visualization
history_x = zeros(nx, nsim+1);
history_u = zeros(nu, nsim);
history_x(:,1) = x_current;

for i = 1:nsim
    % 1. Update initial state
    ocp_solver.set('constr_x0', x_current); 
    
    % 2. Solve (Call without assigning an output)
    ocp_solver.solve();
    
    % 3. Retrieve status separately
    status = ocp_solver.get('status');
    if status ~= 0
        warning('Solver failed with status %d at step %d', status, i);
    end
    
    % 4. Extract control
    u_applied = ocp_solver.get('u', 0);
    
    % 5. Plant Simulation (Using ode15s for stiffness)
    % We wrap the dynamics in a function handle for ode15s
    f_plant = @(t, x_val) maglevSystemDynamics(x_val, u_applied, paramsFast, 'fast');
    [~, x_next_traj] = ode15s(f_plant, [0, 1/20], x_current);
    x_current = x_next_traj(end, :)';
    
    % 6. Log data
    history_x(:, i+1) = x_current;
    history_u(:, i) = u_applied;
end

%% Visualization of MagLev NMPC Results
h = 1/20;
t = 0:h:(nsim*h); % Time vector for states
t_u = 0:h:((nsim-1)*h); % Time vector for controls

figure('Name', 'MagLev NMPC Performance', 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);

% --- 1. 3D Trajectory ---
subplot(2, 3, 1);
plot3(history_x(1,:), history_x(2,:), history_x(3,:), 'b', 'LineWidth', 2);
hold on;
plot3(yref(1), yref(2), yref(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Reference
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectory');
legend('Actual', 'Target');

% --- 2. Position Tracking ---
subplot(2, 3, 2);
plot(t, history_x(1:3, :), 'LineWidth', 1.5);
hold on;
plot(t, repmat(yref(1:3), 1, length(t)), '--k');
title('Position [m]');
xlabel('Time [s]');
legend('x', 'y', 'z');
grid on;

% --- 3. Orientation (Euler Angles) ---
subplot(2, 3, 3);
plot(t, rad2deg(history_x(4:6, :)), 'LineWidth', 1.5);
title('Orientation [deg]');
xlabel('Time [s]');
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)');
grid on;

% --- 4. Control Inputs (Currents) ---
subplot(2, 3, 4);
stairs(t_u, history_u', 'LineWidth', 1.5);
hold on;
yline(ocp.constraints.ubu(1), 'r--', 'Upper Limit');
yline(ocp.constraints.lbu(1), 'r--', 'Lower Limit');
title('Control Inputs (Solenoid Currents)');
xlabel('Time [s]'); ylabel('Current [A]');
legend('I_1', 'I_2', 'I_3', 'I_4');
grid on;

% --- 5. Velocity ---
subplot(2, 3, 5);
plot(t, history_x(7:9, :), 'LineWidth', 1.5);
title('Linear Velocity [m/s]');
xlabel('Time [s]');
legend('v_x', 'v_y', 'v_z');
grid on;

% --- 6. Angular Velocity ---
subplot(2, 3, 6);
plot(t, rad2deg(history_x(10:12, :)), 'LineWidth', 1.5);
title('Angular Velocity [deg/s]');
xlabel('Time [s]');
legend('\omega_x', '\omega_y', '\omega_z');
grid on;