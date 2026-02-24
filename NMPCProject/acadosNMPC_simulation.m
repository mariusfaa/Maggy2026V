%% --- PROJECT SETUP ---
clear all; clc;

acados_root = '/home/mariujf/acados';
project_root = '/home/mariujf/Maggy2026V/NMPCProject';

setenv('ACADOS_SOURCE_DIR', acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
setenv('ACADOS_INSTALL_DIR', acados_root); 

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external', 'jsonlab'));
addpath(fullfile(acados_root, 'external', 'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% --- Model setup ---
% state and input variables
nx = 12;
nu = 4;
x = SX.sym('x',nx);
u = SX.sym('u',nu);
xdot = SX.sym('xdot',nx);

% Load and correct parameters
parameters_maggy_V4;
correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

%% --- BUILD CASADI FUNCTION ---
f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
f_func = casadi.Function('f', {x, u}, {f_expl});

%% --- FIND TRUE EQUILIBRIUM USING OPTIMIZATION ---
fprintf('--- Searching for true equilibrium (z, u) ---\n');

z_var = SX.sym('z_eq');
u_var = SX.sym('u_eq', nu);

x_eq_sym = [0; 0; z_var; zeros(9,1)];
f_eq_sym = f_func(x_eq_sym, u_var);
accel = f_eq_sym(7:12);

cost = accel' * accel;

nlp = struct('x', [z_var; u_var], 'f', cost);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 3, 'max_iter', 1000)));

x0_guess = [0.030; zeros(nu,1)];
lbx = [0.015; -1*ones(nu,1)];
ubx = [0.060;  1*ones(nu,1)];

sol = solver_eq('x0', x0_guess, 'lbx', lbx, 'ubx', ubx);
opt = full(sol.x);

zEq_cas = opt(1);
uEq = opt(2:5);
xEq = [0; 0; zEq_cas; zeros(9,1)];

fprintf('Optimal equilibrium: z=%.6f m\n', zEq_cas);
fprintf('  u = [%.4f, %.4f, %.4f, %.4f] A\n', uEq(1), uEq(2), uEq(3), uEq(4));
f_check = full(f_func(xEq, uEq));
fprintf('  residual = %.4e\n', norm(f_check(7:12)));
fprintf('  ax=%.4e ay=%.4e az=%.4e\n', f_check(7), f_check(8), f_check(9));
fprintf('  ax=%.4e ay=%.4e az=%.4e\n', f_check(10), f_check(11), f_check(12));

%% --- OCP SETUP ---
ocp = AcadosOcp();
ocp.model.name = 'maglev_nmpc';
ocp.model.x = x;
ocp.model.u = u;
ocp.model.xdot = xdot;
ocp.model.f_impl_expr = xdot - f_expl;

% Solver options
N = 10;
Tf = 0.1;
dt_mpc = Tf / N;  % 0.01s

ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = Tf;
ocp.solver_options.integrator_type = 'IRK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps = 10;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';

% Cost
ocp.cost.cost_type   = 'NONLINEAR_LS';
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';

Q = diag([1e2, 1e2, 1e3, ...    % x, y, z
          1e4, 1e4, 1e1, ...    % roll, pitch, yaw
          1e1, 1e1, 1e2, ...    % vx, vy, vz
          1e4, 1e4, 1e0]);      % wx, wy, wz
R = eye(nu) * 0.1;

ocp.cost.W   = blkdiag(Q, R);
ocp.cost.W_0 = blkdiag(Q, R);
ocp.cost.W_e = Q;

ocp.model.cost_y_expr   = [x; u];
ocp.model.cost_y_expr_0 = [x; u];
ocp.model.cost_y_expr_e = x;

% References include equilibrium current
yref   = [xEq; uEq];
yref_e = xEq;

ocp.cost.yref   = yref;
ocp.cost.yref_0 = yref;
ocp.cost.yref_e = yref_e;

% Input constraints
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -1 * ones(nu,1); 
ocp.constraints.ubu =  1 * ones(nu,1);
ocp.constraints.x0 = xEq;

%% --- BUILD SOLVER ---
fprintf('\n--- Building acados solver ---\n');
ocp_solver = AcadosOcpSolver(ocp);

% Warm start at true equilibrium
for k = 0:N
    ocp_solver.set('x', xEq, k);
end
for k = 0:N-1
    ocp_solver.set('u', uEq, k);
end

%% --- HELPER: Run simulation with multiple RTI iterations ---
% SQP_RTI does only 1 QP iteration per solve() call. For states far from
% the warm-start, one iteration produces poor controls. Calling solve()
% multiple times (with the SAME x0) lets the optimizer converge before
% we apply the control. This is the standard "preparation + feedback"
% approach for real-time iteration schemes.

n_rti_iter = 5;  % Number of RTI iterations per control step

function [x_next, u_applied, status, diverged] = ...
    nmpc_step(ocp_solver, f_func, x_current, xEq, uEq, dt_mpc, n_rti, N)
    
    ocp_solver.set('constr_x0', x_current);
    
    % Multiple RTI iterations for better convergence
    for j = 1:n_rti
        ocp_solver.solve();
    end
    status = ocp_solver.get('status');
    u_applied = ocp_solver.get('u', 0);
    
    % Clamp control for safety
    u_applied = max(min(u_applied, 1), -1);
    
    % Simulate plant
    f_plant = @(t, xv) full(f_func(xv, u_applied));
    [~, x_traj] = ode15s(f_plant, [0, dt_mpc], x_current);
    x_next = x_traj(end,:)';
    
    % Divergence detection
    diverged = abs(x_next(3)) > 0.5 || ...      % z way out of range
               max(abs(x_next(4:6))) > pi || ... % angles > 180 deg
               any(isnan(x_next)) || any(isinf(x_next));
    
    % Warm-start next iteration: shift the trajectory
    for k = 0:N-2
        ocp_solver.set('x', ocp_solver.get('x', k+1), k);
        ocp_solver.set('u', ocp_solver.get('u', k+1), k);
    end
    ocp_solver.set('x', ocp_solver.get('x', N), N-1);
    ocp_solver.set('u', uEq, N-1);
    ocp_solver.set('x', xEq, N);
end

%% --- Simulate and control system

sim_steps = 200;
x_current = xEq;
plot_x = zeros(nx,sim_steps+1);
plot_u = zeros(nu,sim_steps);

% Re-warm-start
for k = 0:N, ocp_solver.set('x', xEq, k); end
for k = 0:N-1, ocp_solver.set('u', uEq, k); end

for i = 1:sim_steps
    [x_current, u_applied, status, diverged] = ...
        nmpc_step(ocp_solver, f_func, x_current, xEq, uEq, dt_mpc, n_rti_iter, N);
    
    % Get timing after solve (called inside nmpc_step)
    time_tot = ocp_solver.get('time_tot');

    fprintf('Step %3d: status=%d, solve_time=%.4f ms, |u-uEq|=%.4e\n', ...
        i, status, time_tot*1000, norm(u_applied - uEq));
    
    plot_x(:,i+1) = x_current;
    plot_u(:,i) = u_applied;
    
    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
        break;
    end
end

%% --- SAVE SIMULATION DATA ---
save_filename = 'nmpc_results.mat';

% Package the results for the separate script
sim_data = struct();
sim_data.t = (0:sim_steps) * dt_mpc;
sim_data.x = plot_x;
sim_data.u = plot_u;
sim_data.xEq = xEq;
sim_data.uEq = uEq;
sim_data.dt = dt_mpc;
sim_data.params = paramsFast; % Save physical parameters for context

save(save_filename, '-struct', 'sim_data');
fprintf('\nSimulation data saved to: %s\n', save_filename);