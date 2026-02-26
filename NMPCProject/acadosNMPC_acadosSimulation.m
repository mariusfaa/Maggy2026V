%% --- PROJECT SETUP ---
% Do NOT use 'clear all' or 'clear' — it unloads the acados MEX libraries
% from memory, causing slow re-initialisation on every re-run.
clearvars -except ocp_solver sim_solver; clc;

acados_root  = '/home/mariujf/acados';
project_root = '/home/mariujf/Maggy2026V/NMPCProject';

setenv('ACADOS_SOURCE_DIR',        acados_root);
setenv('ENV_ACADOS_INSTALL_DIR',   acados_root);
setenv('ACADOS_INSTALL_DIR',       acados_root);

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% --- Model setup ---
nx   = 12;
nu   = 4;
x    = SX.sym('x',    nx);
u    = SX.sym('u',    nu);
xdot = SX.sym('xdot', nx);

parameters_maggy_V4;
correctionFactorFast       = computeSolenoidRadiusCorrectionFactor(params, 'fast');
paramsFast                 = params;
paramsFast.solenoids.r     = correctionFactorFast * paramsFast.solenoids.r;

%% --- BUILD CASADI FUNCTION ---
f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
f_func = casadi.Function('f', {x, u}, {f_expl});

%% --- FIND EQUILIBRIUM ---
fprintf('--- Searching for true equilibrium (z, u) ---\n');

z_var    = SX.sym('z_eq');
u_var    = SX.sym('u_eq', nu);
x_eq_sym = [0; 0; z_var; zeros(9,1)];
accel    = f_func(x_eq_sym, u_var);
accel    = accel(7:12);
cost     = accel' * accel;

nlp       = struct('x', [z_var; u_var], 'f', cost);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 3, 'max_iter', 1000)));

sol = solver_eq('x0', [0.030; zeros(nu,1)], ...
                'lbx', [0.015; -1*ones(nu,1)], ...
                'ubx', [0.060;  1*ones(nu,1)]);
opt     = full(sol.x);
zEq_cas = opt(1);
uEq     = opt(2:5);
xEq     = [0; 0; zEq_cas; zeros(9,1)];

fprintf('Optimal equilibrium: z=%.6f m\n', zEq_cas);
fprintf('  u = [%.4f, %.4f, %.4f, %.4f] A\n', uEq(1), uEq(2), uEq(3), uEq(4));
f_check = full(f_func(xEq, uEq));
fprintf('  residual = %.4e\n', norm(f_check(7:12)));

%% --- OCP SETUP ---
N      = 10;
Tf     = 0.1;
dt_mpc = Tf / N;   % 0.01 s

ocp = AcadosOcp();
ocp.model.name        = 'maglev_nmpc';
ocp.model.x           = x;
ocp.model.u           = u;
ocp.model.xdot        = xdot;
ocp.model.f_impl_expr = xdot - f_expl;

ocp.solver_options.N_horizon             = N;
ocp.solver_options.tf                    = Tf;
ocp.solver_options.integrator_type       = 'IRK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 10;
ocp.solver_options.nlp_solver_type       = 'SQP_RTI';
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';

Q = diag([1e2, 1e2, 1e3, 1e4, 1e4, 1e1, 1e1, 1e1, 1e2, 1e4, 1e4, 1e0]);
R = eye(nu) * 0.1;

ocp.cost.cost_type   = 'NONLINEAR_LS';
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.cost.W           = blkdiag(Q, R);
ocp.cost.W_0         = blkdiag(Q, R);
ocp.cost.W_e         = Q;

ocp.model.cost_y_expr   = [x; u];
ocp.model.cost_y_expr_0 = [x; u];
ocp.model.cost_y_expr_e = x;

ocp.cost.yref   = [xEq; uEq];
ocp.cost.yref_0 = [xEq; uEq];
ocp.cost.yref_e = xEq;

ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu   = -1 * ones(nu,1);
ocp.constraints.ubu   =  1 * ones(nu,1);
ocp.constraints.x0    = xEq;

%% --- BUILD / REUSE OCP SOLVER ---
if ~exist('ocp_solver', 'var') || ~isvalid(ocp_solver)
    fprintf('\n--- Building acados OCP solver ---\n');
    if exist(fullfile('c_generated_code', 'maglev_nmpc_solver'), 'dir')
        ocp.solver_options.compile_interface = false;
    end
    ocp_solver = AcadosOcpSolver(ocp);
else
    fprintf('\n--- Reusing OCP solver from previous run ---\n');
end

% Warm start
for k = 0:N,   ocp_solver.set('x', xEq, k); end
for k = 0:N-1, ocp_solver.set('u', uEq, k); end

%% --- BUILD / REUSE SIM SOLVER ---
if ~exist('sim_solver', 'var') || ~isvalid(sim_solver)
    fprintf('\n--- Building acados sim solver ---\n');
    sim = AcadosSim();
    sim.model.name        = 'maglev_sim';
    sim.model.x           = x;
    sim.model.u           = u;
    sim.model.xdot        = xdot;
    sim.model.f_impl_expr = xdot - f_expl;

    sim.solver_options.Tsim            = dt_mpc;
    sim.solver_options.integrator_type = 'IRK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 10;

    if exist(fullfile('c_generated_code', 'maglev_sim'), 'dir')
        sim.solver_options.compile_interface = false;
    end
    sim_solver = AcadosSimSolver(sim);
else
    fprintf('\n--- Reusing sim solver from previous run ---\n');
end

%% --- HELPER: one NMPC step ---
n_rti_iter = 5;

function [x_next, u_applied, status, diverged] = ...
    nmpc_step(ocp_solver, sim_solver, x_current, xEq, uEq, n_rti, N)

    ocp_solver.set('constr_x0', x_current);

    for j = 1:n_rti
        ocp_solver.solve();
    end
    status    = ocp_solver.get('status');
    u_applied = ocp_solver.get('u', 0);
    u_applied = max(min(u_applied, 1), -1);

    % --- Plant simulation ---
    sim_solver.set('x', x_current);
    sim_solver.set('u', u_applied);
    sim_solver.solve();
    x_next = sim_solver.get('xn');   % 'xn' is the correct field per acados docs

    % --- Divergence check ---
    diverged = abs(x_next(3)) > 0.5       || ...
               max(abs(x_next(4:6))) > pi  || ...
               any(isnan(x_next))          || ...
               any(isinf(x_next));

    % --- Warm-start shift ---
    % Bulk get: 2 MEX calls total instead of 2*N interleaved get+set calls
    x_traj = ocp_solver.get('x');              % nx x (N+1)
    u_traj = ocp_solver.get('u');              % nu x N

    % Shift in plain MATLAB (no MEX overhead)
    x_traj_shift = [x_traj(:, 2:end), xEq];   % drop first, append xEq
    u_traj_shift = [u_traj(:, 2:end), uEq];   % drop first, append uEq

    % Write back
    for k = 0:N
        ocp_solver.set('x', x_traj_shift(:, k+1), k);
    end
    for k = 0:N-1
        ocp_solver.set('u', u_traj_shift(:, k+1), k);
    end
end

%% --- Simulation loop ---
sim_steps = 200;
x_current = xEq;
plot_x    = zeros(nx, sim_steps+1);
plot_u    = zeros(nu, sim_steps);
plot_x(:,1) = xEq;

% Re-warm-start before loop
for k = 0:N,   ocp_solver.set('x', xEq, k); end
for k = 0:N-1, ocp_solver.set('u', uEq, k); end

for i = 1:sim_steps
    [x_current, u_applied, status, diverged] = ...
        nmpc_step(ocp_solver, sim_solver, x_current, xEq, uEq, n_rti_iter, N);

    time_tot = ocp_solver.get('time_tot');
    fprintf('Step %3d: status=%d, solve_time=%.4f ms, |u-uEq|=%.4e\n', ...
        i, status, time_tot*1000, norm(u_applied - uEq));

    plot_x(:,i+1) = x_current;
    plot_u(:,i)   = u_applied;

    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
        break;
    end
end

%% --- SAVE ---
save_filename   = 'nmpc_results.mat';
sim_data        = struct();
sim_data.t      = (0:sim_steps) * dt_mpc;
sim_data.x      = plot_x;
sim_data.u      = plot_u;
sim_data.xEq    = xEq;
sim_data.uEq    = uEq;
sim_data.dt     = dt_mpc;
sim_data.params = paramsFast;

save(save_filename, '-struct', 'sim_data');
fprintf('\nSimulation data saved to: %s\n', save_filename);