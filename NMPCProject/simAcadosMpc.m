%% --- PROJECT SETUP ---
% Do NOT use 'clear all' or 'clear' — it unloads the acados MEX libraries
% from memory, causing slow re-initialisation on every re-run.
%clearvars -except ocp_solver sim_solver; clc;

simSetupInit;

acados_root  = '/home/h3rl/acados';
project_root = '/mnt/c/Users/halva/Downloads/Maggy2026V/NMPCProject';

%setenv('ACADOS_SOURCE_DIR',        acados_root);
%setenv('ENV_ACADOS_INSTALL_DIR',   acados_root);
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
x    = MX.sym('x',    nx);
u    = MX.sym('u',    nu);
xdot = MX.sym('xdot', nx);

fprintf('--- Setting up parameters ---\n');
paramsFast                 = params;
paramsFast.magnet.n        = 30;  % current sheet is more accurate → fewer points needed (was 100)
opts = struct();
opts.rho_max = 0.1;      % 10cm
opts.z_max = 0.06;       % 6cm
opts.method = 'linear';  % 'linear' is much faster in generated C code than 'bspline'
opts.N_rho = 100;        % denser grid compensates for linear interpolation
opts.N_z = 100;
paramsFast.luts = buildCurrentSheetLuts(paramsFast, opts);

%% --- BUILD CASADI FUNCTION ---
fprintf('--- Setting up casadi dynamics ---\n');
f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
f_func = casadi.Function('f', {x, u}, {f_expl});

%% --- PROFILE CASADI FUNCTION ---
fprintf('--- Profiling CasADi dynamics ---\n');
fprintf('  f_func: %d nodes in graph\n', f_func.n_nodes());
x_test = [0; 0; 0.03; zeros(9,1)];
u_test = zeros(nu, 1);
% Warm up
for i = 1:5, f_func(x_test, u_test); end
% Time function evaluation
n_eval = 200;
tic;
for i = 1:n_eval
    f_func(x_test, u_test);
end
t_func = toc / n_eval * 1000;
fprintf('  f(x,u) evaluation:  %.4f ms  (%.1f us)\n', t_func, t_func*1000);
% Time Jacobian
J_func = f_func.jacobian();
out_seed = ones(nx, 1);
for i = 1:5, J_func(x_test, u_test, out_seed); end
tic;
for i = 1:n_eval
    J_func(x_test, u_test, out_seed);
end
t_jac = toc / n_eval * 1000;
fprintf('  Jacobian evaluation: %.4f ms  (%.1f us)\n', t_jac, t_jac*1000);

%% --- FIND EQUILIBRIUM ---
fprintf('--- Searching for true equilibrium (z, u) ---\n');

if 0
    z_var    = MX.sym('z_eq');
    u_var    = MX.sym('u_eq', nu);
    x_eq_sym = [0; 0; z_var; zeros(9,1)];
    accel    = f_func(x_eq_sym, u_var);
    % xyz ax ay az dx dy dz dax day daz
    cost    = [accel(7:11); 1e7*u_var];
    cost     = cost' * cost;
    
    nlp       = struct('x', [z_var; u_var], 'f', cost);
    solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
        struct('ipopt', struct('print_level', 3, 'max_iter', 10000)));
    
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

end

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
ocp.solver_options.sim_method_num_steps  = 3;   % was 10 — IRK4 converges fast
ocp.solver_options.nlp_solver_type       = 'SQP_RTI';
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';


Q = diag([1e6, 1e6, 1e2, 1e1, 1e1, 0, 1e2, 1e2, 1e2, 1e2, 1e2, 0]);
Q = diag([1e6, 1e6, 1e1, 1e1, 1e1, 0, 1e1, 1e1, 1e2, 5e1, 5e1, 0]);
R = eye(nu) * 5e1;

ocp.cost.cost_type   = 'LINEAR_LS';
ocp.cost.cost_type_0 = 'LINEAR_LS';
ocp.cost.cost_type_e = 'LINEAR_LS';

ocp.cost.Vx_0 = blkdiag(Q,zeros(nu));
ocp.cost.Vx = blkdiag(Q,zeros(nu));
ocp.cost.Vx_e = Q;

ocp.cost.Vu_0 = blkdiag(zeros(nx),R);
ocp.cost.Vu = blkdiag(zeros(nx),R);

ocp.cost.W_0         = blkdiag(Q, R);
ocp.cost.W           = blkdiag(Q, R);
ocp.cost.W_e         = Q;

% ocp.model.cost_y_expr   = [x; u];
% ocp.model.cost_y_expr_0 = [x; u];
% ocp.model.cost_y_expr_e = x;

ocp.cost.yref   = [xEq; zeros(nu,1)];
ocp.cost.yref_0 = [xEq; zeros(nu,1)];
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
    sim.solver_options.num_steps       = 3;   % match OCP integrator

    if exist(fullfile('c_generated_code', 'maglev_sim'), 'dir')
        sim.solver_options.compile_interface = false;
    end
    sim_solver = AcadosSimSolver(sim);
else
    fprintf('\n--- Reusing sim solver from previous run ---\n');
end

%% --- HELPER: one NMPC step ---
n_rti_iter = 2;   % SQP_RTI designed for 1-2 iterations (was 5)

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
x_current = xEq + [0 0.0001 0.0005 pi/32 0 0 zeros(1, 6)].';
plot_x    = zeros(nx, sim_steps+1);
plot_u    = zeros(nu, sim_steps);
plot_x(:,1) = x_current;

% Re-warm-start before loop
for k = 0:N,   ocp_solver.set('x', xEq, k); end
for k = 0:N-1, ocp_solver.set('u', uEq, k); end

for i = 1:sim_steps
    [x_current, u_applied, status, diverged] = ...
        nmpc_step(ocp_solver, sim_solver, x_current, xEq, uEq, n_rti_iter, N);

    time_tot = ocp_solver.get('time_tot');
    fprintf('Step %3d: status=%d, solve_time=%.4f ms, |u|=%.4e\n', ...
        i, status, time_tot*1000, norm(u_applied));

    plot_x(:,i+1) = x_current;
    plot_u(:,i)   = u_applied;

    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
        break;
    end
end

%% --- SAVE ---
save_filename   = 'results_acados.mat';
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