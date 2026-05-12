%clear; clc; close all;

acados_root  = getenv('ACADOS_INSTALL_DIR');
project_root = getenv('ACADOS_PROJECT_DIR');
assert(~isempty(acados_root),  'ACADOS_INSTALL_DIR not set. Source env.sh first.');
assert(~isempty(project_root), 'ACADOS_PROJECT_DIR not set. Source env.sh first.');

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));
addpath(genpath(fullfile(project_root, 'model_matlab')));
addpath(genpath(fullfile(project_root, 'model_reduced_casadi')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

addpath(fullfile(project_root, 'tinympc-matlab',   'build'));
addpath(fullfile(project_root, 'tinympc-matlab',   'src'));

cd(project_root);

import casadi.*

% --- Define the Function ---
x = MX.sym('x', 10, 1);
u = MX.sym('u', 4, 1);
dt_mpc = 0.001;

% Ensure getLinsys returns MX symbols compatible with Function

[Ad, Bd, c_lin] = getLinsys(x, u, dt_mpc);
lsys = Function('lsys', {x, u}, {Ad, Bd, c_lin});

% --- Setup Build Directory ---
current_dir = pwd;
build_dir = fullfile(project_root, 'build', 'util');
mex_file = fullfile(build_dir, "lsys.mexa64");

if ~exist(build_dir, 'dir')
    mkdir(build_dir);
end

if ~exist(mex_file, 'file')
    
    cd(build_dir);
    % --- Generate and Compile ---
    % Generate C code with MEX support
    lsys.generate('lsys.c', struct('mex', true));
    clear lsys;
    
    % Compile using MATLAB's MEX compiler
    % -O enables optimizations
    try
        mex lsys.c -O
        fprintf('MEX compilation of lsys successful.\n');
    catch ME
        cd(current_dir);
        rethrow(ME);
    end

    % Return to project root
    cd(current_dir);
end

addpath(build_dir); % Add build folder so MATLAB finds the new .mex file

% install osqp
if ~exist('install_osqp.m', 'file')
    websave('install_osqp.m','https://raw.githubusercontent.com/osqp/osqp-matlab/master/package/install_osqp.m');
    install_osqp
end

% loop osqp


if ~exist('sim_solver', 'var')
    sim_solver = getSimSolver(getSimModel(32,1),0.001);
end

nx = 10;
nu = 4;

% Constraints
u0 = zeros(nu,1);
% xEq = zeros(nx,1); xEq(3,1) = 0.0306;
% x0 = xEq; x0(3,1) = 0.04; x0(4,1) = deg2rad(0);
umin = -ones(nu,1);
umax = ones(nu,1);

[Ad, Bd, c_lin] = lsys(xEq, u0);
Ad = full(Ad); Bd = full(Bd); c_lin = full(c_lin);

% Objective function
cost = getCostTerms(xEq,u0,0.001,true);
Q = cost.Q;
QN = cost.Qt;
R = cost.R;

% Initial and reference states
xr = xEq;

% Prediction horizon
N = 20;

% Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
% - quadratic objective
P = blkdiag( kron(speye(N), Q), QN, kron(speye(N), R) );
% - linear objective
q = [repmat(-Q*xr, N, 1); -QN*xr; zeros(N*nu, 1)];
% - linear dynamics
Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), Ad);
Bu = kron([sparse(1, N); speye(N)], Bd);
Aeq = [Ax, Bu];
leq = [-x0; repmat(-c_lin, N, 1)];
ueq = leq;
% - input constraints only (no state bounds, matching acados solnmpc)
Aineq = [sparse((N+1)*nx, (N+1)*nx + N*nu); ...
         sparse(N*nu, (N+1)*nx), speye(N*nu)];
lineq = [-Inf((N+1)*nx + N*nu, 1)];
lineq(end-N*nu+1:end) = repmat(umin, N, 1);
uineq = [Inf((N+1)*nx + N*nu, 1)];
uineq(end-N*nu+1:end) = repmat(umax, N, 1);
% - OSQP constraints
A = [Aeq; Aineq];
l = [leq; lineq];
u = [ueq; uineq];

% Create an OSQP object
prob = osqp;

% Setup workspace (tighten tolerances for MPC accuracy)
prob.setup(P, q, A, l, u, ...
    'eps_abs', 1e-7, ...
    'eps_rel', 1e-7, ...
    'max_iter', 4000, ...
    'polish', true);

% Simulate in closed loop
nsim = 500;
xhist = zeros(nx, nsim+1);
xhist(:,1) = x0;
for i = 1 : nsim
    % Solve
    res = prob.solve();

    % Check solver status
    if ~strcmp(res.info.status, 'solved')
        error('OSQP did not solve the problem!')
    end

    % Apply first control input to the plant
    ctrl = res.x((N+1)*nx+1:(N+1)*nx+nu);
    x0 = sim_solver.simulate(x0, ctrl);% x0 = Ad*x0 + Bd*ctrl; using acados sim
    xhist(:,i+1) = x0;

    % Update initial state
    l(1:nx) = -x0;
    u(1:nx) = -x0;
    % Re-linearize at current state
    [Ad,Bd,c_lin] = lsys(x0,ctrl);
    Ad = full(Ad); Bd = full(Bd); c_lin = full(c_lin);

    % Update dynamics constraints (preserve sparsity pattern)
    Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), sparse(Ad));
    Bu = kron([sparse(1, N); speye(N)], sparse(Bd));
    Aeq = [Ax, Bu];
    leq = [-x0; repmat(-c_lin, N, 1)];
    ueq = leq;
    A_new = [Aeq; Aineq];
    % Ensure same sparsity pattern as original A
    [iA, jA] = find(A);
    A_new_fixed = sparse(iA, jA, full(A_new(sub2ind(size(A_new), iA, jA))), size(A,1), size(A,2));
    l = [leq; lineq];
    u = [ueq; uineq];
    prob.update('Ax', nonzeros(A_new_fixed), 'l', l, 'u', u);
end

% Plot z-position over time
figure;
t = (0:nsim) * dt_mpc;
plot(t*1e3, xhist(3,:)*1e3, '-o');
yline(xEq(3)*1e3, '--r');
xlabel('Time [ms]');
ylabel('z [mm]');
title('z-position trajectory');
legend('z', 'z_{eq}');
grid on;