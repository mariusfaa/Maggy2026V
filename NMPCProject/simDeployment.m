% Computes things needed for deployment in platformio project
clear; clc; close all;

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
cd(project_root);

import casadi.*

%% ================================================================
%  CONFIGURATION — edit this section
%  ================================================================

% Controllers to run (any subset of these)
out_folder = fullfile(project_root, 'results_deployment');
N_horizon = 20;
dt_mpc    = 0.001;

% Simulation settings
T_sim   = 0.5;       % simulation duration (s)
dt_plant = 0.0001;   % plant integration step (s)

% Initial perturbation from equilibrium
dx0 = zeros(12,1);

% Models (AcadosModel objects)
%   plant_model — used by acados sim_solver (the "real" plant)
%   ctrl_model  — used internally by all controllers
%                  (NMPC: OCP model, LMPC/SolNMPC/LQR: Jacobian source)
plant_model = getSimModel(32);    % accurate plant
ctrl_model  = getSimModel(16,1);    % controller internal model

% Skip existing results? (set false to overwrite)
skip_existing = false;

%% ================================================================
%  SETUP — paths, model, equilibrium, plant solver
%  ================================================================

modelId = MaglevModel.Accurate;
parameters_maggy_V4;

nx = 10;
nu = 4;
params_acc = params;
params_acc.magnet.n = 80;
params_acc.magnet.n_axial = 21;
[zEq, ~, ~, ~] = computeSystemEquilibria(params_acc, modelId);
xEq_full = [0; 0; zEq(1); zeros(9,1)];
xEq = xEq_full([1:5, 7:11]);
uEq = zeros(nu, 1);

x0_full = xEq_full + dx0;
x0 = x0_full([1:5, 7:11]);

% Build plant sim solver (shared across all runs)
%sim_solver = getSimSolver(plant_model, dt_plant);

if ~exist(out_folder, 'dir'); mkdir(out_folder); end
if ~exist('build', 'dir'); mkdir('build'); end

%% --- Linearize at equilibrium ---
fprintf('\n--- Computing linearization at equilibrium ---\n');

% we use the plant model because this is more accurate to what we would do
% for offline precomputation of the equilibrium anyways
x_cas  = plant_model.x;
u_cas  = plant_model.u;
f_expl = plant_model.f_expl_expr;

jac_fun = Function('jac_fun', {x_cas, u_cas}, ...
                   {jacobian(f_expl, x_cas), jacobian(f_expl, u_cas)});

[Ac_val, Bc_val] = jac_fun(xEq, uEq);
Ac = full(Ac_val);
Bc = full(Bc_val);

fprintf('  Continuous-time eigenvalues (real part): %s\n', ...
    mat2str(sort(real(eig(Ac)))', 4));

% Discretize via matrix exponential (ZOH)
M_exp = expm([Ac Bc; zeros(nu, nx+nu)] * dt_mpc);
Ad = M_exp(1:nx, 1:nx);
Bd = M_exp(1:nx, nx+1:end);

% Offset so xEq is a fixed point: x[k+1] = Ad*x[k] + Bd*u[k] + c
c_offset = (eye(nx) - Ad) * xEq;   % uEq = 0

fprintf('  Discrete-time eigenvalues (|z|): %s\n', ...
    mat2str(sort(abs(eig(Ad)))', 4));

%% --- OCP SETUP ---
fprintf('\n--- Setting up OCP ---\n');

x_sym = MX.sym('x', nx);
u_sym = MX.sym('u', nu);

mdl = AcadosModel();
mdl.name          = 'maglev_lmpc';
mdl.x             = x_sym;
mdl.u             = u_sym;
mdl.disc_dyn_expr = Ad * x_sym + Bd * u_sym + c_offset;

ocp = AcadosOcp();
ocp.model = mdl;

ocp.solver_options.N_horizon             = N_horizon;
ocp.solver_options.tf                    = dt_mpc * N_horizon;
ocp.solver_options.integrator_type       = 'DISCRETE';
ocp.solver_options.nlp_solver_type       = 'SQP';
ocp.solver_options.nlp_solver_max_iter   = 1;
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.ext_fun_compile_flags = '-O2';

ocp.cost        = getCost(xEq, uEq,dt_mpc);
ocp.constraints = getConstraints(x0);

solver_dir = fullfile('build', 'deployment');
ocp.code_gen_opts.code_export_directory = fullfile(solver_dir, 'c_generated_code');
ocp.code_gen_opts.json_file = fullfile(solver_dir, [mdl.name '_ocp.json']);
solver_creation_opts = struct('output_dir', solver_dir, ...
    'build', false, ...
    'generate', true, ...
    'check_reuse_possible', false, ...
    'compile_mex_wrapper', false);
ocp_solver = AcadosOcpSolver(ocp, solver_creation_opts);