%% simSetup — single source of truth for all simulation scripts
%
% Provides: acados_root, project_root, params, modelId,
%           nx, nu, xEq, uEq, dt, t, x0, u0

%% --- Paths (from environment variables set by env.sh) ---
acados_root  = getenv('ACADOS_INSTALL_DIR');
project_root = getenv('ACADOS_PROJECT_DIR');

assert(~isempty(acados_root),  'ACADOS_INSTALL_DIR not set. Source env.sh first.');
assert(~isempty(project_root), 'ACADOS_PROJECT_DIR not set. Source env.sh first.');

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_matlab')));
% addpath(genpath(fullfile(project_root, 'model_casadi')));
addpath(genpath(fullfile(project_root, 'model_reduced_casadi')));
addpath(genpath(fullfile(project_root, 'model_dipole_casadi')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

%% --- Parameters ---
modelId = MaglevModel.Accurate;
params = load_params(modelId);

nx = 12;
nu = length(params.solenoids.r);

%% --- Equilibrium ---
[zEq, ~, ~, ~] = computeSystemEquilibria(params, modelId);

xEq = [0; 0; zEq(1); zeros(9,1)];
uEq = zeros(nu, 1);

%% --- Simulation time ---
dt = 0.001;
t  = 0:dt:0.5;

%% --- Initial conditions ---
x0 = xEq + [0; 0.001; 0.001; 0; 0; 0; zeros(6,1)];
u0 = [-0.25; 0.5; -0.5; 0.75];
