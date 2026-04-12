% Compare ode simulator vs acados simulator for the accurate model

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

% Load a very 'Accurate' parameter set
modelId = MaglevModel.Accurate;
parameters_maggy_V41;
params.magnet.n = 80;
params.magnet.n_axial = 21;
save_filename = 'ode_accurate.mat';

% find equlibrium
[zEq, ~, ~, ~] = computeSystemEquilibria(params, modelId);
xEq_full = [0; 0; zEq(1); zeros(9,1)];
xEq = [0; 0; zEq(1); zeros(7,1)];
uEq = zeros(nu, 1);

f_ode = @(x, u) maglevSystemDynamics(x, u, params, modelId);

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
t = [0, t(end)];

tic;
[t_out, x_out] = ode15s(@(t, x) f_ode(x, u0), t, x0, opts);
t_elapsed = toc;
x_out = x_out';

fprintf('ode15s done in %.3f s (%d output points)\n', t_elapsed, size(x_out, 2));