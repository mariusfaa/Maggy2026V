% setup project default paths
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

import casadi.*

nx = 10; nu = 4;

problem = struct();

% Model settings
problem.umax = 1.0;
problem.plant_model = getSimModel(32);    % accurate plant
problem.ctrl_model  = getSimModel(16,1);    % controller internal model

% Cost matricies
problem.Q = diag([...
    1e6, 1e6, ...       % x, y (lateral — less critical)
    1e7, ...             % z (unstable direction — highest priority)
    1e2, 1e2, ...       % roll, pitch
    1e0, 1e0, 1e0, ...  % vx, vy, vz (vz high for damping)
    1e1, 1e1 ...        % wx, wy
]);
problem.R = eye(nu) * 1e0; % actuator penalty
problem.Qt = problem.Q; % terminal cost

% Controller selection
problem.name = 'default';
problem.controllers = {'lqr', 'lmpc', 'solnmpc', 'nmpc'};
problem.outfolder = fullfile(project_root, 'results');
problem.N = [20];
problem.dt_mpc = [0.001];
problem.dt_plant = 0.0001;
problem.T_sim = 0.1; % simulation time in seconds
problem.dx0 = zeros(nx,1); % Problem pertubation from equilibrium
problem.xEq = getSimEquilibrium(problem.plant_model,MX.zeros(nx,1));

problem.sim_solver = getSimSolver(problem.plant_model, problem.dt_plant);