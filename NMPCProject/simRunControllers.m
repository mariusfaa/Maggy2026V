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
addpath(genpath(fullfile(project_root, 'model_reduced_casadi')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

cd(project_root);

%% --- Parameters ---
modelId = MaglevModel.Accurate;
parameters_maggy_V4;

nx = 10;
nu = 4;

% --- Equilibrium ---
params_acc = params;
params_acc.magnet.n = 80;
params_acc.magnet.n_axial = 21;
[zEq, ~, ~, ~] = computeSystemEquilibria(params, modelId);
xEq_full = [0; 0; zEq(1); zeros(9,1)];
xEq = [0; 0; zEq(1); zeros(7,1)];
uEq = zeros(nu, 1);

% --- MPC parameters ---
% define what you want to simulate here
types = {'nmpc','solmpc','lmpc'};
N_horizon = 30; % can be array
dt_mpc = 0.001; % can be array

% --- Simulation time ---
dt = 0.0001;
t  = 0:dt:0.2;

% --- Initial conditions ---
x0_full = xEq_full + [-0.0005; 0.00025; 0.0007; deg2rad(5); 0; 0; zeros(6,1)];
x0 = x0_full([1:5,7:11]); 
u0 = [-0.25; 0.5; -0.5; 0.75];

% Setup sim object — rebuild if variable missing or build dir was cleaned
if ~exist("sim_solver","var")
    model = getSimModel();
    sim_solver = getSimSolver(model, dt);
end

if ~exist("results", "dir"); mkdir("results"); end
% --- Run all combinations of N_horizon, dt_mpc, and types ---

% Step 1: build struct array of all run configurations
runs = {};
k = 1;

for i = 1:length(types)
    for j = 1:length(N_horizon)
        for l = 1:length(dt_mpc)
            
            runs{k}.type    = types{i};
            runs{k}.N_hor   = N_horizon(j);
            runs{k}.dt_mpc  = dt_mpc(l);
            runs{k}.x0      = x0;
            runs{k}.u0      = u0;

            % create unique filename
            runs{k}.outfile = sprintf("results/%s_N%d_dt%.4g.mat", ...
                types{i}, N_horizon(j), dt_mpc(l));

            k = k + 1;
        end
    end
end

% Step 2: iterate over runs and execute
for k = 1:length(runs)
    run = runs{k};

    % check if .mat file already exists

    fprintf("Running %s | N=%d | dt=%.4g\n", ...
        run.type, run.N_hor, run.dt_mpc);

    switch run.type
        case 'nmpc'
            simAcadosNmpc(run);

        case 'lmpc'
            simAcadosLmpc(run);

        case 'solmpc'
            simAcadosSolmpc(run);

        otherwise
            error("Unknown type: %s", run.type);
    end
end

% Step 3: compare results
files = getFiles(types, N_horizon, dt_mpc);
clear prefix;
aCompareSimulations;

return;