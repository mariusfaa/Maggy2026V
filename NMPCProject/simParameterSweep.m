%% simParameterSweep — Sweep N_horizon and dt_mpc for all controllers
%
% Runs simAcadosNmpc, simAcadosLmpc, simAcadosSolmpc for every combination
% of N_horizon and dt_mpc. Results are saved per-run with the standard
% naming convention into results/.

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
params = load_params(modelId);

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

% --- Simulation time ---
dt = 0.0001;

% --- Initial conditions ---
x0_full = xEq_full + [-0.0007; 0.0005; 0.001; deg2rad(5); 0; 0; zeros(6,1)];
x0 = x0_full([1:5,7:11]);
u0 = [-0.25; 0.5; -0.5; 0.75];

% --- Build sim solver (depends only on dt, reuse across all runs) ---
if ~exist("sim_solver","var") || ~exist(fullfile('build','sim'), 'dir')
    model = getSimModel();
    sim_solver = getSimSolver(model, dt);
end

out_folder = fullfile(project_root, 'results');
if ~exist(out_folder, 'dir'); mkdir(out_folder); end
if ~exist('build', 'dir'); mkdir('build'); end

%% --- Sweep parameters ---
N_horizon_list = [10, 20, 30, 40, 50];
dt_mpc_list    = 0.001:0.001:0.003;

n_runs = numel(N_horizon_list) * numel(dt_mpc_list);
run = 0;

for i_N = 1:numel(N_horizon_list)
    for i_dt = 1:numel(dt_mpc_list)
        N_horizon = N_horizon_list(i_N);
        dt_mpc    = dt_mpc_list(i_dt);
        run = run + 1;

        assert(mod(dt_mpc, dt) == 0, 'dt_mpc must be a multiple of dt');

        fprintf('\n========== Run %d/%d: N=%d, dt_mpc=%.0f us ==========\n', ...
            run, n_runs, N_horizon, dt_mpc*1e6);

        % Reset time vector (sim scripts trim t on divergence)
        t = 0:dt:0.2;

        simAcadosNmpc;

        t = 0:dt:0.2;
        simAcadosLmpc;

        t = 0:dt:0.2;
        simAcadosSolmpc;
    end
end

fprintf('\n========== Parameter sweep complete: %d runs ==========\n', n_runs);
