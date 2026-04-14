%% simMain — Single entry point for running and comparing controllers
%
% Configure which controllers to run, sweep parameters, and simulation
% settings in the section below. Results are saved per-run to results/
% and a comparison plot is generated at the end.
%
% Usage:
%   simMain              — run with defaults below
%   Edit the config section, then run again

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
controllers = {'lqr', 'lmpc', 'solnmpc', 'nmpc'};
out_folder = fullfile(project_root, 'results');
N_horizon_list = [11, 15:5:40];
dt_mpc_list    = [0.001,0.002,0.003];

% Sweep parameters (lists — all combinations are run)
% Configuration for sc1, where dare terminal cost on
N_horizon_list = [11, 15:5:40];
dt_mpc_list    = [0.001,0.0015,0.002];

% Configuration for dare vs 10Q
controllers = {'lmpc', 'solnmpc', 'nmpc'};
N_horizon_list = [10,20,30];
dt_mpc_list    = [0.001];
out_folder = fullfile(project_root, 'results_nodare');

% Simulation settings
T_sim   = 0.5;       % simulation duration (s)
dt_plant = 0.0001;   % plant integration step (s)

% Initial perturbation from equilibrium
dx0 = [-0.0005; 0.00025; 0.0007; deg2rad(5); 0; 0; zeros(6,1)];

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
sim_solver = getSimSolver(plant_model, dt_plant);

if ~exist(out_folder, 'dir'); mkdir(out_folder); end
if ~exist('build', 'dir'); mkdir('build'); end

% Alias for sim scripts that use 'dt' instead of 'dt_plant'
dt = dt_plant;

%% ================================================================
%% RUN SWEEP
%% ================================================================

n_ctrl = numel(controllers);
n_N    = numel(N_horizon_list);
n_dt   = numel(dt_mpc_list);
n_total = n_ctrl * n_N * n_dt;
run_count = 0;

all_files = {};  % collect result filenames for plotting

for i_N = 1:n_N
    for i_dt = 1:n_dt
        N_horizon = N_horizon_list(i_N);
        dt_mpc    = dt_mpc_list(i_dt);

        assert(mod(dt_mpc, dt_plant) == 0, ...
            'dt_mpc (%.0f us) must be a multiple of dt_plant (%.0f us)', ...
            dt_mpc*1e6, dt_plant*1e6);

        for i_c = 1:n_ctrl
            ctrl = controllers{i_c};
            run_count = run_count + 1;

            save_filename = fullfile(out_folder, ...
                getFilename(ctrl, N_horizon, dt_mpc));

            if skip_existing && isfile(save_filename)
                fprintf('[%d/%d] SKIP %s (exists)\n', run_count, n_total, save_filename);
                all_files{end+1} = save_filename; %#ok<SAGROW>
                continue;
            end

            fprintf('\n========== [%d/%d] %s | N=%d | dt_mpc=%.0f us ==========\n', ...
                run_count, n_total, upper(ctrl), N_horizon, dt_mpc*1e6);

            t = 0:dt_plant:T_sim;

            switch ctrl
                case 'lqr',    simAcadosLqr;
                case 'lmpc',   simAcadosLmpc;
                case 'solnmpc', simAcadosSolnmpc;
                case 'nmpc',   simAcadosNmpc;
                otherwise
                    error('Unknown controller: %s', ctrl);
            end

            all_files{end+1} = save_filename; %#ok<SAGROW>
        end
    end
end

fprintf('\n========== Sweep complete: %d runs ==========\n', n_total);

%% ================================================================
%% LOAD ALL RESULTS
%% ================================================================

% De-duplicate file list
all_files = unique(all_files);

results = {};
for i = 1:numel(all_files)
    if ~isfile(all_files{i})
        fprintf('  MISSING: %s\n', all_files{i});
        continue;
    end
    d = load(all_files{i});
    d.file = all_files{i};
    results{end+1} = d; %#ok<SAGROW>
end

if isempty(results)
    fprintf('No results to plot.\n');
    return;
end

fprintf('Loaded %d result files.\n', numel(results));

%% ================================================================
%% COMPARISON PLOTS
%% ================================================================

%simMainPlot(results, xEq, uEq);

fprintf('\nDone.\n');
