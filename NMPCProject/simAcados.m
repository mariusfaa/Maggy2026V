%% --- PROJECT SETUP ---
% Do NOT use 'clear all' or 'clear' — it unloads the acados MEX libraries
% from memory, causing slow re-initialisation on every re-run.
%clearvars -except ocp_solver sim_solver; clc;

simSetup;

acados_root  = '/home/halva/acados';
project_root = '/mnt/c/Users/halva/Downloads/Maggy2026V/NMPCProject';

setenv('ACADOS_INSTALL_DIR',       acados_root);

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

save_filename   = 'results_acados.mat';

%% --- Model setup ---

fprintf('--- Setting up model ---\n');

if ~exist("model","var")
    model = get_maggy_model(params, MaglevModel.Fast);
end

%% --- BUILD / REUSE SIM SOLVER ---

fprintf('\n--- Building acados sim solver ---\n');

if ~exist("sim_solver","var")
    sim = AcadosSim();
    sim.model = model;
    
    sim.solver_options.Tsim            = dt;
    sim.solver_options.integrator_type = 'ERK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 1;   % match OCP integrator
    
    sim_solver = AcadosSimSolver(sim);
end

x_traj = zeros(nx,numel(t));
u_traj = zeros(nu,numel(t));

u = u0;
x = x0;

for i=1:numel(t)
    % record state BEFORE stepping (so x_traj(:,1) = x0 at t=0)
    x_traj(:,i)=x;
    u_traj(:,i)=u;

    % simulate step
    x = sim_solver.simulate(x,u);

    % --- Divergence check ---
    diverged = abs(x(3)) > 0.5       || ...
               max(abs(x(4:6))) > pi  || ...
               any(isnan(x))          || ...
               any(isinf(x));

    fprintf('Step %3d:|x|=%.4e\n', i, norm(x([1:5,7:11])));

    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
    end
end

%% --- SAVE ---
sim_data        = struct();
sim_data.t      = t;
sim_data.x      = x_traj;
sim_data.u      = u_traj;
sim_data.xEq    = xEq;
sim_data.uEq    = uEq;
sim_data.dt     = dt;
sim_data.params = params;

save(save_filename, '-struct', 'sim_data');
fprintf('\nSimulation data saved to: %s\n', save_filename);