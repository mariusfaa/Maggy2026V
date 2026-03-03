%% --- PROJECT SETUP ---
% Do NOT use 'clear all' or 'clear' — it unloads the acados MEX libraries
% from memory, causing slow re-initialisation on every re-run.
%clearvars -except ocp_solver sim_solver; clc;

simSetupInit;

acados_root  = '/home/h3rl/acados';
project_root = '/mnt/c/Users/halva/Downloads/Maggy2026V/NMPCProject';

setenv('ACADOS_INSTALL_DIR',       acados_root);

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% --- Model setup ---

fprintf('--- Setting up parameters ---\n');

params.magnet.n        = 100;  % current sheet is more accurate → fewer points needed (was 100)
opts = struct();
opts.rho_max = 0.1;      % 10cm
opts.z_max = 0.06;       % 6cm
opts.method = 'linear';  % 'linear' is much faster in generated C code than 'bspline'
opts.N_rho = 100;        % denser grid compensates for linear interpolation
opts.N_z = 100;
params.luts = buildCurrentSheetLuts(params, opts);

%% --- OCP SETUP ---
% N      = 10;
% Tf     = 0.1;
% dt_mpc = Tf / N;   % 0.01 s
% 
% ocp = AcadosOcp();
% ocp.model = get_maggy_model(params);

%% --- BUILD / REUSE SIM SOLVER ---

fprintf('\n--- Building acados sim solver ---\n');
if exist('c_generated_code/maglev_sim', 'dir')
    rmdir('c_generated_code/maglev_sim', 's');
end
sim = AcadosSim();
sim.model = get_maggy_model(params);

sim.solver_options.Tsim            = dt;
sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages      = 4;
sim.solver_options.num_steps       = 1;   % match OCP integrator

sim_solver = AcadosSimSolver(sim);

u = u0;
x = x0;

x_traj = zeros(nx,numel(t));
u_traj = zeros(nu,numel(t));

x_traj(:,1)=x0;
u_traj(:,1)=u0;

for i=1:numel(t)
    sim_solver.set('x', x);
    sim_solver.set('u', u);
    sim_solver.solve();
    x = sim_solver.get('xn');

    x_traj(:,i+1)=x;
    u_traj(:,i+1)=u;

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
save_filename   = 'results_acados.mat';
sim_data        = struct();
sim_data.t      = [t, t(end)+dt];  % x_traj has numel(t)+1 columns (x0 + one per step)
sim_data.x      = x_traj;
sim_data.u      = u_traj;
sim_data.xEq    = xEq;
sim_data.uEq    = uEq;
sim_data.dt     = dt;
sim_data.params = params;

save(save_filename, '-struct', 'sim_data');
fprintf('\nSimulation data saved to: %s\n', save_filename);