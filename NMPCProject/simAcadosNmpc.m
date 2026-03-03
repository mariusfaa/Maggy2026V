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

%% --- Model setup ---

fprintf('--- Setting up model ---\n');

model = get_maggy_model(params);

%% --- OCP SETUP ---
fprintf('\n--- Setting up OCP ---\n');

N      = 20;
Tf     = N * dt;
dt_mpc = dt;

ocp = AcadosOcp();
ocp.model = model;

ocp.solver_options.N_horizon             = N;
ocp.solver_options.tf                    = Tf;
ocp.solver_options.integrator_type       = 'ERK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 1;
ocp.solver_options.nlp_solver_type       = 'SQP';
ocp.solver_options.nlp_solver_max_iter   = 100;
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';

% Cost weights
Q = diag([1e6, 1e6, 1e2, 1e1, 1e1, 0, 1e2, 1e2, 1e2, 1e2, 1e2, 0]);
R = eye(nu) * 5e1;

% LINEAR_LS cost: y = Vx*x + Vu*u, cost = (y - yref)' * W * (y - yref)
ocp.cost.cost_type   = 'LINEAR_LS';
ocp.cost.cost_type_0 = 'LINEAR_LS';
ocp.cost.cost_type_e = 'LINEAR_LS';

% Selection matrices: y = [x; u]
ny = nx + nu;
ocp.cost.Vx   = [eye(nx); zeros(nu, nx)];   % ny x nx
ocp.cost.Vu   = [zeros(nx, nu); eye(nu)];   % ny x nu
ocp.cost.Vx_0 = ocp.cost.Vx;
ocp.cost.Vu_0 = ocp.cost.Vu;
ocp.cost.Vx_e = eye(nx);                    % nx x nx (terminal, no u)

% Weight matrices
ocp.cost.W   = blkdiag(Q, R);
ocp.cost.W_0 = blkdiag(Q, R);
ocp.cost.W_e = Q;

% References
ocp.cost.yref   = [xEq; uEq];
ocp.cost.yref_0 = [xEq; uEq];
ocp.cost.yref_e = xEq;

ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu   = -1 * ones(nu,1);
ocp.constraints.ubu   =  1 * ones(nu,1);
ocp.constraints.x0    = x0;

ocp_solver = AcadosOcpSolver(ocp);

% Warm-start: initialize all shooting nodes
for k = 0:N
    ocp_solver.set('x', xEq, k);
end
for k = 0:N-1
    ocp_solver.set('u', uEq, k);
end

%% --- SIM SETUP ---
fprintf('\n--- Setting up SIM ---\n');

sim = AcadosSim();
sim.model = model;

sim.solver_options.Tsim            = dt;
sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages      = 4;
sim.solver_options.num_steps       = 1;   % match OCP integrator

sim_solver = AcadosSimSolver(sim);


%% --- Simulation loop ---
N_sim = 200;
x_sim = zeros(nx, N_sim+1);
u_sim = zeros(nu, N_sim);

x_sim(:,1) = x0;

for i = 1:N_sim
    % Update initial state constraint
    ocp_solver.set('constr_x0', x_sim(:,i));

    % Solve OCP
    ocp_solver.solve();
    status = ocp_solver.get('status');
    if status ~= 0
        fprintf('  *** OCP solver failed at step %d (status %d) ***\n', i, status);
    end

    % Extract first control action
    u = ocp_solver.get('u', 0);
    u_sim(:,i) = u;

    % Simulate one step with plant model
    sim_solver.set('x', x_sim(:,i));
    sim_solver.set('u', u);
    sim_solver.solve();
    x_sim(:,i+1) = sim_solver.get('xn');

    % --- Divergence check ---
    xn = x_sim(:,i+1);
    diverged = abs(xn(3)) > 0.5       || ...
               max(abs(xn(4:5))) > pi  || ...
               any(isnan(xn))          || ...
               any(isinf(xn));

    fprintf('Step %3d: |x|=%.4e  status=%d\n', i, norm(xn([1:5,7:11])), status);

    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
        break;
    end
end

%% --- SAVE ---
save_filename   = 'results_acados_mpc.mat';
sim_data        = struct();
ylsim_data.t      = [t, t(end)+dt];  % x_traj has numel(t)+1 columns (x0 + one per step)
sim_data.x      = x_sim;
sim_data.u      = u_sim;
sim_data.xEq    = xEq;
sim_data.uEq    = uEq;
sim_data.dt     = dt;
sim_data.params = params;

save(save_filename, '-struct', 'sim_data');
fprintf('\nSimulation data saved to: %s\n', save_filename);