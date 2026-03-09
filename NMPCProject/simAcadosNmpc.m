%% --- NMPC Simulation ---
% Do NOT use 'clear all' or 'clear' — it unloads the acados MEX libraries
% from memory, causing slow re-initialisation on every re-run.
%clearvars -except model ocp_solver sim_solver; clc;

simSetup;

x0 = xEq + [0 0.0005 0.001 0 0 0 zeros(1, 6)].';
umax = 1;

acados_root  = '/home/halva/acados';
project_root = '/mnt/c/Users/halva/Downloads/Maggy2026V/NMPCProject';

setenv('ACADOS_INSTALL_DIR', acados_root);

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_impl_casadi')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

save_filename = 'results_acados_mpc.mat';

%% --- MPC parameters ---
dt_mpc = 0.001;
N_horizon = 10;              % Number of shooting intervals
Tf        = dt_mpc*N_horizon;          % Prediction horizon [s]

%% --- Model setup ---
fprintf('--- Setting up model (Fast, analytical) ---\n');

if ~exist("model","var")
    model_opts.field_method = 'analytical';
    model = get_maggy_model(params, MaglevModel.Fast, model_opts);
end

%% --- OCP SETUP ---
fprintf('\n--- Setting up OCP ---\n');

if ~exist("ocp_solver","var")
    ocp = AcadosOcp();
    ocp.model = model;

    % Solver options — tuned for speed
    ocp.solver_options.N_horizon             = N_horizon;
    ocp.solver_options.tf                    = Tf;
    ocp.solver_options.integrator_type       = 'ERK';
    ocp.solver_options.sim_method_num_stages = 4;
    ocp.solver_options.sim_method_num_steps  = 1;
    ocp.solver_options.nlp_solver_type       = 'SQP_RTI';  % single iteration per call
    ocp.solver_options.qp_solver             = 'FULL_CONDENSING_HPIPM';  % dense QP, fast for small N
    ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';
    ocp.solver_options.globalization         = 'FIXED_STEP';  % no line search
    ocp.solver_options.regularize_method     = 'NO_REGULARIZE';  % GN is already PSD
    ocp.solver_options.qp_solver_warm_start  = 2;  % warm-start primal+dual
    ocp.solver_options.levenberg_marquardt   = 1e-4;  % small damping
    ocp.solver_options.ext_fun_compile_flags = '-O3';  % aggressive optimization

    % --- Cost: LINEAR_LS ---
    % y = Vx*x + Vu*u,  cost = (y - yref)' * W * (y - yref)
    %
    % State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
    Q = diag([...
        1e6, 1e6, ...       % x, y — lateral positioning
        1e5, ...             % z — critical for levitation
        1e2, 1e2, 0, ...    % roll, pitch, yaw (yaw free)
        1e1, 1e1, 1e1, ...  % linear velocities
        1e1, 1e1, 0 ...     % angular velocities (wz free)
    ]);
    R = eye(nu) * 1e1;      % Input penalty (low to allow actuation)

    ocp.cost.cost_type   = 'LINEAR_LS';
    ocp.cost.cost_type_0 = 'LINEAR_LS';
    ocp.cost.cost_type_e = 'LINEAR_LS';

    ny_cost = nx + nu;
    ocp.cost.Vx   = [eye(nx); zeros(nu, nx)];   % (nx+nu) x nx
    ocp.cost.Vu   = [zeros(nx, nu); eye(nu)];   % (nx+nu) x nu
    ocp.cost.Vx_0 = ocp.cost.Vx;
    ocp.cost.Vu_0 = ocp.cost.Vu;
    ocp.cost.Vx_e = eye(nx);                    % nx x nx (terminal, no u)

    ocp.cost.W   = blkdiag(Q, R);
    ocp.cost.W_0 = blkdiag(Q, R);
    ocp.cost.W_e = Q;

    ocp.cost.yref   = [xEq; uEq];
    ocp.cost.yref_0 = [xEq; uEq];
    ocp.cost.yref_e = xEq;

    % Input constraints
    ocp.constraints.idxbu = 0:nu-1;
    ocp.constraints.lbu   = -umax * ones(nu, 1);
    ocp.constraints.ubu   =  umax * ones(nu, 1);
    ocp.constraints.x0    = x0;

    ocp_solver = AcadosOcpSolver(ocp);
end

% Warm-start: initialize all shooting nodes to equilibrium
for k = 0:N_horizon
    ocp_solver.set('x', xEq, k);
end
for k = 0:N_horizon-1
    ocp_solver.set('u', uEq, k);
end

%% --- SIM SOLVER (plant model) ---
fprintf('\n--- Setting up sim solver ---\n');

if ~exist("sim_solver","var")
    sim = AcadosSim();
    sim.model = model;

    sim.solver_options.Tsim            = dt;
    sim.solver_options.integrator_type = 'ERK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 1;

    sim_solver = AcadosSimSolver(sim);
end

%% --- Simulation loop ---
% MPC runs at dt_mpc = Tf/N_horizon. Plant sim runs at dt.
% Between MPC calls, control is held constant for n_sub plant steps.
n_sub  = round(dt_mpc / dt);
%assert(abs(n_sub * dt - dt_mpc) < 1e-12, ...
%    'dt_mpc (%.2e) must be an integer multiple of dt (%.2e)', dt_mpc, dt);

N_sim   = numel(t);          % total plant steps
N_mpc   = floor(N_sim / n_sub);  % number of MPC calls

x_sim = zeros(nx, N_sim);
u_sim = zeros(nu, N_sim);
t_mpc_log  = zeros(1, N_mpc);   % MPC solve time per call
t_sub_log  = zeros(1, N_mpc);   % Plant sim time per MPC interval
t_step_log = zeros(1, N_mpc);   % Total time per MPC interval
% acados internal timing
t_lin_log  = zeros(1, N_mpc);
t_qp_log   = zeros(1, N_mpc);
t_reg_log  = zeros(1, N_mpc);

x = x0;
u = uEq;

fprintf('\n--- Running NMPC simulation ---\n');
fprintf('  T=%.3fs, dt=%.1f us, dt_mpc=%.1f us, n_sub=%d\n', ...
    t(end), dt*1e6, dt_mpc*1e6, n_sub);
fprintf('  Plant steps: %d, MPC calls: %d\n', N_sim, N_mpc);

diverged = false;

for k = 1:N_mpc
    tic_step = tic;

    % --- MPC solve ---
    tic_mpc = tic;
    ocp_solver.set('constr_x0', x);
    ocp_solver.solve();
    t_mpc_log(k) = toc(tic_mpc);

    t_lin_log(k) = ocp_solver.get('time_lin');
    t_qp_log(k)  = ocp_solver.get('time_qp_sol');
    t_reg_log(k) = ocp_solver.get('time_reg');

    status = ocp_solver.get('status');
    if status ~= 0 && status ~= 2
        fprintf('  *** OCP solver warning at MPC step %d (status %d) ***\n', k, status);
    end

    % Extract first control action (receding horizon)
    u = ocp_solver.get('u', 0);

    % Warm-shift solution for next call
    for i = 0:N_horizon-2
        ocp_solver.set('x', ocp_solver.get('x', i+1), i);
        ocp_solver.set('u', ocp_solver.get('u', i+1), i);
    end
    ocp_solver.set('x', ocp_solver.get('x', N_horizon), N_horizon);
    ocp_solver.set('u', ocp_solver.get('u', N_horizon-1), N_horizon-1);

    % --- Sub-step: simulate plant n_sub times with held u ---
    tic_sub = tic;
    for j = 1:n_sub
        idx = (k-1)*n_sub + j;
        if idx > N_sim, break; end

        x_sim(:, idx) = x;
        u_sim(:, idx) = u;

        x = sim_solver.simulate(x, u);
    end
    t_sub_log(k) = toc(tic_sub);

    t_step_log(k) = toc(tic_step);

    % Divergence check
    I = [1:5,7:11];
    diverged = abs(x(3)) > 0.5       || ...
               max(abs(x(4:5))) > pi  || ...
               any(isnan(x(I)))          || ...
               any(isinf(x(I)));

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  mpc=%.0f us (lin=%.0f qp=%.0f)  sim=%.0f us  total=%.0f us\n', ...
        k, x(3)*1e3, norm(u), ...
        t_mpc_log(k)*1e6, t_lin_log(k)*1e6, t_qp_log(k)*1e6, ...
        t_sub_log(k)*1e6, t_step_log(k)*1e6);

    if diverged
        fprintf('  *** DIVERGED at MPC step %d ***\n', k);
        last_idx = min((k)*n_sub, N_sim);
        x_sim = x_sim(:, 1:last_idx);
        u_sim = u_sim(:, 1:last_idx);
        t = t(1:last_idx);
        t_mpc_log  = t_mpc_log(1:k);
        t_sub_log  = t_sub_log(1:k);
        t_step_log = t_step_log(1:k);
        break;
    end
end

%% --- Performance summary ---
fprintf('\n--- NMPC Performance ---\n');
fprintf('MPC  solve: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_mpc_log)*1e6, max(t_mpc_log)*1e6, median(t_mpc_log)*1e6);
fprintf('  linearize: mean=%.0f us, median=%.0f us\n', ...
    mean(t_lin_log)*1e6, median(t_lin_log)*1e6);
fprintf('  QP solve:  mean=%.0f us, median=%.0f us\n', ...
    mean(t_qp_log)*1e6, median(t_qp_log)*1e6);
fprintf('  regularize:mean=%.0f us, median=%.0f us\n', ...
    mean(t_reg_log)*1e6, median(t_reg_log)*1e6);
fprintf('Plant sim:  mean=%.0f us, max=%.0f us, median=%.0f us  (%d sub-steps)\n', ...
    mean(t_sub_log)*1e6, max(t_sub_log)*1e6, median(t_sub_log)*1e6, n_sub);
fprintf('Total step: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_step_log)*1e6, max(t_step_log)*1e6, median(t_step_log)*1e6);
fprintf('Real-time factor: %.2fx (dt_mpc=%.0f us, avg step=%.0f us)\n', ...
    dt_mpc / mean(t_step_log), dt_mpc*1e6, mean(t_step_log)*1e6);
fprintf('\nFinal state: z=%.4f mm (eq=%.4f mm)\n', x(3)*1e3, xEq(3)*1e3);
fprintf('Final |pos error|=%.4f mm, |ang error|=%.4f deg\n', ...
    norm(x(1:3)-xEq(1:3))*1e3, norm(x(4:6)-xEq(4:6))*180/pi);

%% --- SAVE ---
sim_data        = struct();
sim_data.t      = t;
sim_data.x      = x_sim;
sim_data.u      = u_sim;
sim_data.xEq    = xEq;
sim_data.uEq    = uEq;
sim_data.dt     = dt;
sim_data.t_mpc  = t_mpc_log;
sim_data.t_sim  = t_sub_log;
sim_data.t_step = t_step_log;

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
