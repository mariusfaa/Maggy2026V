%% simAcadosNmpc_crossmodel — Dipole MPC controller + Accurate plant
%
% The MPC uses the Dipole model internally (fast predictions).
% The plant is simulated with the Accurate model (ground truth).
% This tests whether the dipole model is good enough for control.

simSetup;
import casadi.*

umax = 1;
nx = 10;

save_filename = 'results_crossmodel_dipole_ctrl_accurate_plant.mat';

%% --- Equilibrium ---
% Use Accurate equilibrium as ground truth reference
params_acc = load_params(MaglevModel.Accurate);
[zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);

% Dipole equilibrium (for MPC reference — should match after calibration)
params_dip = load_params(MaglevModel.Dipole);
[zEq_dip, ~, ~, ~] = computeSystemEquilibria(params_dip, MaglevModel.Dipole);

fprintf('Accurate eq: z = %.4f mm\n', zEq_acc(1)*1e3);
fprintf('Dipole eq:   z = %.4f mm\n', zEq_dip(1)*1e3);

% MPC targets the dipole equilibrium (its own model's eq)
xEq_mpc = [0; 0; zEq_dip(1); zeros(7,1)];
uEq = zeros(nu, 1);

% Initial condition: perturbation around Accurate equilibrium
x0_full = [0; 0; zEq_acc(1); zeros(9,1)] + [0; 0.001; 0.001; 0; 0; 0; zeros(6,1)];
x0 = x0_full([1:5,7:11]);

%% --- MPC parameters ---
dt_mpc    = 0.001;
N_horizon = 40;
Tf        = dt_mpc * N_horizon;

%% --- Build MPC model (Dipole) ---
fprintf('\n--- Building MPC model (Dipole) ---\n');
model_mpc = get_maggy_model(MaglevModel.Dipole);

%% --- Build Plant model (Accurate, with LUTs for speed) ---
fprintf('\n--- Building Plant model (Accurate + LUT) ---\n');
model_plant = get_maggy_model(MaglevModel.Accurate, use_luts=true);

%% --- OCP SETUP (Dipole controller) ---
fprintf('\n--- Setting up OCP (Dipole) ---\n');

ocp = AcadosOcp();
ocp.model = model_mpc;

ocp.solver_options.N_horizon             = N_horizon;
ocp.solver_options.tf                    = Tf;
ocp.solver_options.integrator_type       = 'ERK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 1;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.nlp_solver_max_iter = 200;
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.globalization = 'MERIT_BACKTRACKING';
ocp.solver_options.ext_fun_compile_flags = '-O2';

% --- Cost: LINEAR_LS ---
Q = diag([...
    1e4, 1e4, ...       % x, y
    1e6, ...             % z
    1e3, 1e3, ...       % roll, pitch
    1e2, 1e2, 1e3, ...  % vx, vy, vz
    1e2, 1e2 ...        % wx, wy
]);
R = eye(nu) * 1e0;

ocp.cost.cost_type   = 'LINEAR_LS';
ocp.cost.cost_type_0 = 'LINEAR_LS';
ocp.cost.cost_type_e = 'LINEAR_LS';

ocp.cost.Vx   = [eye(nx); zeros(nu, nx)];
ocp.cost.Vu   = [zeros(nx, nu); eye(nu)];
ocp.cost.Vx_0 = ocp.cost.Vx;
ocp.cost.Vu_0 = ocp.cost.Vu;
ocp.cost.Vx_e = eye(nx);

ocp.cost.W   = blkdiag(Q, R);
ocp.cost.W_0 = blkdiag(Q, R);
ocp.cost.W_e = 10 * Q;

ocp.cost.yref   = [xEq_mpc; uEq];
ocp.cost.yref_0 = [xEq_mpc; uEq];
ocp.cost.yref_e = xEq_mpc;

% Input constraints
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu   = -umax * ones(nu, 1);
ocp.constraints.ubu   =  umax * ones(nu, 1);
ocp.constraints.x0    = x0;

ocp_solver = AcadosOcpSolver(ocp);

% Warm-start
for k = 0:N_horizon
    ocp_solver.set('x', xEq_mpc, k);
end
for k = 0:N_horizon-1
    ocp_solver.set('u', uEq, k);
end

%% --- SIM SOLVER (Accurate plant) ---
fprintf('\n--- Setting up sim solver (Accurate plant) ---\n');

sim = AcadosSim();
sim.model = model_plant;

sim.solver_options.Tsim            = dt;
sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages      = 4;
sim.solver_options.num_steps       = 1;

sim_solver = AcadosSimSolver(sim);

%% --- Simulation loop ---
n_sub  = round(dt_mpc / dt);
N_sim  = numel(t);
N_mpc  = floor(N_sim / n_sub);

x_sim = zeros(nx, N_sim);
u_sim = zeros(nu, N_sim);
t_mpc_log  = zeros(1, N_mpc);
t_sub_log  = zeros(1, N_mpc);
t_step_log = zeros(1, N_mpc);
t_lin_log  = zeros(1, N_mpc);
t_qp_log   = zeros(1, N_mpc);

x = x0;
u = uEq;

fprintf('\n--- Running cross-model NMPC (Dipole ctrl -> Accurate plant) ---\n');
fprintf('  T=%.3fs, dt=%.1f us, dt_mpc=%.1f us, n_sub=%d\n', ...
    t(end), dt*1e6, dt_mpc*1e6, n_sub);
fprintf('  Plant steps: %d, MPC calls: %d\n', N_sim, N_mpc);

diverged = false;

for k = 1:N_mpc
    tic_step = tic;

    % --- MPC solve (Dipole model) ---
    ocp_solver.set('constr_x0', x);
    ocp_solver.solve();
    t_mpc_log(k) = ocp_solver.get('time_tot');
    t_lin_log(k) = ocp_solver.get('time_lin');
    t_qp_log(k)  = ocp_solver.get('time_qp_sol');

    status = ocp_solver.get('status');
    if status ~= 0 && status ~= 2
        fprintf('  *** OCP solver warning at MPC step %d (status %d) ***\n', k, status);
    end

    u = ocp_solver.get('u', 0);

    % Warm-shift
    for i = 0:N_horizon-2
        ocp_solver.set('x', ocp_solver.get('x', i+1), i);
        ocp_solver.set('u', ocp_solver.get('u', i+1), i);
    end
    ocp_solver.set('x', ocp_solver.get('x', N_horizon), N_horizon);
    ocp_solver.set('u', ocp_solver.get('u', N_horizon-1), N_horizon-1);

    % --- Sub-step: simulate plant (Accurate model) ---
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
    diverged = abs(x(3)) > 0.5       || ...
               max(abs(x(4:5))) > pi  || ...
               any(isnan(x))       || ...
               any(isinf(x));

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  mpc=%.0f us  sim=%.0f us  total=%.0f us\n', ...
        k, x(3)*1e3, norm(u), ...
        t_mpc_log(k)*1e6, t_sub_log(k)*1e6, t_step_log(k)*1e6);

    if diverged
        fprintf('  *** DIVERGED at MPC step %d ***\n', k);
        last_idx = min(k*n_sub, N_sim);
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
fprintf('\n--- Cross-model NMPC Performance ---\n');
fprintf('Controller: Dipole model\n');
fprintf('Plant:      Accurate model\n');
fprintf('MPC  solve: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_mpc_log)*1e6, max(t_mpc_log)*1e6, median(t_mpc_log)*1e6);
fprintf('  linearize: mean=%.0f us, median=%.0f us\n', ...
    mean(t_lin_log)*1e6, median(t_lin_log)*1e6);
fprintf('  QP solve:  mean=%.0f us, median=%.0f us\n', ...
    mean(t_qp_log)*1e6, median(t_qp_log)*1e6);
fprintf('Plant sim:  mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_sub_log)*1e6, max(t_sub_log)*1e6, median(t_sub_log)*1e6);
fprintf('Total step: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_step_log)*1e6, max(t_step_log)*1e6, median(t_step_log)*1e6);

% Measure settling — distance to MPC target eq
xEq_true = [0; 0; zEq_acc(1); zeros(7,1)];  % true plant equilibrium
fprintf('\nFinal state: z=%.4f mm\n', x(3)*1e3);
fprintf('Distance to Accurate eq: |pos|=%.4f mm, |ang|=%.4f deg\n', ...
    norm(x(1:3)-xEq_true(1:3))*1e3, norm(x(4:5)-xEq_true(4:5))*180/pi);
fprintf('Distance to Dipole eq:   |pos|=%.4f mm, |ang|=%.4f deg\n', ...
    norm(x(1:3)-xEq_mpc(1:3))*1e3, norm(x(4:5)-xEq_mpc(4:5))*180/pi);

if ~diverged
    fprintf('\nSUCCESS: Dipole controller stabilized the Accurate plant.\n');
else
    fprintf('\nFAILED: System diverged.\n');
end

%% --- SAVE ---
sim_data        = struct();
sim_data.t      = t;
sim_data.x      = x_sim;
sim_data.u      = u_sim;
sim_data.xEq    = xEq_mpc;
sim_data.xEq_true = xEq_true;
sim_data.uEq    = uEq;
sim_data.dt     = dt;
sim_data.t_mpc  = t_mpc_log;
sim_data.t_sim  = t_sub_log;
sim_data.t_step = t_step_log;

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
