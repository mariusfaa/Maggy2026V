%% simLmpc — Static Linear MPC (linearized once at equilibrium)
%
% Linearizes the reduced 10-state model analytically at the equilibrium
% using CasADi, discretizes via matrix exponential, then solves a fixed-
% coefficient QP at every step using acados (DISCRETE dynamics).
%
% Discrete model (absolute coordinates):
%   x[k+1] = Ad*x[k] + Bd*u[k] + c      c = (I-Ad)*xEq  (uEq=0)
%
% Plant: get_accurate_sim_model() — same as simAcadosNmpc.

simSetup;
import casadi.*

umax = 1;
nx   = 10;
nu   = 4;

x0  = x0([1:5, 7:11]);
xEq = xEq([1:5, 7:11]);
% uEq is zeros(nu,1) from simSetup

save_filename = 'results_lmpc.mat';

%% --- MPC parameters ---
dt_mpc    = 0.001;
N_horizon = 30;
Tf        = dt_mpc * N_horizon;

%% --- Linearize at equilibrium ---
fprintf('\n--- Computing linearization at equilibrium ---\n');

x_cas = MX.sym('x', nx);
u_cas = MX.sym('u', nu);

params_lin              = load_params(MaglevModel.Accurate);
params_lin.magnet.n     = 16;
params_lin.magnet.n_axial = 1;

f_expl = maglevSystemDynamicsReduced_casadi(x_cas, u_cas, params_lin, MaglevModel.Accurate);
jac_fun = Function('jac_fun', {x_cas, u_cas}, ...
                   {jacobian(f_expl, x_cas), jacobian(f_expl, u_cas)});

[Ac_val, Bc_val] = jac_fun(xEq, uEq);
Ac = full(Ac_val);
Bc = full(Bc_val);

fprintf('  Continuous-time eigenvalues (real part): %s\n', ...
    mat2str(sort(real(eig(Ac)))', 4));

% Discretize via matrix exponential (ZOH)
M  = expm([Ac Bc; zeros(nu, nx+nu)] * dt_mpc);
Ad = M(1:nx, 1:nx);
Bd = M(1:nx, nx+1:end);

% Offset so xEq is a fixed point: x[k+1] = Ad*x[k] + Bd*u[k] + c
c_offset = (eye(nx) - Ad) * xEq;   % uEq = 0

fprintf('  Discrete-time eigenvalues (|z|): %s\n', ...
    mat2str(sort(abs(eig(Ad)))', 4));

%% --- OCP SETUP ---
fprintf('\n--- Setting up OCP ---\n');

if ~exist('ocp_solver', 'var')
    x_sym = MX.sym('x', nx);
    u_sym = MX.sym('u', nu);

    mdl = AcadosModel();
    mdl.name          = 'maglev_lmpc';
    mdl.x             = x_sym;
    mdl.u             = u_sym;
    mdl.disc_dyn_expr = Ad * x_sym + Bd * u_sym + c_offset;

    Q = diag([1e4, 1e4, 1e6, 1e3, 1e3, 1e2, 1e2, 1e3, 1e2, 1e2]);
    R = eye(nu) * 1e0;

    ocp = AcadosOcp();
    ocp.model = mdl;

    ocp.solver_options.N_horizon             = N_horizon;
    ocp.solver_options.tf                    = Tf;
    ocp.solver_options.integrator_type       = 'DISCRETE';
    ocp.solver_options.nlp_solver_type       = 'SQP';
    ocp.solver_options.nlp_solver_max_iter   = 1;
    ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    ocp.solver_options.ext_fun_compile_flags = '-O2';

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

    ocp.cost.yref   = [xEq; uEq];
    ocp.cost.yref_0 = [xEq; uEq];
    ocp.cost.yref_e = xEq;

    ocp.constraints.idxbu = 0:nu-1;
    ocp.constraints.lbu   = -umax * ones(nu, 1);
    ocp.constraints.ubu   =  umax * ones(nu, 1);
    ocp.constraints.x0    = x0;

    ocp_solver = AcadosOcpSolver(ocp);
end

% Warm-start at equilibrium
for k = 0:N_horizon,   ocp_solver.set('x', xEq, k); end
for k = 0:N_horizon-1, ocp_solver.set('u', uEq, k); end

%% --- Plant solver ---
fprintf('\n--- Setting up plant solver ---\n');

if ~exist('sim_solver', 'var')
    sim = AcadosSim();
    sim.model = get_accurate_sim_model();
    sim.solver_options.Tsim            = dt;
    sim.solver_options.integrator_type = 'ERK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 1;
    sim_solver = AcadosSimSolver(sim);
end

%% --- Simulation loop ---
n_sub  = round(dt_mpc / dt);
N_sim  = numel(t);
N_mpc  = floor(N_sim / n_sub);

x_sim      = zeros(nx, N_sim);
u_sim      = zeros(nu, N_sim);
t_mpc_log  = zeros(1, N_mpc);
t_qp_log   = zeros(1, N_mpc);
t_sub_log  = zeros(1, N_mpc);
t_step_log = zeros(1, N_mpc);
cost_log   = zeros(1, N_mpc);

x = x0;
u = uEq;

fprintf('\n--- Running static LMPC ---\n');
fprintf('  T=%.3fs, dt=%.1f us, dt_mpc=%.1f us, n_sub=%d\n', ...
    t(end), dt*1e6, dt_mpc*1e6, n_sub);
fprintf('  Plant steps: %d, MPC calls: %d\n', N_sim, N_mpc);

diverged = false;

for k = 1:N_mpc
    tic_step = tic;

    % --- QP solve ---
    ocp_solver.set('constr_x0', x);
    ocp_solver.solve();
    t_mpc_log(k) = ocp_solver.get('time_tot');
    t_qp_log(k)  = ocp_solver.get('time_qp_sol');
    cost_log(k)  = ocp_solver.get_cost();

    status = ocp_solver.get('status');
    if status ~= 0 && status ~= 2
        fprintf('  *** OCP solver warning at step %d (status %d) ***\n', k, status);
    end

    u = ocp_solver.get('u', 0);

    % Warm-shift
    for i = 0:N_horizon-2
        ocp_solver.set('x', ocp_solver.get('x', i+1), i);
        ocp_solver.set('u', ocp_solver.get('u', i+1), i);
    end
    ocp_solver.set('x', ocp_solver.get('x', N_horizon), N_horizon);
    ocp_solver.set('u', ocp_solver.get('u', N_horizon-1), N_horizon-1);

    % --- Plant sub-steps ---
    t_sub_acc = 0;
    for j = 1:n_sub
        idx = (k-1)*n_sub + j;
        if idx > N_sim, break; end
        x_sim(:, idx) = x;
        u_sim(:, idx) = u;
        x = sim_solver.simulate(x, u);
        t_sub_acc = t_sub_acc + sim_solver.get('time_tot');
    end
    t_sub_log(k) = t_sub_acc;
    t_step_log(k) = toc(tic_step);

    diverged = abs(x(3)) > 0.5      || ...
               max(abs(x(4:5))) > pi || ...
               any(isnan(x))         || ...
               any(isinf(x));

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  qp=%.0f us  sim=%.0f us  total=%.0f us\n', ...
        k, x(3)*1e3, norm(u), t_mpc_log(k)*1e6, t_sub_log(k)*1e6, t_step_log(k)*1e6);

    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', k);
        last_idx   = min(k*n_sub, N_sim);
        x_sim      = x_sim(:, 1:last_idx);
        u_sim      = u_sim(:, 1:last_idx);
        t          = t(1:last_idx);
        t_mpc_log  = t_mpc_log(1:k);
        t_sub_log  = t_sub_log(1:k);
        t_step_log = t_step_log(1:k);
        break;
    end
end

%% --- Summary ---
fprintf('\n--- Static LMPC Performance ---\n');
fprintf('OCP total:  mean=%.0f us, median=%.0f us, max=%.0f us\n', ...
    mean(t_mpc_log)*1e6, median(t_mpc_log)*1e6, max(t_mpc_log)*1e6);
fprintf('  QP solve: mean=%.0f us, median=%.0f us\n', ...
    mean(t_qp_log)*1e6, median(t_qp_log)*1e6);
fprintf('Plant sim:  mean=%.0f us, max=%.0f us  (%d sub-steps, acados internal)\n', ...
    mean(t_sub_log)*1e6, max(t_sub_log)*1e6, n_sub);
fprintf('Total step: mean=%.0f us, max=%.0f us  (wall-clock, incl. MATLAB overhead)\n', ...
    mean(t_step_log)*1e6, max(t_step_log)*1e6);
fprintf('Real-time factor: %.2fx (dt_mpc=%.0f us)\n', ...
    dt_mpc/mean(t_step_log), dt_mpc*1e6);
fprintf('\nFinal: z=%.4f mm (eq=%.4f mm), |pos err|=%.4f mm\n', ...
    x(3)*1e3, xEq(3)*1e3, norm(x(1:3)-xEq(1:3))*1e3);

%% --- Save ---
sim_data             = struct();
sim_data.t           = t;
sim_data.x           = x_sim;
sim_data.u           = u_sim;
sim_data.xEq         = xEq;
sim_data.uEq         = uEq;
sim_data.dt          = dt;
sim_data.dt_mpc      = dt_mpc;
sim_data.t_mpc       = t_mpc_log;
sim_data.t_qp        = t_qp_log;
sim_data.t_sim       = t_sub_log;
sim_data.t_step      = t_step_log;
sim_data.Ad          = Ad;
sim_data.Bd          = Bd;

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
