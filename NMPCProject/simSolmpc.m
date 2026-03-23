%% simSolmpc — Successive Online Linearization MPC
%
% At each MPC step, computes the Jacobian at the current state x once,
% discretizes via matrix exponential, and applies the same (Ad, Bd, c)
% to all N shooting nodes. No per-node relinearization.
%
% Discrete model (absolute coordinates, linearized at x_cur):
%   x[k+1] = Ad*x[k] + Bd*u[k] + c      linearized at (x_cur, u_prev)
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

save_filename = 'results_solmpc.mat';

%% --- MPC parameters ---
dt_mpc    = 0.001;
N_horizon = 30;
Tf        = dt_mpc * N_horizon;

%% --- Build CasADi Jacobian function ---
fprintf('\n--- Building Jacobian function ---\n');

x_cas = MX.sym('x', nx);
u_cas = MX.sym('u', nu);

modelId = MaglevModel.Accurate;
params_lin              = load_params(modelId);
params_lin.magnet.n     = 16;
params_lin.magnet.n_axial = 1;

f_expl  = maglevSystemDynamicsReduced_casadi(x_cas, u_cas, params_lin, modelId);
Ac_sym  = jacobian(f_expl, x_cas);
Bc_sym  = jacobian(f_expl, u_cas);

% Single JIT-compiled function: (x,u) -> [Ac(:); Bc(:); f0]
jit_opts   = struct('jit', true, 'jit_options', struct('flags', '-O2'));
lin_casadi = Function('lin_casadi', {x_cas, u_cas}, ...
                      {[Ac_sym(:); Bc_sym(:); f_expl]}, jit_opts);

% Initial linearization at equilibrium for warm-start
raw = full(lin_casadi(xEq, uEq));
Ac  = reshape(raw(1       : nx*nx),       nx, nx);
Bc  = reshape(raw(nx*nx+1 : nx*nx+nx*nu), nx, nu);
f0  = raw(nx*nx+nx*nu+1 : end);
b   = f0 - Ac*xEq - Bc*uEq;
M   = expm([[Ac, Bc, b]; zeros(nu+1, nx+nu+1)] * dt_mpc);
Ad  = M(1:nx, 1:nx);
Bd  = M(1:nx, nx+1:nx+nu);
c   = M(1:nx, end);
p0  = [Ad(:); Bd(:); c(:)];

fprintf('  Discrete eigenvalues at eq (|z|): %s\n', ...
    mat2str(sort(abs(eig(Ad)))', 4));

%% --- OCP SETUP (parameterized: p = [Ad(:); Bd(:); c(:)]) ---
fprintf('\n--- Setting up OCP ---\n');

np = nx*nx + nx*nu + nx;

if ~exist('ocp_solver', 'var')
    x_sym = MX.sym('x', nx);
    u_sym = MX.sym('u', nu);
    p_sym = MX.sym('p', np);

    p_Ad = reshape(p_sym(1          : nx*nx),        nx, nx);
    p_Bd = reshape(p_sym(nx*nx+1    : nx*nx+nx*nu),  nx, nu);
    p_c  =         p_sym(nx*nx+nx*nu+1 : end);

    mdl = AcadosModel();
    mdl.name          = 'maglev_solmpc';
    mdl.x             = x_sym;
    mdl.u             = u_sym;
    mdl.p             = p_sym;
    mdl.disc_dyn_expr = p_Ad * x_sym + p_Bd * u_sym + p_c;

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

    ocp.constraints.idxbu    = 0:nu-1;
    ocp.constraints.lbu      = -umax * ones(nu, 1);
    ocp.constraints.ubu      =  umax * ones(nu, 1);
    ocp.constraints.x0       = x0;

    ocp.parameter_values = p0;

    ocp_solver = AcadosOcpSolver(ocp);
end

% Warm-start at equilibrium
ocp_solver.set('p', p0);
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
t_jac_log  = zeros(1, N_mpc);
t_sub_log  = zeros(1, N_mpc);
t_step_log = zeros(1, N_mpc);

x = x0;
u = uEq;

fprintf('\n--- Running SOL-MPC ---\n');
fprintf('  T=%.3fs, dt=%.1f us, dt_mpc=%.1f us, n_sub=%d\n', ...
    t(end), dt*1e6, dt_mpc*1e6, n_sub);
fprintf('  Plant steps: %d, MPC calls: %d\n', N_sim, N_mpc);

diverged = false;

for k = 1:N_mpc
    tic_step = tic;

    % --- Relinearize at (x, u) ---
    tic_jac = tic;
    raw = full(lin_casadi(x, u));
    Ac  = reshape(raw(1       : nx*nx),       nx, nx);
    Bc  = reshape(raw(nx*nx+1 : nx*nx+nx*nu), nx, nu);
    f0  = raw(nx*nx+nx*nu+1 : end);
    b   = f0 - Ac*x - Bc*u;
    M   = expm([[Ac, Bc, b]; zeros(nu+1, nx+nu+1)] * dt_mpc);
    Ad  = M(1:nx, 1:nx);
    Bd  = M(1:nx, nx+1:nx+nu);
    c   = M(1:nx, end);
    ocp_solver.set('p', [Ad(:); Bd(:); c(:)]);
    t_jac_log(k) = toc(tic_jac);

    % --- QP solve ---
    ocp_solver.set('constr_x0', x);
    ocp_solver.solve();
    t_mpc_log(k) = ocp_solver.get('time_tot');

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
    tic_sub = tic;
    for j = 1:n_sub
        idx = (k-1)*n_sub + j;
        if idx > N_sim, break; end
        x_sim(:, idx) = x;
        u_sim(:, idx) = u;
        x = sim_solver.simulate(x, u);
    end
    t_sub_log(k)  = toc(tic_sub);
    t_step_log(k) = toc(tic_step);

    diverged = abs(x(3)) > 0.5      || ...
               max(abs(x(4:5))) > pi || ...
               any(isnan(x))         || ...
               any(isinf(x));

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  jac=%.0f us  qp=%.0f us  sim=%.0f us\n', ...
        k, x(3)*1e3, norm(u), t_jac_log(k)*1e6, t_mpc_log(k)*1e6, t_sub_log(k)*1e6);

    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', k);
        last_idx   = min(k*n_sub, N_sim);
        x_sim      = x_sim(:, 1:last_idx);
        u_sim      = u_sim(:, 1:last_idx);
        t          = t(1:last_idx);
        t_mpc_log  = t_mpc_log(1:k);
        t_jac_log  = t_jac_log(1:k);
        t_sub_log  = t_sub_log(1:k);
        t_step_log = t_step_log(1:k);
        break;
    end
end

%% --- Summary ---
t_total_log = t_mpc_log + t_jac_log;
fprintf('\n--- SOL-MPC Performance ---\n');
fprintf('Jacobian:   mean=%.0f us, median=%.0f us\n', mean(t_jac_log)*1e6, median(t_jac_log)*1e6);
fprintf('QP solve:   mean=%.0f us, median=%.0f us\n', mean(t_mpc_log)*1e6, median(t_mpc_log)*1e6);
fprintf('Total ctrl: mean=%.0f us, median=%.0f us, max=%.0f us\n', ...
    mean(t_total_log)*1e6, median(t_total_log)*1e6, max(t_total_log)*1e6);
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
sim_data.t_jac       = t_jac_log;
sim_data.t_sim       = t_sub_log;
sim_data.t_step      = t_step_log;
sim_data.Ad_eq       = reshape(p0(1:nx*nx), nx, nx);
sim_data.Bd_eq       = reshape(p0(nx*nx+1:nx*nx+nx*nu), nx, nu);

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
