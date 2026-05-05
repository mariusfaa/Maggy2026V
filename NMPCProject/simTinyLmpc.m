%% simTinyLmpc â€” Static Linear MPC control loop with TinyMPC
%
% Same problem as simAcadosLmpc, but the QP is solved with the TinyMPC
% MATLAB wrapper (ADMM-based) instead of acados HPIPM. The plant is
% still simulated with the acados sim_solver so the comparison is only
% in the OCP/QP solver.
%
% Discrete model (absolute coordinates):
%   x[k+1] = Ad*x[k] + Bd*u[k] + c      c = (I-Ad)*xEq  (uEq=0)

% simSetup;
import casadi.*

%% --- Linearize at equilibrium ---
fprintf('\n--- Computing linearization at equilibrium ---\n');

x_cas  = plant_model.x;
u_cas  = plant_model.u;
f_expl = plant_model.f_expl_expr;

jac_fun = Function('jac_fun', {x_cas, u_cas}, ...
                   {jacobian(f_expl, x_cas), jacobian(f_expl, u_cas)});

[Ac_val, Bc_val] = jac_fun(xEq, uEq);
Ac = full(Ac_val);
Bc = full(Bc_val);

fprintf('  Continuous-time eigenvalues (real part): %s\n', ...
    mat2str(sort(real(eig(Ac)))', 4));

% Discretize via matrix exponential (ZOH)
M_exp = expm([Ac Bc; zeros(nu, nx+nu)] * dt_mpc);
Ad = M_exp(1:nx, 1:nx);
Bd = M_exp(1:nx, nx+1:end);

% Affine offset so xEq is a fixed point: x[k+1] = Ad*x[k] + Bd*u[k] + c
c_offset = (eye(nx) - Ad) * xEq;   % uEq = 0

fprintf('  Discrete-time eigenvalues (|z|): %s\n', ...
    mat2str(sort(abs(eig(Ad)))', 4));

%% --- OCP SETUP (TinyMPC) ---
fprintf('\n--- Setting up TinyMPC OCP ---\n');

cost        = getCost(xEq, uEq, dt_mpc);
constraints = getConstraints(x0);

W = cost.W;
Q = W(1:nx, 1:nx);
R = W(nx+1:end, nx+1:end);

u_min = constraints.lbu(:);
u_max = constraints.ubu(:);
if isprop(constraints, 'lbx') && ~isempty(constraints.lbx)
    x_min = constraints.lbx(:);
    x_max = constraints.ubx(:);
else
    x_min = [];
    x_max = [];
end

% Affine offset c_offset is passed as fdyn so dynamics seen by TinyMPC
% are x[k+1] = Ad*x[k] + Bd*u[k] + c_offset.
solver = TinyMPC();
solver.setup(Ad, Bd, Q, R, N_horizon, ...
    'rho', 1.0, ...
    'fdyn', c_offset, ...
    'verbose', false, ...
    'max_iter', 500, ...
    'abs_pri_tol', 1e-4, ...
    'abs_dua_tol', 1e-4);

solver.set_bound_constraints(x_min, x_max, u_min, u_max);

solver.set_x_ref(repmat(xEq, 1, N_horizon));
solver.set_u_ref(repmat(uEq, 1, N_horizon-1));

%% --- Simulation loop ---
n_sub  = round(dt_mpc / dt);
N_sim  = numel(t);
N_mpc  = floor(N_sim / n_sub);

x_sim = zeros(nx, N_sim);
u_sim = zeros(nu, N_sim);

ocp_time_tot      = zeros(1, N_mpc);
ocp_time_lin      = zeros(1, N_mpc);
ocp_time_reg      = zeros(1, N_mpc);
ocp_time_sim      = zeros(1, N_mpc);
ocp_time_glob     = zeros(1, N_mpc);
ocp_time_qp       = zeros(1, N_mpc);
ocp_time_qp_xcond = zeros(1, N_mpc);
ocp_qp_iter       = zeros(1, N_mpc);
ocp_sqp_iter      = zeros(1, N_mpc);
ocp_nlp_iter      = zeros(1, N_mpc);
ocp_residuals     = zeros(4, N_mpc);
ocp_status        = zeros(1, N_mpc);
ocp_cost          = zeros(1, N_mpc);

sim_time_tot      = zeros(1, N_mpc);
step_time_tot     = zeros(1, N_mpc);

x = x0;
u = uEq;

fprintf('\n--- Running TinyLMPC simulation ---\n');
fprintf('  T=%.3fs, dt=%.1f us, dt_mpc=%.1f us, n_sub=%d\n', ...
    t(end), dt*1e6, dt_mpc*1e6, n_sub);
fprintf('  Plant steps: %d, MPC calls: %d\n', N_sim, N_mpc);

diverged = false;

for k = 1:N_mpc
    solver.set_x0(x);

    t_solve = tic;
    status = solver.solve();
    ocp_time_tot(k) = toc(t_solve);
    ocp_time_qp(k)  = ocp_time_tot(k);   % TinyMPC IS the QP solver
    ocp_status(k)   = status;

    solution = solver.get_solution();

    if status ~= 0
        fprintf('  *** TinyMPC warning at MPC step %d (status %d) ***\n', k, status);
    end

    u = solution.controls(:, 1);

    tacc = 0;
    for j = 1:n_sub
        idx = (k-1)*n_sub + j;
        if idx > N_sim, break; end

        x_sim(:, idx) = x;
        u_sim(:, idx) = u;

        x = sim_solver.simulate(x, u);
        tacc = tacc + sim_solver.get('time_tot');
    end
    sim_time_tot(k)  = tacc;
    step_time_tot(k) = sim_time_tot(k) + ocp_time_tot(k);

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  mpc=%.0f us (qp=%.0f)  sim=%.0f us\n', ...
        k, x(3)*1e3, norm(u), ...
        ocp_time_tot(k)*1e6, ocp_time_qp(k)*1e6, ...
        sim_time_tot(k)*1e6);

    if isDiverged(x)
        diverged = true;
        fprintf('  *** DIVERGED at MPC step %d ***\n', k);
        last_idx = min(k*n_sub, N_sim);
        x_sim = x_sim(:, 1:last_idx);
        u_sim = u_sim(:, 1:last_idx);
        t = t(1:last_idx);
        ocp_time_tot      = ocp_time_tot(1:k);
        ocp_time_lin      = ocp_time_lin(1:k);
        ocp_time_reg      = ocp_time_reg(1:k);
        ocp_time_sim      = ocp_time_sim(1:k);
        ocp_time_glob     = ocp_time_glob(1:k);
        ocp_time_qp       = ocp_time_qp(1:k);
        ocp_time_qp_xcond = ocp_time_qp_xcond(1:k);
        ocp_qp_iter       = ocp_qp_iter(1:k);
        ocp_sqp_iter      = ocp_sqp_iter(1:k);
        ocp_nlp_iter      = ocp_nlp_iter(1:k);
        ocp_residuals     = ocp_residuals(:,1:k);
        ocp_status        = ocp_status(1:k);
        ocp_cost          = ocp_cost(1:k);
        sim_time_tot      = sim_time_tot(1:k);
        step_time_tot     = step_time_tot(1:k);
        break;
    end
end

if ~diverged
    filled = N_mpc * n_sub;
    x_sim  = x_sim(:, 1:filled);
    u_sim  = u_sim(:, 1:filled);
    t      = t(1:filled);
end

%% --- SAVE ---
sim_data             = struct();
sim_data.controller  = 'tinylmpc';
sim_data.t           = t;
sim_data.x           = x_sim;
sim_data.u           = u_sim;
sim_data.x0          = x0;
sim_data.xEq         = xEq;
sim_data.uEq         = uEq;
sim_data.dt          = dt;
sim_data.dt_mpc      = dt_mpc;
sim_data.N_horizon   = N_horizon;
sim_data.diverged    = diverged;
sim_data.ocp_time_tot      = ocp_time_tot;
sim_data.ocp_time_lin      = ocp_time_lin;
sim_data.ocp_time_reg      = ocp_time_reg;
sim_data.ocp_time_sim      = ocp_time_sim;
sim_data.ocp_time_glob     = ocp_time_glob;
sim_data.ocp_time_qp       = ocp_time_qp;
sim_data.ocp_time_qp_xcond = ocp_time_qp_xcond;
sim_data.ocp_qp_iter       = ocp_qp_iter;
sim_data.ocp_sqp_iter      = ocp_sqp_iter;
sim_data.ocp_nlp_iter      = ocp_nlp_iter;
sim_data.ocp_residuals     = ocp_residuals;
sim_data.ocp_status        = ocp_status;
sim_data.sim_time_tot      = sim_time_tot;
sim_data.step_time_tot     = step_time_tot;
sim_data.ocp_cost          = ocp_cost;
sim_data.cost              = computeCost(x_sim - xEq, u_sim - uEq);
sim_data.cost_cum          = cumsum(sim_data.cost);
sim_data.Ad                = Ad;
sim_data.Bd                = Bd;

save_filename = fullfile(out_folder, getFilename('tinylmpc', N_horizon, dt_mpc));
save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
