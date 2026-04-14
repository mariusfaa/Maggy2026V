%% simSolnmpc — Successive Online Linearization NMPC
%
% At each MPC step, linearizes the dynamics at (x, u), discretizes via
% Tustin (bilinear) transform, and passes (Ad, Bd, c) as parameters to
% the OCP.  No per-node relinearization — single linearization per MPC step.
%
% Discrete model (linearized at x_cur):
%   x[k+1] = Ad*x[k] + Bd*u[k] + c

% simSetup;
import casadi.*

%% --- Build dynamics evaluation function ---
fprintf('\n--- Building dynamics function ---\n');

x_cas  = ctrl_model.x;
u_cas  = ctrl_model.u;
f_expl = ctrl_model.f_expl_expr;

% Symbolic Jacobians via CasADi AD
Ac_sym = jacobian(f_expl, x_cas);
Bc_sym = jacobian(f_expl, u_cas);

% Legacy CasADi-only linearization (returns Ac(:), Bc(:), f0 — still needs MATLAB expm)
lin_casadi = Function('lin_casadi', {x_cas, u_cas}, ...
                      {[Ac_sym(:); Bc_sym(:); f_expl]});

% For hybrid FD mode: batched evaluation (1 CasADi call for all perturbations)
f_fun = Function('f_fun', {x_cas, u_cas}, {f_expl});
n_fd  = 1 + 5*2 + nu*2;  % nominal + 5 state pairs + 4 input pairs = 19
f_batch = f_fun.map(n_fd);

%% --- Build compiled discretization function (Tustin / bilinear) ---
%  Single compiled CasADi Function: (x, u) → [Ad(:); Bd(:); c]
%  Performs Jacobian + affine offset + Tustin discretization in one
%  compiled C call, eliminating all MATLAB interpreter overhead.
%
%  Tustin (bilinear transform, order-2, A-stable):
%    Ad = (I - Ac*dt/2) \ (I + Ac*dt/2)
%    Bd = (I - Ac*dt/2) \ (Bc*dt)
%    c  = (I - Ac*dt/2) \ (b*dt)
fprintf('  Building compiled discretize function (Tustin) ...\n');

% Affine offset: f(x,u) ≈ Ac*x + Bc*u + b  →  b = f(x,u) - Ac*x - Bc*u
b_sym = f_expl - Ac_sym * x_cas - Bc_sym * u_cas;

% Tustin discretization
I_nx   = MX.eye(nx);
Lhs    = I_nx - Ac_sym * (dt_mpc / 2);   % (I - Ac*dt/2)
Ad_sym = solve(Lhs, I_nx + Ac_sym * (dt_mpc / 2));
Bd_sym = solve(Lhs, Bc_sym * dt_mpc);
c_sym  = solve(Lhs, b_sym  * dt_mpc);

discretize_fn = Function('discretize', {x_cas, u_cas}, ...
    {vertcat(Ad_sym(:), Bd_sym(:), c_sym)});

% Linearization method:
%   'compiled' — full CasADi discretize (Jacobian + Tustin, compiled C)  [DEFAULT]
%   'casadi'   — CasADi Jacobians + MATLAB Tustin
%   'fd'       — hybrid finite-difference Jacobians + MATLAB Tustin
lin_method = 'compiled';
fd_delta   = 1e-7;

% Sparsity structure of f_expl (used by 'fd' mode):
%   Rows 1-5:  kinematic (dx/dt = v)  → Ac(1:5,:) = [0 I], Bc(1:5,:) = 0
%   Rows 6-10: forces/torques depend on positions (cols 1-5) and inputs only
%              No velocity dependence (cols 6-10 are structurally zero)
%   → Only need FD for Ac(6:10,1:5) and Bc(6:10,:): 9 perturbations total

%% --- Initial linearization at equilibrium (warm-start) ---
p0 = full(discretize_fn(xEq, uEq));

% Validate compiled Tustin against MATLAB Tustin at equilibrium
raw_ref = full(lin_casadi(xEq, uEq));
Ac_ref  = reshape(raw_ref(1:nx*nx), nx, nx);
Bc_ref  = reshape(raw_ref(nx*nx+1:nx*nx+nx*nu), nx, nu);
f0_ref  = raw_ref(nx*nx+nx*nu+1:end);
b_ref   = f0_ref - Ac_ref*xEq - Bc_ref*uEq;
Lhs_ref = eye(nx) - Ac_ref * (dt_mpc / 2);
Ad_ref  = Lhs_ref \ (eye(nx) + Ac_ref * (dt_mpc / 2));
Bd_ref  = Lhs_ref \ (Bc_ref * dt_mpc);
c_ref   = Lhs_ref \ (b_ref  * dt_mpc);
p0_ref  = [Ad_ref(:); Bd_ref(:); c_ref];
tustin_err = max(abs(p0 - p0_ref));
fprintf('  CasADi vs MATLAB Tustin validation error: %.2e\n', tustin_err);
assert(tustin_err < 1e-8, 'Compiled Tustin disagrees with MATLAB Tustin');

% Extract Ad for eigenvalue display
Ad = reshape(p0(1:nx*nx), nx, nx);

fprintf('  Linearization method: %s\n', lin_method);
fprintf('  Discrete eigenvalues at eq (|z|): %s\n', ...
    mat2str(sort(abs(eig(Ad)))', 4));

%% --- OCP SETUP ---
fprintf('\n--- Setting up OCP ---\n');

np    = nx*nx + nx*nu + nx;
x_sym = MX.sym('x', nx);
u_sym = MX.sym('u', nu);
p_sym = MX.sym('p', np);

p_Ad = reshape(p_sym(1             : nx*nx),       nx, nx);
p_Bd = reshape(p_sym(nx*nx+1       : nx*nx+nx*nu), nx, nu);
p_c  =         p_sym(nx*nx+nx*nu+1 : end);

mdl = AcadosModel();
mdl.name          = 'maglev_solnmpc';
mdl.x             = x_sym;
mdl.u             = u_sym;
mdl.p             = p_sym;
mdl.disc_dyn_expr = p_Ad * x_sym + p_Bd * u_sym + p_c;

ocp = AcadosOcp();
ocp.model = mdl;

ocp.solver_options.N_horizon             = N_horizon;
ocp.solver_options.tf                    = dt_mpc * N_horizon;
ocp.solver_options.integrator_type       = 'DISCRETE';
ocp.solver_options.nlp_solver_type       = 'SQP';
ocp.solver_options.nlp_solver_max_iter   = 1;
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.ext_fun_compile_flags = '-O2';

ocp.cost        = getCost(xEq, uEq,dt_mpc);
ocp.constraints = getConstraints(x0);

ocp.parameter_values = p0;

save_filename = fullfile(out_folder, getFilename('solnmpc', N_horizon, dt_mpc));

solver_dir = fullfile('build', 'solnmpc');
ocp.code_gen_opts.code_export_directory = fullfile(solver_dir, 'c_generated_code');
ocp.code_gen_opts.json_file = fullfile(solver_dir, [mdl.name '_ocp.json']);
ocp_solver = AcadosOcpSolver(ocp, struct('output_dir', solver_dir));

% Warm-start: initialize all shooting nodes to equilibrium
ocp_solver.set('p', p0);
for k = 0:N_horizon
    ocp_solver.set('x', xEq, k);
end
for k = 0:N_horizon-1
    ocp_solver.set('u', uEq, k);
end

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
ocp_residuals     = zeros(4, N_mpc);  % [res_stat; res_eq; res_ineq; res_comp]
ocp_status        = zeros(1, N_mpc);
ocp_cost          = zeros(1, N_mpc);

jac_time      = zeros(1, N_mpc);  % linearize + discretize time
sim_time_tot  = zeros(1, N_mpc);
step_time_tot = zeros(1, N_mpc);

x = x0;
u = uEq;

fprintf('\n--- Running SOL-MPC simulation ---\n');
fprintf('  T=%.3fs, dt=%.1f us, dt_mpc=%.1f us, n_sub=%d\n', ...
    t(end), dt*1e6, dt_mpc*1e6, n_sub);
fprintf('  Plant steps: %d, MPC calls: %d\n', N_sim, N_mpc);

diverged = false;

for k = 1:N_mpc
    % --- Relinearize + discretize at (x, u) ---
    tic_jac = tic;

    if strcmp(lin_method, 'compiled')
        % Full pipeline in compiled CasADi C: Jacobian + expm → (Ad, Bd, c)
        p_new = full(discretize_fn(x, u));
        ocp_solver.set('p', p_new);

    elseif strcmp(lin_method, 'casadi')
        % CasADi Jacobians + MATLAB expm (for comparison / validation)
        raw = full(lin_casadi(x, u));
        Ac  = reshape(raw(1          : nx*nx),        nx, nx);
        Bc  = reshape(raw(nx*nx+1    : nx*nx+nx*nu),  nx, nu);
        f0  = raw(nx*nx+nx*nu+1 : end);
        b   = f0 - Ac*x - Bc*u;
        L   = eye(nx) - Ac * (dt_mpc / 2);
        Ad = L \ (eye(nx) + Ac * (dt_mpc / 2));
        Bd = L \ (Bc * dt_mpc);
        c  = L \ (b  * dt_mpc);
        ocp_solver.set('p', [Ad(:); Bd(:); c(:)]);

    else  % 'fd'
        % Hybrid FD exploiting sparsity — single batched CasADi call
        %   Rows 1-5:  kinematic → Ac(1:5,:) = [0 I], Bc(1:5,:) = 0
        %   Rows 6-10: only depend on positions (cols 1-5) and inputs
        %   → 9 perturbation directions, 19 evals batched into 1 call
        X_batch = repmat(x, 1, n_fd);
        U_batch = repmat(u, 1, n_fd);
        col = 2;
        for i = 1:5
            X_batch(i, col)   = x(i) + fd_delta;
            X_batch(i, col+1) = x(i) - fd_delta;
            col = col + 2;
        end
        for i = 1:nu
            U_batch(i, col)   = u(i) + fd_delta;
            U_batch(i, col+1) = u(i) - fd_delta;
            col = col + 2;
        end

        F_all = full(f_batch(X_batch, U_batch));
        f0 = F_all(:, 1);
        inv2d = 1 / (2*fd_delta);

        Ac = zeros(nx, nx);
        Ac(1:5, 6:10) = eye(5);
        col = 2;
        for i = 1:5
            Ac(6:10, i) = (F_all(6:10, col) - F_all(6:10, col+1)) * inv2d;
            col = col + 2;
        end
        Bc = zeros(nx, nu);
        for i = 1:nu
            Bc(6:10, i) = (F_all(6:10, col) - F_all(6:10, col+1)) * inv2d;
            col = col + 2;
        end

        b = f0 - Ac*x - Bc*u;
        L = eye(nx) - Ac * (dt_mpc / 2);
        Ad = L \ (eye(nx) + Ac * (dt_mpc / 2));
        Bd = L \ (Bc * dt_mpc);
        c  = L \ (b  * dt_mpc);
        ocp_solver.set('p', [Ad(:); Bd(:); c(:)]);
    end

    jac_time(k) = toc(tic_jac);

    % --- MPC solve ---
    ocp_solver.set('constr_x0', x);
    ocp_solver.solve();
    ocp_time_tot(k)      = ocp_solver.get('time_tot');
    ocp_time_lin(k)      = ocp_solver.get('time_lin');
    ocp_time_reg(k)      = ocp_solver.get('time_reg');
    ocp_time_sim(k)      = ocp_solver.get('time_sim');
    ocp_time_glob(k)     = ocp_solver.get('time_glob');
    ocp_time_qp(k)       = ocp_solver.get('time_qp_sol');
    ocp_time_qp_xcond(k) = ocp_solver.get('time_qp_xcond');
    ocp_qp_iter(k)       = ocp_solver.get('qp_iter');
    ocp_sqp_iter(k)      = ocp_solver.get('sqp_iter');
    ocp_nlp_iter(k)      = ocp_solver.get('nlp_iter');
    ocp_residuals(:,k)   = ocp_solver.get('residuals');
    ocp_cost(k)          = ocp_solver.get_cost();

    status = ocp_solver.get('status');
    ocp_status(k) = status;
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

    % --- Sub-step: simulate plant ---
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
    step_time_tot(k) = jac_time(k) + ocp_time_tot(k) + sim_time_tot(k);

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  jac=%.0f us  mpc=%.0f us (qp=%.0f)  sim=%.0f us\n', ...
        k, x(3)*1e3, norm(u), ...
        jac_time(k)*1e6, ocp_time_tot(k)*1e6, ocp_time_qp(k)*1e6, ...
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
        jac_time          = jac_time(1:k);
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
sim_data.controller  = 'solnmpc';
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
sim_data.jac_time          = jac_time;   % linearize + discretize (compiled CasADi C when lin_method='compiled')
sim_data.lin_method        = lin_method;
sim_data.Ad_eq             = reshape(p0(1:nx*nx),             nx, nx);
sim_data.Bd_eq             = reshape(p0(nx*nx+1:nx*nx+nx*nu), nx, nu);

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
