%% simAcadosLqr — LQR control loop with acados plant simulator
%
% Linearizes the dynamics once at equilibrium using CasADi, discretizes
% via matrix exponential (ZOH), computes the LQR gain from the DARE,
% and applies u = -K*(x - xEq) + uEq with input saturation.
%
% Uses the same acados sim_solver as all other controllers for a fair
% comparison of control performance vs. computational cost.

% simSetup;
import casadi.*

%% --- Linearize at equilibrium ---
fprintf('\n--- Computing linearization at equilibrium ---\n');

% we use the plant model because this is more accurate to what we would do
% for offline precomputation of the equilibrium anyways
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

fprintf('  Discrete-time eigenvalues (|z|): %s\n', ...
    mat2str(sort(abs(eig(Ad)))', 4));

%% --- Compute LQR gain via DARE ---
fprintf('\n--- Computing LQR gain ---\n');

cost_obj = getCost(xEq, uEq, dt_mpc);
W  = cost_obj.W;
Q  = W(1:nx, 1:nx);
R  = W(nx+1:nx+nu, nx+1:nx+nu);

[K, P, cl_eig] = dlqr(Ad, Bd, Q, R);

fprintf('  LQR gain K: [%s]\n', num2str(K(1,:), '%.3f '));
fprintf('  Closed-loop eigenvalues (|z|): %s\n', ...
    mat2str(sort(abs(cl_eig))', 4));

% Input saturation limits (same as MPC constraints)
umax = 1.0;

save_filename = fullfile(out_folder, getFilename('lqr', N_horizon, dt_mpc));

%% --- Simulation loop ---
n_sub  = round(dt_mpc / dt);
N_sim  = numel(t);
N_mpc  = floor(N_sim / n_sub);

x_sim = zeros(nx, N_sim);
u_sim = zeros(nu, N_sim);

ctrl_time     = zeros(1, N_mpc);
sim_time_tot  = zeros(1, N_mpc);
step_time_tot = zeros(1, N_mpc);

x = x0;
u = uEq;

fprintf('\n--- Running LQR simulation ---\n');
fprintf('  T=%.3fs, dt=%.1f us, dt_mpc=%.1f us, n_sub=%d\n', ...
    t(end), dt*1e6, dt_mpc*1e6, n_sub);
fprintf('  Plant steps: %d, LQR calls: %d\n', N_sim, N_mpc);

diverged = false;

for k = 1:N_mpc
    % --- LQR control law ---
    tic_ctrl = tic;
    u = -K * (x - xEq) + uEq;
    u = max(-umax, min(umax, u));  % saturate
    ctrl_time(k) = toc(tic_ctrl);

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
    step_time_tot(k) = sim_time_tot(k) + ctrl_time(k);

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  ctrl=%.0f us  sim=%.0f us\n', ...
        k, x(3)*1e3, norm(u), ...
        ctrl_time(k)*1e6, sim_time_tot(k)*1e6);

    if isDiverged(x)
        diverged = true;
        fprintf('  *** DIVERGED at LQR step %d ***\n', k);
        last_idx = min(k*n_sub, N_sim);
        x_sim         = x_sim(:, 1:last_idx);
        u_sim         = u_sim(:, 1:last_idx);
        t             = t(1:last_idx);
        ctrl_time     = ctrl_time(1:k);
        sim_time_tot  = sim_time_tot(1:k);
        step_time_tot = step_time_tot(1:k);
        break;
    end
end

% Trim trailing zeros
if ~diverged
    filled = N_mpc * n_sub;
    x_sim  = x_sim(:, 1:filled);
    u_sim  = u_sim(:, 1:filled);
    t      = t(1:filled);
end

%% --- SAVE ---
sim_data             = struct();
sim_data.controller  = 'lqr';
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
sim_data.ctrl_time       = ctrl_time;
sim_data.sim_time_tot    = sim_time_tot;
sim_data.step_time_tot   = step_time_tot;
sim_data.cost            = computeCost(x_sim - xEq, u_sim - uEq);
sim_data.cost_cum        = cumsum(sim_data.cost);
sim_data.Ad              = Ad;
sim_data.Bd              = Bd;
sim_data.K               = K;
sim_data.P               = P;
sim_data.Q               = Q;
sim_data.R               = R;

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
