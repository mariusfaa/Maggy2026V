%% simAcadosNmpc — NMPC control loop with acados

% simSetup;
import casadi.*

%% --- OCP SETUP ---
fprintf('\n--- Setting up OCP ---\n');

ocp = AcadosOcp();
ocp.model = ctrl_model;

ocp.solver_options.N_horizon             = N_horizon;
ocp.solver_options.tf                    = dt_mpc * N_horizon;
% ocp.solver_options.nlp_solver_type = 'SQP_RTI';
% ocp.solver_options.globalization = 'MERIT_BACKTRACKING';
ocp.solver_options.ext_fun_compile_flags = '-O2';
ocp.solver_options.integrator_type       = 'IRK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 1;
ocp.solver_options.nlp_solver_type       = 'SQP_RTI';
ocp.solver_options.nlp_solver_tol_stat   = 1e-4;
ocp.solver_options.nlp_solver_tol_eq     = 1e-4;
ocp.solver_options.nlp_solver_tol_ineq   = 1e-4;
ocp.solver_options.nlp_solver_tol_comp   = 1e-4;
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
% ocp.solver_options.qp_solver_iter_max    = 200;
% ocp.solver_options.qp_solver_warm_start  = 1;
% ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';
% ocp.solver_options.regularize_method     = 'CONVEXIFY';


ocp.cost        = getCost(xEq, uEq,dt_mpc);
ocp.constraints = getConstraints(x0);

save_filename = fullfile(out_folder, getFilename('nmpc', N_horizon, dt_mpc));

solver_dir = fullfile('build', 'nmpc');
ocp.code_gen_opts.code_export_directory = fullfile(solver_dir, 'c_generated_code');
ocp.code_gen_opts.json_file = fullfile(solver_dir, [ocp.model.name '_ocp.json']);
ocp_solver = AcadosOcpSolver(ocp, struct('output_dir', solver_dir));

% Warm-start: initialize all shooting nodes to equilibrium
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

sim_time_tot      = zeros(1, N_mpc);
step_time_tot     = zeros(1, N_mpc);

x = x0;
u = uEq;

fprintf('\n--- Running NMPC simulation ---\n');
fprintf('  T=%.3fs, dt=%.1f us, dt_mpc=%.1f us, n_sub=%d\n', ...
    t(end), dt*1e6, dt_mpc*1e6, n_sub);
fprintf('  Plant steps: %d, MPC calls: %d\n', N_sim, N_mpc);

diverged = false;

for k = 1:N_mpc
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
    step_time_tot(k) = sim_time_tot(k) + ocp_time_tot(k);

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  mpc=%.0f us (lin=%.0f qp=%.0f)  sim=%.0f us\n', ...
        k, x(3)*1e3, norm(u), ...
        ocp_time_tot(k)*1e6, ocp_time_lin(k)*1e6, ocp_time_qp(k)*1e6, ...
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

% Trim trailing zeros: N_mpc*n_sub may be < N_sim when N_sim is not divisible by n_sub
if ~diverged
    filled = N_mpc * n_sub;
    x_sim  = x_sim(:, 1:filled);
    u_sim  = u_sim(:, 1:filled);
    t      = t(1:filled);
end

%% --- SAVE ---
sim_data             = struct();
sim_data.controller  = 'nmpc';
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

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
