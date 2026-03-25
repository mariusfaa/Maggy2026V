%% simAcadosNmpc — NMPC control loop with acados

simSetup;
import casadi.*

%% --- OCP SETUP ---
fprintf('\n--- Setting up OCP ---\n');

ocp = AcadosOcp();
ocp.model = getSimModel();

ocp.solver_options.N_horizon             = N_horizon;
ocp.solver_options.tf                    = dt_mpc * N_horizon;
ocp.solver_options.integrator_type       = 'ERK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 1;
ocp.solver_options.nlp_solver_type = 'SQP_RTI'; % 'SQP_RTI' for real-time
ocp.solver_options.nlp_solver_max_iter = 1000;
%ocp.solver_options.nlp_solver_warm_start_first_qp = true;
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'; % FULL_CONDENSING_DAQP, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM
%ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.globalization = 'MERIT_BACKTRACKING';
ocp.solver_options.ext_fun_compile_flags = '-O2';

ocp.cost = getCost(xEq,uEq);
ocp.constraints = getConstraints(x0);

opts = struct();
opts.generate = false;
opts.check_reuse_possible = true;
opts.compile_mex_wrapper = [];
ocp_solver = AcadosOcpSolver(ocp,opts);

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
t_mpc_sol  = zeros(1, N_mpc);
t_sub_log  = zeros(1, N_mpc);
t_step_log = zeros(1, N_mpc);
t_lin_log  = zeros(1, N_mpc);
t_qp_log   = zeros(1, N_mpc);
t_reg_log  = zeros(1, N_mpc);
cost_log   = zeros(1, N_mpc);

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
    t_mpc_sol(k) = ocp_solver.get('time_tot');
    t_lin_log(k) = ocp_solver.get('time_lin');
    t_qp_log(k)  = ocp_solver.get('time_qp_sol');
    t_reg_log(k) = ocp_solver.get('time_reg');
    cost_log(k)  = ocp_solver.get_cost();

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

    % --- Sub-step: simulate plant ---
    t_sub_acc = 0;
    for j = 1:n_sub
        idx = (k-1)*n_sub + j;
        if idx > N_sim, break; end

        x_sim(:, idx) = x;
        u_sim(:, idx) = u;

        x = sim_solver.simulate(x, u);
        t_sub_acc = t_sub_acc + sim_solver.get('time_tot');
    end

    % Divergence check
    diverged = abs(x(3)) > 0.5       || ...
               max(abs(x(4:5))) > pi  || ...
               any(isnan(x))       || ...
               any(isinf(x));

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  mpc=%.0f us (lin=%.0f qp=%.0f)  sim=%.0f us  total=%.0f us\n', ...
        k, x(3)*1e3, norm(u), ...
        t_mpc_sol(k)*1e6, t_lin_log(k)*1e6, t_qp_log(k)*1e6, ...
        t_sub_log(k)*1e6, t_step_log(k)*1e6);

    if diverged
        fprintf('  *** DIVERGED at MPC step %d ***\n', k);
        last_idx = min(k*n_sub, N_sim);
        x_sim = x_sim(:, 1:last_idx);
        u_sim = u_sim(:, 1:last_idx);
        t = t(1:last_idx);
        t_mpc_sol  = t_mpc_sol(1:k);
        t_sub_log  = t_sub_log(1:k);
        t_step_log = t_step_log(1:k);
        break;
    end
end

%% --- Performance summary ---
fprintf('\n--- NMPC Performance ---\n');
fprintf('MPC  solve: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_mpc_sol)*1e6, max(t_mpc_sol)*1e6, median(t_mpc_sol)*1e6);
fprintf('  linearize: mean=%.0f us, median=%.0f us\n', ...
    mean(t_lin_log)*1e6, median(t_lin_log)*1e6);
fprintf('  QP solve:  mean=%.0f us, median=%.0f us\n', ...
    mean(t_qp_log)*1e6, median(t_qp_log)*1e6);
fprintf('  regularize:mean=%.0f us, median=%.0f us\n', ...
    mean(t_reg_log)*1e6, median(t_reg_log)*1e6);
fprintf('Plant sim:  mean=%.0f us, max=%.0f us, median=%.0f us  (%d sub-steps, acados internal)\n', ...
    mean(t_sub_log)*1e6, max(t_sub_log)*1e6, median(t_sub_log)*1e6, n_sub);
fprintf('Total step: mean=%.0f us, max=%.0f us, median=%.0f us  (wall-clock, incl. MATLAB overhead)\n', ...
    mean(t_step_log)*1e6, max(t_step_log)*1e6, median(t_step_log)*1e6);
fprintf('Real-time factor: %.2fx (dt_mpc=%.0f us, avg step=%.0f us)\n', ...
    dt_mpc / mean(t_step_log), dt_mpc*1e6, mean(t_step_log)*1e6);
cost_cum_log = cumsum(cost_log);
fprintf('Cost: final=%.4g, cumulative=%.4g\n', cost_log(end), cost_cum_log(end));
fprintf('\nFinal state: z=%.4f mm (eq=%.4f mm)\n', x(3)*1e3, xEq(3)*1e3);
fprintf('Final |pos error|=%.4f mm, |ang error|=%.4f deg\n', ...
    norm(x(1:3)-xEq(1:3))*1e3, norm(x(4:5)-xEq(4:5))*180/pi);

%% --- SAVE ---
sim_data        = struct();
sim_data.t      = t;
sim_data.x      = x_sim;
sim_data.u      = u_sim;
sim_data.xEq    = xEq;
sim_data.uEq    = uEq;
sim_data.dt     = dt;
sim_data.t_mpc     = t_mpc_sol;
sim_data.t_sim     = t_sub_log;
sim_data.t_step    = t_step_log;
sim_data.cost      = cost_log;
sim_data.cost_cum  = cost_cum_log;

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
