%% simAcadosNmpc — NMPC control loop with acados

simSetup;
import casadi.*

save_filename = "test.mat";

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

ocp_solver = AcadosOcpSolver(ocp);

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

ocp_time_tot  = zeros(1, N_mpc);
ocp_time_lin  = zeros(1, N_mpc);
ocp_time_qp_sol   = zeros(1, N_mpc);
ocp_time_reg  = zeros(1, N_mpc);
ocp_cost   = zeros(1, N_mpc);

sim_time_tot = zeros(1, N_mpc);

tot_time = zeros(1,N_mpc);

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
    ocp_time_tot(k) = ocp_solver.get('time_tot');
    ocp_time_lin(k) = ocp_solver.get('time_lin');
    ocp_time_qp_sol(k)  = ocp_solver.get('time_qp_sol');
    ocp_time_reg(k) = ocp_solver.get('time_reg');
    ocp_cost(k)  = ocp_solver.get_cost();

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
    t_substep_acc = 0;
    for j = 1:n_sub
        idx = (k-1)*n_sub + j;
        if idx > N_sim, break; end

        x_sim(:, idx) = x;
        u_sim(:, idx) = u;

        x = sim_solver.simulate(x, u);
        t_substep_acc = t_substep_acc + sim_solver.get('time_tot');
    end
    sim_time_tot(k) = t_substep_acc;
    tot_time(k) = sim_time_tot(k) + ocp_time_tot(k);

    % Divergence check
    diverged = abs(x(3)) > 0.5       || ...
               max(abs(x(4:5))) > pi  || ...
               any(isnan(x))       || ...
               any(isinf(x));

    fprintf('Step %4d: z=%.4f mm  |u|=%.3f  mpc=%.0f us (lin=%.0f qp=%.0f)  sim=%.0f us\n', ...
        k, x(3)*1e3, norm(u), ...
        ocp_time_tot(k)*1e6, ocp_time_lin(k)*1e6, ocp_time_qp_sol(k)*1e6, ...
        sim_time_tot(k)*1e6);

    if diverged
        fprintf('  *** DIVERGED at MPC step %d ***\n', k);
        last_idx = min(k*n_sub, N_sim);
        x_sim = x_sim(:, 1:last_idx);
        u_sim = u_sim(:, 1:last_idx);
        t = t(1:last_idx);
        ocp_time_tot  = ocp_time_tot(1:k);
        sim_time_tot  = sim_time_tot(1:k);
        tot_time = tot_time(1:k);
        break;
    end
end

%% --- Performance summary ---
fprintf('\n--- NMPC Performance ---\n');
fprintf('MPC  solve: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(ocp_time_tot)*1e6, max(ocp_time_tot)*1e6, median(ocp_time_tot)*1e6);
fprintf('  linearize: mean=%.0f us, median=%.0f us\n', ...
    mean(ocp_time_lin)*1e6, median(ocp_time_lin)*1e6);
fprintf('  QP solve:  mean=%.0f us, median=%.0f us\n', ...
    mean(ocp_time_qp_sol)*1e6, median(ocp_time_qp_sol)*1e6);
fprintf('  regularize:mean=%.0f us, median=%.0f us\n', ...
    mean(ocp_time_reg)*1e6, median(ocp_time_reg)*1e6);
fprintf('Plant sim:  mean=%.0f us, max=%.0f us, median=%.0f us  (%d sub-steps, acados internal)\n', ...
    mean(sim_time_tot)*1e6, max(sim_time_tot)*1e6, median(sim_time_tot)*1e6, n_sub);
fprintf('Total step: mean=%.0f us, max=%.0f us, median=%.0f us  (wall-clock, incl. MATLAB overhead)\n', ...
    mean(t_step_log)*1e6, max(t_step_log)*1e6, median(t_step_log)*1e6);
fprintf('Real-time factor: %.2fx (dt_mpc=%.0f us, avg step=%.0f us)\n', ...
    dt_mpc / mean(t_step_log), dt_mpc*1e6, mean(t_step_log)*1e6);
cost_cum_log = cumsum(ocp_cost);
fprintf('Cost: final=%.4g, cumulative=%.4g\n', ocp_cost(end), cost_cum_log(end));
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
sim_data.t_mpc     = ocp_time_tot;
sim_data.t_sim     = t_sub_log;
sim_data.t_step    = t_step_log;
sim_data.cost      = ocp_cost;
sim_data.cost_cum  = cost_cum_log;

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
