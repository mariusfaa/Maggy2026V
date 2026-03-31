%% acadosNMPC_simulation_reduced.m
% =========================================================================
%  NMPC simulation for the magnetic levitation system using acados.
%  REDUCED-ORDER MODEL: 10 states (yaw angle gamma and yaw rate wz removed)
%
%  The yaw states are uncontrollable/unobservable due to the axial symmetry
%  of the levitating magnet, and do not enter the force/torque calculations
%  when held at zero. Removing them reduces the OCP dimension and saves
%  computation time.
%
%  State vector (10x1):
%    x(1:3)  = [x, y, z]          position
%    x(4:5)  = [alpha, beta]      roll, pitch
%    x(6:8)  = [vx, vy, vz]       linear velocity
%    x(9:10) = [wx, wy]           angular velocity (body frame, x & y)
%
%  This version includes comprehensive data collection for benchmarking
%  NMPC solve-time optimisation experiments (baseline measurement).
%
%  Author: Marius Jullum Faanes
%  Date:   20.03.2026
% =========================================================================

%% --- PROJECT SETUP ---
clearvars -except ocp_solver sim_solver; clc;

acados_root  = '/home/mariujf/acados';
project_root = '/home/mariujf/Maggy2026V/NMPCProject';

setenv('ACADOS_SOURCE_DIR',        acados_root);
setenv('ENV_ACADOS_INSTALL_DIR',   acados_root);
setenv('ACADOS_INSTALL_DIR',       acados_root);

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% --- EXPERIMENT CONFIGURATION ---
% Label this run for comparison across experiments.
% Change experiment_name for each experiment variant (A, B1, B2, ..., J).
experiment_name = 'B1_IRK4-5';
experiment_desc = 'Keep SQP_RTI, halve number of steps in IRK';

fprintf('====================================================================\n');
fprintf('  Experiment: %s\n', experiment_name);
fprintf('  %s\n', experiment_desc);
fprintf('====================================================================\n\n');

%% --- MODEL SETUP (reduced order: 10 states) ---
nx_full = 12;
nx      = 10;   % Reduced: removed gamma (yaw) and wz (yaw rate)
nu      = 4;

% Full-order symbolic variables (used to build dynamics and find equilibrium)
x_full    = SX.sym('x_full', nx_full);
u_sym     = SX.sym('u', nu);

parameters_maggy_V4;
correctionFactorFast       = computeSolenoidRadiusCorrectionFactor(params, 'fast');
paramsFast                 = params;
paramsFast.solenoids.r     = correctionFactorFast * paramsFast.solenoids.r;

%% --- BUILD FULL-ORDER CASADI FUNCTION (for equilibrium search) ---
f_expl_full = maglevSystemDynamicsCasADi(x_full, u_sym, paramsFast);
f_func_full = casadi.Function('f_full', {x_full, u_sym}, {f_expl_full});

%% --- FIND EQUILIBRIUM (using full model with gamma=0, wz=0) ---
fprintf('--- Searching for equilibrium z with u = 0 ---\n');

z_var    = SX.sym('z_eq');
u_zero   = zeros(nu, 1);
x_eq_sym = [0; 0; z_var; zeros(9,1)];
accel    = f_func_full(x_eq_sym, u_zero);
fz_residual = accel(9);   % z-acceleration in full model

nlp       = struct('x', z_var, 'f', fz_residual^2);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 3)));

sol     = solver_eq('x0', 0.030, 'lbx', 0.015, 'ubx', 0.060);
zEq_cas = full(sol.x);
uEq     = zeros(nu, 1);

% Reduced-order equilibrium (10 states)
xEq     = [0; 0; zEq_cas; zeros(7,1)];

fprintf('Equilibrium: z = %.6f m, u = [0, 0, 0, 0]\n', zEq_cas);

%% --- BUILD REDUCED-ORDER DYNAMICS ---
% Reduced state: x_r = [x, y, z, alpha, beta, vx, vy, vz, wx, wy]
x_r    = SX.sym('x_r', nx);
xdot_r = SX.sym('xdot_r', nx);

% Map reduced state to full state (gamma = 0, wz = 0)
x_full_from_r = [x_r(1:5); 0; x_r(6:8); x_r(9:10); 0];

% Evaluate full dynamics
dx_full = f_func_full(x_full_from_r, u_sym);

% Extract reduced derivatives (drop indices 6 and 12)
f_expl_r = [dx_full(1:5); dx_full(7:11)];

%% --- OCP SETUP ---
N      = 20;
Tf     = 0.2;
dt_mpc = Tf / N;   % 0.01 s

% Cost matrices (10 states: x, y, z, roll, pitch, vx, vy, vz, wx, wy)
Q = diag([1e2, 1e2, 1e3, ...    % x, y, z position
          1e3, 1e3, ...          % roll, pitch
          1e1, 1e1, 1e1, ...    % vx, vy, vz
          1e1, 1e1]);            % wx, wy
R = eye(nu) * 1.0;

ocp = AcadosOcp();
ocp.model.name        = 'maglev_nmpc_reduced';
ocp.model.x           = x_r;
ocp.model.u           = u_sym;
ocp.model.xdot        = xdot_r;
ocp.model.f_impl_expr = xdot_r - f_expl_r;

% Solver options
ocp.solver_options.N_horizon             = N;
ocp.solver_options.tf                    = Tf;
ocp.solver_options.integrator_type       = 'IRK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 5;
ocp.solver_options.nlp_solver_type       = 'SQP_RTI';
% ocp.solver_options.nlp_solver_max_iter   = 100;
ocp.solver_options.nlp_solver_tol_stat   = 1e-4;
ocp.solver_options.nlp_solver_tol_eq     = 1e-4;
ocp.solver_options.nlp_solver_tol_ineq   = 1e-4;
ocp.solver_options.nlp_solver_tol_comp   = 1e-4;
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.qp_solver_iter_max    = 200;
ocp.solver_options.qp_solver_warm_start  = 1;
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';
ocp.solver_options.regularize_method     = 'CONVEXIFY';

% Cost
ocp.cost.cost_type   = 'NONLINEAR_LS';
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.cost.W           = blkdiag(Q, R);
ocp.cost.W_0         = blkdiag(Q, R);
ocp.cost.W_e         = Q * 10;

ocp.model.cost_y_expr   = [x_r; u_sym];
ocp.model.cost_y_expr_0 = [x_r; u_sym];
ocp.model.cost_y_expr_e = x_r;

ocp.cost.yref   = [xEq; uEq];
ocp.cost.yref_0 = [xEq; uEq];
ocp.cost.yref_e = xEq;

% Input constraints
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu   = -1 * ones(nu,1);
ocp.constraints.ubu   =  1 * ones(nu,1);

% State constraints (soft) — indices in 10-state vector:
%   0=x, 1=y, 2=z, 3=alpha, 4=beta
n_sbx = 5;
ocp.constraints.idxbx  = [0, 1, 2, 3, 4];
ocp.constraints.lbx    = [-0.025; -0.025; 0.015; -0.35; -0.35];
ocp.constraints.ubx    = [ 0.025;  0.025; 0.055;  0.35;  0.35];
ocp.constraints.idxsbx = 0:4;

ocp.cost.Zl = 1e3 * ones(n_sbx, 1);
ocp.cost.Zu = 1e3 * ones(n_sbx, 1);
ocp.cost.zl = 1e2 * ones(n_sbx, 1);
ocp.cost.zu = 1e2 * ones(n_sbx, 1);

ocp.constraints.x0 = xEq;

% Store constraint bounds for later violation checking
lbx_constr = ocp.constraints.lbx;
ubx_constr = ocp.constraints.ubx;
idx_constr = ocp.constraints.idxbx + 1;  % convert 0-indexed to 1-indexed

% Capture configuration as plain variables before acados modifies the struct
cfg_nlp_solver   = ocp.solver_options.nlp_solver_type;
cfg_integrator   = ocp.solver_options.integrator_type;
cfg_num_stages   = ocp.solver_options.sim_method_num_stages;
cfg_num_steps    = ocp.solver_options.sim_method_num_steps;
cfg_qp_solver    = ocp.solver_options.qp_solver;
cfg_qp_iter_max  = ocp.solver_options.qp_solver_iter_max;
cfg_hessian      = ocp.solver_options.hessian_approx;
cfg_regularize   = ocp.solver_options.regularize_method;
cfg_cost_type    = ocp.cost.cost_type;

%% --- BUILD OCP SOLVER ---
if ~exist('ocp_solver', 'var') || ~isvalid(ocp_solver)
    fprintf('\n--- Building acados OCP solver (reduced, nx=%d) ---\n', nx);
    ocp_solver = AcadosOcpSolver(ocp);
else
    fprintf('\n--- Reusing OCP solver ---\n');
end

%% --- BUILD SIM SOLVER ---
if ~exist('sim_solver', 'var') || ~isvalid(sim_solver)
    fprintf('\n--- Building acados sim solver (reduced, nx=%d) ---\n', nx);
    sim = AcadosSim();
    sim.model.name        = 'maglev_sim_reduced';
    sim.model.x           = x_r;
    sim.model.u           = u_sym;
    sim.model.xdot        = xdot_r;
    sim.model.f_impl_expr = xdot_r - f_expl_r;
    sim.solver_options.Tsim            = dt_mpc;
    sim.solver_options.integrator_type = 'IRK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 10;
    sim_solver = AcadosSimSolver(sim);
else
    fprintf('\n--- Reusing sim solver ---\n');
end

%% --- INITIAL CONDITION ---
% Perturbation direction (excites all reduced DOFs)
%   [x, y, z, alpha, beta, vx, vy, vz, wx, wy]
dx_dir = [0.005; -0.008; 0.010; 0.15; -0.10; ...
          0.01;  -0.01;  0.0;   0.2;  -0.3];

perturbation_scale = 0.05;
x_current = xEq + perturbation_scale * dx_dir;

fprintf('\n--- Initial perturbation (scale = %.2f) ---\n', perturbation_scale);
fprintf('  Position offset:  (%+.2f, %+.2f, %+.2f) mm\n', ...
    (x_current(1)-xEq(1))*1e3, (x_current(2)-xEq(2))*1e3, (x_current(3)-xEq(3))*1e3);
fprintf('  Orientation:      (%+.2f, %+.2f) deg\n', ...
    rad2deg(x_current(4)), rad2deg(x_current(5)));

%% --- TRAJECTORY INITIALISATION ---
% Linear interpolation from perturbed state to equilibrium
for k = 0:N
    alpha_k = k / N;
    ocp_solver.set('x', (1 - alpha_k) * x_current + alpha_k * xEq, k);
end
for k = 0:N-1
    ocp_solver.set('u', uEq, k);
end

%% --- PRE-ALLOCATE DATA COLLECTION ARRAYS ---
sim_steps = 200;

% === State and input trajectories (original) ===
plot_x    = zeros(nx, sim_steps+1);
plot_u    = zeros(nu, sim_steps);
plot_x(:,1) = x_current;

% === Solver status and iteration counts ===
plot_stat    = zeros(1, sim_steps);       % acados status (0=OK, 2=max_iter, 4=QP_fail)
plot_sqp     = zeros(1, sim_steps);       % SQP iterations per step
plot_qp_iter = zeros(1, sim_steps);       % total QP iterations per step (summed over SQP iters)

% === Timing breakdown (seconds) ===
%  time_tot  = total solve wall-clock time
%  time_lin  = linearisation: dynamics eval + Jacobian/Hessian computation
%  time_sim  = integration (subset of time_lin for IRK Newton iterations)
%  time_sim_ad = integrator: external function (CasADi-generated code) eval
%  time_sim_la = integrator: linear algebra (Newton step solves)
%  time_qp_sol = QP solver time (HPIPM)
%  time_reg  = Hessian regularisation (CONVEXIFY)
%  time_glob = globalization (line search / step acceptance)
plot_time_tot    = zeros(1, sim_steps);
plot_time_lin    = zeros(1, sim_steps);
plot_time_sim    = zeros(1, sim_steps);
plot_time_sim_ad = zeros(1, sim_steps);
plot_time_sim_la = zeros(1, sim_steps);
plot_time_qp_sol = zeros(1, sim_steps);
plot_time_reg    = zeros(1, sim_steps);
plot_time_glob   = zeros(1, sim_steps);

% === Per-step tracking error ===
plot_err_norm    = zeros(1, sim_steps+1);  % ||x - xEq||_2 at each timestep
plot_err_pos     = zeros(1, sim_steps+1);  % ||pos - posEq||_2  (mm)
plot_err_ang     = zeros(1, sim_steps+1);  % ||[alpha,beta]||_2 (deg)
plot_err_norm(1) = norm(x_current - xEq);
plot_err_pos(1)  = norm(x_current(1:3) - xEq(1:3)) * 1e3;
plot_err_ang(1)  = rad2deg(norm(x_current(4:5) - xEq(4:5)));

% === Soft constraint violation ===
%  For each step, record the max slack across all stages and all 5 soft
%  constrained states.  Nonzero slack = the state violated its box bound.
plot_max_slack = zeros(1, sim_steps);  % max(sl, su) over all stages

fprintf('\n--- Starting NMPC simulation (%d steps, %.2f s, nx=%d) ---\n', ...
    sim_steps, sim_steps*dt_mpc, nx);

%% --- SIMULATION LOOP ---
for i = 1:sim_steps
    % --- Solve OCP ---
    ocp_solver.set('constr_x0', x_current);
    ocp_solver.solve();

    % === Retrieve solver status and iteration counts ===
    status    = ocp_solver.get('status');
    sqp_iter  = ocp_solver.get('sqp_iter');
    u_applied = ocp_solver.get('u', 0);
    u_applied = max(min(u_applied, 1), -1);

    % === Retrieve full timing breakdown ===
    t_tot    = ocp_solver.get('time_tot');
    t_lin    = ocp_solver.get('time_lin');
    t_qp_sol = ocp_solver.get('time_qp_sol');
    t_reg    = ocp_solver.get('time_reg');

    % These may not be available in all acados versions; wrap in try-catch
    try t_sim    = ocp_solver.get('time_sim');    catch; t_sim    = NaN; end
    try t_sim_ad = ocp_solver.get('time_sim_ad'); catch; t_sim_ad = NaN; end
    try t_sim_la = ocp_solver.get('time_sim_la'); catch; t_sim_la = NaN; end
    try t_glob   = ocp_solver.get('time_glob');   catch; t_glob   = NaN; end

    % === Retrieve QP iteration count ===
    %  qp_iter may return a vector (one count per SQP iteration); sum them.
    try
        qp_it_raw = ocp_solver.get('qp_iter');
        qp_it = sum(qp_it_raw(:));
    catch
        qp_it = NaN;
    end

    % === Retrieve soft constraint slack values ===
    all_slacks = 0;  % collector; will take max at the end
    for k = 0:N
        try
            sl_k = ocp_solver.get('sl', k);
            su_k = ocp_solver.get('su', k);
            all_slacks = [all_slacks; abs(sl_k(:)); abs(su_k(:))]; %#ok<AGROW>
        catch
            % Slacks not available at this stage (e.g., terminal without
            % soft constraints); skip silently.
        end
    end
    max_slack_this_step = max(all_slacks);  % guaranteed scalar

    % --- Simulate plant ---
    sim_solver.set('x', x_current);
    sim_solver.set('u', u_applied);
    sim_solver.solve();
    x_next = sim_solver.get('xn');

    % --- Warm-start shift ---
    x_traj = ocp_solver.get('x');
    u_traj = ocp_solver.get('u');
    x_traj_shift = [x_traj(:, 2:end), xEq];
    u_traj_shift = [u_traj(:, 2:end), uEq];
    for k = 0:N
        ocp_solver.set('x', x_traj_shift(:, k+1), k);
    end
    for k = 0:N-1
        ocp_solver.set('u', u_traj_shift(:, k+1), k);
    end

    % --- Store all data ---
    plot_x(:,i+1)       = x_next;
    plot_u(:,i)         = u_applied;
    plot_stat(i)        = status;
    plot_sqp(i)         = sqp_iter;
    plot_qp_iter(i)     = qp_it;

    plot_time_tot(i)    = t_tot;
    plot_time_lin(i)    = t_lin;
    plot_time_sim(i)    = t_sim;
    plot_time_sim_ad(i) = t_sim_ad;
    plot_time_sim_la(i) = t_sim_la;
    plot_time_qp_sol(i) = t_qp_sol;
    plot_time_reg(i)    = t_reg;
    plot_time_glob(i)   = t_glob;

    plot_err_norm(i+1)  = norm(x_next - xEq);
    plot_err_pos(i+1)   = norm(x_next(1:3) - xEq(1:3)) * 1e3;
    plot_err_ang(i+1)   = rad2deg(norm(x_next(4:5) - xEq(4:5)));

    plot_max_slack(i)   = max_slack_this_step;

    % --- Console output ---
    if i <= 10 || mod(i,20) == 0 || i == sim_steps
        fprintf('Step %3d: st=%d, sqp=%2d, qp=%3d, t=%5.1fms [lin=%.1f sim=%.1f qp=%.1f reg=%.1f], u=[%+.3f %+.3f %+.3f %+.3f], z=%.4fmm, |ang|=%.3fdeg, |w|=%.2fdeg/s\n', ...
            i, status, sqp_iter, qp_it, t_tot*1000, ...
            t_lin*1000, t_sim*1000, t_qp_sol*1000, t_reg*1000, ...
            u_applied(1), u_applied(2), u_applied(3), u_applied(4), ...
            x_next(3)*1e3, rad2deg(norm(x_next(4:5))), rad2deg(norm(x_next(9:10))));
    end

    % --- Divergence check ---
    diverged = abs(x_next(3)) > 0.5 || max(abs(x_next(4:5))) > pi || ...
               any(isnan(x_next)) || any(isinf(x_next));
    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
        break;
    end

    x_current = x_next;
end

%% --- POST-PROCESS: COMPUTE DERIVED METRICS ---
n_actual = min(i, sim_steps);

% Trim arrays to actual length
time_vec      = plot_time_tot(1:n_actual);
sqp_vec       = plot_sqp(1:n_actual);
qp_vec        = plot_qp_iter(1:n_actual);
stat_vec      = plot_stat(1:n_actual);
err_norm_vec  = plot_err_norm(1:n_actual+1);

% --- Settling time ---
%  Defined as the first time the tracking error ||x - xEq|| drops below a
%  threshold and stays below it for the remainder of the simulation.
%  We compute for 1%, 2%, and 5% of the initial error.
initial_err = plot_err_norm(1);
settle_thresholds = [0.01, 0.02, 0.05];  % 1%, 2%, 5%
settling_times    = NaN(size(settle_thresholds));

for s = 1:length(settle_thresholds)
    thr = settle_thresholds(s) * initial_err;
    settled = false;
    for j = 1:n_actual+1
        if all(plot_err_norm(j:n_actual+1) <= thr)
            settling_times(s) = (j-1) * dt_mpc;  % time of first entry into settled band
            settled = true;
            break;
        end
    end
    % If never settled, settling_times(s) stays NaN
end

% --- Hard constraint violations ---
%  Check actual states (not slacks) against the box bounds.
%  This catches true violations, not just slack usage.
n_hard_violations = 0;
max_violation_amount = 0;
for j = 1:n_actual+1
    for c = 1:length(idx_constr)
        val = plot_x(idx_constr(c), j);
        viol_lo = lbx_constr(c) - val;  % positive if violated
        viol_hi = val - ubx_constr(c);  % positive if violated
        worst = max([viol_lo, viol_hi, 0]);
        if worst > 0
            n_hard_violations = n_hard_violations + 1;
            max_violation_amount = max(max_violation_amount, worst);
        end
    end
end

%% --- PRINT COMPREHENSIVE STATISTICS ---
fprintf('\n====================================================================\n');
fprintf('  BASELINE DATA COLLECTION — %s\n', experiment_name);
fprintf('====================================================================\n\n');

fprintf('--- Configuration ---\n');
fprintf('  Experiment:        %s\n', experiment_name);
fprintf('  NLP solver:        %s\n', cfg_nlp_solver);
fprintf('  Integrator:        %s (stages=%d, steps=%d)\n', ...
    cfg_integrator, cfg_num_stages, cfg_num_steps);
fprintf('  QP solver:         %s (max_iter=%d)\n', cfg_qp_solver, cfg_qp_iter_max);
fprintf('  Hessian approx:    %s\n', cfg_hessian);
fprintf('  Cost type:         %s\n', cfg_cost_type);
fprintf('  Horizon:           N=%d, Tf=%.3f s, dt=%.1f ms\n', N, Tf, dt_mpc*1e3);
fprintf('  States:            nx=%d (reduced from 12)\n', nx);
fprintf('  Inputs:            nu=%d\n', nu);
fprintf('  Magnet discret.:   n=%d\n', paramsFast.magnet.n);
fprintf('  Perturbation:      scale=%.2f\n', perturbation_scale);
fprintf('  Equilibrium z:     %.4f mm\n', zEq_cas*1e3);
fprintf('\n');

fprintf('--- Simulation Result ---\n');
fprintf('  Steps completed:   %d / %d\n', n_actual, sim_steps);
fprintf('  Diverged:          %s\n', ternary(n_actual < sim_steps, 'YES', 'NO'));
fprintf('\n');

fprintf('--- Solver Status Distribution ---\n');
fprintf('  Status 0 (converged):   %d  (%.1f%%)\n', ...
    sum(stat_vec==0), 100*sum(stat_vec==0)/n_actual);
fprintf('  Status 1 (other):       %d  (%.1f%%)\n', ...
    sum(stat_vec==1), 100*sum(stat_vec==1)/n_actual);
fprintf('  Status 2 (max SQP):     %d  (%.1f%%)\n', ...
    sum(stat_vec==2), 100*sum(stat_vec==2)/n_actual);
fprintf('  Status 3:               %d  (%.1f%%)\n', ...
    sum(stat_vec==3), 100*sum(stat_vec==3)/n_actual);
fprintf('  Status 4 (QP fail):     %d  (%.1f%%)\n', ...
    sum(stat_vec==4), 100*sum(stat_vec==4)/n_actual);
fprintf('  All converged (st==0):  %s\n', ternary(all(stat_vec==0), 'YES', 'NO'));
fprintf('\n');

fprintf('--- Solve Time [ms] ---\n');
fprintf('  Mean:              %.2f\n', mean(time_vec)*1e3);
fprintf('  Median:            %.2f\n', median(time_vec)*1e3);
fprintf('  Std:               %.2f\n', std(time_vec)*1e3);
fprintf('  Min:               %.2f\n', min(time_vec)*1e3);
fprintf('  Max:               %.2f\n', max(time_vec)*1e3);
fprintf('  95th percentile:   %.2f\n', prctile_(time_vec, 95)*1e3);
fprintf('  99th percentile:   %.2f\n', prctile_(time_vec, 99)*1e3);
fprintf('\n');

fprintf('--- Timing Breakdown (mean) [ms] ---\n');
fprintf('  Linearisation (time_lin):      %.2f  (%.0f%%)\n', ...
    nanmean_(plot_time_lin(1:n_actual))*1e3, ...
    100*nanmean_(plot_time_lin(1:n_actual))/mean(time_vec));
fprintf('  Integration (time_sim):        %.2f  (%.0f%%)\n', ...
    nanmean_(plot_time_sim(1:n_actual))*1e3, ...
    100*nanmean_(plot_time_sim(1:n_actual))/mean(time_vec));
fprintf('    ├─ External func (time_sim_ad): %.2f\n', ...
    nanmean_(plot_time_sim_ad(1:n_actual))*1e3);
fprintf('    └─ Linear algebra (time_sim_la): %.2f\n', ...
    nanmean_(plot_time_sim_la(1:n_actual))*1e3);
fprintf('  QP solve (time_qp_sol):        %.2f  (%.0f%%)\n', ...
    nanmean_(plot_time_qp_sol(1:n_actual))*1e3, ...
    100*nanmean_(plot_time_qp_sol(1:n_actual))/mean(time_vec));
fprintf('  Regularisation (time_reg):     %.2f  (%.0f%%)\n', ...
    nanmean_(plot_time_reg(1:n_actual))*1e3, ...
    100*nanmean_(plot_time_reg(1:n_actual))/mean(time_vec));
fprintf('  Globalization (time_glob):     %.2f\n', ...
    nanmean_(plot_time_glob(1:n_actual))*1e3);
fprintf('  Unaccounted:                   %.2f\n', ...
    (mean(time_vec) - nanmean_(plot_time_lin(1:n_actual)) ...
     - nanmean_(plot_time_qp_sol(1:n_actual)) ...
     - nanmean_(plot_time_reg(1:n_actual)) ...
     - nanmean_(plot_time_glob(1:n_actual)))*1e3);
fprintf('\n');

fprintf('--- Iteration Counts ---\n');
fprintf('  SQP iterations:   mean=%.1f, median=%.0f, max=%d\n', ...
    mean(sqp_vec), median(sqp_vec), max(sqp_vec));
fprintf('  QP  iterations:   mean=%.1f, median=%.0f, max=%d\n', ...
    nanmean_(qp_vec), nanmedian_(qp_vec), nanmax_(qp_vec));
fprintf('\n');

fprintf('--- Tracking Performance ---\n');
fprintf('  Initial ||x-xEq||:           %.4e\n', initial_err);
fprintf('  Final   ||x-xEq||:           %.4e\n', plot_err_norm(n_actual+1));
fprintf('  Final position error:         %.4f mm\n', plot_err_pos(n_actual+1));
fprintf('  Final orientation error:      %.4f deg\n', plot_err_ang(n_actual+1));
fprintf('  Final z:                      %.4f mm  (eq: %.4f mm)\n', ...
    plot_x(3,n_actual+1)*1e3, xEq(3)*1e3);
fprintf('  Settling time (1%% of init):  ');
if isnan(settling_times(1)); fprintf('NOT SETTLED\n');
else; fprintf('%.1f ms\n', settling_times(1)*1e3); end
fprintf('  Settling time (2%% of init):  ');
if isnan(settling_times(2)); fprintf('NOT SETTLED\n');
else; fprintf('%.1f ms\n', settling_times(2)*1e3); end
fprintf('  Settling time (5%% of init):  ');
if isnan(settling_times(3)); fprintf('NOT SETTLED\n');
else; fprintf('%.1f ms\n', settling_times(3)*1e3); end
fprintf('\n');

fprintf('--- Constraint Violations ---\n');
fprintf('  Max soft slack (all stages):  %.4e\n', max(plot_max_slack(1:n_actual)));
fprintf('  Mean soft slack:              %.4e\n', mean(plot_max_slack(1:n_actual)));
fprintf('  Hard violation count:         %d\n', n_hard_violations);
fprintf('  Max hard violation amount:    %.4e\n', max_violation_amount);
fprintf('\n');

fprintf('====================================================================\n');
fprintf('  TARGET: 5 ms    |   CURRENT MEAN: %.2f ms    |   GAP: %.1fx\n', ...
    mean(time_vec)*1e3, mean(time_vec)*1e3 / 5.0);
fprintf('====================================================================\n');

%% --- SAVE COMPREHENSIVE RESULTS ---
sim_data = struct();

% --- Experiment metadata ---
sim_data.experiment_name  = experiment_name;
sim_data.experiment_desc  = experiment_desc;
sim_data.timestamp        = datestr(now, 'yyyy-mm-dd_HH-MM-SS');

% --- Configuration (for reproducibility) ---
sim_data.config.nlp_solver_type       = cfg_nlp_solver;
sim_data.config.integrator_type       = cfg_integrator;
sim_data.config.sim_method_num_stages = cfg_num_stages;
sim_data.config.sim_method_num_steps  = cfg_num_steps;
sim_data.config.qp_solver             = cfg_qp_solver;
sim_data.config.qp_solver_iter_max    = cfg_qp_iter_max;
sim_data.config.hessian_approx        = cfg_hessian;
sim_data.config.regularize_method     = cfg_regularize;
sim_data.config.cost_type             = cfg_cost_type;
sim_data.config.N                     = N;
sim_data.config.Tf                    = Tf;
sim_data.config.dt                    = dt_mpc;
sim_data.config.nx                    = nx;
sim_data.config.nu                    = nu;
sim_data.config.magnet_n              = paramsFast.magnet.n;
sim_data.config.perturbation_scale    = perturbation_scale;

% --- Trajectories ---
sim_data.t  = (0:sim_steps) * dt_mpc;
sim_data.x  = plot_x;
sim_data.u  = plot_u;

% --- Solver status and iterations ---
sim_data.status   = plot_stat;
sim_data.sqp_iter = plot_sqp;
sim_data.qp_iter  = plot_qp_iter;

% --- Full timing breakdown (raw vectors, in seconds) ---
sim_data.timing.time_tot    = plot_time_tot;
sim_data.timing.time_lin    = plot_time_lin;
sim_data.timing.time_sim    = plot_time_sim;
sim_data.timing.time_sim_ad = plot_time_sim_ad;
sim_data.timing.time_sim_la = plot_time_sim_la;
sim_data.timing.time_qp_sol = plot_time_qp_sol;
sim_data.timing.time_reg    = plot_time_reg;
sim_data.timing.time_glob   = plot_time_glob;

% --- Tracking errors ---
sim_data.tracking.err_norm = plot_err_norm;
sim_data.tracking.err_pos  = plot_err_pos;
sim_data.tracking.err_ang  = plot_err_ang;

% --- Constraint violations ---
sim_data.constraints.max_slack          = plot_max_slack;
sim_data.constraints.n_hard_violations  = n_hard_violations;
sim_data.constraints.max_violation      = max_violation_amount;

% --- Settling times ---
sim_data.settling.thresholds_pct = settle_thresholds * 100;  % [1, 2, 5] percent
sim_data.settling.times_s        = settling_times;           % in seconds

% --- Summary statistics (for quick comparison table) ---
sim_data.summary.n_actual                = n_actual;
sim_data.summary.all_converged           = all(stat_vec==0);
sim_data.summary.solve_time_mean_ms      = mean(time_vec)*1e3;
sim_data.summary.solve_time_median_ms    = median(time_vec)*1e3;
sim_data.summary.solve_time_std_ms       = std(time_vec)*1e3;
sim_data.summary.solve_time_max_ms       = max(time_vec)*1e3;
sim_data.summary.solve_time_min_ms       = min(time_vec)*1e3;
sim_data.summary.solve_time_p95_ms       = prctile_(time_vec, 95)*1e3;
sim_data.summary.solve_time_p99_ms       = prctile_(time_vec, 99)*1e3;
sim_data.summary.sqp_iter_mean           = mean(sqp_vec);
sim_data.summary.qp_iter_mean            = nanmean_(qp_vec);
sim_data.summary.time_lin_mean_ms        = nanmean_(plot_time_lin(1:n_actual))*1e3;
sim_data.summary.time_sim_mean_ms        = nanmean_(plot_time_sim(1:n_actual))*1e3;
sim_data.summary.time_qp_sol_mean_ms     = nanmean_(plot_time_qp_sol(1:n_actual))*1e3;
sim_data.summary.time_reg_mean_ms        = nanmean_(plot_time_reg(1:n_actual))*1e3;
sim_data.summary.final_err_norm          = plot_err_norm(n_actual+1);
sim_data.summary.final_err_pos_mm        = plot_err_pos(n_actual+1);
sim_data.summary.final_err_ang_deg       = plot_err_ang(n_actual+1);
sim_data.summary.settling_1pct_ms        = settling_times(1)*1e3;
sim_data.summary.settling_2pct_ms        = settling_times(2)*1e3;
sim_data.summary.settling_5pct_ms        = settling_times(3)*1e3;
sim_data.summary.max_soft_slack          = max(plot_max_slack(1:n_actual));
sim_data.summary.pct_status_0            = 100*sum(stat_vec==0)/n_actual;

% --- Original fields for backward compatibility ---
sim_data.xEq    = xEq;
sim_data.uEq    = uEq;
sim_data.Q      = Q;
sim_data.R      = R;
sim_data.params = paramsFast;

save_filename = sprintf('nmpc_results_%s.mat', lower(strrep(experiment_name, ' ', '_')));
save(save_filename, '-struct', 'sim_data');
fprintf('\nResults saved to %s\n', save_filename);

%% --- PLOTS ---
% Wrapped in try-catch so the script completes even under -nojvm (no display).
try
t_x = (0:n_actual) * dt_mpc;
t_u = (0:n_actual-1) * dt_mpc;

fig1 = figure('Name', sprintf('NMPC Simulation — %s', experiment_name), ...
    'Position', [50 50 1400 900]);

% --- Position ---
subplot(3,2,1); hold on; grid on;
plot(t_x*1e3, plot_x(1,1:n_actual+1)*1e3, 'LineWidth', 1.2);
plot(t_x*1e3, plot_x(2,1:n_actual+1)*1e3, 'LineWidth', 1.2);
plot(t_x*1e3, plot_x(3,1:n_actual+1)*1e3, 'LineWidth', 1.2);
yline(xEq(3)*1e3, '--k', 'z_{eq}');
xlabel('Time [ms]'); ylabel('[mm]');
legend('x','y','z','Location','best');
title('Position');

% --- Orientation (only roll & pitch) ---
subplot(3,2,2); hold on; grid on;
plot(t_x*1e3, rad2deg(plot_x(4,1:n_actual+1)), 'LineWidth', 1.2);
plot(t_x*1e3, rad2deg(plot_x(5,1:n_actual+1)), 'LineWidth', 1.2);
xlabel('Time [ms]'); ylabel('[deg]');
legend('\alpha (roll)','\beta (pitch)','Location','best');
title('Orientation');

% --- Linear velocity ---
subplot(3,2,3); hold on; grid on;
plot(t_x*1e3, plot_x(6,1:n_actual+1)*1e3, 'LineWidth', 1.2);
plot(t_x*1e3, plot_x(7,1:n_actual+1)*1e3, 'LineWidth', 1.2);
plot(t_x*1e3, plot_x(8,1:n_actual+1)*1e3, 'LineWidth', 1.2);
xlabel('Time [ms]'); ylabel('[mm/s]');
legend('v_x','v_y','v_z','Location','best');
title('Linear velocity');

% --- Angular velocity (only wx, wy) ---
subplot(3,2,4); hold on; grid on;
plot(t_x*1e3, rad2deg(plot_x(9,1:n_actual+1)), 'LineWidth', 1.2);
plot(t_x*1e3, rad2deg(plot_x(10,1:n_actual+1)), 'LineWidth', 1.2);
xlabel('Time [ms]'); ylabel('[deg/s]');
legend('\omega_x','\omega_y','Location','best');
title('Angular velocity');

% --- Control inputs ---
subplot(3,2,5); hold on; grid on;
stairs(t_u*1e3, plot_u(1,1:n_actual), 'LineWidth', 1.2);
stairs(t_u*1e3, plot_u(2,1:n_actual), 'LineWidth', 1.2);
stairs(t_u*1e3, plot_u(3,1:n_actual), 'LineWidth', 1.2);
stairs(t_u*1e3, plot_u(4,1:n_actual), 'LineWidth', 1.2);
yline(-1, '--k'); yline(1, '--k');
xlabel('Time [ms]'); ylabel('[A]');
legend('u_1','u_2','u_3','u_4','Location','best');
title('Solenoid currents');

% --- Solver performance ---
subplot(3,2,6); hold on; grid on;
yyaxis left;
stem(t_u*1e3, plot_stat(1:n_actual), 'filled', 'MarkerSize', 3);
ylabel('Status (0=OK)');
yyaxis right;
plot(t_u*1e3, plot_time_tot(1:n_actual)*1000, 'LineWidth', 1.0);
ylabel('Solve time [ms]');
xlabel('Time [ms]');
title('Solver performance');

sgtitle(sprintf('NMPC stabilisation — %s (nx=%d, N=%d, T_f=%.0fms, scale=%.2f)', ...
    experiment_name, nx, N, Tf*1000, perturbation_scale));

savefig(fig1, sprintf('nmpc_simulation_%s.fig', lower(strrep(experiment_name, ' ', '_'))));

%% --- ADDITIONAL FIGURE: TIMING BREAKDOWN & CONVERGENCE ---
fig2 = figure('Name', sprintf('Timing & Convergence — %s', experiment_name), ...
    'Position', [100 100 1400 700]);

% --- Timing breakdown (stacked area) ---
subplot(2,2,1); hold on; grid on;
timing_stack = [plot_time_sim(1:n_actual); ...
                plot_time_qp_sol(1:n_actual); ...
                plot_time_reg(1:n_actual); ...
                max(0, plot_time_tot(1:n_actual) ...
                  - plot_time_sim(1:n_actual) ...
                  - plot_time_qp_sol(1:n_actual) ...
                  - plot_time_reg(1:n_actual))] * 1e3;
% Replace NaN with 0 for stacking
timing_stack(isnan(timing_stack)) = 0;
area(t_u*1e3, timing_stack', 'LineWidth', 0.5);
yline(5, '--r', '5 ms target', 'LineWidth', 1.5);
xlabel('Time [ms]'); ylabel('Solve time [ms]');
legend('Integration', 'QP solve', 'Regularisation', 'Other', 'Location', 'best');
title('Timing breakdown');

% --- Solve time histogram ---
subplot(2,2,2);
histogram(time_vec*1e3, 30, 'FaceColor', [0.3 0.5 0.8]);
hold on; grid on;
xline(mean(time_vec)*1e3, '-r', sprintf('mean=%.1fms', mean(time_vec)*1e3), 'LineWidth', 1.5);
xline(median(time_vec)*1e3, '--b', sprintf('median=%.1fms', median(time_vec)*1e3), 'LineWidth', 1.2);
xline(5, '--k', '5 ms target', 'LineWidth', 1.5);
xlabel('Solve time [ms]'); ylabel('Count');
title('Solve time distribution');

% --- Tracking error convergence ---
subplot(2,2,3); hold on; grid on;
semilogy(t_x*1e3, plot_err_norm(1:n_actual+1), 'LineWidth', 1.2);
for s = 1:length(settle_thresholds)
    yline(settle_thresholds(s)*initial_err, '--', ...
        sprintf('%d%%', settle_thresholds(s)*100), 'Color', [0.5 0.5 0.5]);
end
xlabel('Time [ms]'); ylabel('||x - x_{eq}||');
title('Tracking error convergence');

% --- SQP and QP iterations ---
subplot(2,2,4); hold on; grid on;
yyaxis left;
stem(t_u*1e3, plot_sqp(1:n_actual), 'filled', 'MarkerSize', 3);
ylabel('SQP iterations');
yyaxis right;
stem(t_u*1e3, plot_qp_iter(1:n_actual), '.', 'MarkerSize', 5);
ylabel('QP iterations (total)');
xlabel('Time [ms]');
title('Iteration counts per step');

sgtitle(sprintf('Timing & Convergence — %s', experiment_name));

savefig(fig2, sprintf('nmpc_timing_%s.fig', lower(strrep(experiment_name, ' ', '_'))));

fprintf('Figures saved.\n');

catch plot_err
    fprintf('Skipping plots (no display available: %s).\n', plot_err.message);
    fprintf('Data is fully saved in the .mat file.\n');
end

%% --- HELPER FUNCTIONS ---
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end

function m = nanmean_(x)
    x = x(~isnan(x));
    if isempty(x); m = NaN; else; m = mean(x); end
end

function m = nanmedian_(x)
    x = x(~isnan(x));
    if isempty(x); m = NaN; else; m = median(x); end
end

function m = nanmax_(x)
    x = x(~isnan(x));
    if isempty(x); m = NaN; else; m = max(x); end
end

function p = prctile_(x, pct)
    % Simple percentile without Statistics Toolbox.
    x = sort(x(~isnan(x)));
    n = length(x);
    if n == 0; p = NaN; return; end
    r = (pct/100) * n;
    lo = max(floor(r), 1);
    hi = min(ceil(r), n);
    if lo == hi; p = x(lo); else; p = x(lo) + (r - lo)*(x(hi) - x(lo)); end
end