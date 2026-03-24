%% acadosNMPC_simulation_final.m
% =========================================================================
%  NMPC simulation for the magnetic levitation system using acados.
%
%  This script performs closed-loop NMPC stabilisation from a perturbed
%  initial condition back to the magnetic equilibrium. The perturbation
%  corresponds to a sub-millimetre, sub-degree displacement representative
%  of typical operational disturbances.
%
%  Initial perturbation from equilibrium (scale s = 0.05):
%    Position:         (0.25, −0.40, 0.50) mm
%    Orientation:      (0.43°, −0.29°, 0.86°)
%    Linear velocity:  (0.5, −0.5, 0) mm/s
%    Angular velocity: (0.57°, −0.86°, 0) °/s
%
%  Solver:    acados SQP with Gauss–Newton Hessian approximation
%  QP:        PARTIAL_CONDENSING_HPIPM
%  Integrator: IRK (Gauss–Legendre, 4 stages, 10 steps per shooting interval)
%  Horizon:   N = 20 nodes, Tf = 0.20 s (dt = 10 ms)
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

%% --- MODEL SETUP ---
nx   = 12;
nu   = 4;
x    = SX.sym('x',    nx);
u    = SX.sym('u',    nu);
xdot = SX.sym('xdot', nx);

parameters_maggy_V4;
correctionFactorFast       = computeSolenoidRadiusCorrectionFactor(params, 'fast');
paramsFast                 = params;
paramsFast.solenoids.r     = correctionFactorFast * paramsFast.solenoids.r;

%% --- BUILD CASADI FUNCTION ---
f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
f_func = casadi.Function('f', {x, u}, {f_expl});

%% --- FIND EQUILIBRIUM ---
fprintf('--- Searching for equilibrium z with u = 0 ---\n');

z_var    = SX.sym('z_eq');
u_zero   = zeros(nu, 1);
x_eq_sym = [0; 0; z_var; zeros(9,1)];
accel    = f_func(x_eq_sym, u_zero);
fz_residual = accel(9);

nlp       = struct('x', z_var, 'f', fz_residual^2);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 3)));

sol     = solver_eq('x0', 0.030, 'lbx', 0.015, 'ubx', 0.060);
zEq_cas = full(sol.x);
uEq     = zeros(nu, 1);
xEq     = [0; 0; zEq_cas; zeros(9,1)];

fprintf('Equilibrium: z = %.6f m, u = [0, 0, 0, 0]\n', zEq_cas);

%% --- OCP SETUP ---
N      = 20;
Tf     = 0.2;
dt_mpc = Tf / N;   % 0.01 s

% Cost matrices
Q = diag([1e2, 1e2, 1e3, ...    % x, y, z position
          1e3, 1e3, 1e1, ...    % roll, pitch, yaw angles
          1e1, 1e1, 1e1, ...    % vx, vy, vz
          1e1, 1e1, 1e0]);      % wx, wy, wz
R = eye(nu) * 1.0;

ocp = AcadosOcp();
ocp.model.name        = 'maglev_nmpc';
ocp.model.x           = x;
ocp.model.u           = u;
ocp.model.xdot        = xdot;
ocp.model.f_impl_expr = xdot - f_expl;

% Solver options
ocp.solver_options.N_horizon             = N;
ocp.solver_options.tf                    = Tf;
ocp.solver_options.integrator_type       = 'IRK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 10;
ocp.solver_options.nlp_solver_type       = 'SQP';
ocp.solver_options.nlp_solver_max_iter   = 100;
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

ocp.model.cost_y_expr   = [x; u];
ocp.model.cost_y_expr_0 = [x; u];
ocp.model.cost_y_expr_e = x;

ocp.cost.yref   = [xEq; uEq];
ocp.cost.yref_0 = [xEq; uEq];
ocp.cost.yref_e = xEq;

% Input constraints
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu   = -1 * ones(nu,1);
ocp.constraints.ubu   =  1 * ones(nu,1);

% State constraints (soft)
ocp.constraints.idxbx  = [0, 1, 2, 3, 4];
ocp.constraints.lbx    = [-0.025; -0.025; 0.015; -0.35; -0.35];
ocp.constraints.ubx    = [ 0.025;  0.025; 0.055;  0.35;  0.35];
ocp.constraints.idxsbx = 0:4;

n_sbx = 5;
ocp.cost.Zl = 1e3 * ones(n_sbx, 1);
ocp.cost.Zu = 1e3 * ones(n_sbx, 1);
ocp.cost.zl = 1e2 * ones(n_sbx, 1);
ocp.cost.zu = 1e2 * ones(n_sbx, 1);

ocp.constraints.x0 = xEq;

%% --- BUILD OCP SOLVER ---
if ~exist('ocp_solver', 'var') || ~isvalid(ocp_solver)
    fprintf('\n--- Building acados OCP solver ---\n');
    ocp_solver = AcadosOcpSolver(ocp);
else
    fprintf('\n--- Reusing OCP solver ---\n');
end

%% --- BUILD SIM SOLVER ---
if ~exist('sim_solver', 'var') || ~isvalid(sim_solver)
    fprintf('\n--- Building acados sim solver ---\n');
    sim = AcadosSim();
    sim.model.name        = 'maglev_sim';
    sim.model.x           = x;
    sim.model.u           = u;
    sim.model.xdot        = xdot;
    sim.model.f_impl_expr = xdot - f_expl;
    sim.solver_options.Tsim            = dt_mpc;
    sim.solver_options.integrator_type = 'IRK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 10;
    sim_solver = AcadosSimSolver(sim);
else
    fprintf('\n--- Reusing sim solver ---\n');
end

%% --- INITIAL CONDITION ---
% Perturbation direction (excites all DOFs simultaneously)
dx_dir = [0.005; -0.008; 0.010; 0.15; -0.10; 0.3; ...
          0.01;  -0.01;  0.0;   0.2;  -0.3;  0.0];

perturbation_scale = 0.05;
x_current = xEq + perturbation_scale * dx_dir;

fprintf('\n--- Initial perturbation (scale = %.2f) ---\n', perturbation_scale);
fprintf('  Position offset:  (%+.2f, %+.2f, %+.2f) mm\n', ...
    (x_current(1)-xEq(1))*1e3, (x_current(2)-xEq(2))*1e3, (x_current(3)-xEq(3))*1e3);
fprintf('  Orientation:      (%+.2f, %+.2f, %+.2f) deg\n', ...
    rad2deg(x_current(4)), rad2deg(x_current(5)), rad2deg(x_current(6)));

%% --- TRAJECTORY INITIALISATION ---
% Linear interpolation from perturbed state to equilibrium
for k = 0:N
    alpha_k = k / N;
    ocp_solver.set('x', (1 - alpha_k) * x_current + alpha_k * xEq, k);
end
for k = 0:N-1
    ocp_solver.set('u', uEq, k);
end

%% --- SIMULATION ---
sim_steps = 200;

plot_x    = zeros(nx, sim_steps+1);
plot_u    = zeros(nu, sim_steps);
plot_stat = zeros(1,  sim_steps);
plot_sqp  = zeros(1,  sim_steps);
plot_time = zeros(1,  sim_steps);
plot_x(:,1) = x_current;

fprintf('\n--- Starting NMPC simulation (%d steps, %.2f s) ---\n', sim_steps, sim_steps*dt_mpc);

for i = 1:sim_steps
    % --- Solve OCP ---
    ocp_solver.set('constr_x0', x_current);
    ocp_solver.solve();

    status    = ocp_solver.get('status');
    sqp_iter  = ocp_solver.get('sqp_iter');
    time_tot  = ocp_solver.get('time_tot');
    u_applied = ocp_solver.get('u', 0);
    u_applied = max(min(u_applied, 1), -1);

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

    % --- Log ---
    plot_x(:,i+1) = x_next;
    plot_u(:,i)   = u_applied;
    plot_stat(i)  = status;
    plot_sqp(i)   = sqp_iter;
    plot_time(i)  = time_tot;

    % --- Print ---
    if i <= 10 || mod(i,20) == 0 || i == sim_steps
        fprintf('Step %3d: st=%d, sqp=%2d, t=%5.1fms, u=[%+.3f %+.3f %+.3f %+.3f], z=%.4fmm, |ang|=%.3fdeg, |w|=%.2fdeg/s\n', ...
            i, status, sqp_iter, time_tot*1000, ...
            u_applied(1), u_applied(2), u_applied(3), u_applied(4), ...
            x_next(3)*1e3, rad2deg(norm(x_next(4:6))), rad2deg(norm(x_next(10:12))));
    end

    % --- Divergence check ---
    diverged = abs(x_next(3)) > 0.5 || max(abs(x_next(4:6))) > pi || ...
               any(isnan(x_next)) || any(isinf(x_next));
    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
        break;
    end

    x_current = x_next;
end

%% --- STATISTICS ---
n_actual = min(i, sim_steps);
fprintf('\n--- Simulation summary ---\n');
fprintf('  Steps completed:  %d / %d\n', n_actual, sim_steps);
fprintf('  Solver status:    %d OK, %d max-iter, %d QP-fail\n', ...
    sum(plot_stat(1:n_actual)==0), sum(plot_stat(1:n_actual)==2), sum(plot_stat(1:n_actual)==4));
fprintf('  Avg SQP iter:     %.1f\n', mean(plot_sqp(1:n_actual)));
fprintf('  Avg solve time:   %.2f ms\n', mean(plot_time(1:n_actual))*1000);
fprintf('  Max solve time:   %.2f ms\n', max(plot_time(1:n_actual))*1000);
fprintf('  Final |x-xEq|:    %.4e\n', norm(plot_x(:,n_actual+1) - xEq));
fprintf('  Final z:           %.4f mm (eq: %.4f mm)\n', ...
    plot_x(3,n_actual+1)*1e3, xEq(3)*1e3);

%% --- SAVE ---
sim_data              = struct();
sim_data.t            = (0:sim_steps) * dt_mpc;
sim_data.x            = plot_x;
sim_data.u            = plot_u;
sim_data.status       = plot_stat;
sim_data.sqp_iter     = plot_sqp;
sim_data.solve_time   = plot_time;
sim_data.xEq          = xEq;
sim_data.uEq          = uEq;
sim_data.dt           = dt_mpc;
sim_data.N            = N;
sim_data.Tf           = Tf;
sim_data.Q            = Q;
sim_data.R            = R;
sim_data.params       = paramsFast;
sim_data.perturbation_scale = perturbation_scale;

save('nmpc_results.mat', '-struct', 'sim_data');
fprintf('\nResults saved to nmpc_results.mat\n');

%% --- PLOTS ---
t_x = (0:n_actual) * dt_mpc;
t_u = (0:n_actual-1) * dt_mpc;

fig = figure('Name','NMPC Simulation','Position',[50 50 1400 900]);

% --- Position ---
subplot(3,2,1); hold on; grid on;
plot(t_x*1e3, plot_x(1,1:n_actual+1)*1e3, 'LineWidth', 1.2);
plot(t_x*1e3, plot_x(2,1:n_actual+1)*1e3, 'LineWidth', 1.2);
plot(t_x*1e3, plot_x(3,1:n_actual+1)*1e3, 'LineWidth', 1.2);
yline(xEq(3)*1e3, '--k', 'z_{eq}');
xlabel('Time [ms]'); ylabel('[mm]');
legend('x','y','z','Location','best');
title('Position');

% --- Orientation ---
subplot(3,2,2); hold on; grid on;
plot(t_x*1e3, rad2deg(plot_x(4,1:n_actual+1)), 'LineWidth', 1.2);
plot(t_x*1e3, rad2deg(plot_x(5,1:n_actual+1)), 'LineWidth', 1.2);
plot(t_x*1e3, rad2deg(plot_x(6,1:n_actual+1)), 'LineWidth', 1.2);
xlabel('Time [ms]'); ylabel('[deg]');
legend('\alpha (roll)','\beta (pitch)','\gamma (yaw)','Location','best');
title('Orientation');

% --- Linear velocity ---
subplot(3,2,3); hold on; grid on;
plot(t_x*1e3, plot_x(7,1:n_actual+1)*1e3, 'LineWidth', 1.2);
plot(t_x*1e3, plot_x(8,1:n_actual+1)*1e3, 'LineWidth', 1.2);
plot(t_x*1e3, plot_x(9,1:n_actual+1)*1e3, 'LineWidth', 1.2);
xlabel('Time [ms]'); ylabel('[mm/s]');
legend('v_x','v_y','v_z','Location','best');
title('Linear velocity');

% --- Angular velocity ---
subplot(3,2,4); hold on; grid on;
plot(t_x*1e3, rad2deg(plot_x(10,1:n_actual+1)), 'LineWidth', 1.2);
plot(t_x*1e3, rad2deg(plot_x(11,1:n_actual+1)), 'LineWidth', 1.2);
plot(t_x*1e3, rad2deg(plot_x(12,1:n_actual+1)), 'LineWidth', 1.2);
xlabel('Time [ms]'); ylabel('[deg/s]');
legend('\omega_x','\omega_y','\omega_z','Location','best');
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
plot(t_u*1e3, plot_time(1:n_actual)*1000, 'LineWidth', 1.0);
ylabel('Solve time [ms]');
xlabel('Time [ms]');
title('Solver performance');

sgtitle(sprintf('NMPC stabilisation (N=%d, T_f=%.0fms, perturbation scale=%.2f)', ...
    N, Tf*1000, perturbation_scale));

% Save figure
savefig(fig, 'nmpc_simulation.fig');
fprintf('Figure saved to nmpc_simulation.fig\n');