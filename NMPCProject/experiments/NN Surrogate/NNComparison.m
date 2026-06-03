%%
%% acadosNMPC_NNvsAnalyticComparison_v3.m
%% ─────────────────────────────────────────────────────────────────────
%% Linearity decomposition: NN_perm(pose) + NN_sol(pose) * u
%%
%% Uses EXACTLY the same OCP settings as workingSimulator.m.
%%
%% BEFORE RUNNING (if retrained):
%%   >> clear ocp_solver_nn sim_solver ocp_solver_ana
%%   Then: rm -rf c_generated_code/maglev_nn*
%% ─────────────────────────────────────────────────────────────────────

%% --- PROJECT SETUP ---
clearvars -except ocp_solver_ana ocp_solver_nn sim_solver; clc;

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
addpath(genpath(fullfile(project_root, 'results')));

import casadi.*

%% ═══════════════════════════════════════════════════════════════════
%%  1. ANALYTICAL MODEL
%% ═══════════════════════════════════════════════════════════════════
nx   = 12; nu = 4;
x    = SX.sym('x',    nx);
u    = SX.sym('u',    nu);
xdot = SX.sym('xdot', nx);

parameters_maggy_V4;
correctionFactorFast       = computeSolenoidRadiusCorrectionFactor(params, 'fast');
paramsFast                 = params;
paramsFast.solenoids.r     = correctionFactorFast * paramsFast.solenoids.r;

f_expl_ana = maglevSystemDynamicsCasADi(x, u, paramsFast);
f_func     = casadi.Function('f', {x, u}, {f_expl_ana});

%% --- FIND EQUILIBRIUM ---
fprintf('=== Finding equilibrium ===\n');
z_var    = SX.sym('z_eq');
x_eq_sym = [0; 0; z_var; zeros(9,1)];
accel    = f_func(x_eq_sym, zeros(nu,1));

nlp       = struct('x', z_var, 'f', accel(9)^2);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 0)));
sol     = solver_eq('x0', 0.030, 'lbx', 0.015, 'ubx', 0.060);
zEq_cas = full(sol.x);
uEq     = zeros(nu, 1);
xEq     = [0; 0; zEq_cas; zeros(9,1)];
fprintf('  z_eq = %.6f m, residual = %.4e\n', zEq_cas, norm(full(f_func(xEq, uEq))));

%% ═══════════════════════════════════════════════════════════════════
%%  2. NN SURROGATE (linearity decomposition)
%% ═══════════════════════════════════════════════════════════════════
fprintf('\n=== Loading NN weights (v3 — linearity decomposition) ===\n');

nn_file = fullfile(project_root, 'results', 'nn_weights_v3.mat');
if ~isfile(nn_file)
    error('Weights not found: %s\nRun generate_nn_data_v3.m then train_surrogate_v3.py', nn_file);
end
nn_raw = load(nn_file);

fprintf('  Architecture: %s\n', nn_raw.architecture);
assert(strcmp(nn_raw.output_type, 'linearity_decomposition'), ...
    'Expected linearity_decomposition, got: %s', nn_raw.output_type);

% --- Helper: build MLP forward pass in CasADi ---
    function out = nn_forward(input_vec, W1, b1, W2, b2, W3, b3, Wout, bout, ...
                              in_mean, in_std, out_mean, out_std)
        z = (input_vec - in_mean) ./ in_std;
        h = tanh(W1 * z + b1);
        h = tanh(W2 * h + b2);
        h = tanh(W3 * h + b3);
        raw = Wout * h + bout;
        out = raw .* out_std + out_mean;
    end

% --- Build NN_perm CasADi expression ---
pose = x(1:6);  % 6x1

perm_out = nn_forward(pose, ...
    nn_raw.perm_W1, nn_raw.perm_b1(:), ...
    nn_raw.perm_W2, nn_raw.perm_b2(:), ...
    nn_raw.perm_W3, nn_raw.perm_b3(:), ...
    nn_raw.perm_Wout, nn_raw.perm_bout(:), ...
    nn_raw.perm_input_mean(:), nn_raw.perm_input_std(:), ...
    nn_raw.perm_output_mean(:), nn_raw.perm_output_std(:));  % 6x1

fprintf('  NN_perm: 6→6 built (W1=%dx%d)\n', size(nn_raw.perm_W1));

% --- Build NN_sol CasADi expression ---
sol_flat = nn_forward(pose, ...
    nn_raw.sol_W1, nn_raw.sol_b1(:), ...
    nn_raw.sol_W2, nn_raw.sol_b2(:), ...
    nn_raw.sol_W3, nn_raw.sol_b3(:), ...
    nn_raw.sol_Wout, nn_raw.sol_bout(:), ...
    nn_raw.sol_input_mean(:), nn_raw.sol_input_std(:), ...
    nn_raw.sol_output_mean(:), nn_raw.sol_output_std(:));  % 24x1

fprintf('  NN_sol:  6→24 built (W1=%dx%d)\n', size(nn_raw.sol_W1));

% Reshape sol_flat to 6x4 matrix: each column is one solenoid's contribution
sol_matrix = [sol_flat(1:6), sol_flat(7:12), sol_flat(13:18), sol_flat(19:24)];

% --- Combined acceleration: exact linearity in u ---
accel_nn = perm_out + mtimes(sol_matrix, u);  % 6x1

fprintf('  Combined: accel = NN_perm(pose) + NN_sol_matrix(pose) * u\n');
fprintf('  KEY: df/du = NN_sol_matrix(pose) — no chain rule through NN!\n');

% --- Reconstruct full dynamics ---
I_vec    = paramsFast.magnet.I;
I_matrix = diag(I_vec);
omega    = x(10:12);
I_omega  = I_matrix * omega;
T_gyro   = [omega(2)*I_omega(3) - omega(3)*I_omega(2);
            omega(3)*I_omega(1) - omega(1)*I_omega(3);
            omega(1)*I_omega(2) - omega(2)*I_omega(1)];
gyro_accel = I_matrix \ T_gyro;

f_expl_nn = [x(7:9); x(10:12); accel_nn(1:3); accel_nn(4:6) - gyro_accel];

% --- Equilibrium check ---
f_nn_func = casadi.Function('f_nn', {x, u}, {f_expl_nn});
nn_dx_eq  = full(f_nn_func(xEq, uEq));
ana_dx_eq = full(f_func(xEq, uEq));

fprintf('\n  Equilibrium check:\n');
fprintf('    Analytical residual: %.4e\n', norm(ana_dx_eq));
fprintf('    NN residual:         %.4e\n', norm(nn_dx_eq));
fprintf('    NN accel at eq: lin=[%.3e, %.3e, %.3e], ang=[%.3e, %.3e, %.3e]\n', ...
    nn_dx_eq(7), nn_dx_eq(8), nn_dx_eq(9), nn_dx_eq(10), nn_dx_eq(11), nn_dx_eq(12));

%% ═══════════════════════════════════════════════════════════════════
%%  3. OCP SETTINGS (matching workingSimulator.m)
%% ═══════════════════════════════════════════════════════════════════
N      = 20;
Tf     = 0.2;
dt_mpc = Tf / N;

Q = diag([1e2, 1e2, 1e3, 1e3, 1e3, 1e1, 1e1, 1e1, 1e1, 1e1, 1e1, 1e0]);
R = eye(nu) * 1.0;
n_sbx = 5;

    function ocp = build_ocp(model_name, x, u, xdot, f_expl, ...
                              N, Tf, Q, R, n_sbx, xEq, uEq)
        ocp = AcadosOcp();
        ocp.model.name        = model_name;
        ocp.model.x           = x;
        ocp.model.u           = u;
        ocp.model.xdot        = xdot;
        ocp.model.f_impl_expr = xdot - f_expl;

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
        ocp.solver_options.regularize_method      = 'CONVEXIFY';

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

        ocp.constraints.idxbu  = 0:3;
        ocp.constraints.lbu    = -1 * ones(4,1);
        ocp.constraints.ubu    =  1 * ones(4,1);

        ocp.constraints.idxbx  = [0, 1, 2, 3, 4];
        ocp.constraints.lbx    = [-0.025; -0.025; 0.015; -0.35; -0.35];
        ocp.constraints.ubx    = [ 0.025;  0.025; 0.055;  0.35;  0.35];
        ocp.constraints.idxsbx = 0:4;
        ocp.cost.Zl = 1e3 * ones(n_sbx, 1);
        ocp.cost.Zu = 1e3 * ones(n_sbx, 1);
        ocp.cost.zl = 1e2 * ones(n_sbx, 1);
        ocp.cost.zu = 1e2 * ones(n_sbx, 1);
        ocp.constraints.x0 = xEq;
    end

%% ═══════════════════════════════════════════════════════════════════
%%  4. BUILD SOLVERS
%% ═══════════════════════════════════════════════════════════════════

if ~exist('ocp_solver_ana', 'var') || ~isvalid(ocp_solver_ana)
    fprintf('\n=== Building ANALYTICAL OCP solver ===\n');
    ocp_ana = build_ocp('maglev_ana', x, u, xdot, f_expl_ana, N, Tf, Q, R, n_sbx, xEq, uEq);
    if exist(fullfile('c_generated_code', 'maglev_ana_solver'), 'dir')
        ocp_ana.solver_options.compile_interface = false;
    end
    ocp_solver_ana = AcadosOcpSolver(ocp_ana);
else
    fprintf('\n=== Reusing ANALYTICAL solver ===\n');
end

if ~exist('ocp_solver_nn', 'var') || ~isvalid(ocp_solver_nn)
    fprintf('\n=== Building NN OCP solver (two 3x64 networks) ===\n');
    fprintf('  First compile may take 15-30 min. Cached after.\n');
    ocp_nn = build_ocp('maglev_nn', x, u, xdot, f_expl_nn, N, Tf, Q, R, n_sbx, xEq, uEq);
    if exist(fullfile('c_generated_code', 'maglev_nn_solver'), 'dir')
        ocp_nn.solver_options.compile_interface = false;
    end
    ocp_solver_nn = AcadosOcpSolver(ocp_nn);
else
    fprintf('\n=== Reusing NN solver ===\n');
end

if ~exist('sim_solver', 'var') || ~isvalid(sim_solver)
    fprintf('\n=== Building plant sim solver ===\n');
    sim = AcadosSim();
    sim.model.name        = 'maglev_sim';
    sim.model.x           = x;
    sim.model.u           = u;
    sim.model.xdot        = xdot;
    sim.model.f_impl_expr = xdot - f_expl_ana;
    sim.solver_options.Tsim            = dt_mpc;
    sim.solver_options.integrator_type = 'IRK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 10;
    if exist(fullfile('c_generated_code', 'maglev_sim'), 'dir')
        sim.solver_options.compile_interface = false;
    end
    sim_solver = AcadosSimSolver(sim);
else
    fprintf('\n=== Reusing sim solver ===\n');
end

%% ═══════════════════════════════════════════════════════════════════
%%  5. SIMULATION
%% ═══════════════════════════════════════════════════════════════════

function [plot_x, plot_u, plot_stat, plot_sqp, plot_time, final_step] = ...
    run_simulation(ocp_solver, sim_solver, x0, xEq, uEq, ...
                   sim_steps, N, nx, nu, label)

    plot_x      = zeros(nx, sim_steps+1);
    plot_u      = zeros(nu, sim_steps);
    plot_stat   = zeros(1, sim_steps);
    plot_sqp    = zeros(1, sim_steps);
    plot_time   = zeros(1, sim_steps);
    plot_x(:,1) = x0;
    x_current   = x0;

    ocp_solver.reset();
    for k = 0:N
        alpha_k = k / N;
        ocp_solver.set('x', (1 - alpha_k) * x0 + alpha_k * xEq, k);
    end
    for k = 0:N-1, ocp_solver.set('u', uEq, k); end

    final_step = sim_steps;
    for i = 1:sim_steps
        ocp_solver.set('constr_x0', x_current);
        ocp_solver.solve();

        status    = ocp_solver.get('status');
        sqp_iter  = ocp_solver.get('sqp_iter');
        time_tot  = ocp_solver.get('time_tot');
        u_applied = max(min(ocp_solver.get('u', 0), 1), -1);

        sim_solver.set('x', x_current);
        sim_solver.set('u', u_applied);
        sim_solver.solve();
        x_next = sim_solver.get('xn');

        % Warm-start shift (matching workingSimulator.m)
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

        plot_x(:,i+1) = x_next;
        plot_u(:,i)   = u_applied;
        plot_stat(i)  = status;
        plot_sqp(i)   = sqp_iter;
        plot_time(i)  = time_tot;

        if i <= 10 || mod(i,20) == 0 || i == sim_steps
            fprintf('[%s] Step %3d/%d: st=%d, sqp=%2d, t=%5.1fms, |u-uEq|=%.3e, z=%.4fmm\n', ...
                label, i, sim_steps, status, sqp_iter, time_tot*1000, ...
                norm(u_applied - uEq), x_next(3)*1e3);
        end

        diverged = abs(x_next(3)) > 0.5 || max(abs(x_next(4:6))) > pi || ...
                   any(isnan(x_next)) || any(isinf(x_next));
        if diverged
            fprintf('[%s]  *** DIVERGED at step %d ***\n', label, i);
            final_step = i;
            break;
        end
        x_current = x_next;
    end
end

%% ═══════════════════════════════════════════════════════════════════
%%  6. RUN
%% ═══════════════════════════════════════════════════════════════════
sim_steps = 200;
dx_dir = [0.005; -0.008; 0.010; 0.15; -0.10; 0.3; 0.01; -0.01; 0.0; 0.2; -0.3; 0.0];
x0 = xEq + 0.05 * dx_dir;

fprintf('\n================================================================\n');
fprintf('  Perturbation (scale=0.05):\n');
fprintf('  Pos: (%+.2f, %+.2f, %+.2f) mm,  Ang: (%+.2f, %+.2f, %+.2f) deg\n', ...
    (x0(1:3)-xEq(1:3))'*1e3, rad2deg(x0(4:6))');
fprintf('================================================================\n');

fprintf('\n========== RUN 1: ANALYTICAL ==========\n');
[x_ana, u_ana, stat_ana, sqp_ana, t_ana, final_ana] = ...
    run_simulation(ocp_solver_ana, sim_solver, x0, xEq, uEq, sim_steps, N, nx, nu, 'ANA');

if final_ana < sim_steps
    fprintf('\n*** Analytical diverged! Fix before proceeding. ***\n');
    return;
end
fprintf('  Analytical: OK (%d steps), final |x-xEq|=%.4e\n', final_ana, norm(x_ana(:,end)-xEq));

fprintf('\n========== RUN 2: NN (linearity decomposition) ==========\n');
[x_nn, u_nn, stat_nn, sqp_nn, t_nn, final_nn] = ...
    run_simulation(ocp_solver_nn, sim_solver, x0, xEq, uEq, sim_steps, N, nx, nu, ' NN');

%% ═══════════════════════════════════════════════════════════════════
%%  7. RESULTS
%% ═══════════════════════════════════════════════════════════════════
fprintf('\n================================================================\n');
fprintf('  RESULTS\n');
fprintf('================================================================\n');

n_cmp = min(final_ana, final_nn);
if n_cmp > 0
    pos_err = (x_nn(1:3,1:n_cmp+1) - x_ana(1:3,1:n_cmp+1)) * 1e3;
    ang_err = (x_nn(4:6,1:n_cmp+1) - x_ana(4:6,1:n_cmp+1)) * 180/pi;
    fprintf('  Trajectory difference over %d steps:\n', n_cmp);
    fprintf('    Max pos err: Δx=%.4f, Δy=%.4f, Δz=%.4f mm\n', ...
        max(abs(pos_err(1,:))), max(abs(pos_err(2,:))), max(abs(pos_err(3,:))));
    fprintf('    Max ang err: Δr=%.4f, Δp=%.4f, Δy=%.4f deg\n', ...
        max(abs(ang_err(1,:))), max(abs(ang_err(2,:))), max(abs(ang_err(3,:))));
end

fprintf('\n  Timing:\n');
fprintf('    Analytical: mean=%.2f ms, max=%.2f ms\n', ...
    mean(t_ana(1:final_ana))*1e3, max(t_ana(1:final_ana))*1e3);
if final_nn > 0
    fprintf('    NN:         mean=%.2f ms, max=%.2f ms\n', ...
        mean(t_nn(1:final_nn))*1e3, max(t_nn(1:final_nn))*1e3);
    fprintf('    Speedup:    %.2fx\n', mean(t_ana(1:final_ana))/mean(t_nn(1:final_nn)));
end

fprintf('\n  Status:\n');
fprintf('    Analytical: %d OK, %d max-iter, %d QP-fail\n', ...
    sum(stat_ana(1:final_ana)==0), sum(stat_ana(1:final_ana)==2), sum(stat_ana(1:final_ana)==4));
if final_nn > 0
    fprintf('    NN:         %d OK, %d max-iter, %d QP-fail\n', ...
        sum(stat_nn(1:final_nn)==0), sum(stat_nn(1:final_nn)==2), sum(stat_nn(1:final_nn)==4));
end

if final_nn == sim_steps
    mp = max(max(abs(pos_err))); ma = max(max(abs(ang_err)));
    if mp < 0.1 && ma < 0.5
        fprintf('\n  EXCELLENT: NN closely tracks analytical\n');
    elseif mp < 1.0 && ma < 2.0
        fprintf('\n  GOOD: NN stabilises with moderate deviation\n');
    else
        fprintf('\n  MARGINAL: NN stabilises but deviates significantly\n');
    end
else
    fprintf('\n  FAILED: NN diverged at step %d\n', final_nn);
end

%% ═══════════════════════════════════════════════════════════════════
%%  8. SAVE & PLOT
%% ═══════════════════════════════════════════════════════════════════
results = struct('t', (0:sim_steps)*dt_mpc, 'x0', x0, 'xEq', xEq, 'uEq', uEq, ...
    'x_ana', x_ana, 'u_ana', u_ana, 'stat_ana', stat_ana, 't_ana', t_ana, 'final_ana', final_ana, ...
    'x_nn', x_nn, 'u_nn', u_nn, 'stat_nn', stat_nn, 't_nn', t_nn, 'final_nn', final_nn);
save('nn_comparison_results_v3.mat', '-struct', 'results');

t_vec = (0:sim_steps)*dt_mpc;

figure('Position',[50 200 900 700]);
titles_l = {'x [mm]','y [mm]','z [mm]'}; titles_r = {'roll [°]','pitch [°]','yaw [°]'};
for j = 1:3
    subplot(3,2,2*j-1);
    plot(t_vec*1e3, x_ana(j,:)*1e3, 'b-', 'LineWidth',1.5); hold on;
    plot(t_vec*1e3, x_nn(j,:)*1e3, 'r--', 'LineWidth',1.5);
    if j==3, yline(xEq(3)*1e3,'k:'); end
    ylabel(titles_l{j}); grid on;
    if j==1, legend('Analytical','NN','Location','best'); end

    subplot(3,2,2*j);
    plot(t_vec*1e3, x_ana(j+3,:)*180/pi, 'b-', 'LineWidth',1.5); hold on;
    plot(t_vec*1e3, x_nn(j+3,:)*180/pi, 'r--', 'LineWidth',1.5);
    ylabel(titles_r{j}); grid on;
end
sgtitle('Analytical vs NN (linearity decomposition)');

figure('Position',[100 100 700 350]);
plot((1:final_ana)*dt_mpc*1e3, t_ana(1:final_ana)*1e3, 'b.', 'MarkerSize',8); hold on;
if final_nn>0
    plot((1:final_nn)*dt_mpc*1e3, t_nn(1:final_nn)*1e3, 'r.', 'MarkerSize',8);
end
yline(dt_mpc*1e3, 'k--');
ylabel('Solve time [ms]'); xlabel('Time [ms]');
legend('Analytical','NN','Real-time limit'); grid on;
title('OCP Solve Time');

fprintf('\nDone.\n');