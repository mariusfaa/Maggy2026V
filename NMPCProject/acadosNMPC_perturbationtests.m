%% --- PROJECT SETUP ---
clear all; clc;

acados_root = '/home/mariujf/acados';
project_root = '/home/mariujf/Maggy2026V/NMPCProject';

setenv('ACADOS_SOURCE_DIR', acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
setenv('ACADOS_INSTALL_DIR', acados_root); 

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external', 'jsonlab'));
addpath(fullfile(acados_root, 'external', 'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% --- MODEL SETUP ---
nx = 12;
nu = 4;
x = SX.sym('x', nx); 
u = SX.sym('u', nu);
xdot = SX.sym('xdot', nx);

parameters_maggy_V4;
correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

%% --- BUILD CASADI FUNCTION ---
f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
f_func = casadi.Function('f', {x, u}, {f_expl});

%% --- FIND TRUE EQUILIBRIUM USING OPTIMIZATION ---
fprintf('--- Searching for true equilibrium (z, u) ---\n');

z_var = SX.sym('z_eq');
u_var = SX.sym('u_eq', nu);

x_eq_sym = [0; 0; z_var; zeros(9,1)];
f_eq_sym = f_func(x_eq_sym, u_var);
accel = f_eq_sym(7:12);

cost = accel' * accel;

nlp = struct('x', [z_var; u_var], 'f', cost);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 3, 'max_iter', 1000)));

x0_guess = [0.030; zeros(nu,1)];
lbx = [0.015; -5*ones(nu,1)];
ubx = [0.060;  5*ones(nu,1)];

sol = solver_eq('x0', x0_guess, 'lbx', lbx, 'ubx', ubx);
opt = full(sol.x);

zEq_cas = opt(1);
uEq = opt(2:5);
xEq = [0; 0; zEq_cas; zeros(9,1)];

fprintf('Optimal equilibrium: z=%.6f m\n', zEq_cas);
fprintf('  u = [%.4f, %.4f, %.4f, %.4f] A\n', uEq(1), uEq(2), uEq(3), uEq(4));
f_check = full(f_func(xEq, uEq));
fprintf('  residual = %.4e\n', norm(f_check(7:12)));
fprintf('  ax=%.4e ay=%.4e az=%.4e\n', f_check(7), f_check(8), f_check(9));
fprintf('  ax=%.4e ay=%.4e az=%.4e\n', f_check(10), f_check(11), f_check(12));

%% --- OCP SETUP ---
ocp = AcadosOcp();
ocp.model.name = 'maglev_nmpc';
ocp.model.x = x;
ocp.model.u = u;
ocp.model.xdot = xdot;
ocp.model.f_impl_expr = xdot - f_expl;

% Solver options
N = 20;
Tf = 0.5;
dt_mpc = Tf / N;  % 0.025s

ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = Tf;
ocp.solver_options.integrator_type = 'IRK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps = 10;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';

% Cost
ocp.cost.cost_type   = 'NONLINEAR_LS';
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';

Q = diag([1e2, 1e2, 1e3, ...    % x, y, z
          1e4, 1e4, 1e1, ...    % roll, pitch, yaw
          1e1, 1e1, 1e2, ...    % vx, vy, vz
          1e4, 1e4, 1e0]);      % wx, wy, wz
R = eye(nu) * 0.1;

ocp.cost.W   = blkdiag(Q, R);
ocp.cost.W_0 = blkdiag(Q, R);
ocp.cost.W_e = Q;

ocp.model.cost_y_expr   = [x; u];
ocp.model.cost_y_expr_0 = [x; u];
ocp.model.cost_y_expr_e = x;

% References include equilibrium current
yref   = [xEq; uEq];
yref_e = xEq;

ocp.cost.yref   = yref;
ocp.cost.yref_0 = yref;
ocp.cost.yref_e = yref_e;

% Input constraints
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -5 * ones(nu,1); 
ocp.constraints.ubu =  5 * ones(nu,1);
ocp.constraints.x0 = xEq;

%% --- BUILD SOLVER ---
fprintf('\n--- Building acados solver ---\n');
ocp_solver = AcadosOcpSolver(ocp);

% Warm start at true equilibrium
for k = 0:N
    ocp_solver.set('x', xEq, k);
end
for k = 0:N-1
    ocp_solver.set('u', uEq, k);
end

%% --- HELPER: Run simulation with multiple RTI iterations ---
% SQP_RTI does only 1 QP iteration per solve() call. For states far from
% the warm-start, one iteration produces poor controls. Calling solve()
% multiple times (with the SAME x0) lets the optimizer converge before
% we apply the control. This is the standard "preparation + feedback"
% approach for real-time iteration schemes.

n_rti_iter = 5;  % Number of RTI iterations per control step

function [x_next, u_applied, status, diverged] = ...
    nmpc_step(ocp_solver, f_func, x_current, xEq, uEq, dt_mpc, n_rti, N)
    
    ocp_solver.set('constr_x0', x_current);
    
    % Multiple RTI iterations for better convergence
    for j = 1:n_rti
        ocp_solver.solve();
    end
    status = ocp_solver.get('status');
    u_applied = ocp_solver.get('u', 0);
    
    % Clamp control for safety
    u_applied = max(min(u_applied, 5), -5);
    
    % Simulate plant
    f_plant = @(t, xv) full(f_func(xv, u_applied));
    [~, x_traj] = ode15s(f_plant, [0, dt_mpc], x_current);
    x_next = x_traj(end,:)';
    
    % Divergence detection
    diverged = abs(x_next(3)) > 0.5 || ...      % z way out of range
               max(abs(x_next(4:6))) > pi || ... % angles > 180 deg
               any(isnan(x_next)) || any(isinf(x_next));
    
    % Warm-start next iteration: shift the trajectory
    for k = 0:N-2
        ocp_solver.set('x', ocp_solver.get('x', k+1), k);
        ocp_solver.set('u', ocp_solver.get('u', k+1), k);
    end
    ocp_solver.set('x', ocp_solver.get('x', N), N-1);
    ocp_solver.set('u', uEq, N-1);
    ocp_solver.set('x', xEq, N);
end

%% ===================================================================
%  TEST 1: Hold equilibrium (sanity check)
%  ===================================================================
fprintf('\n===== TEST 1: Starting at equilibrium =====\n');

nsim1 = 20;
x_current = xEq;
hist1_x = zeros(nx, nsim1+1);
hist1_u = zeros(nu, nsim1);
hist1_x(:,1) = x_current;

% Re-warm-start
for k = 0:N, ocp_solver.set('x', xEq, k); end
for k = 0:N-1, ocp_solver.set('u', uEq, k); end

for i = 1:nsim1
    [x_current, u_applied, status, diverged] = ...
        nmpc_step(ocp_solver, f_func, x_current, xEq, uEq, dt_mpc, n_rti_iter, N);
    
    fprintf('Step %3d: status=%d, |u-uEq|=%.4e, |u|=%.4e\n', ...
        i, status, norm(u_applied - uEq), norm(u_applied));
    
    hist1_x(:,i+1) = x_current;
    hist1_u(:,i) = u_applied;
    
    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
        break;
    end
end

fprintf('\nTest 1 diagnostics:\n');
fprintf('  max position drift: %.4e m\n', max(abs(hist1_x(1:3,1:i+1) - xEq(1:3)), [], 'all'));
fprintf('  max angle drift:    %.4e rad\n', max(abs(hist1_x(4:6,1:i+1)), [], 'all'));
fprintf('  max |u - uEq|:      %.4e A\n', max(vecnorm(hist1_u(:,1:i) - uEq)));

test1_ok = max(abs(hist1_x(1:3,1:i+1) - xEq(1:3)), [], 'all') < 0.01;
if test1_ok
    fprintf('*** TEST 1 PASSED ***\n');
else
    fprintf('*** TEST 1 FAILED ***\n');
end

%% ===================================================================
%  TEST 2: Graduated z perturbations (find basin of attraction)
%  ===================================================================
if test1_ok
    dz_tests = [0.0005, 0.001, 0.002, 0.005];  % 0.5mm, 1mm, 2mm, 5mm
    
    for ti = 1:length(dz_tests)
        dz = dz_tests(ti);
        fprintf('\n===== TEST 2.%d: dz = %.1f mm =====\n', ti, dz*1000);
        
        x_current = xEq + [0; 0; -dz; zeros(9,1)];
        
        % Re-warm-start at equilibrium
        for k = 0:N, ocp_solver.set('x', xEq, k); end
        for k = 0:N-1, ocp_solver.set('u', uEq, k); end
        
        nsim2 = 100;
        hist2_x = zeros(nx, nsim2+1);
        hist2_u = zeros(nu, nsim2);
        hist2_x(:,1) = x_current;
        n_fail2 = 0;
        last_step = nsim2;
        
        for i = 1:nsim2
            [x_current, u_applied, status, diverged] = ...
                nmpc_step(ocp_solver, f_func, x_current, xEq, uEq, dt_mpc, n_rti_iter, N);
            
            if status ~= 0 && status ~= 2
                n_fail2 = n_fail2 + 1;
            end
            
            hist2_x(:,i+1) = x_current;
            hist2_u(:,i) = u_applied;
            
            % Print first 10 steps and every 10th after
            if i <= 10 || mod(i,10) == 0
                fprintf('Step %3d: status=%d, z=%.6f, |angles|=%.4e, |u|=%.4e\n', ...
                    i, status, x_current(3), norm(x_current(4:6)), norm(u_applied));
            end
            
            if diverged
                fprintf('  *** DIVERGED at step %d ***\n', i);
                last_step = i;
                break;
            end
        end
        
        z_err = hist2_x(3, last_step+1) - zEq_cas;
        fprintf('Test 2.%d summary: %d failures, z_error=%.4e m (%.2f mm)\n', ...
            ti, n_fail2, z_err, z_err*1000);
        
        % Store the best result for plotting
        if ti == length(dz_tests) || (exist('best2','var') == 0)
            best2.hist_x = hist2_x(:, 1:last_step+1);
            best2.hist_u = hist2_u(:, 1:last_step);
            best2.nsim = last_step;
            best2.dz = dz;
        end
        if ~diverged
            best2.hist_x = hist2_x(:, 1:last_step+1);
            best2.hist_u = hist2_u(:, 1:last_step);
            best2.nsim = last_step;
            best2.dz = dz;
        end
    end
end

%% ===================================================================
%  TEST 3: Graduated tilt perturbations (find angular basin of attraction)
%  ===================================================================
if test1_ok
    roll_tests = [pi/200, pi/100, pi/60, pi/40];  % ~0.9°, 1.8°, 3°, 4.5°
    
    for ti = 1:length(roll_tests)
        roll0 = roll_tests(ti);
        fprintf('\n===== TEST 3.%d: 2mm + roll = %.2f deg =====\n', ti, rad2deg(roll0));
        
        x_current = xEq + [0; 0; -0.002; roll0; 0; 0; zeros(6,1)];
        
        % Re-warm-start
        for k = 0:N, ocp_solver.set('x', xEq, k); end
        for k = 0:N-1, ocp_solver.set('u', uEq, k); end
        
        nsim3 = 200;
        hist3_x = zeros(nx, nsim3+1);
        hist3_u = zeros(nu, nsim3);
        hist3_x(:,1) = x_current;
        n_fail3 = 0;
        last_step3 = nsim3;
        
        for i = 1:nsim3
            [x_current, u_applied, status, diverged] = ...
                nmpc_step(ocp_solver, f_func, x_current, xEq, uEq, dt_mpc, n_rti_iter, N);
            
            if status ~= 0 && status ~= 2
                n_fail3 = n_fail3 + 1;
            end
            
            hist3_x(:,i+1) = x_current;
            hist3_u(:,i) = u_applied;
            
            if i <= 10 || mod(i,10) == 0
                fprintf('Step %3d: status=%d, z=%.6f, roll=%.4f deg, |u|=%.4e\n', ...
                    i, status, x_current(3), rad2deg(x_current(4)), norm(u_applied));
            end
            
            if diverged
                fprintf('  *** DIVERGED at step %d ***\n', i);
                last_step3 = i;
                break;
            end
        end
        
        fprintf('Test 3.%d summary: %d failures, z_err=%.4e, roll_final=%.4f deg\n', ...
            ti, n_fail3, hist3_x(3,last_step3+1)-zEq_cas, rad2deg(hist3_x(4,last_step3+1)));
        
        % Store last completed test for plotting
        best3.hist_x = hist3_x(:, 1:last_step3+1);
        best3.hist_u = hist3_u(:, 1:last_step3);
        best3.nsim = last_step3;
        best3.roll0 = roll0;
        best3.diverged = diverged;
    end
end

%% ===================================================================
%  VISUALIZATION
%  ===================================================================
figure('Name', 'MagLev NMPC Results', 'Color', 'w', ...
       'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.85]);

% --- Best Test 2 result ---
if exist('best2','var')
    ns = best2.nsim;
    t2 = (0:ns)*dt_mpc;
    t2u = (0:ns-1)*dt_mpc;
    
    subplot(3,3,1);
    plot(t2, best2.hist_x(1:3,:), 'LineWidth', 1.5); hold on;
    yline(zEq_cas, 'k--', 'z_{eq}');
    title(sprintf('Test 2 (dz=%.1fmm): Position', best2.dz*1000));
    xlabel('t [s]'); ylabel('[m]');
    legend('x','y','z'); grid on;
    
    subplot(3,3,2);
    plot(t2, rad2deg(best2.hist_x(4:6,:)), 'LineWidth', 1.5);
    title('Test 2: Orientation'); xlabel('t [s]'); ylabel('[deg]');
    legend('\alpha','\beta','\gamma'); grid on;
    
    subplot(3,3,3);
    stairs(t2u, best2.hist_u', 'LineWidth', 1.5); hold on;
    for j=1:nu
        yline(uEq(j), 'k--');
    end
    title('Test 2: Control'); xlabel('t [s]'); ylabel('[A]');
    legend('I_1','I_2','I_3','I_4'); grid on;
end

% --- Test 3 ---
if exist('best3','var')
    ns3 = best3.nsim;
    t3 = (0:ns3)*dt_mpc;
    t3u = (0:ns3-1)*dt_mpc;
    
    subplot(3,3,4);
    plot(t3, best3.hist_x(1:3, 1:ns3+1), 'LineWidth', 1.5); hold on;
    yline(zEq_cas, 'k--', 'z_{eq}');
    title('Test 3: Position'); xlabel('t [s]'); ylabel('[m]');
    legend('x','y','z'); grid on;
    
    subplot(3,3,5);
    plot(t3, rad2deg(best3.hist_x(4:6, 1:ns3+1)), 'LineWidth', 1.5);
    title('Test 3: Orientation'); xlabel('t [s]'); ylabel('[deg]');
    legend('\alpha','\beta','\gamma'); grid on;
    
    subplot(3,3,6);
    stairs(t3u, best3.hist_u(:, 1:ns3)', 'LineWidth', 1.5); hold on;
    for j=1:nu
        yline(uEq(j), 'k--');
    end
    title('Test 3: Control'); xlabel('t [s]'); ylabel('[A]');
    legend('I_1','I_2','I_3','I_4'); grid on;
    
    subplot(3,3,7);
    plot(t3, best3.hist_x(7:9, 1:ns3+1), 'LineWidth', 1.5);
    title('Test 3: Linear Vel'); xlabel('t [s]'); ylabel('[m/s]');
    legend('v_x','v_y','v_z'); grid on;
    
    subplot(3,3,8);
    plot(t3, rad2deg(best3.hist_x(10:12, 1:ns3+1)), 'LineWidth', 1.5);
    title('Test 3: Angular Vel'); xlabel('t [s]'); ylabel('[deg/s]');
    legend('\omega_x','\omega_y','\omega_z'); grid on;
    
    subplot(3,3,9);
    plot3(best3.hist_x(1, 1:ns3+1), best3.hist_x(2, 1:ns3+1), best3.hist_x(3, 1:ns3+1), ...
        'b', 'LineWidth', 2);
    hold on; plot3(0,0,zEq_cas,'rx','MarkerSize',10,'LineWidth',2);
    title('Test 3: 3D Trajectory'); xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on; axis equal;
end

%% --- Save results ---
save('nmpc_results_full.mat', ...
    'hist1_x', 'hist1_u', ...
    'best2', ...
    'best3', ...
    'dt_mpc', 'xEq', 'uEq', 'zEq_cas', ...
    'n_rti_iter');
fprintf('\nResults saved to nmpc_results_full.mat\n');