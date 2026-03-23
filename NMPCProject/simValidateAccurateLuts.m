%% simValidateAccurateLuts — Validate CasADi Accurate+LUT model
%
% Two validations:
%   Part 1: Force/torque comparison at specific states (no integration)
%   Part 2: Closed-loop NMPC trajectory (stable comparison)

simSetup;
import casadi.*

nx_red = 10;
x0_red = x0([1:5, 7:11]);
xEq_red = xEq([1:5, 7:11]);

%% ================================================================
%% Part 1: Force/torque comparison at multiple test points
%% ================================================================
fprintf('\n========================================\n');
fprintf('Part 1: Force/Torque Spot Check\n');
fprintf('========================================\n');

params_fast = load_params(MaglevModel.Fast);
m_val = params_fast.magnet.m;
I_val = params_fast.magnet.I;
g_val = params_fast.physical.g;

% Build CasADi functions for force evaluation
params_fast_nolut = load_params(MaglevModel.Fast);
params_fast_nolut.lut_opts.enabled = false;

x_sym = casadi.MX.sym('x', nx_red);
u_sym = casadi.MX.sym('u', nu);

% Fast analytical CasADi
[fx_f,fy_f,fz_f,tx_f,ty_f,~] = computeForceAndTorque_casadi(x_sym, u_sym, params_fast_nolut, MaglevModel.Fast);
f_casadi_fast = casadi.Function('f_fast', {x_sym, u_sym}, {fx_f,fy_f,fz_f,tx_f,ty_f});

% Accurate+LUT CasADi
params_acc_lut = load_params(MaglevModel.Accurate);
params_acc_lut.luts = buildLuts(params_acc_lut, MaglevModel.Accurate);
[fx_a,fy_a,fz_a,tx_a,ty_a,~] = computeForceAndTorque_casadi(x_sym, u_sym, params_acc_lut, MaglevModel.Accurate);
f_casadi_acc = casadi.Function('f_acc', {x_sym, u_sym}, {fx_a,fy_a,fz_a,tx_a,ty_a});

% Test points: equilibrium + various perturbations
test_states = {
    'Equilibrium',                xEq_red,                                            zeros(4,1);
    'z+1mm',                      xEq_red + [0;0;0.001;0;0;0;0;0;0;0],               zeros(4,1);
    'z-1mm',                      xEq_red + [0;0;-0.001;0;0;0;0;0;0;0],              zeros(4,1);
    'y+2mm',                      xEq_red + [0;0.002;0;0;0;0;0;0;0;0],               zeros(4,1);
    'roll+5deg',                  xEq_red + [0;0;0;5*pi/180;0;0;0;0;0;0],            zeros(4,1);
    'Eq + u=[1,0,0,0]',          xEq_red,                                            [1;0;0;0];
    'Eq + u=[0.5,-0.5,0.5,-0.5]',xEq_red,                                            [0.5;-0.5;0.5;-0.5];
    'z+1mm + u=[1,1,1,1]',       xEq_red + [0;0;0.001;0;0;0;0;0;0;0],               ones(4,1);
};

nTests = size(test_states, 1);
F_matlab = zeros(5, nTests);
F_casadi_f = zeros(5, nTests);
F_casadi_a = zeros(5, nTests);

fprintf('\n%-25s  %8s %8s %8s %8s %8s    %8s %8s\n', ...
    'Test point', 'fx', 'fy', 'fz', 'tx', 'ty', 'err_Fast', 'err_Acc');
fprintf('%s\n', repmat('-', 1, 100));

for i = 1:nTests
    label = test_states{i,1};
    x_test = test_states{i,2};
    u_test = test_states{i,3};

    % MATLAB reference (12-state, Fast model)
    x12 = [x_test(1:5); 0; x_test(6:10); 0];
    [fx_m,fy_m,fz_m,tx_m,ty_m,~] = computeForceAndTorque(x12, u_test, params_fast, MaglevModel.Fast);
    F_matlab(:,i) = [fx_m; fy_m; fz_m; tx_m; ty_m];

    % CasADi Fast
    [r1,r2,r3,r4,r5] = f_casadi_fast(x_test, u_test);
    F_casadi_f(:,i) = full([r1;r2;r3;r4;r5]);

    % CasADi Accurate+LUT
    [r1,r2,r3,r4,r5] = f_casadi_acc(x_test, u_test);
    F_casadi_a(:,i) = full([r1;r2;r3;r4;r5]);

    err_f = norm(F_matlab(:,i) - F_casadi_f(:,i));
    err_a = norm(F_matlab(:,i) - F_casadi_a(:,i));

    fprintf('%-25s  %8.4f %8.4f %8.4f %8.6f %8.6f    %8.2e %8.2e\n', ...
        label, F_matlab(1,i), F_matlab(2,i), F_matlab(3,i), ...
        F_matlab(4,i), F_matlab(5,i), err_f, err_a);
end

fprintf('\nCasADi Fast vs MATLAB Fast: max force error = %.2e N\n', ...
    max(abs(F_matlab - F_casadi_f), [], 'all'));
fprintf('CasADi Acc+LUT vs MATLAB Fast: max force error = %.2e N\n', ...
    max(abs(F_matlab - F_casadi_a), [], 'all'));

%% ================================================================
%% Part 2: Closed-loop NMPC comparison
%% ================================================================
fprintf('\n========================================\n');
fprintf('Part 2: Closed-Loop NMPC (Accurate+LUT)\n');
fprintf('========================================\n');

% Build MPC model
clear model ocp_solver sim_solver;
model = get_maggy_model(MaglevModel.Accurate, use_luts=true);

% Recompute equilibrium for this model
params_acc = load_params(MaglevModel.Accurate);
% For the CasADi model the surface integration matches Fast model
% so use Fast equilibrium for consistency
[zEq_fast, ~, ~, ~] = computeSystemEquilibria(params_fast, MaglevModel.Fast);
xEq_mpc = [0; 0; zEq_fast(1); zeros(7,1)];
uEq_mpc = zeros(nu, 1);

x0_mpc = [0; 0; zEq_fast(1); zeros(7,1)] + [0; 0.001; 0.001; 0; 0; zeros(5,1)];

umax = 4;
dt_mpc = 0.001;
N_horizon = 20;
Tf = dt_mpc * N_horizon;

%% OCP setup
ocp = AcadosOcp();
ocp.model = model;

ocp.solver_options.N_horizon = N_horizon;
ocp.solver_options.tf = Tf;
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps = 1;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.nlp_solver_max_iter = 200;
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.globalization = 'MERIT_BACKTRACKING';
ocp.solver_options.ext_fun_compile_flags = '-O2';

Q = diag([1e4,1e4,1e6,1e3,1e3,1e2,1e2,1e3,1e2,1e2]);
R = eye(nu) * 1e0;

ocp.cost.cost_type   = 'LINEAR_LS';
ocp.cost.cost_type_0 = 'LINEAR_LS';
ocp.cost.cost_type_e = 'LINEAR_LS';

ocp.cost.Vx   = [eye(nx_red); zeros(nu, nx_red)];
ocp.cost.Vu   = [zeros(nx_red, nu); eye(nu)];
ocp.cost.Vx_0 = ocp.cost.Vx;
ocp.cost.Vu_0 = ocp.cost.Vu;
ocp.cost.Vx_e = eye(nx_red);

ocp.cost.W   = blkdiag(Q, R);
ocp.cost.W_0 = blkdiag(Q, R);
ocp.cost.W_e = 10 * Q;

ocp.cost.yref   = [xEq_mpc; uEq_mpc];
ocp.cost.yref_0 = [xEq_mpc; uEq_mpc];
ocp.cost.yref_e = xEq_mpc;

ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu   = -umax * ones(nu, 1);
ocp.constraints.ubu   =  umax * ones(nu, 1);
ocp.constraints.x0    = x0_mpc;

ocp_solver = AcadosOcpSolver(ocp);

% Warm-start
for k = 0:N_horizon
    ocp_solver.set('x', xEq_mpc, k);
end
for k = 0:N_horizon-1
    ocp_solver.set('u', uEq_mpc, k);
end

% Sim solver (plant)
sim = AcadosSim();
sim.model = model;
sim.solver_options.Tsim = dt;
sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages = 4;
sim.solver_options.num_steps = 1;
sim_solver = AcadosSimSolver(sim);

%% Run NMPC loop
t_sim = 0:dt:0.5;
N_sim = numel(t_sim);
N_mpc = floor(N_sim / 1);  % n_sub=1 (dt_mpc == dt)

x_cl = zeros(nx_red, N_sim);
u_cl = zeros(nu, N_sim);
t_mpc_log = zeros(1, N_mpc);

x_cur = x0_mpc;
u_cur = uEq_mpc;
diverged = false;

fprintf('\nRunning NMPC (Accurate+LUT)...\n');

for k = 1:N_mpc
    if k > N_sim, break; end

    x_cl(:, k) = x_cur;

    ocp_solver.set('constr_x0', x_cur);
    ocp_solver.solve();
    t_mpc_log(k) = ocp_solver.get('time_tot');

    u_cur = ocp_solver.get('u', 0);
    u_cl(:, k) = u_cur;

    % Warm-shift
    for i = 0:N_horizon-2
        ocp_solver.set('x', ocp_solver.get('x', i+1), i);
        ocp_solver.set('u', ocp_solver.get('u', i+1), i);
    end
    ocp_solver.set('x', ocp_solver.get('x', N_horizon), N_horizon);
    ocp_solver.set('u', ocp_solver.get('u', N_horizon-1), N_horizon-1);

    x_cur = sim_solver.simulate(x_cur, u_cur);

    diverged = abs(x_cur(3)) > 0.5 || max(abs(x_cur(4:5))) > pi || any(isnan(x_cur));
    if diverged
        fprintf('  DIVERGED at step %d\n', k);
        x_cl = x_cl(:, 1:k); u_cl = u_cl(:, 1:k);
        t_sim = t_sim(1:k); t_mpc_log = t_mpc_log(1:k);
        break;
    end
end

if ~diverged
    fprintf('SUCCESS: Accurate+LUT NMPC stabilized the system.\n');
end

fprintf('MPC solve: median=%.0f us, mean=%.0f us\n', ...
    median(t_mpc_log)*1e6, mean(t_mpc_log)*1e6);
fprintf('Final: z=%.4f mm (eq=%.4f mm), |pos err|=%.4f mm\n', ...
    x_cur(3)*1e3, xEq_mpc(3)*1e3, norm(x_cur(1:3)-xEq_mpc(1:3))*1e3);

%% Save closed-loop results
sim_data = struct();
sim_data.t = t_sim; sim_data.x = x_cl; sim_data.u = u_cl;
sim_data.xEq = xEq_mpc; sim_data.uEq = uEq_mpc; sim_data.dt = dt;
sim_data.t_mpc = t_mpc_log;
save('results_acados_mpc_accurate_lut.mat', '-struct', 'sim_data');

%% ================================================================
%% Plot
%% ================================================================
figure(1); clf;
set(gcf, 'Name', 'Accurate+LUT Validation', 'Position', [50 50 1600 800]);

% --- Part 1 subplot: Force comparison bar chart ---
subplot(2,4,1); hold on; grid on; box on;
err_fast_pct = abs(F_matlab - F_casadi_f) ./ max(abs(F_matlab), 1e-10) * 100;
err_acc_pct  = abs(F_matlab - F_casadi_a) ./ max(abs(F_matlab), 1e-10) * 100;
bar([max(err_fast_pct, [], 2), max(err_acc_pct, [], 2)]);
set(gca, 'XTickLabel', {'fx','fy','fz','tx','ty'});
ylabel('Max relative error (%)');
title('Force/Torque Error');
legend({'CasADi Fast','Acc+LUT'}, 'Location','best', 'FontSize', 7);

% --- Part 2 subplots: Closed-loop NMPC ---
labels_pos = {'x','y','z'};
colors_p = [1 0 0; 0 0.6 0; 0 0 1];

subplot(2,4,2); hold on; grid on; box on;
for i = 1:3
    plot(t_sim, x_cl(i,:)*1e3, 'Color', colors_p(i,:), 'LineWidth', 1.5, 'DisplayName', labels_pos{i});
end
yline(xEq_mpc(3)*1e3, 'k:', 'HandleVisibility', 'off');
ylabel('mm'); title('Position'); legend('Location','best');

subplot(2,4,3); hold on; grid on; box on;
plot(t_sim, x_cl(4,:)*180/pi, 'r-', 'LineWidth', 1.5, 'DisplayName', 'roll');
plot(t_sim, x_cl(5,:)*180/pi, 'g-', 'LineWidth', 1.5, 'DisplayName', 'pitch');
ylabel('deg'); title('Orientation'); legend('Location','best');

subplot(2,4,4); hold on; grid on; box on;
colors_u = [1 0 0; 0 0.6 0; 0 0 1; 0.5 0 0.5];
for i = 1:4
    plot(t_sim, u_cl(i,:), 'Color', colors_u(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('u%d',i));
end
ylabel('A'); title('Inputs'); legend('Location','best');

% Velocities
subplot(2,4,5); hold on; grid on; box on;
for i = 1:3
    plot(t_sim, x_cl(5+i,:), 'Color', colors_p(i,:), 'LineWidth', 1.5);
end
ylabel('m/s'); title('Linear Velocity'); xlabel('Time (s)');

subplot(2,4,6); hold on; grid on; box on;
plot(t_sim, x_cl(9,:), 'r-', 'LineWidth', 1.5);
plot(t_sim, x_cl(10,:), 'g-', 'LineWidth', 1.5);
ylabel('rad/s'); title('Angular Velocity'); xlabel('Time (s)');

% MPC timing
subplot(2,4,7); hold on; grid on; box on;
plot(t_sim, t_mpc_log*1e6, '.', 'MarkerSize', 3);
yline(median(t_mpc_log)*1e6, 'r-', 'LineWidth', 1.5);
ylabel('\mus'); title(sprintf('MPC Solve Time (med=%.0f\\mus)', median(t_mpc_log)*1e6));
xlabel('Time (s)');

% Summary text
subplot(2,4,8); axis off;
text(0.1, 0.9, 'Accurate+LUT NMPC', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('Model: Accurate + LUT'), 'FontSize', 10);
text(0.1, 0.6, sprintf('Horizon: N=%d, dt=%.0fms', N_horizon, dt_mpc*1e3), 'FontSize', 10);
text(0.1, 0.45, sprintf('umax: %.0f A', umax), 'FontSize', 10);
text(0.1, 0.3, sprintf('MPC solve: %.0f us (median)', median(t_mpc_log)*1e6), 'FontSize', 10);
text(0.1, 0.15, sprintf('Final |pos err|: %.3f mm', norm(x_cur(1:3)-xEq_mpc(1:3))*1e3), 'FontSize', 10);
if ~diverged
    text(0.1, 0.0, 'STATUS: STABLE', 'FontSize', 12, 'Color', [0 0.6 0], 'FontWeight', 'bold');
else
    text(0.1, 0.0, 'STATUS: DIVERGED', 'FontSize', 12, 'Color', [1 0 0], 'FontWeight', 'bold');
end

sgtitle('Accurate+LUT Model Validation', 'FontSize', 14, 'FontWeight', 'bold');

saveas(figure(1), 'figures/validate_accurate_lut.png');
fprintf('\nFigure saved to figures/validate_accurate_lut.png\n');
