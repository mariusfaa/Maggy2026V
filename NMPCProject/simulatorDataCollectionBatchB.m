%% experimentB_integration.m
% =========================================================================
%  Experiment B: Integration Complexity Reduction
%
%  Iterates through all sub-experiments (B1–B6) testing different
%  integrator types, stage counts, and step counts for the OCP solver.
%
%  The PLANT simulator is kept at IRK(4,10) throughout — only the OCP
%  integrator changes.  Each sub-experiment gets a unique acados model name
%  to avoid C-code / MEX / shared-library collisions:
%
%     OCP model:   'mlev_B1', 'mlev_B2', ...   (unique per experiment)
%     Plant sim:   'mlev_plant'                 (shared, built once)
%
%  Sub-experiments:
%     B1: IRK  stages=4  steps=5   (half the steps of baseline)
%     B2: IRK  stages=2  steps=5
%     B3: IRK  stages=2  steps=3
%     B4: ERK  stages=4  steps=5
%     B5: ERK  stages=4  steps=3
%     B6: ERK  stages=4  steps=1
%
%  Author: Marius Jullum Faanes
%  Date:   2026-03-29
% =========================================================================

%% === CLEAN SLATE ===
clearvars; clc;

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

%% === COMMON MODEL SETUP (done once) ===
fprintf('====================================================================\n');
fprintf('  Experiment B — Integration Complexity Reduction (batch)\n');
fprintf('====================================================================\n\n');

nx_full = 12;
nx      = 10;
nu      = 4;

x_full  = SX.sym('x_full', nx_full);
u_sym   = SX.sym('u', nu);

parameters_maggy_V4;
correctionFactorFast   = computeSolenoidRadiusCorrectionFactor(params, 'fast');
paramsFast             = params;
paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

f_expl_full = maglevSystemDynamicsCasADi(x_full, u_sym, paramsFast);
f_func_full = casadi.Function('f_full', {x_full, u_sym}, {f_expl_full});

% --- Equilibrium ---
fprintf('--- Finding equilibrium ---\n');
z_var       = SX.sym('z_eq');
x_eq_sym    = [0; 0; z_var; zeros(9,1)];
accel       = f_func_full(x_eq_sym, zeros(nu,1));
fz_residual = accel(9);

nlp       = struct('x', z_var, 'f', fz_residual^2);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 0)));
sol       = solver_eq('x0', 0.030, 'lbx', 0.015, 'ubx', 0.060);
zEq_cas   = full(sol.x);
uEq       = zeros(nu, 1);
xEq       = [0; 0; zEq_cas; zeros(7,1)];
fprintf('  Equilibrium z = %.6f m\n\n', zEq_cas);

% --- Reduced-order dynamics (symbolic, reused by all solvers) ---
x_r    = SX.sym('x_r', nx);
xdot_r = SX.sym('xdot_r', nx);

x_full_from_r = [x_r(1:5); 0; x_r(6:8); x_r(9:10); 0];
dx_full       = f_func_full(x_full_from_r, u_sym);
f_expl_r      = [dx_full(1:5); dx_full(7:11)];

% --- Cost matrices (same for all) ---
N      = 20;
Tf     = 0.2;
dt_mpc = Tf / N;

Q = diag([1e2, 1e2, 1e3, 1e3, 1e3, 1e1, 1e1, 1e1, 1e1, 1e1]);
R = eye(nu) * 1.0;
n_sbx = 5;

% --- Perturbation (same for all) ---
dx_dir = [0.005; -0.008; 0.010; 0.15; -0.10; ...
          0.01;  -0.01;  0.0;   0.2;  -0.3];
perturbation_scale = 0.05;
x_init = xEq + perturbation_scale * dx_dir;

sim_steps = 200;

%% === BUILD PLANT SIMULATOR (once, high accuracy, never changes) ===
fprintf('--- Building plant simulator (IRK, 4 stages, 10 steps) ---\n');
plant_sim = AcadosSim();
plant_sim.model.name        = 'mlev_plant';
plant_sim.model.x           = x_r;
plant_sim.model.u           = u_sym;
plant_sim.model.xdot        = xdot_r;
plant_sim.model.f_impl_expr = xdot_r - f_expl_r;
plant_sim.solver_options.Tsim            = dt_mpc;
plant_sim.solver_options.integrator_type = 'IRK';
plant_sim.solver_options.num_stages      = 4;
plant_sim.solver_options.num_steps       = 10;
plant_solver = AcadosSimSolver(plant_sim);
fprintf('  Plant simulator ready.\n\n');

%% === DEFINE SUB-EXPERIMENTS ===
%  Each row: { id, integrator_type, num_stages, num_steps, description }
experiments = {
    'B1', 'IRK', 4, 5,  'IRK stages=4 steps=5  (half steps)';
    'B2', 'IRK', 2, 5,  'IRK stages=2 steps=5';
    'B3', 'IRK', 2, 3,  'IRK stages=2 steps=3';
    'B4', 'ERK', 4, 5,  'ERK stages=4 steps=5';
    'B5', 'ERK', 4, 3,  'ERK stages=4 steps=3';
    'B6', 'ERK', 4, 1,  'ERK stages=4 steps=1  (single step)';
};

n_exp = size(experiments, 1);

%% === MAIN LOOP OVER SUB-EXPERIMENTS ===
for e = 1:n_exp
    exp_id     = experiments{e,1};
    integ_type = experiments{e,2};
    n_stages   = experiments{e,3};
    n_steps    = experiments{e,4};
    exp_desc   = experiments{e,5};
    exp_name   = sprintf('B_%s', exp_id);

    % Unique model name → unique C code, MEX, shared libs
    ocp_model_name = sprintf('mlev_%s', exp_id);

    fprintf('\n############################################################\n');
    fprintf('  SUB-EXPERIMENT %s  (%d/%d)\n', exp_id, e, n_exp);
    fprintf('  %s\n', exp_desc);
    fprintf('  OCP model name: %s\n', ocp_model_name);
    fprintf('############################################################\n\n');

    % =================================================================
    %  BUILD OCP SOLVER for this sub-experiment
    % =================================================================
    ocp = AcadosOcp();
    ocp.model.name = ocp_model_name;
    ocp.model.x    = x_r;
    ocp.model.u    = u_sym;
    ocp.model.xdot = xdot_r;

    % Set BOTH explicit and implicit dynamics expressions.
    % acados uses the one matching integrator_type and ignores the other.
    ocp.model.f_impl_expr = xdot_r - f_expl_r;   % used by IRK
    ocp.model.f_expl_expr = f_expl_r;             % used by ERK

    % Solver options
    ocp.solver_options.N_horizon             = N;
    ocp.solver_options.tf                    = Tf;
    ocp.solver_options.integrator_type       = integ_type;
    ocp.solver_options.sim_method_num_stages = n_stages;
    ocp.solver_options.sim_method_num_steps  = n_steps;
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

    % Cost (identical to baseline)
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

    % Constraints (identical to baseline)
    ocp.constraints.idxbu  = 0:nu-1;
    ocp.constraints.lbu    = -1 * ones(nu,1);
    ocp.constraints.ubu    =  1 * ones(nu,1);

    ocp.constraints.idxbx  = [0, 1, 2, 3, 4];
    ocp.constraints.lbx    = [-0.025; -0.025; 0.015; -0.35; -0.35];
    ocp.constraints.ubx    = [ 0.025;  0.025; 0.055;  0.35;  0.35];
    ocp.constraints.idxsbx = 0:4;

    ocp.cost.Zl = 1e3 * ones(n_sbx, 1);
    ocp.cost.Zu = 1e3 * ones(n_sbx, 1);
    ocp.cost.zl = 1e2 * ones(n_sbx, 1);
    ocp.cost.zu = 1e2 * ones(n_sbx, 1);

    ocp.constraints.x0 = xEq;

    % Capture config before acados modifies the struct
    cfg_integ  = integ_type;
    cfg_stages = n_stages;
    cfg_steps  = n_steps;

    % Build
    fprintf('  Building OCP solver (%s, %d stages, %d steps)...\n', ...
        integ_type, n_stages, n_steps);
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('  OCP solver ready.\n');

    % =================================================================
    %  INITIALISE AND RUN SIMULATION
    % =================================================================
    x_current = x_init;

    % Trajectory initialisation
    for k = 0:N
        alpha_k = k / N;
        ocp_solver.set('x', (1 - alpha_k) * x_current + alpha_k * xEq, k);
    end
    for k = 0:N-1
        ocp_solver.set('u', uEq, k);
    end

    % Pre-allocate
    plot_x         = zeros(nx, sim_steps+1);
    plot_u         = zeros(nu, sim_steps);
    plot_stat      = zeros(1, sim_steps);
    plot_sqp       = zeros(1, sim_steps);
    plot_qp_iter   = zeros(1, sim_steps);
    plot_time_tot  = zeros(1, sim_steps);
    plot_time_lin  = zeros(1, sim_steps);
    plot_time_sim  = zeros(1, sim_steps);
    plot_time_sim_ad = zeros(1, sim_steps);
    plot_time_sim_la = zeros(1, sim_steps);
    plot_time_qp   = zeros(1, sim_steps);
    plot_time_reg  = zeros(1, sim_steps);
    plot_time_glob = zeros(1, sim_steps);
    plot_err_norm  = zeros(1, sim_steps+1);
    plot_err_pos   = zeros(1, sim_steps+1);
    plot_err_ang   = zeros(1, sim_steps+1);
    plot_max_slack = zeros(1, sim_steps);
    plot_x(:,1)       = x_current;
    plot_err_norm(1)   = norm(x_current - xEq);
    plot_err_pos(1)    = norm(x_current(1:3) - xEq(1:3)) * 1e3;
    plot_err_ang(1)    = rad2deg(norm(x_current(4:5) - xEq(4:5)));

    fprintf('  Running simulation (%d steps)...\n', sim_steps);

    for i = 1:sim_steps
        % --- Solve OCP ---
        ocp_solver.set('constr_x0', x_current);
        ocp_solver.solve();

        status    = ocp_solver.get('status');
        sqp_iter  = ocp_solver.get('sqp_iter');
        u_applied = ocp_solver.get('u', 0);
        u_applied = max(min(u_applied, 1), -1);

        t_tot    = ocp_solver.get('time_tot');
        t_lin    = ocp_solver.get('time_lin');
        t_qp_sol = ocp_solver.get('time_qp_sol');
        t_reg    = ocp_solver.get('time_reg');
        try t_sim    = ocp_solver.get('time_sim');    catch; t_sim    = NaN; end
        try t_sim_ad = ocp_solver.get('time_sim_ad'); catch; t_sim_ad = NaN; end
        try t_sim_la = ocp_solver.get('time_sim_la'); catch; t_sim_la = NaN; end
        try t_glob   = ocp_solver.get('time_glob');   catch; t_glob   = NaN; end
        try
            qp_raw = ocp_solver.get('qp_iter');
            qp_it  = sum(qp_raw(:));
        catch; qp_it = NaN; end

        % Soft constraint slacks
        all_slacks = 0;
        for k = 0:N
            try
                sl_k = ocp_solver.get('sl', k);
                su_k = ocp_solver.get('su', k);
                all_slacks = [all_slacks; abs(sl_k(:)); abs(su_k(:))]; %#ok<AGROW>
            catch; end
        end

        % --- Simulate plant (high-accuracy, shared solver) ---
        plant_solver.set('x', x_current);
        plant_solver.set('u', u_applied);
        plant_solver.solve();
        x_next = plant_solver.get('xn');

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

        % --- Store ---
        plot_x(:,i+1)        = x_next;
        plot_u(:,i)          = u_applied;
        plot_stat(i)         = status;
        plot_sqp(i)          = sqp_iter;
        plot_qp_iter(i)      = qp_it;
        plot_time_tot(i)     = t_tot;
        plot_time_lin(i)     = t_lin;
        plot_time_sim(i)     = t_sim;
        plot_time_sim_ad(i)  = t_sim_ad;
        plot_time_sim_la(i)  = t_sim_la;
        plot_time_qp(i)      = t_qp_sol;
        plot_time_reg(i)     = t_reg;
        plot_time_glob(i)    = t_glob;
        plot_err_norm(i+1)   = norm(x_next - xEq);
        plot_err_pos(i+1)    = norm(x_next(1:3) - xEq(1:3)) * 1e3;
        plot_err_ang(i+1)    = rad2deg(norm(x_next(4:5) - xEq(4:5)));
        plot_max_slack(i)    = max(all_slacks);

        % --- Print selected steps ---
        if i <= 3 || i == sim_steps
            fprintf('    Step %3d: st=%d, sqp=%2d, t=%6.1fms, z=%.4fmm, |e|=%.2e\n', ...
                i, status, sqp_iter, t_tot*1e3, x_next(3)*1e3, norm(x_next-xEq));
        end

        % --- Divergence check ---
        if abs(x_next(3)) > 0.5 || max(abs(x_next(4:5))) > pi || ...
                any(isnan(x_next)) || any(isinf(x_next))
            fprintf('    *** DIVERGED at step %d ***\n', i);
            break;
        end

        x_current = x_next;
    end
    n_actual = min(i, sim_steps);

    % =================================================================
    %  COMPUTE SUMMARY STATISTICS
    % =================================================================
    tv  = plot_time_tot(1:n_actual);
    sv  = plot_sqp(1:n_actual);
    qv  = plot_qp_iter(1:n_actual);
    stv = plot_stat(1:n_actual);

    initial_err = plot_err_norm(1);
    thresholds  = [0.01, 0.02, 0.05];
    settling    = NaN(size(thresholds));
    for s = 1:length(thresholds)
        thr = thresholds(s) * initial_err;
        for j = 1:n_actual+1
            if all(plot_err_norm(j:n_actual+1) <= thr)
                settling(s) = (j-1) * dt_mpc;
                break;
            end
        end
    end

    % Print quick summary for this experiment
    fprintf('\n  --- %s Summary ---\n', exp_id);
    fprintf('    Steps:       %d / %d\n', n_actual, sim_steps);
    fprintf('    Converged:   %.0f%%\n', 100*sum(stv==0)/n_actual);
    fprintf('    Solve time:  mean=%.1f ms, median=%.1f ms, max=%.1f ms\n', ...
        mean(tv)*1e3, median(tv)*1e3, max(tv)*1e3);
    fprintf('    SQP iters:   mean=%.1f, max=%d\n', mean(sv), max(sv));
    fprintf('    Final |e|:   %.2e\n', plot_err_norm(n_actual+1));
    fprintf('    Settle 1%%:  %.0f ms\n', settling(1)*1e3);

    % =================================================================
    %  SAVE RESULTS  (same format as baseline for compareExperiments.m)
    % =================================================================
    sd = struct();
    sd.experiment_name = exp_name;
    sd.experiment_desc = exp_desc;
    sd.timestamp       = datestr(now, 'yyyy-mm-dd_HH-MM-SS');

    sd.config.nlp_solver_type       = 'SQP';
    sd.config.integrator_type       = cfg_integ;
    sd.config.sim_method_num_stages = cfg_stages;
    sd.config.sim_method_num_steps  = cfg_steps;
    sd.config.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    sd.config.qp_solver_iter_max    = 200;
    sd.config.hessian_approx        = 'GAUSS_NEWTON';
    sd.config.regularize_method     = 'CONVEXIFY';
    sd.config.cost_type             = 'NONLINEAR_LS';
    sd.config.N                     = N;
    sd.config.Tf                    = Tf;
    sd.config.dt                    = dt_mpc;
    sd.config.nx                    = nx;
    sd.config.nu                    = nu;
    sd.config.magnet_n              = paramsFast.magnet.n;
    sd.config.perturbation_scale    = perturbation_scale;

    sd.t  = (0:sim_steps) * dt_mpc;
    sd.x  = plot_x;
    sd.u  = plot_u;

    sd.status   = plot_stat;
    sd.sqp_iter = plot_sqp;
    sd.qp_iter  = plot_qp_iter;

    sd.timing.time_tot    = plot_time_tot;
    sd.timing.time_lin    = plot_time_lin;
    sd.timing.time_sim    = plot_time_sim;
    sd.timing.time_sim_ad = plot_time_sim_ad;
    sd.timing.time_sim_la = plot_time_sim_la;
    sd.timing.time_qp_sol = plot_time_qp;
    sd.timing.time_reg    = plot_time_reg;
    sd.timing.time_glob   = plot_time_glob;

    sd.tracking.err_norm = plot_err_norm;
    sd.tracking.err_pos  = plot_err_pos;
    sd.tracking.err_ang  = plot_err_ang;

    sd.constraints.max_slack         = plot_max_slack;
    sd.constraints.n_hard_violations = 0;    % computed below
    sd.constraints.max_violation     = 0;

    lbx_c = [-0.025; -0.025; 0.015; -0.35; -0.35];
    ubx_c = [ 0.025;  0.025; 0.055;  0.35;  0.35];
    idx_c = [1, 2, 3, 4, 5];
    n_viol = 0; max_viol = 0;
    for j = 1:n_actual+1
        for c = 1:length(idx_c)
            v = max([lbx_c(c) - plot_x(idx_c(c),j), ...
                     plot_x(idx_c(c),j) - ubx_c(c), 0]);
            if v > 0; n_viol = n_viol + 1; max_viol = max(max_viol, v); end
        end
    end
    sd.constraints.n_hard_violations = n_viol;
    sd.constraints.max_violation     = max_viol;

    sd.settling.thresholds_pct = thresholds * 100;
    sd.settling.times_s        = settling;

    sd.summary.n_actual              = n_actual;
    sd.summary.all_converged         = all(stv==0);
    sd.summary.solve_time_mean_ms    = mean(tv)*1e3;
    sd.summary.solve_time_median_ms  = median(tv)*1e3;
    sd.summary.solve_time_std_ms     = std(tv)*1e3;
    sd.summary.solve_time_max_ms     = max(tv)*1e3;
    sd.summary.solve_time_min_ms     = min(tv)*1e3;
    sd.summary.solve_time_p95_ms     = prctile_(tv, 95)*1e3;
    sd.summary.solve_time_p99_ms     = prctile_(tv, 99)*1e3;
    sd.summary.sqp_iter_mean         = mean(sv);
    sd.summary.qp_iter_mean          = nanmean_(qv);
    sd.summary.time_lin_mean_ms      = nanmean_(plot_time_lin(1:n_actual))*1e3;
    sd.summary.time_sim_mean_ms      = nanmean_(plot_time_sim(1:n_actual))*1e3;
    sd.summary.time_qp_sol_mean_ms   = nanmean_(plot_time_qp(1:n_actual))*1e3;
    sd.summary.time_reg_mean_ms      = nanmean_(plot_time_reg(1:n_actual))*1e3;
    sd.summary.final_err_norm        = plot_err_norm(n_actual+1);
    sd.summary.final_err_pos_mm      = plot_err_pos(n_actual+1);
    sd.summary.final_err_ang_deg     = plot_err_ang(n_actual+1);
    sd.summary.settling_1pct_ms      = settling(1)*1e3;
    sd.summary.settling_2pct_ms      = settling(2)*1e3;
    sd.summary.settling_5pct_ms      = settling(3)*1e3;
    sd.summary.max_soft_slack        = max(plot_max_slack(1:n_actual));
    sd.summary.pct_status_0          = 100*sum(stv==0)/n_actual;

    sd.xEq    = xEq;
    sd.uEq    = uEq;
    sd.Q      = Q;
    sd.R      = R;
    sd.params = paramsFast;

    fname = sprintf('nmpc_results_%s.mat', lower(exp_name));
    save(fname, '-struct', 'sd');
    fprintf('    Saved: %s\n', fname);

    % =================================================================
    %  CLEAN UP OCP SOLVER  (free MEX memory before building next one)
    % =================================================================
    clear ocp_solver ocp;
    fprintf('    OCP solver cleared.\n');
end

%% === FINAL SUMMARY TABLE ===
fprintf('\n\n');
fprintf('====================================================================\n');
fprintf('  Experiment B — All sub-experiments complete\n');
fprintf('====================================================================\n\n');

fprintf('%-6s | %-5s %3s %3s | %8s %8s %8s | %5s | %8s | %8s\n', ...
    'ID', 'Integ', 'St', 'Stp', 'Mean ms', 'Med ms', 'Max ms', ...
    'SQP', 'Final|e|', 'Sett1%');
fprintf('%s\n', repmat('-', 1, 85));

for e = 1:n_exp
    exp_name = sprintf('b_%s', lower(experiments{e,1}));
    fname = sprintf('nmpc_results_%s.mat', exp_name);
    if exist(fname, 'file')
        d = load(fname);
        s = d.summary;
        fprintf('%-6s | %-5s %3d %3d | %7.1f %8.1f %8.1f | %5.1f | %8.2e | %7.0f\n', ...
            d.experiment_name, d.config.integrator_type, ...
            d.config.sim_method_num_stages, d.config.sim_method_num_steps, ...
            s.solve_time_mean_ms, s.solve_time_median_ms, s.solve_time_max_ms, ...
            s.sqp_iter_mean, s.final_err_norm, s.settling_1pct_ms);
    end
end
fprintf('\nRun compareExperiments to see full comparison with baseline.\n');

%% === HELPER FUNCTIONS (no Statistics Toolbox) ===
function m = nanmean_(x)
    x = x(~isnan(x)); if isempty(x); m = NaN; else; m = mean(x); end
end

function p = prctile_(x, pct)
    x = sort(x(~isnan(x))); n = length(x);
    if n == 0; p = NaN; return; end
    r = (pct/100)*n; lo = max(floor(r),1); hi = min(ceil(r),n);
    if lo == hi; p = x(lo); else; p = x(lo)+(r-lo)*(x(hi)-x(lo)); end
end