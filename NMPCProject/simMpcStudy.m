%%
%% simMpcStudy — Systematic MPC comparison: LMPC vs NMPC-RTI vs NMPC-SQP
%
% Sweeps formulation × QP solver × horizon × perturbation, running each
% config as a cross-model simulation (Fast controller, Accurate+LUT plant).
% Results are saved per-config and aggregated into a summary CSV.

simSetup;
import casadi.*

umax = 1;    % physical hardware limit [A]
nx = 10;
dt_mpc = 0.001;  % 1 ms target embedded sample time

output_dir = 'mpcresults';
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

%% --- Equilibria ---
% Fast model equilibrium (used by MPC controller)
params_fast = load_params(MaglevModel.Fast);
[zEq_fast, ~, ~, ~] = computeSystemEquilibria(params_fast, MaglevModel.Fast);
xEq = [0; 0; zEq_fast(1); zeros(7,1)];
uEq = zeros(nu, 1);

% Accurate model equilibrium (ground truth plant)
params_acc = load_params(MaglevModel.Accurate);
[zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);

fprintf('Fast equilibrium:     z = %.4f mm\n', zEq_fast(1)*1e3);
fprintf('Accurate equilibrium: z = %.4f mm\n', zEq_acc(1)*1e3);

%% --- Cost weights (fixed across all configs) ---
Q = diag([...
    1e4, 1e4, ...       % x, y
    1e6, ...             % z
    1e3, 1e3, ...       % roll, pitch
    1e2, 1e2, 1e3, ...  % vx, vy, vz
    1e2, 1e2 ...        % wx, wy
]);
R = eye(nu) * 1e0;

%% --- LMPC: Linearize Fast model at equilibrium ---
fprintf('\n--- Computing LMPC linearization ---\n');

% Use 12-state MATLAB dynamics, then reduce to 10-state
f_12 = @(x12, u) maglevSystemDynamics(x12, u, params_fast, MaglevModel.Fast);
h_12 = @(x12, u) x12;  % identity output
xEq_12 = [0; 0; zEq_fast(1); zeros(9,1)];

delta = 1e-7;
[A12, B12, ~, ~] = finiteDifferenceLinearization(f_12, h_12, xEq_12, uEq, delta);

% Reduce to 10-state: keep [1:5, 7:11] (remove yaw=6, wz=12)
keep = [1:5, 7:11];
Ac = A12(keep, keep);
Bc = B12(keep, :);

% Discretize via matrix exponential: [Ad Bd; 0 I] = expm([Ac Bc; 0 0]*dt_mpc)
M_exp = expm([Ac Bc; zeros(nu, nx+nu)] * dt_mpc);
Ad = M_exp(1:nx, 1:nx);
Bd = M_exp(1:nx, nx+1:end);

fprintf('  Ac eigenvalues (real): %s\n', mat2str(real(eig(Ac))', 4));
fprintf('  Ad spectral radius: %.6f\n', max(abs(eig(Ad))));

%% --- Build NMPC model (once) ---
fprintf('\n--- Building NMPC model (Fast, analytical) ---\n');
model_nmpc = get_maggy_model(MaglevModel.Fast, use_luts=false);

%% --- Build Plant model (Accurate + LUT, once) ---
fprintf('\n--- Building Plant model (Accurate + LUT) ---\n');
model_plant = get_maggy_model(MaglevModel.Accurate, use_luts=true);

sim_plant = AcadosSim();
sim_plant.model = model_plant;
sim_plant.solver_options.Tsim            = dt;
sim_plant.solver_options.integrator_type = 'ERK';
sim_plant.solver_options.num_stages      = 4;
sim_plant.solver_options.num_steps       = 1;
plant_solver = AcadosSimSolver(sim_plant);
fprintf('  Plant solver built.\n');

%% --- Define parameter grid ---
formulations  = {'LMPC', 'NMPC_RTI', 'NMPC_SQP'};
qp_solvers    = {'PARTIAL_CONDENSING_HPIPM', 'FULL_CONDENSING_DAQP', 'FULL_CONDENSING_HPIPM'};
horizons      = [10, 20, 30, 40, 60];
perturbations = [0.001, 0.003, 0.005];  % z-offset [m]

configs = struct([]);
idx = 0;
for iF = 1:length(formulations)
    for iQP = 1:length(qp_solvers)
        for iH = 1:length(horizons)
            for iP = 1:length(perturbations)
                idx = idx + 1;
                configs(idx).formulation = formulations{iF};
                configs(idx).qp_solver   = qp_solvers{iQP};
                configs(idx).N_horizon   = horizons(iH);
                configs(idx).pert_m      = perturbations(iP);
            end
        end
    end
end

nConfigs = length(configs);
fprintf('\n=== simMpcStudy: %d configurations ===\n\n', nConfigs);

%% --- Summary storage ---
summary = cell(nConfigs, 1);

%% --- Run each configuration ---
for ic = 1:nConfigs
    cfg = configs(ic);

    % Short labels
    qp_short = strrep(strrep(cfg.qp_solver, 'FULL_CONDENSING_', 'FC_'), ...
                       'PARTIAL_CONDENSING_', 'PC_');
    tag = sprintf('%s_%s_H%d_p%dmm', ...
        cfg.formulation, qp_short, cfg.N_horizon, round(cfg.pert_m*1e3));

    save_file = fullfile(output_dir, ['study_' tag '.mat']);
    fprintf('\n--- [%d/%d] %s ---\n', ic, nConfigs, tag);

    % Skip if result already exists
    if exist(save_file, 'file')
        fprintf('  Already exists, skipping.\n');
        summary{ic} = sprintf('%-55s  SKIPPED', tag);
        continue;
    end

    try
        % --- Initial condition: perturb around Accurate equilibrium ---
        x0 = [0; 0; zEq_acc(1) + cfg.pert_m; zeros(7,1)];

        Tf = dt_mpc * cfg.N_horizon;

        % --- Build OCP ---
        ocp = AcadosOcp();

        if strcmp(cfg.formulation, 'LMPC')
            % Discrete linear model
            x_sym = MX.sym('x', nx);
            u_sym = MX.sym('u', nu);

            mdl = AcadosModel();
            mdl.name = ['study_' tag];
            mdl.x = x_sym;
            mdl.u = u_sym;
            mdl.disc_dyn_expr = Ad * x_sym + Bd * u_sym;

            ocp.model = mdl;
            ocp.solver_options.integrator_type = 'DISCRETE';
            ocp.solver_options.nlp_solver_type = 'SQP';
            ocp.solver_options.nlp_solver_max_iter = 1;  % linear, 1 iter suffices
        else
            % Nonlinear model — clone and rename
            mdl = AcadosModel();
            mdl.name = ['study_' tag];
            mdl.x = model_nmpc.x;
            mdl.u = model_nmpc.u;
            mdl.xdot = model_nmpc.xdot;
            mdl.f_impl_expr = model_nmpc.f_impl_expr;
            mdl.f_expl_expr = model_nmpc.f_expl_expr;

            ocp.model = mdl;
            ocp.solver_options.integrator_type       = 'ERK';
            ocp.solver_options.sim_method_num_stages = 4;
            ocp.solver_options.sim_method_num_steps  = 1;

            if strcmp(cfg.formulation, 'NMPC_RTI')
                ocp.solver_options.nlp_solver_type    = 'SQP_RTI';
                ocp.solver_options.nlp_solver_max_iter = 200;
            else  % NMPC_SQP
                ocp.solver_options.nlp_solver_type    = 'SQP';
                ocp.solver_options.nlp_solver_max_iter = 200;
                ocp.solver_options.globalization       = 'MERIT_BACKTRACKING';
            end
        end

        ocp.solver_options.N_horizon = cfg.N_horizon;
        ocp.solver_options.tf        = Tf;
        ocp.solver_options.qp_solver = cfg.qp_solver;
        ocp.solver_options.ext_fun_compile_flags = '-O2';

        % Cost
        ocp.cost.cost_type   = 'LINEAR_LS';
        ocp.cost.cost_type_0 = 'LINEAR_LS';
        ocp.cost.cost_type_e = 'LINEAR_LS';

        ocp.cost.Vx   = [eye(nx); zeros(nu, nx)];
        ocp.cost.Vu   = [zeros(nx, nu); eye(nu)];
        ocp.cost.Vx_0 = ocp.cost.Vx;
        ocp.cost.Vu_0 = ocp.cost.Vu;
        ocp.cost.Vx_e = eye(nx);

        ocp.cost.W   = blkdiag(Q, R);
        ocp.cost.W_0 = blkdiag(Q, R);
        ocp.cost.W_e = 10 * Q;

        ocp.cost.yref   = [xEq; uEq];
        ocp.cost.yref_0 = [xEq; uEq];
        ocp.cost.yref_e = xEq;

        % Input constraints
        ocp.constraints.idxbu = 0:nu-1;
        ocp.constraints.lbu   = -umax * ones(nu, 1);
        ocp.constraints.ubu   =  umax * ones(nu, 1);
        ocp.constraints.x0    = x0;

        tic_build = tic;
        ocp_solver = AcadosOcpSolver(ocp);
        t_build_ocp = toc(tic_build);
        fprintf('  OCP build: %.1f s\n', t_build_ocp);

        % Warm-start
        for k = 0:cfg.N_horizon
            ocp_solver.set('x', xEq, k);
        end
        for k = 0:cfg.N_horizon-1
            ocp_solver.set('u', uEq, k);
        end

        % --- Simulation loop ---
        n_sub  = round(dt_mpc / dt);
        t_vec  = 0:dt:2.0;
        N_sim  = numel(t_vec);
        N_mpc  = floor(N_sim / n_sub);

        x_sim = zeros(nx, N_sim);
        u_sim = zeros(nu, N_sim);
        t_mpc_log    = zeros(1, N_mpc);
        t_lin_log    = zeros(1, N_mpc);
        t_qp_log     = zeros(1, N_mpc);
        t_reg_log    = zeros(1, N_mpc);
        status_log   = zeros(1, N_mpc);
        sqp_iter_log = zeros(1, N_mpc);
        cost_log     = zeros(1, N_mpc);

        x_cur = x0;
        u_cur = uEq;
        diverged = false;
        n_mpc_run = N_mpc;

        for k = 1:N_mpc
            % MPC solve
            ocp_solver.set('constr_x0', x_cur);
            ocp_solver.solve();

            t_mpc_log(k)    = ocp_solver.get('time_tot');
            t_lin_log(k)    = ocp_solver.get('time_lin');
            t_qp_log(k)     = ocp_solver.get('time_qp_sol');
            t_reg_log(k)    = ocp_solver.get('time_reg');
            status_log(k)   = ocp_solver.get('status');
            sqp_iter_log(k) = ocp_solver.get('sqp_iter');

            if status_log(k) ~= 0 && status_log(k) ~= 2
                fprintf('  *** OCP status %d at step %d ***\n', status_log(k), k);
            end

            u_cur = ocp_solver.get('u', 0);

            % Stage cost
            dx = x_cur - xEq;
            du = u_cur - uEq;
            cost_log(k) = dx' * Q * dx + du' * R * du;

            % Warm-shift
            for i = 0:cfg.N_horizon-2
                ocp_solver.set('x', ocp_solver.get('x', i+1), i);
                ocp_solver.set('u', ocp_solver.get('u', i+1), i);
            end
            ocp_solver.set('x', ocp_solver.get('x', cfg.N_horizon), cfg.N_horizon);
            ocp_solver.set('u', ocp_solver.get('u', cfg.N_horizon-1), cfg.N_horizon-1);

            % Plant sub-steps (Accurate model)
            for j = 1:n_sub
                idx_s = (k-1)*n_sub + j;
                if idx_s > N_sim, break; end
                x_sim(:, idx_s) = x_cur;
                u_sim(:, idx_s) = u_cur;
                x_cur = plant_solver.simulate(x_cur, u_cur);
            end

            % Divergence check
            diverged = abs(x_cur(3)) > 0.5       || ...
                       max(abs(x_cur(4:5))) > pi  || ...
                       any(isnan(x_cur))          || ...
                       any(isinf(x_cur));

            if diverged
                fprintf('  DIVERGED at MPC step %d\n', k);
                n_mpc_run = k;
                last_idx = min(k*n_sub, N_sim);
                x_sim        = x_sim(:, 1:last_idx);
                u_sim        = u_sim(:, 1:last_idx);
                t_vec        = t_vec(1:last_idx);
                t_mpc_log    = t_mpc_log(1:k);
                t_lin_log    = t_lin_log(1:k);
                t_qp_log     = t_qp_log(1:k);
                t_reg_log    = t_reg_log(1:k);
                status_log   = status_log(1:k);
                sqp_iter_log = sqp_iter_log(1:k);
                cost_log     = cost_log(1:k);
                break;
            end
        end

        % --- Compute metrics ---
        % Use Accurate equilibrium as settling target (cross-model offset)
        xEq_acc = [0; 0; zEq_acc(1); zeros(7,1)];
        z_target = xEq_acc(3);

        % Settling time: 2% band around final value (accounts for SS offset)
        if ~diverged && numel(t_vec) > 10
            z_final = mean(x_sim(3, end-9:end));
            z_excursion = abs(x0(3) - z_final);
            z_band = 0.02 * z_excursion;
            z_band = max(z_band, 1e-5);  % minimum 10 um band
            z_err = abs(x_sim(3,:) - z_final);
            settled = z_err < z_band;
            last_outside = find(~settled, 1, 'last');
            if isempty(last_outside)
                t_settle = 0;
            elseif last_outside < numel(t_vec)
                t_settle = t_vec(last_outside);
            else
                t_settle = NaN;  % never settled
            end
        else
            t_settle = NaN;
        end

        % Overshoot: max z deviation beyond initial perturbation from target
        z_overshoot = max(abs(x_sim(3,:) - z_target)) - abs(x0(3) - z_target);
        z_overshoot = max(z_overshoot, 0);

        % Steady-state error (relative to Accurate equilibrium)
        if ~diverged && numel(t_vec) > 10
            ss_err = mean(abs(x_sim(3, end-9:end) - z_target));
        else
            ss_err = NaN;
        end

        % Timing stats
        med_mpc = median(t_mpc_log) * 1e6;
        med_lin = median(t_lin_log) * 1e6;
        med_qp  = median(t_qp_log)  * 1e6;
        med_reg = median(t_reg_log) * 1e6;
        mean_mpc = mean(t_mpc_log) * 1e6;
        max_mpc  = max(t_mpc_log) * 1e6;

        fprintf('  Steps: %d | t_mpc=%.0f us (lin=%.0f qp=%.0f) | settle=%.3f s | div=%d\n', ...
            n_mpc_run, med_mpc, med_lin, med_qp, t_settle, diverged);

        % Save
        sim_data              = struct();
        sim_data.t            = t_vec;
        sim_data.x            = x_sim;
        sim_data.u            = u_sim;
        sim_data.xEq          = xEq;
        sim_data.uEq          = uEq;
        sim_data.dt           = dt;
        sim_data.dt_mpc       = dt_mpc;
        sim_data.config       = cfg;
        sim_data.t_mpc        = t_mpc_log;
        sim_data.t_lin        = t_lin_log;
        sim_data.t_qp         = t_qp_log;
        sim_data.t_reg        = t_reg_log;
        sim_data.status       = status_log;
        sim_data.sqp_iter     = sqp_iter_log;
        sim_data.cost         = cost_log;
        sim_data.t_build      = t_build_ocp;
        sim_data.diverged     = diverged;
        sim_data.t_settle     = t_settle;
        sim_data.z_overshoot  = z_overshoot;
        sim_data.ss_err       = ss_err;
        sim_data.n_mpc_run    = n_mpc_run;

        save(save_file, '-struct', 'sim_data');
        fprintf('  Saved: %s\n', save_file);

        summary{ic} = sprintf('%-55s  mpc=%5.0f us  qp=%4.0f  settle=%.3f  div=%d', ...
            tag, med_mpc, med_qp, t_settle, diverged);

        clear ocp_solver ocp mdl;

    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        summary{ic} = sprintf('%-55s  ERROR: %s', tag, ME.message);
    end
end

%% --- Print summary table ---
fprintf('\n\n==================== MPC STUDY SUMMARY ====================\n');
fprintf('%-55s  %-60s\n', 'Configuration', 'Results');
fprintf('%s\n', repmat('-', 1, 120));
for ic = 1:nConfigs
    fprintf('%s\n', summary{ic});
end

%% --- Aggregate into CSV (recompute metrics from trajectory data) ---
fprintf('\n--- Generating summary CSV ---\n');
csv_file = fullfile(output_dir, 'mpc_study_summary.csv');
fid = fopen(csv_file, 'w');
fprintf(fid, 'formulation,qp_solver,horizon,perturbation_mm,diverged,t_settle_s,z_overshoot_mm,ss_err_mm,med_mpc_us,med_lin_us,med_qp_us,med_reg_us,mean_mpc_us,max_mpc_us,mean_sqp_iter,t_build_s\n');

files = dir(fullfile(output_dir, 'study_*.mat'));
for i = 1:numel(files)
    d = load(fullfile(files(i).folder, files(i).name));
    if ~isfield(d, 'config'), continue; end
    c = d.config;

    % Recompute metrics using Accurate equilibrium as target
    z_target = zEq_acc(1);
    x0_i = [0; 0; zEq_acc(1) + c.pert_m; zeros(7,1)];

    if ~d.diverged && numel(d.t) > 10
        z_final = mean(d.x(3, end-9:end));
        z_excursion = abs(x0_i(3) - z_final);
        z_band = max(0.02 * z_excursion, 1e-5);
        z_err = abs(d.x(3,:) - z_final);
        settled = z_err < z_band;
        last_out = find(~settled, 1, 'last');
        if isempty(last_out)
            ts = 0;
        elseif last_out < numel(d.t)
            ts = d.t(last_out);
        else
            ts = NaN;
        end
        ss = mean(abs(d.x(3, end-9:end) - z_target));
    else
        ts = NaN;
        ss = NaN;
    end

    zo = max(abs(d.x(3,:) - z_target)) - abs(x0_i(3) - z_target);
    zo = max(zo, 0);

    fprintf(fid, '%s,%s,%d,%.0f,%d,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f\n', ...
        c.formulation, c.qp_solver, c.N_horizon, c.pert_m*1e3, ...
        d.diverged, ts, zo*1e3, ss*1e3, ...
        median(d.t_mpc)*1e6, median(d.t_lin)*1e6, median(d.t_qp)*1e6, ...
        median(d.t_reg)*1e6, mean(d.t_mpc)*1e6, max(d.t_mpc)*1e6, ...
        mean(d.sqp_iter), d.t_build);
end
fclose(fid);
fprintf('Summary CSV: %s\n', csv_file);
fprintf('\nResults saved in: %s/\n', output_dir);
