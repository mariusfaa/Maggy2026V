%% 
%% simBenchMpc — Sweep NMPC solver settings and compare performance
%
% Loops over combinations of MPC settings (QP solver, horizon, integrator,
% etc.), runs closed-loop NMPC simulations, and saves results.

simSetup;
import casadi.*

umax = 4;
nx = 10;

output_dir = 'mpcresults';
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

%% --- Recompute equilibrium using the MPC model (Fast) ---
params_fast = load_params(MaglevModel.Fast);
[zEq_fast, ~, ~, ~] = computeSystemEquilibria(params_fast, MaglevModel.Fast);
xEq = [0; 0; zEq_fast(1); zeros(7,1)];
uEq = zeros(nu, 1);

x0_full = [0; 0; zEq_fast(1); zeros(9,1)] + [0; 0.001; 0.001; 0; 0; 0; zeros(6,1)];
x0 = x0_full([1:5,7:11]);

%% --- Cost weights (fixed across all configs) ---
Q = diag([...
    1e4, 1e4, ...       % x, y
    1e6, ...             % z
    1e3, 1e3, ...       % roll, pitch
    1e2, 1e2, 1e3, ...  % vx, vy, vz
    1e2, 1e2 ...        % wx, wy
]);
R = eye(nu) * 1e0;

%% --- Define parameter grid ---
configs = struct([]);

qp_solvers   = {'PARTIAL_CONDENSING_HPIPM', 'FULL_CONDENSING_DAQP', 'FULL_CONDENSING_HPIPM', 'FULL_CONDENSING_QPOASES'};
nlp_solvers  = {'SQP_RTI'};%, 'SQP', 'SQP_WITH_FEASIBLE_QP'};
horizons     = [10, 20, 30];
dt_mpcs      = [0.001, 0.002, 0.003, 0.005];
ocp_stages   = [4];
n_radials    = [12];

idx = 0;
for iQP = 1:length(qp_solvers)
    for iNLP = 1:length(nlp_solvers)
        for iH = 1:length(horizons)
            for iDt = 1:length(dt_mpcs)
                for iStg = 1:length(ocp_stages)
                    for iNR = 1:length(n_radials)
                        idx = idx + 1;
                        configs(idx).qp_solver    = qp_solvers{iQP};
                        configs(idx).nlp_solver   = nlp_solvers{iNLP};
                        configs(idx).N_horizon    = horizons(iH);
                        configs(idx).dt_mpc       = dt_mpcs(iDt);
                        configs(idx).ocp_stages   = ocp_stages(iStg);
                        configs(idx).n_radial     = n_radials(iNR);
                    end
                end
            end
        end
    end
end

nConfigs = length(configs);
fprintf('\n=== simBenchMpc: %d configurations ===\n\n', nConfigs);

%% --- Summary storage ---
summary = cell(nConfigs, 1);

%% --- Run each configuration ---
for ic = 1:nConfigs
    cfg = configs(ic);

    % Short QP label
    qp_short = strrep(strrep(cfg.qp_solver, 'FULL_CONDENSING_', ''), 'PARTIAL_CONDENSING_', 'P');
    tag = sprintf('%s_%s_H%d_dt%g_s%d_nr%d', ...
        cfg.nlp_solver, qp_short, cfg.N_horizon, cfg.dt_mpc*1e3, ...
        cfg.ocp_stages, cfg.n_radial);

    save_file = fullfile(output_dir, ['mpc_' tag '.mat']);
    fprintf('\n--- [%d/%d] %s ---\n', ic, nConfigs, tag);

    % Skip if result already exists
    if exist(save_file, 'file')
        fprintf('  Already exists, skipping.\n');
        summary{ic} = sprintf('%-50s  SKIPPED (exists)', tag);
        continue;
    end

    try
        % Build model
        params_cfg = load_params(MaglevModel.Fast);
        params_cfg.magnet.n = cfg.n_radial;
        params_cfg.lut_opts.enabled = false;

        x_sym    = MX.sym('x',    nx);
        u_sym    = MX.sym('u',    nu);
        xdot_sym = MX.sym('xdot', nx);

        f_expl = maglevSystemDynamicsReduced_casadi(x_sym, u_sym, params_cfg, MaglevModel.Fast);

        mdl = AcadosModel();
        mdl.name        = ['mpc_' tag];
        mdl.x           = x_sym;
        mdl.u           = u_sym;
        mdl.xdot        = xdot_sym;
        mdl.f_impl_expr = xdot_sym - f_expl;
        mdl.f_expl_expr = f_expl;

        Tf = cfg.dt_mpc * cfg.N_horizon;

        % --- OCP setup ---
        ocp = AcadosOcp();
        ocp.model = mdl;

        ocp.solver_options.N_horizon             = cfg.N_horizon;
        ocp.solver_options.tf                    = Tf;
        ocp.solver_options.integrator_type       = 'ERK';
        ocp.solver_options.sim_method_num_stages = cfg.ocp_stages;
        ocp.solver_options.sim_method_num_steps  = 1;
        ocp.solver_options.nlp_solver_type       = cfg.nlp_solver;
        ocp.solver_options.nlp_solver_max_iter   = 200;
        ocp.solver_options.qp_solver             = cfg.qp_solver;
        ocp.solver_options.globalization          = 'MERIT_BACKTRACKING';
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

        % Constraints
        ocp.constraints.idxbu = 0:nu-1;
        ocp.constraints.lbu   = -umax * ones(nu, 1);
        ocp.constraints.ubu   =  umax * ones(nu, 1);
        ocp.constraints.x0    = x0;

        tic_build = tic;
        ocp_solver = AcadosOcpSolver(ocp);
        t_build_ocp = toc(tic_build);
        fprintf('  OCP build: %.1f s\n', t_build_ocp);

        % --- Plant sim solver (fixed high-quality integrator) ---
        sim = AcadosSim();
        sim.model = mdl;
        sim.solver_options.Tsim            = dt;
        sim.solver_options.integrator_type = 'IRK';
        sim.solver_options.num_stages      = 1;
        sim.solver_options.num_steps       = 2;

        sim_solver = AcadosSimSolver(sim);

        % Warm-start OCP
        for k = 0:cfg.N_horizon
            ocp_solver.set('x', xEq, k);
        end
        for k = 0:cfg.N_horizon-1
            ocp_solver.set('u', uEq, k);
        end

        % --- Simulation loop ---
        n_sub  = round(cfg.dt_mpc / dt);
        t_vec  = 0:dt:0.5;
        N_sim  = numel(t_vec);
        N_mpc  = floor(N_sim / n_sub);

        x_sim = zeros(nx, N_sim);
        u_sim = zeros(nu, N_sim);
        t_mpc_log      = zeros(1, N_mpc);
        t_lin_log      = zeros(1, N_mpc);
        t_qp_log       = zeros(1, N_mpc);
        t_reg_log      = zeros(1, N_mpc);
        t_sub_log      = zeros(1, N_mpc);
        status_log     = zeros(1, N_mpc);
        sqp_iter_log   = zeros(1, N_mpc);
        cost_log       = zeros(1, N_mpc);

        x_cur = x0;
        u_cur = uEq;
        diverged = false;
        n_mpc_run = N_mpc;

        for k = 1:N_mpc
            % MPC solve
            ocp_solver.set('constr_x0', x_cur);
            ocp_solver.solve();

            t_mpc_log(k) = ocp_solver.get('time_tot');
            t_lin_log(k) = ocp_solver.get('time_lin');
            t_qp_log(k)  = ocp_solver.get('time_qp_sol');
            t_reg_log(k) = ocp_solver.get('time_reg');
            status_log(k) = ocp_solver.get('status');
            sqp_iter_log(k) = ocp_solver.get('sqp_iter');

            if status_log(k) ~= 0 && status_log(k) ~= 2
                fprintf('  *** OCP status %d at step %d ***\n', status_log(k), k);
            end

            u_cur = ocp_solver.get('u', 0);

            % Stage cost: (x-xEq)'Q(x-xEq) + (u-uEq)'R(u-uEq)
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

            % Plant sub-steps (acados internal timing)
            t_sim_k = 0;
            for j = 1:n_sub
                idx = (k-1)*n_sub + j;
                if idx > N_sim, break; end
                x_sim(:, idx) = x_cur;
                u_sim(:, idx) = u_cur;
                x_cur = sim_solver.simulate(x_cur, u_cur);
                t_sim_k = t_sim_k + sim_solver.get('time_tot');
            end
            t_sub_log(k) = t_sim_k;

            % Divergence check
            diverged = abs(x_cur(3)) > 0.5       || ...
                       max(abs(x_cur(4:5))) > pi  || ...
                       any(isnan(x_cur))          || ...
                       any(isinf(x_cur));

            if diverged
                fprintf('  DIVERGED at MPC step %d\n', k);
                n_mpc_run = k;
                last_idx = min(k*n_sub, N_sim);
                x_sim      = x_sim(:, 1:last_idx);
                u_sim      = u_sim(:, 1:last_idx);
                t_vec      = t_vec(1:last_idx);
                t_mpc_log    = t_mpc_log(1:k);
                t_lin_log    = t_lin_log(1:k);
                t_qp_log     = t_qp_log(1:k);
                t_reg_log    = t_reg_log(1:k);
                t_sub_log    = t_sub_log(1:k);
                status_log   = status_log(1:k);
                sqp_iter_log = sqp_iter_log(1:k);
                cost_log     = cost_log(1:k);
                break;
            end
        end

        % Performance stats (all acados internal times)
        t_total_log = t_mpc_log + t_sub_log;  % OCP solve + plant sim
        med_mpc = median(t_mpc_log) * 1e6;
        med_lin = median(t_lin_log) * 1e6;
        med_qp  = median(t_qp_log) * 1e6;
        med_sim = median(t_sub_log) * 1e6;
        med_tot = median(t_total_log) * 1e6;
        rt_factor = cfg.dt_mpc / mean(t_total_log);

        fprintf('  MPC steps: %d  |  mpc=%.0f us (lin=%.0f qp=%.0f)  sim=%.0f us  total=%.0f us  RT=%.2fx\n', ...
            n_mpc_run, med_mpc, med_lin, med_qp, med_sim, med_tot, rt_factor);

        % Save
        sim_data          = struct();
        sim_data.t        = t_vec;
        sim_data.x        = x_sim;
        sim_data.u        = u_sim;
        sim_data.xEq      = xEq;
        sim_data.uEq      = uEq;
        sim_data.dt       = dt;
        sim_data.t_mpc    = t_mpc_log;
        sim_data.t_lin    = t_lin_log;
        sim_data.t_qp     = t_qp_log;
        sim_data.t_reg    = t_reg_log;
        sim_data.t_sim    = t_sub_log;
        sim_data.t_total  = t_total_log;
        sim_data.status   = status_log;
        sim_data.sqp_iter = sqp_iter_log;
        sim_data.cost     = cost_log;
        sim_data.config   = cfg;
        sim_data.t_build  = t_build_ocp;
        sim_data.diverged = diverged;

        save(save_file, '-struct', 'sim_data');
        fprintf('  Saved: %s\n', save_file);

        summary{ic} = sprintf('%-50s  mpc=%5.0f us  lin=%5.0f  qp=%4.0f  RT=%.2fx  div=%d', ...
            tag, med_mpc, med_lin, med_qp, rt_factor, diverged);

        % Clean up solvers
        clear ocp_solver sim_solver ocp sim mdl f_expl x_sym u_sym xdot_sym;

    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        summary{ic} = sprintf('%-50s  ERROR: %s', tag, ME.message);
    end
end

%% --- Print summary table ---
fprintf('\n\n==================== MPC BENCHMARK SUMMARY ====================\n');
fprintf('%-50s  %-60s\n', 'Configuration', 'Results');
fprintf('%s\n', repmat('-', 1, 112));
for ic = 1:nConfigs
    fprintf('%s\n', summary{ic});
end
fprintf('\nResults saved in: %s/\n', output_dir);
