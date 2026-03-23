%%
%% simMpcAdaptiveLMPC — Adaptive LMPC with periodic Jacobian relinearization
%
% Instead of linearizing only once at the equilibrium (static LMPC) or
% solving a full NLP at every step (NMPC), this script evaluates the
% Jacobian at the current operating point every K steps, re-discretizes,
% and updates the DISCRETE dynamics inside acados.
%
% The CasADi symbolic Jacobian is computed analytically (no finite
% differences at runtime), making each re-evaluation cheap.
%
% Sweep parameter: relinearization period K = {1, 2, 5, 10, 20, 50, Inf}
%   K = 1   : update every step   (maximum adaptation, closest to NMPC)
%   K = Inf : never update         (equivalent to static LMPC)
%
% Cross-model simulation: Fast controller model, Accurate+LUT plant.

simSetup;
import casadi.*

umax = 1;
nx   = 10;
nu_  = 4;          % avoid shadowing outer 'nu' from simSetup
dt_mpc = 0.001;

output_dir = 'mpcresults';
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

%% ================================================================
%% Equilibria
%% ================================================================

params_fast = load_params(MaglevModel.Fast);
[zEq_fast, ~, ~, ~] = computeSystemEquilibria(params_fast, MaglevModel.Fast);
xEq = [0; 0; zEq_fast(1); zeros(7,1)];
uEq = zeros(nu_, 1);

params_acc = load_params(MaglevModel.Accurate);
[zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);

fprintf('Fast equilibrium:     z = %.4f mm\n', zEq_fast(1)*1e3);
fprintf('Accurate equilibrium: z = %.4f mm\n', zEq_acc(1)*1e3);

%% ================================================================
%% Cost weights
%% ================================================================

Q = diag([1e4, 1e4, 1e6, 1e3, 1e3, 1e2, 1e2, 1e3, 1e2, 1e2]);
R = eye(nu_);

%% ================================================================
%% Build CasADi analytical Jacobian function
%% ================================================================

fprintf('\n--- Building CasADi Jacobian function ---\n');

x_cas = MX.sym('x', nx);
u_cas = MX.sym('u', nu_);

% Evaluate reduced 10-state CasADi dynamics (Fast, analytical, no LUT)
params_fast.lut_opts.enabled = false;
f_expl = maglevSystemDynamicsReduced_casadi(x_cas, u_cas, params_fast, MaglevModel.Fast);

% Symbolic Jacobians
Ac_sym = jacobian(f_expl, x_cas);   % 10x10
Bc_sym = jacobian(f_expl, u_cas);   % 10x4

% Compile into a CasADi Function for fast evaluation
jac_fun = Function('jac_fun', {x_cas, u_cas}, {Ac_sym, Bc_sym}, ...
                   {'x', 'u'}, {'Ac', 'Bc'});
fprintf('  Jacobian function built (%d outputs).\n', jac_fun.n_out());

% Verify at equilibrium
[Ac_eq_cas, Bc_eq_cas] = jac_fun(xEq, uEq);
Ac_eq = full(Ac_eq_cas);
Bc_eq = full(Bc_eq_cas);
fprintf('  Equilibrium eigenvalues (real): %s\n', mat2str(real(eig(Ac_eq))', 4));

% Discretize equilibrium Jacobian for initial guess
M_exp = expm([Ac_eq Bc_eq; zeros(nu_, nx+nu_)] * dt_mpc);
Ad_eq = M_exp(1:nx, 1:nx);
Bd_eq = M_exp(1:nx, nx+1:end);

%% ================================================================
%% Build Plant solver (Accurate + LUT, once)
%% ================================================================

fprintf('\n--- Building Plant solver (Accurate + LUT) ---\n');
model_plant = get_maggy_model(MaglevModel.Accurate, use_luts=true);

sim_plant = AcadosSim();
sim_plant.model = model_plant;
sim_plant.solver_options.Tsim            = dt;
sim_plant.solver_options.integrator_type = 'ERK';
sim_plant.solver_options.num_stages      = 4;
sim_plant.solver_options.num_steps       = 1;
plant_solver = AcadosSimSolver(sim_plant);
fprintf('  Plant solver built.\n');

%% ================================================================
%% Sweep configuration
%% ================================================================

K_values    = [1, 2, 5, 10, 20, 50, Inf];
N_horizon   = 20;       % fixed horizon for this study
qp_solver   = 'PARTIAL_CONDENSING_HPIPM';
perturbations = [0.001, 0.003, 0.005];  % [m]

Tf = dt_mpc * N_horizon;
t_sim_end = 2.0;  % simulation duration [s]

nK = numel(K_values);
nP = numel(perturbations);
nConfigs = nK * nP;

fprintf('\n=== simMpcAdaptiveLMPC: %d configurations (%d K-values x %d perturbations) ===\n', ...
    nConfigs, nK, nP);

%% ================================================================
%% Build a SINGLE acados OCP with parameterized A,B
%% ================================================================
%
% Model parameters p = [vec(Ad); vec(Bd)] — flattened column-major.
% disc_dyn_expr = reshape(p_Ad, nx, nx) * x + reshape(p_Bd, nx, nu) * u

fprintf('\n--- Building parameterized LMPC OCP ---\n');

np_total = nx*nx + nx*nu_;

x_sym = MX.sym('x', nx);
u_sym = MX.sym('u', nu_);
p_sym = MX.sym('p', np_total);

% Extract Ad and Bd from parameter vector
p_Ad = reshape(p_sym(1:nx*nx), nx, nx);
p_Bd = reshape(p_sym(nx*nx+1:end), nx, nu_);

disc_dyn = p_Ad * x_sym + p_Bd * u_sym;

%% ================================================================
%% Main sweep loop
%% ================================================================

summary = {};

for iK = 1:nK
    K = K_values(iK);
    if isinf(K)
        K_str = 'Inf';
    else
        K_str = num2str(K);
    end

    for iP = 1:nP
        pert_m = perturbations(iP);
        pert_mm = round(pert_m * 1e3);

        tag = sprintf('ALMPC_K%s_H%d_p%dmm', K_str, N_horizon, pert_mm);
        save_file = fullfile(output_dir, ['study_' tag '.mat']);

        fprintf('\n--- [K=%s, pert=%dmm] %s ---\n', K_str, pert_mm, tag);

        if exist(save_file, 'file')
            fprintf('  Already exists, skipping.\n');
            summary{end+1} = sprintf('%-55s  SKIPPED', tag); %#ok<SAGROW>
            continue;
        end

        try
            % Build a fresh OCP for each config (acados requires unique names)
            mdl = AcadosModel();
            mdl.name = ['study_' tag];
            mdl.x = x_sym;
            mdl.u = u_sym;
            mdl.p = p_sym;
            mdl.disc_dyn_expr = disc_dyn;

            ocp = AcadosOcp();
            ocp.model = mdl;

            ocp.solver_options.N_horizon  = N_horizon;
            ocp.solver_options.tf         = Tf;
            ocp.solver_options.integrator_type    = 'DISCRETE';
            ocp.solver_options.nlp_solver_type    = 'SQP';
            ocp.solver_options.nlp_solver_max_iter = 1;
            ocp.solver_options.qp_solver  = qp_solver;
            ocp.solver_options.ext_fun_compile_flags = '-O2';

            % Cost
            ocp.cost.cost_type   = 'LINEAR_LS';
            ocp.cost.cost_type_0 = 'LINEAR_LS';
            ocp.cost.cost_type_e = 'LINEAR_LS';

            ocp.cost.Vx   = [eye(nx); zeros(nu_, nx)];
            ocp.cost.Vu   = [zeros(nx, nu_); eye(nu_)];
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
            ocp.constraints.idxbu = 0:nu_-1;
            ocp.constraints.lbu   = -umax * ones(nu_, 1);
            ocp.constraints.ubu   =  umax * ones(nu_, 1);

            % Initial condition
            x0 = [0; 0; zEq_acc(1) + pert_m; zeros(7,1)];
            ocp.constraints.x0 = x0;

            % Default parameter values (equilibrium linearization)
            p0 = [Ad_eq(:); Bd_eq(:)];
            ocp.parameter_values = p0;

            tic_build = tic;
            ocp_solver = AcadosOcpSolver(ocp);
            t_build = toc(tic_build);
            fprintf('  OCP build: %.1f s\n', t_build);

            % Warm-start
            for k = 0:N_horizon
                ocp_solver.set('x', xEq, k);
            end
            for k = 0:N_horizon-1
                ocp_solver.set('u', uEq, k);
            end

            % --- Simulation loop ---
            n_sub  = round(dt_mpc / dt);
            t_vec  = 0:dt:t_sim_end;
            N_sim  = numel(t_vec);
            N_mpc  = floor(N_sim / n_sub);

            x_sim = zeros(nx, N_sim);
            u_sim = zeros(nu_, N_sim);
            t_mpc_log    = zeros(1, N_mpc);
            t_lin_log    = zeros(1, N_mpc);
            t_qp_log     = zeros(1, N_mpc);
            t_reg_log    = zeros(1, N_mpc);
            t_jac_log    = zeros(1, N_mpc);  % Jacobian evaluation time
            status_log   = zeros(1, N_mpc);
            n_updates    = 0;

            x_cur = x0;
            u_cur = uEq;
            diverged = false;
            n_mpc_run = N_mpc;

            % Current discrete matrices (start with equilibrium)
            Ad_cur = Ad_eq;
            Bd_cur = Bd_eq;

            for k = 1:N_mpc
                % --- Periodic Jacobian update ---
                t_jac_k = 0;
                if ~isinf(K) && (mod(k-1, K) == 0)
                    tic_jac = tic;

                    % Evaluate CasADi Jacobian at current state
                    [Ac_val, Bc_val] = jac_fun(x_cur, u_cur);
                    Ac_num = full(Ac_val);
                    Bc_num = full(Bc_val);

                    % Discretize via matrix exponential
                    M_exp_k = expm([Ac_num Bc_num; zeros(nu_, nx+nu_)] * dt_mpc);
                    Ad_cur = M_exp_k(1:nx, 1:nx);
                    Bd_cur = M_exp_k(1:nx, nx+1:end);

                    t_jac_k = toc(tic_jac);
                    n_updates = n_updates + 1;

                    % Update parameters for all stages
                    p_new = [Ad_cur(:); Bd_cur(:)];
                    ocp_solver.set('p', p_new);
                end
                t_jac_log(k) = t_jac_k;

                % MPC solve
                ocp_solver.set('constr_x0', x_cur);
                ocp_solver.solve();

                t_mpc_log(k) = ocp_solver.get('time_tot');
                t_lin_log(k) = ocp_solver.get('time_lin');
                t_qp_log(k)  = ocp_solver.get('time_qp_sol');
                t_reg_log(k) = ocp_solver.get('time_reg');
                status_log(k) = ocp_solver.get('status');

                if status_log(k) ~= 0 && status_log(k) ~= 2
                    fprintf('  *** OCP status %d at step %d ***\n', status_log(k), k);
                end

                u_cur = ocp_solver.get('u', 0);

                % Warm-shift
                for i = 0:N_horizon-2
                    ocp_solver.set('x', ocp_solver.get('x', i+1), i);
                    ocp_solver.set('u', ocp_solver.get('u', i+1), i);
                end
                ocp_solver.set('x', ocp_solver.get('x', N_horizon), N_horizon);
                ocp_solver.set('u', ocp_solver.get('u', N_horizon-1), N_horizon-1);

                % Plant sub-steps (Accurate model)
                for j = 1:n_sub
                    idx_s = (k-1)*n_sub + j;
                    if idx_s > N_sim, break; end
                    x_sim(:, idx_s) = x_cur;
                    u_sim(:, idx_s) = u_cur;
                    x_cur = plant_solver.simulate(x_cur, u_cur);
                end

                % Divergence check
                diverged = abs(x_cur(3)) > 0.5 || ...
                           max(abs(x_cur(4:5))) > pi || ...
                           any(isnan(x_cur)) || any(isinf(x_cur));

                if diverged
                    fprintf('  DIVERGED at step %d\n', k);
                    n_mpc_run = k;
                    last_idx = min(k*n_sub, N_sim);
                    x_sim     = x_sim(:, 1:last_idx);
                    u_sim     = u_sim(:, 1:last_idx);
                    t_vec     = t_vec(1:last_idx);
                    t_mpc_log = t_mpc_log(1:k);
                    t_lin_log = t_lin_log(1:k);
                    t_qp_log  = t_qp_log(1:k);
                    t_reg_log = t_reg_log(1:k);
                    t_jac_log = t_jac_log(1:k);
                    status_log = status_log(1:k);
                    break;
                end
            end

            % --- Compute metrics ---
            z_target = zEq_acc(1);

            if ~diverged && numel(t_vec) > 10
                z_final = mean(x_sim(3, end-9:end));
                z_excursion = abs(x0(3) - z_final);
                z_band = max(0.02 * z_excursion, 1e-5);
                z_err = abs(x_sim(3,:) - z_final);
                settled = z_err < z_band;
                last_outside = find(~settled, 1, 'last');
                if isempty(last_outside)
                    t_settle = 0;
                elseif last_outside < numel(t_vec)
                    t_settle = t_vec(last_outside);
                else
                    t_settle = NaN;
                end
                ss_err = mean(abs(x_sim(3, end-9:end) - z_target));
            else
                t_settle = NaN;
                ss_err = NaN;
            end

            z_overshoot = max(abs(x_sim(3,:) - z_target)) - abs(x0(3) - z_target);
            z_overshoot = max(z_overshoot, 0);

            % Timing: total = acados solve + Jacobian evaluation
            t_total_log = t_mpc_log + t_jac_log;
            med_mpc = median(t_mpc_log) * 1e6;
            med_jac = median(t_jac_log) * 1e6;
            med_tot = median(t_total_log) * 1e6;
            med_qp  = median(t_qp_log) * 1e6;

            fprintf('  Steps: %d | mpc=%.0f us + jac=%.0f us = %.0f us total | settle=%.3f s | updates=%d | div=%d\n', ...
                n_mpc_run, med_mpc, med_jac, med_tot, t_settle, n_updates, diverged);

            % Save
            sim_data = struct();
            sim_data.t         = t_vec;
            sim_data.x         = x_sim;
            sim_data.u         = u_sim;
            sim_data.xEq       = xEq;
            sim_data.uEq       = uEq;
            sim_data.dt        = dt;
            sim_data.dt_mpc    = dt_mpc;
            sim_data.config    = struct('formulation','ALMPC', ...
                                        'K_update', K, ...
                                        'N_horizon', N_horizon, ...
                                        'qp_solver', qp_solver, ...
                                        'pert_m', pert_m);
            sim_data.t_mpc     = t_mpc_log;
            sim_data.t_lin     = t_lin_log;
            sim_data.t_qp      = t_qp_log;
            sim_data.t_reg     = t_reg_log;
            sim_data.t_jac     = t_jac_log;
            sim_data.t_total   = t_total_log;
            sim_data.status    = status_log;
            sim_data.sqp_iter  = ones(size(t_mpc_log));
            sim_data.t_build   = t_build;
            sim_data.diverged  = diverged;
            sim_data.t_settle  = t_settle;
            sim_data.z_overshoot = z_overshoot;
            sim_data.ss_err    = ss_err;
            sim_data.n_updates = n_updates;
            sim_data.n_mpc_run = n_mpc_run;

            save(save_file, '-struct', 'sim_data');
            fprintf('  Saved: %s\n', save_file);

            summary{end+1} = sprintf('%-55s  tot=%5.0f us  settle=%.3f  updates=%4d  div=%d', ...
                tag, med_tot, t_settle, n_updates, diverged); %#ok<SAGROW>

            clear ocp_solver ocp mdl;

        catch ME
            fprintf('  ERROR: %s\n', ME.message);
            summary{end+1} = sprintf('%-55s  ERROR: %s', tag, ME.message); %#ok<SAGROW>
        end
    end
end

%% ================================================================
%% Summary table
%% ================================================================

fprintf('\n\n==================== ADAPTIVE LMPC SUMMARY ====================\n');
for i = 1:numel(summary)
    fprintf('%s\n', summary{i});
end

%% ================================================================
%% Quick comparison plot (if results available)
%% ================================================================

fprintf('\n--- Generating comparison plot ---\n');

fig_dir = 'figures';
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

% Collect results for 3mm perturbation
ref_pert = 3;  % mm
K_labels = {};
settle_times = [];
solve_times  = [];
jac_times    = [];

for iK = 1:nK
    K = K_values(iK);
    if isinf(K), K_str = 'Inf'; else, K_str = num2str(K); end
    tag = sprintf('ALMPC_K%s_H%d_p%dmm', K_str, N_horizon, ref_pert);
    fpath = fullfile(output_dir, ['study_' tag '.mat']);

    if ~exist(fpath, 'file'), continue; end
    d = load(fpath);

    if d.diverged, continue; end

    K_labels{end+1} = sprintf('K=%s', K_str); %#ok<SAGROW>
    settle_times(end+1) = d.t_settle * 1e3; %#ok<SAGROW>
    solve_times(end+1)  = median(d.t_mpc) * 1e6; %#ok<SAGROW>
    jac_times(end+1)    = median(d.t_jac) * 1e6; %#ok<SAGROW>
end

if ~isempty(K_labels)
    font_name = 'Times New Roman';
    font_ax = 9; font_lab = 10; font_leg = 8;
    w_full = 17; h_tall = 12;

    fig = figure('Units','centimeters','Position',[2 2 w_full h_tall], ...
                 'PaperUnits','centimeters','PaperSize',[w_full h_tall], ...
                 'PaperPosition',[0 0 w_full h_tall], 'Color','w');

    % Top: Settling time vs K
    ax1 = subplot(2,1,1); hold on; grid on;
    bar(settle_times, 'FaceColor', [0.30 0.60 0.90]);
    xticks(1:numel(K_labels)); xticklabels(K_labels);
    ylabel('Settling time (ms)', 'Interpreter', 'latex');
    title(sprintf('Adaptive LMPC: relinearization period (N=%d, %d mm pert.)', ...
          N_horizon, ref_pert), 'Interpreter', 'latex');
    set(ax1, 'FontName', font_name, 'FontSize', font_ax, ...
        'TickDir', 'out', 'Box', 'on');

    % Bottom: Timing breakdown
    ax2 = subplot(2,1,2); hold on; grid on;
    b = bar([solve_times(:), jac_times(:)], 'stacked');
    b(1).FaceColor = [0.30 0.60 0.90]; b(1).DisplayName = 'QP solve';
    b(2).FaceColor = [0.90 0.50 0.20]; b(2).DisplayName = 'Jacobian eval';
    xticks(1:numel(K_labels)); xticklabels(K_labels);
    ylabel('Median total time ($\mu$s)', 'Interpreter', 'latex');
    xlabel('Relinearization period $K$', 'Interpreter', 'latex');
    yline(1000, 'k--', 'LineWidth', 1, 'Label', '1 ms limit', ...
          'FontSize', font_leg, 'LabelHorizontalAlignment', 'left');
    legend('Location', 'northwest', 'FontSize', font_leg);
    set(ax2, 'FontName', font_name, 'FontSize', font_ax, ...
        'TickDir', 'out', 'Box', 'on');

    exportgraphics(fig, fullfile(fig_dir, 'almpc_comparison.pdf'), ...
                   'ContentType', 'vector');
    fprintf('  Saved: %s\n', fullfile(fig_dir, 'almpc_comparison.pdf'));

    % Trajectory overlay — z position for all K values
    fig2 = figure('Units','centimeters','Position',[2 2 w_full 7], ...
                  'PaperUnits','centimeters','PaperSize',[w_full 7], ...
                  'PaperPosition',[0 0 w_full 7], 'Color','w');
    ax3 = axes; hold on; grid on;
    colors = lines(nK);
    ci = 0;
    for iK = 1:nK
        K = K_values(iK);
        if isinf(K), K_str = 'Inf'; else, K_str = num2str(K); end
        tag = sprintf('ALMPC_K%s_H%d_p%dmm', K_str, N_horizon, ref_pert);
        fpath = fullfile(output_dir, ['study_' tag '.mat']);
        if ~exist(fpath, 'file'), continue; end
        d = load(fpath);
        if d.diverged, continue; end
        ci = ci + 1;
        plot(d.t * 1e3, d.x(3,:) * 1e3, '-', 'Color', colors(ci,:), ...
             'LineWidth', 1.0, 'DisplayName', sprintf('K=%s', K_str));
    end
    ylabel('$z$ (mm)', 'Interpreter', 'latex');
    xlabel('Time (ms)', 'Interpreter', 'latex');
    title(sprintf('Adaptive LMPC trajectories (N=%d, %d mm pert.)', ...
          N_horizon, ref_pert), 'Interpreter', 'latex');
    legend('Location', 'best', 'FontSize', font_leg);
    set(ax3, 'FontName', font_name, 'FontSize', font_ax, ...
        'TickDir', 'out', 'Box', 'on');

    exportgraphics(fig2, fullfile(fig_dir, 'almpc_trajectories.pdf'), ...
                   'ContentType', 'vector');
    fprintf('  Saved: %s\n', fullfile(fig_dir, 'almpc_trajectories.pdf'));
end

fprintf('\n=== simMpcAdaptiveLMPC complete ===\n');
