%% simSolmpcAnalysis — SOL-MPC sweep: relinearization period K vs perturbation
%
% Sweeps relinearization period K = {1, 2, 5, 10, 20, 50, Inf}:
%   K = 1   : update every step  (= simSolmpc)
%   K = Inf : never update        (= static LMPC)
%
% Linearization uses the same correct formula as simSolmpc:
%   b = f(x,u) - Ac*x - Bc*u   (affine drift)
%   expm([[Ac,Bc,b]; zeros(nu+1,...)] * dt) -> Ad, Bd, c
%   p = [Ad(:); Bd(:); c(:)]
%
% Plant: get_accurate_sim_model().

simSetup;
import casadi.*

umax  = 1;
nx    = 10;
nu    = 4;
dt_mpc = 0.001;

output_dir = 'mpcresults';
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

%% --- Equilibria ---
x0  = x0([1:5, 7:11]);
xEq = xEq([1:5, 7:11]);

%% --- Cost weights ---
Q = diag([1e4, 1e4, 1e6, 1e3, 1e3, 1e2, 1e2, 1e3, 1e2, 1e2]);
R = eye(nu);

%% --- JIT-compiled linearization function: (x,u) -> [Ac(:); Bc(:); f0] ---
fprintf('\n--- Building linearization function ---\n');

x_cas = MX.sym('x', nx);
u_cas = MX.sym('u', nu);

modelId    = MaglevModel.Accurate;
params_lin = load_params(modelId);
params_lin.magnet.n       = 16;
params_lin.magnet.n_axial = 1;

f_expl     = maglevSystemDynamicsReduced_casadi(x_cas, u_cas, params_lin, modelId);
Ac_sym     = jacobian(f_expl, x_cas);
Bc_sym     = jacobian(f_expl, u_cas);

jit_opts   = struct('jit', true, 'jit_options', struct('flags', '-O2'));
lin_casadi = Function('lin_casadi_analysis', {x_cas, u_cas}, ...
                      {[Ac_sym(:); Bc_sym(:); f_expl]}, jit_opts);

% Helper: linearize + discretize at (xl, ul) -> (Ad, Bd, c)
function [Ad, Bd, c] = lin_disc(xl, ul, lin_casadi, dt, nx, nu)
    raw = full(lin_casadi(xl, ul));
    Ac  = reshape(raw(1       : nx*nx),       nx, nx);
    Bc  = reshape(raw(nx*nx+1 : nx*nx+nx*nu), nx, nu);
    f0  = raw(nx*nx+nx*nu+1 : end);
    b   = f0 - Ac*xl - Bc*ul;
    M   = expm([[Ac, Bc, b]; zeros(nu+1, nx+nu+1)] * dt);
    Ad  = M(1:nx, 1:nx);
    Bd  = M(1:nx, nx+1:nx+nu);
    c   = M(1:nx, end);
end

% Equilibrium linearization (used for initial params and K=Inf)
[Ad_eq, Bd_eq, c_eq] = lin_disc(xEq, uEq, lin_casadi, dt_mpc, nx, nu);
p0_eq = [Ad_eq(:); Bd_eq(:); c_eq(:)];
fprintf('  Discrete eigenvalues at eq (|z|): %s\n', mat2str(sort(abs(eig(Ad_eq)))', 4));

%% --- Plant solver ---
sim = AcadosSim();
sim.model = get_accurate_sim_model();
sim.solver_options.Tsim            = dt;
sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages      = 4;
sim.solver_options.num_steps       = 1;
plant_solver = AcadosSimSolver(sim);
fprintf('  Plant solver built.\n');

%% --- Sweep configuration ---
K_values      = [1, 2, 5, 10, 20, 50, Inf];
N_horizon     = 20;
qp_solver_str = 'PARTIAL_CONDENSING_HPIPM';
perturbations = [0.001, 0.003, 0.005];  % [m]

Tf        = dt_mpc * N_horizon;
t_sim_end = 2.0;

nK = numel(K_values);
nP = numel(perturbations);

fprintf('\n=== simSolmpcAnalysis: %d configs (%d K x %d pert) ===\n', nK*nP, nK, nP);

%% --- Parameterized OCP symbols (shared across configs) ---
% p = [Ad(:); Bd(:); c(:)]
np = nx*nx + nx*nu + nx;

x_sym = MX.sym('x', nx);
u_sym = MX.sym('u', nu);
p_sym = MX.sym('p', np);

p_Ad = reshape(p_sym(1          : nx*nx),       nx, nx);
p_Bd = reshape(p_sym(nx*nx+1    : nx*nx+nx*nu), nx, nu);
p_c  =         p_sym(nx*nx+nx*nu+1 : end);

disc_dyn = p_Ad * x_sym + p_Bd * u_sym + p_c;

%% --- Main sweep ---
summary = {};

for iK = 1:nK
    K = K_values(iK);
    K_str = num2str(K);
    if isinf(K), K_str = 'Inf'; end

    for iP = 1:nP
        pert_m  = perturbations(iP);
        pert_mm = round(pert_m * 1e3);

        tag       = sprintf('SOLMPC_K%s_H%d_p%dmm', K_str, N_horizon, pert_mm);
        save_file = fullfile(output_dir, ['study_' tag '.mat']);

        fprintf('\n--- [K=%s, pert=%dmm] ---\n', K_str, pert_mm);

        if exist(save_file, 'file')
            fprintf('  Already exists, skipping.\n');
            summary{end+1} = sprintf('%-55s  SKIPPED', tag); %#ok<SAGROW>
            continue;
        end

        try
            mdl = AcadosModel();
            mdl.name          = ['study_' tag];
            mdl.x             = x_sym;
            mdl.u             = u_sym;
            mdl.p             = p_sym;
            mdl.disc_dyn_expr = disc_dyn;

            ocp = AcadosOcp();
            ocp.model = mdl;

            ocp.solver_options.N_horizon             = N_horizon;
            ocp.solver_options.tf                    = Tf;
            ocp.solver_options.integrator_type       = 'DISCRETE';
            ocp.solver_options.nlp_solver_type       = 'SQP';
            ocp.solver_options.nlp_solver_max_iter   = 1;
            ocp.solver_options.qp_solver             = qp_solver_str;
            ocp.solver_options.ext_fun_compile_flags = '-O2';

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

            ocp.constraints.idxbu = 0:nu-1;
            ocp.constraints.lbu   = -umax * ones(nu, 1);
            ocp.constraints.ubu   =  umax * ones(nu, 1);

            x0_pert = [0; 0; zEq(1) + pert_m; zeros(7,1)];
            ocp.constraints.x0 = x0_pert;

            ocp.parameter_values = p0_eq;

            tic_build  = tic;
            ocp_solver = AcadosOcpSolver(ocp);
            t_build    = toc(tic_build);
            fprintf('  OCP build: %.1f s\n', t_build);

            % Warm-start
            ocp_solver.set('p', p0_eq);
            for k = 0:N_horizon,   ocp_solver.set('x', xEq, k); end
            for k = 0:N_horizon-1, ocp_solver.set('u', uEq, k); end

            % --- Simulation loop ---
            n_sub  = round(dt_mpc / dt);
            t_vec  = 0:dt:t_sim_end;
            N_sim  = numel(t_vec);
            N_mpc  = floor(N_sim / n_sub);

            x_sim      = zeros(nx, N_sim);
            u_sim      = zeros(nu, N_sim);
            t_mpc_log  = zeros(1, N_mpc);
            t_jac_log  = zeros(1, N_mpc);
            status_log = zeros(1, N_mpc);
            n_updates  = 0;

            x_cur    = x0_pert;
            u_cur    = uEq;
            diverged = false;
            n_mpc_run = N_mpc;

            % Current linearization (start at equilibrium)
            Ad_cur = Ad_eq;
            Bd_cur = Bd_eq;
            c_cur  = c_eq;

            for k = 1:N_mpc
                % --- Periodic relinearization at (x_cur, u_cur) ---
                t_jac_k = 0;
                if ~isinf(K) && (mod(k-1, K) == 0)
                    tic_jac = tic;
                    [Ad_cur, Bd_cur, c_cur] = lin_disc(x_cur, u_cur, lin_casadi, dt_mpc, nx, nu);
                    ocp_solver.set('p', [Ad_cur(:); Bd_cur(:); c_cur(:)]);
                    t_jac_k   = toc(tic_jac);
                    n_updates = n_updates + 1;
                end
                t_jac_log(k) = t_jac_k;

                % QP solve
                ocp_solver.set('constr_x0', x_cur);
                ocp_solver.solve();
                t_mpc_log(k)  = ocp_solver.get('time_tot');
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

                % Plant sub-steps
                for j = 1:n_sub
                    idx_s = (k-1)*n_sub + j;
                    if idx_s > N_sim, break; end
                    x_sim(:, idx_s) = x_cur;
                    u_sim(:, idx_s) = u_cur;
                    x_cur = plant_solver.simulate(x_cur, u_cur);
                end

                diverged = abs(x_cur(3)) > 0.5      || ...
                           max(abs(x_cur(4:5))) > pi || ...
                           any(isnan(x_cur))         || ...
                           any(isinf(x_cur));

                if diverged
                    fprintf('  DIVERGED at step %d\n', k);
                    n_mpc_run  = k;
                    last_idx   = min(k*n_sub, N_sim);
                    x_sim      = x_sim(:, 1:last_idx);
                    u_sim      = u_sim(:, 1:last_idx);
                    t_vec      = t_vec(1:last_idx);
                    t_mpc_log  = t_mpc_log(1:k);
                    t_jac_log  = t_jac_log(1:k);
                    status_log = status_log(1:k);
                    break;
                end
            end

            % --- Metrics ---
            z_target = zEq(1);
            if ~diverged && numel(t_vec) > 10
                z_final     = mean(x_sim(3, end-9:end));
                z_excursion = abs(x0_pert(3) - z_final);
                z_band      = max(0.02 * z_excursion, 1e-5);
                settled     = abs(x_sim(3,:) - z_final) < z_band;
                last_out    = find(~settled, 1, 'last');
                if isempty(last_out)
                    t_settle = 0;
                elseif last_out < numel(t_vec)
                    t_settle = t_vec(last_out);
                else
                    t_settle = NaN;
                end
                ss_err = mean(abs(x_sim(3, end-9:end) - z_target));
            else
                t_settle = NaN;
                ss_err   = NaN;
            end

            z_overshoot = max(0, max(abs(x_sim(3,:) - z_target)) - abs(x0_pert(3) - z_target));

            t_total_log = t_mpc_log + t_jac_log;
            med_mpc = median(t_mpc_log) * 1e6;
            med_jac = median(t_jac_log) * 1e6;
            med_tot = median(t_total_log) * 1e6;

            fprintf('  Steps=%d  mpc=%.0f+jac=%.0f=%.0f us  settle=%.3f s  updates=%d  div=%d\n', ...
                n_mpc_run, med_mpc, med_jac, med_tot, t_settle, n_updates, diverged);

            % Save
            sim_data            = struct();
            sim_data.t          = t_vec;
            sim_data.x          = x_sim;
            sim_data.u          = u_sim;
            sim_data.xEq        = xEq;
            sim_data.uEq        = uEq;
            sim_data.dt         = dt;
            sim_data.dt_mpc     = dt_mpc;
            sim_data.config     = struct('K_update', K, 'N_horizon', N_horizon, ...
                                         'pert_m', pert_m);
            sim_data.t_mpc      = t_mpc_log;
            sim_data.t_jac      = t_jac_log;
            sim_data.t_total    = t_total_log;
            sim_data.status     = status_log;
            sim_data.t_build    = t_build;
            sim_data.diverged   = diverged;
            sim_data.t_settle   = t_settle;
            sim_data.z_overshoot = z_overshoot;
            sim_data.ss_err     = ss_err;
            sim_data.n_updates  = n_updates;
            sim_data.n_mpc_run  = n_mpc_run;

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

%% --- Summary ---
fprintf('\n\n==================== SOL-MPC ANALYSIS SUMMARY ====================\n');
for i = 1:numel(summary)
    fprintf('%s\n', summary{i});
end

%% --- Comparison plot ---
fprintf('\n--- Generating comparison plot ---\n');

fig_dir = 'figures';
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

ref_pert   = 3;  % mm
K_labels   = {};
settle_times = [];
solve_times  = [];
jac_times    = [];

for iK = 1:nK
    K = K_values(iK);
    if isinf(K), K_str = 'Inf'; else, K_str = num2str(K); end
    tag   = sprintf('SOLMPC_K%s_H%d_p%dmm', K_str, N_horizon, ref_pert);
    fpath = fullfile(output_dir, ['study_' tag '.mat']);
    if ~exist(fpath, 'file'), continue; end
    d = load(fpath);
    if d.diverged, continue; end
    K_labels{end+1}      = sprintf('K=%s', K_str); %#ok<SAGROW>
    settle_times(end+1)  = d.t_settle * 1e3; %#ok<SAGROW>
    solve_times(end+1)   = median(d.t_mpc) * 1e6; %#ok<SAGROW>
    jac_times(end+1)     = median(d.t_jac) * 1e6; %#ok<SAGROW>
end

if ~isempty(K_labels)
    font_name = 'Times New Roman';
    font_ax = 9; font_lab = 10; font_leg = 8;
    w_full = 17; h_tall = 12;

    fig = figure('Units','centimeters','Position',[2 2 w_full h_tall], ...
                 'PaperUnits','centimeters','PaperSize',[w_full h_tall], ...
                 'PaperPosition',[0 0 w_full h_tall],'Color','w');

    ax1 = subplot(2,1,1); hold on; grid on;
    bar(settle_times, 'FaceColor', [0.30 0.60 0.90]);
    xticks(1:numel(K_labels)); xticklabels(K_labels);
    ylabel('Settling time (ms)', 'Interpreter','latex');
    title(sprintf('SOL-MPC: relinearization period (N=%d, %d mm pert.)', N_horizon, ref_pert), ...
          'Interpreter','latex');
    set(ax1,'FontName',font_name,'FontSize',font_ax,'TickDir','out','Box','on');

    ax2 = subplot(2,1,2); hold on; grid on;
    b_bar = bar([solve_times(:), jac_times(:)], 'stacked');
    b_bar(1).FaceColor = [0.30 0.60 0.90]; b_bar(1).DisplayName = 'QP solve';
    b_bar(2).FaceColor = [0.90 0.50 0.20]; b_bar(2).DisplayName = 'Jacobian eval';
    xticks(1:numel(K_labels)); xticklabels(K_labels);
    ylabel('Median total time ($\mu$s)', 'Interpreter','latex');
    xlabel('Relinearization period $K$', 'Interpreter','latex');
    yline(1000,'k--','LineWidth',1,'Label','1 ms limit','FontSize',font_leg, ...
          'LabelHorizontalAlignment','left');
    legend('Location','northwest','FontSize',font_leg);
    set(ax2,'FontName',font_name,'FontSize',font_ax,'TickDir','out','Box','on');

    exportgraphics(fig, fullfile(fig_dir,'solmpc_K_comparison.pdf'),'ContentType','vector');
    fprintf('  Saved: solmpc_K_comparison.pdf\n');

    % Trajectory overlay
    fig2 = figure('Units','centimeters','Position',[2 2 w_full 7], ...
                  'PaperUnits','centimeters','PaperSize',[w_full 7], ...
                  'PaperPosition',[0 0 w_full 7],'Color','w');
    ax3 = axes; hold on; grid on;
    colors = lines(nK);
    ci = 0;
    for iK = 1:nK
        K = K_values(iK);
        if isinf(K), K_str = 'Inf'; else, K_str = num2str(K); end
        tag   = sprintf('SOLMPC_K%s_H%d_p%dmm', K_str, N_horizon, ref_pert);
        fpath = fullfile(output_dir, ['study_' tag '.mat']);
        if ~exist(fpath,'file'), continue; end
        d = load(fpath);
        if d.diverged, continue; end
        ci = ci + 1;
        plot(d.t*1e3, d.x(3,:)*1e3, '-', 'Color', colors(ci,:), ...
             'LineWidth', 1.0, 'DisplayName', sprintf('K=%s', K_str));
    end
    yline(zEq(1)*1e3, 'k--', 'LineWidth', 0.8);
    ylabel('$z$ (mm)','Interpreter','latex');
    xlabel('Time (ms)','Interpreter','latex');
    title(sprintf('SOL-MPC trajectories (N=%d, %d mm pert.)', N_horizon, ref_pert), ...
          'Interpreter','latex');
    legend('Location','best','FontSize',font_leg);
    set(ax3,'FontName',font_name,'FontSize',font_ax,'TickDir','out','Box','on');

    exportgraphics(fig2, fullfile(fig_dir,'solmpc_K_trajectories.pdf'),'ContentType','vector');
    fprintf('  Saved: solmpc_K_trajectories.pdf\n');
end

fprintf('\n=== simSolmpcAnalysis complete ===\n');
