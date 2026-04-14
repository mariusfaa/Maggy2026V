%% degStudy — Rotation-limit (pitch angle) study
%
% Sweeps the initial pitch angle θ from 0° to 60° in 10° steps with a
% fixed Δz = +2 mm offset.  All four controllers (LQR, LMPC, SOL-NMPC,
% NMPC) are run with N = 20, Δt_mpc = 1 ms, T_sim = 0.5 s.
%
% If the summary .mat file already exists the simulation is skipped and
% only the plots are regenerated.
%
% Outputs:
%   results_degstudy/deg_study_summary.mat  — summary struct
%   results_degstudy/<ctrl>_1ms_N20_theta<deg*10>.mat — per-run files
%   Master_thesis_2025/figures/             — thesis-quality figures
%
% See also: magnetNStudy, integratorStudy

clear; clc; close all;

%% ================================================================
%% Path setup
%% ================================================================
acados_root  = getenv('ACADOS_INSTALL_DIR');
project_root = getenv('ACADOS_PROJECT_DIR');

assert(~isempty(acados_root),  'ACADOS_INSTALL_DIR not set. Source env.sh first.');
assert(~isempty(project_root), 'ACADOS_PROJECT_DIR not set. Source env.sh first.');

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));
addpath(genpath(fullfile(project_root, 'model_matlab')));
addpath(genpath(fullfile(project_root, 'model_reduced_casadi')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

cd(project_root);

summary_path = fullfile('results_degstudy', 'deg_study_summary.mat');

%% ================================================================
%% Run simulation or load existing results
%% ================================================================
if exist(summary_path, 'file')
    fprintf('=== Loading existing results from %s ===\n', summary_path);
    S = load(summary_path);
else
    fprintf('=== No existing results — running simulation sweep ===\n');
    run_deg_study_sim(project_root);
    S = load(summary_path);
end

%% ================================================================
%% Unpack summary
%% ================================================================
summary        = S.summary;
theta_deg_list = S.theta_deg_list;
controllers    = S.controllers;
N_horizon      = S.N_horizon;
dt_mpc         = S.dt_mpc;
T_sim          = S.T_sim;
dz             = S.dz;

n_ctrl  = numel(controllers);
n_theta = numel(theta_deg_list);

%% ================================================================
%% Build data matrices
%% ================================================================
% diverged_mat(i_ctrl, i_theta): logical, true if diverged
% settling_mat(i_ctrl, i_theta): settling time (s), NaN if diverged
% cost_mat(i_ctrl, i_theta):     final cumulative cost, NaN if diverged
diverged_mat  = true(n_ctrl, n_theta);
settling_mat  = nan(n_ctrl, n_theta);
cost_mat      = nan(n_ctrl, n_theta);
ctrl_labels   = {'LQR', 'LMPC', 'SOL-NMPC', 'NMPC'};

for i_c = 1:n_ctrl
    ctrl = controllers{i_c};
    mask = strcmp({summary.controller}, ctrl);
    thetas   = [summary(mask).theta_deg];
    divflags = [summary(mask).diverged];
    for i_t = 1:n_theta
        idx = find(abs(thetas - theta_deg_list(i_t)) < 0.01, 1);
        if isempty(idx), continue; end
        diverged_mat(i_c, i_t) = divflags(idx);
        if ~divflags(idx)
            s = summary(mask);
            fpath = s(idx).file;
            if isfile(fpath)
                d = load(fpath);
                if isfield(d, 'x_sim') && isfield(d, 't_sim')
                    xs = d.x_sim;  ts = d.t_sim;
                elseif isfield(d, 'x') && isfield(d, 't')
                    xs = d.x;  ts = d.t;
                else
                    continue;
                end
                if size(xs,1) > size(xs,2), xs = xs'; end
                if size(ts,2) == 1, ts = ts'; end
                xEq = d.xEq;
                pos_err = vecnorm(xs(1:3,:) - xEq(1:3), 2, 1) * 1e3;
                threshold = 0.1;  % mm
                settled_idx = find(pos_err < threshold, 1, 'first');
                if ~isempty(settled_idx)
                    settling_mat(i_c, i_t) = ts(settled_idx);
                else
                    settling_mat(i_c, i_t) = NaN;
                end
                if isfield(d, 'cost_cum')
                    cost_mat(i_c, i_t) = d.cost_cum(end);
                end
            end
        end
    end
end

%% ================================================================
%% Critical angle per controller
%% ================================================================
fprintf('\n--- Critical angle (first divergence) per controller ---\n');
crit_angle = nan(1, n_ctrl);
for i_c = 1:n_ctrl
    first_div = find(diverged_mat(i_c,:), 1, 'first');
    if isempty(first_div)
        fprintf('  %-10s: stable up to %.1f deg (max tested)\n', ...
            ctrl_labels{i_c}, max(theta_deg_list));
        crit_angle(i_c) = max(theta_deg_list) + 10;
    else
        crit = theta_deg_list(first_div);
        crit_angle(i_c) = crit;
        if first_div > 1
            last_ok = theta_deg_list(first_div - 1);
            fprintf('  %-10s: diverges at %.1f deg  (last stable: %.1f deg)\n', ...
                ctrl_labels{i_c}, crit, last_ok);
        else
            fprintf('  %-10s: diverges at %.1f deg  (unstable from start)\n', ...
                ctrl_labels{i_c}, crit);
        end
    end
end

%% ================================================================
%% Print summary table
%% ================================================================
fprintf('\n=== Degree Study Summary (N=%d, dt=%.0f ms, Dz=+%.0f mm, T=%.1f s) ===\n', ...
    N_horizon, dt_mpc*1e3, dz*1e3, T_sim);
fprintf('\n%-10s', 'theta(deg)');
for i_c = 1:n_ctrl
    fprintf('  %10s', ctrl_labels{i_c});
end
fprintf('\n%s\n', repmat('-', 1, 10 + n_ctrl*12));
for i_t = 1:n_theta
    fprintf('%-10.1f', theta_deg_list(i_t));
    for i_c = 1:n_ctrl
        if diverged_mat(i_c, i_t)
            fprintf('  %10s', 'DIVERGED');
        elseif isnan(settling_mat(i_c, i_t))
            fprintf('  %10s', 'OK');
        else
            fprintf('  %8.1f ms', settling_mat(i_c, i_t)*1e3);
        end
    end
    fprintf('\n');
end

%% ================================================================
%% Generate figures
%% ================================================================
fprintf('\n=== Generating figures ===\n');

fig_dir = fullfile(project_root, 'Master_thesis_2025', 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

W_FULL = 17;   H_STD = 7;   H_TALL = 9;
FONT_NAME = 'Times New Roman';
FONT_AX = 9;  FONT_LAB = 10;  FONT_LEG = 8;  FONT_TITLE = 10;

C_LQR  = [0.50 0.50 0.50];
C_LMPC = [0.00 0.45 0.74];
C_SOL  = [0.85 0.33 0.10];
C_NMPC = [0.47 0.67 0.19];
ctrl_colors = [C_LQR; C_LMPC; C_SOL; C_NMPC];

%% --- Figure 1: Feasibility heatmap ---
fig1 = mkfig(W_FULL, H_TALL);
feas_img = double(~diverged_mat);
imagesc(theta_deg_list, 1:n_ctrl, feas_img);
colormap([0.90 0.30 0.30; 0.30 0.75 0.40]);
set(gca, 'YTick', 1:n_ctrl, 'YTickLabel', ctrl_labels);
set(gca, 'XTick', theta_deg_list(1:2:end));
xlabel('Initial pitch angle (deg)', 'FontSize', FONT_LAB);
title(sprintf('Controller feasibility vs. initial pitch angle (N=%d, dt=%d ms)', ...
    N_horizon, dt_mpc*1e3), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
hold on;
for i_c = 1:n_ctrl
    for i_t = 1:n_theta
        if diverged_mat(i_c, i_t)
            txt = '\times';
        else
            txt = '\checkmark';
        end
        text(theta_deg_list(i_t), i_c, txt, ...
            'HorizontalAlignment', 'center', 'FontSize', 8, ...
            'Interpreter', 'tex', 'Color', 'w', 'FontWeight', 'bold');
    end
end
hold off;
cb = colorbar;
cb.Ticks = [0.25 0.75];
cb.TickLabels = {'Diverged', 'Stable'};
cb.FontSize = FONT_AX;
style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
savefig_thesis(fig1, 'deg_feasibility_heatmap', fig_dir);

%% --- Figure 2: Critical angle bar chart ---
fig2 = mkfig(W_FULL, H_STD);
bar_data = min(crit_angle, max(theta_deg_list));
b = bar(1:n_ctrl, bar_data, 0.6);
b.FaceColor = 'flat';
for i_c = 1:n_ctrl
    b.CData(i_c,:) = ctrl_colors(i_c,:);
end
set(gca, 'XTick', 1:n_ctrl, 'XTickLabel', ctrl_labels);
ylabel('Critical pitch angle (deg)', 'FontSize', FONT_LAB);
title('Maximum stable initial pitch angle per controller', ...
    'FontSize', FONT_TITLE, 'FontWeight', 'bold');
ylim([0, max(theta_deg_list) + 5]);
grid on; box on;
hold on;
for i_c = 1:n_ctrl
    if crit_angle(i_c) > max(theta_deg_list)
        lbl = sprintf('>%.0f deg', max(theta_deg_list));
    else
        lbl = sprintf('%.1f deg', crit_angle(i_c));
    end
    text(i_c, bar_data(i_c) + 1.5, lbl, ...
        'HorizontalAlignment', 'center', 'FontSize', FONT_AX, ...
        'FontName', FONT_NAME);
end
hold off;
style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
savefig_thesis(fig2, 'deg_critical_angle', fig_dir);

%% --- Figure 3: Settling time vs angle ---
fig3 = mkfig(W_FULL, H_STD);
hold on; grid on; box on;
markers = {'s', 'o', 'd', '^'};
for i_c = 1:n_ctrl
    valid = ~diverged_mat(i_c,:) & ~isnan(settling_mat(i_c,:));
    if any(valid)
        plot(theta_deg_list(valid), settling_mat(i_c, valid)*1e3, ...
            ['-' markers{i_c}], 'Color', ctrl_colors(i_c,:), ...
            'LineWidth', 1.5, 'MarkerSize', 5, ...
            'MarkerFaceColor', ctrl_colors(i_c,:), ...
            'DisplayName', ctrl_labels{i_c});
    end
end
xlabel('Initial pitch angle (deg)', 'FontSize', FONT_LAB);
ylabel('Settling time (ms)', 'FontSize', FONT_LAB);
title(sprintf('Settling time vs. initial pitch (N=%d, dt=%d ms)', ...
    N_horizon, dt_mpc*1e3), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', FONT_LEG);
style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
savefig_thesis(fig3, 'deg_settling_time', fig_dir);

%% --- Figure 4: Cumulative cost vs angle ---
fig4 = mkfig(W_FULL, H_STD);
hold on; grid on; box on;
for i_c = 1:n_ctrl
    valid = ~diverged_mat(i_c,:) & ~isnan(cost_mat(i_c,:));
    if any(valid)
        semilogy(theta_deg_list(valid), cost_mat(i_c, valid), ...
            ['-' markers{i_c}], 'Color', ctrl_colors(i_c,:), ...
            'LineWidth', 1.5, 'MarkerSize', 5, ...
            'MarkerFaceColor', ctrl_colors(i_c,:), ...
            'DisplayName', ctrl_labels{i_c});
    end
end
xlabel('Initial pitch angle (deg)', 'FontSize', FONT_LAB);
ylabel('Cumulative cost', 'FontSize', FONT_LAB);
title(sprintf('Cumulative cost vs. initial pitch (N=%d, dt=%d ms)', ...
    N_horizon, dt_mpc*1e3), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', FONT_LEG);
style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
savefig_thesis(fig4, 'deg_cumulative_cost', fig_dir);

fprintf('\n=== Degree study complete ===\n');
fprintf('Figures saved to: %s\n', fig_dir);

%% ================================================================
%% Local helper functions
%% ================================================================
function fig = mkfig(w_cm, h_cm)
    fig = figure('Units', 'centimeters', ...
                 'Position', [2 2 w_cm h_cm], ...
                 'PaperUnits', 'centimeters', ...
                 'PaperSize', [w_cm h_cm], ...
                 'PaperPosition', [0 0 w_cm h_cm], ...
                 'Color', 'w');
end

function style_ax(ax, fn, fs_ax, fs_lab)
    set(ax, 'FontName', fn, 'FontSize', fs_ax, ...
            'TickDir', 'out', 'TickLength', [0.015 0.015], ...
            'LineWidth', 0.6, 'Box', 'on');
    set(ax.XLabel, 'FontSize', fs_lab, 'FontName', fn);
    set(ax.YLabel, 'FontSize', fs_lab, 'FontName', fn);
end

function savefig_thesis(fig, name, out_dir)
    pdf_path = fullfile(out_dir, [name '.pdf']);
    exportgraphics(fig, pdf_path, 'ContentType', 'vector');
    fprintf('  Saved: %s\n', pdf_path);
    png_path = fullfile(out_dir, [name '.png']);
    exportgraphics(fig, png_path, 'Resolution', 300);
end

%% ================================================================
%% Simulation runner (called only when summary .mat is missing)
%% ================================================================
function run_deg_study_sim(project_root)
%RUN_DEG_STUDY_SIM  Build each controller solver ONCE, then sweep angles.
    import casadi.*

    controllers = {'lqr', 'lmpc', 'solnmpc', 'nmpc'};
    out_folder  = fullfile(project_root, 'results_degstudy');
    N_horizon = 30;
    dt_mpc    = 0.001;
    T_sim     = 0.5;
    dt_plant  = 0.0001;
    dz        = 0.002;
    theta_deg_list = 0 : 10 : 60;
    theta_rad_list = deg2rad(theta_deg_list);
    n_theta = numel(theta_deg_list);
    skip_existing = true;

    modelId = MaglevModel.Accurate;
    parameters_maggy_V4;
    nx = 10;  nu = 4;
    params_acc = params;
    params_acc.magnet.n       = 80;
    params_acc.magnet.n_axial = 21;
    [zEq, ~, ~, ~] = computeSystemEquilibria(params_acc, modelId);
    xEq_full = [0; 0; zEq(1); zeros(9,1)];
    xEq = xEq_full([1:5, 7:11]);
    uEq = zeros(nu, 1);

    plant_model = getSimModel(32);
    ctrl_model  = getSimModel(16, 1);
    sim_solver  = getSimSolver(plant_model, dt_plant);
    if ~exist(out_folder, 'dir'); mkdir(out_folder); end
    if ~exist('build', 'dir');    mkdir('build');     end

    n_sub = round(dt_mpc / dt_plant);
    t_vec = 0 : dt_plant : T_sim;
    N_sim = numel(t_vec);
    N_mpc = floor(N_sim / n_sub);
    make_fname = @(ctrl, theta_deg) ...
        sprintf('%s_1ms_N20_theta%04d.mat', ctrl, round(theta_deg * 10));
    x0_dummy = xEq;
    x0_dummy(3) = xEq(3) + dz;

    %% BUILD CONTROLLERS (once each)
    fprintf('\n===== Building all controller solvers =====\n');

    % LQR
    fprintf('\n--- [LQR] Linearize + DARE ---\n');
    x_cas = plant_model.x;  u_cas = plant_model.u;
    f_expl = plant_model.f_expl_expr;
    jac_fun = Function('jac_fun', {x_cas, u_cas}, ...
                       {jacobian(f_expl, x_cas), jacobian(f_expl, u_cas)});
    [Ac_val, Bc_val] = jac_fun(xEq, uEq);
    Ac = full(Ac_val);  Bc = full(Bc_val);
    M_exp = expm([Ac Bc; zeros(nu, nx+nu)] * dt_mpc);
    Ad = M_exp(1:nx, 1:nx);  Bd = M_exp(1:nx, nx+1:end);
    cost_obj = getCost(xEq, uEq, dt_mpc);
    W = cost_obj.W;
    Q = W(1:nx, 1:nx);  R = W(nx+1:nx+nu, nx+1:nx+nu);
    [K_lqr, P_lqr, ~] = dlqr(Ad, Bd, Q, R);
    umax = 1.0;
    fprintf('  LQR gain computed.\n');

    % LMPC
    fprintf('\n--- [LMPC] Building OCP solver ---\n');
    c_offset = (eye(nx) - Ad) * xEq;
    x_sym = MX.sym('x', nx);  u_sym = MX.sym('u', nu);
    mdl_lmpc = AcadosModel();
    mdl_lmpc.name = 'maglev_lmpc';
    mdl_lmpc.x = x_sym;  mdl_lmpc.u = u_sym;
    mdl_lmpc.disc_dyn_expr = Ad * x_sym + Bd * u_sym + c_offset;
    ocp_lmpc = AcadosOcp();
    ocp_lmpc.model = mdl_lmpc;
    ocp_lmpc.solver_options.N_horizon             = N_horizon;
    ocp_lmpc.solver_options.tf                    = dt_mpc * N_horizon;
    ocp_lmpc.solver_options.integrator_type       = 'DISCRETE';
    ocp_lmpc.solver_options.nlp_solver_type       = 'SQP';
    ocp_lmpc.solver_options.nlp_solver_max_iter   = 1;
    ocp_lmpc.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    ocp_lmpc.solver_options.ext_fun_compile_flags = '-O2';
    ocp_lmpc.cost        = getCost(xEq, uEq, dt_mpc);
    ocp_lmpc.constraints = getConstraints(x0_dummy);
    solver_dir_lmpc = fullfile('build', 'lmpc');
    ocp_lmpc.code_gen_opts.code_export_directory = fullfile(solver_dir_lmpc, 'c_generated_code');
    ocp_lmpc.code_gen_opts.json_file = fullfile(solver_dir_lmpc, [mdl_lmpc.name '_ocp.json']);
    lmpc_solver = AcadosOcpSolver(ocp_lmpc, struct('output_dir', solver_dir_lmpc));
    fprintf('  LMPC solver built.\n');

    % SOL-NMPC
    fprintf('\n--- [SOL-NMPC] Building OCP solver ---\n');
    x_cas_c = ctrl_model.x;  u_cas_c = ctrl_model.u;
    f_expl_c = ctrl_model.f_expl_expr;
    Ac_sym = jacobian(f_expl_c, x_cas_c);
    Bc_sym = jacobian(f_expl_c, u_cas_c);
    b_sym  = f_expl_c - Ac_sym * x_cas_c - Bc_sym * u_cas_c;
    I_nx   = MX.eye(nx);
    Lhs    = I_nx - Ac_sym * (dt_mpc / 2);
    Ad_sym = solve(Lhs, I_nx + Ac_sym * (dt_mpc / 2));
    Bd_sym = solve(Lhs, Bc_sym * dt_mpc);
    c_sym  = solve(Lhs, b_sym  * dt_mpc);
    discretize_fn = Function('discretize', {x_cas_c, u_cas_c}, ...
        {vertcat(Ad_sym(:), Bd_sym(:), c_sym)});
    np = nx*nx + nx*nu + nx;
    x_sym2 = MX.sym('x', nx);  u_sym2 = MX.sym('u', nu);
    p_sym  = MX.sym('p', np);
    p_Ad = reshape(p_sym(1:nx*nx), nx, nx);
    p_Bd = reshape(p_sym(nx*nx+1:nx*nx+nx*nu), nx, nu);
    p_c  = p_sym(nx*nx+nx*nu+1:end);
    mdl_sol = AcadosModel();
    mdl_sol.name = 'maglev_solnmpc';
    mdl_sol.x = x_sym2;  mdl_sol.u = u_sym2;  mdl_sol.p = p_sym;
    mdl_sol.disc_dyn_expr = p_Ad * x_sym2 + p_Bd * u_sym2 + p_c;
    ocp_sol = AcadosOcp();
    ocp_sol.model = mdl_sol;
    ocp_sol.solver_options.N_horizon             = N_horizon;
    ocp_sol.solver_options.tf                    = dt_mpc * N_horizon;
    ocp_sol.solver_options.integrator_type       = 'DISCRETE';
    ocp_sol.solver_options.nlp_solver_type       = 'SQP';
    ocp_sol.solver_options.nlp_solver_max_iter   = 1;
    ocp_sol.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    ocp_sol.solver_options.ext_fun_compile_flags = '-O2';
    ocp_sol.cost        = getCost(xEq, uEq, dt_mpc);
    ocp_sol.constraints = getConstraints(x0_dummy);
    p0 = full(discretize_fn(xEq, uEq));
    ocp_sol.parameter_values = p0;
    solver_dir_sol = fullfile('build', 'solnmpc');
    ocp_sol.code_gen_opts.code_export_directory = fullfile(solver_dir_sol, 'c_generated_code');
    ocp_sol.code_gen_opts.json_file = fullfile(solver_dir_sol, [mdl_sol.name '_ocp.json']);
    sol_solver = AcadosOcpSolver(ocp_sol, struct('output_dir', solver_dir_sol));
    fprintf('  SOL-NMPC solver built.\n');

    % NMPC
    fprintf('\n--- [NMPC] Building OCP solver ---\n');
    ocp_nmpc = AcadosOcp();
    ocp_nmpc.model = ctrl_model;
    ocp_nmpc.solver_options.N_horizon             = N_horizon;
    ocp_nmpc.solver_options.tf                    = dt_mpc * N_horizon;
    ocp_nmpc.solver_options.ext_fun_compile_flags = '-O2';
    ocp_nmpc.solver_options.integrator_type       = 'IRK';
    ocp_nmpc.solver_options.sim_method_num_stages = 4;
    ocp_nmpc.solver_options.sim_method_num_steps  = 1;
    ocp_nmpc.solver_options.nlp_solver_type       = 'SQP'; % may use sqp_rti
    ocp_nmpc.solver_options.nlp_solver_tol_stat   = 1e-4;
    ocp_nmpc.solver_options.nlp_solver_tol_eq     = 1e-4;
    ocp_nmpc.solver_options.nlp_solver_tol_ineq   = 1e-4;
    ocp_nmpc.solver_options.nlp_solver_tol_comp   = 1e-4;
    ocp_nmpc.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    ocp_nmpc.cost        = getCost(xEq, uEq, dt_mpc);
    ocp_nmpc.constraints = getConstraints(x0_dummy);
    solver_dir_nmpc = fullfile('build', 'nmpc');
    ocp_nmpc.code_gen_opts.code_export_directory = fullfile(solver_dir_nmpc, 'c_generated_code');
    ocp_nmpc.code_gen_opts.json_file = fullfile(solver_dir_nmpc, [ctrl_model.name '_ocp.json']);
    nmpc_solver = AcadosOcpSolver(ocp_nmpc, struct('output_dir', solver_dir_nmpc));
    fprintf('  NMPC solver built.\n');

    fprintf('\n===== All solvers built — starting angle sweep =====\n');

    %% ANGLE SWEEP (reuse solvers, vary x0 only)
    n_ctrl  = numel(controllers);
    n_total = n_ctrl * n_theta;
    run_count = 0;
    summary = struct('controller', {}, 'theta_deg', {}, 'theta_rad', {}, ...
                     'diverged', {}, 'file', {});
    for i_c = 1:n_ctrl
        ctrl = controllers{i_c};
        fprintf('\n===== Controller: %s =====\n', upper(ctrl));
        for i_theta = 1:n_theta
            theta_deg = theta_deg_list(i_theta);
            theta_rad = theta_rad_list(i_theta);
            run_count = run_count + 1;
            fname     = make_fname(ctrl, theta_deg);
            save_file = fullfile(out_folder, fname);
            if skip_existing && isfile(save_file)
                fprintf('[%3d/%d] SKIP  %6s | theta = %5.1f deg (exists)\n', ...
                    run_count, n_total, upper(ctrl), theta_deg);
                idx = numel(summary) + 1;
                d = load(save_file);
                summary(idx).controller = ctrl;
                summary(idx).theta_deg  = theta_deg;
                summary(idx).theta_rad  = theta_rad;
                summary(idx).diverged   = d.diverged;
                summary(idx).file       = save_file;
                continue;
            end
            fprintf('[%3d/%d]  %6s | theta = %5.1f deg ... ', ...
                run_count, n_total, upper(ctrl), theta_deg);
            dx0 = [0; 0; dz; 0; theta_rad; 0; zeros(6,1)];
            x0_full = xEq_full + dx0;
            x0 = x0_full([1:5, 7:11]);
            switch ctrl
                case 'lqr'
                    sim_data = run_lqr(x0, xEq, uEq, K_lqr, umax, ...
                        sim_solver, t_vec, n_sub, N_sim, N_mpc, nx, nu, ...
                        dt_plant, dt_mpc, N_horizon, Ad, Bd, Q, R, P_lqr);
                case 'lmpc'
                    sim_data = run_mpc(lmpc_solver, 'lmpc', x0, xEq, uEq, ...
                        sim_solver, t_vec, n_sub, N_sim, N_mpc, nx, nu, ...
                        dt_plant, dt_mpc, N_horizon, [], Ad, Bd);
                case 'solnmpc'
                    sim_data = run_mpc(sol_solver, 'solnmpc', x0, xEq, uEq, ...
                        sim_solver, t_vec, n_sub, N_sim, N_mpc, nx, nu, ...
                        dt_plant, dt_mpc, N_horizon, discretize_fn, [], []);
                case 'nmpc'
                    sim_data = run_mpc(nmpc_solver, 'nmpc', x0, xEq, uEq, ...
                        sim_solver, t_vec, n_sub, N_sim, N_mpc, nx, nu, ...
                        dt_plant, dt_mpc, N_horizon, [], [], []);
            end
            save(save_file, '-struct', 'sim_data');
            idx = numel(summary) + 1;
            summary(idx).controller = ctrl;
            summary(idx).theta_deg  = theta_deg;
            summary(idx).theta_rad  = theta_rad;
            summary(idx).diverged   = sim_data.diverged;
            summary(idx).file       = save_file;
            if sim_data.diverged
                fprintf('DIVERGED\n');
            else
                fprintf('OK  (cost=%.2e)\n', sim_data.cost_cum(end));
            end
        end
    end
    fprintf('\n========== Sweep complete: %d runs ==========\n', n_total);
    save(fullfile(out_folder, 'deg_study_summary.mat'), ...
        'summary', 'theta_deg_list', 'theta_rad_list', ...
        'N_horizon', 'dt_mpc', 'T_sim', 'dz', 'controllers');
    fprintf('Summary saved to: %s\n', fullfile(out_folder, 'deg_study_summary.mat'));
end

%% ====================================================================
%  run_lqr — simulate one trajectory with pre-computed LQR gain
%  ====================================================================
function sim_data = run_lqr(x0, xEq, uEq, K, umax, ...
    sim_solver, t_vec, n_sub, N_sim, N_mpc, nx, nu, ...
    dt_plant, dt_mpc, N_horizon, Ad, Bd, Q, R, P)
    x_sim = zeros(nx, N_sim);
    u_sim = zeros(nu, N_sim);
    ctrl_time     = zeros(1, N_mpc);
    sim_time_tot  = zeros(1, N_mpc);
    step_time_tot = zeros(1, N_mpc);
    x = x0;  u = uEq;  diverged = false;  t_out = t_vec;
    for k = 1:N_mpc
        tic_ctrl = tic;
        u = -K * (x - xEq) + uEq;
        u = max(-umax, min(umax, u));
        ctrl_time(k) = toc(tic_ctrl);
        tacc = 0;
        for j = 1:n_sub
            idx = (k-1)*n_sub + j;
            if idx > N_sim, break; end
            x_sim(:, idx) = x;  u_sim(:, idx) = u;
            x = sim_solver.simulate(x, u);
            tacc = tacc + sim_solver.get('time_tot');
        end
        sim_time_tot(k)  = tacc;
        step_time_tot(k) = sim_time_tot(k) + ctrl_time(k);
        if isDiverged(x)
            diverged = true;
            last_idx = min(k*n_sub, N_sim);
            x_sim = x_sim(:, 1:last_idx);  u_sim = u_sim(:, 1:last_idx);
            t_out = t_vec(1:last_idx);
            ctrl_time = ctrl_time(1:k);
            sim_time_tot = sim_time_tot(1:k);
            step_time_tot = step_time_tot(1:k);
            break;
        end
    end
    if ~diverged
        filled = N_mpc * n_sub;
        x_sim = x_sim(:, 1:filled);  u_sim = u_sim(:, 1:filled);
        t_out = t_vec(1:filled);
    end
    sim_data.controller = 'lqr';
    sim_data.t = t_out;  sim_data.x = x_sim;  sim_data.u = u_sim;
    sim_data.x0 = x0;  sim_data.xEq = xEq;  sim_data.uEq = uEq;
    sim_data.dt = dt_plant;  sim_data.dt_mpc = dt_mpc;
    sim_data.N_horizon = N_horizon;  sim_data.diverged = diverged;
    sim_data.ctrl_time = ctrl_time;
    sim_data.sim_time_tot = sim_time_tot;
    sim_data.step_time_tot = step_time_tot;
    sim_data.cost = computeCost(x_sim - xEq, u_sim - uEq);
    sim_data.cost_cum = cumsum(sim_data.cost);
    sim_data.Ad = Ad;  sim_data.Bd = Bd;
    sim_data.K = K;  sim_data.P = P;  sim_data.Q = Q;  sim_data.R = R;
end

%% ====================================================================
%  run_mpc — simulate one trajectory with a pre-built acados OCP solver
%  ====================================================================
function sim_data = run_mpc(ocp_solver, ctrl_name, x0, xEq, uEq, ...
    sim_solver, t_vec, n_sub, N_sim, N_mpc, nx, nu, ...
    dt_plant, dt_mpc, N_horizon, discretize_fn, Ad_save, Bd_save)
    is_sol = strcmp(ctrl_name, 'solnmpc');
    % Reset warm-start to equilibrium before each trajectory
    if is_sol
        p0 = full(discretize_fn(xEq, uEq));
        ocp_solver.set('p', p0);
    end
    for kk = 0:N_horizon
        ocp_solver.set('x', xEq, kk);
    end
    for kk = 0:N_horizon-1
        ocp_solver.set('u', uEq, kk);
    end
    % Warm-up solves: run several SQP iterations at the initial x0 so
    % the shooting nodes converge before the closed-loop sim starts.
    % Without this, SQP_RTI (1 iter) diverges at large angles because
    % the equilibrium warm-start is too far from the actual trajectory.
    n_warmup = 5;
    ocp_solver.set('constr_x0', x0);
    for ww = 1:n_warmup
        if is_sol
            p_new = full(discretize_fn(x0, uEq));
            ocp_solver.set('p', p_new);
        end
        ocp_solver.solve();
    end
    x_sim = zeros(nx, N_sim);  u_sim = zeros(nu, N_sim);
    ocp_time_tot = zeros(1, N_mpc);  ocp_time_lin = zeros(1, N_mpc);
    ocp_time_reg = zeros(1, N_mpc);  ocp_time_sim = zeros(1, N_mpc);
    ocp_time_glob = zeros(1, N_mpc); ocp_time_qp = zeros(1, N_mpc);
    ocp_time_qp_xcond = zeros(1, N_mpc);
    ocp_qp_iter = zeros(1, N_mpc);   ocp_sqp_iter = zeros(1, N_mpc);
    ocp_nlp_iter = zeros(1, N_mpc);
    ocp_residuals = zeros(4, N_mpc);
    ocp_status = zeros(1, N_mpc);    ocp_cost = zeros(1, N_mpc);
    jac_time = zeros(1, N_mpc);
    sim_time_tot = zeros(1, N_mpc);  step_time_tot = zeros(1, N_mpc);
    x = x0;  u = uEq;  diverged = false;  t_out = t_vec;
    for k = 1:N_mpc
        if is_sol
            tic_jac = tic;
            p_new = full(discretize_fn(x, u));
            ocp_solver.set('p', p_new);
            jac_time(k) = toc(tic_jac);
        end
        ocp_solver.set('constr_x0', x);
        ocp_solver.solve();
        ocp_time_tot(k)      = ocp_solver.get('time_tot');
        ocp_time_lin(k)      = ocp_solver.get('time_lin');
        ocp_time_reg(k)      = ocp_solver.get('time_reg');
        ocp_time_sim(k)      = ocp_solver.get('time_sim');
        ocp_time_glob(k)     = ocp_solver.get('time_glob');
        ocp_time_qp(k)       = ocp_solver.get('time_qp_sol');
        ocp_time_qp_xcond(k) = ocp_solver.get('time_qp_xcond');
        ocp_qp_iter(k)       = ocp_solver.get('qp_iter');
        ocp_sqp_iter(k)      = ocp_solver.get('sqp_iter');
        ocp_nlp_iter(k)      = ocp_solver.get('nlp_iter');
        ocp_residuals(:,k)   = ocp_solver.get('residuals');
        ocp_cost(k)          = ocp_solver.get_cost();
        ocp_status(k)        = ocp_solver.get('status');
        u = ocp_solver.get('u', 0);
        for i = 0:N_horizon-2
            ocp_solver.set('x', ocp_solver.get('x', i+1), i);
            ocp_solver.set('u', ocp_solver.get('u', i+1), i);
        end
        ocp_solver.set('x', ocp_solver.get('x', N_horizon), N_horizon);
        ocp_solver.set('u', ocp_solver.get('u', N_horizon-1), N_horizon-1);
        tacc = 0;
        for j = 1:n_sub
            idx = (k-1)*n_sub + j;
            if idx > N_sim, break; end
            x_sim(:, idx) = x;  u_sim(:, idx) = u;
            x = sim_solver.simulate(x, u);
            tacc = tacc + sim_solver.get('time_tot');
        end
        sim_time_tot(k) = tacc;
        if is_sol
            step_time_tot(k) = jac_time(k) + ocp_time_tot(k) + sim_time_tot(k);
        else
            step_time_tot(k) = ocp_time_tot(k) + sim_time_tot(k);
        end
        if isDiverged(x)
            diverged = true;
            last_idx = min(k*n_sub, N_sim);
            x_sim = x_sim(:, 1:last_idx);  u_sim = u_sim(:, 1:last_idx);
            t_out = t_vec(1:last_idx);
            ocp_time_tot = ocp_time_tot(1:k);  ocp_time_lin = ocp_time_lin(1:k);
            ocp_time_reg = ocp_time_reg(1:k);  ocp_time_sim = ocp_time_sim(1:k);
            ocp_time_glob = ocp_time_glob(1:k); ocp_time_qp = ocp_time_qp(1:k);
            ocp_time_qp_xcond = ocp_time_qp_xcond(1:k);
            ocp_qp_iter = ocp_qp_iter(1:k);  ocp_sqp_iter = ocp_sqp_iter(1:k);
            ocp_nlp_iter = ocp_nlp_iter(1:k);
            ocp_residuals = ocp_residuals(:,1:k);
            ocp_status = ocp_status(1:k);  ocp_cost = ocp_cost(1:k);
            jac_time = jac_time(1:k);
            sim_time_tot = sim_time_tot(1:k);  step_time_tot = step_time_tot(1:k);
            break;
        end
    end
    if ~diverged
        filled = N_mpc * n_sub;
        x_sim = x_sim(:, 1:filled);  u_sim = u_sim(:, 1:filled);
        t_out = t_vec(1:filled);
    end
    sim_data.controller = ctrl_name;
    sim_data.t = t_out;  sim_data.x = x_sim;  sim_data.u = u_sim;
    sim_data.x0 = x0;  sim_data.xEq = xEq;  sim_data.uEq = uEq;
    sim_data.dt = dt_plant;  sim_data.dt_mpc = dt_mpc;
    sim_data.N_horizon = N_horizon;  sim_data.diverged = diverged;
    sim_data.ocp_time_tot = ocp_time_tot;  sim_data.ocp_time_lin = ocp_time_lin;
    sim_data.ocp_time_reg = ocp_time_reg;  sim_data.ocp_time_sim = ocp_time_sim;
    sim_data.ocp_time_glob = ocp_time_glob;
    sim_data.ocp_time_qp = ocp_time_qp;
    sim_data.ocp_time_qp_xcond = ocp_time_qp_xcond;
    sim_data.ocp_qp_iter = ocp_qp_iter;  sim_data.ocp_sqp_iter = ocp_sqp_iter;
    sim_data.ocp_nlp_iter = ocp_nlp_iter;
    sim_data.ocp_residuals = ocp_residuals;
    sim_data.ocp_status = ocp_status;
    sim_data.sim_time_tot = sim_time_tot;  sim_data.step_time_tot = step_time_tot;
    sim_data.ocp_cost = ocp_cost;
    sim_data.cost = computeCost(x_sim - xEq, u_sim - uEq);
    sim_data.cost_cum = cumsum(sim_data.cost);
    if is_sol
        sim_data.jac_time = jac_time;
        sim_data.lin_method = 'compiled';
    end
    if ~isempty(Ad_save)
        sim_data.Ad = Ad_save;  sim_data.Bd = Bd_save;
    end
end