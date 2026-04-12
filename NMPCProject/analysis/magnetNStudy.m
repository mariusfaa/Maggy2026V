%% magnetNStudy — Model fidelity vs magnet.n (radial discretization)
%
% Sweeps magnet.n with n_axial=1 fixed via acados/CasADi IRK(4,1).
% Single-step comparison of 500 random (x,u) samples against ode15s
% reference at n=80, n_axial=21.
%
% See also: nAxialStudy.m (n_axial effect via MATLAB ode15s)
%
% Outputs:
%   analysis/magnet_n_study.mat     — results
%   Master_thesis_2025/figures/     — thesis-quality figures

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

save_path = fullfile('analysis', 'magnet_n_study.mat');

if exist(save_path, 'file')
    fprintf('=== Loading existing results from %s ===\n', save_path);
    results = load(save_path);
else
    import casadi.*

    %% ================================================================
    %% Parameters
    %% ================================================================
    modelId = MaglevModel.Accurate;
    parameters_maggy_V4;

    nx = 10;
    nu = 4;

    [zEq, ~, ~, ~] = computeSystemEquilibria(params, modelId);
    xEq_full = [0; 0; zEq(1); zeros(9,1)];
    xEq      = xEq_full([1:5, 7:11]);

    %% ================================================================
    %% Sampling
    %% ================================================================
    N_samples = 500;
    rng(42);

    dx_range = [
        2e-3; 2e-3; 2e-3;
        deg2rad(10); deg2rad(10);
        0.1; 0.1; 0.1;
        5; 5;
    ];
    u_max = 1.0;

    X_samples = xEq + dx_range .* (2*rand(nx, N_samples) - 1);
    U_samples = u_max * (2*rand(nu, N_samples) - 1);

    int_type   = 'IRK';
    int_stages = 4;
    int_steps  = 1;
    dt         = 0.001;

    %% ================================================================
    %% ode15s reference (n=80, n_axial=21)
    %% ================================================================
    fprintf('=== Computing ode15s reference (n=80, n_axial=21) ===\n');

    params_ref = params;
    params_ref.magnet.n       = 80;
    params_ref.magnet.n_axial = 21;

    f_ode_ref = @(x, u) maglevSystemDynamics(x, u, params_ref, modelId);
    ode_opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-14);

    x_ref = zeros(nx, N_samples);
    tic;
    for i = 1:N_samples
        x10 = X_samples(:, i);
        x12 = [x10(1:5); 0; x10(6:10); 0];
        [~, x_out] = ode15s(@(t, x) f_ode_ref(x, U_samples(:, i)), [0, dt], x12, ode_opts);
        x12_end = x_out(end, :)';
        x_ref(:, i) = x12_end([1:5, 7:11]);
    end
    fprintf('  Done in %.1f s\n', toc);

    %% ================================================================
    %% Sweep magnet.n (n_axial=1 fixed)
    %% ================================================================
    n_list = [2, 4, 6, 8, 10, 12, 14, 16, 20, 24, 32, 48, 64];
    n_n = numel(n_list);

    fprintf('\n=== n sweep (n_axial=1) ===\n');

    for i = 1:n_n
        mn = n_list(i);
        fprintf('[%2d/%d] n=%d ... ', i, n_n, mn);
        d = run_config(mn, 1, dt, int_type, int_stages, int_steps, ...
                       params, modelId, nx, N_samples, X_samples, U_samples, x_ref);
        results.n_sweep(i) = d;
        if d.failed
            fprintf('FAILED\n');
        else
            fprintf('pos_rmse=%.4e mm  t=%.0f us\n', d.pos_rmse, d.time_mean*1e6);
        end
    end

    results.n_list     = n_list;
    results.dt         = dt;
    results.N_samples  = N_samples;
    results.X_samples  = X_samples;
    results.U_samples  = U_samples;
    results.xEq        = xEq;
    results.int_type   = int_type;
    results.int_stages = int_stages;
    results.int_steps  = int_steps;

    cd(project_root);
    save(save_path, '-struct', 'results');
    fprintf('\nResults saved to: %s\n', save_path);
end  % if exist(save_path)

%% ================================================================
%% Extract data
%% ================================================================
n_list    = results.n_list;
n_n       = numel(n_list);
dt        = results.dt;
N_samples = results.N_samples;

n_pos_rmse   = zeros(1, n_n);
n_pos_max    = zeros(1, n_n);
n_ang_rmse   = zeros(1, n_n);
n_vel_rmse   = zeros(1, n_n);
n_time       = zeros(1, n_n);
n_build_time = zeros(1, n_n);
for i = 1:n_n
    d = results.n_sweep(i);
    if ~d.failed
        n_pos_rmse(i)   = d.pos_rmse;
        n_pos_max(i)    = d.pos_max;
        n_ang_rmse(i)   = d.ang_rmse;
        n_vel_rmse(i)   = d.vel_rmse;
        n_time(i)       = d.time_mean * 1e6;
        n_build_time(i) = d.model_build_time;
    end
end

%% ================================================================
%% Print summary table
%% ================================================================
fprintf('\n=== n sweep (n_axial=1, dt=%.0f us) ===\n', dt*1e6);
fprintf('%-6s  %12s  %12s  %12s  %12s  %10s  %10s\n', ...
    'n', 'Pos RMSE', 'Pos Max', 'Ang RMSE', 'Vel RMSE', 'Step time', 'Build');
fprintf('%-6s  %12s  %12s  %12s  %12s  %10s  %10s\n', ...
    '', '(mm)', '(mm)', '(deg)', '(m/s)', '(us)', '(s)');
fprintf('%s\n', repmat('-', 1, 85));
for i = 1:n_n
    d = results.n_sweep(i);
    if d.failed
        fprintf('%-6d  %12s\n', n_list(i), 'FAILED');
    else
        fprintf('%-6d  %12.4e  %12.4e  %12.4e  %12.4e  %10.1f  %10.2f\n', ...
            n_list(i), d.pos_rmse, d.pos_max, d.ang_rmse, d.vel_rmse, ...
            d.time_mean*1e6, d.model_build_time);
    end
end

%% ================================================================
%% Generate figures
%% ================================================================
fprintf('\n=== Generating figures ===\n');

fig_dir = fullfile(project_root, 'Master_thesis_2025', 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

W_FULL = 17;
H_STD  = 7;

FONT_NAME  = 'Times New Roman';
FONT_AX    = 9;
FONT_LAB   = 10;
FONT_LEG   = 8;
FONT_TITLE = 10;

C_BLUE   = [0.00 0.45 0.74];
C_RED    = [0.85 0.33 0.10];
C_GREEN  = [0.47 0.67 0.19];

%% --- Figure 1: Accuracy vs n (dual y-axis: RMSE + step time) ---
fig1 = mkfig(W_FULL, H_STD);

yyaxis left;
semilogy(n_list, n_pos_rmse, 'o-', 'Color', C_BLUE, 'LineWidth', 1.5, ...
    'MarkerSize', 6, 'MarkerFaceColor', C_BLUE);
ylabel('Position RMSE (mm)', 'FontSize', FONT_LAB);
set(gca, 'YColor', C_BLUE);

yyaxis right;
plot(n_list, n_time, 's-', 'Color', C_RED, 'LineWidth', 1.5, ...
    'MarkerSize', 6, 'MarkerFaceColor', C_RED);
ylabel('Step time (\mus)', 'FontSize', FONT_LAB);
set(gca, 'YColor', C_RED);

xlabel('magnet.n', 'FontSize', FONT_LAB);
title(sprintf('Model accuracy and cost vs. magnet.n (dt = %.0f \\mus)', dt*1e6), ...
    'FontSize', FONT_TITLE, 'FontWeight', 'bold');
grid on; box on;
legend({'Pos RMSE', 'Step time'}, 'Location', 'east', 'FontSize', FONT_LEG);
style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);

savefig_thesis(fig1, 'magn_accuracy_vs_n', fig_dir);

%% --- Figure 2: Error distribution boxplot per n ---
fig2 = mkfig(W_FULL, H_STD);

sel_idx = find(ismember(n_list, [4, 8, 10, 12, 16, 32, 64]));
n_sel = numel(sel_idx);

box_data = [];
box_groups = [];
box_labels = {};
for j = 1:n_sel
    i = sel_idx(j);
    d = results.n_sweep(i);
    if ~d.failed
        box_data = [box_data, d.pos_err_all];
        box_groups = [box_groups, j * ones(1, numel(d.pos_err_all))];
        box_labels{j} = sprintf('n=%d', n_list(i));
    end
end

if ~isempty(box_data)
    boxplot(box_data, box_groups, 'Labels', box_labels, ...
        'Whisker', 1.5, 'Symbol', '.', 'Colors', 'k');
    ylabel('Single-step position error (mm)', 'FontSize', FONT_LAB);
    xlabel('magnet.n', 'FontSize', FONT_LAB);
    title(sprintf('Error distribution (%d samples, dt = %.0f \\mus)', ...
        N_samples, dt*1e6), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
    grid on; box on;
    set(gca, 'YScale', 'log');
    style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig2, 'magn_error_distribution', fig_dir);

%% --- Figure 3: Timing distribution boxplot per n ---
fig3 = mkfig(W_FULL, H_STD);

box_data_t = [];
box_groups_t = [];
box_labels_t = {};
for j = 1:n_sel
    i = sel_idx(j);
    d = results.n_sweep(i);
    if ~d.failed && isfield(d, 'time_all')
        box_data_t = [box_data_t, d.time_all];
        box_groups_t = [box_groups_t, j * ones(1, numel(d.time_all))];
        box_labels_t{j} = sprintf('n=%d', n_list(i));
    end
end

if ~isempty(box_data_t)
    boxplot(box_data_t, box_groups_t, 'Labels', box_labels_t, ...
        'Whisker', 1.5, 'Symbol', '.', 'Colors', 'k');
    ylabel('Single-step time (\mus)', 'FontSize', FONT_LAB);
    xlabel('magnet.n', 'FontSize', FONT_LAB);
    title(sprintf('Step time distribution (%d samples, dt = %.0f \\mus)', ...
        N_samples, dt*1e6), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
    grid on; box on;
    style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig3, 'magn_time_distribution', fig_dir);

%% --- Figure 4: All error types vs n ---
fig4 = mkfig(W_FULL, H_STD);
hold on; grid on; box on;

semilogy(n_list, n_pos_rmse, 'o-', 'Color', C_BLUE, 'LineWidth', 1.3, ...
    'MarkerSize', 5, 'MarkerFaceColor', C_BLUE, 'DisplayName', 'Position (mm)');
semilogy(n_list, n_ang_rmse, 's-', 'Color', C_RED, 'LineWidth', 1.3, ...
    'MarkerSize', 5, 'MarkerFaceColor', C_RED, 'DisplayName', 'Angle (deg)');
semilogy(n_list, n_vel_rmse, 'd-', 'Color', C_GREEN, 'LineWidth', 1.3, ...
    'MarkerSize', 5, 'MarkerFaceColor', C_GREEN, 'DisplayName', 'Velocity (m/s)');

xlabel('magnet.n', 'FontSize', FONT_LAB);
ylabel('RMSE vs. n=80 reference', 'FontSize', FONT_LAB);
title('All state errors vs. magnet.n', 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', FONT_LEG);
style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);

savefig_thesis(fig4, 'magn_all_errors_vs_n', fig_dir);

fprintf('\n=== Magnet N study complete ===\n');
fprintf('Figures saved to: %s\n', fig_dir);

%% ================================================================
%% Local helper functions
%% ================================================================

function fig = mkfig(w_cm, h_cm)
    fig = figure('Units', 'centimeters', ...
                 'Position',      [2  2  w_cm h_cm], ...
                 'PaperUnits',    'centimeters', ...
                 'PaperSize',     [w_cm h_cm], ...
                 'PaperPosition', [0   0  w_cm h_cm], ...
                 'Color', 'w');
end

function style_ax(ax, fn, fs_ax, fs_lab)
    set(ax, 'FontName',    fn, ...
            'FontSize',    fs_ax, ...
            'TickDir',     'out', ...
            'TickLength',  [0.015 0.015], ...
            'LineWidth',   0.6, ...
            'Box',         'on');
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

function d = run_config(mn, mn_axial, dt, int_type, int_stages, int_steps, ...
                        params, modelId, nx, N_samples, X_samples, U_samples, x_ref)
    import casadi.*

    params_n = params;
    params_n.magnet.n       = mn;
    params_n.magnet.n_axial = mn_axial;

    x_sym    = MX.sym('x',    nx);
    u_sym    = MX.sym('u',    4);
    xdot_sym = MX.sym('xdot', nx);

    tic_model = tic;
    f_expl = maglevSystemDynamicsReduced_casadi(x_sym, u_sym, params_n, modelId);
    d.model_build_time = toc(tic_model);

    mdl = AcadosModel();
    mdl.name        = sprintf('maglev_n%d_ax%d', mn, mn_axial);
    mdl.x           = x_sym;
    mdl.u           = u_sym;
    mdl.xdot        = xdot_sym;
    mdl.f_impl_expr = xdot_sym - f_expl;
    mdl.f_expl_expr = f_expl;

    sim = AcadosSim();
    sim.model = mdl;
    sim.solver_options.Tsim            = dt;
    sim.solver_options.integrator_type = int_type;
    sim.solver_options.num_stages      = int_stages;
    sim.solver_options.num_steps       = int_steps;

    solver_dir = fullfile('build', 'magnetn', sprintf('n%d_ax%d', mn, mn_axial));
    sim.code_gen_opts.code_export_directory = fullfile(solver_dir, 'c_generated_code');
    sim.code_gen_opts.json_file = fullfile(solver_dir, [mdl.name '_sim.json']);

    try
        tic_build = tic;
        sim_solver = AcadosSimSolver(sim, struct('output_dir', solver_dir));
        d.solver_build_time = toc(tic_build);
    catch ME
        d.failed    = true;
        d.error_msg = ME.message;
        return;
    end

    x_acados = zeros(nx, N_samples);
    t_step   = zeros(1, N_samples);
    for i = 1:N_samples
        x_acados(:, i) = sim_solver.simulate(X_samples(:, i), U_samples(:, i));
        t_step(i) = sim_solver.get('time_tot');
    end
    clear sim_solver;

    err = x_acados - x_ref;
    pos_err = vecnorm(err(1:3, :), 2, 1) * 1e3;
    ang_err = vecnorm(err(4:5, :), 2, 1) * (180/pi);
    vel_err = vecnorm(err(6:end, :), 2, 1);

    d.failed      = false;
    d.pos_rmse    = rms(pos_err);
    d.pos_max     = max(pos_err);
    d.pos_mean    = mean(pos_err);
    d.pos_median  = median(pos_err);
    d.pos_std     = std(pos_err);
    d.ang_rmse    = rms(ang_err);
    d.ang_max     = max(ang_err);
    d.vel_rmse    = rms(vel_err);
    d.vel_max     = max(vel_err);
    % Filter out negative/bogus timing values from acados
    t_valid = t_step(t_step > 0);
    if isempty(t_valid), t_valid = t_step; end
    d.time_mean   = mean(t_valid);
    d.time_median = median(t_valid);
    d.time_max    = max(t_valid);
    d.time_all    = t_valid * 1e6;
    d.pos_err_all = pos_err;
end
