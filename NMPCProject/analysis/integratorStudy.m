%% integratorStudy — Single-step integrator accuracy comparison
%
% Compares acados integrator configurations (ERK/IRK × stages × steps)
% against an ode15s single-step reference using the SAME model (n=16).
%
% Approach:
%   1. Sample N_samples random (x, u) points near the operating region
%   2. For each point, take ONE integration step with each acados config
%   3. Compare the result against ode15s with tight tolerances
%   4. Aggregate single-step errors across all sample points
%
% This isolates pure numerical integration error from model fidelity error.
%
% Outputs:
%   analysis/integrator_study.mat   — all results
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

save_path = fullfile('analysis', 'integrator_study.mat');

if exist(save_path, 'file')
    fprintf('=== Loading existing results from %s ===\n', save_path);
    results = load(save_path);
    configs  = results.configs;
    dt_list  = results.dt_list;
    n_configs = numel(configs);
    n_dt      = numel(dt_list);
    N_samples = results.N_samples;
    fprintf('  %d configs x %d timesteps, %d samples\n', n_configs, n_dt, N_samples);
else
    import casadi.*

    %% ================================================================
    %% Parameters
    %% ================================================================
    modelId = MaglevModel.Accurate;
    parameters_maggy_V4;

    nx = 10;
    nu = 4;

    % Model discretization — same for both acados and ode15s reference
    magnet_n       = 16;
    magnet_n_axial = 1;

    params_sim = params;
    params_sim.magnet.n       = magnet_n;
    params_sim.magnet.n_axial = magnet_n_axial;

    % Equilibrium
    [zEq, ~, ~, ~] = computeSystemEquilibria(params, modelId);
    xEq_full = [0; 0; zEq(1); zeros(9,1)];
    xEq      = xEq_full([1:5, 7:11]);
    uEq      = zeros(nu, 1);

    %% ================================================================
    %% Sampling settings
    %% ================================================================
    N_samples = 500;
    rng(42);  % reproducibility

    % Perturbation ranges around equilibrium (physically meaningful)
    % Position: ±2mm, angles: ±10deg, velocities: ±0.1m/s, ang vel: ±5rad/s
    dx_range = [
        2e-3;    % x (m)
        2e-3;    % y (m)
        2e-3;    % z (m)
        deg2rad(10);  % roll (rad)
        deg2rad(10);  % pitch (rad)
        0.1;     % vx (m/s)
        0.1;     % vy (m/s)
        0.1;     % vz (m/s)
        5;       % wx (rad/s)
        5;       % wy (rad/s)
    ];

    % Input range: ±1A (full actuator range)
    u_max = 1.0;

    % Generate random samples
    X_samples = xEq + dx_range .* (2*rand(nx, N_samples) - 1);
    U_samples = u_max * (2*rand(nu, N_samples) - 1);

    % Timesteps to test
    dt_list = [0.0001, 0.001, 0.0015, 0.002];

    fprintf('=== Integrator single-step study ===\n');
    fprintf('  %d sample points, %d timesteps\n', N_samples, numel(dt_list));

    %% ================================================================
    %% ode15s reference: single-step for each (x, u, dt)
    %% ================================================================
    % Use the SAME CasADi model as acados to isolate integration error
    % from model implementation differences (MATLAB 12-state vs CasADi 10-state)
    fprintf('\n=== Computing ode15s reference (CasADi model, n=%d) ===\n', magnet_n);

    x_sym_ref = MX.sym('x_ref', nx);
    u_sym_ref = MX.sym('u_ref', nu);
    f_expl_ref = maglevSystemDynamicsReduced_casadi(x_sym_ref, u_sym_ref, params_sim, modelId);
    f_casadi = Function('f_casadi', {x_sym_ref, u_sym_ref}, {f_expl_ref});

    f_ode_10 = @(t, x, u) full(f_casadi(x, u));
    ode_opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-14);

    n_dt = numel(dt_list);
    x_ref = zeros(nx, N_samples, n_dt);

    for idt = 1:n_dt
        dt = dt_list(idt);
        fprintf('  dt = %.0f us: ', dt*1e6);
        tic;
        for i = 1:N_samples
            ui = U_samples(:, i);
            [~, x_out] = ode15s(@(t, x) f_ode_10(t, x, ui), [0, dt], X_samples(:, i), ode_opts);
            x_ref(:, i, idt) = x_out(end, :)';
        end
        t_elapsed = toc;
        fprintf('%.1f s (%.1f ms/sample)\n', t_elapsed, t_elapsed/N_samples*1e3);
    end

    %% ================================================================
    %% Define integrator configs
    %% ================================================================
    configs = struct([]);
    k = 0;

    % ERK: stages 1..4, steps 1,2,4,8
    for stages = [1, 2, 3, 4]
        for steps = [1, 2, 4, 8]
            k = k + 1;
            configs(k).type   = 'ERK';
            configs(k).stages = stages;
            configs(k).steps  = steps;
            configs(k).label  = sprintf('ERK(%d,%d)', stages, steps);
        end
    end

    % IRK: stages 1..4, steps 1,2,4
    for stages = [1, 2, 3, 4]
        for steps = [1, 2, 4]
            k = k + 1;
            configs(k).type   = 'IRK';
            configs(k).stages = stages;
            configs(k).steps  = steps;
            configs(k).label  = sprintf('IRK(%d,%d)', stages, steps);
        end
    end

    n_configs = numel(configs);
    fprintf('\n=== Sweeping %d integrator configs x %d timesteps = %d runs ===\n', ...
        n_configs, n_dt, n_configs * n_dt);

    %% ================================================================
    %% Build CasADi model for acados
    %% ================================================================
    fprintf('\n=== Building CasADi model (n=%d, n_axial=%d) ===\n', magnet_n, magnet_n_axial);

    x_sym    = MX.sym('x',    nx);
    u_sym    = MX.sym('u',    nu);
    xdot_sym = MX.sym('xdot', nx);

    f_expl = maglevSystemDynamicsReduced_casadi(x_sym, u_sym, params_sim, modelId);

    base_model = AcadosModel();
    base_model.name        = 'maglev_intstudy';
    base_model.x           = x_sym;
    base_model.u           = u_sym;
    base_model.xdot        = xdot_sym;
    base_model.f_impl_expr = xdot_sym - f_expl;
    base_model.f_expl_expr = f_expl;

    %% ================================================================
    %% Run sweep
    %% ================================================================

    % Pre-allocate results
    % For each (config, dt): store error statistics across all N_samples points
    results = struct();
    results.configs    = configs;
    results.dt_list    = dt_list;
    results.N_samples  = N_samples;
    results.X_samples  = X_samples;
    results.U_samples  = U_samples;
    results.xEq        = xEq;
    results.magnet_n   = magnet_n;

    for idt = 1:n_dt
        dt = dt_list(idt);
        fprintf('\n--- dt = %.0f us ---\n', dt*1e6);

        for k = 1:n_configs
            cfg = configs(k);
            fprintf('[%2d/%d] %s ... ', k, n_configs, cfg.label);

            % Build solver
            sim = AcadosSim();
            sim.model = base_model;
            sim.solver_options.Tsim            = dt;
            sim.solver_options.integrator_type = cfg.type;
            sim.solver_options.num_stages      = cfg.stages;
            sim.solver_options.num_steps       = cfg.steps;

            solver_dir = fullfile('build', 'intstudy', ...
                sprintf('%s_s%d_n%d_dt%d', cfg.type, cfg.stages, cfg.steps, round(dt*1e6)));
            sim.code_gen_opts.code_export_directory = fullfile(solver_dir, 'c_generated_code');
            sim.code_gen_opts.json_file = fullfile(solver_dir, [base_model.name '_sim.json']);

            try
                tic_build = tic;
                sim_solver = AcadosSimSolver(sim, struct('output_dir', solver_dir));
                build_time = toc(tic_build);
            catch ME
                fprintf('BUILD FAILED: %s\n', ME.message);
                results.data(k, idt).failed    = true;
                results.data(k, idt).error_msg = ME.message;
                continue;
            end

            % Single-step for each sample point
            x_acados = zeros(nx, N_samples);
            t_step   = zeros(1, N_samples);

            for i = 1:N_samples
                x_acados(:, i) = sim_solver.simulate(X_samples(:, i), U_samples(:, i));
                t_step(i) = sim_solver.get('time_tot');
            end

            clear sim_solver;

            % Compute per-sample errors against ode15s reference
            err = x_acados - x_ref(:, :, idt);  % nx x N_samples

            % Position error (states 1:3, in mm)
            pos_err_norm = vecnorm(err(1:3, :), 2, 1) * 1e3;
            % Angle error (states 4:5, in deg)
            ang_err_norm = vecnorm(err(4:5, :), 2, 1) * (180/pi);
            % Velocity error (states 6:10)
            vel_err_norm = vecnorm(err(6:end, :), 2, 1);

            % Statistics across samples
            results.data(k, idt).failed     = false;
            results.data(k, idt).build_time = build_time;

            results.data(k, idt).pos_rmse    = rms(pos_err_norm);
            results.data(k, idt).pos_mean    = mean(pos_err_norm);
            results.data(k, idt).pos_max     = max(pos_err_norm);
            results.data(k, idt).pos_median  = median(pos_err_norm);
            results.data(k, idt).pos_std     = std(pos_err_norm);

            results.data(k, idt).ang_rmse    = rms(ang_err_norm);
            results.data(k, idt).ang_max     = max(ang_err_norm);

            results.data(k, idt).vel_rmse    = rms(vel_err_norm);
            results.data(k, idt).vel_max     = max(vel_err_norm);

            % Filter out negative/bogus timing values from acados
            t_valid = t_step(t_step > 0);
            if isempty(t_valid), t_valid = t_step; end
            results.data(k, idt).time_mean   = mean(t_valid);
            results.data(k, idt).time_median = median(t_valid);
            results.data(k, idt).time_max    = max(t_valid);

            % Store raw errors and timings for distribution plots
            results.data(k, idt).pos_err_all  = pos_err_norm;
            results.data(k, idt).ang_err_all  = ang_err_norm;
            results.data(k, idt).time_step_all = t_valid;  % raw per-sample step times (s)

            fprintf('pos_rmse=%.3e mm  pos_max=%.3e mm  t=%.0f us\n', ...
                results.data(k, idt).pos_rmse, results.data(k, idt).pos_max, ...
                results.data(k, idt).time_mean * 1e6);
        end
    end

    %% ================================================================
    %% Save results
    %% ================================================================
    save(save_path, '-struct', 'results');
    fprintf('\nResults saved to: %s\n', save_path);
end  % if exist(save_path)

%% ================================================================
%% Print summary tables
%% ================================================================
for idt = 1:n_dt
    dt_us = dt_list(idt) * 1e6;
    fprintf('\n=== dt = %.0f us ===\n', dt_us);
    fprintf('%-12s  %10s  %10s  %10s  %10s  %8s\n', ...
        'Config', 'Pos RMSE', 'Pos Max', 'Ang RMSE', 'Vel RMSE', 'Time');
    fprintf('%-12s  %10s  %10s  %10s  %10s  %8s\n', ...
        '', '(mm)', '(mm)', '(deg)', '(m/s)', '(us)');
    fprintf('%s\n', repmat('-', 1, 75));

    for k = 1:n_configs
        cfg = configs(k);
        d = results.data(k, idt);
        if d.failed
            fprintf('%-12s  %10s\n', cfg.label, 'FAILED');
        else
            fprintf('%-12s  %10.3e  %10.3e  %10.3e  %10.3e  %8.1f\n', ...
                cfg.label, d.pos_rmse, d.pos_max, d.ang_rmse, d.vel_rmse, ...
                d.time_mean * 1e6);
        end
    end
end

%% ================================================================
%% Generate figures
%% ================================================================
fprintf('\n=== Generating figures ===\n');

fig_dir = fullfile(project_root, 'figures', 'integrator');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

W_FULL = 17;   % cm
H_STD  = 7;

FONT_NAME  = 'Times New Roman';
FONT_AX    = 9;
FONT_LAB   = 10;
FONT_LEG   = 8;
FONT_TITLE = 10;

C_ERK = [0.00 0.45 0.74];   % blue
C_IRK = [0.85 0.33 0.10];   % orange

% Reference timestep for boxplots
idt_mpc = find(dt_list == 0.001, 1);
if isempty(idt_mpc), idt_mpc = n_dt; end

% Build list of all non-failed configs at the reference dt, sorted by type then label
sel_idx   = [];
sel_label = {};
sel_type  = {};
for k = 1:n_configs
    d = results.data(k, idt_mpc);
    if d.failed, continue; end
    sel_idx(end+1)     = k; %#ok<AGROW>
    sel_label{end+1}   = configs(k).label; %#ok<AGROW>
    sel_type{end+1}    = configs(k).type;  %#ok<AGROW>
end
n_sel = numel(sel_idx);

% Assign colors per bar: blue for ERK, orange for IRK
sel_colors = zeros(n_sel, 3);
for j = 1:n_sel
    if strcmp(sel_type{j}, 'ERK')
        sel_colors(j,:) = C_ERK;
    else
        sel_colors(j,:) = C_IRK;
    end
end

%% --- Figure 1: Position RMSE boxplot per configuration ---
fig1 = mkfig(W_FULL, H_STD + 2);

box_data   = [];
box_groups = [];
for j = 1:n_sel
    d = results.data(sel_idx(j), idt_mpc);
    box_data   = [box_data, d.pos_err_all]; %#ok<AGROW>
    box_groups = [box_groups, j * ones(1, numel(d.pos_err_all))]; %#ok<AGROW>
end

if ~isempty(box_data)
    bp = boxplot(box_data, box_groups, ...
        'Labels', sel_label, 'Whisker', 1.5, 'Symbol', '.', 'Colors', 'k');
    % Color each box by ERK/IRK
    h_boxes = findobj(gca, 'Tag', 'Box');
    for j = 1:numel(h_boxes)
        % boxplot draws boxes in reverse order
        idx_rev = n_sel - j + 1;
        patch(get(h_boxes(j), 'XData'), get(h_boxes(j), 'YData'), ...
            sel_colors(idx_rev, :), 'FaceAlpha', 0.4, 'EdgeColor', sel_colors(idx_rev, :));
    end
    set(gca, 'YScale', 'log');
    ylabel('Single-step position error (mm)', 'FontSize', FONT_LAB);
    xlabel('Integrator configuration', 'FontSize', FONT_LAB);
    title(sprintf('Position error distribution (%d samples, dt = %.0f \\mus)', ...
        N_samples, dt_list(idt_mpc)*1e6), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
    grid on; box on;
    % Legend for ERK / IRK
    hold on;
    h_erk = patch(NaN, NaN, C_ERK, 'FaceAlpha', 0.4, 'EdgeColor', C_ERK, 'DisplayName', 'ERK');
    h_irk = patch(NaN, NaN, C_IRK, 'FaceAlpha', 0.4, 'EdgeColor', C_IRK, 'DisplayName', 'IRK');
    legend([h_erk, h_irk], 'Location', 'best', 'FontSize', FONT_LEG);
    set(gca, 'XTickLabelRotation', 45);
    style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig1, 'int_error_boxplot', fig_dir);

%% --- Figure 2: Step time boxplot per configuration ---
fig2 = mkfig(W_FULL, H_STD + 2);

% Check if raw timing data is available
has_raw_timing = isfield(results.data, 'time_step_all');

box_data_t   = [];
box_groups_t = [];
use_mean_fallback = false;

if has_raw_timing
    for j = 1:n_sel
        d = results.data(sel_idx(j), idt_mpc);
        if ~isempty(d.time_step_all)
            t_us = d.time_step_all * 1e6;  % convert to microseconds
            box_data_t   = [box_data_t, t_us]; %#ok<AGROW>
            box_groups_t = [box_groups_t, j * ones(1, numel(t_us))]; %#ok<AGROW>
        end
    end
end

if isempty(box_data_t)
    % Fallback: use mean timing as a bar chart
    use_mean_fallback = true;
    fprintf('  (No raw timing data — using mean values as bar chart)\n');
    mean_times = zeros(1, n_sel);
    for j = 1:n_sel
        d = results.data(sel_idx(j), idt_mpc);
        mean_times(j) = d.time_mean * 1e6;
    end
    b = bar(1:n_sel, mean_times, 0.6);
    b.FaceColor = 'flat';
    for j = 1:n_sel
        b.CData(j,:) = sel_colors(j,:);
    end
    set(gca, 'XTick', 1:n_sel, 'XTickLabel', sel_label);
else
    bp = boxplot(box_data_t, box_groups_t, ...
        'Labels', sel_label, 'Whisker', 1.5, 'Symbol', '.', 'Colors', 'k');
    % Color each box by ERK/IRK
    h_boxes = findobj(gca, 'Tag', 'Box');
    for j = 1:numel(h_boxes)
        idx_rev = n_sel - j + 1;
        patch(get(h_boxes(j), 'XData'), get(h_boxes(j), 'YData'), ...
            sel_colors(idx_rev, :), 'FaceAlpha', 0.4, 'EdgeColor', sel_colors(idx_rev, :));
    end
end

ylabel('Step time (\\mus)', 'FontSize', FONT_LAB);
xlabel('Integrator configuration', 'FontSize', FONT_LAB);
title(sprintf('Computation time per step (%d samples, dt = %.0f \\mus)', ...
    N_samples, dt_list(idt_mpc)*1e6), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
grid on; box on;
hold on;
h_erk = patch(NaN, NaN, C_ERK, 'FaceAlpha', 0.4, 'EdgeColor', C_ERK, 'DisplayName', 'ERK');
h_irk = patch(NaN, NaN, C_IRK, 'FaceAlpha', 0.4, 'EdgeColor', C_IRK, 'DisplayName', 'IRK');
legend([h_erk, h_irk], 'Location', 'best', 'FontSize', FONT_LEG);
set(gca, 'XTickLabelRotation', 45);
style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);

savefig_thesis(fig2, 'int_timing_boxplot', fig_dir);

fprintf('\n=== Integrator study complete ===\n');
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
