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
    dt_list = [0.0001, 0.001, 0.002, 0.003];

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

            % Store raw errors for distribution plots
            results.data(k, idt).pos_err_all = pos_err_norm;
            results.data(k, idt).ang_err_all = ang_err_norm;

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

fig_dir = fullfile(project_root, 'Master_thesis_2025', 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

W_FULL = 17;   % cm
H_STD  = 7;
H_TALL = 10;

FONT_NAME  = 'Times New Roman';
FONT_AX    = 9;
FONT_LAB   = 10;
FONT_LEG   = 8;
FONT_TITLE = 10;

C_ERK = [0.00 0.45 0.74];   % blue
C_IRK = [0.85 0.33 0.10];   % orange

erk_stages_list = [1, 2, 3, 4];
erk_steps_list  = [1, 2, 4, 8];
irk_stages_list = [1, 2, 3, 4];
irk_steps_list  = [1, 2, 4];

%% --- Figure 1: Accuracy vs computation time (one subplot per dt) ---
fig1 = mkfig(W_FULL, H_TALL);

for idt = 1:n_dt
    subplot(2, 2, idt); hold on; grid on; box on;

    for k = 1:n_configs
        d = results.data(k, idt);
        cfg = configs(k);
        if d.failed, continue; end

        if strcmp(cfg.type, 'ERK'), clr = C_ERK; marker = 'o';
        else,                       clr = C_IRK; marker = 's'; end
        sz = 20 + cfg.stages * 15;

        scatter(d.time_mean*1e6, d.pos_rmse, sz, clr, marker, 'filled', ...
            'MarkerFaceAlpha', 0.7, 'HandleVisibility', 'off');
        text(d.time_mean*1e6 * 1.08, d.pos_rmse, ...
            sprintf('(%d,%d)', cfg.stages, cfg.steps), ...
            'FontSize', 5, 'FontName', FONT_NAME, 'Color', clr * 0.7);
    end

    set(gca, 'XScale', 'log', 'YScale', 'log');
    xlabel('Time/step (\mus)', 'FontSize', FONT_LAB);
    ylabel('Pos RMSE (mm)', 'FontSize', FONT_LAB);
    title(sprintf('dt = %.0f \\mus', dt_list(idt)*1e6), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
    if idt == 1
        scatter(NaN, NaN, 50, C_ERK, 'o', 'filled', 'DisplayName', 'ERK');
        scatter(NaN, NaN, 50, C_IRK, 's', 'filled', 'DisplayName', 'IRK');
        legend('Location', 'best', 'FontSize', FONT_LEG);
    end
    style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig1, 'int_accuracy_vs_time', fig_dir);

%% --- Figure 2: Error heatmaps at dt=1ms ---
idt_mpc = find(dt_list == 0.001, 1);
if isempty(idt_mpc), idt_mpc = n_dt; end

fig2 = mkfig(W_FULL, H_STD);

erk_map = NaN(numel(erk_stages_list), numel(erk_steps_list));
irk_map = NaN(numel(irk_stages_list), numel(irk_steps_list));

for k = 1:n_configs
    cfg = configs(k);
    d = results.data(k, idt_mpc);
    if d.failed, continue; end
    if strcmp(cfg.type, 'ERK')
        si = find(erk_stages_list == cfg.stages);
        ni = find(erk_steps_list  == cfg.steps);
        erk_map(si, ni) = log10(max(d.pos_rmse, 1e-15));
    else
        si = find(irk_stages_list == cfg.stages);
        ni = find(irk_steps_list  == cfg.steps);
        irk_map(si, ni) = log10(max(d.pos_rmse, 1e-15));
    end
end

all_vals = [erk_map(:); irk_map(:)];
all_vals = all_vals(isfinite(all_vals));
if numel(all_vals) >= 2 && (max(all_vals) - min(all_vals)) > 0.01
    clim_range = [min(all_vals)-0.5, max(all_vals)+0.5];
elseif ~isempty(all_vals)
    clim_range = [mean(all_vals)-2, mean(all_vals)+2];
else
    clim_range = [-6, 0];
end

for ip = 1:2
    subplot(1, 2, ip);
    if ip == 1
        map = erk_map; stages_l = erk_stages_list; steps_l = erk_steps_list; lbl = 'ERK';
    else
        map = irk_map; stages_l = irk_stages_list; steps_l = irk_steps_list; lbl = 'IRK';
    end
    h = imagesc(1:numel(steps_l), 1:numel(stages_l), map);
    set(h, 'AlphaData', ~isnan(map));
    set(gca, 'Color', [0.85 0.85 0.85], 'YDir', 'normal');
    clim(clim_range);
    colormap(gca, flipud(parula));
    xticks(1:numel(steps_l));
    xticklabels(arrayfun(@num2str, steps_l, 'UniformOutput', false));
    yticks(1:numel(stages_l));
    yticklabels(arrayfun(@num2str, stages_l, 'UniformOutput', false));
    xlabel('Steps', 'FontSize', FONT_LAB);
    ylabel('Stages', 'FontSize', FONT_LAB);
    title(sprintf('%s: log_{10} Pos RMSE (mm) at dt=%.0f\\mus', lbl, dt_list(idt_mpc)*1e6), ...
        'FontSize', FONT_TITLE);
    for si = 1:numel(stages_l)
        for ni = 1:numel(steps_l)
            if isfinite(map(si, ni))
                text(ni, si, sprintf('%.2f', map(si, ni)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 7, 'FontName', FONT_NAME);
            else
                text(ni, si, 'X', 'HorizontalAlignment', 'center', ...
                    'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
            end
        end
    end
    if ip == 2
        cb = colorbar;
        cb.Label.String = 'log_{10} RMSE (mm)';
        cb.Label.FontSize = FONT_AX;
    end
    style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig2, 'int_error_heatmap', fig_dir);

%% --- Figure 3: Timing heatmap at dt=1ms ---
fig3 = mkfig(W_FULL, H_STD);

erk_time = NaN(numel(erk_stages_list), numel(erk_steps_list));
irk_time = NaN(numel(irk_stages_list), numel(irk_steps_list));

for k = 1:n_configs
    cfg = configs(k);
    d = results.data(k, idt_mpc);
    if d.failed, continue; end
    if strcmp(cfg.type, 'ERK')
        si = find(erk_stages_list == cfg.stages);
        ni = find(erk_steps_list  == cfg.steps);
        erk_time(si, ni) = d.time_mean * 1e6;
    else
        si = find(irk_stages_list == cfg.stages);
        ni = find(irk_steps_list  == cfg.steps);
        irk_time(si, ni) = d.time_mean * 1e6;
    end
end

all_times = [erk_time(:); irk_time(:)];
all_times = all_times(isfinite(all_times));
if isempty(all_times)
    clim_time = [0, 100];
elseif (max(all_times) - min(all_times)) < 1
    clim_time = [mean(all_times)*0.8, mean(all_times)*1.2];
else
    clim_time = [0, max(all_times) * 1.1];
end

for ip = 1:2
    subplot(1, 2, ip);
    if ip == 1
        map = erk_time; stages_l = erk_stages_list; steps_l = erk_steps_list; lbl = 'ERK';
    else
        map = irk_time; stages_l = irk_stages_list; steps_l = irk_steps_list; lbl = 'IRK';
    end
    h = imagesc(1:numel(steps_l), 1:numel(stages_l), map);
    set(h, 'AlphaData', ~isnan(map));
    set(gca, 'Color', [0.85 0.85 0.85], 'YDir', 'normal');
    clim(clim_time);
    xticks(1:numel(steps_l));
    xticklabels(arrayfun(@num2str, steps_l, 'UniformOutput', false));
    yticks(1:numel(stages_l));
    yticklabels(arrayfun(@num2str, stages_l, 'UniformOutput', false));
    xlabel('Steps', 'FontSize', FONT_LAB);
    ylabel('Stages', 'FontSize', FONT_LAB);
    title(sprintf('%s: Mean step time (\\mus)', lbl), 'FontSize', FONT_TITLE);
    for si = 1:numel(stages_l)
        for ni = 1:numel(steps_l)
            if isfinite(map(si, ni))
                text(ni, si, sprintf('%.0f', map(si, ni)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 7, 'FontName', FONT_NAME);
            end
        end
    end
    if ip == 2
        cb = colorbar;
        cb.Label.String = '\mus';
        cb.Label.FontSize = FONT_AX;
    end
    style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig3, 'int_timing_heatmap', fig_dir);

%% --- Figure 4: Error distribution boxplot at dt=1ms (selected configs) ---
fig4 = mkfig(W_FULL, H_STD);

sel_cfgs = {'ERK(1,1)', 'ERK(2,1)', 'ERK(4,1)', 'ERK(4,4)', 'IRK(1,1)', 'IRK(2,1)', 'IRK(4,1)'};
n_sel = numel(sel_cfgs);

box_data = [];
box_groups = [];
for j = 1:n_sel
    idx = find(strcmp({configs.label}, sel_cfgs{j}));
    if isempty(idx), continue; end
    d = results.data(idx, idt_mpc);
    if d.failed, continue; end
    box_data = [box_data, d.pos_err_all];
    box_groups = [box_groups, j * ones(1, numel(d.pos_err_all))];
end

if ~isempty(box_data)
    boxplot(box_data, box_groups, 'Labels', sel_cfgs(1:max(box_groups)), ...
        'Whisker', 1.5, 'Symbol', '.', 'Colors', 'k');
    ylabel('Single-step position error (mm)', 'FontSize', FONT_LAB);
    title(sprintf('Error distribution across %d samples at dt = %.0f \\mus', ...
        N_samples, dt_list(idt_mpc)*1e6), 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
    grid on; box on;
    set(gca, 'YScale', 'log');
    style_ax(gca, FONT_NAME, FONT_AX, FONT_LAB);
end

savefig_thesis(fig4, 'int_error_distribution', fig_dir);

%% --- Figure 5: Position RMSE vs dt for selected integrators ---
fig5 = mkfig(W_FULL, H_STD);
ax = axes; hold on; grid on; box on;

sel_cfgs2   = {'ERK(1,1)', 'ERK(2,1)', 'ERK(4,1)', 'ERK(4,4)', 'IRK(2,1)', 'IRK(4,1)'};
sel_colors  = {[0.8 0.2 0.2], [0.8 0.6 0.0], C_ERK, [0.0 0.7 0.5], [0.7 0.3 0.7], C_IRK};
sel_markers = {'o', 's', 'd', '^', 'v', 'p'};

for j = 1:numel(sel_cfgs2)
    idx = find(strcmp({configs.label}, sel_cfgs2{j}));
    if isempty(idx), continue; end

    rmse_vals = NaN(1, n_dt);
    for idt = 1:n_dt
        d = results.data(idx, idt);
        if ~d.failed, rmse_vals(idt) = d.pos_rmse; end
    end

    ok = isfinite(rmse_vals);
    if any(ok)
        plot(dt_list(ok)*1e6, rmse_vals(ok), ['-' sel_markers{j}], ...
            'Color', sel_colors{j}, 'LineWidth', 1.2, 'MarkerSize', 6, ...
            'MarkerFaceColor', sel_colors{j}, 'DisplayName', sel_cfgs2{j});
    end
end

set(ax, 'XScale', 'log', 'YScale', 'log');
xlabel('Timestep (\mus)', 'FontSize', FONT_LAB);
ylabel('Position RMSE (mm)', 'FontSize', FONT_LAB);
title('Single-step integrator error vs. timestep', 'FontSize', FONT_TITLE, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', FONT_LEG);
style_ax(ax, FONT_NAME, FONT_AX, FONT_LAB);

savefig_thesis(fig5, 'int_rmse_vs_dt', fig_dir);

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
