%% simBenchmark — Sweep sim solver settings and compare performance
%
% Loops over combinations of integrator settings, runs identical open-loop
% simulations, and saves results for comparison with simCompare.

simSetup;
import casadi.*

nx_red = 10;
x0_red = x0([1:5,7:11]);
xEq_red = xEq([1:5,7:11]);

output_dir = 'simresults';
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

%% --- Define parameter grid ---
configs = struct([]);

% Integrator types to test
integrators  = {'ERK', 'IRK'};
stages_list  = [1, 2, 4];
steps_list   = [1, 2, 4];
n_radial     = [8, 12, 16];

idx = 0;
for iInt = 1:length(integrators)
    for iStg = 1:length(stages_list)
        for iStp = 1:length(steps_list)
            for iN = 1:length(n_radial)
                idx = idx + 1;
                configs(idx).integrator  = integrators{iInt};
                configs(idx).num_stages  = stages_list(iStg);
                configs(idx).num_steps   = steps_list(iStp);
                configs(idx).n_radial    = n_radial(iN);
            end
        end
    end
end

nConfigs = length(configs);
fprintf('\n=== simBenchmark: %d configurations ===\n\n', nConfigs);

%% --- Summary table header ---
summary = cell(nConfigs, 1);

%% --- Run each configuration ---
for ic = 1:nConfigs
    cfg = configs(ic);
    tag = sprintf('%s_s%d_n%d_nr%d', cfg.integrator, cfg.num_stages, ...
        cfg.num_steps, cfg.n_radial);
    save_file = fullfile(output_dir, ['sim_' tag '.mat']);

    fprintf('\n--- [%d/%d] %s ---\n', ic, nConfigs, tag);

    % Skip if result already exists
    if exist(save_file, 'file')
        fprintf('  Already exists, skipping.\n');
        summary{ic} = sprintf('%-30s  SKIPPED (exists)', tag);
        continue;
    end

    try
        % Build model with this config's radial discretization
        params_cfg = load_params(MaglevModel.Fast);
        params_cfg.magnet.n = cfg.n_radial;
        params_cfg.lut_opts.enabled = false;

        % Build CasADi dynamics
        x_sym    = MX.sym('x',    nx_red);
        u_sym    = MX.sym('u',    nu);
        xdot_sym = MX.sym('xdot', nx_red);

        f_expl = maglevSystemDynamicsReduced_casadi(x_sym, u_sym, params_cfg, MaglevModel.Fast);

        mdl = AcadosModel();
        mdl.name        = ['bench_' tag];
        mdl.x           = x_sym;
        mdl.u           = u_sym;
        mdl.xdot        = xdot_sym;
        mdl.f_impl_expr = xdot_sym - f_expl;
        mdl.f_expl_expr = f_expl;

        % Build sim solver
        sim = AcadosSim();
        sim.model = mdl;
        sim.solver_options.Tsim            = dt;
        sim.solver_options.integrator_type = cfg.integrator;
        sim.solver_options.num_stages      = cfg.num_stages;
        sim.solver_options.num_steps       = cfg.num_steps;
        sim.solver_options.ext_fun_compile_flags = '-O2';

        tic_build = tic;
        solver = AcadosSimSolver(sim);
        t_build = toc(tic_build);
        fprintf('  Build time: %.1f s\n', t_build);

        % Run simulation
        t_vec = 0:dt:0.5;  % local copy — never modify the original
        N_sim = numel(t_vec);
        x_traj = zeros(nx_red, N_sim);
        u_traj = zeros(nu, N_sim);
        t_tot  = zeros(1, N_sim);
        t_la   = zeros(1, N_sim);
        t_ad   = zeros(1, N_sim);

        x_cur = x0_red;
        u_cur = u0;
        diverged = false;
        n_steps_run = N_sim;

        for i = 1:N_sim
            x_traj(:, i) = x_cur;
            u_traj(:, i) = u_cur;

            x_cur = solver.simulate(x_cur, u_cur);
            t_tot(i) = solver.get('time_tot');
            t_la(i)  = solver.get('time_la');
            t_ad(i)  = solver.get('time_ad');

            diverged = abs(x_cur(3)) > 0.5       || ...
                       max(abs(x_cur(4:5))) > pi  || ...
                       any(isnan(x_cur))          || ...
                       any(isinf(x_cur));

            if diverged
                fprintf('  DIVERGED at step %d\n', i);
                n_steps_run = i;
                x_traj = x_traj(:, 1:i);
                u_traj = u_traj(:, 1:i);
                t_vec  = t_vec(1:i);
                t_tot  = t_tot(1:i);
                t_la   = t_la(1:i);
                t_ad   = t_ad(1:i);
                break;
            end
        end

        % Performance stats
        med_us = median(t_tot) * 1e6;
        avg_us = mean(t_tot) * 1e6;
        max_us = max(t_tot) * 1e6;

        fprintf('  Steps: %d  |  median=%.0f us  avg=%.0f us  max=%.0f us\n', ...
            n_steps_run, med_us, avg_us, max_us);

        % Save (compatible with simCompare)
        sim_data      = struct();
        sim_data.t    = t_vec;
        sim_data.x    = x_traj;
        sim_data.u    = u_traj;
        sim_data.xEq  = xEq_red;
        sim_data.uEq  = uEq;
        sim_data.dt   = dt;
        sim_data.t_tot = t_tot;
        sim_data.t_la  = t_la;
        sim_data.t_ad  = t_ad;
        sim_data.config  = cfg;
        sim_data.t_build = t_build;

        save(save_file, '-struct', 'sim_data');
        fprintf('  Saved: %s\n', save_file);

        summary{ic} = sprintf('%-30s  med=%5.0f us  avg=%5.0f us  max=%5.0f us  steps=%d', ...
            tag, med_us, avg_us, max_us, n_steps_run);

        % Clean up solver to force rebuild next iteration
        clear solver sim mdl f_expl x_sym u_sym xdot_sym;

    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        summary{ic} = sprintf('%-30s  ERROR: %s', tag, ME.message);
    end
end

%% --- Print summary table ---
fprintf('\n\n========== BENCHMARK SUMMARY ==========\n');
fprintf('%-30s  %-50s\n', 'Configuration', 'Results');
fprintf('%s\n', repmat('-', 1, 82));
for ic = 1:nConfigs
    fprintf('%s\n', summary{ic});
end
fprintf('\nResults saved in: %s/\n', output_dir);
