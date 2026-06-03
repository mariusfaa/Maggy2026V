%% run_magnet_n_sweep.m
% Sweep params.magnet.n and measure its effect on acados build time,
% solver speed, and closed-loop behavior for the maglev NMPC controllers.
%
% Resume behavior:
%   The script writes one MAT file per (variant, n, repetition) and a CSV
%   summary. On restart, completed runs are skipped automatically.
%
% Output:
%   results_magnet_n_sweep/raw/*.mat
%   results_magnet_n_sweep/magnet_n_sweep_summary.csv

clearvars; clc;

%% -------------------- USER CONFIGURATION --------------------
cfg = struct();

% --- Enter the path to your local acados installation here ---
cfg.acados_root = '';   % e.g. 'C:\Users\you\acados' or '/home/you/acados'
cfg.project_root = fileparts(mfilename('fullpath'));
if isempty(cfg.project_root)
    cfg.project_root = pwd;
end

% The values below are deliberately broad enough for an overnight run.
% Edit this vector if the largest n values become impractically slow.
cfg.n_values = [4 6 8 10 12 16 20 30 40 60 80 100];
cfg.repetitions = 3;

% 'full' reproduces workingSimulator.m.
% 'reduced' reproduces workingSimulatorReducedOrder.m.
cfg.variants = {'full', 'reduced'};

cfg.results_dir = fullfile(cfg.project_root, 'results_magnet_n_sweep');
cfg.raw_dir = fullfile(cfg.results_dir, 'raw');
cfg.summary_csv = fullfile(cfg.results_dir, 'magnet_n_sweep_summary.csv');

% Keep generated acados code from each run for reproducibility/code-size
% measurements. This uses disk space, but is useful for thesis evidence.
cfg.keep_generated_code = true;

% Set true to stop the script when one case fails. False records the error
% and continues with the next case.
cfg.stop_on_error = false;

% Compiler optimisation flags applied to CasADi-generated dynamics .c files
% (interpolated into the acados Makefile template CFLAGS). Lower than the
% acados default '-O2' so the very large inlined Jacobian translation units
% do not OOM gcc at large magnet.n.
cfg.ext_fun_compile_flags = '-O1';

% Dynamics implementation switch. 'inline' reproduces the original sweep
% (entire trapezoidal sum expanded into the symbolic graph). 'mapped' wraps
% the per-point field integrand in a casadi.Function and applies it across
% the magnet.n circumference points via .map(n), so the generated C emits
% the per-point body once instead of n times.
cfg.dynamics_impl = 'mapped';   % 'inline' | 'mapped'

% NOTE: an 'external'/'external_mapped' path was prototyped and removed
% (see analysis.md §12). The installed acados version does not expose a
% precompiled-.so dynamics interface that the casadi-codegen ABI can plug
% into directly, so Option 2 of the original plan is not feasible against
% this toolchain without either patching acados or wrapping every call
% through a generic-ABI adapter. The mapped path (Option 1) remains the
% production lever for the n ceiling.

% When dynamics_impl ~= 'inline' the run_id is suffixed with the impl tag
% so a mapped sweep does not collide with prior inline raw/MAT files in the
% same results directory.

%% -------------------- SETUP --------------------
if ~exist(cfg.results_dir, 'dir'); mkdir(cfg.results_dir); end
if ~exist(cfg.raw_dir, 'dir'); mkdir(cfg.raw_dir); end

diary_file = fullfile(cfg.results_dir, ['magnet_n_sweep_log_' datestr(now, 'yyyymmdd_HHMMSS') '.txt']);
diary(diary_file);
cleanup_diary = onCleanup(@() diary('off'));

setupProjectPaths(cfg);

metadata = collectEnvironmentMetadata(cfg);
save(fullfile(cfg.results_dir, 'experiment_metadata.mat'), 'cfg', 'metadata');

fprintf('=== Magnet n sweep started %s ===\n', datestr(now, 31));
fprintf('Results directory: %s\n', cfg.results_dir);
fprintf('Summary CSV:       %s\n', cfg.summary_csv);
fprintf('n values:          %s\n', mat2str(cfg.n_values));
fprintf('repetitions:       %d\n', cfg.repetitions);
fprintf('variants:          %s\n\n', strjoin(cfg.variants, ', '));

%% -------------------- RUN SWEEP --------------------
% Loop order is n-outer, variant-inner, rep-innermost so that an early
% abort still leaves both variants represented at every completed n.
for n_idx = 1:numel(cfg.n_values)
    magnet_n = cfg.n_values(n_idx);

    for v = 1:numel(cfg.variants)
        variant = cfg.variants{v};

        for rep = 1:cfg.repetitions
            run_id = sprintf('%s_n%04d_rep%02d%s', variant, magnet_n, rep, ...
                runIdImplSuffix(cfg.dynamics_impl));
            raw_file = fullfile(cfg.raw_dir, [run_id '.mat']);

            if isCompletedRun(raw_file, cfg.summary_csv, run_id)
                fprintf('[SKIP] %s already completed.\n', run_id);
                continue;
            end

            fprintf('\n[RUN] %s started %s\n', run_id, datestr(now, 31));
            try
                result = runSingleCase(cfg, variant, magnet_n, rep, run_id);
                result.completed = true;
                result.error_message = "";
                save(raw_file, 'result', '-v7.3');
                appendSummaryRow(cfg.summary_csv, result);
                fprintf('[DONE] %s total %.2f s, build %.2f s, mean solve %.3f ms\n', ...
                    run_id, result.total_wall_time_s, result.build_total_time_s, ...
                    result.solve_time_reported_mean_ms);
            catch ME
                result = struct();
                result.run_id = string(run_id);
                result.variant = string(variant);
                result.magnet_n = magnet_n;
                result.repetition = rep;
                result.completed = false;
                result.dynamics_impl = string(cfg.dynamics_impl);
                result.ext_fun_compile_flags = string(cfg.ext_fun_compile_flags);
                result.error_message = string(getReport(ME, 'extended', 'hyperlinks', 'off'));
                result.timestamp_end = string(datetime('now'));
                save(raw_file, 'result', '-v7.3');
                appendSummaryRow(cfg.summary_csv, result);

                fprintf(2, '[FAIL] %s\n%s\n', run_id, result.error_message);
                if cfg.stop_on_error
                    cleanWorkingCodeDir(cfg);
                    rethrow(ME);
                end
            end

            % Clear the acados working code-export dir between runs so the
            % per-run archive sizes (and disk usage) are not contaminated
            % by residue from previous iterations.
            cleanWorkingCodeDir(cfg);
        end
    end
end

fprintf('\n=== Magnet n sweep finished %s ===\n', datestr(now, 31));

%% ========================================================================
function setupProjectPaths(cfg)
    setenv('ACADOS_SOURCE_DIR',      cfg.acados_root);
    setenv('ENV_ACADOS_INSTALL_DIR', cfg.acados_root);
    setenv('ACADOS_INSTALL_DIR',     cfg.acados_root);

    addpath(fullfile(cfg.acados_root, 'interfaces', 'acados_matlab_octave'));
    addpath(fullfile(cfg.acados_root, 'external',   'jsonlab'));
    addpath(fullfile(cfg.acados_root, 'external',   'casadi-matlab'));

    addpath(genpath(fullfile(cfg.project_root, 'model_implementations')));
    addpath(genpath(fullfile(cfg.project_root, 'system_parameters')));
    addpath(genpath(fullfile(cfg.project_root, 'utilities')));
end

function metadata = collectEnvironmentMetadata(cfg)
    metadata = struct();
    metadata.timestamp = string(datetime('now'));
    metadata.matlab_version = string(version);
    metadata.computer = string(computer);
    metadata.acados_root = string(cfg.acados_root);
    metadata.project_root = string(cfg.project_root);
    metadata.script = string(mfilename('fullpath'));

    try
        metadata.casadi_version = string(casadi.CasadiMeta.version());
    catch
        metadata.casadi_version = "";
    end

    try
        [status, cmdout] = system('hostname');
        if status == 0
            metadata.hostname = string(strtrim(cmdout));
        else
            metadata.hostname = "";
        end
    catch
        metadata.hostname = "";
    end
end

function tf = isCompletedRun(raw_file, summary_csv, run_id)
    tf = false;
    if ~exist(raw_file, 'file')
        tf = isCompletedInSummary(summary_csv, run_id);
        return;
    end
    try
        s = load(raw_file, 'result');
        tf = isfield(s, 'result') && isfield(s.result, 'completed') && isequal(s.result.completed, true);
    catch
        tf = false;
    end

    if ~tf
        tf = isCompletedInSummary(summary_csv, run_id);
    end
end

function tf = isCompletedInSummary(summary_csv, run_id)
    tf = false;
    if ~exist(summary_csv, 'file')
        return;
    end
    try
        t = readtable(summary_csv, 'TextType', 'string');
        if ~any(strcmp(t.Properties.VariableNames, 'run_id')) || ...
           ~any(strcmp(t.Properties.VariableNames, 'completed'))
            return;
        end
        idx = strcmp(string(t.run_id), string(run_id));
        if any(idx)
            completed_values = string(t.completed(idx));
            tf = any(completed_values == "1" | lower(completed_values) == "true");
        end
    catch
        tf = false;
    end
end

function result = runSingleCase(cfg, variant, magnet_n, rep, run_id)
    import casadi.*

    total_tic = tic;
    timestamp_start = datetime('now');

    parameters_maggy_V4;
    params_base = params;
    params_base.magnet.n = magnet_n;

    correction_tic = tic;
    correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params_base, 'fast');
    correction_time_s = toc(correction_tic);

    if isnan(correctionFactorFast)
        error('NMPCSweep:NoEquilibrium', ...
            'no_equilibrium: computeSolenoidRadiusCorrectionFactor found no real equilibrium for n=%d.', ...
            magnet_n);
    end

    paramsFast = params_base;
    paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

    switch lower(variant)
        case 'full'
            setup = buildFullSetup(paramsFast, run_id, cfg);
        case 'reduced'
            setup = buildReducedSetup(paramsFast, run_id, cfg);
        otherwise
            error('Unknown variant "%s". Expected "full" or "reduced".', variant);
    end

    build_tic = tic;
    fprintf('  Building OCP solver...\n');
    ocp_tic = tic;
    ocp_solver = AcadosOcpSolver(setup.ocp);
    build_ocp_time_s = toc(ocp_tic);

    fprintf('  Building simulation solver...\n');
    sim_tic = tic;
    sim_solver = AcadosSimSolver(setup.sim);
    build_sim_time_s = toc(sim_tic);
    build_total_time_s = toc(build_tic);

    initializeWarmStart(ocp_solver, setup);

    sim_result = simulateClosedLoop(ocp_solver, sim_solver, setup);
    total_wall_time_s = toc(total_tic);

    generated_code_dir = archiveGeneratedCodeDirectory(cfg, setup.ocp.model.name, run_id);
    generated_code_bytes = directorySizeBytes(generated_code_dir);

    result = makeResultStruct(cfg, setup, sim_result, params_base, paramsFast, ...
        correctionFactorFast, correction_time_s, build_ocp_time_s, ...
        build_sim_time_s, build_total_time_s, total_wall_time_s, ...
        generated_code_dir, generated_code_bytes, variant, magnet_n, rep, ...
        run_id, timestamp_start);

    if ~cfg.keep_generated_code
        removeGeneratedCodeDirectory(generated_code_dir);
    end
end

function setup = buildFullSetup(paramsFast, run_id, cfg)
    import casadi.*

    nx = 12;
    nu = 4;
    x = makeSym(cfg.dynamics_impl, 'x', nx);
    u = makeSym(cfg.dynamics_impl, 'u', nu);
    xdot = makeSym(cfg.dynamics_impl, 'xdot', nx);

    graph_tic = tic;
    f_expl = buildDynamicsExpr(x, u, paramsFast, cfg.dynamics_impl);
    f_func = casadi.Function(['f_' run_id], {x, u}, {f_expl});
    graph_build_time_s = toc(graph_tic);

    graph_metrics = captureGraphMetrics(f_func, f_expl, x, u);

    [xEq, uEq, zEq, equilibrium_time_s] = findFullEquilibrium(f_func, nu, cfg.dynamics_impl);

    N = 10;
    Tf = 0.05;
    dt_mpc = Tf / N;

    Q = diag([1e2, 1e2, 1e3, ...
              1e3, 1e3, 1e1, ...
              1e1*0.3, 1e1*0.3, 1e1*0.3, ...
              1e1*0.3, 1e1*0.3, 1e0*0.3]);
    R = eye(nu) * 0.1;

    ocp = AcadosOcp();
    ocp.model.name = ['maglev_nmpc_' run_id];
    ocp.model.x = x;
    ocp.model.u = u;
    ocp.model.xdot = xdot;

    ocp.model.f_impl_expr = xdot - f_expl;

    ocp = applyCommonOcpOptions(ocp, N, Tf, 'PARTIAL_CONDENSING_HPIPM', cfg);
    ocp.cost.W = blkdiag(Q, R);
    ocp.cost.W_0 = blkdiag(Q, R);
    ocp.cost.W_e = Q * 3;
    ocp.model.cost_y_expr = [x; u];
    ocp.model.cost_y_expr_0 = [x; u];
    ocp.model.cost_y_expr_e = x;
    ocp.cost.yref = [xEq; uEq];
    ocp.cost.yref_0 = [xEq; uEq];
    ocp.cost.yref_e = xEq;
    ocp = applyCommonConstraints(ocp, nu, xEq, 1e5, 1e2);

    sim = AcadosSim();
    sim.model.name = ['maglev_sim_' run_id];
    sim.model.x = x;
    sim.model.u = u;
    sim.model.xdot = xdot;
    sim.model.f_impl_expr = xdot - f_expl;
    sim.solver_options.Tsim = dt_mpc;
    sim.solver_options.integrator_type = 'IRK';
    sim.solver_options.num_stages = 1;
    sim.solver_options.num_steps = 1;
    sim.solver_options.ext_fun_compile_flags = cfg.ext_fun_compile_flags;

    % Same experiment as the reduced model, embedded in the full state:
    % [x y z alpha beta gamma vx vy vz wx wy wz]. The reduced model omits
    % gamma and wz, so those entries are held at zero here.
    dx_dir = [0.005; -0.008; 0.010; 0.15; -0.10; 0.0; ...
              0.01; -0.01; 0.0; 0.2; -0.3; 0.0];
    perturbation_scale = 0.05;
    dist_time = 5.0;
    push_magnitude = [0; 0; 0; 0; 0; 0; -0.005; 0.0025; -0.2; 0; 0; 0];

    setup = packageSetup('full', nx, nu, N, Tf, dt_mpc, Q, R, ocp, sim, ...
        xEq, uEq, zEq, dx_dir, perturbation_scale, 2000, dist_time, ...
        push_magnitude, ...
        graph_build_time_s, equilibrium_time_s, graph_metrics, ...
        cfg.ext_fun_compile_flags, cfg.dynamics_impl);
end

function setup = buildReducedSetup(paramsFast, run_id, cfg)
    import casadi.*

    nx_full = 12;
    nx = 10;
    nu = 4;

    x_full = makeSym(cfg.dynamics_impl, 'x_full', nx_full);
    u = makeSym(cfg.dynamics_impl, 'u', nu);

    graph_tic = tic;
    f_expl_full = buildDynamicsExpr(x_full, u, paramsFast, cfg.dynamics_impl);
    f_func_full = casadi.Function(['f_full_' run_id], {x_full, u}, {f_expl_full});
    graph_build_full_time_s = toc(graph_tic);

    [~, uEq, zEq, equilibrium_time_s] = findFullEquilibrium(f_func_full, nu, cfg.dynamics_impl);
    xEq = [0; 0; zEq; zeros(7, 1)];

    x_r = makeSym(cfg.dynamics_impl, 'x_r', nx);
    xdot_r = makeSym(cfg.dynamics_impl, 'xdot_r', nx);
    x_full_from_r = [x_r(1:5); 0; x_r(6:8); x_r(9:10); 0];

    % Build the dynamics graph directly on the 10-state input. Calling
    % f_func_full(x_full_from_r, u) here would add an extra casadi.Function
    % call boundary on every evaluation -- harmless under SX (which inlines
    % function calls) but a significant runtime penalty under MX where the
    % call is preserved. By invoking buildDynamicsExpr again on the
    % x_r-derived 12-state vector, the mapped per-point Function ends up
    % directly under x_r with only its own call boundary, matching the
    % full-variant path.
    graph_tic = tic;
    f_expl_full_r = buildDynamicsExpr(x_full_from_r, u, paramsFast, cfg.dynamics_impl);
    f_expl_r = [f_expl_full_r(1:5); f_expl_full_r(7:11)];
    f_func_r = casadi.Function(['f_r_' run_id], {x_r, u}, {f_expl_r});
    graph_build_time_s = graph_build_full_time_s + toc(graph_tic);

    graph_metrics = captureGraphMetrics(f_func_r, f_expl_r, x_r, u);

    N = 10;
    Tf = 0.05;
    dt_mpc = Tf / N;

    Q = diag([1e2, 1e2, 1e3, ...
              1e3, 1e3, ...
              1e1*0.3, 1e1*0.3, 1e1*0.3, ...
              1e1*0.3, 1e1*0.3]);
    R = eye(nu) * 0.1;

    ocp = AcadosOcp();
    ocp.model.name = ['maglev_nmpc_reduced_' run_id];
    ocp.model.x = x_r;
    ocp.model.u = u;
    ocp.model.xdot = xdot_r;

    ocp.model.f_impl_expr = xdot_r - f_expl_r;

    ocp = applyCommonOcpOptions(ocp, N, Tf, 'PARTIAL_CONDENSING_HPIPM', cfg);
    ocp.cost.W = blkdiag(Q, R);
    ocp.cost.W_0 = blkdiag(Q, R);
    ocp.cost.W_e = Q * 3;
    ocp.model.cost_y_expr = [x_r; u];
    ocp.model.cost_y_expr_0 = [x_r; u];
    ocp.model.cost_y_expr_e = x_r;
    ocp.cost.yref = [xEq; uEq];
    ocp.cost.yref_0 = [xEq; uEq];
    ocp.cost.yref_e = xEq;
    ocp = applyCommonConstraints(ocp, nu, xEq, 1e3, 1e3);

    sim = AcadosSim();
    sim.model.name = ['maglev_sim_reduced_' run_id];
    sim.model.x = x_r;
    sim.model.u = u;
    sim.model.xdot = xdot_r;
    sim.model.f_impl_expr = xdot_r - f_expl_r;
    sim.solver_options.Tsim = dt_mpc;
    sim.solver_options.integrator_type = 'IRK';
    sim.solver_options.num_stages = 1;
    sim.solver_options.num_steps = 1;
    sim.solver_options.ext_fun_compile_flags = cfg.ext_fun_compile_flags;

    dx_dir = [0.005; -0.008; 0.010; 0.15; -0.10; ...
              0.01; -0.01; 0.0; 0.2; -0.3];
    perturbation_scale = 0.05;
    dist_time = 5.0;
    push_magnitude = [0; 0; 0; 0; 0; -0.005; 0.0025; -0.2; 0; 0];

    setup = packageSetup('reduced', nx, nu, N, Tf, dt_mpc, Q, R, ocp, sim, ...
        xEq, uEq, zEq, dx_dir, perturbation_scale, 2000, dist_time, ...
        push_magnitude, graph_build_time_s, equilibrium_time_s, ...
        graph_metrics, cfg.ext_fun_compile_flags, cfg.dynamics_impl);
end

function [xEq, uEq, zEq, equilibrium_time_s] = findFullEquilibrium(f_func, nu, dynamics_impl)
    import casadi.*

    equilibrium_tic = tic;
    z_var = makeSym(dynamics_impl, 'z_eq', 1);
    u_zero = zeros(nu, 1);
    x_eq_sym = [0; 0; z_var; zeros(9, 1)];
    accel = f_func(x_eq_sym, u_zero);
    fz_residual = accel(9);

    nlp = struct('x', z_var, 'f', fz_residual^2);
    solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
        struct('ipopt', struct('print_level', 0), 'print_time', false));

    sol = solver_eq('x0', 0.030, 'lbx', 0.015, 'ubx', 0.060);
    zEq = full(sol.x);
    uEq = zeros(nu, 1);
    xEq = [0; 0; zEq; zeros(9, 1)];
    equilibrium_time_s = toc(equilibrium_tic);
end

function ocp = applyCommonOcpOptions(ocp, N, Tf, qp_solver, cfg)
    ocp.solver_options.N_horizon = N;
    ocp.solver_options.tf = Tf;
    ocp.solver_options.integrator_type = 'IRK';
    ocp.solver_options.sim_method_num_stages = 1;
    ocp.solver_options.sim_method_num_steps = 1;
    ocp.solver_options.nlp_solver_type = 'SQP_RTI';
    ocp.solver_options.nlp_solver_tol_stat = 1e-4;
    ocp.solver_options.nlp_solver_tol_eq = 1e-4;
    ocp.solver_options.nlp_solver_tol_ineq = 1e-4;
    ocp.solver_options.nlp_solver_tol_comp = 1e-4;
    ocp.solver_options.qp_solver = qp_solver;
    ocp.solver_options.qp_solver_iter_max = 200;
    ocp.solver_options.qp_solver_warm_start = 1;
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
    ocp.solver_options.regularize_method = 'CONVEXIFY';
    ocp.solver_options.ext_fun_compile_flags = cfg.ext_fun_compile_flags;

    ocp.cost.cost_type =   'NONLINEAR_LS';
    ocp.cost.cost_type_0 = 'NONLINEAR_LS';
    ocp.cost.cost_type_e = 'NONLINEAR_LS';
end

function ocp = applyCommonConstraints(ocp, nu, xEq, slack_Z, slack_z)
    ocp.constraints.idxbu = 0:nu-1;
    ocp.constraints.lbu = -1 * ones(nu, 1);
    ocp.constraints.ubu =  1 * ones(nu, 1);

    ocp.constraints.idxbx = [0, 1, 2, 3, 4];
    ocp.constraints.lbx = [-0.025; -0.025; 0.015; -0.35; -0.35];
    ocp.constraints.ubx = [ 0.025;  0.025; 0.055;  0.35;  0.35];
    ocp.constraints.idxsbx = 0:4;

    n_sbx = 5;
    ocp.cost.Zl = slack_Z * ones(n_sbx, 1);
    ocp.cost.Zu = slack_Z * ones(n_sbx, 1);
    ocp.cost.zl = slack_z * ones(n_sbx, 1);
    ocp.cost.zu = slack_z * ones(n_sbx, 1);

    ocp.constraints.x0 = xEq;
end

function setup = packageSetup(variant, nx, nu, N, Tf, dt_mpc, Q, R, ocp, sim, ...
        xEq, uEq, zEq, dx_dir, perturbation_scale, sim_steps, dist_time, ...
        push_magnitude, graph_build_time_s, equilibrium_time_s, ...
        graph_metrics, ext_fun_compile_flags, dynamics_impl)
    setup = struct();
    setup.variant = string(variant);
    setup.nx = nx;
    setup.nu = nu;
    setup.N = N;
    setup.Tf = Tf;
    setup.dt = dt_mpc;
    setup.Q = Q;
    setup.R = R;
    setup.ocp = ocp;
    setup.sim = sim;
    setup.xEq = xEq;
    setup.uEq = uEq;
    setup.zEq = zEq;
    setup.dx_dir = dx_dir;
    setup.perturbation_scale = perturbation_scale;
    setup.x0 = xEq + perturbation_scale * dx_dir;
    setup.sim_steps = sim_steps;
    setup.dist_time = dist_time;
    setup.push_magnitude = push_magnitude;
    setup.graph_build_time_s = graph_build_time_s;
    setup.equilibrium_time_s = equilibrium_time_s;
    setup.graph_metrics = graph_metrics;
    setup.ext_fun_compile_flags = string(ext_fun_compile_flags);
    setup.dynamics_impl = string(dynamics_impl);
end

function suffix = runIdImplSuffix(dynamics_impl)
    % Produce a run_id suffix that distinguishes non-default dynamics
    % implementations. Empty for the baseline ('inline') so existing raw
    % files and CSV rows remain reachable.
    switch lower(string(dynamics_impl))
        case "inline"
            suffix = '';
        otherwise
            suffix = ['_' char(lower(string(dynamics_impl)))];
    end
end

function s = makeSym(dynamics_impl, name, n)
    % Sym constructor matched to the dynamics implementation. 'mapped'
    % requires MX so that f_point.map(magnet_n) is preserved as a single
    % call in the host graph (SX inlines function calls into the expanded
    % graph, defeating the purpose of .map).
    import casadi.*
    switch lower(string(dynamics_impl))
        case "mapped"
            s = MX.sym(name, n);
        otherwise
            s = SX.sym(name, n);
    end
end

function f_expl = buildDynamicsExpr(x, u, paramsFast, dynamics_impl)
    % Dispatch between the inline trapezoidal sum (original, n-fold inlined)
    % and the mapped variant (single-point field Function applied via
    % .map(magnet_n)). Both produce a 12x1 explicit-ODE expression in (x,u).
    switch lower(string(dynamics_impl))
        case "inline"
            f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
        case "mapped"
            f_expl = maglevSystemDynamicsCasADiMapped(x, u, paramsFast);
        otherwise
            error('NMPCSweep:UnknownDynamicsImpl', ...
                'Unknown cfg.dynamics_impl "%s" (expected inline|mapped).', ...
                dynamics_impl);
    end
end

function metrics = captureGraphMetrics(f_func, f_expl, x, u)
    % Real CasADi graph metrics for the dynamics expression that acados
    % codegens. Replaces the post-hoc .c-line-count proxy used by the prior
    % sweep. nnz_jac_x / nnz_jac_u summarise the size of the analytic
    % Jacobian that gcc has to compile.
    metrics = struct();
    try
        metrics.n_instructions = double(f_func.n_instructions());
    catch
        metrics.n_instructions = NaN;
    end
    try
        metrics.n_nodes = double(f_func.n_nodes());
    catch
        metrics.n_nodes = NaN;
    end
    try
        metrics.nnz_jac_x = double(jacobian(f_expl, x).nnz());
    catch
        metrics.nnz_jac_x = NaN;
    end
    try
        metrics.nnz_jac_u = double(jacobian(f_expl, u).nnz());
    catch
        metrics.nnz_jac_u = NaN;
    end
end

function initializeWarmStart(ocp_solver, setup)
    for k = 0:setup.N
        alpha_k = k / setup.N;
        ocp_solver.set('x', (1 - alpha_k) * setup.x0 + alpha_k * setup.xEq, k);
    end
    for k = 0:setup.N-1
        ocp_solver.set('u', setup.uEq, k);
    end
end

function sim_result = simulateClosedLoop(ocp_solver, sim_solver, setup)
    nx = setup.nx;
    nu = setup.nu;
    sim_steps = setup.sim_steps;
    dt = setup.dt;

    x_current = setup.x0;

    x_log = nan(nx, sim_steps + 1);
    u_log = nan(nu, sim_steps);
    status_log = nan(1, sim_steps);
    sqp_iter_log = nan(1, sim_steps);
    solve_time_reported_s = nan(1, sim_steps);
    solve_wall_time_s = nan(1, sim_steps);
    sim_wall_time_s = nan(1, sim_steps);
    state_error_norm = nan(1, sim_steps + 1);
    position_error_norm_m = nan(1, sim_steps + 1);
    angle_norm_rad = nan(1, sim_steps + 1);

    x_log(:, 1) = x_current;
    state_error_norm(1) = norm(x_current - setup.xEq);
    position_error_norm_m(1) = norm(x_current(1:3) - setup.xEq(1:3));
    angle_norm_rad(1) = norm(x_current(4:min(6, nx)));

    n_actual = 0;
    diverged = false;
    disturbance_applied = false;

    if isempty(setup.dist_time)
        dist_step = inf;
    else
        dist_step = round(setup.dist_time / dt);
    end

    for i = 1:sim_steps
        ocp_solver.set('constr_x0', x_current);

        solve_tic = tic;
        ocp_solver.solve();
        solve_wall_time_s(i) = toc(solve_tic);

        status_log(i) = ocp_solver.get('status');
        sqp_iter_log(i) = ocp_solver.get('sqp_iter');
        solve_time_reported_s(i) = ocp_solver.get('time_tot');
        u_applied = ocp_solver.get('u', 0);
        u_applied = max(min(u_applied, 1), -1);

        sim_tic = tic;
        sim_solver.set('x', x_current);
        sim_solver.set('u', u_applied);
        sim_solver.solve();
        x_next = sim_solver.get('xn');
        sim_wall_time_s(i) = toc(sim_tic);

        if i == dist_step
            x_next = x_next + setup.push_magnitude;
            disturbance_applied = true;
        end

        x_traj = ocp_solver.get('x');
        u_traj = ocp_solver.get('u');
        x_traj_shift = [x_traj(:, 2:end), setup.xEq];
        u_traj_shift = [u_traj(:, 2:end), setup.uEq];
        for k = 0:setup.N
            ocp_solver.set('x', x_traj_shift(:, k + 1), k);
        end
        for k = 0:setup.N-1
            ocp_solver.set('u', u_traj_shift(:, k + 1), k);
        end

        x_log(:, i + 1) = x_next;
        u_log(:, i) = u_applied;
        state_error_norm(i + 1) = norm(x_next - setup.xEq);
        position_error_norm_m(i + 1) = norm(x_next(1:3) - setup.xEq(1:3));
        if nx == 12
            angle_norm_rad(i + 1) = norm(x_next(4:6));
            diverged = abs(x_next(3)) > 0.5 || max(abs(x_next(4:6))) > pi || ...
                any(isnan(x_next)) || any(isinf(x_next));
        else
            angle_norm_rad(i + 1) = norm(x_next(4:5));
            diverged = abs(x_next(3)) > 0.5 || max(abs(x_next(4:5))) > pi || ...
                any(isnan(x_next)) || any(isinf(x_next));
        end

        n_actual = i;
        x_current = x_next;

        if i <= 5 || mod(i, 100) == 0 || i == sim_steps || diverged
            fprintf('  step %4d/%4d: st=%d, reported=%.3f ms, wall=%.3f ms, |e|=%.3e\n', ...
                i, sim_steps, status_log(i), solve_time_reported_s(i) * 1000, ...
                solve_wall_time_s(i) * 1000, state_error_norm(i + 1));
        end

        if diverged
            fprintf('  Diverged at step %d.\n', i);
            break;
        end
    end

    idx = 1:n_actual;
    idx_x = 1:(n_actual + 1);

    sim_result = struct();
    sim_result.n_actual = n_actual;
    sim_result.diverged = diverged;
    sim_result.disturbance_applied = disturbance_applied;
    sim_result.x = x_log(:, idx_x);
    sim_result.u = u_log(:, idx);
    sim_result.status = status_log(idx);
    sim_result.sqp_iter = sqp_iter_log(idx);
    sim_result.solve_time_reported_s = solve_time_reported_s(idx);
    sim_result.solve_wall_time_s = solve_wall_time_s(idx);
    sim_result.sim_wall_time_s = sim_wall_time_s(idx);
    sim_result.state_error_norm = state_error_norm(idx_x);
    sim_result.position_error_norm_m = position_error_norm_m(idx_x);
    sim_result.angle_norm_rad = angle_norm_rad(idx_x);
end

function result = makeResultStruct(cfg, setup, sim_result, params_base, paramsFast, ...
        correctionFactorFast, correction_time_s, build_ocp_time_s, ...
        build_sim_time_s, build_total_time_s, total_wall_time_s, ...
        generated_code_dir, generated_code_bytes, variant, magnet_n, rep, ...
        run_id, timestamp_start)

    st = sim_result.status;
    solve_reported_ms = sim_result.solve_time_reported_s * 1000;
    solve_wall_ms = sim_result.solve_wall_time_s * 1000;
    sim_wall_ms = sim_result.sim_wall_time_s * 1000;

    result = struct();
    result.run_id = string(run_id);
    result.variant = string(variant);
    result.magnet_n = magnet_n;
    result.repetition = rep;
    result.timestamp_start = string(timestamp_start);
    result.timestamp_end = string(datetime('now'));

    result.nx = setup.nx;
    result.nu = setup.nu;
    result.N = setup.N;
    result.Tf = setup.Tf;
    result.dt = setup.dt;
    result.sim_steps_requested = setup.sim_steps;
    result.sim_steps_completed = sim_result.n_actual;
    if isempty(setup.dist_time)
        result.disturbance_time_s = NaN;
    else
        result.disturbance_time_s = setup.dist_time;
    end
    result.disturbance_applied = sim_result.disturbance_applied;
    result.perturbation_scale = setup.perturbation_scale;

    result.graph_build_time_s = setup.graph_build_time_s;
    result.equilibrium_time_s = setup.equilibrium_time_s;
    result.n_instructions = setup.graph_metrics.n_instructions;
    result.n_nodes = setup.graph_metrics.n_nodes;
    result.nnz_jac_x = setup.graph_metrics.nnz_jac_x;
    result.nnz_jac_u = setup.graph_metrics.nnz_jac_u;
    result.ext_fun_compile_flags = setup.ext_fun_compile_flags;
    result.dynamics_impl = setup.dynamics_impl;
    result.correction_factor_fast = correctionFactorFast;
    result.correction_time_s = correction_time_s;
    result.build_ocp_time_s = build_ocp_time_s;
    result.build_sim_time_s = build_sim_time_s;
    result.build_total_time_s = build_total_time_s;
    result.total_wall_time_s = total_wall_time_s;
    result.generated_code_dir = string(generated_code_dir);
    result.generated_code_bytes = generated_code_bytes;

    result.zEq_m = setup.zEq;
    result.final_state_error_norm = sim_result.state_error_norm(end);
    result.final_position_error_m = sim_result.position_error_norm_m(end);
    result.final_angle_norm_deg = rad2deg(sim_result.angle_norm_rad(end));
    result.rms_state_error_norm = rmsNoToolbox(sim_result.state_error_norm);
    result.rms_position_error_m = rmsNoToolbox(sim_result.position_error_norm_m);
    result.max_position_error_m = max(sim_result.position_error_norm_m);
    result.max_angle_norm_deg = rad2deg(max(sim_result.angle_norm_rad));

    result.status_ok_count = sum(st == 0);
    result.status_max_iter_count = sum(st == 2);
    result.status_qp_fail_count = sum(st == 4);
    result.status_nonzero_count = sum(st ~= 0);
    result.diverged = sim_result.diverged;

    result.sqp_iter_mean = mean(sim_result.sqp_iter, 'omitnan');
    result.sqp_iter_max = max(sim_result.sqp_iter);
    result.solve_time_reported_mean_ms = mean(solve_reported_ms, 'omitnan');
    result.solve_time_reported_median_ms = median(solve_reported_ms, 'omitnan');
    result.solve_time_reported_p95_ms = percentileNoToolbox(solve_reported_ms, 95);
    result.solve_time_reported_max_ms = max(solve_reported_ms);
    result.solve_wall_mean_ms = mean(solve_wall_ms, 'omitnan');
    result.solve_wall_median_ms = median(solve_wall_ms, 'omitnan');
    result.solve_wall_p95_ms = percentileNoToolbox(solve_wall_ms, 95);
    result.solve_wall_max_ms = max(solve_wall_ms);
    result.sim_wall_mean_ms = mean(sim_wall_ms, 'omitnan');
    result.sim_wall_max_ms = max(sim_wall_ms);
    result.mean_control_abs = mean(abs(sim_result.u(:)), 'omitnan');
    result.max_control_abs = max(abs(sim_result.u(:)));

    result.params_base = params_base;
    result.params_fast = paramsFast;
    result.xEq = setup.xEq;
    result.uEq = setup.uEq;
    result.Q = setup.Q;
    result.R = setup.R;
    result.raw = sim_result;
    result.cfg_snapshot = cfg;
end

function appendSummaryRow(summary_csv, result)
    row = summaryRow(result);
    if exist(summary_csv, 'file')
        old = readtable(summary_csv, 'TextType', 'string');
        if any(strcmp(old.Properties.VariableNames, 'run_id'))
            old(strcmp(string(old.run_id), string(result.run_id)), :) = [];
        end
        try
            [old, row] = alignTableSchemas(old, row);
            out = [old; row];
        catch ME
            backup = [summary_csv '.broken_' datestr(now, 'yyyymmdd_HHMMSS')];
            movefile(summary_csv, backup);
            fprintf(2, ['[WARN] appendSummaryRow could not merge with ' ...
                'existing CSV (%s); moved old file to %s and starting fresh.\n'], ...
                ME.message, backup);
            out = row;
        end
    else
        out = row;
    end
    writetable(out, summary_csv);
end

function [a, b] = alignTableSchemas(a, b)
    % Make tables a and b share the same set of columns, in b's order,
    % filling any missing columns with type-appropriate defaults. Lets the
    % CSV survive schema additions (e.g. new graph-metric columns) across
    % sweeps without forcing the user to wipe the prior CSV.
    names_a = a.Properties.VariableNames;
    names_b = b.Properties.VariableNames;

    for i = 1:numel(names_b)
        name = names_b{i};
        if ~any(strcmp(names_a, name))
            a.(name) = repmat(missingValueForName(name), height(a), 1);
        end
    end
    for i = 1:numel(names_a)
        name = names_a{i};
        if ~any(strcmp(names_b, name))
            b.(name) = repmat(missingValueForName(name), height(b), 1);
        end
    end

    a = a(:, names_b(ismember(names_b, a.Properties.VariableNames)));
    % After the loops above a now has every column in names_b plus possibly
    % extras; reorder to b's order, appending any extras at the end.
    final_order = [names_b, setdiff(a.Properties.VariableNames, names_b, 'stable')];
    a = a(:, final_order);
    b = b(:, final_order);
end

function row = summaryRow(r)
    names = summaryVariableNames();
    vals = cell(1, numel(names));
    for i = 1:numel(names)
        name = names{i};
        if isfield(r, name)
            vals{i} = r.(name);
        else
            vals{i} = missingValueForName(name);
        end
    end
    row = cell2table(vals, 'VariableNames', names);
end

function names = summaryVariableNames()
    names = {'run_id', 'variant', 'magnet_n', 'repetition', 'completed', ...
        'timestamp_start', 'timestamp_end', 'nx', 'N', 'Tf', 'dt', ...
        'sim_steps_requested', 'sim_steps_completed', 'disturbance_time_s', ...
        'disturbance_applied', 'perturbation_scale', 'graph_build_time_s', ...
        'equilibrium_time_s', 'n_instructions', 'n_nodes', 'nnz_jac_x', ...
        'nnz_jac_u', 'ext_fun_compile_flags', 'dynamics_impl', ...
        'correction_factor_fast', 'correction_time_s', ...
        'build_ocp_time_s', 'build_sim_time_s', 'build_total_time_s', ...
        'total_wall_time_s', 'generated_code_bytes', 'zEq_m', ...
        'status_ok_count', 'status_max_iter_count', 'status_qp_fail_count', ...
        'status_nonzero_count', 'diverged', 'sqp_iter_mean', 'sqp_iter_max', ...
        'solve_time_reported_mean_ms', 'solve_time_reported_median_ms', ...
        'solve_time_reported_p95_ms', 'solve_time_reported_max_ms', ...
        'solve_wall_mean_ms', 'solve_wall_median_ms', 'solve_wall_p95_ms', ...
        'solve_wall_max_ms', 'sim_wall_mean_ms', 'sim_wall_max_ms', ...
        'final_state_error_norm', 'final_position_error_m', ...
        'final_angle_norm_deg', 'rms_state_error_norm', ...
        'rms_position_error_m', 'max_position_error_m', ...
        'max_angle_norm_deg', 'mean_control_abs', 'max_control_abs', ...
        'generated_code_dir', 'error_message'};
end

function value = missingValueForName(name)
    string_names = {'run_id', 'variant', 'timestamp_start', 'timestamp_end', ...
        'generated_code_dir', 'error_message', 'ext_fun_compile_flags', ...
        'dynamics_impl'};
    logical_names = {'completed', 'disturbance_applied', 'diverged'};
    if any(strcmp(name, string_names))
        value = "";
    elseif any(strcmp(name, logical_names))
        value = false;
    else
        value = NaN;
    end
end

function p = percentileNoToolbox(x, pct)
    x = x(~isnan(x));
    if isempty(x)
        p = NaN;
        return;
    end
    x = sort(x(:));
    rank = 1 + (numel(x) - 1) * pct / 100;
    lo = floor(rank);
    hi = ceil(rank);
    if lo == hi
        p = x(lo);
    else
        p = x(lo) + (rank - lo) * (x(hi) - x(lo));
    end
end

function y = rmsNoToolbox(x)
    x = x(~isnan(x));
    if isempty(x)
        y = NaN;
    else
        y = sqrt(mean(x.^2));
    end
end

function generated_code_dir = findGeneratedCodeDirectory(project_root, model_name)
    candidates = { ...
        fullfile(project_root, 'c_generated_code'), ...
        fullfile(project_root, [model_name '_c_generated_code']), ...
        fullfile(project_root, 'acados_ocp_' + string(model_name))};

    generated_code_dir = "";
    for i = 1:numel(candidates)
        if exist(candidates{i}, 'dir')
            generated_code_dir = string(candidates{i});
            return;
        end
    end
end

function archived_dir = archiveGeneratedCodeDirectory(cfg, model_name, run_id)
    generated_code_dir = findGeneratedCodeDirectory(cfg.project_root, model_name);
    if strlength(string(generated_code_dir)) == 0 || ~exist(char(generated_code_dir), 'dir')
        archived_dir = "";
        return;
    end

    if ~cfg.keep_generated_code
        archived_dir = generated_code_dir;
        return;
    end

    archive_root = fullfile(cfg.results_dir, 'generated_code');
    if ~exist(archive_root, 'dir'); mkdir(archive_root); end

    archived_dir = string(fullfile(archive_root, run_id));
    if exist(char(archived_dir), 'dir')
        rmdir(char(archived_dir), 's');
    end
    copyfile(char(generated_code_dir), char(archived_dir));
end

function bytes = directorySizeBytes(dir_name)
    bytes = NaN;
    if strlength(string(dir_name)) == 0 || ~exist(char(dir_name), 'dir')
        return;
    end
    listing = dir(fullfile(char(dir_name), '**', '*'));
    bytes = sum([listing(~[listing.isdir]).bytes]);
end

function cleanWorkingCodeDir(cfg)
    target = fullfile(cfg.project_root, 'c_generated_code');
    if exist(target, 'dir')
        % acados / setupProjectPaths put this dir on the MATLAB path during
        % the build, so rmdir triggers a benign "Removed ... from the MATLAB
        % path" warning. Suppress that one ID only.
        warn_state = warning('off', 'MATLAB:RMDIR:RemovedFromPath');
        cleanup = onCleanup(@() warning(warn_state));
        try
            rmdir(target, 's');
        catch ME
            fprintf(2, '[WARN] cleanWorkingCodeDir failed: %s\n', ME.message);
        end
    end
end

function removeGeneratedCodeDirectory(dir_name)
    if strlength(string(dir_name)) == 0 || ~exist(char(dir_name), 'dir')
        return;
    end
    project_root = string(fileparts(mfilename('fullpath')));
    target = string(char(java.io.File(char(dir_name)).getCanonicalPath()));
    root = string(char(java.io.File(char(project_root)).getCanonicalPath()));
    if startsWith(target, root)
        rmdir(char(target), 's');
    end
end
