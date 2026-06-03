function R = run_closed_loop(ocp_solver, plant_solver, M, cfg)
% RUN_CLOSED_LOOP  Execute one closed-loop NMPC simulation and return a raw record.
%
%   R = run_closed_loop(ocp_solver, plant_solver, M, cfg)
%
% Inputs (caller is responsible for validate_cfg before calling)
%   ocp_solver     AcadosOcpSolver from build_ocp
%   plant_solver   AcadosSimSolver from build_plant
%   M              struct from build_model
%   cfg            validated cfg (struct)
%
% Output struct R
%   .X_plant       12 x (n_actual+1)        plant trajectory
%   .X_ctrl        nx_ctrl x (n_actual+1)   controller-side trajectory
%   .U             4 x n_actual             applied inputs (post-clipping)
%   .status_seq    1 x n_actual             acados per-step status
%   .sqp_iter_seq  1 x n_actual             SQP iterations per step
%   .qp_iter_seq   1 x n_actual             summed QP iterations per step
%   .timing        struct of 1 x n_actual arrays
%       .tot       acados-reported total time per step
%       .lin       linearization
%       .qp_sol    QP solve time
%       .reg       regularization
%       .sim       integrator
%       .sim_ad    integrator adjoint
%       .sim_la    integrator linear-algebra
%       .glob      global / overhead
%       .wall      tic/toc around set(constr_x0)+solve+get(u,0)
%   .n_actual      number of steps actually completed
%   .diverged      true if the run broke early on a divergence check
%   .dt            Ts = Tf / N
%   .disturbance_step  step index at which the disturbance was injected,
%                       or NaN if there was no disturbance
%
% Warmstart strategies (cfg.warmstart_strategy)
%   'linear_to_eq'      INITIAL: linear interp from x0_ctrl to xEq_ctrl;
%                        INTRA-LOOP: shift previous trajectory by 1, append xEq/uEq
%                        (this matches workingSimulator.m)
%   'shift_trajectory'  INITIAL: none (acados defaults); INTRA-LOOP: shift
%   'cold'              INITIAL: none; INTRA-LOOP: none
%   'previous_run'      INITIAL: none; INTRA-LOOP: none (relies on acados
%                        internal state across solves; equivalent to 'cold'
%                        when ocp_solver is freshly built).
%
% Divergence is detected with the same thresholds as workingSimulator.m:
% |z_plant| > 0.5, |angles_plant| > pi, or any NaN/Inf in the state.

    if nargin < 4; error('run_closed_loop:nargin','Usage: run_closed_loop(ocp_solver, plant_solver, M, cfg)'); end

    N         = cfg.N;
    sim_steps = cfg.sim_steps;
    nx_ctrl   = M.nx_ctrl;
    nu        = M.nu;
    dt        = cfg.Tf / N;

    % --- Initial state -------------------------------------------------
    ic = cfg.ic_descriptor;
    x_plant = M.xEq_plant + ic.scale * ic.direction(:);
    x_ctrl  = M.project_to_ctrl(x_plant);

    % --- Pre-allocate logs --------------------------------------------
    X_plant     = zeros(12,      sim_steps + 1);
    X_ctrl      = zeros(nx_ctrl, sim_steps + 1);
    U           = zeros(nu,      sim_steps);
    status_seq  = zeros(1, sim_steps);
    sqp_iter_seq= zeros(1, sim_steps);
    qp_iter_seq = nan(1,   sim_steps);
    T_tot   = nan(1, sim_steps);
    T_lin   = nan(1, sim_steps);
    T_qp    = nan(1, sim_steps);
    T_reg   = nan(1, sim_steps);
    T_sim   = nan(1, sim_steps);
    T_simad = nan(1, sim_steps);
    T_simla = nan(1, sim_steps);
    T_glob  = nan(1, sim_steps);
    T_wall  = nan(1, sim_steps);

    X_plant(:, 1) = x_plant;
    X_ctrl (:, 1) = x_ctrl;

    % --- Resolve disturbance schedule ----------------------------------
    if ~isempty(cfg.disturbance_schedule)
        ds = cfg.disturbance_schedule;
        dist_step = max(1, min(sim_steps, round(ds.t_apply / dt)));
        dist_vec  = ds.vector(:);
        if numel(dist_vec) ~= 12
            error('run_closed_loop:bad_disturbance', ...
                  'cfg.disturbance_schedule.vector must be 12-element (plant-side); got %d', numel(dist_vec));
        end
        if ~strcmp(ds.kind, 'state_additive')
            error('run_closed_loop:unsupported_disturbance', ...
                  'Only kind="state_additive" supported (got "%s")', ds.kind);
        end
    else
        dist_step = NaN;
        dist_vec  = [];
    end

    % --- Initial warmstart --------------------------------------------
    apply_initial_warmstart(ocp_solver, M, cfg, x_ctrl);

    % --- Main loop -----------------------------------------------------
    diverged = false;
    n_actual = sim_steps;
    for i = 1:sim_steps
        % --- Time the solve (wall-clock includes set/solve/get) -------
        twall = tic;
        ocp_solver.set('constr_x0', x_ctrl);
        ocp_solver.solve();
        u0 = ocp_solver.get('u', 0);
        T_wall(i) = toc(twall);

        % --- Status & iteration counts -------------------------------
        status_seq(i)   = ocp_solver.get('status');
        try; sqp_iter_seq(i) = ocp_solver.get('sqp_iter'); catch; end
        try; qp_iter_seq(i)  = ocp_solver.get('qp_iter');  catch; end

        % --- Timing breakdown (try each; acados omits some by config) -
        T_tot(i)   = try_get(ocp_solver, 'time_tot');
        T_lin(i)   = try_get(ocp_solver, 'time_lin');
        T_qp(i)    = try_get(ocp_solver, 'time_qp_sol');
        T_reg(i)   = try_get(ocp_solver, 'time_reg');
        T_sim(i)   = try_get(ocp_solver, 'time_sim');
        T_simad(i) = try_get(ocp_solver, 'time_sim_ad');
        T_simla(i) = try_get(ocp_solver, 'time_sim_la');
        T_glob(i)  = try_get(ocp_solver, 'time_glob');

        % --- Saturate input and apply to plant ------------------------
        u0 = max(min(u0, cfg.ubu(:)), cfg.lbu(:));
        U(:, i) = u0;

        plant_solver.set('x', x_plant);
        plant_solver.set('u', u0);
        plant_solver.solve();
        x_next_plant = plant_solver.get('xn');

        % --- Disturbance ----------------------------------------------
        if ~isnan(dist_step) && i == dist_step
            x_next_plant = x_next_plant + dist_vec;
        end

        % --- Project to controller side ------------------------------
        x_next_ctrl = M.project_to_ctrl(x_next_plant);

        % --- Intra-loop warmstart ------------------------------------
        apply_intra_warmstart(ocp_solver, M, cfg);

        % --- Log ------------------------------------------------------
        X_plant(:, i+1) = x_next_plant;
        X_ctrl (:, i+1) = x_next_ctrl;

        % --- Divergence check (matches workingSimulator.m) -----------
        if abs(x_next_plant(3)) > 0.5 ...
                || max(abs(x_next_plant(4:6))) > pi ...
                || any(~isfinite(x_next_plant)) ...
                || any(~isfinite(u0))
            diverged = true;
            n_actual = i;
            break;
        end

        % --- Advance --------------------------------------------------
        x_plant = x_next_plant;
        x_ctrl  = x_next_ctrl;
    end

    % --- Pack ---------------------------------------------------------
    R = struct();
    R.X_plant       = X_plant(:, 1:(n_actual + 1));
    R.X_ctrl        = X_ctrl (:, 1:(n_actual + 1));
    R.U             = U(:,    1:n_actual);
    R.status_seq    = status_seq  (1:n_actual);
    R.sqp_iter_seq  = sqp_iter_seq(1:n_actual);
    R.qp_iter_seq   = qp_iter_seq (1:n_actual);
    R.timing = struct( ...
        'tot',    T_tot   (1:n_actual), ...
        'lin',    T_lin   (1:n_actual), ...
        'qp_sol', T_qp    (1:n_actual), ...
        'reg',    T_reg   (1:n_actual), ...
        'sim',    T_sim   (1:n_actual), ...
        'sim_ad', T_simad (1:n_actual), ...
        'sim_la', T_simla (1:n_actual), ...
        'glob',   T_glob  (1:n_actual), ...
        'wall',   T_wall  (1:n_actual)  ...
    );
    R.n_actual         = n_actual;
    R.diverged         = diverged;
    R.dt               = dt;
    R.disturbance_step = dist_step;
end

% ---------------------------------------------------------------------- %

function apply_initial_warmstart(ocp_solver, M, cfg, x_ctrl)
    switch cfg.warmstart_strategy
        case 'linear_to_eq'
            for k = 0:cfg.N
                a = k / cfg.N;
                ocp_solver.set('x', (1-a)*x_ctrl + a*M.xEq_ctrl, k);
            end
            for k = 0:(cfg.N-1)
                ocp_solver.set('u', M.uEq, k);
            end
        case {'cold','shift_trajectory','previous_run'}
            % no initial warmstart -- acados uses its internal defaults
        otherwise
            error('run_closed_loop:bad_warmstart','Unknown warmstart_strategy "%s"', cfg.warmstart_strategy);
    end
end

function apply_intra_warmstart(ocp_solver, M, cfg)
    switch cfg.warmstart_strategy
        case {'linear_to_eq','shift_trajectory'}
            % Shift previous trajectory forward by one node, append xEq/uEq.
            x_traj = ocp_solver.get('x');
            u_traj = ocp_solver.get('u');
            x_shift = [x_traj(:, 2:end), M.xEq_ctrl];
            u_shift = [u_traj(:, 2:end), M.uEq];
            for k = 0:cfg.N
                ocp_solver.set('x', x_shift(:, k+1), k);
            end
            for k = 0:(cfg.N-1)
                ocp_solver.set('u', u_shift(:, k+1), k);
            end
        case {'cold','previous_run'}
            % no intra-loop reset
    end
end

function v = try_get(solver, name)
    try
        v = solver.get(name);
    catch
        v = NaN;
    end
end
