function R = run_sim(ocp_solver, plant_solver, x0_plant, M, cfg)
% RUN_SIM  Closed-loop simulation. Plant is 12-state; controller is
% 10- or 12-state via M.project_to_ctrl. Captures wall-clock and acados
% timing per step.
%
%   R = run_sim(ocp_solver, plant_solver, x0_plant, M, cfg)
%
% Inputs
%   ocp_solver   AcadosOcpSolver from build_ocp
%   plant_solver AcadosSimSolver from build_plant (always 12-state)
%   x0_plant     12-dim initial plant state
%   M            struct from build_model
%   cfg          fields used here: .N, .Tf, .sim_steps
%
% Output struct R
%   .X_plant   12 x (sim_steps+1)
%   .U          nu x sim_steps
%   .X_ctrl    nx_ctrl x (sim_steps+1)   (projection of X_plant)
%   .status,.sqp_iter,.qp_iter   1 x sim_steps
%   .timing.tot,.lin,.qp_sol,.reg,.sim,.sim_ad,.sim_la,.glob (1xsim_steps)
%   .timing.wall  (1 x sim_steps) tic/toc around set+solve+get
%   .n_actual   number of completed steps (= sim_steps unless diverged)
%   .diverged   logical
%   .dt         Tf/N

    N         = cfg.N;
    Tf        = cfg.Tf;
    nu        = M.nu;
    nx_ctrl   = M.nx_ctrl;
    sim_steps = cfg.sim_steps;
    dt        = Tf / N;

    X_plant = zeros(12, sim_steps+1);
    X_ctrl  = zeros(nx_ctrl, sim_steps+1);
    U       = zeros(nu, sim_steps);
    status  = zeros(1, sim_steps);
    sqp_it  = zeros(1, sim_steps);
    qp_it   = zeros(1, sim_steps);
    t_tot   = zeros(1, sim_steps);
    t_lin   = zeros(1, sim_steps);
    t_qp    = zeros(1, sim_steps);
    t_reg   = zeros(1, sim_steps);
    t_sim   = nan(1, sim_steps);
    t_sad   = nan(1, sim_steps);
    t_sla   = nan(1, sim_steps);
    t_glob  = nan(1, sim_steps);
    t_wall  = zeros(1, sim_steps);

    x_plant = x0_plant(:);
    X_plant(:,1) = x_plant;
    x_ctrl       = M.project_to_ctrl(x_plant);
    X_ctrl(:,1)  = x_ctrl;

    % Warm-start trajectory: linear interp from current ctrl state -> equilibrium.
    for k = 0:N
        a = k / N;
        ocp_solver.set('x', (1 - a) * x_ctrl + a * M.xEq_ctrl, k);
    end
    for k = 0:N-1
        ocp_solver.set('u', M.uEq, k);
    end

    n_actual = sim_steps;
    diverged = false;

    for i = 1:sim_steps
        % --- Timed: state-in to control-out (the hardware-deadline metric) ---
        t_step = tic;
        ocp_solver.set('constr_x0', x_ctrl);
        ocp_solver.solve();
        u_applied = ocp_solver.get('u', 0);
        t_wall(i) = toc(t_step);

        u_applied = max(min(u_applied, 1), -1);

        t_tot(i)    = ocp_solver.get('time_tot');
        t_lin(i)    = ocp_solver.get('time_lin');
        t_qp(i)     = ocp_solver.get('time_qp_sol');
        t_reg(i)    = ocp_solver.get('time_reg');
        try, t_sim(i)  = ocp_solver.get('time_sim');    catch, end
        try, t_sad(i)  = ocp_solver.get('time_sim_ad'); catch, end
        try, t_sla(i)  = ocp_solver.get('time_sim_la'); catch, end
        try, t_glob(i) = ocp_solver.get('time_glob');   catch, end

        status(i)  = ocp_solver.get('status');
        sqp_it(i)  = ocp_solver.get('sqp_iter');
        try
            qr = ocp_solver.get('qp_iter');
            qp_it(i) = sum(qr(:));
        catch
            qp_it(i) = NaN;
        end

        % --- Plant step (NOT timed) ---
        plant_solver.set('x', x_plant);
        plant_solver.set('u', u_applied);
        plant_solver.solve();
        x_plant_next = plant_solver.get('xn');

        % --- Warm-start shift (NOT timed) ---
        x_traj = ocp_solver.get('x');
        u_traj = ocp_solver.get('u');
        for k = 0:N
            if k < N
                ocp_solver.set('x', x_traj(:, k+2), k);
            else
                ocp_solver.set('x', M.xEq_ctrl, k);
            end
        end
        for k = 0:N-1
            if k < N-1
                ocp_solver.set('u', u_traj(:, k+2), k);
            else
                ocp_solver.set('u', M.uEq, k);
            end
        end

        U(:,i) = u_applied;

        % --- Divergence test on plant ---
        if abs(x_plant_next(3)) > 0.5 || ...
                max(abs(x_plant_next(4:5))) > pi || ...
                any(isnan(x_plant_next)) || any(isinf(x_plant_next))
            diverged = true;
            n_actual = i;
            X_plant(:, i+1) = x_plant_next;
            X_ctrl(:,  i+1) = M.project_to_ctrl(x_plant_next);
            break;
        end

        x_plant = x_plant_next;
        x_ctrl  = M.project_to_ctrl(x_plant);
        X_plant(:, i+1) = x_plant;
        X_ctrl(:,  i+1) = x_ctrl;
    end

    R.X_plant  = X_plant;
    R.X_ctrl   = X_ctrl;
    R.U        = U;
    R.status   = status;
    R.sqp_iter = sqp_it;
    R.qp_iter  = qp_it;
    R.timing.tot    = t_tot;
    R.timing.lin    = t_lin;
    R.timing.qp_sol = t_qp;
    R.timing.reg    = t_reg;
    R.timing.sim    = t_sim;
    R.timing.sim_ad = t_sad;
    R.timing.sim_la = t_sla;
    R.timing.glob   = t_glob;
    R.timing.wall   = t_wall;
    R.n_actual = n_actual;
    R.diverged = diverged;
    R.dt       = dt;
end
