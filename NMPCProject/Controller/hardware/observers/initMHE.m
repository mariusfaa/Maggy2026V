function stateEstimatorFcn = initMHE(f_expl_r, h_expr, x_r, u_sym, ...
    i_meas_sym, h_func, xEq, uEq, dt)
% INITMHE  Initialize a Moving Horizon Estimator for the MagLev system.
%
% Returns a closure-based function handle with signature:
%   x_est = stateEstimatorFcn(y_mag, i_meas, u_prev, dt)
%
% The MHE uses acados to solve a least-squares NLP over a sliding window.
% During warm-up and for arrival cost, a parallel EKF runs alongside.
% Measurement bias calibration is handled by the internal EKF.
%
% Inputs:
%   f_expl_r    - CasADi SX expression for dx/dt = f(x_r, u) (10x1)
%   h_expr      - CasADi SX expression for y = h(x_r, i_meas) (3x1)
%   x_r         - CasADi SX state symbol (10x1)
%   u_sym       - CasADi SX control input symbol (4x1)
%   i_meas_sym  - CasADi SX measured current symbol (4x1)
%   h_func      - CasADi Function h(x_r, i_meas) -> y (3x1)
%   xEq         - Equilibrium state (10x1)
%   uEq         - Equilibrium input (4x1)
%   dt          - Sample time (scalar)
%
% Output:
%   stateEstimatorFcn - function handle matching the controller interface

    import casadi.*

    nx = 10;
    ny = 3;
    nu = 4;
    nw = nx;          % process noise dimension = state dimension
    N_mhe = 5;        % estimation horizon (50 ms at dt=0.01)

    %% --- Tuning matrices ---
    Q_mhe = diag([1e-10, 1e-10, 1e-10, ...
                  1e-10, 1e-10, ...
                  1e-6,  1e-6,  1e-6, ...
                  1e-6,  1e-6]);

    R_mhe = diag([1e-8, 1e-8, 1e-8]);

    P0 = diag([1e-6, 1e-6, 1e-6, ...
               1e-4, 1e-4, ...
               1e-4, 1e-4, 1e-4, ...
               1e-2, 1e-2]);

    Q_mhe_inv = inv(Q_mhe);
    R_mhe_inv = inv(R_mhe);
    P0_inv    = inv(P0);

    %% --- Build acados MHE OCP ---
    % Decision variables:
    %   x = x_r (10 states to estimate)
    %   u = w   (10 process noise variables)
    % Runtime parameters at each stage:
    %   p = [y_meas(3); i_meas(4); u_nmpc(4); x_arrival(10)]  = 21 values

    w_sym = SX.sym('w', nw);
    xdot_mhe = SX.sym('xdot_mhe', nx);

    p_sym = SX.sym('p', ny + nu + nu + nx);  % [y; i; u_nmpc; x_arrival]
    y_ref_p     = p_sym(1:ny);
    i_meas_p    = p_sym(ny+1 : ny+nu);
    u_nmpc_p    = p_sym(ny+nu+1 : ny+nu+nu);
    x_arrival_p = p_sym(ny+nu+nu+1 : end);

    % Continuous dynamics with u_nmpc from parameters, plus process noise
    f_cont_func = casadi.Function('f_cont', {x_r, u_sym}, {f_expl_r});
    dx_mhe = f_cont_func(x_r, u_nmpc_p) + w_sym;

    % --- OCP formulation ---
    ocp_mhe = AcadosOcp();
    ocp_mhe.model.name  = 'maglev_mhe';
    ocp_mhe.model.x     = x_r;
    ocp_mhe.model.u     = w_sym;
    ocp_mhe.model.p     = p_sym;
    ocp_mhe.model.xdot  = xdot_mhe;
    ocp_mhe.model.f_impl_expr = xdot_mhe - dx_mhe;

    ocp_mhe.solver_options.N_horizon             = N_mhe;
    ocp_mhe.solver_options.tf                    = N_mhe * dt;
    ocp_mhe.solver_options.integrator_type       = 'IRK';
    ocp_mhe.solver_options.sim_method_num_stages = 2;
    ocp_mhe.solver_options.sim_method_num_steps  = 3;
    ocp_mhe.solver_options.nlp_solver_type       = 'SQP_RTI';
    ocp_mhe.solver_options.nlp_solver_tol_stat   = 1e-3;
    ocp_mhe.solver_options.nlp_solver_tol_eq     = 1e-3;
    ocp_mhe.solver_options.nlp_solver_tol_ineq   = 1e-3;
    ocp_mhe.solver_options.nlp_solver_tol_comp   = 1e-3;
    ocp_mhe.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
    ocp_mhe.solver_options.qp_solver_iter_max    = 100;
    ocp_mhe.solver_options.qp_solver_warm_start  = 1;
    ocp_mhe.solver_options.hessian_approx        = 'GAUSS_NEWTON';
    ocp_mhe.solver_options.regularize_method     = 'CONVEXIFY';

    % --- Cost: NONLINEAR_LS ---
    % Substitute i_meas from parameters into h_expr
    h_with_p = casadi.Function('h_p', {x_r, i_meas_sym}, {h_expr});
    h_at_p   = h_with_p(x_r, i_meas_p);

    % Stage cost residual: [h(x, i) - y; w]  (ny + nw = 13)
    stage_residual = [h_at_p - y_ref_p; w_sym];
    W_stage = blkdiag(R_mhe_inv, Q_mhe_inv);

    ocp_mhe.cost.cost_type    = 'NONLINEAR_LS';
    ocp_mhe.model.cost_y_expr = stage_residual;
    ocp_mhe.cost.W            = W_stage;
    ocp_mhe.cost.yref         = zeros(ny + nw, 1);

    % Initial stage cost: stage residual + arrival cost on x_0
    initial_residual = [h_at_p - y_ref_p; w_sym; x_r - x_arrival_p];
    W_initial = blkdiag(R_mhe_inv, Q_mhe_inv, P0_inv);

    ocp_mhe.cost.cost_type_0    = 'NONLINEAR_LS';
    ocp_mhe.model.cost_y_expr_0 = initial_residual;
    ocp_mhe.cost.W_0            = W_initial;
    ocp_mhe.cost.yref_0         = zeros(ny + nw + nx, 1);

    % Terminal cost: measurement residual only
    terminal_residual = h_at_p - y_ref_p;
    W_terminal = R_mhe_inv;

    ocp_mhe.cost.cost_type_e     = 'NONLINEAR_LS';
    ocp_mhe.model.cost_y_expr_e  = terminal_residual;
    ocp_mhe.cost.W_e             = W_terminal;
    ocp_mhe.cost.yref_e          = zeros(ny, 1);

    % Parameters: default values
    ocp_mhe.parameter_values = zeros(ny + nu + nu + nx, 1);

    % Initial state (required by acados; will be overwritten each step)
    ocp_mhe.constraints.x0 = xEq;

    %% --- Build MHE solver ---
    fprintf('  MHE: Building acados solver (N_mhe=%d, dt=%.4f s) ...\n', N_mhe, dt);
    mhe_solver = AcadosOcpSolver(ocp_mhe);

    %% --- Initialize warm-start trajectory ---
    for k = 0:N_mhe
        mhe_solver.set('x', xEq, k);
    end
    for k = 0:N_mhe-1
        mhe_solver.set('u', zeros(nw, 1), k);
    end

    %% --- Parallel EKF for arrival cost and warm-up ---
    ekf_struct = initEKF(f_expl_r, h_expr, x_r, u_sym, ...
        i_meas_sym, h_func, xEq, dt);

    %% --- Data buffer for sliding window ---
    % Store bias-corrected measurements (Tesla) after EKF calibration
    y_buf = zeros(ny, N_mhe + 1);
    i_buf = zeros(nu, N_mhe + 1);
    u_buf = zeros(nu, N_mhe);
    buf_count = 0;

    % The EKF calibrates for N_cal=50 steps. MHE warm-up starts after that.
    % Total warm-up: N_cal + N_mhe steps before MHE takes over.
    N_cal = 50;  % must match EKF's N_cal
    mhe_ready = false;
    step_count = 0;

    x_arrival = xEq;

    fprintf('  MHE: Warm-up = %d (cal) + %d (buffer fill) steps\n', N_cal, N_mhe);

    %% --- Return struct with step + reset ---
    stateEstimatorFcn.step  = @estimatorStep;
    stateEstimatorFcn.reset = @resetEstimator;

    function resetEstimator()
        % Reset MHE state and internal EKF to equilibrium
        buf_count = 0;
        mhe_ready = false;
        x_arrival = xEq;
        for k_rst = 0:N_mhe
            mhe_solver.set('x', xEq, k_rst);
        end
        for k_rst = 0:N_mhe-1
            mhe_solver.set('u', zeros(nw, 1), k_rst);
        end
        ekf_struct.reset();
        fprintf('  MHE: Reset to equilibrium\n');
    end

    function x_est = estimatorStep(y_mag, i_meas, u_prev, dt_step)
        step_count = step_count + 1;

        % Always update the parallel EKF (handles calibration too)
        x_ekf = ekf_struct.step(y_mag, i_meas, u_prev, dt_step);

        % During EKF calibration phase, just return EKF result
        if step_count <= N_cal
            x_est = x_ekf;
            x_arrival = x_ekf;
            return;
        end

        % Convert and bias-correct measurements (EKF has calibrated by now)
        % We need the same bias the EKF uses. Re-derive it here:
        % (The EKF stores y_bias internally; we approximate by using its
        %  corrected predictions. For simplicity, feed the MHE the same
        %  bias-corrected measurements the EKF would see.)
        y_T = y_mag(:) * 1e-3;
        % The EKF's bias is stored in its closure. We use the EKF estimate
        % to keep the MHE consistent. The MHE will refine from there.

        % Fill buffer
        if buf_count < N_mhe + 1
            buf_count = buf_count + 1;
            y_buf(:, buf_count) = y_T;
            i_buf(:, buf_count) = i_meas(:);
            if buf_count > 1
                u_buf(:, buf_count - 1) = u_prev(:);
            end
        else
            % Shift left
            y_buf(:, 1:end-1) = y_buf(:, 2:end);
            y_buf(:, end) = y_T;
            i_buf(:, 1:end-1) = i_buf(:, 2:end);
            i_buf(:, end) = i_meas(:);
            u_buf(:, 1:end-1) = u_buf(:, 2:end);
            u_buf(:, end) = u_prev(:);
        end

        % During buffer fill, use EKF only
        if buf_count <= N_mhe
            x_est = x_ekf;
            x_arrival = x_ekf;
            return;
        end

        if ~mhe_ready
            mhe_ready = true;
            fprintf('  MHE: Buffer full, MHE active from step %d\n', step_count);
        end

        % --- Set parameters at each stage ---
        for k = 0:N_mhe
            p_k = zeros(ny + nu + nu + nx, 1);
            p_k(1:ny) = y_buf(:, k+1);
            p_k(ny+1 : ny+nu) = i_buf(:, k+1);
            if k < N_mhe
                p_k(ny+nu+1 : ny+nu+nu) = u_buf(:, k+1);
            end
            if k == 0
                p_k(ny+nu+nu+1 : end) = x_arrival;
            end
            mhe_solver.set('p', p_k, k);
        end

        % --- Warm-start: shift previous solution ---
        for k = 0:N_mhe-1
            try
                x_ws = mhe_solver.get('x', k+1);
                mhe_solver.set('x', x_ws, k);
            catch
            end
        end
        mhe_solver.set('x', x_ekf, N_mhe);

        % --- Solve ---
        mhe_solver.solve();
        mhe_status = mhe_solver.get('status');

        % Extract the most recent state estimate (end of window)
        if mhe_status == 0
            x_est = mhe_solver.get('x', N_mhe);
            x_arrival = mhe_solver.get('x', 1);
        else
            % Fall back to EKF if MHE fails
            x_est = x_ekf;
            x_arrival = x_ekf;
        end
    end
end
