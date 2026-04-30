function stateEstimatorFcn = initEKF(f_expl_r, h_expr, x_r, u_sym, ...
    i_meas_sym, h_func, xEq, dt)
% INITEKF  Initialize an Extended Kalman Filter for the MagLev system.
%
% Returns a struct with fields:
%   .step   - function handle: x_est = step(y_mag, i_meas, u_prev, dt)
%   .reset  - function handle: reset()  — resets EKF state to equilibrium
%
% Only position and orientation (states 1-5) are estimated from measurements.
% Velocity and angular rate states (6-10) are always held at zero because
% they are unobservable from 3 magnetic field measurements and cause
% divergence when allowed to accumulate through the prediction step.
%
% Inputs:
%   f_expl_r    - CasADi SX, reduced-order dynamics (10x1)
%   h_expr      - CasADi SX, measurement model (3x1)
%   x_r         - CasADi SX state symbol (10x1)
%   u_sym       - CasADi SX control input symbol (4x1)
%   i_meas_sym  - CasADi SX measured currents symbol (4x1)
%   h_func      - CasADi Function h(x_r, i_meas) -> y (3x1)
%   xEq         - Equilibrium state (10x1 double)
%   dt          - Sample time (scalar double)
%
% Output:
%   stateEstimatorFcn - struct with .step and .reset

    import casadi.*

    nx = 10;
    ny = 3;
    nu = 4;

    % Indices for the 5 observable states (position + orientation)
    ix_pos = 1:5;   % [x, y, z, alpha, beta]
    ix_vel = 6:10;  % [vx, vy, vz, wx, wy] — NOT estimated
    np = length(ix_pos);  % 5

    %% --- Measurement Jacobian via finite differences (5 states only) ---
    delta_fd = 1e-7;

    function H = computeH(x_lin, i_lin)
        H = zeros(ny, np);
        y0 = full(h_func(x_lin, i_lin));
        for j = 1:np
            x_pert = x_lin;
            x_pert(ix_pos(j)) = x_pert(ix_pos(j)) + delta_fd;
            yp = full(h_func(x_pert, i_lin));
            H(:, j) = (yp - y0) / delta_fd;
        end
    end

    %% --- Tuning matrices (5x5 and 3x3) ---
    % Process noise on position/orientation only
    Q_ekf = diag([1e-10, 1e-10, 1e-10, ...   % x, y, z
                  1e-8,  1e-8]);              % alpha, beta

    % Measurement noise: conservative for magnetic model mismatch
    R_ekf = diag([1e-6, 1e-6, 1e-6]);

    % Initial covariance (5x5)
    P0 = diag([1e-10, 1e-10, 1e-10, ...       % position
               1e-8,  1e-8]);                  % orientation

    %% --- Measurement offset calibration ---
    N_cal       = 50;
    cal_count   = 0;
    y_cal_sum   = zeros(ny, 1);
    y_bias      = zeros(ny, 1);
    y_model_eq  = full(h_func(xEq, zeros(nu, 1)));
    calibrated  = false;

    fprintf('  EKF: Model prediction at equilibrium: [%.4f, %.4f, %.4f] mT\n', ...
        y_model_eq(1)*1e3, y_model_eq(2)*1e3, y_model_eq(3)*1e3);
    fprintf('  EKF: Calibrating bias over first %d steps (%.1f s)\n', N_cal, N_cal*dt);

    %% --- Trust ramp-up ---
    N_ramp      = 100;
    ramp_count  = 0;

    %% --- Innovation gating ---
    gate_threshold = 16.3;  % chi-squared, 3 DOF, 99.9%

    %% --- Maximum correction per step (5 states) ---
    max_dx = [0.5e-3; 0.5e-3; 0.5e-3; ...    % position: 0.5 mm
              0.01;   0.01];                    % orientation: 0.01 rad

    %% --- Initialize persistent state ---
    x_hat = xEq;       % full 10-state (velocities always zero)
    P     = P0;         % 5x5 covariance
    I_np  = eye(np);

    fprintf('  EKF: Position-only estimator (5 of 10 states)\n');
    fprintf('  EKF: Velocities forced to zero every step\n');
    fprintf('  EKF: nx_est=%d, ny=%d, dt=%.4f s\n', np, ny, dt);
    fprintf('  EKF: R = %.0e, Q_pos = %.0e, Q_ang = %.0e\n', R_ekf(1,1), Q_ekf(1,1), Q_ekf(4,4));
    fprintf('  EKF: Trust ramp-up over %d steps (%.1f s)\n', N_ramp, N_ramp*dt);

    %% --- Return struct with step + reset ---
    stateEstimatorFcn.step  = @estimatorStep;
    stateEstimatorFcn.reset = @resetEstimator;

    function resetEstimator()
        % Reset EKF state to equilibrium (call after QP failure)
        x_hat = xEq;
        P     = P0;
        ramp_count = 0;
        fprintf('  EKF: Reset to equilibrium\n');
    end

    function x_est = estimatorStep(y_mag, i_meas, u_prev, ~)
        % Convert measurements from mT to Tesla
        y_T = y_mag(:) * 1e-3;

        % --- Calibration phase ---
        if ~calibrated
            cal_count = cal_count + 1;
            y_cal_sum = y_cal_sum + y_T;
            if cal_count >= N_cal
                y_bias = (y_cal_sum / N_cal) - y_model_eq;
                calibrated = true;
                fprintf('  EKF: Calibrated bias = [%.4f, %.4f, %.4f] mT\n', ...
                    y_bias(1)*1e3, y_bias(2)*1e3, y_bias(3)*1e3);
                fprintf('  EKF: Active estimation starting (ramp over %d steps)\n', N_ramp);
            end
            x_est = xEq;
            return;
        end

        % Bias correction
        y_corrected = y_T - y_bias;

        % --- Predict (position-only, no dynamics propagation) ---
        % Since velocities are zero, the prediction is simply:
        %   position stays the same (x_next ≈ x + v*dt = x + 0)
        % This avoids the unstable dynamics propagation that caused divergence.
        x_pred = x_hat;  % velocities are already zero, so x_pred = x_hat
        P_pred = P + Q_ekf;
        P_pred = (P_pred + P_pred') / 2;

        % --- Measurement update (5 observable states only) ---
        y_pred = full(h_func(x_pred, i_meas));
        H_k    = computeH(x_pred, i_meas);
        innov  = y_corrected - y_pred;

        S = H_k * P_pred * H_k' + R_ekf;
        S = (S + S') / 2;
        S = S + 1e-12 * eye(ny);  % regularize

        % Innovation gating
        try
            md2 = innov' * (S \ innov);
        catch
            P = P_pred;
            x_est = x_hat;
            return;
        end

        if md2 > gate_threshold || isnan(md2) || isinf(md2)
            P = P_pred;
            x_est = x_hat;
            return;
        end

        % Kalman gain (5x3)
        K = P_pred * H_k' / S;

        if any(isnan(K(:))) || any(isinf(K(:)))
            P = P_pred;
            x_est = x_hat;
            return;
        end

        % Trust ramp
        ramp_count = ramp_count + 1;
        gain_scale = min(ramp_count / N_ramp, 1.0);

        % Compute correction for 5 position/orientation states
        dx = gain_scale * (K * innov);
        dx = max(min(dx, max_dx), -max_dx);

        % Update position/orientation states only
        x_hat(ix_pos) = x_pred(ix_pos) + dx;
        % Velocities stay at zero
        x_hat(ix_vel) = 0;

        % Joseph form covariance update (5x5)
        if norm(innov) > 1e-15
            K_eff = dx * innov' / (innov' * innov);
        else
            K_eff = zeros(np, ny);
        end
        IKH = I_np - K_eff * H_k;
        P   = IKH * P_pred * IKH' + K_eff * R_ekf * K_eff';
        P   = (P + P') / 2;
        P   = P + 1e-15 * I_np;

        x_est = x_hat;
    end
end
