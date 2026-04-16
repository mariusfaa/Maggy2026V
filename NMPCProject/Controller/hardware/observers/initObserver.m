function stateEstimatorFcn = initObserver(observerType, paramsFast, ...
    f_expl_r, x_r, u_sym, xEq, uEq, dt)
% INITOBSERVER  Factory function for MagLev state observers.
%
% Builds the CasADi measurement model, then initializes either an EKF or
% MHE observer and returns a closure-based function handle.
%
% Inputs:
%   observerType - 'EKF' or 'MHE'
%   paramsFast   - System parameters (with corrected solenoid radius)
%   f_expl_r     - CasADi SX expression for reduced-order dynamics (10x1)
%   x_r          - CasADi SX state symbol (10x1)
%   u_sym        - CasADi SX control input symbol (4x1)
%   xEq          - Equilibrium state (10x1 double)
%   uEq          - Equilibrium input (4x1 double)
%   dt           - Sample time (scalar double)
%
% Output:
%   stateEstimatorFcn - function handle:
%       x_est = stateEstimatorFcn(y_mag, i_meas, u_prev, dt)

    import casadi.*

    nu = 4;

    fprintf('\n--- Initializing %s observer ---\n', observerType);

    %% --- Build CasADi measurement model ---
    i_meas_sym = SX.sym('i_meas', nu);

    [h_expr, h_func] = computeMeasurementModelCasADi(x_r, i_meas_sym, paramsFast);

    fprintf('  Measurement model built (3 outputs at sensor [%.4f, %.4f, %.4f] m)\n', ...
        paramsFast.sensors.x(1), paramsFast.sensors.y(1), paramsFast.sensors.z(1));

    %% --- Validate measurement model at equilibrium ---
    y_casadi = full(h_func(xEq, zeros(nu, 1)));
    fprintf('  h(xEq, 0) = [%.4f, %.4f, %.4f] mT  (model prediction at equilibrium)\n', ...
        y_casadi(1)*1e3, y_casadi(2)*1e3, y_casadi(3)*1e3);
    fprintf('  Compare with your sensor readings at rest to check model accuracy.\n');
    fprintf('  The EKF will auto-calibrate the bias over the first 20 steps.\n');

    %% --- Dispatch to observer ---
    % Both observers return a struct with .step and .reset fields.
    switch upper(observerType)
        case 'EKF'
            stateEstimatorFcn = initEKF(f_expl_r, h_expr, x_r, u_sym, ...
                i_meas_sym, h_func, xEq, dt);

        case 'MHE'
            stateEstimatorFcn = initMHE(f_expl_r, h_expr, x_r, u_sym, ...
                i_meas_sym, h_func, xEq, uEq, dt);

        otherwise
            error('Unknown observer type: %s. Use ''EKF'' or ''MHE''.', observerType);
    end

    fprintf('--- %s observer ready ---\n\n', observerType);
end
