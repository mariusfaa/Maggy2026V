
function [x_est, P_est] = Correction(x_pred, P_pred, meas, input, filterType, obs) %#codegen
%CORRECTION  Measurement update for KF / EKF / UKF.
%
%  Inputs (all filters)
%    x_pred     - predicted state                 (n x 1)
%    P_pred     - predicted error covariance      (n x n)
%    y          - raw measurement vector          (p x 1)
%    Cd         - discrete output (observation) matrix  (p x n)
%                 * KF  : fixed linearisation matrix
%                 * EKF : Jacobian dh/dx evaluated at x_pred
%                         (compute with finiteDifferenceLinearization)
%                 * UKF : not used (pass [] if desired)
%    Rd         - measurement noise covariance    (p x p)
%    filter_type - 'KF' | 'EKF' | 'UKF'          (string)
%
%  Additional inputs for EKF / UKF
%    h          - nonlinear measurement handle  h(x) -> y_hat   (p x 1)
%                 NOTE: h must return measurements in the same
%                 scaled units applied to y below.
%
%  Additional inputs for UKF only
%    ukf_params - same struct as in Prediction

    % --- Measurement scaling (applied once, consistently) ---
    assert(length(meas) >= 3);
    meas(1) =  1e-3*meas(1);
    meas(2) =  1e-3*meas(2);
    meas(3) = -1e-3*meas(3);

    nx = obs.nx;
    assert(nx <= 10);
    nz = obs.nz;
    assert(nz <= 3);

    x_est = zeros(nx, 1);
    P_est = zeros(nx, nx);

    In = eye(nx);

    switch filterType
    % ------------------------------------------------------------------
        case 1
        S     = obs.Cd*P_pred*obs.Cd' + obs.R;
        L     = (P_pred*obs.Cd') / S;                   % Kalman gain
        x_est = x_pred + L*(meas - obs.Cd*x_pred - obs.Dd*input);
        P_est = (In - L*obs.Cd)*P_pred*(In - L*obs.Cd)' + L*obs.R*L';

    % ------------------------------------------------------------------
        case 2
        z_pred = maglevSystemMeasurements_xred(x_pred, input);                         % nonlinear measurement prediction
        Cd = finiteDifferenceC(obs.xLp, obs.uLp, obs.delta, nx, nz);
        S      = Cd*P_pred*Cd' + obs.R;
        L      = (P_pred*Cd') / S;                  % Kalman gain (uses Jacobian Cd)
        x_est  = x_pred + L*(meas - z_pred);
        P_est  = (In - L*Cd)*P_pred*(In - L*Cd)' + L*obs.R*L';

    % ------------------------------------------------------------------
        case 3

        % --- Sigma points centred on predicted state ---
        S_chol = chol((nx + obs.lambda)*P_pred, 'lower');
        sigma  = [x_pred,  x_pred + S_chol,  x_pred - S_chol];

        % --- Propagate sigma points through nonlinear h ---
        gamma = zeros(nz, 2*nx+1);
        for i = 1:2*nx+1
            gamma(:,i) = maglevSystemMeasurements_xred(sigma(:,i), input);
        end

        % --- Predicted measurement mean ---
        z_pred = gamma * obs.Wm;

        % --- Innovation covariance Pyy and cross-covariance Pxy ---
        Pyy = obs.R;
        Pxy = zeros(nx, nz);
        for i = 1:2*nx+1
            dy  = gamma(:,i)  - z_pred;
            dx  = sigma(:,i)  - x_pred;
            Pyy = Pyy + obs.Wc(i)*(dy*dy');
            Pxy = Pxy + obs.Wc(i)*(dx*dy');
        end

        L     = Pxy / Pyy;                          % Kalman gain
        x_est = x_pred + L*(meas - z_pred);
        P_est = P_pred - L*Pyy*L';

        otherwise
        error('Filter_type must be KF, EKF, or UKF.');
    end
end



function C = finiteDifferenceC(xLp,uLp,delta,nx,nz)
    
    C = zeros(nz,nx);
    
    for i = 1:nx
        e = zeros(nx, 1);
        e(i) = delta;
        C(:,i) = (maglevSystemMeasurements_xred(xLp + e, uLp) - maglevSystemMeasurements_xred(xLp - e, uLp)) / (2*delta);
    end
end
