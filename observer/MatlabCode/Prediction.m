
function [x_pred, P_pred] = Prediction(x_est, P_est, input, filterType, obs) %#codegen
%PREDICTION  One-step ahead prediction for KF / EKF / UKF.
%
%  Inputs (all filters)
%    x_hat      - current state estimate          (n x 1)
%    P_hat      - current error covariance        (n x n)
%    u          - control input                   (m x 1)
%    Ad, Bd     - discrete state / input matrices (n x n, n x m)
%                 * KF  : fixed linearisation matrices
%                 * EKF : Jacobian evaluated at current x_hat, u
%                         (compute with finiteDifferenceLinearization
%                          before calling this function)
%                 * UKF : not used (pass [] if desired)
%    Qd         - process noise covariance        (n x n)
%    filter_type - 'KF' | 'EKF' | 'UKF'          (string)
%
%  Additional inputs for EKF / UKF
%    f          - nonlinear dynamics handle  f(x, u) -> x_next   (n x 1)
%
%  Additional inputs for UKF only
%    ukf_params - struct with fields:
%                   .alpha  (spread, e.g. 1e-3)
%                   .beta   (prior knowledge, e.g. 2 for Gaussian)
%                   .kappa  (secondary scaling, e.g. 0)

    nx = obs.nx;
    assert(nx <= 10);

    x_pred = zeros(nx, 1);
    P_pred = zeros(nx, nx);

    switch filterType
    % ------------------------------------------------------------------
        case 1
        x_pred = obs.Ad*x_est + obs.Bd*input;
        P_pred = obs.Ad*P_est*obs.Ad' + obs.Qd;

    % ------------------------------------------------------------------
        case 2
        % State prediction through nonlinear model
        x_pred = stateTrans(x_est, input, obs.dt);
        % Covariance prediction through linearised (Jacobian) model
        Ad = finiteDifferenceAd(x_est,input,obs.delta,nx,obs.dt);
        P_pred = Ad*P_est*Ad' + obs.Qd;
        

    % ------------------------------------------------------------------
        case 3
        % --- Generate sigma points ---
        S      = chol((nx + obs.lambda)*P_est, 'lower');   % Cholesky factor
        cols   = [x_est,  x_est + S,  x_est - S];    % (n x 2n+1)

        % --- Propagate sigma points through nonlinear f ---
        sigma_pred = zeros(nx, 2*nx+1);
        for i = 1:2*nx+1
            sigma_pred(:,i) = stateTrans(cols(:,i), input, obs.dt);
        end

        % --- Weighted mean ---
        x_pred = sigma_pred * obs.Wm;

        % --- Weighted covariance ---
        P_pred = obs.Qd;
        for i = 1:2*nx+1
            d      = sigma_pred(:,i) - x_pred;
            P_pred = P_pred + obs.Wc(i)*(d*d');
        end

        otherwise
        error('Filter_type must be KF, EKF, or UKF.');
    end
end


function xn = stateTrans(x,u,dt)
    xn = x + dt*maglevSystemDynamics_xred(x,u);
end


function A = finiteDifferenceAd(xLp,uLp,delta,nx,dt)
    
    A = zeros(nx,nx);
    
    for i = 1:nx
        e = zeros(nx, 1);
        e(i) = delta;
        A(:,i) = (stateTrans(xLp + e, uLp, dt) - stateTrans(xLp - e, uLp, dt)) / (2*delta);
    end
end
