
% Constants

obs.zEq = 0.030119178665731;
obs.xLp = [0,0,obs.zEq(1),zeros(1,3)]';
obs.uLp = zeros(4,1);


obs.dt = 0.01;
obs.nx = 6;
obs.nz = 3;

% Linearization
obs.delta = 1e-6; % Step-size used in numerical linearization
[obs.A,obs.B,obs.C,obs.D] = finiteDifferenceLinearization(@maglevSystemDynamics_xred,@maglevSystemMeasurements_xred,obs.xLp,obs.uLp,obs.delta);


% Discretizing system
sysd = c2d(ss(obs.A,obs.B,obs.C,obs.D), obs.dt, 'zoh');
obs.Ad = sysd.A;
obs.Bd = sysd.B;
obs.Cd = sysd.C;
obs.Dd = sysd.D;


% Noise

obs.P0 = eye(obs.nx,obs.nx);
obs.R = 1e-8*eye(obs.nz,obs.nz);

obs.NSD = [1e-3 0 0
           0 1e-3 0
           0 0 1e-3];

obs.G = [zeros(obs.nx/2,obs.nx/2);
         eye(obs.nx/2,obs.nx/2)]; % Noise only enters lower states (derivatives)

obs.Q = obs.G*obs.NSD*obs.G';

obs.Qd = vanLoan(obs.A, obs.Q, obs.dt, obs.nx);





% UKF

obs.ukf_params.alpha = 1e-3;
obs.ukf_params.beta = 2;
obs.ukf_params.kappa = 0;

[obs.Wm, obs.Wc, obs.lambda] = ukf_weights(obs.nx, obs.ukf_params);


% Functions

function [Wm, Wc, lambda] = ukf_weights(n, ukf_params)
%UKF_WEIGHTS  Compute mean and covariance weights for the Unscented Transform.
    alpha  = ukf_params.alpha;   % spread of sigma points (e.g. 1e-3)
    beta   = ukf_params.beta;    % prior distribution info (2 optimal for Gaussian)
    kappa  = ukf_params.kappa;   % secondary scaling (usually 0 or 3-n)

    lambda = alpha^2*(n + kappa) - n;

    % Mean weights  (2n+1 column vector)
    Wm    = repmat(1/(2*(n + lambda)), 2*n+1, 1);
    Wm(1) = lambda/(n + lambda);

    % Covariance weights
    Wc    = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);
end

function Qd = vanLoan(A, Qc, dt, n)
    M     = [-A, Qc; zeros(n,n), A'];
    Phi   = expm(M * dt);
    Phi12 = Phi(1:n,     n+1:end);
    Phi22 = Phi(n+1:end, n+1:end);
    Qd    = (Phi22'*Phi12 + (Phi22'*Phi12)') / 2; % averaging for symmetry
end