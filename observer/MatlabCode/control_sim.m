
clear; clc;
addpath(genpath('~/MSc/Maggy2026V/observer/MatlabCode'));
parameters_maggy_V4;

% correction factors
correctionFactorFast = 0.564394804131228; %computeSolenoidRadiusCorrectionFactor(params,'fast');
correctionFactorAccurate = 0.548863066449565; %computeSolenoidRadiusCorrectionFactor(params,'accurate');

fprintf('Fast correction factor %.2f\n', correctionFactorFast)
fprintf('Accurate correction factor %.2f\n', correctionFactorAccurate)

paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

paramsAccurate = params;
paramsAccurate.solenoids.r = correctionFactorAccurate*paramsAccurate.solenoids.r;

% equilibria
zEq = 0.030119178665731; %[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');
zEqAq = 0.030627801083773; %[zEqAq, ~, ~, ~] = computeSystemEquilibria(paramsAccurate,'accurate');
zEqFil = 0.030627801083773; %[zEqFil, ~, ~, ~] = computeSystemEquilibria(params,'filament');

% functions
f = @(x,u) maglevSystemDynamics_fast(x,u);
h = @(x,u) maglevSystemMeasurements_fast(x,u);

% Define the point to linearize around
xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(4,1);

% Linearization
delta = 1e-12; % Step-size used in numerical linearization
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);

I = [1:5,7:11];
Ixred = [1:3,7:9];
Ared = A(I,I); Axred = A(Ixred,Ixred);
Bred = B(I,:); Bxred = B(Ixred,:);
Cred = C(:,I); Cxred = C(:,Ixred);
Dred = D(:,:); Dxred = D(:,:);

% discretizing
dt = 0.005; % 200 Hz
sysd = c2d(ss(A,B,C,D), dt, 'zoh');
sysd_red = c2d(ss(Ared,Bred,Cred,Dred), dt, 'zoh');
sysd_xred = c2d(ss(Axred,Bxred,Cxred,Dxred), dt, 'zoh');
Ad = sysd_xred.A;
Bd = sysd_xred.B;
Cd = sysd_xred.C;
Dd = sysd_xred.D;


% control
% Cost matrices
Qlqr = diag([1e6,1e6,1e4, 1e1,1e1, 1e2,1e2,1e2, 1e2,1e2]);
Rlqr = 1e-0*eye(length(params.solenoids.r));

% Feedback gain
K = dlqr(sysd_red.A,sysd_red.B,Qlqr,Rlqr);

%% Observer
clear obs;

% arguments
filterVariant = 0;
nx = 6;
useSRformulation = false;
RK4Iterations = 0;
updateJacobians = true;
updateQ = false;
cubature = false;

% select the 6 states of interest from 10 state vector
I10to6 = [1:3, 6:8];

% initial covariance
P0 = 1e-6*eye(nx, nx);

% measurement noise
R = 1e-8*eye(3,3);

nxh = nx/2; % nx half
% Linear integrating part of system. Only used to discretize Q
Aint = [zeros(nxh,nxh) eye(nxh,nxh); zeros(nxh,nxh*2)];

% process noise
if nx == 10
    NSD = diag([1 1 1 1 1]);
elseif nx == 6
    NSD = diag([1 1 1]);
end

% how noise enters state space
G = [zeros(nxh,nxh);
     eye(nxh,nxh)];

% continuous time "covariance"
Q = G*NSD*G';

Qd = van_loan(Aint, Q, dt);

% creating observer
obs = Observer(filterVariant, dt, nx, xLp(1:nx), R, Qd, P0, ...
    'useSRformulation', useSRformulation, ...
    'RK4Iterations',    RK4Iterations,    ...
    'updateJacobians',  updateJacobians,  ...
    'updateQ',          updateQ,          ...
    'cubature',         cubature);

%% Simulation
nx_sim = 10; % number of states used by the simulator
nu = 4;      % number of inputs
nz = 3;      % number of measurements

% Initial condition
x0 = xLp(1:nx_sim) + [3e-4 -3e-4 0.001 1e-3 1e-3 zeros(1,5)].';

% Preallocate arrays
tSpan = 0:dt:1;
N = length(tSpan);

x = zeros(nx_sim, N);
x(:,1) = x0;

x_est = zeros(nx, N);
x_est(:,1) = xLp(1:nx);

u = zeros(nu, N);
u(:,1) = -K * (x0 - xLp(1:nx_sim)) - uLp;

z = zeros(nz, N);
z(:,1) = maglevSystemMeasurements_red(x0, u(:,1), 0);

error = zeros(nx, N);
if nx == 6
    error(:,1) = x_est(:,1) - x0(I10to6);
else
    error(:,1) = x_est(:,1) - x0;
end

meas_pred = zeros(nz, N);
nis = zeros(1, N);
nees = zeros(1, N);

std_state = zeros(nx, N);
std_meas = zeros(nz, N);

cond_P = zeros(1, N);
det_P = zeros(1, N);
cond_S = zeros(1, N);
det_S = zeros(1, N);

runtime = zeros(1, N);

% Simulation noise and disturbances
%
%generate_measurement_noise(zeros(nz,1), R, N);
%generate_process_noise(dt, N);
%
meas_noise = load('measurement_noise.mat').noise;
proc_noise_mat = load('process_noise.mat').noise;
proc_noise = [zeros(5,N); proc_noise_mat.acc; proc_noise_mat.rot];

% Simulation loop - start from k=2 to skip time=0 and simulate from previous to current
for k = 2:N
    % Current time
    t_current = tSpan(k);
    t_previous = tSpan(k-1);
    
    % Simulate continuous plant from previous timestep to current timestep
    [~, x_cont] = ode15s(@(t,xf) maglevSystemDynamics_red(xf, u(:,k-1), 0, proc_noise(:,k)), [t_previous t_current], x(:,k-1));
    x(:,k) = x_cont(end,:).';
    
    % Get measurement at current time
    z_ = maglevSystemMeasurements_red(x(:,k), u(:,k-1), 0);

    % converting from Telsa to milliTesla, inverting bz and adding noise to simulate real sensor
    z_ = (z_.*[1 1 -1]' + meas_noise(:,k))*1e+3;

    z(:,k) = z_;
    
    % Estimation
    x_est(:,k) = obs.run(u(:,k-1), z(:,k));
    
    % Compute control input using true state
    u(:,k) = -K * (x(:,k) - xLp(1:nx_sim)) - uLp;

    % Save other data
    meas_pred(:,k) = obs.getMeasPred.*[1 1 -1]'*1e+3;

    S = obs.getInnovationCovariance;
    innovation = obs.getInnovation;
    nis(k) = innovation'*(S\innovation);

    P = obs.getCovariance;
    if nx == 6
        error_ = x_est(:,k) - x(I10to6,k);
    else
        error_ = x_est(:,k) - x(:,k);
    end
    error(:,k) = error_;
    nees(k) = error_'*(P\error_);

    std_state(:,k) = sqrt(diag(P));
    std_meas(:,k) = sqrt(diag(S));

    cond_P(k) = cond(P);
    det_P(k)  = det(P);
    cond_S(k) = cond(S);
    det_S(k)  = det(S);

    runtime(k) = obs.lastRuntimeUs();
end

t = tSpan;

%% save data
header = {'t', ...
    'x','y','z','alpha','beta','x_dot','y_dot','z_dot','alpha_dot','beta_dot', ...
    'x_est','y_est','z_est','alpha_est','beta_est','x_dot_est','y_dot_est','z_dot_est','alpha_dot_est','beta_dot_est', ...
    'err_x','err_y','err_z','err_alpha','err_beta','err_x_dot','err_y_dot','err_z_dot','err_alpha_dot','err_beta_dot', ...
    'x_std','y_std','z_std','alpha_std','beta_std','x_dot_std','y_dot_std','z_dot_std','alpha_dot_std','beta_dot_std', ...
    'bx','by','bz', ...
    'bx_est','by_est','bz_est', ...
    'bx_std','by_std','bz_std', ...
    'nis','nees','cond_P','det_P','cond_S','det_S','runtime', ...
    'filterVariant','nx','useSRformulation','RK4Iterations','updateJacobians','updateQ','cubature'};

if nx == 6
    % remove state ests
    header(1+10+4) = [];
    header(1+10+5-1) = [];
    header(1+10+9-2) = [];
    header(1+10+10-3) = [];

    % remove errors
    header(1+20+4-4) = [];
    header(1+20+5-5) = [];
    header(1+20+9-6) = [];
    header(1+20+10-7) = [];

    % remove std
    header(1+30+4-8) = [];
    header(1+30+5-9) = [];
    header(1+30+9-10) = [];
    header(1+30+10-11) = [];
end

dataMatrix = [tSpan', ...
    x(1:nx_sim,:)'*1e+3, ...
    x_est(1:nx,:)'*1e+3, ...
    error(1:nx,:)'*1e+3, ...
    std_state(1:nx,:)'*1e+3, ...
    z(1:nz,:)', ...
    meas_pred(1:nz,:)', ...
    std_meas(1:nz,:)'*1e+3, ...
    nis', nees', cond_P', det_P', cond_S', det_S', runtime', ...
    repelem(filterVariant, N)', repelem(nx, N)', repelem(useSRformulation, N)', ...
    repelem(RK4Iterations, N)', repelem(updateJacobians, N)', repelem(updateQ, N)', repelem(cubature, N)'];

% Combine header and data
outputData = [header; num2cell(dataMatrix)];

% Write to CSV file
writecell(outputData, 'simulation_results.csv');

%% functions
function Qd = van_loan(A, Q, dt)
    n = size(A, 1);

    M = [-A         Q;
         zeros(n,n) A'];

    phi = expm(M*dt);

    phi12 = phi(1:n,     n+1:end);
    phi22 = phi(n+1:end, n+1:end);

    Qd = phi22' * phi12;
end

function noise = generate_measurement_noise(mu, R, N)
    rng("default");
    noise = mvnrnd(mu, R, N)';
    save('measurement_noise.mat', 'noise');
end

function noise = generate_process_noise(dt, N)

    fs = 1/dt; % sampling frequency [Hz]

    % noise magnitudes
    sigmaAcc = 0.01;   % translational acceleration disturbance [m/s^2]
    sigmaAng = 0.0001; % angular acceleration disturbance [rad/s^2]
    
    rng("default");
    
    % random noise
    rawAcc = randn(3,N);
    rawRot   = randn(2,N);
    
    % band limiting to prevent arbitrarily fast changes in noise
    fc = 10; % higher cutoff frequency [Hz]

    accNoise = zeros(3,N);
    for i = 1:3
        accNoise(i,:) = lowpass(rawAcc(i,:),fc,fs);
    end

    rotNoise = zeros(2,N);
    for i = 1:2
        rotNoise(i,:) = lowpass(rawRot(i,:),fc,fs);
    end
    
    % normalize RMS
    accNoise = accNoise ./ std(accNoise,0,2);
    rotNoise = rotNoise ./ std(rotNoise,0,2);
    
    % scale
    noise.acc = sigmaAcc*accNoise;
    noise.rot = sigmaAng*rotNoise;
    
    save('process_noise.mat', 'noise');
end