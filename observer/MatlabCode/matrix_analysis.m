
clear;
addpath(genpath('~/MSc/MatlabCode'));
parameters_maggy_V4;

%% correction factors
%correctionFactorFast = 0.558361864615352; %v2
%correctionFactorAccurate = 0.553376368593826; %v2
correctionFactorFast = 0.564394804131228; %computeSolenoidRadiusCorrectionFactor(params,'fast');
correctionFactorAccurate = 0.548863066449565; %computeSolenoidRadiusCorrectionFactor(params,'accurate');

fprintf('Fast correction factor %.2f\n', correctionFactorFast)
fprintf('Accurate correction factor %.2f\n', correctionFactorAccurate)

paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

paramsAccurate = params;
paramsAccurate.solenoids.r = correctionFactorAccurate*paramsAccurate.solenoids.r;

%% equilibria
%zEq = 0.036501894370622; %v2
%zEqAq = 0.036960265688713; %v2
%zEqFil = 0.036960265688713; %v2
zEq = 0.030119178665731; %[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');
zEqAq = 0.030627801083773; %[zEqAq, ~, ~, ~] = computeSystemEquilibria(paramsAccurate,'accurate');
zEqFil = 0.030627801083773; %[zEqFil, ~, ~, ~] = computeSystemEquilibria(params,'filament');

%% linearization
%f = @(x,u) maglevSystemDynamics(x,u,paramsFast,'fast');
%h = @(x,u) maglevSystemMeasurements(x,u,params,'fast');
f = @(x,u) maglevSystemDynamics_fast(x,u);
h = @(x,u) maglevSystemMeasurements_fast(x,u);
fAq = @(x,u) maglevSystemDynamics_accurate(x,u);
hAq = @(x,u) maglevSystemMeasurements_accurate(x,u);
fFil = @(x,u) maglevSystemDynamics_filament(x,u);
hFil = @(x,u) maglevSystemMeasurements_filament(x,u);

% Define the point to linearize around
xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
xLpAq = [0,0,zEqAq(1),zeros(1,9)]'; % Linearizing around the equilibria
xLpFil = [0,0,zEqFil(1),zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(length(params.solenoids.r),1);

% Linearization
delta = 1e-6; % Step-size used in numerical linearization
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);
[AAq,BAq,CAq,DAq] = finiteDifferenceLinearization(fAq,hAq,xLpAq,uLp,delta);
[AFil,BFil,CFil,DFil] = finiteDifferenceLinearization(fFil,hFil,xLpFil,uLp,delta);

I = [1:5,7:11];
Ared = A(I,I); AAqred = AAq(I,I); AFilred = AFil(I,I); Axred = Ared(1:end-2,1:end-2);
Bred = B(I,:); BAqred = BAq(I,:); BFilred = BFil(I,:); Bxred = Bred(1:end-2,:);
Cred = C(:,I); CAqred = CAq(:,I); CFilred = CFil(:,I); Cxred = Cred(:,1:end-2);
Dred = D(:,:); DAqred = DAq(:,:); DFilred = DFil(:,:);

%% discretizing
dt = 0.01; % 100 Hz
sysd = c2d(ss(Ared,Bred,Cred,Dred), dt, 'zoh');
Ad = sysd.A;
Bd = sysd.B;

%threshold = 1e-9;
%Ad(abs(Ad) < threshold) = 0;
%Bd(abs(Bd) < threshold) = 0;
%Ad = round(Ad, 9);
%Bd = round(Bd, 9);

%% van loan

nred = 10;
nxred = 8;

% Noise spectral density
NSDred = [1 0 0 0 0;
          0 1 0 0 0;
          0 0 1 0 0;
          0 0 0 1 0;
          0 0 0 0 1];

NSDxred = [1 0 0;
           0 1 0;
           0 0 1];

% affected states
Gred = [0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 0 0;
        1 0 0 0 0;
        0 1 0 0 0;
        0 0 1 0 0;
        0 0 0 1 0;
        0 0 0 0 1];

Gxred = Gred(1:8, 1:3);

Qred = Gred*NSDred*Gred';

Qxred = Gxred*NSDxred*Gxred';

Mred = [-Ared          Qred;
        zeros(nred,nred)  Ared'];

Mxred = [-Axred          Qxred;
         zeros(nxred,nxred)  Axred'];

Phired = expm(Mred * dt);

Phixred = expm(Mxred * dt);

Phi12red = Phired(1:nred, nred+1:end);
Phi22red = Phired(nred+1:end, nred+1:end);

Phi12xred = Phixred(1:nxred, nxred+1:end);
Phi22xred = Phixred(nxred+1:end, nxred+1:end);

Qd = Phi22red' * Phi12red;
Qd_xred = Phi22xred' * Phi12xred;

%threshold = 1e-9;
%Qd(abs(Qd) < threshold) = 0;
Qd = round(Qd, 9);
Qd_xred = round(Qd_xred, 9);
%% matrix analysis
ObsvMat = obsv(Ared, Cred);
fprintf('Rank of observability matrix %.2f\n', rank(ObsvMat));

ObsvMatd = obsv(Ad, Cred);
fprintf('Rank of discrete observability matrix %.2f\n', rank(ObsvMatd));

%% save data

%writematrix(Ared, 'A_fast.csv');
%writematrix(Ad, 'Ad_fast.csv');
%writematrix(Bred, 'B_fast.csv');
%writematrix(Bd, 'Bd_fast.csv');
%writematrix(Cred, 'C_fast.csv');
%writematrix(Dred, 'D_fast.csv');
%writematrix(ObsvMat, 'ObsvMat.csv');
%writematrix(Qd_xred, 'Qd_fast.csv');
%writematrix(Qd, 'Qd_xred.csv');

%writematrix(AAqred, 'A_accurate.csv');
%writematrix(BAqred, 'B_accurate.csv');
%writematrix(CAqred, 'C_accurate.csv');
%writematrix(DAqred, 'D_accurate.csv');
%writematrix(AFilred, 'A_filament.csv');
%writematrix(BFilred, 'B_filament.csv');
%writematrix(CFilred, 'C_filament.csv');
%writematrix(DFilred, 'D_filament.csv');

