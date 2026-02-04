%% --- PROJECT SETUP ---
clear all; clc;

% 1. DEFINE PATHS (Double check these once more)
acados_root = 'C:\Users\mariujf\acados'; 
project_root = 'C:\Users\mariujf\NMPCProject'; 

% 2. THE NUCLEAR FIX FOR PATHS
% We must set these BEFORE importing casadi or creating the solver.
setenv('ACADOS_SOURCE_DIR', acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
% This forces the MEX compiler to look in the acados folder for link_libs.json
setenv('ACADOS_INSTALL_DIR', acados_root); 

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external', 'jsonlab'));
addpath(fullfile(acados_root, 'external', 'casadi-matlab'));

% Project folders
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

parameters_maggy_V4;

correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');

fprintf('Fast correction factor %.2f\n', correctionFactorFast)

paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');

f = @(x,u) maglevSystemDynamicsCasADi(x,u,paramsFast);

xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(length(params.solenoids.r),1);

delta = 1e-6;
[A,B] = casadiFiniteDifferenceLinearization(f,xLp,uLp,delta);

fprintf('Size of A: %.f, %.f\n', size(A))
fprintf('Size of B: %.f, %.f\n', size(B))

I = [1:5,7:11];
Ared = A(I,I);
Bred = B(I,:);

Q = diag([1e6,1e6,1e2, 1e1,1e1, 1e2,1e2,1e2, 1e2,1e2]);
R = 1e-0*eye(length(params.solenoids.r));

Kred = round(lqr(Ared,Bred,Q,R),3); % Rounding can sometimes be dangerous!

K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];

x0 = xLp + [0.001 -0.003 0.04 pi/5 0 0 zeros(1, 6)].';
tSpan = linspace(0,1,100);

