addpath(genpath("."));
load_params(MaglevModel.Accurate);

nx = 12;
nu = length(params.solenoids.r);
ny = 3;

% find eq
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(params,modelId);

xEq = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uEq = zeros(nu,1);

dt = 0.0001;
t = 0:dt:0.1;

% set initial conditions for sim
x0 = xEq + [0 0.002 0.015 0 0 0 zeros(1, 6)].';
%x0 = xEq + [0 0.001 0.001 0 0 0 zeros(1, 6)].';
% x0 = xEq;

%u0 = ones(nu,1) * 1;
u0 = [-0.25;0.5;-0.5;0.75];