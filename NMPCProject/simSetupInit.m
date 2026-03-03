addpath(genpath("."));
load_params(MaglevModel.Accurate);

nx = 12;
nu = length(params.solenoids.r);
ny = 3;

% Setting up system equations
f = @(x,u) maglevSystemDynamics(x,u,params,modelId);
h = @(x,u) maglevSystemMeasurements(x,u,params,modelId);

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(params,modelId);

xEq = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uEq = zeros(nu,1);

dt = 0.001;
t = 0:dt:0.1;

x0 = xEq + [0 0.002 0.015 0 0 0 zeros(1, 6)].';
u0 = ones(nu,1);