function [A,B,C,D] = maglevSystemLinearized(xLp,uLp,params,modelId)

% Setting up system equations with speficied model and parameters for
% simpler representation
f = @(x,u) maglevSystemDynamics(x,u,params,modelId);
h = @(x,u) maglevSystemMeasurements(x,u,params,modelId);

% Linearization
delta = 1e-6; % Step-size used in numerical linearization
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);