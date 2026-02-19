function [fz,fzInv] = computeVerticalForce(z,params,modelId)

% Define no input
u = 0*ones(length(params.solenoids.r),1);

% Compute normal graf
fz = zeros(size(z));
for i = 1:length(z)
    x = [0;0;z(i);zeros(9,1)];
    [~,~,fz(i)] = computeForceAndTorque(x,u,params,modelId);
end
fz = fz - 9.81*params.magnet.m;

% Compute inverted graph
paramsInv = params;
paramsInv.magnet.J = -paramsInv.magnet.J;

fzInv = zeros(size(z));
for i = 1:length(z)
    x = [0;0;z(i);zeros(9,1)];
    [~,~,fzInv(i)] = computeForceAndTorque(x,u,paramsInv,modelId);
end
fzInv = fzInv - 9.81*params.magnet.m;