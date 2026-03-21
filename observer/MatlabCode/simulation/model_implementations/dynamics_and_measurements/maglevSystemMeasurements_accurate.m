function y = maglevSystemMeasurements_accurate(x,u) %#codegen
modelName = 'accurate';
persistent params;
if isempty(params)
    %% Parameters
% Solenoids (Tuned to real solenoids and data from gikfun)
params.solenoids.x  = 0.02*[1,0,-1,0];
params.solenoids.y  = 0.02*[0,1,0,-1];
params.solenoids.r  = 0.0185/2*ones(1,4)*0.548863066449565; % mod; 'accurate' parameter correction
params.solenoids.l  = 0.012*ones(1,4);
params.solenoids.z  = 0.012*ones(1,4)/2;
params.solenoids.nw = 480;

% % Permanent magnets (Tuned to the real magnets)
params.permanent.x  = sqrt(0.5*0.035^2)*[1,-1,1,-1];
params.permanent.y  = sqrt(0.5*0.035^2)*[1,1,-1,-1];
params.permanent.r  = 0.5*0.020*ones(1,4);                                     
params.permanent.l  = 2*0.0040*ones(1,4);
params.permanent.z  = 2*0.0040*ones(1,4)/2+0.2e-3; % (where they are centered)
params.permanent.J  = 1.15;

% Levitating magnet (Tuned using the "equivalent magnet" principle)
params.magnet.r     = 0.025;
params.magnet.l     = 0.0040;
params.magnet.J     = -1.1;
params.magnet.m     = 0.060; % (weight on kitchen scale, golden magnet)
params.magnet.I     = [6.1686e-06, 6.1686e-06, 1.1274e-05];
params.magnet.n     = 100;

% Sensors (7, 2, 3)
params.sensors.x  = -0.0003;%, -0.0326856, 0.0130152];
params.sensors.y  = 0;%, 0.0137257, 0.0324254];
params.sensors.z  = 0;%, 0, 0];%-0.2e-3;

% Physical constants
params.physical.g   = 9.81;                                                % Gravitational acceleration [m/s^2]
params.physical.mu0 = 4*pi*1e-7;   
end
% MAGLEVSYSTEMMEASUREMENTS implements the function h in the ODE 
%   dxdt = f(x,u); 
%      y = h(x,u);
% defining the sensor measurements of a magnetic levitation system. The 
% system is fully defined by the params struct, and the magnetic model to 
% be used for the measurements is defined by modelName, which can be either
% 'fast', 'accurate' or 'filament'. The measurements computes the effect
% from all magnetic components of the system.
%
% Example:
%   params; (from parameter file)
%   modelName = 'fast';
%   x = [0,0,0.1,0,0,0,0,0,0,0,0,0]';
%   u = [1,0,-1,0]';
%
%   h = @(x,u) maglevSystemMeasuremets(x,u,params,modelName);
%   y = h(x,u);
%
% See also MAGLEVSYSTEMDYNAMICS,
%          COMPUTEFIELDTOTAL.

% MAGLEVSYSTEMMEASUREMENTS uses the implementation described in [1].

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% References:
% [1] Engmark, Hans Alvar, and Kiet Tuan Hoang. 
%     "Modeling and Control of a Magnetic Levitation Platform." 
%     IFAC-PapersOnLine 56.2 (2023): 7276-7281.

[bx,by,bz] = computeFieldTotal(params.sensors.x,params.sensors.y,params.sensors.z,x,u,params,modelName);
y = reshape([bx(:)'; by(:)'; bz(:)'],3*numel(bx),1);
