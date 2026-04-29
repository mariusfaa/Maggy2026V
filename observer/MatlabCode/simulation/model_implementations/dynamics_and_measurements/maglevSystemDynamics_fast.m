function dx = maglevSystemDynamics_fast(x,u) %#codegen
modelName = 'fast';
persistent params;
if isempty(params)
    %% Parameters
% Solenoids (Tuned to real solenoids and data from gikfun)
params.solenoids.x  = 0.02*[1,0,-1,0];
params.solenoids.y  = 0.02*[0,1,0,-1];
params.solenoids.r  = 0.0185/2*ones(1,4)*0.564394804131228; % mod; 'fast' parameter correction
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
params.sensors.x  = [-0.0003]%, -0.0326856, 0.0130152];
params.sensors.y  = [0]%, 0.0137257, 0.0324254];
params.sensors.z  = [0];%, 0, 0];%-0.2e-3;

% Physical constants
params.physical.g   = 9.81;                                                % Gravitational acceleration [m/s^2]
params.physical.mu0 = 4*pi*1e-7;   
end

% MAGLEVSYSTEMDYNAMICS implements the function f in the ODE dxdt = f(x,u)
% defining the dynamics of a magnetic levitation system. The system is
% fully defined by the params struct, and the magnetic model to be used is
% defined by modelName, which can be either 'fast', 'accurate' or 
% 'filament'.
%
% Example:
%   params; (from parameter file)
%   modelName = 'fast';
%   u = @(t) ...; (user defined)
%   f = @(t,x) maglevSystemDynamics(x,u(t),params,modelName);
%
%   t = [0, 10]; x0 = [0,0,0.1,0,0,0,0,0,0,0,0,0]';
%   [t,x] = ode15s(f,t,x0);
%
% See also MAGLEVSYSTEMMEASUREMENTS.

% MAGLEVSYSTEMDYNAMICS uses the implementation described in [1].

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% References:
% [1] Engmark, Hans Alvar, and Kiet Tuan Hoang. 
%     "Modeling and Control of a Magnetic Levitation Platform." 
%     IFAC-PapersOnLine 56.2 (2023): 7276-7281.

% Computing force and torque on levitating magnet
[fx,fy,fz,tx,ty,tz] = computeForceAndTorque(x,u,params,modelName);

% Setting up system matrices
A = [
    zeros(6), eye(6);
    zeros(6), zeros(6)
    ];

B = [
    zeros(6);
    eye(6)
    ];

% Mass and inertia properties of the magnet
M = [
    params.magnet.m*eye(3), zeros(3);
    zeros(3), diag(params.magnet.I)
    ];

% Computing the nonlinear function f(x,u)
f = M\([fx;fy;fz;tx;ty;tz]-[zeros(3,1);cross(x(10:12),diag(params.magnet.I)*x(10:12))])...
    -[zeros(2,1);params.physical.g;zeros(3,1)];

dx = A*x+B*f;
end
