function [fx,fy,fz] = computeForceAndTorque_xred(x,u,params,modelName)
% COMPUTEFORCEANDTORQUE computes the magnetic force and torque produced by 
% a base of permanent magnets and solenoids, defined by params,
% on a levitating magnet defined by params and x. The force is computed
% using the magnet/solenoid model defined by modelName.
%
% u is the current in running through the solenoids (its size defined by 
% the number of solenoids in params). modelName is either 'fast', 
% 'accurate' or 'fillament'.
%
% Example:
%   x = [0,0,0.05,0,0,0,0,0,0,0,0,0]'; u = [1,0,-1,0]'; 
%   params; (from parameter file)
%   modelName = 'fast';
%   [fx,fy,fz,tx,ty,tz] = computeForceAndTorque(x,u,params,modelName);
%
% See also COMPUTEFIELDBASE.

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% Additional parameters
K  = -params.magnet.J/params.physical.mu0;

% Points along circumfrence
theta = linspace(0,2*pi-2*pi/params.magnet.n,params.magnet.n);
px = params.magnet.r*cos(theta);
py = params.magnet.r*sin(theta);
pz = zeros(size(px));

p = [px;py;pz] + x(1:3);

% Compute magnetic field
[bx,by,bz] = computeFieldBase(p(1,:),p(2,:),p(3,:),u,params,modelName);

% Compute force
tangent = [cos(theta+pi/2); sin(theta+pi/2); zeros(size(theta))];
F = cross(K*params.magnet.l*tangent,[bx;by;bz]);

fx = params.magnet.r*trapz([theta,2*pi],[F(1,:),F(1,1)]);
fy = params.magnet.r*trapz([theta,2*pi],[F(2,:),F(2,1)]);
fz = params.magnet.r*trapz([theta,2*pi],[F(3,:),F(3,1)]);