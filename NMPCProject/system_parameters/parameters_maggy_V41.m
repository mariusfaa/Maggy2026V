%{
### Parameters for the maglev system ####
The geometrical parameters are based on the actual maglev system. The
physical parameters are taken from sources online, and could be incorrect
w.r.t., the real system.
%}

%% Parameters
% Solenoids (Tuned to real solenoids and data from gikfun)
params.solenoids.x  = 0.02*[1,0,-1,0];
params.solenoids.y  = 0.02*[0,1,0,-1];
params.solenoids.r  = 0.0185/2*ones(1,4);
params.solenoids.l  = 0.012*ones(1,4);
params.solenoids.z  = params.solenoids.l/2;
params.solenoids.nw = 480;

% Permanent magnets (Tuned to the real magnets)
params.permanent.x  = sqrt(0.5*0.035^2)*[1,-1,1,-1];
params.permanent.y  = sqrt(0.5*0.035^2)*[1,1,-1,-1];
params.permanent.r  = 0.5*0.020*ones(1,4);                                     
params.permanent.l  = 2*0.0040*ones(1,4);
params.permanent.z  = params.permanent.l/2+0.2e-3; % (where they are centered)
params.permanent.J  = 1.15;

% Levitating magnet (Tuned using the "equivalent magnet" principle)
params.magnet.r     = 0.025;
params.magnet.l     = 0.0040;
params.magnet.J     = -1.1;
params.magnet.m     = 0.060; % (weight on kitchen scale, golden magnet)
params.magnet.I     = [6.1686e-06, 6.1686e-06, 1.1274e-05];
params.magnet.n     = 8; % Radial discretization points on levitating disk (higher is more accurate, but also more computationally expensive)
params.magnet.n_axial = 1; % keep odd for best accuracy

% Sensors - only one sensor 
params.sensors.x  = [-0.0003];
params.sensors.y  = [0];
params.sensors.z  = [0];

% Physical constants
params.physical.g   = 9.81;                                                % Gravitational acceleration [m/s^2]
params.physical.mu0 = 4*pi*1e-7;                                           % Permeability of free space (air) [N/A^2]

% LUT options for CasADi model
params.lut_opts.enabled = true;
params.lut_opts.method       = 'linear';        % 'bspline' or 'linear'

% Solenoid 2D LUT bounds (derived from geometry + margin)
% rho_max must cover the max distance from ANY solenoid to ANY point in the
% workspace. Worst case: solenoid at one edge, eval point at opposite corner.
perm3d_xy_val = max(abs([params.permanent.x, params.permanent.y])) + max(params.permanent.r) + params.magnet.r + 0.005;
sol_xy_max = max(abs([params.solenoids.x, params.solenoids.y]));
params.lut_opts.sol2d_max_rho = sqrt(2) * perm3d_xy_val + sol_xy_max;  % diagonal + solenoid offset
params.lut_opts.sol2d_max_z   = 0.06;                           % solenoid LUT z range [m]
params.lut_opts.sol2d_N_rho   = 100;                            % solenoid LUT rho grid points
params.lut_opts.sol2d_N_z     = 100;                            % solenoid LUT z grid points

% Permanent magnet 3D LUT bounds (derived from geometry + margin)
perm_xy_extent = max(abs([params.permanent.x, params.permanent.y])) + max(params.permanent.r);
params.lut_opts.perm3d_max_xy     = perm_xy_extent + params.magnet.r + 0.005;  % xy bound with margin for magnet surface
perm_z_lo = max(params.solenoids.z + params.solenoids.l/2) + 1e-3;        % just above solenoid tops
params.lut_opts.perm3d_max_z      = [perm_z_lo, 0.06];              % perm LUT z range [m]
params.lut_opts.perm3d_N_xy   = 31;                             % perm LUT xy grid points
params.lut_opts.perm3d_N_z    = 100;                            % perm LUT z grid points