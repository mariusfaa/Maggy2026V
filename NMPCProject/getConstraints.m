function constraints = getConstraints(x0,umax)
    arguments
        x0 (1,:) double
        umax double = 1.0
    end

% Return ocp constraint object that is persistant for all models

import casadi.*

nx = 10;
nu = 4;


max_xy_m = 0.01;
min_z_m = 0.01;
max_z_m = 0.05;
max_ang_rad = pi;


constraints = AcadosOcpConstraints();
% Bounds on inputs
constraints.idxbu = 0:nu-1;
constraints.lbu   = -umax * ones(nu, 1);
constraints.ubu   =  umax * ones(nu, 1);

% Bounds on states
constraints.x0    = x0;
constraints.idxbx = 0:4; % only bounds on positional states
constraints.lbx   = [-max_xy_m, -max_xy_m, min_z_m, -max_ang_rad, -max_ang_rad];
constraints.ubx   = [max_xy_m,   max_xy_m, max_z_m,  max_ang_rad,  max_ang_rad];

% Do we need to set these? do they affect anything
% think they might be set implicit when x0 for the ocp is set.
% constraints.idxbx_0 = constraints.idxbx;
% constraints.lbu_0 = constraints.lbu;
% constraints.lbx_0 = constraints.lbx;
% 
% constraints.idxbx_e = constraints.idxbx;
% constraints.lbu_e = constraints.lbu;
% constraints.lbx_e = constraints.lbx;

end