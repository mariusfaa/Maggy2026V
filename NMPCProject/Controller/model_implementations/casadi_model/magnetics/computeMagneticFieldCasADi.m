function [bx, by, bz] = computeMagneticFieldCasADi(x, y, z, u, params)
% COMPUTEMAGNETICFIELD  Total magnetic field at query points.
%
%   [bx, by, bz] = computeMagneticField(x, y, z, u, params)
%
%   Superposes contributions from all permanent magnets and all
%   current-controlled solenoids defined in params.
%
% Inputs:
%   x, y, z  - Query point coordinates (1xN CasADi SX/MX)
%   u        - Solenoid currents (CasADi SX/MX vector)
%   params   - Parameter struct with fields:
%                params.physical.mu0
%                params.permanent.{x,y,z,r,l,J}
%                params.solenoids.{x,y,z,r,nw}
%
% Outputs:
%   bx, by, bz - Cartesian field components (1xN CasADi SX/MX)
%
% See also: computeCircularWireField

    import casadi.*

    n_points = length(x);
    bx = SX.zeros(1, n_points);
    by = SX.zeros(1, n_points);
    bz = SX.zeros(1, n_points);

    mu0 = params.physical.mu0;

    %% Permanent magnets
    n_perm = length(params.permanent.r);
    for i = 1:n_perm
        I_perm = params.permanent.J / mu0 * params.permanent.l(i);

        [dbx, dby, dbz] = computeCircularWireFieldCasADi( ...
            x - params.permanent.x(i), ...
            y - params.permanent.y(i), ...
            z - params.permanent.z(i), ...
            params.permanent.r(i), I_perm, mu0);

        bx = bx + dbx;
        by = by + dby;
        bz = bz + dbz;
    end

    %% Solenoids (current-controlled)
    n_sol = length(params.solenoids.r);
    for i = 1:n_sol
        [dbx, dby, dbz] = computeCircularWireFieldCasADi( ...
            x - params.solenoids.x(i), ...
            y - params.solenoids.y(i), ...
            z - params.solenoids.z(i), ...
            params.solenoids.r(i), u(i), mu0);

        bx = bx + dbx * params.solenoids.nw;
        by = by + dby * params.solenoids.nw;
        bz = bz + dbz * params.solenoids.nw;
    end
end
