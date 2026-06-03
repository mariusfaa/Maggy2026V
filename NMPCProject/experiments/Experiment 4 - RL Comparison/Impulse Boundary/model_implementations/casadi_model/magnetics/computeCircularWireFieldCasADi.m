function [bx, by, bz] = computeCircularWireFieldCasADi(x, y, z, r, I, mu0)
% COMPUTECIRCULARWIREFIELD  Magnetic field of a circular current loop
% (Cartesian interface).
%
%   [bx, by, bz] = computeCircularWireField(x, y, z, r, I, mu0)
%
%   Converts Cartesian coordinates to cylindrical, delegates to
%   computeCircularWireFieldPolar, and converts the result back.
%
% Inputs:
%   x, y, z  - Cartesian offsets from the loop centre (1xN, CasADi SX/MX)
%   r        - Loop radius  (scalar, numeric)
%   I        - Loop current (scalar, CasADi SX/MX or numeric)
%   mu0      - Permeability of free space (scalar, numeric)
%
% Outputs:
%   bx, by, bz - Cartesian field components (1xN, CasADi SX/MX)
%
% See also: computeCircularWireFieldPolar

    import casadi.*

    rho = sqrt(x.^2 + y.^2);
    phi = atan2(y, x);

    [bPhi, bRho, bz] = computeCircularWireFieldPolarCasADi(rho, z, r, I, mu0);

    bx = bRho .* cos(phi) - bPhi .* sin(phi);
    by = bRho .* sin(phi) + bPhi .* cos(phi);
end
